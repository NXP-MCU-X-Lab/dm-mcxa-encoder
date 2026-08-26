#!/usr/bin/env python3

import argparse
import collections
import statistics
import struct
import sys
import time


CF_BY_DATA_ID = {0: 0x02, 3: 0x1A}
FRAME_SIZE_BY_DATA_ID = {0: 6, 3: 11}
SF_VALID = 0x00
SF_COUNTING_ERROR = 0x10
ALMC_COUNTING_ERROR = 0x04
ENID_16BIT = 0x10
DIAG_CF = 0xF1
DIAG_VERSION = 1
DIAG_FRAME_SIZE = 25


def crc8_tformat(data):
    remainder = 0
    for byte in data:
        remainder ^= byte
        for _ in range(8):
            remainder = ((remainder << 1) ^ 0x01) & 0xFF if remainder & 0x80 else (remainder << 1) & 0xFF
    return remainder


def build_id0_frame(counts, status):
    frame = bytearray((CF_BY_DATA_ID[0], status, counts & 0xFF, (counts >> 8) & 0xFF, 0))
    frame.append(crc8_tformat(frame))
    return bytes(frame)


def build_id3_frame(counts, status):
    almc = 0 if status == SF_VALID else ALMC_COUNTING_ERROR
    frame = bytearray(
        (
            CF_BY_DATA_ID[3],
            status,
            counts & 0xFF,
            (counts >> 8) & 0xFF,
            0,
            ENID_16BIT,
            0,
            0,
            0,
            almc,
        )
    )
    frame.append(crc8_tformat(frame))
    return bytes(frame)


def build_diag_frame(status, valid, calibration_source, mag16_raw, mag15_raw, sample_count, overrun_count):
    frame = bytearray(
        struct.pack(
            "<BBBBIffII",
            DIAG_CF,
            DIAG_VERSION,
            1 if valid else 0,
            calibration_source,
            status,
            mag16_raw,
            mag15_raw,
            sample_count,
            overrun_count,
        )
    )
    frame.append(crc8_tformat(frame))
    return bytes(frame)


def decode_frame(frame, data_id):
    expected_size = FRAME_SIZE_BY_DATA_ID[data_id]
    expected_cf = CF_BY_DATA_ID[data_id]

    if len(frame) != expected_size:
        raise ValueError(f"expected {expected_size} bytes, got {len(frame)}")
    if frame[0] != expected_cf:
        raise ValueError(f"unexpected CF 0x{frame[0]:02X}")
    if frame[4] != 0:
        raise ValueError(f"ABS2 is not zero: 0x{frame[4]:02X}")

    expected_crc = crc8_tformat(frame[:-1])
    if frame[-1] != expected_crc:
        raise ValueError(f"CRC 0x{frame[-1]:02X}, expected 0x{expected_crc:02X}")

    counts = frame[2] | (frame[3] << 8)
    result = {
        "status": frame[1],
        "counts": counts,
        "angle_deg": counts * 360.0 / 65536.0,
        "enid": None,
        "multi_turn": None,
        "almc": None,
    }

    if data_id == 3:
        multi_turn = frame[6] | (frame[7] << 8) | (frame[8] << 16)
        if multi_turn & 0x800000:
            multi_turn -= 1 << 24
        result["enid"] = frame[5]
        result["multi_turn"] = multi_turn
        result["almc"] = frame[9]

    return result


def decode_diag_frame(frame):
    if len(frame) != DIAG_FRAME_SIZE:
        raise ValueError(f"expected {DIAG_FRAME_SIZE} bytes, got {len(frame)}")
    if frame[0] != DIAG_CF:
        raise ValueError(f"unexpected diagnostic CF 0x{frame[0]:02X}")
    if frame[1] != DIAG_VERSION:
        raise ValueError(f"diagnostic version {frame[1]}, expected {DIAG_VERSION}")

    expected_crc = crc8_tformat(frame[:-1])
    if frame[-1] != expected_crc:
        raise ValueError(f"CRC 0x{frame[-1]:02X}, expected 0x{expected_crc:02X}")

    fields = struct.unpack("<BBBBIffII", frame[:-1])
    return {
        "valid": fields[2],
        "calibration_source": fields[3],
        "status": fields[4],
        "mag16_raw": fields[5],
        "mag15_raw": fields[6],
        "sample_count": fields[7],
        "overrun_count": fields[8],
    }


def run_self_test():
    vectors = (
        (0, 0x0000, SF_VALID, "020000000002"),
        (0, 0x1234, SF_VALID, "020034120024"),
        (0, 0xFFFF, SF_VALID, "0200ffff0002"),
        (0, 0x0000, SF_COUNTING_ERROR, "021000000012"),
        (0, 0x1234, SF_COUNTING_ERROR, "021034120034"),
        (3, 0x0000, SF_VALID, "1a0000000010000000000a"),
        (3, 0x1234, SF_VALID, "1a0034120010000000002c"),
        (3, 0xFFFF, SF_VALID, "1a00ffff0010000000000a"),
        (3, 0x0000, SF_COUNTING_ERROR, "1a1000000010000000041e"),
        (3, 0x1234, SF_COUNTING_ERROR, "1a10341200100000000438"),
    )

    for data_id, counts, status, expected_hex in vectors:
        build_frame = build_id0_frame if data_id == 0 else build_id3_frame
        frame = build_frame(counts, status)
        if frame.hex() != expected_hex:
            raise AssertionError(f"ID{data_id} {counts:04X}: {frame.hex()} != {expected_hex}")
        decoded = decode_frame(frame, data_id)
        if (decoded["status"], decoded["counts"]) != (status, counts):
            raise AssertionError(f"decode failed for {expected_hex}")

    diag_frame = build_diag_frame(0x12, True, 1, 0.5, 0.25, 123, 4)
    diag = decode_diag_frame(diag_frame)
    if diag != {
        "valid": 1,
        "calibration_source": 1,
        "status": 0x12,
        "mag16_raw": 0.5,
        "mag15_raw": 0.25,
        "sample_count": 123,
        "overrun_count": 4,
    }:
        raise AssertionError("diagnostic frame decode failed")

    print(f"self-test passed: {len(vectors) + 1} vectors")


def read_exact(port, size):
    data = bytearray()
    while len(data) < size:
        chunk = port.read(size - len(data))
        if not chunk:
            break
        data.extend(chunk)
    return bytes(data)


def signed_delta(current, previous):
    return ((current - previous + 0x8000) & 0xFFFF) - 0x8000


def run_serial_test(args):
    try:
        import serial
    except ImportError as error:
        raise RuntimeError("pyserial is required: python -m pip install pyserial") from error

    cf = CF_BY_DATA_ID[args.data_id]
    frame_size = FRAME_SIZE_BY_DATA_ID[args.data_id]
    latencies_us = []
    valid_count = 0
    error_count = 0
    counting_error_count = 0
    almc_count = 0
    wraps = 0
    max_step = 0
    previous_counts = None

    with serial.Serial(args.port, args.baud, timeout=args.timeout) as port:
        port.reset_input_buffer()
        for index in range(args.count):
            start_ns = time.perf_counter_ns()
            port.write(bytes((cf,)))
            frame = read_exact(port, frame_size)
            latencies_us.append((time.perf_counter_ns() - start_ns) / 1000.0)
            decoded = decode_frame(frame, args.data_id)

            if decoded["status"] == SF_VALID:
                valid_count += 1
            else:
                error_count += 1
            if decoded["status"] & SF_COUNTING_ERROR:
                counting_error_count += 1

            if args.data_id == 3:
                if decoded["enid"] != args.expect_enid:
                    raise ValueError(f"ENID 0x{decoded['enid']:02X}, expected 0x{args.expect_enid:02X}")
                if decoded["almc"] != 0:
                    almc_count += 1

            counts = decoded["counts"]
            if previous_counts is not None:
                delta = signed_delta(counts, previous_counts)
                max_step = max(max_step, abs(delta))
                if abs(counts - previous_counts) > 0x8000:
                    wraps += 1
            previous_counts = counts

            if args.verbose:
                details = ""
                if args.data_id == 3:
                    details = (
                        f" ENID=0x{decoded['enid']:02X}"
                        f" ABM={decoded['multi_turn']} ALMC=0x{decoded['almc']:02X}"
                    )
                print(
                    f"{index:6d}  {frame.hex(' ')}  {counts:5d}  "
                    f"{decoded['angle_deg']:9.4f} deg  SF=0x{decoded['status']:02X}{details}"
                )

            if args.interval_ms > 0:
                time.sleep(args.interval_ms / 1000.0)

    print(
        f"data_id={args.data_id} frames={args.count} valid={valid_count} errors={error_count} "
        f"counting_error={counting_error_count} almc={almc_count} "
        f"wraps={wraps} max_step={max_step} counts"
    )
    print(
        f"host_round_trip_us min={min(latencies_us):.1f} "
        f"mean={statistics.fmean(latencies_us):.1f} max={max(latencies_us):.1f}"
    )


def run_diag_test(args):
    try:
        import serial
    except ImportError as error:
        raise RuntimeError("pyserial is required: python -m pip install pyserial") from error

    latencies_us = []
    status_counts = collections.Counter()
    last = None

    with serial.Serial(args.port, args.baud, timeout=args.timeout) as port:
        port.reset_input_buffer()
        for index in range(args.count):
            start_ns = time.perf_counter_ns()
            port.write(bytes((DIAG_CF,)))
            frame = read_exact(port, DIAG_FRAME_SIZE)
            latencies_us.append((time.perf_counter_ns() - start_ns) / 1000.0)
            last = decode_diag_frame(frame)
            status_counts[last["status"]] += 1

            if args.verbose:
                print(
                    f"{index:6d}  {frame.hex(' ')}  status=0x{last['status']:08X} "
                    f"valid={last['valid']} source={last['calibration_source']} "
                    f"mag16={last['mag16_raw']:.6f} mag15={last['mag15_raw']:.6f} "
                    f"samples={last['sample_count']} overruns={last['overrun_count']}"
                )

            if args.interval_ms > 0:
                time.sleep(args.interval_ms / 1000.0)

    statuses = ",".join(f"0x{status:08X}:{count}" for status, count in sorted(status_counts.items()))
    print(f"diag frames={args.count} statuses={statuses}")
    print(
        f"last valid={last['valid']} source={last['calibration_source']} "
        f"status=0x{last['status']:08X} mag16={last['mag16_raw']:.6f} "
        f"mag15={last['mag15_raw']:.6f} samples={last['sample_count']} "
        f"overruns={last['overrun_count']}"
    )
    print(
        f"host_round_trip_us min={min(latencies_us):.1f} "
        f"mean={statistics.fmean(latencies_us):.1f} max={max(latencies_us):.1f}"
    )


def run_hold_test(args):
    """Poll hard while the shaft is stationary.

    The firmware applies a 0.015 deg dead-band (~2.7 counts) and a 1-count
    hysteresis to angle_counts, and angle_counts is exactly what T-Format
    reports. If those really do ride on the control path, a stationary shaft
    yields a delta of identically zero and only a handful of distinct values --
    which is a servo feedback stiction band, not a display nicety. This measures
    it rather than inferring it from the source.
    """
    import serial

    cf = CF_BY_DATA_ID[args.data_id]
    frame_size = FRAME_SIZE_BY_DATA_ID[args.data_id]
    counts = []

    with serial.Serial(args.port, args.baud, timeout=args.timeout) as port:
        port.reset_input_buffer()
        for _ in range(args.count):
            port.write(bytes((cf,)))
            counts.append(decode_frame(read_exact(port, frame_size), args.data_id)["counts"])

    deltas = [signed_delta(b, a) for a, b in zip(counts, counts[1:])]
    distinct = sorted(set(counts))
    moved = sum(1 for d in deltas if d != 0)

    print(f"stationary hold: frames={len(counts)} distinct_positions={len(distinct)} "
          f"span={max(counts) - min(counts)} counts nonzero_deltas={moved}")
    if len(distinct) <= 8:
        print(f"  positions seen: {distinct}")
    if deltas:
        print(f"  delta range: {min(deltas)} .. {max(deltas)} counts")
    print(f"  0.015 deg dead-band = {0.015 * 65536 / 360.0:.1f} counts; "
          f"1-count hysteresis on top")


def run_log(args):
    """Timestamped position log for offline analysis.

    Doubles as the INL acceptance test: rotate slowly and steadily, then Fourier
    analyse position against a fitted ramp. Keep the speed low -- the published
    angle passes a 100 Hz tracking observer, and the dominant INL ripple sits at
    mechanical order 64, i.e. 64 * rpm / 60 Hz, so above roughly 60 rpm the very
    ripple being measured is filtered away.
    """
    import serial

    cf = CF_BY_DATA_ID[args.data_id]
    frame_size = FRAME_SIZE_BY_DATA_ID[args.data_id]
    rows = []

    with serial.Serial(args.port, args.baud, timeout=args.timeout) as port:
        port.reset_input_buffer()
        t0 = time.perf_counter_ns()
        for _ in range(args.count):
            port.write(bytes((cf,)))
            frame = read_exact(port, frame_size)
            t = time.perf_counter_ns() - t0
            decoded = decode_frame(frame, args.data_id)
            rows.append((t / 1e9, decoded["counts"], decoded["status"]))

    with open(args.log, "w", encoding="utf-8", newline="") as handle:
        handle.write("time_s,counts,angle_deg,status\n")
        for t, c, sf in rows:
            handle.write(f"{t:.6f},{c},{c * 360.0 / 65536.0:.6f},{sf}\n")

    duration = rows[-1][0] - rows[0][0] if len(rows) > 1 else 0.0
    rate = (len(rows) - 1) / duration if duration > 0 else 0.0
    print(f"wrote {len(rows)} samples to {args.log} over {duration:.3f} s "
          f"({rate:.0f} frames/s)")


def parse_int(value):
    return int(value, 0)


def parse_args():
    parser = argparse.ArgumentParser(description="Validate T-Format ID0/ID3 and the private debug diagnostic frame.")
    parser.add_argument("--port", help="Serial port, for example COM12")
    parser.add_argument("--data-id", type=int, choices=CF_BY_DATA_ID, default=3)
    parser.add_argument("--baud", type=int, default=2_500_000)
    parser.add_argument("--count", type=int, default=1000)
    parser.add_argument("--timeout", type=float, default=0.1, help="Read timeout in seconds")
    parser.add_argument("--interval-ms", type=float, default=1.0)
    parser.add_argument("--expect-enid", type=parse_int, default=ENID_16BIT)
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--diag", action="store_true", help="Read the private DEBUG diagnostic frame")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--hold-test", action="store_true",
                        help="Poll a stationary shaft and report the output dead-band / hysteresis")
    parser.add_argument("--log", metavar="FILE",
                        help="Write a timestamped position CSV instead of the summary test")
    return parser.parse_args()


def main():
    args = parse_args()
    try:
        run_self_test()
        if args.self_test:
            return 0
        if not args.port:
            raise RuntimeError("--port is required unless --self-test is used")
        if args.hold_test:
            run_hold_test(args)
        elif args.log:
            run_log(args)
        else:
            if args.diag:
                run_diag_test(args)
            else:
                run_serial_test(args)
    except (AssertionError, RuntimeError, ValueError, ImportError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
