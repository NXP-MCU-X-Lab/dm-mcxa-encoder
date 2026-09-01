#!/usr/bin/env python3

import argparse
import statistics
import sys
import time


CF_BY_DATA_ID = {
    0x0: 0x02,
    0x1: 0x8A,
    0x2: 0x92,
    0x3: 0x1A,
    0x6: 0x32,
    0x7: 0xBA,
    0x8: 0xC2,
    0xC: 0x62,
    0xD: 0xEA,
}
RESPONSE_SIZE = {
    0x0: 6,
    0x1: 6,
    0x2: 4,
    0x3: 11,
    0x6: 4,
    0x7: 6,
    0x8: 6,
    0xC: 6,
    0xD: 4,
}
RESET_DATA_IDS = {"error": 0x7, "position": 0x8, "multiturn": 0xC}
RESET_COUNT = 10
RESET_INTERVAL_S = 40e-6

SF_COUNTING_ERROR = 0x10
ADF_BUSY = 0x80
ADF_ADDRESS_MASK = 0x7F
ENID_16BIT = 0x10
EEPROM_PAGE_SIZE = 127
EEPROM_DATA_PAGES = 6
EEPROM_PAGE_COUNT = 8

SF_NAMES = (
    (0x10, "counting-error"),
    (0x20, "equipment-alarm"),
    (0x40, "parity-error"),
    (0x80, "delimiter-error"),
)

ALMC_NAMES = (
    (0x01, "overspeed"),
    (0x02, "full-absolute"),
    (0x04, "counting-error"),
    (0x08, "counter-overflow"),
    (0x10, "overheat"),
    (0x20, "multiturn-error"),
    (0x40, "battery-error"),
    (0x80, "battery-alarm"),
)


def crc8_tformat(data):
    remainder = 0
    for byte in data:
        remainder ^= byte
        for _ in range(8):
            if remainder & 0x80:
                remainder = ((remainder << 1) ^ 0x01) & 0xFF
            else:
                remainder = (remainder << 1) & 0xFF
    return remainder


def with_crc(data):
    frame = bytearray(data)
    frame.append(crc8_tformat(frame))
    return bytes(frame)


def read_exact(port, size):
    data = bytearray()
    while len(data) < size:
        chunk = port.read(size - len(data))
        if not chunk:
            break
        data.extend(chunk)
    if len(data) != size:
        raise TimeoutError(f"expected {size} bytes, got {len(data)}")
    return bytes(data)


def check_frame(frame, data_id):
    expected_size = RESPONSE_SIZE[data_id]
    expected_cf = CF_BY_DATA_ID[data_id]
    if len(frame) != expected_size:
        raise ValueError(f"expected {expected_size} bytes, got {len(frame)}")
    if frame[0] != expected_cf:
        raise ValueError(f"CF 0x{frame[0]:02X}, expected 0x{expected_cf:02X}")
    expected_crc = crc8_tformat(frame[:-1])
    if frame[-1] != expected_crc:
        raise ValueError(f"CRC 0x{frame[-1]:02X}, expected 0x{expected_crc:02X}")


def decode_response(frame, data_id):
    check_frame(frame, data_id)
    result = {"data_id": data_id, "sf": None, "counts": None,
              "angle_deg": None, "abm": None, "enid": None, "almc": None}

    if data_id in (0x6, 0xD):
        result["address"] = frame[1] & ADF_ADDRESS_MASK
        result["busy"] = bool(frame[1] & ADF_BUSY)
        result["data"] = frame[2]
        return result

    result["sf"] = frame[1]
    if data_id in (0x0, 0x3, 0x7, 0x8, 0xC):
        if frame[4] != 0:
            raise ValueError(f"ABS2 is not zero: 0x{frame[4]:02X}")
        result["counts"] = frame[2] | (frame[3] << 8)
        result["angle_deg"] = result["counts"] * 360.0 / 65536.0
    if data_id == 0x1:
        result["abm"] = frame[2] | (frame[3] << 8) | (frame[4] << 16)
    elif data_id == 0x2:
        result["enid"] = frame[2]
    elif data_id == 0x3:
        result["enid"] = frame[5]
        result["abm"] = frame[6] | (frame[7] << 8) | (frame[8] << 16)
        result["almc"] = frame[9]
    return result


def describe_almc(value):
    if value == 0:
        return "none"
    return "|".join(name for bit, name in ALMC_NAMES if value & bit)


def describe_sf(value):
    if value == 0:
        return "none"
    return "|".join(name for bit, name in SF_NAMES if value & bit)


def position_request(data_id):
    return bytes((CF_BY_DATA_ID[data_id],))


def eeprom_read_request(address):
    return with_crc((CF_BY_DATA_ID[0xD], address & ADF_ADDRESS_MASK))


def eeprom_write_request(address, value):
    return with_crc((CF_BY_DATA_ID[0x6], address & ADF_ADDRESS_MASK, value & 0xFF))


def select_eeprom_page(port, page):
    response = transact(port, eeprom_write_request(ADF_ADDRESS_MASK, page), 0x6)
    if response["busy"]:
        raise RuntimeError("EEPROM is busy; keep the shaft stationary and retry")
    if response["address"] != ADF_ADDRESS_MASK or response["data"] != page:
        raise ValueError(f"page select returned 0x{response['data']:02X}, expected {page}")


def read_eeprom_byte(port, address):
    return transact(port, eeprom_read_request(address), 0xD)


def write_eeprom_byte(port, address, value, args):
    deadline = time.monotonic() + args.write_timeout
    response = transact(port, eeprom_write_request(address, value), 0x6)
    if response["address"] != address or response["data"] != value:
        raise ValueError("ID6 response does not echo ADF/EDF")

    while response["busy"] and time.monotonic() < deadline:
        time.sleep(max(args.interval_ms, 0.1) / 1000.0)
        try:
            response = read_eeprom_byte(port, address)
        except TimeoutError:
            port.reset_input_buffer()
    if response["busy"]:
        raise RuntimeError("EEPROM write remains busy; keep the shaft stationary")
    if response["data"] != value:
        raise ValueError(f"readback 0x{response['data']:02X}, expected 0x{value:02X}")


def read_eeprom_page(port, page):
    select_eeprom_page(port, page)
    data = bytearray()
    for address in range(EEPROM_PAGE_SIZE):
        response = read_eeprom_byte(port, address)
        if response["busy"]:
            raise RuntimeError("EEPROM became busy during page read")
        data.append(response["data"])
    return bytes(data)


def print_eeprom_page(page, data):
    print(f"Page{page}")
    for offset in range(0, len(data), 16):
        values = " ".join(f"{value:02X}" for value in data[offset:offset + 16])
        print(f"  {offset:02X}: {values}")


def transact(port, request, data_id):
    port.write(request)
    return decode_response(read_exact(port, RESPONSE_SIZE[data_id]), data_id)


def open_port(args):
    try:
        import serial
    except ImportError as error:
        raise RuntimeError("pyserial is required: python -m pip install pyserial") from error
    return serial.Serial(args.port, args.baud, timeout=args.timeout)


def run_self_test():
    vectors = (
        (bytes.fromhex("02 00 34 12 00 24"), 0x0, 0x1234),
        (bytes.fromhex("02 10 34 12 00 34"), 0x0, 0x1234),
        (bytes.fromhex("92 00 10 82"), 0x2, None),
        (bytes.fromhex("1A 00 34 12 00 10 00 00 00 00 2C"), 0x3, 0x1234),
        (bytes.fromhex("EA 20 A5 6F"), 0xD, None),
    )
    for frame, data_id, counts in vectors:
        decoded = decode_response(frame, data_id)
        if counts is not None and decoded["counts"] != counts:
            raise AssertionError(f"ID{data_id:X} position decode failed")
    if eeprom_write_request(0x20, 0xA5) != bytes.fromhex("32 20 A5 B7"):
        raise AssertionError("ID6 request vector failed")
    if eeprom_read_request(0x20) != bytes.fromhex("EA 20 CA"):
        raise AssertionError("IDD request vector failed")
    if eeprom_write_request(0x7F, 1) != bytes.fromhex("32 7F 01 4C"):
        raise AssertionError("page select request vector failed")
    if EEPROM_PAGE_SIZE * EEPROM_DATA_PAGES != 762:
        raise AssertionError("EEPROM geometry failed")
    print(f"self-test passed: {len(vectors) + 4} vectors")


def run_poll(args):
    latencies_us = []
    errors = 0
    previous_counts = None
    max_step = 0

    with open_port(args) as port:
        port.reset_input_buffer()
        for index in range(args.count):
            start = time.perf_counter_ns()
            decoded = transact(port, position_request(args.data_id), args.data_id)
            latencies_us.append((time.perf_counter_ns() - start) / 1000.0)

            if decoded["sf"] and decoded["sf"] & SF_COUNTING_ERROR:
                errors += 1
            if decoded["counts"] is not None:
                if previous_counts is not None:
                    delta = ((decoded["counts"] - previous_counts + 0x8000) & 0xFFFF) - 0x8000
                    max_step = max(max_step, abs(delta))
                previous_counts = decoded["counts"]

            if args.verbose:
                fields = [f"{index:6d}", f"ID{args.data_id:X}",
                          f"SF=0x{decoded['sf']:02X}({describe_sf(decoded['sf'])})"]
                if decoded["counts"] is not None:
                    fields.extend((f"counts={decoded['counts']}",
                                   f"angle={decoded['angle_deg']:.4f} deg"))
                if decoded["abm"] is not None:
                    fields.append(f"ABM={decoded['abm']}")
                if decoded["enid"] is not None:
                    fields.append(f"ENID=0x{decoded['enid']:02X}")
                if decoded["almc"] is not None:
                    fields.append(f"ALMC={describe_almc(decoded['almc'])}")
                print("  ".join(fields))
            if args.interval_ms > 0:
                time.sleep(args.interval_ms / 1000.0)

    print(f"ID{args.data_id:X}: frames={args.count} counting_errors={errors} max_step={max_step}")
    print(f"host_round_trip_us min={min(latencies_us):.1f} "
          f"mean={statistics.fmean(latencies_us):.1f} max={max(latencies_us):.1f}")


def run_reset(args):
    data_id = RESET_DATA_IDS[args.reset]
    with open_port(args) as port:
        port.reset_input_buffer()
        for index in range(RESET_COUNT):
            decoded = transact(port, position_request(data_id), data_id)
            if index + 1 < RESET_COUNT:
                time.sleep(RESET_INTERVAL_S)
    suffix = " (unsupported single-turn product)" if args.reset == "multiturn" else ""
    print(f"{args.reset} reset sequence: {RESET_COUNT} frames{suffix}; "
          f"SF=0x{decoded['sf']:02X} counts={decoded['counts']} "
          f"angle={decoded['angle_deg']:.4f} deg")


def run_eeprom_read(args):
    address = args.eeprom_read
    with open_port(args) as port:
        port.reset_input_buffer()
        select_eeprom_page(port, args.page)
        decoded = read_eeprom_byte(port, address)
    print(f"Page{args.page}[0x{decoded['address']:02X}] = 0x{decoded['data']:02X} "
          f"busy={int(decoded['busy'])}")


def run_eeprom_write(args):
    address, value = args.eeprom_write
    with open_port(args) as port:
        port.reset_input_buffer()
        select_eeprom_page(port, args.page)
        write_eeprom_byte(port, address, value, args)
    print(f"Page{args.page}[0x{address:02X}] saved as 0x{value:02X}")


def run_eeprom_read_pages(args):
    pages = range(EEPROM_PAGE_COUNT) if args.eeprom_read_all else (args.page,)
    with open_port(args) as port:
        port.reset_input_buffer()
        for page in pages:
            print_eeprom_page(page, read_eeprom_page(port, page))


def run_eeprom_write_page(args):
    if args.page >= EEPROM_DATA_PAGES:
        raise ValueError("Page6 and Page7 are read-only")
    with open(args.eeprom_write_page, "rb") as handle:
        desired = handle.read()
    if len(desired) != EEPROM_PAGE_SIZE:
        raise ValueError(f"page image must contain exactly {EEPROM_PAGE_SIZE} bytes")

    changed = 0
    with open_port(args) as port:
        port.reset_input_buffer()
        current = read_eeprom_page(port, args.page)
        for address, value in enumerate(desired):
            if value != current[address]:
                write_eeprom_byte(port, address, value, args)
                changed += 1
        if read_eeprom_page(port, args.page) != desired:
            raise ValueError("page readback verification failed")
    print(f"Page{args.page} verified; {changed} byte(s) written")


def run_log(args):
    if args.data_id not in (0x0, 0x3):
        raise RuntimeError("--log requires --data-id 0 or 3")
    rows = []
    with open_port(args) as port:
        port.reset_input_buffer()
        start = time.perf_counter_ns()
        for _ in range(args.count):
            decoded = transact(port, position_request(args.data_id), args.data_id)
            rows.append(((time.perf_counter_ns() - start) / 1e9,
                         decoded["counts"], decoded["angle_deg"], decoded["sf"]))
            if args.interval_ms > 0:
                time.sleep(args.interval_ms / 1000.0)

    with open(args.log, "w", encoding="utf-8", newline="") as handle:
        handle.write("time_s,counts,angle_deg,sf\n")
        for timestamp, counts, angle, sf in rows:
            handle.write(f"{timestamp:.9f},{counts},{angle:.6f},{sf}\n")
    print(f"wrote {len(rows)} frames to {args.log}")


def parse_int(value):
    return int(value, 0)


def parse_args():
    parser = argparse.ArgumentParser(description="MCXA344 standard T-Format test tool")
    parser.add_argument("--port", help="Serial port, for example COM78")
    parser.add_argument("--baud", type=int, default=2_500_000)
    parser.add_argument("--data-id", type=parse_int, choices=(0, 1, 2, 3), default=0)
    parser.add_argument("--count", type=int, default=1000)
    parser.add_argument("--timeout", type=float, default=0.1)
    parser.add_argument("--interval-ms", type=float, default=1.0)
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--page", type=int, choices=range(EEPROM_PAGE_COUNT), default=0)
    operation = parser.add_mutually_exclusive_group()
    operation.add_argument("--reset", choices=tuple(RESET_DATA_IDS))
    operation.add_argument("--eeprom-read", type=parse_int, metavar="ADDRESS")
    operation.add_argument("--eeprom-write", type=parse_int, nargs=2,
                           metavar=("ADDRESS", "VALUE"))
    operation.add_argument("--eeprom-read-page", action="store_true")
    operation.add_argument("--eeprom-read-all", action="store_true")
    operation.add_argument("--eeprom-write-page", metavar="FILE")
    operation.add_argument("--log", metavar="FILE")
    parser.add_argument("--write-timeout", type=float, default=5.0)
    return parser.parse_args()


def main():
    args = parse_args()
    try:
        run_self_test()
        if args.self_test:
            return 0
        if not args.port:
            raise RuntimeError("--port is required unless --self-test is used")
        if args.eeprom_read is not None and not 0 <= args.eeprom_read < EEPROM_PAGE_SIZE:
            raise ValueError("EEPROM address must be 0x00..0x7E")
        if args.eeprom_write:
            address, value = args.eeprom_write
            if not 0 <= address < EEPROM_PAGE_SIZE:
                raise ValueError("EEPROM address must be 0x00..0x7E")
            if not 0 <= value <= 0xFF:
                raise ValueError("EEPROM value must be 0x00..0xFF")
            if args.page >= EEPROM_DATA_PAGES:
                raise ValueError("Page6 and Page7 are read-only")
        if args.reset:
            run_reset(args)
        elif args.eeprom_read is not None:
            run_eeprom_read(args)
        elif args.eeprom_write:
            run_eeprom_write(args)
        elif args.eeprom_read_page or args.eeprom_read_all:
            run_eeprom_read_pages(args)
        elif args.eeprom_write_page:
            run_eeprom_write_page(args)
        elif args.log:
            run_log(args)
        else:
            run_poll(args)
    except (AssertionError, RuntimeError, ValueError, OSError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
