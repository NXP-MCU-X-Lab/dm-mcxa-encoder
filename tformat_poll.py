#!/usr/bin/env python3
"""
Tamagawa T-Format encoder polling demo (100 Hz) using pyserial.

- Sends CF ID3 (0x1A) to read ABS+ENID+ABM+ALMC
- Sends CF IDD (0xEA) to read EEPROM byte (e.g., temperature)
- Parses and prints values each cycle

Minimal options: only --port. Other parameters fixed for simplicity.
"""

import argparse
import sys
import time
import serial

# CF codes
CF_ID0_ABS = 0x02
CF_ID3_ALL = 0x1A
CF_IDD_EEPROM_READ = 0xEA

# Expected response lengths
RESP_LEN_ID0 = 1 + 1 + 3 + 1  # CF + SF + ABS0..ABS2 + CRC = 6
RESP_LEN_ID3 = 1 + 1 + 8 + 1  # CF + SF + ABS0..ABS2 + ENID + ABM0..2 + ALMC + CRC = 11
RESP_LEN_IDD = 1 + 1 + 1 + 1  # CF + ADF + EDF + CRC = 4


# Fixed parameters for demo
BAUD = 2_500_000
RATE_HZ = 100.0
TIMEOUT_S = 0.05
CRC_MODE = 'xor'   # fixed: XOR CRC
TEMP_ADF = 0xE1    # fixed ADF for temperature


def crc_xor(data: bytes) -> int:
    c = 0
    for b in data:
        c ^= b
    return c & 0xFF


def calc_crc(data: bytes) -> int:
    return crc_xor(data)


def read_exact(ser: serial.Serial, n: int, timeout_s: float) -> bytes:
    deadline = time.monotonic() + timeout_s
    buf = bytearray()
    while len(buf) < n and time.monotonic() < deadline:
        chunk = ser.read(n - len(buf))
        if chunk:
            buf.extend(chunk)
    return bytes(buf)


def request_id3(ser: serial.Serial, timeout_s: float) -> dict:
    # Request
    req = bytes([CF_ID3_ALL])
    ser.reset_input_buffer()
    ser.write(req)
    # Response: 11 bytes
    resp = read_exact(ser, RESP_LEN_ID3, timeout_s)
    if len(resp) != RESP_LEN_ID3:
        raise TimeoutError('ID3 response timeout: got %d bytes' % len(resp))
    if resp[0] != CF_ID3_ALL:
        raise ValueError('ID3 bad CF echo: 0x%02X' % resp[0])
    crc_calc = calc_crc(resp[:-1])
    crc_resp = resp[-1]
    if crc_calc != crc_resp:
        raise ValueError('ID3 CRC mismatch: calc=0x%02X resp=0x%02X' % (crc_calc, crc_resp))

    sf = resp[1]
    abs_val = resp[2] | (resp[3] << 8) | (resp[4] << 16)
    enid = resp[5]
    abm_val = resp[6] | (resp[7] << 8) | (resp[8] << 16)
    almc = resp[9]
    return {
        'sf': sf,
        'abs': abs_val,
        'enid': enid,
        'abm': abm_val,
        'almc': almc,
        'crc': crc_resp,
    }


def request_idd(ser: serial.Serial, adf: int, timeout_s: float) -> dict:
    # Build request: CF + ADF + CRC(CF+ADF)
    cf = CF_IDD_EEPROM_READ
    req_wo_crc = bytes([cf, adf & 0xFF])
    req_crc = calc_crc(req_wo_crc)
    req = req_wo_crc + bytes([req_crc])
    ser.reset_input_buffer()
    ser.write(req)
    # Response: 4 bytes
    resp = read_exact(ser, RESP_LEN_IDD, timeout_s)
    if len(resp) != RESP_LEN_IDD:
        raise TimeoutError('IDD response timeout: got %d bytes' % len(resp))
    if resp[0] != cf or resp[1] != (adf & 0xFF):
        raise ValueError('IDD bad echo: CF=0x%02X ADF=0x%02X' % (resp[0], resp[1]))
    crc_calc = calc_crc(resp[:-1])
    crc_resp = resp[-1]
    if crc_calc != crc_resp:
        raise ValueError('IDD CRC mismatch: calc=0x%02X resp=0x%02X' % (crc_calc, crc_resp))
    edf = resp[2]
    return {
        'edf': edf,
        'crc': crc_resp,
    }


def main():
    ap = argparse.ArgumentParser(description='Tamagawa T-Format 100Hz polling demo (minimal)')
    ap.add_argument('--port', required=True, help='Serial port (e.g., COM4)')
    args = ap.parse_args()

    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=BAUD,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=TIMEOUT_S,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
            write_timeout=0.05,
        )
    except Exception as e:
        print('Error opening serial port:', e)
        return 1

    print('Polling %s at %.1f Hz, baud=%d' % (args.port, RATE_HZ, BAUD))

    interval = 1.0 / RATE_HZ
    next_t = time.monotonic()
    try:
        while True:
            # Send ID3 and parse
            try:
                data = request_id3(ser, timeout_s=TIMEOUT_S)
            except Exception as e:
                print('ID3 error:', e)
                data = None

            # Send IDD (temperature)
            try:
                tdata = request_idd(ser, adf=TEMP_ADF, timeout_s=TIMEOUT_S)
            except Exception as e:
                print('IDD error:', e)
                tdata = None

            if data is not None:
                print('[ABS=%7d | ABM=%7d | ENID=0x%02X | ALMC=0x%02X | SF=0x%02X | CRC=0x%02X] '
                      % (data['abs'], data['abm'], data['enid'], data['almc'], data['sf'], data['crc']), end='')
            else:
                print('[ABS/ABM read failed] ', end='')

            if tdata is not None:
                print('TEMP(raw)=0x%02X (CRC=0x%02X)' % (tdata['edf'], tdata['crc']))
            else:
                print('TEMP read failed')

            next_t += interval
            sleep_s = next_t - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                # If behind schedule, reset schedule
                next_t = time.monotonic()
    except KeyboardInterrupt:
        print('\nStopped by user')
    finally:
        try:
            ser.close()
        except Exception:
            pass
    return 0


if __name__ == '__main__':
    sys.exit(main())