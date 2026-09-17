#!/usr/bin/env python3
"""Decode the final 1 Mbps G030 angle/contact telemetry packet.

Example:
    python decode_g030_encoder_uart.py --port /dev/ttyUSB0
"""

import argparse
import struct
import sys
import time

try:
    import serial
except ImportError as exc:
    raise SystemExit("Missing pyserial. Install with: python -m pip install pyserial") from exc


SOF = b"\xA5\x5A"
FRAME_SIZE = 19


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def decode(frame: bytes) -> dict:
    if len(frame) != FRAME_SIZE or frame[:2] != SOF:
        raise ValueError("bad frame boundary")
    expected_crc = struct.unpack_from("<H", frame, 17)[0]
    actual_crc = crc16_ccitt_false(frame[2:17])
    if actual_crc != expected_crc:
        raise ValueError(f"CRC {actual_crc:04X} != {expected_crc:04X}")

    return {
        "version": frame[2],
        "node": frame[3],
        "sequence": struct.unpack_from("<H", frame, 4)[0],
        "time_ms": struct.unpack_from("<I", frame, 6)[0],
        "flags": frame[10],
        "raw": struct.unpack_from("<HHH", frame, 11),
    }


def raw_to_deg(raw: int) -> float:
    return raw * 360.0 / 16384.0


def main() -> None:
    parser = argparse.ArgumentParser(description="Decode G030 1 kHz encoder telemetry")
    parser.add_argument("--port", required=True, help="USB-UART device, e.g. /dev/ttyUSB0 or COM5")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--print-hz", type=float, default=20.0, help="terminal update rate")
    args = parser.parse_args()

    period = 1.0 / args.print_hz
    buffer = bytearray()
    good_frames = 0
    bad_frames = 0
    last_print = 0.0
    latest = None

    with serial.Serial(args.port, args.baud, timeout=0.1) as uart:
        print(f"Listening on {args.port} at {args.baud} baud; Ctrl+C to stop.")
        while True:
            buffer.extend(uart.read(512))

            while True:
                start = buffer.find(SOF)
                if start < 0:
                    if len(buffer) > 1:
                        del buffer[:-1]
                    break
                if start > 0:
                    del buffer[:start]
                if len(buffer) < FRAME_SIZE:
                    break

                candidate = bytes(buffer[:FRAME_SIZE])
                try:
                    latest = decode(candidate)
                    good_frames += 1
                    del buffer[:FRAME_SIZE]
                except ValueError:
                    bad_frames += 1
                    del buffer[0]

            now = time.monotonic()
            if latest is not None and now - last_print >= period:
                flags = latest["flags"]
                degrees = [raw_to_deg(v) for v in latest["raw"]]
                valid = "".join("Y" if flags & (1 << (index + 1)) else "-" for index in range(3))
                foot = "DOWN" if flags & 0x01 else "UP"
                fault = "FAULT" if flags & 0x10 else "OK"
                print(
                    f"\rnode={latest['node']} seq={latest['sequence']:5d} "
                    f"t={latest['time_ms']:8d}ms foot={foot:4s} health={fault:5s} "
                    f"valid={valid} "
                    f"angle=[{degrees[0]:7.2f}, {degrees[1]:7.2f}, {degrees[2]:7.2f}] deg "
                    f"frames={good_frames} bad={bad_frames}",
                    end="",
                    flush=True,
                )
                last_print = now


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
        sys.exit(0)
