#!/usr/bin/env python3
"""
serial_monitor.py

Lightweight serial monitor for diagnosing the Aero hand serial responses.
Usage:
  ./scripts/serial_monitor.py /dev/ttyUSB0 921600 --duration 10

Prints timestamp, hex dump and byte count for each read chunk.
"""

import argparse
import serial
import time
import sys


def main():
    p = argparse.ArgumentParser(description="Serial hex monitor")
    p.add_argument("port", help="Serial port path, e.g. /dev/ttyUSB0")
    p.add_argument("baud", nargs="?", type=int, default=921600, help="Baud rate (default 921600)")
    p.add_argument("--duration", "-d", type=float, default=10.0, help="Monitor duration in seconds (default 10s)")
    p.add_argument("--timeout", "-t", type=float, default=0.05, help="Serial read timeout in seconds (default 0.05)")
    p.add_argument("--max-read", type=int, default=256, help="Max bytes to read per call (default 256)")
    args = p.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=args.timeout)
    except Exception as e:
        print(f"Failed to open {args.port} @ {args.baud}: {e}")
        sys.exit(2)

    print(f"Opened {args.port} @ {args.baud}, monitoring for {args.duration}s (timeout={args.timeout})")
    start = time.time()
    try:
        while time.time() - start < args.duration:
            data = ser.read(args.max_read)
            now = time.time()
            if data:
                # print time, length and hex
                print(f"{now:.6f} | len={len(data):3d} | {data.hex()}")
            else:
                # no data read during timeout
                print(f"{now:.6f} | len=0 | <no data>")
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        ser.close()
        print("Closed")


if __name__ == '__main__':
    main()
