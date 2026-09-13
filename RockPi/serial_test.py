#!/usr/bin/env python3
"""Simple STM32 serial tester for the mecanum car.

Use this before ROS:
  Windows: python serial_test.py COM5 --baud 9600
  Linux:   python3 serial_test.py /dev/ttyUSB0 --baud 9600

Then type short commands:
  ?        ping
  D        read IMU data
  U        read MPU id
  YAW0     reset yaw
  HOLD 1   heading hold on
  HOLD 0   heading hold off
  M 10 0 0 forward slowly
  M 0 10 0 strafe left slowly
  T 90     turn to relative/target heading per STM32 firmware
  S        stop
"""

import argparse
import sys
import threading
import time

try:
    import serial
except ImportError:
    print("Missing dependency: pyserial")
    print("Install it with: pip install pyserial")
    sys.exit(1)


def reader(port, stop_event):
    while not stop_event.is_set():
        try:
            line = port.readline()
        except serial.SerialException as exc:
            print(f"\n[serial error] {exc}")
            stop_event.set()
            break
        if line:
            text = line.decode("utf-8", errors="replace").rstrip("\r\n")
            print(f"< {text}")


def main():
    parser = argparse.ArgumentParser(description="Test STM32 car serial protocol")
    parser.add_argument("port", help="Serial port, for example COM5 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=9600, help="Baud rate, default 9600")
    parser.add_argument("--timeout", type=float, default=0.2, help="Read timeout seconds")
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=args.timeout)
    except serial.SerialException as exc:
        print(f"Cannot open {args.port}: {exc}")
        sys.exit(1)

    stop_event = threading.Event()
    thread = threading.Thread(target=reader, args=(ser, stop_event), daemon=True)
    thread.start()

    print(f"Opened {args.port} at {args.baud} baud")
    print("Type commands, or 'exit' to quit. First try: ?")

    try:
        while True:
            command = input("> ").strip()
            if command.lower() in {"exit", "quit"}:
                break
            if not command:
                continue
            ser.write((command + "\n").encode("ascii", errors="ignore"))
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            ser.write(b"S\n")
        except serial.SerialException:
            pass
        stop_event.set()
        ser.close()
        print("Closed serial port")


if __name__ == "__main__":
    main()
