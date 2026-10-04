"""
Talk to the robot AND record its telemetry to a CSV, in one terminal.

    python log_serial.py --port /dev/ttyUSB0 --out ../data/run_001.csv
    (Windows: --port COM5)

* Everything the robot prints is shown on screen.
* Lines starting with '#' are messages and are NOT written to the CSV.
* A line starting with 't_ms' is the column header; the lines after it are data.
* Whatever you type is sent to the robot, e.g.  mode balance   kp 20   outer 1   x
* Press Ctrl+C to stop. Capture one mode per file (the columns differ per mode).
"""
import argparse
import sys
import threading
import time

import serial  # pip install pyserial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="e.g. /dev/ttyUSB0 or COM5 (use loop:// for a self-test)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", default="run.csv")
    args = ap.parse_args()

    ser = serial.serial_for_url(args.port, args.baud, timeout=0.1)
    time.sleep(2.0)  # Arduino resets when the port opens
    stop = threading.Event()

    def send_loop():
        for line in sys.stdin:
            ser.write((line.strip() + "\n").encode())
            if stop.is_set():
                break

    threading.Thread(target=send_loop, daemon=True).start()

    rows = 0
    header_written = False
    with open(args.out, "w", newline="") as f:
        try:
            while True:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode(errors="ignore").strip()
                if not line:
                    continue
                print(line)
                if line.startswith("#"):
                    continue
                if line.startswith("t_ms"):
                    if not header_written:
                        f.write(line + "\n")
                        header_written = True
                    continue
                if header_written:
                    f.write(line + "\n")
                    rows += 1
                    f.flush()
        except KeyboardInterrupt:
            pass
        finally:
            stop.set()
            ser.write(b"x\n")  # always leave the robot in idle (motors off)
            ser.close()
    print(f"\nSaved {rows} rows to {args.out}")


if __name__ == "__main__":
    main()
