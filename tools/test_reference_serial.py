"""Wheel-up serial smoke test for the reference Arduino motor controller."""

from __future__ import annotations

import argparse
import time

import serial


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--pulse", type=float, default=0.6)
    args = parser.parse_args()

    with serial.Serial(args.port, args.baud, timeout=1.0) as board:
        # UNO resets when the USB serial port opens.
        time.sleep(2.0)
        board.reset_input_buffer()

        for command in ("PING", "STOP", "FORWARD", "STOP", "BACKWARD",
                        "STOP", "LEFT", "STOP", "RIGHT", "STOP"):
            board.write((command + "\n").encode("ascii"))
            board.flush()
            response = board.readline().decode("ascii", errors="replace").strip()
            print(f"{command:>8} -> {response}")
            if command not in ("PING", "STOP"):
                time.sleep(args.pulse)

        board.write(b"STOP\n")
        board.flush()


if __name__ == "__main__":
    main()
