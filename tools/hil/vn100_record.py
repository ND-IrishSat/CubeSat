"""Record VN-100 data to CSV for offline filter work. Not flight code.

Run from the repo root:
    python tools\\hil\\vn100_record.py --port COM5
    python tools\\hil\\vn100_record.py --port COM5 --duration 60 --still 5 --note slow-rotations

Stop early with Ctrl+C (the file is still saved).

Columns (SI units, sensor body frame):
    t                    seconds since the recording started
    gyro_x, _y, _z       rad/s
    mag_x, _y, _z        tesla
    accel_x, _y, _z      m/s^2
    vn_qw, _qx, _qy, _qz VN-100 onboard attitude [w, x, y, z]: the reference to compare against
    valid                1 if every reading in the row was valid, else 0
    phase                "still" during the first --still seconds, then "move"
"""
from __future__ import annotations

import argparse
import csv
import re
import time
from datetime import datetime
from pathlib import Path

from cloversat.config.device_params import Vn100Params
from cloversat.devices.hardware.vn100_driver import Vn100Driver

HEADER = [
    "t",
    "gyro_x", "gyro_y", "gyro_z",
    "mag_x", "mag_y", "mag_z",
    "accel_x", "accel_y", "accel_z",
    "vn_qw", "vn_qx", "vn_qy", "vn_qz",
    "valid", "phase",
]


def fmt(values) -> list[str]:
    """Numbers -> compact strings (keeps tiny tesla values like 2.2e-05 exact enough)."""
    return [f"{float(v):.7g}" for v in values]


def default_path(note: str) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    slug = re.sub(r"[^A-Za-z0-9_-]+", "-", note).strip("-")
    name = f"vn100_{stamp}" + (f"_{slug}" if slug else "") + ".csv"
    return Path("logs") / "vn100" / name


def main() -> int:
    ap = argparse.ArgumentParser(description="Record VN-100 data to CSV.")
    ap.add_argument("--port", required=True, help="e.g. COM5")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--duration", type=float, default=0.0,
                    help="seconds to record (0 = until Ctrl+C)")
    ap.add_argument("--still", type=float, default=5.0,
                    help="seconds at the start to keep the board still")
    ap.add_argument("--note", default="", help="short label added to the file name")
    ap.add_argument("--out", help="output path (default: logs/vn100/vn100_<time>_<note>.csv)")
    ap.add_argument("--countdown", type=int, default=3,
                    help="seconds of 3-2-1 countdown before recording starts")
    ap.add_argument("--end-still", type=float, default=10.0,
                    help="seconds at the end to put the board back and keep it still "
                         "(only used with --duration)")
    args = ap.parse_args()

    out = Path(args.out) if args.out else default_path(args.note)
    out.parent.mkdir(parents=True, exist_ok=True)

    imu = Vn100Driver(Vn100Params(port=args.port, baud=args.baud))
    try:
        imu.init()
    except Exception as e:
        print(f"Could not connect to the VN-100 on {args.port}: {e}")
        return 1

    print(f"Connected: {imu.model}")
    print(f"Writing {out}")
    print(f"Get ready: hold the board STILL for the first {args.still:.0f} s.")
    for i in range(args.countdown, 0, -1):
        print(f"  Starting in {i}...", end="\r")
        time.sleep(1)
    print("  Recording!          ")

    t0 = None
    rows = invalid = 0
    #t0 = None
    #announced_move = False
    #announced_move = False
    
    last_phase = None
    end_start = args.duration - args.end_still if args.duration else None
    moves = ["TILT front up/down", "ROLL left/right", "TURN left/right"]
    last_status = time.monotonic()
    last_flush = time.monotonic()
    
    try:
        with open(out, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(HEADER)
            while True:
                gyro, mag, accel = imu.read_all()
                q = imu.read_attitude()

                if t0 is None:
                    t0 = gyro.t
                t = gyro.t - t0
                if args.duration and t >= args.duration:
                    break

                if t < args.still:
                    phase = "still"
                elif end_start is not None and t >= end_start:
                    phase = "end_still"
                else:
                    phase = "move"

                # which guided move we're on: the move time is split into 3 equal parts
                move_part = ""
                if phase == "move":
                    move_len = (end_start - args.still) if end_start else 45.0
                    idx = min(int((t - args.still) / (move_len / 3)), 2)
                    move_part = moves[idx]

                announce = f"{phase}:{move_part}"
                if announce != last_phase:            # beep + message on every change
                    if phase == "move":
                        print(f"\a\n>>> {move_part}  (slowly, ~2-3 s per swing)")
                    elif phase == "end_still":
                        print("\a\n>>> PUT IT BACK on the start mark and LET GO.")
                    last_phase = announce

                ok = gyro.valid and mag.valid and accel.valid and q.valid
                writer.writerow([f"{t:.4f}", *fmt(gyro.value), *fmt(mag.value),
                                 *fmt(accel.value), *fmt(q.value), int(ok), phase])
                rows += 1
                invalid += not ok

                now = time.monotonic()
                if now - last_status >= 0.25:      # refresh the countdown 4x per second
                    if phase == "still":
                        msg = f"HOLD STILL        {args.still - t:4.1f} s left"
                    elif phase == "end_still":
                        msg = f"HOLD STILL (end)  {args.duration - t:4.1f} s left"
                    elif end_start is not None:
                        msg = f"{move_part:<20s} {end_start - t:4.1f} s until put-back"
                    else:
                        msg = f"{move_part:<20s} {t:5.1f} s   (Ctrl+C to stop)"
                    print(f"  {msg}   rows={rows}  invalid={invalid}     ", end="\r")
                    last_status = now
                if now - last_flush >= 1.0:        # save to disk once a second
                    f.flush()
                    last_flush = now
    except KeyboardInterrupt:
        pass
    finally:
        imu.close()

    print(f"\nSaved {rows} rows ({invalid} invalid) to {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())