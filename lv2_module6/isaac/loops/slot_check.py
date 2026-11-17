"""Real-robot basket slot check: send the arm to each slot pose with goto.py --auto (carry_<side> -> q_over -> --stage,
no stop at the vias), then type the error you measure there. One row per slot in logs/slot_check_<ts>.csv.
Run while ./manipulator-launcher.sh is up.

    python3 slot_check.py                              # slots 0..11, q_low, box table
    python3 slot_check.py --slots 0 1 4 5 --stage over
    python3 slot_check.py --shape cyl --speed 10

Error = where the hand (or the puck in it) is minus the slot centre, in mm, robot frame: dx forward, dy left, dz up
(dz optional). At the prompt: "dx dy [dz] [# note]", Enter = skip (no measurement), r = move to this slot again, q = quit.
Typing the error starts the move to the next slot (the first move waits for Enter).
goto.py also writes its own logs/goto_<ts>*.csv per slot (joint offset / shake at the slot).
"""

import argparse
import csv
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--slots", type=int, nargs="+", default=list(range(12)), help="slots to visit, in this order")
parser.add_argument("--stage", choices=["over", "low", "lift", "grasp"], default="low", help="slot pose to stop at")
parser.add_argument("--shape", choices=["box", "cyl"], default="box", help="puck shape table")
parser.add_argument("--speed", type=float, default=15.0, help="goto.py --speed [deg/s]")
parser.add_argument("--hold", type=float, default=1.0, help="goto.py --hold at the slot [s]")
parser.add_argument("--self-test", action="store_true")


def parse_error(text):
    """'3 -1.5' / '3 -1.5 2 # bent' -> (dx, dy, dz, note); '' -> None. ValueError on anything else"""
    vals, _, note = text.partition("#")
    nums = [float(v) for v in vals.replace(",", " ").split()]
    if not nums and not note.strip():
        return None
    if len(nums) not in (2, 3):
        raise ValueError("dx dy [dz] in mm")
    return (*nums, *([""] * (3 - len(nums))), note.strip())


def self_test():
    assert parse_error("") is None
    assert parse_error("3 -1.5") == (3.0, -1.5, "", "")
    assert parse_error("3,-1.5,2 # bent") == (3.0, -1.5, 2.0, "bent")
    for bad in ("3", "1 2 3 4", "x y"):
        try:
            parse_error(bad)
        except ValueError:
            continue
        raise AssertionError(bad)
    print("self-test ok")


def main():
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    if any(not 0 <= s <= 11 for s in args.slots):
        raise SystemExit("--slots: 0-11")
    (HERE / "logs").mkdir(exist_ok=True)
    path = HERE / "logs" / f"slot_check_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    print(f"slots {args.slots}, stage {args.stage}, shape {args.shape}, speed {args.speed} deg/s -> {path}")
    input("Enter = move to the first slot (direct JTC, no collision check), Ctrl-C = quit ")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time", "slot", "stage", "shape", "goto_ok", "dx_mm", "dy_mm", "dz_mm", "note"])
        i = 0
        while i < len(args.slots):
            slot = args.slots[i]
            print(f"\n=== slot {slot} ({args.stage}, {args.shape})")
            rc = subprocess.call([sys.executable, str(HERE / "goto.py"), "--slot", str(slot), "--stage", args.stage,
                                  "--shape", args.shape, "--speed", str(args.speed), "--hold", str(args.hold), "--auto"])
            # goto.py exits non-zero when the arm is already there ("start = target") — that is fine to measure too
            while True:
                text = input(f"slot {slot} error 'dx dy [dz] [# note]' mm, Enter = skip, r = again, q = quit: ").strip()
                if text in ("q", "r"):
                    break
                try:
                    err = parse_error(text)
                    break
                except ValueError as e:
                    print(f"  {e}")
            if text == "q":
                break
            if text == "r":
                continue
            w.writerow([time.strftime("%H:%M:%S"), slot, args.stage, args.shape, rc == 0, *(err or ("", "", "", "skip"))])
            f.flush()
            i += 1
    print(f"\nsaved {path}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nstopped")
