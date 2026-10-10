"""Command-line Tracker serial discovery."""

import argparse
import time

from .driver import ViveTracker


def main() -> int:
    parser = argparse.ArgumentParser(description="List Vive Tracker devices seen by libsurvive")
    parser.add_argument("--seconds", type=float, default=5.0)
    args = parser.parse_args()
    tracker = ViveTracker()
    tracker.connect()
    try:
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            for name, info in tracker.get_devices().items():
                print(f"name={name} | serial={info.get('serial')}")
            time.sleep(0.5)
    finally:
        tracker.disconnect()
    return 0
