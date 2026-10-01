#!/usr/bin/env python3
"""Tell physical trackers apart: pick one up and rotate it, and its dongle slot lights up.

Prints one live line per tracker with a motion bar based on how fast its orientation
changes. Rotation works without SLAM, so no map or init routine is needed.
"""

import argparse
import sys
import time

import numpy as np

from pyvut import UltimateTrackerAPI
from pyvut.tracker_core import set_tracker_core_verbose


def mac_slot(mac: str) -> int:
    return int(mac.split(":")[1], 16) & 0xF


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host-slot", type=int, default=None,
                        help="Dongle slot to make the SLAM host; default is the first tracker heard from.")
    args = parser.parse_args()

    api = UltimateTrackerAPI(host_slot=args.host_slot)
    set_tracker_core_verbose(False)
    last_quat = {}
    motion = {}

    print("Pick up one tracker at a time and rotate it. Ctrl+C to stop.\n")
    with api:
        try:
            while True:
                for pose in api.iter_latest_poses():
                    slot = mac_slot(pose.mac)
                    q = np.asarray(pose.rotation, dtype=np.float64)
                    if slot in last_quat:
                        # angle between orientations (deg), smoothed
                        dot = abs(float(np.dot(q, last_quat[slot]))) / max(np.linalg.norm(q) * np.linalg.norm(last_quat[slot]), 1e-9)
                        angle = np.degrees(2 * np.arccos(min(dot, 1.0)))
                        motion[slot] = 0.7 * motion.get(slot, 0.0) + 0.3 * angle
                    last_quat[slot] = q

                lines = []
                for slot in sorted(last_quat):
                    m = motion.get(slot, 0.0)
                    bar = "#" * min(int(m * 2), 30)
                    role = "HOST  " if api.tracker_group.comms.is_host(slot) else "client"
                    moving = "  <-- MOVING" if m > 2.0 else ""
                    lines.append(f"slot {slot} [{role}] |{bar:<30}|{moving}")
                sys.stdout.write("\x1b[H\x1b[J" + "\n".join(lines or ["waiting for trackers..."]) + "\n")
                sys.stdout.flush()
                time.sleep(0.1)
        except KeyboardInterrupt:
            print()


if __name__ == "__main__":
    main()
