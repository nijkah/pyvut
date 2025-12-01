#!/usr/bin/env python3
"""Simple demo that prints live 6DoF poses using pyvut's UltimateTrackerAPI."""

import argparse
import time
from math import asin, atan2, pi

from pyvut import TrackerPose, UltimateTrackerAPI  # noqa: E402

def quat_to_euler_deg(quat) -> tuple:
    w, x, y, z = quat
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = max(min(t2, 1.0), -1.0)
    pitch = asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = atan2(t3, t4)

    return tuple(angle * 180.0 / pi for angle in (roll, pitch, yaw))

def format_pose(pose: TrackerPose) -> str:
    pos = ", ".join(f"{axis: .3f}" for axis in pose.position)
    rot = ", ".join(f"{axis: .3f}" for axis in pose.rotation)
    euler = ", ".join(f"{angle: .2f}" for angle in quat_to_euler_deg(pose.rotation))
    return (
        f"tracker={pose.tracker_index} mac={pose.mac} status={pose.tracking_status} "
        f"pos=({pos}) quat=({rot}) euler_deg=({euler}) buttons={pose.buttons:#06x}"
        f" timestamp_ms={pose.timestamp_ms}"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode",
        choices=["DONGLE_USB", "TRACKER_USB"],
        default="DONGLE_USB",
        help="Transport to use for tracker communication.",
    )
    parser.add_argument(
        "--wifi-info",
        dest="wifi_info",
        help="Optional path to a wifi_info.json file (defaults to pyvut/wifi_info.json).",
    )
    args = parser.parse_args()

    def on_pose(pose: TrackerPose) -> None:
        print(format_pose(pose))

    print(
        "Starting UltimateTrackerAPI… Rotations are reported as quaternions (w,x,y,z) and Euler angles (roll, pitch, yaw in degrees)."
        " Trackers emit raw (w,z,y,x) order but pyvut normalizes this for you. Press Ctrl+C to stop."
    )
    try:
        with UltimateTrackerAPI(mode=args.mode, wifi_info_path=args.wifi_info) as api:
            api.add_pose_callback(on_pose)
            while True:
                time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping…")


if __name__ == "__main__":
    main()
