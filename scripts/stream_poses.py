#!/usr/bin/env python3
"""Simple demo that prints live 6DoF poses using pyvut's UltimateTrackerAPI."""

import argparse
import pathlib
import sys
import time

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from pyvut import TrackerPose, UltimateTrackerAPI  # noqa: E402


def format_pose(pose: TrackerPose) -> str:
    pos = ", ".join(f"{axis: .3f}" for axis in pose.position)
    rot = ", ".join(f"{axis: .3f}" for axis in pose.rotation)
    return (
        f"tracker={pose.tracker_index} mac={pose.mac} status={pose.tracking_status} "
        f"pos=({pos}) rot=({rot}) buttons={pose.buttons:#06x}"
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

    print("Starting UltimateTrackerAPI… Press Ctrl+C to stop.")
    try:
        with UltimateTrackerAPI(mode=args.mode, wifi_info_path=args.wifi_info) as api:
            api.add_pose_callback(on_pose)
            while True:
                time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping…")


if __name__ == "__main__":
    main()
