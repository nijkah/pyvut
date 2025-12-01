#!/usr/bin/env python3
"""Demonstrate the multiprocess-based tracker interface with optional visualization."""

import argparse
import sys
import time
from typing import Optional

import numpy as np

try:
    import pygame
except ImportError:  # pragma: no cover - optional dependency
    pygame = None

from pyvut import TrackerService

POSE_REFRESH_S = 0.02


def quat_to_euler_deg(quat: np.ndarray) -> np.ndarray:
    w, x, y, z = quat
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = np.clip(t2, -1.0, 1.0)
    pitch = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(t3, t4)

    return np.degrees(np.array([roll, pitch, yaw]))


def format_pose_line(pose, age_ms: Optional[float]) -> str:
    pos = ", ".join(f"{axis: .3f}" for axis in pose.position)
    rot = ", ".join(f"{axis: .3f}" for axis in pose.rotation)
    euler = ", ".join(f"{axis: .2f}" for axis in quat_to_euler_deg(pose.rotation))
    age_text = f" age={age_ms:.1f}ms" if age_ms is not None else ""
    return (
        f"tracker={pose.tracker_index} mac={pose.mac} buttons=0x{pose.buttons:04x} status={pose.tracking_status}"
        f" pos=({pos}) quat=({rot}) euler_deg=({euler}){age_text}"
    )


# --- Simple pygame visualization helpers ------------------------------------------------------

BLACK, RED, GREEN, BLUE = (0, 0, 0), (255, 0, 0), (0, 255, 0), (0, 128, 255)


def quaternion_rotation_matrix(quat: np.ndarray) -> np.ndarray:
    w, x, y, z = quat
    r00 = 2 * (w * w + x * x) - 1
    r01 = 2 * (x * y - w * z)
    r02 = 2 * (x * z + w * y)
    r10 = 2 * (x * y + w * z)
    r11 = 2 * (w * w + y * y) - 1
    r12 = 2 * (y * z - w * x)
    r20 = 2 * (x * z - w * y)
    r21 = 2 * (y * z + w * x)
    r22 = 2 * (w * w + z * z) - 1
    return np.array([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]])


def draw_axes(surface, origin, quat, scale=100.0):
    rot_mat = quaternion_rotation_matrix(quat)
    axes = rot_mat * scale
    colors = (RED, GREEN, BLUE)
    for idx in range(3):
        end = origin + axes[:, idx]
        pygame.draw.line(surface, colors[idx], origin[:2], end[:2], 4)


class SimpleVisualizer:
    def __init__(self, tracker_service: TrackerService, tracker_index: int, window_size=(800, 800)):
        if pygame is None:
            raise RuntimeError("pygame is required for visualization but is not installed")
        self._service = tracker_service
        self._tracker_index = tracker_index
        self._window_size = window_size

    def run(self):
        pygame.init()
        pygame.display.set_caption("Multiprocess Tracker Visualizer")
        screen = pygame.display.set_mode(self._window_size)
        font = pygame.font.SysFont("Consolas", 20)
        clock = pygame.time.Clock()

        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            screen.fill(BLACK)
            pose = self._service.get_pose(self._tracker_index)
            if pose:
                center = np.array(self._window_size) / 2.0
                pos_px = center + pose.position[:2] * -300.0
                pygame.draw.circle(screen, (255, 255, 255), pos_px, 8)
                draw_axes(screen, np.array([*pos_px, 0.0]), pose.rotation)
                text = format_pose_line(pose, self._service.last_pose_age_ms)
                surface = font.render(text, True, (255, 255, 0))
                screen.blit(surface, (20, 20))
            else:
                surface = font.render("Waiting for tracker data...", True, (255, 255, 0))
                screen.blit(surface, (20, 20))

            pygame.display.flip()
            clock.tick(60)

        pygame.quit()


# --- CLI --------------------------------------------------------------------------------------


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tracker-index", type=int, default=0, help="Tracker index (0-4) to monitor")
    parser.add_argument(
        "--tracker-mode",
        choices=["DONGLE_USB", "TRACKER_USB"],
        default="DONGLE_USB",
        help="Transport used for Vive tracker communication",
    )
    parser.add_argument("--wifi-info", dest="wifi_info", help="Optional path to wifi_info.json")
    parser.add_argument("--visualize", action="store_true", help="Enable pygame visualization instead of terminal logs")
    parser.add_argument("--refresh", type=float, default=POSE_REFRESH_S, help="Seconds between terminal pose prints")
    return parser.parse_args()


def main():
    args = parse_args()
    tracker_service = TrackerService(mode=args.tracker_mode, wifi_info_path=args.wifi_info)

    try:
        if args.visualize:
            if pygame is None:
                raise RuntimeError("pygame is not installed; install it or omit --visualize")
            vis = SimpleVisualizer(tracker_service, args.tracker_index)
            vis.run()
        else:
            print("Streaming tracker poses via multiprocess TrackerService. Press Ctrl+C to stop.")
            while True:
                pose = tracker_service.get_pose(args.tracker_index)
                if pose is not None:
                    print(format_pose_line(pose, tracker_service.last_pose_age_ms))
                else:
                    print("Waiting for tracker data...")
                time.sleep(args.refresh)
    except KeyboardInterrupt:
        print("\nStopping…")
    finally:
        tracker_service.stop()


if __name__ == "__main__":
    main()
