"""High-level API for streaming 6DoF poses from VIVE Ultimate Trackers."""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, Iterable, List, Optional

import numpy as np

from .tracker_core import ViveTrackerGroup, mac_str

logger = logging.getLogger(__name__)


@dataclass
class TrackerPose:
    """Representation of a single 6DoF pose sample coming from a tracker."""

    tracker_index: int
    mac: str
    buttons: int
    tracking_status: int
    timestamp_ms: int
    position: np.ndarray
    rotation: np.ndarray
    acceleration: np.ndarray
    angular_velocity: np.ndarray


PoseCallback = Callable[[TrackerPose], None]


class UltimateTrackerAPI:
    """Convenience wrapper that exposes tracker poses through a simple API."""

    def __init__(
        self,
        mode: str = "DONGLE_USB",
        poll_interval: float = 0.001,
        wifi_info_path: Optional[str] = None,
    ) -> None:
        self._group = ViveTrackerGroup(mode=mode, wifi_info_path=wifi_info_path)
        self._poll_interval = max(0.0, poll_interval)
        self._pose_callbacks: List[PoseCallback] = []
        self._latest_pose: Dict[int, TrackerPose] = {}
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._running = threading.Event()

        self._group.add_pose_listener(self._handle_pose_event)

    @property
    def tracker_group(self) -> ViveTrackerGroup:
        """Access the underlying ViveTrackerGroup for advanced workflows."""

        return self._group

    def start(self) -> None:
        """Start polling HID devices for pose data in a background thread."""

        if self._thread and self._thread.is_alive():
            return

        self._running.set()
        self._thread = threading.Thread(target=self._loop_forever, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Stop polling and join the background thread."""

        self._running.clear()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._thread = None

    def __enter__(self) -> "UltimateTrackerAPI":
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.stop()

    def add_pose_callback(self, callback: PoseCallback) -> None:
        """Register a callback that receives TrackerPose objects as they stream in."""

        if callback not in self._pose_callbacks:
            self._pose_callbacks.append(callback)

    def remove_pose_callback(self, callback: PoseCallback) -> None:
        self._pose_callbacks = [cb for cb in self._pose_callbacks if cb != callback]

    def get_latest_pose(self, tracker_index: int) -> Optional[TrackerPose]:
        """Return the most recent pose for the requested tracker index, if available."""

        with self._lock:
            return self._latest_pose.get(tracker_index)

    def iter_latest_poses(self) -> Iterable[TrackerPose]:
        """Iterate over the most recent poses for all trackers that have reported."""

        with self._lock:
            return list(self._latest_pose.values())

    def _loop_forever(self) -> None:
        while self._running.is_set():
            self._group.do_loop()
            if self._poll_interval:
                time.sleep(self._poll_interval)

    def _handle_pose_event(self, sample: Dict) -> None:
        pose = TrackerPose(
            tracker_index=sample["tracker_index"],
            mac=mac_str(sample["mac"]),
            buttons=sample["buttons"],
            tracking_status=sample["tracking_status"],
            timestamp_ms=sample["timestamp_ms"],
            position=sample["position"],
            rotation=sample["rotation"],
            acceleration=sample["acceleration"],
            angular_velocity=sample["angular_velocity"],
        )

        with self._lock:
            self._latest_pose[pose.tracker_index] = pose

        for callback in list(self._pose_callbacks):
            try:
                callback(pose)
            except Exception:  # pragma: no cover - defensive logging
                logger.exception("Pose callback raised an exception")