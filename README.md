# VIVE Ultimate Tracker Reverse Engineering

Practical notes for poking at HTC's VIVE Ultimate Trackers over USB HID and the wireless dongle. The repo contains low-level scripts, higher-level helpers, and a pygame visualizer for anyone exploring PCVR/SLAM behavior on Linux.

> **Acknowledgement:** This fork builds upon [`vive_ultimate_tracker_re`](https://github.com/shinyquagsire23/vive_ultimate_tracker_re) reverse-engineering work.

## Prerequisites

- Hardware: at least one VIVE Ultimate Tracker; optional wireless dongle (preferred for multi-tracker work) or direct USB connection. Host PC must be able to talk to HID devices and, for map resets, provide `adb` access.
- Software: Python 3.x plus the `hid`, `numpy`, and `pygame` packages (`pip install hid numpy pygame`).
- HID dependency note: this code imports `hid` (not `hidapi`). On Debian/Ubuntu install `libhidapi-hidraw0` and `libhidapi-libusb0` (e.g., `sudo apt install libhidapi-hidraw0 libhidapi-libusb0`) before `pip install hid` so the native bindings build.
- Permissions: Linux users may need udev rules so HID devices are accessible without `sudo`.
- Wi-Fi setup: edit `pyvut/wifi_info.json` with the SSID, password, country, and frequency you want the SLAM host tracker to broadcast.

## Quick Start Cheatsheet

### Direct USB tracker (`hid_test.py`)
1. Plug a tracker directly over USB.
2. `python hid_test.py` enumerates HID interface 0, configures camera policy/FPS, and requests PCVR power (`set_power_pcvr(1)`).
3. Uncomment the loop at the bottom to continuously parse incoming pose packets (`while True: parse_incoming(); kick_watchdog()`).

### Wireless dongle (`rf_hid_test.py`)
1. Plug the wireless dongle into the host PC.
2. `python rf_hid_test.py` enumerates the dongle, issues safe queries (fusion mode, role ID, IDs/SN, ROM version, capability dumps), and prints responses.
3. Example RF control and pairing helpers live near the bottom (e.g., `send_rf_command(0x1D, ...)`). Avoid flash/write opcodes unless you are prepared for risky behavior.

### pyvut package (`pyvut/`)
1. Update Wi-Fi credentials in `pyvut/wifi_info.json`.
2. Pair trackers with the dongle; the pyvut helper auto-selects the first tracker as SLAM host.
3. `python pyvut/tracker_core.py` streams ACK/status traffic, auto-sends SLAM role and Wi-Fi ACKs, and keeps pose state for up to five trackers.
4. `python scripts/visualize_pygame.py` renders simple 3D markers for live position/orientation; close the window to exit.

## Install as a Package

The repository now ships as a standard Python package. Install it into a virtual environment (editable mode recommended while hacking):

```bash
python -m venv .venv
source .venv/bin/activate
pip install -e .[visualizer]
```

The core dependencies are `hid` and `numpy`; the optional `visualizer` extra pulls in `pygame` for `scripts/visualize_pygame.py`.

## Python API for 6DoF Poses

Import `pyvut.UltimateTrackerAPI` to receive tracker poses (position + quaternion rotation + acceleration) through callbacks or polling. The API reuses the existing HID plumbing and exposes each pose as a `TrackerPose` dataclass.

```python
import time
from pyvut import TrackerPose, UltimateTrackerAPI


def on_pose(pose: TrackerPose) -> None:
	print(
		f"Tracker {pose.tracker_index} @ {pose.mac}: "
		f"pos={pose.position} rot={pose.rotation} tracking={pose.tracking_status}"
	)


with UltimateTrackerAPI(mode="DONGLE_USB") as api:
	api.add_pose_callback(on_pose)
	while True:
		time.sleep(1)
```

- `UltimateTrackerAPI` supports both dongle (`mode="DONGLE_USB"`) and direct USB tracker (`mode="TRACKER_USB"`) paths.
- Use `api.get_latest_pose(idx)` to fetch the current pose for a tracker without registering callbacks.
- Provide a custom Wi-Fi config via `UltimateTrackerAPI(..., wifi_info_path="/path/to/wifi_info.json")` if you do not want to edit the packaged default.
- Prefer a quick CLI demo? Run `python scripts/stream_poses.py --mode DONGLE_USB` to print live pose samples.

## Repository Map

- `hid_test.py`: minimal direct-USB HID pokes; bring up PCVR mode, send haptics, parse pose packets.
- `rf_hid_test.py`: experimental HID access to the wireless dongle; includes many RF/dongle command IDs plus hardware queries.
- `pyvut/`: higher-level helpers and enums for HID/RF communication.
	- `tracker_core.py`: `DongleHID`, `TrackerHID`, and `ViveTrackerGroup` abstractions for pairing, SLAM role assignment, Wi-Fi ACKs, and pose tracking.
	- `clear_maps.sh`: adb helper to wipe `/data/lambda` and `/data/mapdata` on a tracker.
	- `enums_*.py`: command and response constants (USB HID, RF, Wi-Fi, status, ACKs, dongle commands).
- `scripts/visualize_pygame.py`: lightweight pygame visualizer for up to 5 trackers.
- `scripts/stream_poses.py`: CLI demo that prints pose samples via `UltimateTrackerAPI`.
- `ota_parse.py`: extracts and CRC-checks partitions from HTC OTA firmware images (`trackers/firmware/TX_FW.ota`).
- `how-to-use.md`: this README distilled version; keep it handy for detailed background.

## Script Highlights

### Direct USB helpers (`hid_test.py`)
- `set_tracking_mode(mode)`: toggle between gyro and SLAM modes (IDs defined in `pyvut/enums_horusd_status.py`).
- `send_haptic(...)`: trigger haptics on the connected tracker.
- `parse_pose_data(...)`: decode position, rotation, and acceleration from raw HID packets.

### Wireless dongle experiments (`rf_hid_test.py`)
- Enumerates dongle HID endpoints, queries role IDs, SNs, ROM versions, and capability bitfields.
- Includes example pairing (`send_rf_command(0x1D, ...)`) and control commands for trackers.
- `fuzz_blacklist` and inline comments flag dangerous opcodes—respect them.

### pyvut helpers (`pyvut/`)
- `DongleHID` pairs trackers, assigns SLAM host/role, forwards ACKs, and maintains Wi-Fi credentials.
- `TrackerHID` offers a direct USB transport option (`ViveTrackerGroup(mode="TRACKER_USB")`).
- `ViveTrackerGroup` keeps arrays of poses and exposes `get_pos(idx)`/`get_rot(idx)` for consumers.
- SLAM/map helpers react to ACKs (`ACK_MAP_STATUS`, `ACK_LAMBDA_STATUS`, etc.) by requesting or ending maps when trackers get stuck.

## OTA Parsing (`ota_parse.py`)

1. Place `trackers/firmware/TX_FW.ota` in the repo.
2. Run `python ota_parse.py`.
3. The script lists OTA segments, validates HTC's CRC-128, and dumps each segment as `seg_<index>_<mem_addr>.bin` for further inspection.

## Tips, Caveats, and Safety

- HID interface numbers are currently hard-coded (interface 0 for HID1); if enumeration fails, inspect `hid.enumerate(...)` output for the right path.
- Many command IDs were inferred from binaries or experimentation; logging dumps raw payloads to help future reverse engineering.
- Several dongle commands reboot or brick hardware. Review `DCMDS_THAT_RESTART` / `DCMDS_THAT_WRITE_FLASH` in `pyvut/enums_horusd_dongle.py` before experimenting.
- Scripts often run tight loops without throttling; sprinkle `time.sleep(...)` if USB polling needs to be kinder to your system.
- `clear_maps.sh` wipes tracker-side SLAM data via `adb shell`; handy when map state gets wedged.