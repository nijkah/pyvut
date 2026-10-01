# 6DoF tracking with the VIVE Ultimate Tracker on Ubuntu without an HMD

This guide shows how to capture full 6DoF poses from HTC's VIVE Ultimate Trackers on Ubuntu using USB HID or the wireless dongle without owning the Vive HMD (Head Mounted Display).

**Acknowledgement:** This fork builds upon [`vive_ultimate_tracker_re`](https://github.com/shinyquagsire23/vive_ultimate_tracker_re) reverse-engineering work.

## Prerequisites

- **Hardware:** at least one VIVE Ultimate Tracker; optional wireless dongle (preferred for multi-tracker work) or direct USB connection. Host PC must be able to talk to HID devices and, for map resets, provide `adb` access.

### Constructing map (SteamVR / VIVE) - only available on Windows

Do the initial SLAM map construction on Windows with SteamVR + VIVE Streaming Hub; after the tracker stores the map, you can return to Ubuntu for everything else in this guide.

1. Install SteamVR.
2. Enable the SteamVR "null driver" (virtual headset) following [SteamVRNoHeadset](https://github.com/username223/SteamVRNoHeadset).
3. Install the VIVE Streaming Hub from HTC's site.
4. Follow the VIVE Streaming Hub instructions to create a new map; you can ignore the final step that asks for a SteamVR headset connection.
5. Once your trackers indicate they are "ready" in Streaming Hub, verify any additional requirements from the sections below (Wi-Fi config, dongle pairing, etc.).

### Software & configuration

- Python 3.x plus the `hid`, `numpy`, and `pygame` packages (`pip install hid numpy pygame`).
- HID dependency note: this code imports `hid` (not `hidapi`). On Debian/Ubuntu install `libhidapi-hidraw0` and `libhidapi-libusb0` (e.g., `sudo apt install libhidapi-hidraw0 libhidapi-libusb0`) before `pip install hid` so the native bindings build.
- Permissions: Ubuntu users may need udev rules so HID devices are accessible without `sudo`; for quick tests you can run `sudo chmod a+r /dev/hidraw${NUM}` on the device path printed by `hid_test.py` or `rf_hid_test.py`.
- Wi-Fi setup: set your country code in `pyvut/wifi_info.json`. The SLAM host tracker creates its own AP (`DIRECT-xx-<serial>`) and pyvut forwards its credentials to the clients; the SSID/password fields are not used.

## Getting started with the wireless dongle

This is the path that has been tested end to end: three trackers on one dongle, one
shared map, all trackers in one coordinate frame (tracker firmware 1.0.999.178, dongle
ROM 0909.0000.0000.0917).

1. **Build the map on Windows** as described above. Pair the trackers to the same dongle
   you will use on Linux; they stay paired, so there is nothing to pair on Linux.
   The tracker that was host during mapping stores the map; note its serial (shown in
   Streaming Hub, and on Linux as the end of its AP name `DIRECT-xx-<serial>`).
2. **Install** pyvut with the viewer extra (`pip install -e .[viewer]`) and give your
   user access to the dongle, e.g. with a udev rule
   (`/etc/udev/rules.d/70-vive-tracker.rules`):
   ```
   SUBSYSTEM=="hidraw", ATTRS{idVendor}=="0bb4", TAG+="uaccess"
   ```
   then `sudo udevadm control --reload-rules && sudo udevadm trigger` and replug.
3. **Set your Wi-Fi country** in `pyvut/wifi_info.json` (`"country": "DE"`, `"US"`, ...).
   Only the country is used; the host tracker creates its own AP and pyvut passes its
   credentials to the clients.
4. **Switch on the host tracker first** (the one holding the map), then the others. The
   first tracker pyvut hears from becomes the SLAM host (or pick a slot with
   `--host-slot`).
5. **Run** `python scripts/mujoco_viewer.py`. Hold each tracker at chest height facing the
   mapped area, keep it still for a few seconds, then move it slowly. Lying on a desk it
   will not relocalize.
   - Host: `MAP_STATUS -1,1` (map exists) -> `0,6` (`MAP_REUSE_OK`) -> tracking.
   - Clients: `MAP_STATUS 0,1` (got the host map) -> `0,6` -> tracking in the host's frame.
6. **Pairing new trackers**: `python scripts/pair_trackers.py`, then hold a tracker's
   power button until it flashes blue quickly. pyvut no longer opens pairing on startup,
   because the pair request resets the flagged dongle slots.

Tracking status in pose packets: `2` = position + rotation, `3` = rotation only,
`4` = recently lost (position frozen). `0x10` is set on top while the tracker lies still
(e.g. `18` = OK). `scripts/identify_trackers.py` shows which slot a physical tracker is in.

### How pyvut sets the trackers up

By default pyvut sends the same setup as VIVE Streaming Hub, decoded from USB captures
of Streaming Hub talking to the dongle. After a tracker's device info dump ends (`NAZZ`):

| | ACKs |
|---|---|
| every tracker | `ANI<slot>` `ATW` `ACF60` `ATS<unix time>` `Wc<country>` `ACF60` `pAS_TS0` `FW2` |
| host | `ATM21`; if it hasn't reported its AP yet, `FW2` + `WH` after its next `LP` |
| client | `ATM11`; on its `WS` request: `Wc` `Wf<freq>` `Ws<ssid>` `Wt<rest>` `Wp<pass>` `FW1` |

Streaming Hub's map wizard sends `AHT1,1,1,1`, `FW2`, `Wc..`, `ATM20`, then `pMS_SC`,
`pMS_ILA`, `pMS_IHA`, `pMS_ES` and finally `ALE` (end map) to the host. pyvut doesn't
drive the wizard; build the map in Streaming Hub.

Options (`UltimateTrackerAPI` arguments / script flags):
- `host_slot` / `--host-slot N`: make the tracker in dongle slot N the host.
- `all_hosts` / `--all-hosts`: every tracker is its own host with its own map and frame;
  the viewer's K key fits the frames onto one reference tracker.
- `legacy_init` / `--legacy-init`: pyvut's earlier setup (`ARI`, `ATM-1`, `ATH`, `AWH`,
  `ATM20`, status polling); with it clients never received the host map.

Known open points: the host drops to status 4 more often than the clients; restarting
SLAM on a slot (viewer keys 0-4) with the default setup is untested; meanings of `FW`,
`pAS_TS0`, `AHT` and the host's `p83`/`p94` messages are guesses or unknown.

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
1. Set your Wi-Fi country in `pyvut/wifi_info.json`.
2. Pair trackers with the dongle (`scripts/pair_trackers.py`); the first tracker pyvut hears from becomes the SLAM host.
3. `python -m pyvut.tracker_core` streams ACK/status traffic, sends the tracker setup and Wi-Fi ACKs, and keeps pose state for up to five trackers.
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
- `scripts/mujoco_viewer.py`: MuJoCo 3D viewer with trails and a per-tracker status panel (`pip install -e .[viewer]`).
- `scripts/pair_trackers.py`: open pairing only on dongle slots without a connected tracker.
- `scripts/identify_trackers.py`: rotate a tracker to see which dongle slot it is in.
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
- SLAM/map state from the trackers' `MS`/`LS` ACKs is tracked per slot (`tracker_map_state`); with `legacy_init=True` pyvut also polls SLAM status and nudges stuck clients.

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
