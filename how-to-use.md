# How to use this repo

Practical notes for working with the VIVE Ultimate Tracker reverse‑engineering tools in this repository.

## Prerequisites
- Hardware: VIVE Ultimate Tracker(s); optional wireless dongle (preferred), or direct USB connection to a tracker. A host PC that can talk to the trackers over USB; adb access if you need to wipe tracker-side map data.
- Software: Python 3.x with `hid` and `numpy`; `pygame` is only needed for the simple visualizer. Install with `pip install hid numpy pygame`.
- USB permissions: on Linux you may need appropriate udev rules to access HID devices without sudo.
- Wi‑Fi info: edit `pyvut/wifi_info.json` with the SSID/password/country/frequency you want the host tracker to broadcast for SLAM clients.

## Quick file map
- `hid_test.py`: minimal direct-USB HID pokes against a tracker; useful for bringing up PCVR mode and dumping raw pose packets.
- `rf_hid_test.py`: experimental HID access to the wireless dongle; includes many RF/dongle command IDs and queries hardware IDs.
- `pyvut/`: higher-level helpers and enums for HID/RF communication plus a pygame visualizer.
  - `tracker_core.py`: classes for dongle HID (`DongleHID`), direct tracker HID (`TrackerHID`), and a group wrapper (`ViveTrackerGroup`).
  - `clear_maps.sh`: adb helper to wipe map data on trackers.
  - `enums_*.py`: command/response constants split by domain (USB HID, RF, Wi‑Fi, status, ACKs, dongle commands).
- `scripts/visualize_pygame.py`: draws simple 3D markers for up to 5 trackers using live pose data.
- `scripts/stream_poses.py`: console script that prints pose updates via the `UltimateTrackerAPI`.
- `ota_parse.py`: extracts and CRC-checks partitions from HTC OTA firmware images (expects `trackers/firmware/TX_FW.ota`).

## Direct USB testing (`hid_test.py`)
1) Plug a tracker directly over USB.  
2) Run `python hid_test.py`. It enumerates HID interface 0, sets camera policy/FPS, and requests PCVR power (`set_power_pcvr(1)`).  
3) To watch incoming pose packets, uncomment the loop at the bottom (`while True: parse_incoming(); kick_watchdog()`).

Handy helpers in this file:
- `set_tracking_mode(mode)`: switch between gyro/SLAM modes (IDs in `pyvut/enums_horusd_status.py`).
- `send_haptic(...)`: trigger haptics on the USB-connected tracker.
- `parse_pose_data(...)`: prints decoded position/rotation/acceleration from raw packets.

## Wireless dongle experiments (`rf_hid_test.py`)
1) Plug in the wireless dongle.  
2) Run `python rf_hid_test.py` to enumerate it and send a handful of safe queries (fusion mode, role ID, IDs/SN, ROM version, capability queries).  
3) The bottom of the script shows example calls for pairing (`send_rf_command(0x1D, ...)`) and tracker control commands.

⚠️ Safety: several dongle commands can reboot or brick hardware (see comments such as “BRICKED MY DONGLE :(” and the `fuzz_blacklist` list). Avoid experimenting with `DCMD_21`/`0x21` or other flash/write commands unless you know what you are doing.

## pyvut helpers (`pyvut`)
Core flow lives in `pyvut/tracker_core.py`:
- `DongleHID` pairs trackers, assigns one as SLAM host, sends ACKs/Wi‑Fi credentials, and parses pose + ACK traffic from up to 5 trackers.
- `TrackerHID` talks directly to a single tracker over USB.
- `ViveTrackerGroup` glues either transport together, maintains pose state arrays, and exposes `get_pos(idx)` / `get_rot(idx)` for consumers.

Using the helpers with the dongle:
1) Update `pyvut/wifi_info.json` with your Wi‑Fi details (host SSID/password/country/frequency).  
2) Pair trackers with the dongle (the script listens for pair events and picks the first tracker as SLAM host).  
3) Run `python pyvut/tracker_core.py` to print live ACK/status traffic; it will auto-send SLAM role/host/Wi‑Fi ACKs.  
4) For a quick visualization, run `python scripts/visualize_pygame.py`. Cubes are drawn for up to 5 trackers; orientation/position update in real time. Close the window to exit.

Using the helpers over direct USB instead of the dongle:
- In `tracker_core.py`, change the constructor to `ViveTrackerGroup(mode="TRACKER_USB")` (see the `__main__` block) to bind to a directly connected tracker.

Map/SLAM helpers:
- `clear_maps.sh` wipes `/data/lambda` and `/data/mapdata` via adb on a tracker (useful when SLAM gets stuck).
- The toybox will query and react to map state/pose state via ACKs (`ACK_MAP_STATUS`, `ACK_LAMBDA_STATUS`, etc.), auto-requesting maps or ending maps when stuck.

## OTA parsing (`ota_parse.py`)
Place `TX_FW.ota` under `trackers/firmware/` and run `python ota_parse.py`. The script will:
- Read the OTA header, list segments, validate HTC’s CRC-128, and dump each segment to `seg_<index>_<mem_addr>.bin` for further analysis.

## Tips and caveats
- HID interface numbers are hard-coded (interface 0 for HID1). If enumeration fails, check `hid.enumerate(...)` output for correct paths.
- Many command IDs/constants are still experimental guesses pulled from binaries; log output often prints raw dumps to help reverse engineer further.
- When extending, keep dangerous dongle commands in mind: `DCMDS_THAT_RESTART` and `DCMDS_THAT_WRITE_FLASH` in `enums_horusd_dongle.py` highlight risky operations.
- Most scripts run endless loops without throttling; add sleeps if you need to reduce USB polling load.
