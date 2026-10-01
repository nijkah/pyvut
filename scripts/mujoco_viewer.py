#!/usr/bin/env python3
"""Live MuJoCo visualization of VIVE Ultimate Tracker poses.

Each tracker is a mocap body (puck + RGB axes: X red, Y green, Z blue) that follows
the live pose, with a fading trail of recent positions.

Keys (in the viewer window):
    Space  re-center: make the current position of tracker 0 the origin
    C      clear trails
    M      end (save) the host's map so it can be shared with clients
    0-4    restart SLAM on that dongle slot (fresh map); do it while holding the tracker
           up facing the room, keep it still ~5s, then move it slowly sideways
    K      calibrate (--all-hosts): press once, move all trackers together (rigidly, e.g. in
           one hand) through ~10 s of varied motion, press again. Each tracker's frame is
           fitted onto the reference tracker's path; axes are aligned at the end.
"""

import argparse
import time

import mujoco
import mujoco.viewer
import numpy as np

from pyvut import UltimateTrackerAPI
from pyvut.enums_horusd_status import (
    KEY_RECEIVED_HOST_MAP,
    KEY_TRANSMISSION_READY,
    map_status_to_str,
    pose_status_to_str,
)
from pyvut.tracker_core import set_tracker_core_verbose

MAX_TRACKERS = 5
POSE_OK = 2  # position + rotation valid; other states are rotation-only / frozen
# Firmware sets 0x10 on top of the state (seen while the tracker lies still), e.g. 18 = OK.
STATUS_MASK = 0x0F

COLORS = [
    (0.20, 0.55, 0.95),
    (0.95, 0.55, 0.15),
    (0.30, 0.80, 0.40),
    (0.85, 0.30, 0.75),
    (0.95, 0.85, 0.20),
]

# Tracker frame is Y-up; MuJoCo is Z-up. Rotating +90deg about X maps y->z, z->-y.
Q_Y_UP_TO_Z_UP = np.array([np.cos(np.pi / 4), np.sin(np.pi / 4), 0.0, 0.0])


def build_model_xml() -> str:
    bodies = []
    for i, (r, g, b) in enumerate(COLORS[:MAX_TRACKERS]):
        bodies.append(f"""
    <body name="tracker{i}" mocap="true" pos="{i * 0.3} 0 -10">
      <geom name="puck{i}" type="cylinder" size="0.04 0.012" rgba="{r} {g} {b} 1"/>
      <geom type="box" size="0.015 0.015 0.006" pos="0 0 -0.018" rgba="0.15 0.15 0.15 1"/>
      <geom type="capsule" fromto="0 0 0 0.1 0 0" size="0.004" rgba="1 0.2 0.2 1"/>
      <geom type="capsule" fromto="0 0 0 0 0.1 0" size="0.004" rgba="0.2 1 0.2 1"/>
      <geom type="capsule" fromto="0 0 0 0 0 0.1" size="0.004" rgba="0.2 0.4 1 1"/>
    </body>""")
    return f"""
<mujoco model="vive_ultimate_trackers">
  <visual>
    <headlight ambient="0.4 0.4 0.4" diffuse="0.6 0.6 0.6"/>
    <global offwidth="1280" offheight="720"/>
  </visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512"
             rgb1="0.22 0.24 0.27" rgb2="0.18 0.20 0.22" mark="edge" markrgb="0.35 0.38 0.42"/>
    <material name="grid" texture="grid" texrepeat="10 10" texuniform="true"/>
  </asset>
  <worldbody>
    <light pos="0 0 4" dir="0 0 -1" diffuse="0.6 0.6 0.6"/>
    <geom name="floor" type="plane" size="5 5 0.05" material="grid"/>
    <geom name="origin_x" type="capsule" fromto="0 0 0.001 0.3 0 0.001" size="0.006" rgba="1 0.2 0.2 0.6" contype="0" conaffinity="0"/>
    <geom name="origin_y" type="capsule" fromto="0 0 0.001 0 0.3 0.001" size="0.006" rgba="0.2 1 0.2 0.6" contype="0" conaffinity="0"/>
    {''.join(bodies)}
  </worldbody>
</mujoco>
"""


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def pose_matrix(pos, quat) -> np.ndarray:
    quat = np.asarray(quat, dtype=np.float64)
    quat = quat / max(np.linalg.norm(quat), 1e-9)
    rot = np.zeros(9)
    mujoco.mju_quat2Mat(rot, quat)
    m = np.eye(4)
    m[:3, :3] = rot.reshape(3, 3)
    m[:3, 3] = pos
    return m


def quat_to_rot(quat) -> np.ndarray:
    return pose_matrix(np.zeros(3), quat)[:3, :3]


def rot_to_quat(rot) -> np.ndarray:
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, np.asarray(rot, dtype=np.float64).flatten())
    return quat


def rot_angle_deg(rot) -> float:
    return float(np.degrees(np.arccos(np.clip((np.trace(rot) - 1) / 2, -1.0, 1.0))))


def kabsch(src, dst):
    """Rigid transform (R, t) minimizing |R @ src + t - dst| over paired points."""
    src, dst = np.asarray(src), np.asarray(dst)
    cs, cd = src.mean(0), dst.mean(0)
    u, _, vt = np.linalg.svd((src - cs).T @ (dst - cd))
    d = np.sign(np.linalg.det(vt.T @ u.T))
    rot = vt.T @ np.diag([1.0, 1.0, d]) @ u.T
    return rot, cd - rot @ cs


def apply_calibration(pos, quat, cal):
    """Express a pose from a tracker's own SLAM frame in the reference tracker's frame.

    Positions use the transform fitted from the joint motion; orientations use a separate
    offset so the axes match, independent of how the quaternion convention relates to the
    position frame."""
    return cal["R"] @ np.asarray(pos) + cal["t"], rot_to_quat(cal["Rq"] @ quat_to_rot(quat))


def finish_calibration(recording, poses_by_slot, min_samples=40, min_spread=0.15):
    ref_slot = recording["ref"]
    identity = {"R": np.eye(3), "t": np.zeros(3), "Rq": np.eye(3)}
    result = {ref_slot: identity}
    ref_pose = poses_by_slot.get(ref_slot)
    for slot, pairs in recording["samples"].items():
        if len(pairs) < min_samples:
            print(f"  slot {slot}: only {len(pairs)} samples, skipped")
            continue
        src = np.array([a for a, _ in pairs])
        dst = np.array([b for _, b in pairs])
        spread = float(np.linalg.norm(dst.std(0)))
        if spread < min_spread:
            print(f"  slot {slot}: motion spread {spread:.2f} m too small (need {min_spread} m), skipped")
            continue
        rot, t = kabsch(src, dst)
        rms = float(np.sqrt(np.mean(np.sum((src @ rot.T + t - dst) ** 2, axis=1))))
        pose = poses_by_slot.get(slot)
        if ref_pose is not None and pose is not None:
            rot_q = quat_to_rot(ref_pose.rotation) @ quat_to_rot(pose.rotation).T
        else:
            rot_q = rot
        result[slot] = {"R": rot, "t": t, "Rq": rot_q}
        print(f"  slot {slot}: {len(pairs)} samples, spread {spread:.2f} m, fit RMS {rms * 100:.1f} cm, "
              f"frame rotation {rot_angle_deg(rot):.1f} deg "
              f"(orientation-based estimate differs by {rot_angle_deg(rot.T @ rot_q):.1f} deg)")
    return result


def to_mujoco_frame(pos, quat, y_up: bool):
    pos = np.asarray(pos, dtype=np.float64)
    quat = np.asarray(quat, dtype=np.float64)
    norm = np.linalg.norm(quat)
    quat = quat / norm if norm > 1e-6 else np.array([1.0, 0.0, 0.0, 0.0])
    if y_up:
        pos = np.array([pos[0], -pos[2], pos[1]])
        quat = quat_mul(Q_Y_UP_TO_Z_UP, quat)
    return pos, quat


def draw_trails(scn, trails):
    scn.ngeom = 0
    identity = np.eye(3).flatten()
    for i, trail in enumerate(trails):
        r, g, b = COLORS[i]
        n = len(trail)
        for j, p in enumerate(trail):
            if scn.ngeom >= scn.maxgeom:
                return
            alpha = 0.1 + 0.8 * (j + 1) / n
            mujoco.mjv_initGeom(
                scn.geoms[scn.ngeom],
                type=mujoco.mjtGeom.mjGEOM_SPHERE,
                size=np.array([0.006, 0, 0]),
                pos=p,
                mat=identity,
                rgba=np.array([r, g, b, alpha], dtype=np.float32),
            )
            scn.ngeom += 1


def status_text(pose, map_state, slam, is_host) -> str:
    state = pose.tracking_status & STATUS_MASK
    try:
        state_str = pose_status_to_str(state)
    except IndexError:
        state_str = f"state {state}"
    try:
        map_str = map_status_to_str(map_state)
    except IndexError:
        map_str = f"map {map_state}"
    if state == POSE_OK:
        hint = "position OK"
    elif map_state == 9:  # MAP_REBUILD_WAIT_FOR_STATIC
        hint = "rotation only: set the tracker down still, then move it slowly"
    else:
        hint = "rotation only: move slowly, point cameras at textured areas"
    if not slam:  # only filled by the status polling of --legacy-init
        share = ""
        if is_host and state == POSE_OK and map_state == 3:
            hint += "; press M when the area is mapped"
    elif is_host:
        share = f"tx_ready={slam.get(KEY_TRANSMISSION_READY, '?')}"
        if state == POSE_OK and map_state == 3:  # MAP_REBUILT: built but not saved
            hint += "; press M when the area is mapped"
    else:
        share = f"got_host_map={slam.get(KEY_RECEIVED_HOST_MAP, '?')}"
    pos = ", ".join(f"{v:+.3f}" for v in pose.position)
    return f"{state_str} ({pose.tracking_status})  {map_str}  {share}\npos=({pos})  {hint}"


def mac_slot(mac: str) -> int:
    # Same slot derivation as tracker_core.mac_to_idx (second MAC byte, low nibble)
    return int(mac.split(":")[1], 16) & 0xF


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mode", choices=["DONGLE_USB", "TRACKER_USB"], default="DONGLE_USB")
    parser.add_argument("--wifi-info", dest="wifi_info", help="Optional path to a wifi_info.json file.")
    parser.add_argument("--height", type=float, default=1.0,
                        help="Vertical offset (m) added so the tracker origin sits above the floor.")
    parser.add_argument("--no-y-up", dest="y_up", action="store_false",
                        help="Treat tracker coordinates as already Z-up (skip axis conversion).")
    parser.add_argument("--trail", type=int, default=400, help="Number of trail points per tracker (0 disables).")
    parser.add_argument("--all-hosts", action="store_true",
                        help="Run every tracker as its own SLAM host; align their frames with K.")
    parser.add_argument("--ref-serial", default=None,
                        help="Serial suffix (e.g. 00239, shown in the panel) of the K calibration reference tracker; "
                             "default is the lowest slot. Slots change on re-pairing, serials don't.")
    parser.add_argument("--host-slot", type=int, default=None,
                        help="Dongle slot (0-4) of the tracker to use as SLAM host; default is the first tracker heard from.")
    parser.add_argument("--legacy-init", action="store_true",
                        help="Use pyvut's original tracker setup instead of the VIVE Streaming Hub sequence.")
    parser.add_argument("--verbose", action="store_true", help="Print raw tracker_core debug output.")
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_string(build_model_xml())
    data = mujoco.MjData(model)
    mocap_ids = [model.body_mocapid[model.body(f"tracker{i}").id] for i in range(MAX_TRACKERS)]
    puck_ids = [model.geom(f"puck{i}").id for i in range(MAX_TRACKERS)]

    origin = np.zeros(3)
    calibration = {}  # slot -> {"R", "t": position transform, "Rq": orientation offset} into the reference frame
    cal_recording = None  # while recording: {"ref": slot, "samples": {slot: [(pos, ref_pos)]}, "last": ref_pos}
    trails = [[] for _ in range(MAX_TRACKERS)]
    last_trail_pos = [None] * MAX_TRACKERS
    pending_keys = []

    def on_key(keycode: int) -> None:
        pending_keys.append(keycode)

    print("Connecting to trackers… move the tracker around to initialize SLAM. Close the window to exit.")
    with UltimateTrackerAPI(mode=args.mode, wifi_info_path=args.wifi_info, host_slot=args.host_slot,
                            all_hosts=args.all_hosts, legacy_init=args.legacy_init) as api:
        set_tracker_core_verbose(args.verbose)
        with mujoco.viewer.launch_passive(model, data, key_callback=on_key) as viewer:
            viewer.cam.lookat[:] = [0, 0, args.height]
            viewer.cam.distance = 3.0
            viewer.cam.elevation = -25
            last_status_print = 0.0

            while viewer.is_running():
                poses = {p.tracker_index: p for p in api.iter_latest_poses() if p.tracker_index < MAX_TRACKERS}

                while pending_keys:
                    key = pending_keys.pop()
                    if key == 32 and 0 in poses:  # Space
                        pos, _ = to_mujoco_frame(poses[0].position, poses[0].rotation, args.y_up)
                        origin = np.array([pos[0], pos[1], 0.0])
                        trails = [[] for _ in range(MAX_TRACKERS)]
                        last_trail_pos = [None] * MAX_TRACKERS
                        print(f"Re-centered origin to {origin.round(3)}")
                    elif ord("0") <= key <= ord("4"):
                        slot = key - ord("0")
                        if api.tracker_group.comms.restart_slam(slot):
                            calibration.pop(slot, None)
                            print(f"Restarted SLAM on slot {slot}: hold it still ~5s, then move it slowly sideways")
                        else:
                            print(f"No tracker in slot {slot}")
                    elif key == ord("K"):
                        by_slot = {mac_slot(p.mac): p for p in poses.values()}
                        if cal_recording is not None:
                            print(f"Calibration: fitting {len(cal_recording['samples'])} tracker(s) "
                                  f"onto reference slot {cal_recording['ref']}")
                            calibration = finish_calibration(cal_recording, by_slot)
                            cal_recording = None
                            trails = [[] for _ in range(MAX_TRACKERS)]
                            last_trail_pos = [None] * MAX_TRACKERS
                        else:
                            serials = api.tracker_group.tracker_serials
                            if args.ref_serial:
                                ref_slot = next((s for s, n in serials.items() if n.endswith(args.ref_serial)), None)
                            else:
                                ref_slot = min(by_slot) if by_slot else None
                            if ref_slot not in by_slot:
                                print(f"No reference tracker found ({args.ref_serial or 'lowest slot'})")
                            else:
                                calibration = {}
                                cal_recording = {"ref": ref_slot, "samples": {}, "last": None}
                                print(f"Calibration recording (reference slot {ref_slot}): move ALL trackers together "
                                      "for ~10 s — translate ~0.5 m in several directions, tilt/turn a bit — "
                                      "then press K again")
                    elif key == ord("M"):
                        host = api.tracker_group.comms.current_host_id
                        if host >= 0:
                            api.tracker_group.comms.lambda_end_map(host)
                            print(f"Sent END_MAP to host slot {host}")
                        else:
                            print("No SLAM host yet")
                    elif key == ord("C"):
                        trails = [[] for _ in range(MAX_TRACKERS)]
                        last_trail_pos = [None] * MAX_TRACKERS

                if cal_recording is not None:
                    by_slot = {mac_slot(p.mac): p for p in poses.values()
                               if (p.tracking_status & STATUS_MASK) == POSE_OK}
                    ref = by_slot.get(cal_recording["ref"])
                    if ref is not None:
                        ref_pos = np.asarray(ref.position, dtype=np.float64)
                        last = cal_recording["last"]
                        if last is None or np.linalg.norm(ref_pos - last) > 0.005:
                            cal_recording["last"] = ref_pos
                            for slot, p in by_slot.items():
                                if slot != cal_recording["ref"]:
                                    cal_recording["samples"].setdefault(slot, []).append(
                                        (np.asarray(p.position, dtype=np.float64), ref_pos))

                with viewer.lock():
                    for i, pose in poses.items():
                        raw_pos, raw_quat = pose.position, pose.rotation
                        if mac_slot(pose.mac) in calibration:
                            raw_pos, raw_quat = apply_calibration(raw_pos, raw_quat, calibration[mac_slot(pose.mac)])
                        pos, quat = to_mujoco_frame(raw_pos, raw_quat, args.y_up)
                        pos = pos - origin + np.array([0.0, 0.0, args.height])
                        tracking = (pose.tracking_status & STATUS_MASK) == POSE_OK
                        mid = mocap_ids[i]
                        # Rotation is always valid; position only when fully tracked.
                        data.mocap_quat[mid] = quat
                        if tracking or data.mocap_pos[mid][2] < -5:
                            data.mocap_pos[mid] = pos
                        r, g, b = COLORS[i]
                        model.geom_rgba[puck_ids[i]] = (r, g, b, 1.0) if tracking else (0.9, 0.2, 0.2, 0.6)

                        if tracking and args.trail > 0:
                            if last_trail_pos[i] is None or np.linalg.norm(pos - last_trail_pos[i]) > 0.01:
                                trails[i].append(pos.copy())
                                last_trail_pos[i] = pos
                                del trails[i][:-args.trail]

                    mujoco.mj_forward(model, data)
                    draw_trails(viewer.user_scn, trails)

                map_states = api.tracker_group.tracker_map_state
                comms = api.tracker_group.comms
                left = "\n\n".join(
                    f"Tracker {i} slot {mac_slot(p.mac)} "
                    f"#{api.tracker_group.tracker_serials.get(mac_slot(p.mac), '?')} "
                    f"{'HOST' if comms.is_host(mac_slot(p.mac)) else 'client'}"
                    f"{' cal' if mac_slot(p.mac) in calibration else ''}\n"
                    for i, p in sorted(poses.items())
                ) or "Waiting"
                right = "\n\n".join(
                    status_text(p, map_states[mac_slot(p.mac)],
                                api.tracker_group.slam_status.get(mac_slot(p.mac), {}),
                                comms.is_host(mac_slot(p.mac)))
                    for _, p in sorted(poses.items())
                ) or "no tracker poses yet"
                if cal_recording is not None:
                    counts = ", ".join(f"slot {k}: {len(v)}" for k, v in sorted(cal_recording["samples"].items()))
                    left = "CALIBRATING\n\n" + left
                    right = (f"move all trackers together, K to finish ({counts or 'no samples yet'})\n\n" + right)
                overlay = (mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT, left, right)
                viewer.set_texts(overlay)

                now = time.time()
                if now - last_status_print > 2.0 and poses:
                    last_status_print = now
                    slam = api.tracker_group.slam_status
                    print("  ".join(
                        f"[{i}] status={p.tracking_status} pos=({', '.join(f'{v:+.3f}' for v in p.position)})"
                        f" map={map_states[mac_slot(p.mac)]}"
                        # polled values exist only with --legacy-init
                        + ("" if not slam.get(mac_slot(p.mac)) else
                           f" tx_ready={slam[mac_slot(p.mac)].get(KEY_TRANSMISSION_READY, '?')}"
                           if comms.is_host(mac_slot(p.mac)) else
                           f" got_host_map={slam[mac_slot(p.mac)].get(KEY_RECEIVED_HOST_MAP, '?')}")
                        for i, p in sorted(poses.items())
                    ))

                viewer.sync()
                time.sleep(1 / 60)


if __name__ == "__main__":
    main()
