#!/usr/bin/env python3
"""Keep the dongle's pairing window open and report pair events / slot states.

The pair request takes one flag per dongle slot, and flagging a slot resets it (dropping
any tracker connected there). So this helper waits for the dongle's slot report, then
periodically opens pairing only on requested slots that are NOT connected, so a tracker
put into pairing mode (fast-flashing blue) can pair without kicking the others.

Only uses RF_BEHAVIOR_PAIR_DEVICE; no reset / clear-pairing / flash commands.
"""

import argparse
import struct
import time

from pyvut.enums_horusd_dongle import DCMD_REQUEST_RF_CHANGE_BEHAVIOR, RF_BEHAVIOR_PAIR_DEVICE
from pyvut.tracker_core import DongleHID, mac_str, set_tracker_core_verbose


def slot_connected(state: int) -> bool:
    # Observed slot states: 0x32000008 = tracker connected, 0x320fa008 = not connected
    # (reported for empty slots too), 0x4000003 / 0x1000003 = unpaired. The 0x8 bit
    # (PAIR_STATE_PAIRED) is set in both of the first two, so it can't tell them apart.
    return state & 0xFF == 0x08 and state & 0x000FFF00 == 0


def open_pairing(dongle: DongleHID, slots) -> None:
    flags = [1 if i in slots else 0 for i in range(5)]
    dongle.send_cmd(DCMD_REQUEST_RF_CHANGE_BEHAVIOR, struct.pack("<BBBBBBB", RF_BEHAVIOR_PAIR_DEVICE, 1, *flags))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--slots", default="0,1,2,3,4",
                        help="Comma-separated dongle slots that may be opened for pairing; connected slots are always skipped.")
    parser.add_argument("--interval", type=float, default=20.0, help="Seconds between re-sending the pair request.")
    parser.add_argument("--duration", type=float, default=300.0, help="Stop after this many seconds.")
    parser.add_argument("--verbose", action="store_true", help="Print raw tracker_core debug output.")
    args = parser.parse_args()
    slots = {int(s) for s in args.slots.split(",") if s.strip()}

    set_tracker_core_verbose(args.verbose)
    dongle = DongleHID()
    print(f"Waiting for the dongle's slot report, then opening pairing for {args.duration:.0f}s. "
          "Hold a tracker's power button ~2s until it flashes blue quickly.")

    start = time.time()
    last_open = 0.0
    seen_macs = set()
    last_states = None

    while time.time() - start < args.duration:
        dongle.do_loop()

        for mac in dongle.device_macs:
            if bytes(mac) not in seen_macs:
                seen_macs.add(bytes(mac))
                print(f"PAIR EVENT: {mac_str(mac)}")

        states = tuple(dongle.pair_state)
        if states != last_states:
            last_states = states
            desc = "  ".join(
                f"[{i}] {s:#010x}{' CONNECTED' if slot_connected(s) else ''}" for i, s in enumerate(states)
            )
            print(f"{time.time() - start:6.1f}s slots: {desc}")

        # pair_state stays all-zero until the first status report arrives
        if any(dongle.pair_state) and time.time() - last_open > args.interval:
            free = {i for i in slots if not slot_connected(dongle.pair_state[i])}
            if free:
                open_pairing(dongle, free)
                print(f"{time.time() - start:6.1f}s opened pairing on slots {sorted(free)} "
                      "(a paired tracker that is switched off loses its slot; limit with --slots)")
            last_open = time.time()

    print(f"Done. Trackers that sent a pair event: {[mac_str(m) for m in seen_macs] or 'none'}")


if __name__ == "__main__":
    main()
