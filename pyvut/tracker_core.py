import hid
import struct
import numpy as np
import time
import json
import logging
from builtins import print as _builtin_print
from pathlib import Path

from .enums_usb import *
from .enums_horusd_hid import *
from .enums_horusd_wifi import * # currently unused
from .enums_horusd_misc import * # currently unused
from .enums_horusd_status import *
from .enums_horusd_rf_report import *
from .enums_horusd_ack import *
from .enums_horusd_dongle import *


logger = logging.getLogger(__name__)
_VERBOSE_PRINTS = True


def set_tracker_core_verbose(enabled: bool) -> None:
    """Globally enable/disable verbose logging for tracker_core."""

    global _VERBOSE_PRINTS
    _VERBOSE_PRINTS = enabled


def verbose_print(*args, **kwargs):
    if _VERBOSE_PRINTS:
        _builtin_print(*args, **kwargs)
        return

    sep = kwargs.get("sep", " ")
    end = kwargs.get("end", "\n")
    try:
        message = sep.join(str(arg) for arg in args) + end.rstrip("\n")
    except Exception:
        message = "<unprintable message>"
    logger.debug(message)

# Sometimes it will not track w/o:
# adb shell setprop persist.lambda.trans_setup 1
# adb shell setprop persist.lambda.normalmode 0 
# adb shell setprop persist.lambda.3rdhost 1 
# tracking_mode 1 seems to fuck up trans_setup?

# client trackers sometimes need:
# adb shell rm -rf /data/lambda/ 
# adb shell mkdir -p /data/lambda/
# adb shell rm /data/mapdata/\*

# adb shell cat /data/tracking_log/horusd.log.0
# adb shell cat /data/tracking_log/slam.log.1
# adb shell cat /data/tracking_log/slam.log.\* > test_slam_client.log 
# adb shell cat /data/tracking_log/slam.log.\* > test_slam_host.log 

# Doesn't work :(
# cp /data/property/persist.lambda.3rdhost /data/property/persist.tracking.mode.largescale
# rm /data/property/persist.tracking.mode.largescale

#Unpaired 23:31:fd:c2:7b:cd, 0x1
#Unpaired 23:30:42:b7:82:d3, 0x1
# Unpaired 23:32:fd:c2:7b:cd, 0x1

# Last AP a SLAM host reported (WIFI_HOST_SSID). With keep_map the host keeps running
# that AP but doesn't report it again, so clients get these cached credentials.
HOST_AP_CACHE = Path.home() / ".cache" / "pyvut" / "last_host_ap.json"

def load_host_ap_cache():
    try:
        with HOST_AP_CACHE.open("r", encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None

def save_host_ap_cache(ssid, passwd, freq):
    try:
        HOST_AP_CACHE.parent.mkdir(parents=True, exist_ok=True)
        with HOST_AP_CACHE.open("w", encoding="utf-8") as f:
            json.dump({"ssid": ssid, "pass": passwd, "freq": freq}, f)
    except OSError as exc:
        verbose_print(f"Couldn't save host AP cache: {exc}")

def current_milli_time():
    return round(time.time() * 1000)

def hex_dump(b, prefix=""):
    p = prefix
    b = bytes(b)
    for i in range(0, len(b)):
        if i != 0 and i % 16 == 0:
            verbose_print(p)
            p = prefix
        p += ("%02x " % b[i])
    verbose_print(p)

def mac_str(b):
    return hex(b[0])[2:] + ":" + hex(b[1])[2:] + ":" + hex(b[2])[2:] + ":" + hex(b[3])[2:] + ":" + hex(b[4])[2:] + ":" + hex(b[5])[2:] 

def mac_to_idx(b):
    if type(b) is int:
        return b
    return b[1] & 0xF

def do_u8_checksum(data):
    out = 0
    for i in range(0, len(data)):
        out ^= data[i]
    return out

# Base class which implements ACK commands
class Ackable(object):

    def lambda_end_map(self, device_addr):
        self.send_ack_to(mac_to_idx(device_addr), ACK_END_MAP)

    def send_ack_to(self, idx, ack):
        verbose_print("UNIMPLEMENTED")

    def send_ack_to_all(self, ack):
        for i in range(0, 5):
            self.send_ack_to(i, ack)

    def wifi_connect(self, idx):
        self.send_ack_to(idx, ACK_WIFI_CONNECT)

    #def wifi_set_ssid_password(self, idx, ssid, passwd):
    #    self.send_ack_to(idx, f"{ACK_WIFI_SSID_PASS}{ssid},{passwd}")

    def wifi_set_ssid(self, idx, ssid):
        verbose_print(ssid[:8])
        self._wifi_set_ssid_full(idx, ssid[:8])
        for i in range(8, len(ssid), 8):
            verbose_print(ssid[i:i+8])
            self._wifi_set_ssid_append(idx, ssid[i:i+8])

    def _wifi_set_ssid_full(self, idx, ssid):
        self.send_ack_to(idx, ACK_WIFI_SSID_FULL + ssid)

    def _wifi_set_ssid_append(self, idx, ssid):
        self.send_ack_to(idx, ACK_WIFI_SSID_APPEND + ssid)

    def wifi_set_country(self, idx, country):
        self.send_ack_to(idx, ACK_WIFI_COUNTRY + country)

    def wifi_set_password(self, idx, passwd):
        self.send_ack_to(idx, ACK_WIFI_PW + passwd)

    def wifi_set_freq(self, idx, freq):
        self.send_ack_to(idx, f"{ACK_WIFI_FREQ}{freq}")

    def ack_set_role_id(self, idx, rid):
        self.send_ack_to(idx, f"{ACK_ROLE_ID}{rid}")

    def ack_set_tracking_mode(self, idx, mode):
        self.send_ack_to(idx, f"{ACK_TRACKING_MODE}{mode}")

    def ack_set_tracking_host(self, idx, val):
        self.send_ack_to(idx, f"{ACK_TRACKING_HOST}{val}")

    def ack_set_wifi_host(self, idx, val):
        self.send_ack_to(idx, f"{ACK_WIFI_HOST}{val}")

    def ack_set_new_id(self, idx, val):
        self.send_ack_to(idx, f"{ACK_NEW_ID}{val}")

    def ack_lambda_ask_status(self, idx, key_id):
        self.send_ack_to(idx, f"{ACK_LAMBDA_ASK_STATUS}{key_id}")

    def ack_lambda_property(self, idx):
        self.send_ack_to(idx, ACK_LAMBDA_PROPERTY)

class DongleHID(Ackable):

    def __init__(self, wifi_info_path=None, open_pairing=False, host_slot=None, all_hosts=False, keep_map=False, host_mode_delay=0.0):
        self.calib_1 = ""
        self.calib_2 = ""
        self.device_macs = []
        self.got_a_pair = False
        self.num_paired = 0
        self.device_hid1 = None
        self.current_host_id = -1
        # Dongle slot to make the SLAM host; None = first tracker we hear from
        self.host_slot = host_slot
        # Every tracker runs as its own SLAM host (independent maps/frames, no map sharing)
        self.all_hosts = all_hosts
        self.host_slots = set()
        # Skip the tracking-mode -1 reset for hosts, so they can reuse a stored map
        # (e.g. one built in VIVE Streaming Hub) instead of starting a fresh one
        self.keep_map = keep_map
        # Seconds to wait between the tracking mode reset and the host mode, instead of
        # sending them back to back; slot -> time the host mode is due
        self.host_mode_delay = host_mode_delay
        self.pending_host_mode = {}
        self.tick_periodic = 0
        # MACs we've sent the role/tracking-mode setup ACKs to
        self.initialized_macs = set()
        # slot -> device ID sent via ACK_NEW_ID; host is 0, clients must be unique and != 0
        self.device_ids = {}
        # client slots that asked for Wi-Fi credentials before the host reported them
        self.wifi_pending = set()

        self.pose_callback = None
        self.ack_callback = None
        self.connected_callback = None
        self.disconnected_callback = None

        self.pair_state = [0,0,0,0,0]
        self.connected_to_host = [False]*5
        self.has_host_map = [False]*5

        self.last_host_map_ask_ms = current_milli_time()

        self.host_ssid = ""
        self.host_passwd = ""
        self.host_freq = ""
        if keep_map:
            cached_ap = load_host_ap_cache()
            if cached_ap:
                verbose_print(f"Using cached host AP {cached_ap['ssid']} for clients")
                self.host_ssid = cached_ap["ssid"]
                self.host_passwd = cached_ap["pass"]
                self.host_freq = cached_ap["freq"]

        device_list = hid.enumerate(VID_VIVE, PID_DONGLE)
        verbose_print(device_list)

        # TODO better error handling
        device_dict_hid1 = [device for device in device_list if device['interface_number'] == HID1_INTERFACE_NUM][0]
        self.device_hid1 = hid.Device(path=device_dict_hid1['path'])

        verbose_print(self.get_PCBID()) # RequestPCBID
        verbose_print(self.get_SKUID()) # RequestSKUID
        verbose_print(self.get_SN()) # RequestSN
        verbose_print(self.get_ShipSN()) # RequestShipSN
        verbose_print(self.get_CapFPC()) # RequestCapFPC
        verbose_print(self.get_ROMVersion()) # QueryROMVersion

        #verbose_print(self.send_cmd(0xEF, bytes([0x06]) + "800001FF".encode("utf-8"))) # Write PCB ID
        #verbose_print(self.send_cmd(0xEF, bytes([0x07]) + "00059E00".encode("utf-8"))) # Write SKU ID
        #verbose_print(self.send_cmd(0xEF, bytes([0x08]) + "43AD339B07725".encode("utf-8"))) # Write SN
        #verbose_print(self.send_cmd(0xEF, bytes([0x09]) + "FA3983C06815".encode("utf-8"))) # Write Ship SN

        # hex_dump(self.send_cmd(0x21, [0])) # bricked my dongle :(

        # 0x1D = ReportRequestRFChangeBehavior?
        # The 5 trailing bytes are per-slot flags. Opening pairing on a slot resets it and
        # drops the tracker paired there, so this is opt-in (scripts/pair_trackers.py pairs
        # only unconnected slots). Already-paired trackers reconnect without it.
        bEnabled = 1
        if open_pairing:
            verbose_print(self.send_cmd(DCMD_REQUEST_RF_CHANGE_BEHAVIOR, struct.pack("<BBBBBBB", RF_BEHAVIOR_PAIR_DEVICE, bEnabled, 1, 1, 1, 0, 0))) # PairDevice
        #verbose_print(self.send_cmd(DCMD_REQUEST_RF_CHANGE_BEHAVIOR, struct.pack("<BBBBBBB", RF_BEHAVIOR_RX_POWER_SAVING, bEnabled, 1, 0, 0, 0, 0))) # RxPowerSaving?
        #verbose_print(self.send_cmd(DCMD_REQUEST_RF_CHANGE_BEHAVIOR, struct.pack("<BBBBBB", RF_BEHAVIOR_RESTART_RF, 1, 0, 0, 0, 0))) # Same as 1 w/ bEnabled
        # 3 = SetLpf (7,8,9,10), not available
        # what is 4?
        #verbose_print(self.send_cmd(DCMD_REQUEST_RF_CHANGE_BEHAVIOR, struct.pack("<B", RF_BEHAVIOR_FACTORY_RESET))) # FactoryReset?
        #verbose_print(self.send_cmd(DCMD_REQUEST_RF_CHANGE_BEHAVIOR, struct.pack("<BBBBBB", RF_BEHAVIOR_6, 1, 0, 0, 0, 0))) # ClearPairingInfo?

        wifi_path = Path(wifi_info_path) if wifi_info_path else Path(__file__).with_name('wifi_info.json')
        with wifi_path.open('r', encoding='utf-8') as f:
            self.wifi_info = json.load(f)

    def parse_response(self, data):
        err_ret, cmd_id, data_len, unk2 = struct.unpack("<BBBH", data[:5])
        #verbose_print(f"unk: {hex(unk)} cmd_id: {hex(cmd_id)} data_len: {hex(data_len)} unk2: {hex(unk2)} ")

        ret = data[5:5+data_len-4]
        return err_ret, cmd_id, ret

    def parse_incoming(self, data):
        cmd_id, data_len, unk = struct.unpack("<BBH", data[:4])
        #verbose_print(f"unk: {hex(unk)} cmd_id: {hex(cmd_id)} data_len: {hex(data_len)} unk2: {hex(unk2)} ")

        ret = data[4:4+data_len-4]
        return cmd_id, ret

    def parse_tracker_status(self, data):
        unk, data_len = struct.unpack("<BB", data[:2])
        #verbose_print(f"unk: {hex(unk)} data_len: {hex(data_len)}")
        hex_dump(data[:2])
        data = data[2:2+data_len]
        hex_dump(data[0:1])
        data = data[1:]
        pair_state = [0,0,0,0,0]
        unk, pair_state[0],pair_state[1],pair_state[2],pair_state[3],pair_state[4],  = struct.unpack("<HLLLLL", data)
        verbose_print(hex(unk),hex(pair_state[0]),hex(pair_state[1]),hex(pair_state[2]),hex(pair_state[3]),hex(pair_state[4]))

        self.pair_state = pair_state

        # Fallback for disconnects
        if self.current_host_id >= 0:
            if self.pair_state[self.current_host_id] & PAIR_STATE_PAIRED == 0:
                self.handle_disconnected(self.current_host_id)

        # pair state:
        # 0x4000003 - unpaired, pairing info present?
        # 0x1000003 - unpaired, pairing info not present?
        # 0x320fc008 - paired
        # 0x320ff808 - paired
        # 0x320fa808 - paired


    def send_raw(self, data=None, pad=True):
        if data is None:
            data = []
        out = bytes(data)

        if pad:
            out += bytes([0x0] * (0x40 - len(out)))
        #verbose_print("Sending raw:")
        #hex_dump(out)

        try:
            ret = self.device_hid1.send_feature_report(out)

            resp = self.device_hid1.get_feature_report(0, 0x40)
            #hex_dump(resp)
                
            return resp
        except:
            return bytes([])

        return bytes([])

    def send_cmd(self, cmd_id, data=None):
        if data is None:
            data = []
        if type(data) is str:
            data = data.encode("utf-8")
        out = struct.pack("<BBB", 0, cmd_id, len(data)+2)
        out += bytes(data)

        out += bytes([0x0] * (0x41 - len(out)))

        #verbose_print(f"Sending idk:")
        #hex_dump(out)

        try:
            ret = self.device_hid1.send_feature_report(out)
            for i in range(0, 10):
                resp = self.device_hid1.get_feature_report(0, 0x40)
                #hex_dump(resp)
                err_ret, cmd_ret, data_ret = self.parse_response(resp)
                #hex_dump(data_ret)
                if err_ret:
                    verbose_print(f"Got error response: {hex(err_ret)}")
                if cmd_ret == cmd_id:
                    return data_ret
        except:
            return bytes([])

        return bytes([])

    def send_F4(self, trackers, subcmd, data=None):
        if data is None:
            data = []
        if len(trackers) != 5:
            return bytes([])
        checksummed_data = bytes(trackers) + bytes([subcmd]) + bytes(data)
        out_data = bytes([do_u8_checksum(checksummed_data)]) + checksummed_data
        hex_dump(out_data)
        return self.send_cmd(DCMD_F4, out_data)

    def is_host(self, device_addr):
        return mac_to_idx(device_addr) in self.host_slots

    def is_client(self, device_addr):
        return not self.is_host(device_addr)

    def client_has_host_map(self, device_addr):
        return self.has_host_map[mac_to_idx(device_addr)]

    def is_client_connected(self, device_addr):
        if self.is_host(device_addr): return True
        return self.connected_to_host[mac_to_idx(device_addr)]

    def set_pose_callback(self, callback_fn, callback_ref):
        self.pose_callback = callback_fn
        self.pose_callback_ref = callback_ref

    def parse_tracker_incoming(self, resp):
        cmd_id, pkt_idx, device_addr, type_maybe, data_len = struct.unpack("<BH6sHB", resp[:0xC])
        #hex_dump(resp)
        #verbose_print(hex(cmd_id), hex(pkt_idx), mac_str(device_addr), hex(type_maybe), "data_len:", hex(data_len))

        if type_maybe == 0x101:
            data_raw = resp[0xC:0xC+data_len]
            if self.ack_callback:
                self.ack_callback(self, device_addr, data_raw)
            return
        elif type_maybe == 0x110:
            data = resp[0xC:0xC+data_len]
            #self.parse_pose_data(device_addr, data)
            if self.pose_callback:
                self.pose_callback(self, device_addr, data)
            return

        data = resp[0xC:0xC+data_len]
        data_id = data[0]
        data_real = data[1:]
        verbose_print("   data_id:", hex(data_id), "data:", data_real)

    def get_PCBID(self):
        return self.send_cmd(DCMD_GET_CR_ID, [CR_ID_PCBID]).decode("utf-8")

    def get_SKUID(self):
        return self.send_cmd(DCMD_GET_CR_ID, [CR_ID_SKUID]).decode("utf-8")

    def get_SN(self):
        return self.send_cmd(DCMD_GET_CR_ID, [CR_ID_SN]).decode("utf-8")

    def get_ShipSN(self):
        return self.send_cmd(DCMD_GET_CR_ID, [CR_ID_SHIP_SN]).decode("utf-8")

    def get_CapFPC(self):
        return self.send_cmd(DCMD_GET_CR_ID, [CR_ID_CAP_FPC]).decode("utf-8")

    def get_ROMVersion(self):
        return self.send_cmd(DCMD_QUERY_ROM_VERSION, [0x00]).decode("utf-8")

    def send_ack_to(self, idx, ack):
        if idx < 0:
            return

        mac = [0x23, 0x30, 0x42, 0xB7, 0x82, 0xD3]
        mac[1] |= idx

        # TX_ACK_TO_MAC checks all MAC bytes,
        # TX_ACK_TO_PARTIAL_MAC checks the first 2
        preamble = struct.pack("<BBBBBBBBB", TX_ACK_TO_PARTIAL_MAC, mac[0],mac[1],mac[2],mac[3],mac[4],mac[5],0, 1)
        ack = struct.pack("<B", len(ack)) + ack.encode("utf-8")

        data = preamble + ack

        return self.send_cmd(DCMD_TX, data)

    def handle_disconnected(self, idx):
        if idx < 0:
            return
        if idx == self.current_host_id:
            self.current_host_id = -1
        self.host_slots.discard(idx)

        self.initialized_macs = {m for m in self.initialized_macs if mac_to_idx(m) != idx}
        self.device_ids.pop(idx, None)
        self.wifi_pending.discard(idx)
        self.connected_to_host[idx] = False
        self.has_host_map[idx] = False

        if self.disconnected_callback:
            self.disconnected_callback(self, idx)

    def restart_slam(self, slot):
        """Re-send the role/tracking-mode setup to one tracker, which makes it drop its
        current map and start a fresh one. Returns False if no tracker is known in that slot."""
        macs = [m for m in self.initialized_macs if mac_to_idx(m) == slot]
        if not macs:
            return False
        self.init_tracker(macs[0])
        return True

    def init_tracker(self, paired_mac):
        paired_mac_str = mac_str(paired_mac)
        self.initialized_macs.add(bytes(paired_mac))

        # I really wish they included the index of each tracker *somewhere*, but it seems
        # like the MACs have always been fake anyhow
        self.ack_set_role_id(mac_to_idx(paired_mac), 1)

        # TODO: detect re-pairs and force re-init

        #if self.num_paired <= 1:
        if self.all_hosts:
            make_host = True
        elif self.host_slot is not None:
            make_host = mac_to_idx(paired_mac) == self.host_slot
        else:
            make_host = self.current_host_id == -1 or self.is_host(paired_mac)

        if make_host and self.keep_map:
            verbose_print(f"Keeping stored map on {paired_mac_str} (skipping tracking mode reset)")
        else:
            self.ack_set_tracking_mode(mac_to_idx(paired_mac), -1)

        if make_host:
            test_mode = TRACKING_MODE_SLAM_HOST
            self.current_host_id = mac_to_idx(paired_mac)
            self.host_slots.add(mac_to_idx(paired_mac))
            verbose_print(f"Making {paired_mac_str} the SLAM host")
            self.wifi_set_country(mac_to_idx(paired_mac), self.wifi_info["country"])
            self.ack_set_tracking_host(mac_to_idx(paired_mac), 1)
            self.ack_set_wifi_host(mac_to_idx(paired_mac), 1)
            self.device_ids[mac_to_idx(paired_mac)] = 0
            self.ack_set_new_id(mac_to_idx(paired_mac), 0)
        else:
            test_mode = TRACKING_MODE_SLAM_CLIENT
            #self.wifi_connect(mac_to_idx(paired_mac))
            # Upstream used the slot number, which collides with the host's ID 0 when a
            # client sits in slot 0 and the host doesn't.
            slot = mac_to_idx(paired_mac)
            used = {v for k, v in self.device_ids.items() if k != slot}
            new_id = next(i for i in range(1, 6) if i not in used)
            self.device_ids[slot] = new_id

            self.ack_set_tracking_host(mac_to_idx(paired_mac), 0)
            self.ack_set_wifi_host(mac_to_idx(paired_mac), 0)
            self.ack_set_new_id(mac_to_idx(paired_mac), new_id)

        if make_host and self.host_mode_delay > 0 and not self.keep_map:
            verbose_print(f"Delaying host tracking mode on {paired_mac_str} by {self.host_mode_delay}s")
            self.pending_host_mode[mac_to_idx(paired_mac)] = time.time() + self.host_mode_delay
        else:
            self.ack_set_tracking_mode(mac_to_idx(paired_mac), test_mode)

    def send_pending_host_modes(self):
        now = time.time()
        for slot, due in list(self.pending_host_mode.items()):
            if now >= due:
                verbose_print(f"Sending delayed host tracking mode to slot {slot}")
                self.ack_set_tracking_mode(slot, TRACKING_MODE_SLAM_HOST)
                del self.pending_host_mode[slot]

    def do_loop(self):
        self.send_pending_host_modes()
        resp = self.device_hid1.read(0x400)
        if len(resp) <= 0:
            return
        #verbose_print("dump:")
        #hex_dump(resp)
        #verbose_print("parsed:")
        # 0x18 = paired event, gives the 
        if resp[0] == DRESP_PAIR_EVENT:
            self.got_a_pair = True
            #verbose_print("dump:")
            #hex_dump(resp)
            cmd_ret, data_ret = self.parse_incoming(resp)
            #hex_dump(data_ret)
            unk = data_ret[0]
            is_unpair = data_ret[1] # 0x1, id?
            paired_mac = data_ret[2:]
            paired_mac_str = mac_str(paired_mac)
            if paired_mac not in self.device_macs:
                self.device_macs += [paired_mac]
                self.num_paired += 1
            verbose_print(("Unpaired" if is_unpair else "Paired") + f" {paired_mac_str}, {hex(unk)}")
            #verbose_print(self.device_macs)
            self.calib_1 = ""
            self.calib_2 = ""

            if is_unpair:
                self.initialized_macs.discard(bytes(paired_mac))
                self.handle_disconnected(mac_to_idx(paired_mac))
                return

            self.init_tracker(paired_mac)

            
        elif resp[0] == DRESP_TRACKER_RF_STATUS or resp[0] == DRESP_TRACKER_NEW_RF_STATUS or resp[0] == 0x29:
            verbose_print(f"dump for {hex(resp[0])}:")
            #hex_dump(resp)
            cmd_ret, data_ret = self.parse_incoming(resp)
            if resp[0] == DRESP_TRACKER_RF_STATUS or resp[0] == DRESP_TRACKER_NEW_RF_STATUS:
                self.parse_tracker_status(data_ret)
            else:
                hex_dump(data_ret)
        elif resp[0] == DRESP_TRACKER_INCOMING:
            # Trackers that were already connected before we started never send a
            # pair event, so set them up the first time we hear from them.
            device_addr = bytes(resp[3:9])
            if device_addr not in self.initialized_macs:
                verbose_print(f"Initializing already-connected tracker {mac_str(device_addr)}")
                self.init_tracker(device_addr)
            self.parse_tracker_incoming(resp)
            #self.send_raw(bytes([0]) + resp)

            # These commands feel sketchy...
            #hex_dump(self.send_F4([1,1,1,1,1], 0))
            #hex_dump(self.send_F4([1,1,1,1,1], 1, struct.pack("<BL", 0, 0)))
            
            #self.send_ack_to_all(ACK_TRACKING_MODE + "1")
            #self.send_ack_to_all("ATW")
            
            #self.send_ack_to_all(ACK_LAMBDA_SET_STATUS + f"{KEY_TRANSMISSION_READY},1")
            #self.send_ack_to_all(ACK_LAMBDA_ASK_STATUS + f"{KEY_CURRENT_MAP_ID}")
            
            self.tick_periodic += 1
            if self.tick_periodic > 1000:
                verbose_print("Tick!")
                #self.send_ack_to_all(ACK_LAMBDA_MESSAGE + "1:0")
                

                #self.send_ack_to_all(ACK_LAMBDA_SET_STATUS + f"{KEY_TRANSMISSION_READY},1")
                #self.send_ack_to_all(ACK_ATW)


                for i in range(0, 5):
                    if not self.is_client_connected(i): continue
                    self.ack_lambda_ask_status(i, KEY_TRANSMISSION_READY)
                    self.ack_lambda_ask_status(i, KEY_CURRENT_MAP_ID)
                    self.ack_lambda_ask_status(i, KEY_MAP_STATE)
                    self.ack_lambda_ask_status(i, KEY_CURRENT_TRACKING_STATE)

                    #self.send_ack_to(i, ACK_NEW_ID + f"{i}")
                    #self.wifi_connect(i)
                    #self.wifi_set_ssid(i, self.wifi_info["ssid"])
                    #self.wifi_set_password(i, self.wifi_info["pass"])
                    #self.wifi_set_country(i, self.wifi_info["country"])
                    
                    if not self.is_host(i):
                        self.ack_lambda_ask_status(i, KEY_RECEIVED_HOST_ED)
                        self.ack_lambda_ask_status(i, KEY_RECEIVED_HOST_MAP)
                    
                    #    self.send_ack_to(i, ACK_LAMBDA_COMMAND + f"{ASK_ED}")
                    pass

                self.tick_periodic = 0
            #self.send_ack_to_all(ACK_LAMBDA_MESSAGE)
        else:
            verbose_print("dump:")
            hex_dump(resp)

class TrackerHID(Ackable):

    def __init__(self, wifi_info_path=None):
        self.watchdog_delay = 0
        self.poses_recvd = 0
        self.pose_callback = None
        self.ack_callback = None
        self.connected_callback = None
        self.disconnected_callback = None

        self.device_addr = bytes([0]*6)

        device_list = hid.enumerate(VID_VIVE, PID_TRACKER)
        verbose_print(device_list)

        #Find the device with the particular usage you want
        device_dict_hid1 = [device for device in device_list if device['interface_number'] == HID1_INTERFACE_NUM][0]
        self.device_hid1 = hid.Device(path=device_dict_hid1['path'])

        '''
        device_dict_hid3 = [device for device in device_list if device['interface_number'] == HID3_INTERFACE_NUM][0]
        device_hid3 = hid.Device(path=device_dict_hid3['path'])
        '''

        #self.set_tracking_mode(TRACKING_MODE_SLAM_HOST)
        #self.set_tracking_mode(0xFFFFFFFF)
        #self.set_tracking_mode(TRACKING_MODE_SLAM_HOST)
        self.set_power_pcvr(TRACKING_MODE_SLAM_HOST)

        wifi_path = Path(wifi_info_path) if wifi_info_path else Path(__file__).with_name('wifi_info.json')
        with wifi_path.open('r', encoding='utf-8') as f:
            self.wifi_info = json.load(f)

        '''
        self.send_ack_to(self.device_addr, ACK_FW + "1")
        self.wifi_set_ssid(self.device_addr, self.wifi_info["ssid"])
        self.wifi_set_password(self.device_addr, self.wifi_info["pass"])
        self.wifi_set_country(self.device_addr, self.wifi_info["country"])
        self.wifi_set_freq(self.device_addr, self.wifi_info["freq"])
        self.wifi_connect(self.device_addr)
        '''

    def parse_response(self, data):
        unk, cmd_id, data_len, unk2 = struct.unpack("<BHBB", data[:5])
        #verbose_print(f"unk: {hex(unk)} cmd_id: {hex(cmd_id)} data_len: {hex(data_len)} unk2: {hex(unk2)} ")

        ret = data[5:5+data_len]
        return cmd_id, ret

    def send_command(self, cmd_id, data=None):
        if data is None:
            data = []
        out = struct.pack("<HBB", cmd_id, len(data), 0)
        out += bytes(data)

        out += bytes([0x0] * (0x40 - len(out)))

        try:
            ret = self.device_hid1.send_feature_report(out)
            for i in range(0, 10):
                resp = self.device_hid1.get_feature_report(0, 0x40)
                #hex_dump(resp)
                cmd_ret, data_ret = self.parse_response(resp)
                #hex_dump(data_ret)
                if cmd_ret == cmd_id:
                    return data_ret
        except:
            return bytes([])

        return bytes([])

    # ACK related
    def send_ack_to(self, idx, ack):
        return self.send_ack(ack)

    def send_ack_to_all(self, ack):
        return self.send_ack(ack)

    def send_ack(self, ack):
        return self.send_command(PACKET_SET_ACK, ack.encode("utf-8"))

    def get_ack(self):
        return self.send_command(PACKET_GET_ACK)

    # Serial/device ino related
    def get_str_info(self, idx):
        return self.send_command(PACKET_GET_STR_INFO, [idx]).decode("utf-8")

    def set_str_info(self, idx, val):
        return self.send_command(PACKET_SET_STR_INFO, struct.pack("<BS", idx, val))

    def get_status(self):
        ret = self.send_command(PACKET_GET_STATUS)
        tracking_mode, power, batt, hmd_init, device_status_mask, btn = struct.unpack("<BBBBBL", ret)
        verbose_print(f"tracking_mode={tracking_mode}, power={power}, batt={batt}, hmd_init={hmd_init}, device_status_mask={hex(device_status_mask)}, btn={hex(btn)}")

    def set_tracking_mode(self, mode):
        self.send_command(PACKET_SET_TRACKING_MODE, [mode & 0xFF])

    def send_haptic(self, nDuration, nFrequency, nAmplitude, identify):
        self.send_command(PACKET_SET_HAPTIC, struct.pack("<HHHH", nDuration, nFrequency, nAmplitude, identify))

    # ex: bootloader, fastboot, etc. I think.
    def set_reboot(self, reason=""):
        self.send_command(PACKET_SET_REBOOT, reason.encode("utf-8"))

    def get_property(self, key):
        self.send_command(PACKET_GET_PROPERTY, key.encode("utf-8"))

    def parse_incoming(self):
        resp = self.device_hid1.read(0x400)
        if len(resp) <= 0:
            return
        unk0, pkt_idx, mask, hmd_us, hdcc_status0, hdcc_status1, ack_in_queue, device_status, unk3 = struct.unpack("<BHLQBBBL17s", resp[:0x27])
        #hex_dump(resp)
        verbose_print(unk0, pkt_idx, hex(mask), hmd_us, hex(hdcc_status0), hex(hdcc_status1), ack_in_queue, hex(device_status), unk3)

        for i in range(0, 7):
            pose_len = resp[0x27 + (0x61*i)]
            pose_data = resp[0x28 + (0x61*i) : 0x28 + (0x61*i) + pose_len]
            pose_timestamp, = struct.unpack("<Q", resp[0x27 + (0x61*i) + 0x59 : 0x27 + (0x61*i) + 0x59 + 0x8])
            if pose_len:
                verbose_print("Pose data", i, "len", hex(pose_len), "ts", pose_timestamp)
                #hex_dump(pose_data)
                if self.pose_callback:
                    self.pose_callback(self, self.device_addr, pose_data)
        netsync_str = "netsync: "
        for i in range(0, 7):
            a,b = struct.unpack("<QQ", resp[0x2ce+(i*0x10):0x2ce+(i*0x10)+0x10])
            netsync_str += f"({a},{b}) "
        verbose_print(netsync_str)
        verbose_print("")
        verbose_print("")

    # 1=gyro, 2=body tracking(?), 3=body?
    def set_power_pcvr(self, mode):
        self.send_command(PACKET_SET_POWER_PCVR, [mode & 0xFF])

    def kick_watchdog(self):
        self.watchdog_delay += 1
        if self.watchdog_delay < 1000:
            return
        self.send_command(PACKET_SET_WATCHDOG_KICK)
        self.watchdog_delay = 0

    # default 3?
    def set_camera_policy(self, val):
        self.send_command(PACKET_SET_CAMERA_POLICY, [val])

    # default 3?
    def set_camera_fps(self, val):
        self.send_command(PACKET_SET_CAMERA_FPS, [val])

    def do_loop(self):
        self.parse_incoming()
        self.kick_watchdog()

        #self.ack_lambda_property(self.device_addr)
        #self.lambda_end_map(self.device_addr)
        
        data = self.get_ack()
        # TODO: first byte is always 0xFF, why
        if data and len(data) > 1 and self.ack_callback:
            self.ack_callback(self, self.device_addr, data[1:])

    def is_host(self, device_addr):
        return True # TODO

    def is_client(self, device_addr):
        return not self.is_host(device_addr)

class ViveTrackerGroup():

    def __init__(self, mode="DONGLE_USB", wifi_info_path=None, debug=True, host_slot=None, all_hosts=False, keep_map=False, host_mode_delay=0.0):
        set_tracker_core_verbose(debug)
        self.poses_recvd = [0]*5
        self.pose_quat = [[0.0, 0.0, 0.0, 1.0]] * 5
        self.pose_pos = [[0.0, 0.0, 0.0]] * 5
        self.pose_time = [0] * 5
        self.pose_tracking_status = [0] * 5
        self.pose_btns = [0]*5
        self.last_pose_btns = [0]*5
        #self.wip_pose_btns = [0]*5

        self.tracker_map_state = [0]*5
        # slot -> {SLAM key id: last reported value}, from LAMBDA_PROP_GET_STATUS replies
        self.slam_status = {}
        # slot -> serial suffix, taken from the host AP name (DIRECT-xx-<serial suffix>).
        # Slots and MACs change on re-pairing; the serial identifies the physical tracker.
        self.tracker_serials = {}
        self.stuck_on_static = [0]*5
        self.stuck_on_exists = [0]*5
        self.stuck_on_not_checked = [0]*5
        self.bump_map_once = [True]*5
        self.bump_map_once_2 = [True]*5

        self.pose_listeners = []
        self._active_tracker_slots = []

        # TODO: mix of multiple?
        if mode == "DONGLE_USB":
            self.comms = DongleHID(wifi_info_path=wifi_info_path, host_slot=host_slot, all_hosts=all_hosts, keep_map=keep_map,
                                   host_mode_delay=host_mode_delay)
        elif mode == "TRACKER_USB":
            self.comms = TrackerHID(wifi_info_path=wifi_info_path)

        self.comms.pose_callback = self.parse_pose_data
        self.comms.ack_callback = self.parse_ack
        self.comms.connected_callback = self.handle_connected
        self.comms.disconnected_callback = self.handle_disconnected

    def add_pose_listener(self, listener):
        if listener not in self.pose_listeners:
            self.pose_listeners.append(listener)

    def remove_pose_listener(self, listener):
        self.pose_listeners = [cb for cb in self.pose_listeners if cb != listener]

    def clear_pose_listeners(self):
        self.pose_listeners.clear()

    def handle_connected(self, comms, idx):
        pass

    def handle_disconnected(self, comms, idx):
        if idx < 0:
            return

        self.tracker_map_state[idx] = 0
        self.stuck_on_static[idx] = 0
        self.stuck_on_exists[idx] = 0
        self.stuck_on_not_checked[idx] = 0
        self.bump_map_once[idx] = True
        self.bump_map_once_2[idx] = True
        if idx in self._active_tracker_slots:
            self._active_tracker_slots.remove(idx)

    # TODO: comms -> self
    def handle_map_state(self, comms, device_addr, state):
        self.tracker_map_state[mac_to_idx(device_addr)] = state

        if self.stuck_on_static[mac_to_idx(device_addr)] > 7:
            verbose_print("ok we're stuck, end the map again")
            self.bump_map_once_2[mac_to_idx(device_addr)] = True
            self.stuck_on_static[mac_to_idx(device_addr)] = 0

        if self.stuck_on_exists[mac_to_idx(device_addr)] > 3:
            self.stuck_on_exists[mac_to_idx(device_addr)] = 0

        if self.stuck_on_not_checked[mac_to_idx(device_addr)] > 3:
            self.stuck_on_not_checked[mac_to_idx(device_addr)] = 0

        if state == MAP_REBUILD_WAIT_FOR_STATIC:
            #comms.send_ack_to(mac_to_idx(device_addr), ACK_LAMBDA_COMMAND + f"{RESET_MAP}")
            self.stuck_on_static[mac_to_idx(device_addr)] += 1
            if self.bump_map_once_2[mac_to_idx(device_addr)]:
                if comms.is_client(device_addr):
                    verbose_print("End the map?")
                    #comms.lambda_end_map(comms.current_host_id)
                    #comms.lambda_end_map(device_addr)
                    #comms.send_ack_to(comms.current_host_id, ACK_LAMBDA_COMMAND + f"{ASK_MAP}")
                    #comms.send_ack_to(mac_to_idx(device_addr), ACK_LAMBDA_COMMAND + f"{RESET_MAP}")
                self.bump_map_once_2[mac_to_idx(device_addr)] = False
            #comms.send_ack_to(mac_to_idx(device_addr), ACK_LAMBDA_SET_STATUS + f"{KEY_MAP_STATE},{MAP_REBUILT}")
            #comms.send_ack_to(mac_to_idx(device_addr), ACK_LAMBDA_SET_STATUS + f"{KEY_CURRENT_TRACKING_STATE},{MAP_REBUILD_CREATE_MAP}")
        else:
            self.bump_map_once_2[mac_to_idx(device_addr)] = True
            self.stuck_on_static[mac_to_idx(device_addr)] = 0

        if state == MAP_EXIST:
            if self.stuck_on_exists[mac_to_idx(device_addr)] == 0:# and comms.is_host(device_addr):
                verbose_print("ok we're stuck on exists, end the map again")
                #comms.lambda_end_map(device_addr)
            self.stuck_on_exists[mac_to_idx(device_addr)] += 1
        else:
            self.stuck_on_exists[mac_to_idx(device_addr)] = 0

        if state == MAP_NOT_CHECKED:
            if self.stuck_on_not_checked[mac_to_idx(device_addr)] == 0 and comms.is_client(device_addr) and comms.client_has_host_map(device_addr):
                verbose_print("ok we're stuck on not checked, end the map again")
                comms.lambda_end_map(device_addr)
            self.stuck_on_not_checked[mac_to_idx(device_addr)] += 1
        else:
            self.stuck_on_not_checked[mac_to_idx(device_addr)] = 0

    def parse_pose_data(self, comms, mac, data):
        #verbose_print(comms, self)
        self.poses_recvd[mac_to_idx(mac)] += 1

        if len(data) == 0x2:
            idx, btns = struct.unpack("<BB", data)
            verbose_print(f"({mac_str(mac)})", hex(idx), "btns:", hex(btns))
            return

        if len(data) != 0x25 and len(data) != 0x27:
            verbose_print("Weird pose data.", len(data))
            hex_dump(data)
            return
        idx, btns, pos, rot, acc, rot_vel, tracking_status = struct.unpack("<BB12s8s6s8sB", data[:0x25])
        
        # tracking_status = 2 => pose + rot
        # tracking_status = 3 => rot only
        # tracking_status = 4 => pose frozen (lost tracking), rots

        pos_arr = np.frombuffer(pos, dtype=np.float32, count=3)
        rot_arr = np.frombuffer(rot, dtype=np.float16, count=4)
        # Tracker firmware emits quaternions in [w, z, y, x]; convert to [w, x, y, z]
        rot_arr = np.array([rot_arr[0], rot_arr[3], rot_arr[2], rot_arr[1]], dtype=np.float32)
        acc_arr = np.frombuffer(acc, dtype=np.float16, count=3)
        rot_vel_arr = np.frombuffer(rot_vel, dtype=np.float16, count=4)
        #verbose_print(f"({mac_str(mac)})", hex(idx), pose_status_to_str(tracking_status), "btns:", hex(btns), "pos:", pos_arr, "rot:", rot_arr, "acc:", acc_arr, "rot_vel:", rot_vel_arr, "time_delta", current_milli_time() - self.pose_time[mac_to_idx(mac)])

        #verbose_print(hex(idx), hex(btns), hex(self.pose_btns[mac_to_idx(mac)]), hex(self.last_pose_btns[mac_to_idx(mac)]))
        self.pose_quat[mac_to_idx(mac)] = rot_arr
        self.pose_pos[mac_to_idx(mac)] = pos_arr
        self.pose_time[mac_to_idx(mac)] = current_milli_time()
        self.pose_tracking_status[mac_to_idx(mac)] = tracking_status

        if btns & 0x80:
            self.last_pose_btns[mac_to_idx(mac)] = self.pose_btns[mac_to_idx(mac)]
            self.pose_btns[mac_to_idx(mac)] = ((btns & 0x7F) << 8) | self.pose_btns[mac_to_idx(mac)] & 0xFF
        else:
            self.last_pose_btns[mac_to_idx(mac)] = self.pose_btns[mac_to_idx(mac)]
            self.pose_btns[mac_to_idx(mac)] = btns | self.pose_btns[mac_to_idx(mac)] & 0xFF00

        if (self.pose_btns[mac_to_idx(mac)] & 0x100) == 0x100 and (self.last_pose_btns[mac_to_idx(mac)] & 0x100) == 0x0:# and comms.get_map_state(mac) == MAP_EXIST:
            verbose_print("end map.")
            comms.lambda_end_map(mac)

        self._emit_pose_event(
            idx=idx,
            mac=mac,
            tracking_status=tracking_status,
            buttons=self.pose_btns[mac_to_idx(mac)],
            pos_arr=pos_arr,
            rot_arr=rot_arr,
            acc_arr=acc_arr,
            rot_vel_arr=rot_vel_arr,
        )

    def parse_ack(self, comms, device_addr, data_raw):
        data = data_raw.decode("utf-8")
        if data[0:1] == ACK_CATEGORY_CALIB_1:
            data_real = data[1:]
            comms.calib_1 += data_real
            verbose_print(f"   Got CALIB_1 ({mac_str(device_addr)}):", comms.calib_1)
        elif data[0:1] == ACK_CATEGORY_CALIB_2:
            data_real = data[1:]
            comms.calib_2 += data_real
            verbose_print(f"   Got CALIB_2 ({mac_str(device_addr)}):", comms.calib_2)
        elif data[0:1] == ACK_CATEGORY_DEVICE_INFO:
            data_real = data[1:]
            verbose_print(f"   Got device info ACK ({mac_str(device_addr)}):", data_real[:3])

            # Handle post-deviceinfo commands
            if data_real[0:3] == ACK_AZZ:
                if comms.is_host(device_addr):
                    #comms.lambda_end_map(device_addr)
                    pass
                else:
                    comms.send_ack_to(mac_to_idx(device_addr), ACK_LAMBDA_COMMAND + f"{RESET_MAP}")
                    pass
                comms.send_ack_to(mac_to_idx(device_addr), ACK_FW + "1")
        elif data[0:1] == ACK_CATEGORY_PLAYER:
            data_real = data[1:]
            parts = data_real.split(":")
            idx = int(parts[0])
            args = parts[1]
            

            if idx == LAMBDA_PROP_GET_STATUS:
                args = [int(s) for s in args.split(",")]
                key_id = args[0]
                state = args[1]
                addendum = ""

                if key_id == KEY_RECEIVED_HOST_ED:
                    #if state == 0:
                    #    comms.send_ack_to(mac_to_idx(device_addr), ACK_LAMBDA_COMMAND + f"{ASK_ED}")
                    pass
                elif key_id == KEY_RECEIVED_HOST_MAP:
                    if state == 0 and (current_milli_time() - comms.last_host_map_ask_ms) > 10000 and comms.is_client_connected(device_addr):
                        verbose_print("ask for map again")
                        comms.last_host_map_ask_ms = current_milli_time()
                        #comms.send_ack_to(mac_to_idx(device_addr), ACK_LAMBDA_COMMAND + f"{RESET_MAP}")
                        #comms.send_ack_to(comms.current_host_id, ACK_LAMBDA_COMMAND + f"{ASK_MAP}") # doesn't work?
                        comms.send_ack_to(mac_to_idx(device_addr), ACK_LAMBDA_COMMAND + f"{ASK_MAP}") # doesn't do anything?
                        # TODO: when do we ask for maps?
                        self.bump_map_once[mac_to_idx(device_addr)] = True
                        comms.has_host_map[mac_to_idx(device_addr)] = False
                        pass
                    else:
                        if state > 0:
                            comms.has_host_map[mac_to_idx(device_addr)] = True
                        if self.bump_map_once[mac_to_idx(device_addr)]:
                            #comms.send_ack_to(mac_to_idx(device_addr), ACK_END_MAP)
                            #comms.send_ack_to(mac_to_idx(device_addr), ACK_TRACKING_MODE + "-1")
                            #comms.send_ack_to(mac_to_idx(device_addr), ACK_TRACKING_MODE + "1")
                            self.bump_map_once[mac_to_idx(device_addr)] = False
                        #comms.send_ack_to(mac_to_idx(paired_mac), ACK_END_MAP)
                        #comms.send_ack_to_all(ACK_FW + "1")
                elif key_id == KEY_TRANSMISSION_READY:
                    #comms.send_ack_to(mac_to_idx(device_addr), ACK_LAMBDA_COMMAND + f"{RESET_MAP}")
                    #comms.send_ack_to(mac_to_idx(device_addr), ACK_LAMBDA_COMMAND + f"{ASK_ED}")
                    a='a'
                elif key_id == KEY_MAP_STATE:
                    addendum = f"({map_status_to_str(state)})"

                    self.handle_map_state(comms, device_addr, state)
                elif key_id == KEY_CURRENT_TRACKING_STATE:
                    addendum = f"({pose_status_to_str(state)})"

                self.slam_status.setdefault(mac_to_idx(device_addr), {})[key_id] = state
                verbose_print(f"   Status returned for SLAM key {slam_key_to_str(key_id)} ({mac_str(device_addr)}): {state} {addendum}")

            else:
                verbose_print(f"   Got PLAYER ACK ({mac_str(device_addr)}):", f"CMD{idx}", args)
        elif data[0:2] == ACK_LAMBDA_PROPERTY:
            data_real = data[2:]
            verbose_print(f"   Got LP ACK ({mac_str(device_addr)}):", data_real)
        elif data[0:2] == ACK_LAMBDA_STATUS:
            data_real = data[2:]
            a, b, c = [int(s) for s in data_real.split(",")]
            verbose_print(f"   Got LAMBDA_STATUS ACK ({mac_str(device_addr)}): {a},{b},{c}")
            comms.send_ack_to_all(ACK_FW + "0")
            if b != 2:
                #verbose_print("ask for host map.")
                #comms.send_ack_to(mac_to_idx(device_addr), ACK_LAMBDA_COMMAND + f"{ASK_ED}")
                #comms.send_ack_to(mac_to_idx(device_addr), ACK_LAMBDA_COMMAND + f"{ASK_MAP}")
                #comms.send_ack_to(mac_to_idx(device_addr), ACK_LAMBDA_COMMAND + f"{KF_SYNC}")
                pass
                
        elif data[0:4] == ACK_ERROR_CODE:
            data_real = data[4:]
            verbose_print(f"   Got ERROR ({mac_str(device_addr)}):", data_real)
        elif data[0:2] == ACK_WIFI_HOST_SSID:
            data_real = data[2:]
            ssid, passwd, freq = data_real.split(",")
            verbose_print(f"   Got WIFI_HOST_SSID ACK ({mac_str(device_addr)}): ssid={ssid}, pass={passwd}, freq={freq}")

            comms.host_ssid = ssid
            comms.host_passwd = passwd
            comms.host_freq = freq
            save_host_ap_cache(ssid, passwd, freq)
            self.tracker_serials[mac_to_idx(device_addr)] = ssid.rsplit("-", 1)[-1]

            # Clients that asked before the host reported its AP got empty credentials
            pending = getattr(comms, "wifi_pending", set())
            for idx in sorted(pending):
                verbose_print(f"   Sending host Wi-Fi to waiting client slot {idx}")
                self.send_host_wifi(comms, idx)
            pending.clear()
        elif data[0:2] == ACK_WIFI_SSID_PASS:
            verbose_print(f"   Got WIFI_SSID ACK ({mac_str(device_addr)})")

            if comms.host_ssid:
                self.send_host_wifi(comms, mac_to_idx(device_addr))
            elif hasattr(comms, "wifi_pending"):
                comms.wifi_pending.add(mac_to_idx(device_addr))
        elif data[0:2] == ACK_WIFI_CONNECT:
            ret = int(data[2:])
            verbose_print(f"   Got WIFI_CONNECT ACK ({mac_str(device_addr)}): {ret}")
            comms.connected_to_host[mac_to_idx(device_addr)] = (ret > 0)
            

        elif data[0:2] == ACK_MAP_STATUS:
            data_real = data[2:]
            status = [int(s) for s in data_real.split(",")]

            self.handle_map_state(comms, device_addr, status[1])

            verbose_print(f"   Got MAP_STATUS ({mac_str(device_addr)}):", status, f"({map_status_to_str(status[1])})")
            #comms.send_ack_to_all(ACK_END_MAP)

            # Initial map status:
            # Got MAP_STATUS: -1,10
            # Got MAP_STATUS: 0,10
            # Got MAP_STATUS: 0,3

            # Losing tracking?
            # Got LP ACK: 1,0,1
            # Got MAP_STATUS: -1,0
            # Got MAP_STATUS: -1,1

            # Got MAP_STATUS: 0,6 = mapped and tracking
        elif data[0:3] == ACK_POWER_OFF:
            verbose_print(f".  Got POWER_OFF. ({mac_str(device_addr)})")
            comms.handle_disconnected(mac_to_idx(device_addr))
        elif data[0:3] == ACK_RESET:
            verbose_print(f".  Got RESET ({mac_str(device_addr)}).")
            comms.handle_disconnected(mac_to_idx(device_addr))
        else:
            verbose_print(f"   Got ACK ({mac_str(device_addr)}):", data, "(", data_raw, ")")
        verbose_print("")


    def send_host_wifi(self, comms, idx):
        comms.wifi_set_ssid(idx, comms.host_ssid)
        comms.wifi_set_password(idx, comms.host_passwd)
        #comms.wifi_set_ssid_password(idx, comms.host_ssid, comms.host_passwd)
        comms.wifi_set_freq(idx, comms.host_freq)
        comms.wifi_set_country(idx, comms.wifi_info["country"])

    def get_map_state(self, device_addr):
        return self.tracker_map_state[mac_to_idx(device_addr)]

    def get_pos(self, idx=0):
        return np.array(self.pose_pos[idx])

    def get_rot(self, idx=0):
        return np.array(self.pose_quat[idx])

    def get_pose(self, idx=0):
        return {
            "position": np.array(self.pose_pos[idx], dtype=np.float32),
            "rotation": np.array(self.pose_quat[idx], dtype=np.float32),
            "buttons": int(self.pose_btns[idx]),
            "tracking_status": int(self.pose_tracking_status[idx]),
            "timestamp_ms": int(self.pose_time[idx]),
        }

    def do_loop(self):
        self.comms.do_loop()

    def _emit_pose_event(self, idx, mac, tracking_status, buttons, pos_arr, rot_arr, acc_arr, rot_vel_arr):
        if not self.pose_listeners:
            return

        tracker_slot = mac_to_idx(mac)
        if tracker_slot not in self._active_tracker_slots:
            self._active_tracker_slots.append(tracker_slot)
        contiguous_index = self._active_tracker_slots.index(tracker_slot)

        sample = {
            "tracker_index": contiguous_index,
            "mac": bytes(mac),
            "buttons": buttons,
            "tracking_status": tracking_status,
            "timestamp_ms": self.pose_time[tracker_slot],
            "position": np.array(pos_arr, dtype=np.float32),
            "rotation": np.array(rot_arr, dtype=np.float32),
            "acceleration": np.array(acc_arr, dtype=np.float32),
            "angular_velocity": np.array(rot_vel_arr, dtype=np.float32),
        }

        for listener in list(self.pose_listeners):
            try:
                listener(sample)
            except Exception as exc:
                verbose_print(f"Pose listener raised {exc}")

if __name__ == '__main__':
    trackers = ViveTrackerGroup()
    #trackers = ViveTrackerGroup(mode="TRACKER_USB")

    while True:
        trackers.do_loop()


# What USB-HID PCVR does internally, could be useful for BTLE?
'''
              send_horus_cmd(0,0); // HORUS_CMD_POWER 0
              send_horus_cmd(0xb,0xffff); // HORUS_CMD_UPDATE_DEVICE_ID
              send_horus_cmd(0xc,bVar9); // HORUS_CMD_SET_TRACKING_MODE
              if (command_ == 0xa101) { // ocvr
                uVar24 = 8;
              }
              else {
                uVar24 = 9;
              }
              idk_pcvr_mode(0,uVar24); // HORUS_CMD_POWER, 
'''