#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-or-later
# AP_FLAKE8_CLEAN
"""
Reticulum / LXMF / Sideband Live Drone Bridge for Seeed Wio-LR2021 + TMotorH743.

Provides:
1. Ground Station Reticulum Transport node (`ground`):
   Bridges LAN/WiFi (`AutoInterface` + `TCPServerInterface` for Sideband on Android)
   to the Ground Wio-LR2021 RNode (`/dev/ttyACM5`) over 867.2 MHz UK LoRa RF.
2. Airborne Drone LXMF + MAVLink v2 Companion (`drone`):
   Connects ONLY via the Airborne Wio-LR2021 RNode (`/dev/ttyACM7`) over 867.2 MHz RF
   and bridges live MAVLink v2 telemetry and commands with the ArduCopter flight
   controller (`/dev/ttyACM4`), including Sideband `FIELD_TELEMETRY` sensors and
   `FIELD_ICON_APPEARANCE` ("quadcopter").
3. Raw RNode-over-TCP server (`rnode-tcp` on port 7633):
   Allows Sideband's built-in Hardware -> RNode (TCP) interface on Android to drive
   `/dev/ttyACM5` directly over WiFi.
"""

import argparse
import math
import os
import socket
import struct
import subprocess
import sys
import threading
import time

import LXMF
import RNS
from RNS.vendor import umsgpack as msgpack
import serial

COPTER_MODES = {
    0: "STABILIZE",
    1: "ACRO",
    2: "ALT_HOLD",
    3: "AUTO",
    4: "GUIDED",
    5: "LOITER",
    6: "RTL",
    7: "CIRCLE",
    9: "LAND",
    11: "DRIFT",
    16: "POSHOLD",
    17: "BRAKE",
}

MODE_NAME_TO_NUM = {name.lower(): num for num, name in COPTER_MODES.items()}
MODE_NAME_TO_NUM["althold"] = 2

# MAVLink v2 X.25 CRC_EXTRA table for messages we parse or emit
MAV_CRC_EXTRA = {
    0: 50,    # HEARTBEAT
    1: 124,   # SYS_STATUS
    29: 115,  # SCALED_PRESSURE
    30: 39,   # ATTITUDE
    33: 104,  # GLOBAL_POSITION_INT
    47: 153,  # MISSION_ACK
    65: 118,  # RC_CHANNELS
    70: 124,  # RC_CHANNELS_OVERRIDE
    73: 38,   # MISSION_ITEM_INT
    76: 152,  # COMMAND_LONG
    77: 143,  # COMMAND_ACK
    253: 83,  # STATUSTEXT
}


def mav_crc16(data: bytes, crc_extra: int) -> int:
    crc = 0xFFFF
    for b in list(data) + [crc_extra]:
        tmp = b ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


class MavlinkFCBridge:
    """Thread-safe MAVLink v2 reader and command sender for ArduCopter."""

    def __init__(self, port: str = "/dev/ttyACM4", baud: int = 115200) -> None:
        self.port = port
        self.baud = baud
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.lock = threading.Lock()
        self.seq = 0
        self.running = True

        self.state = {
            "connected": False,
            "armed": False,
            "custom_mode": 0,
            "mode_name": "STABILIZE",
            "vbatt_mv": 0,
            "current_ca": 0,
            "batt_rem": 100,
            "lat_deg": 0.0,
            "lon_deg": 0.0,
            "alt_msl_m": 42.5,
            "alt_rel_m": 0.0,
            "roll_deg": 0.0,
            "pitch_deg": 0.0,
            "yaw_deg": 0.0,
            "press_hpa": 1013.25,
            "temp_c": 25.0,
            "rc_ch": [1500, 1500, 1000, 1500],
            "last_ack_cmd": None,
            "last_ack_res": None,
            "last_ack_time": 0.0,
            "last_mission_ack": None,
            "last_mission_ack_time": 0.0,
            "last_statustext": "",
            "rx_frames": 0,
        }
        self._thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._thread.start()

    def close(self) -> None:
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()

    def _send_mav2(self, msgid: int, payload: bytes) -> None:
        crc_extra = MAV_CRC_EXTRA[msgid]
        with self.lock:
            seq = self.seq & 0xFF
            self.seq += 1
            hdr = bytes([
                0xFD,
                len(payload),
                0x00,
                0x00,
                seq,
                255,
                190,
                msgid & 0xFF,
                (msgid >> 8) & 0xFF,
                (msgid >> 16) & 0xFF,
            ])
            crc = mav_crc16(hdr[1:] + payload, crc_extra)
            pkt = hdr + payload + struct.pack("<H", crc)
            self.ser.write(pkt)
            self.ser.flush()

    def send_command_long(
        self,
        cmd_id: int,
        p1: float = 0.0,
        p2: float = 0.0,
        p3: float = 0.0,
        p4: float = 0.0,
        p5: float = 0.0,
        p6: float = 0.0,
        p7: float = 0.0,
    ) -> None:
        payload = struct.pack(
            "<fffffffHB", p1, p2, p3, p4, p5, p6, p7, cmd_id, 1, 1
        ) if False else struct.pack("<fffffffHBB", p1, p2, p3, p4, p5, p6, p7, cmd_id, 1, 1)
        self._send_mav2(76, payload)

    def send_guided_goto(self, lat_deg: float, lon_deg: float, alt_m: float) -> None:
        lat_e7 = int(round(lat_deg * 1e7))
        lon_e7 = int(round(lon_deg * 1e7))
        payload = struct.pack(
            "<ffffiifHBBBBBB",
            0.0,
            0.0,
            0.0,
            0.0,
            lat_e7,
            lon_e7,
            float(alt_m),
            0,    # seq
            16,   # MAV_CMD_NAV_WAYPOINT
            1,    # target_system
            1,    # target_component
            6,    # MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
            2,    # current = 2 (guided mode target)
            1,    # autocontinue
        ) + bytes([0])
        self._send_mav2(73, payload)

    def send_rc_override(self, c1: int, c2: int, c3: int, c4: int) -> None:
        payload = struct.pack("<HHHHHHHHBB", c1, c2, c3, c4, 0, 0, 0, 0, 1, 1)
        self._send_mav2(70, payload)

    def _rx_loop(self) -> None:
        buf = bytearray()
        while self.running:
            try:
                chunk = self.ser.read(256)
                if not chunk:
                    continue
                buf.extend(chunk)
                while len(buf) >= 12:
                    idx = buf.find(0xFD)
                    if idx < 0:
                        buf.clear()
                        break
                    if idx > 0:
                        del buf[:idx]
                    if len(buf) < 12:
                        break
                    plen = buf[1]
                    total = 12 + plen
                    if len(buf) < total:
                        break
                    frame = bytes(buf[:total])
                    del buf[:total]
                    msgid = frame[7] | (frame[8] << 8) | (frame[9] << 16)
                    pl = frame[10:10 + plen]
                    self._handle_msg(msgid, pl)
            except Exception:
                time.sleep(0.05)

    def _handle_msg(self, msgid: int, pl: bytes) -> None:
        self.state["connected"] = True
        self.state["rx_frames"] += 1
        if msgid == 0 and len(pl) >= 7:
            custom_mode = struct.unpack_from("<I", pl, 0)[0]
            base_mode = pl[6]
            self.state["custom_mode"] = custom_mode
            self.state["mode_name"] = COPTER_MODES.get(custom_mode, f"MODE_{custom_mode}")
            self.state["armed"] = bool(base_mode & 0x80)
        elif msgid == 1 and len(pl) >= 14:
            vbatt = struct.unpack_from("<H", pl, 14)[0]
            curr = struct.unpack_from("<h", pl, 16)[0] if len(pl) >= 18 else 0
            rem = struct.unpack_from("<b", pl, 30)[0] if len(pl) >= 31 else -1
            if vbatt > 0:
                self.state["vbatt_mv"] = vbatt
            self.state["current_ca"] = curr
            if rem >= 0:
                self.state["batt_rem"] = rem
        elif msgid == 29 and len(pl) >= 12:
            press_abs, _press_diff = struct.unpack_from("<ff", pl, 4)
            temp_cdeg = struct.unpack_from("<h", pl, 12)[0] if len(pl) >= 14 else 2500
            self.state["press_hpa"] = press_abs
            self.state["temp_c"] = temp_cdeg / 100.0
        elif msgid == 30 and len(pl) >= 16:
            roll, pitch, yaw = struct.unpack_from("<fff", pl, 4)
            self.state["roll_deg"] = math.degrees(roll)
            self.state["pitch_deg"] = math.degrees(pitch)
            self.state["yaw_deg"] = (math.degrees(yaw) + 360.0) % 360.0
        elif msgid == 33 and len(pl) >= 16:
            lat, lon, alt, rel_alt = struct.unpack_from("<iiii", pl, 4)
            if lat != 0 or lon != 0:
                self.state["lat_deg"] = lat / 1e7
                self.state["lon_deg"] = lon / 1e7
            self.state["alt_msl_m"] = alt / 1000.0
            self.state["alt_rel_m"] = rel_alt / 1000.0
        elif msgid == 65 and len(pl) >= 12:
            c1, c2, c3, c4 = struct.unpack_from("<HHHH", pl, 4)
            self.state["rc_ch"] = [c1, c2, c3, c4]
        elif msgid == 77 and len(pl) >= 3:
            cmd_id, res = struct.unpack_from("<HB", pl, 0)
            self.state["last_ack_cmd"] = cmd_id
            self.state["last_ack_res"] = res
            self.state["last_ack_time"] = time.time()
        elif msgid == 47 and len(pl) >= 3:
            mtype = pl[2]
            self.state["last_mission_ack"] = mtype
            self.state["last_mission_ack_time"] = time.time()
        elif msgid == 253 and len(pl) >= 2:
            txt = pl[1:].split(b"\x00", 1)[0].decode("utf-8", errors="replace").strip()
            if txt:
                self.state["last_statustext"] = txt

    def pack_sideband_telemetry(self) -> bytes:
        s = self.state
        now = int(time.time())
        vb_v = s["vbatt_mv"] / 1000.0
        # Estimate 4S LiPo percentage (13.2V = 0%, 16.8V = 100%)
        pct = max(0.0, min(100.0, (vb_v - 13.2) / (16.8 - 13.2) * 100.0)) if vb_v > 5.0 else 0.0
        info_str = (
            f"TMotorH743 ArduCopter | Mode: {s['mode_name']} ({s['custom_mode']}) | "
            f"Armed: {'YES' if s['armed'] else 'NO (SAFE)'} | "
            f"Vbatt: {vb_v:.2f} V ({s['vbatt_mv']} mV) | "
            f"Attitude: R={s['roll_deg']:+.1f} deg P={s['pitch_deg']:+.1f} deg Y={s['yaw_deg']:.1f} deg | "
            f"RC: {s['rc_ch'][0]}/{s['rc_ch'][1]}/{s['rc_ch'][2]}/{s['rc_ch'][3]} us"
        )
        packed = {
            0x01: now,  # SID_TIME
            0x02: [     # SID_LOCATION
                struct.pack("!i", int(round(s["lat_deg"], 6) * 1e6)),
                struct.pack("!i", int(round(s["lon_deg"], 6) * 1e6)),
                struct.pack("!i", int(round(s["alt_msl_m"], 2) * 1e2)),
                struct.pack("!I", 0),
                struct.pack("!i", int(round(s["yaw_deg"], 2) * 1e2)),
                struct.pack("!H", 150),
                now,
            ],
            0x03: round(s["press_hpa"], 2),                          # SID_PRESSURE
            0x04: [round(pct, 1), False, round(s["temp_c"], 1)],     # SID_BATTERY
            0x07: round(s["temp_c"], 2),                             # SID_TEMPERATURE
            0x0F: info_str,                                          # SID_INFORMATION
        }
        return msgpack.packb(packed)

    def format_summary(self, prefix: str = "TMotorH743 Live Telemetry") -> str:
        s = self.state
        vb_v = s["vbatt_mv"] / 1000.0
        return (
            f"[{prefix}]\n"
            f"• Mode: {s['mode_name']} ({s['custom_mode']}) | Armed: {'YES' if s['armed'] else 'NO'}\n"
            f"• Battery: {vb_v:.2f} V ({s['vbatt_mv']} mV)\n"
            f"• Baro: {s['press_hpa']:.2f} hPa | Temp: {s['temp_c']:.1f} °C | Alt: {s['alt_rel_m']:+.2f} m\n"
            f"• Attitude: Roll {s['roll_deg']:+.1f}° Pitch {s['pitch_deg']:+.1f}° Yaw {s['yaw_deg']:.1f}°\n"
            f"• RC Ch1-4: {s['rc_ch'][0]} / {s['rc_ch'][1]} / {s['rc_ch'][2]} / {s['rc_ch'][3]} us\n"
            f"• MAVLink RX frames: {s['rx_frames']}"
        )


def run_ground_station(rnode_port: str, net_dev: str, tcp_port: int, duration: float) -> int:
    cfg_dir = "/tmp/rns_sideband_ground"
    os.makedirs(cfg_dir, exist_ok=True)
    with open(os.path.join(cfg_dir, "config"), "w") as f:
        f.write(f"""[reticulum]
  enable_transport = Yes
  share_instance = No
  respond_to_probes = Yes

[logging]
  loglevel = 5

[interfaces]
  [[Sideband LAN AutoInterface]]
    type = AutoInterface
    interface_enabled = True
    devices = {net_dev}

  [[Sideband TCP Server]]
    type = TCPServerInterface
    interface_enabled = True
    listen_ip = 0.0.0.0
    listen_port = {tcp_port}

  [[Ground LR2021 RNode 867.2MHz]]
    type = RNodeInterface
    interface_enabled = True
    port = {rnode_port}
    frequency = 867200000
    bandwidth = 125000
    txpower = 14
    spreadingfactor = 7
    codingrate = 5
    flow_control = True
""")
    print(f"[GROUND] Starting Reticulum Transport bridging {net_dev} + TCP:{tcp_port} <-> {rnode_port} (867.2 MHz)...")
    RNS.Reticulum(configdir=cfg_dir, loglevel=RNS.LOG_DEBUG)
    start = time.time()
    try:
        while duration <= 0 or (time.time() - start) < duration:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    return 0


def run_airborne_drone(rnode_port: str, fc_port: str, announce_interval: float, duration: float) -> int:
    cfg_dir = "/tmp/rns_sideband_drone"
    os.makedirs(cfg_dir, exist_ok=True)
    with open(os.path.join(cfg_dir, "config"), "w") as f:
        f.write(f"""[reticulum]
  enable_transport = No
  share_instance = No
  respond_to_probes = Yes

[logging]
  loglevel = 5

[interfaces]
  [[Airborne LR2021 RNode 867.2MHz]]
    type = RNodeInterface
    interface_enabled = True
    port = {rnode_port}
    frequency = 867200000
    bandwidth = 125000
    txpower = 14
    spreadingfactor = 7
    codingrate = 5
    flow_control = True
""")
    print(f"[DRONE] Connecting to Flight Controller on {fc_port}...")
    fc = MavlinkFCBridge(port=fc_port, baud=115200)
    time.sleep(1.2)
    print(fc.format_summary("DRONE FC INIT"))

    print(f"[DRONE] Starting Reticulum (867.2 MHz RNodeInterface ONLY on {rnode_port})...")
    RNS.Reticulum(configdir=cfg_dir, loglevel=RNS.LOG_DEBUG)

    storage_dir = os.path.join(cfg_dir, "lxmf_storage")
    os.makedirs(storage_dir, exist_ok=True)
    id_path = os.path.join(cfg_dir, "drone_identity")
    if os.path.exists(id_path):
        drone_id = RNS.Identity.from_file(id_path)
    else:
        drone_id = RNS.Identity()
        drone_id.to_file(id_path)

    router = LXMF.LXMRouter(identity=drone_id, storagepath=storage_dir)
    drone_dest = router.register_delivery_identity(
        drone_id,
        display_name="LR2021 Drone (TMotorH743)",
        stamp_cost=0,
    )
    drone_hash_hex = RNS.hexrep(drone_dest.hash, delimit=False)
    print(f"[DRONE] LXMF Address: <{drone_hash_hex}> ('LR2021 Drone (TMotorH743)')")

    def send_reply(source_hash: bytes, text: str) -> None:
        dest_id = RNS.Identity.recall(source_hash)
        if dest_id is None:
            RNS.Transport.request_path(source_hash)
            for _ in range(25):
                time.sleep(0.2)
                dest_id = RNS.Identity.recall(source_hash)
                if dest_id is not None:
                    break
        if dest_id is None:
            print(f"[DRONE] Cannot recall identity for {RNS.prettyhexrep(source_hash)}")
            return

        out_dest = RNS.Destination(
            dest_id, RNS.Destination.OUT, RNS.Destination.SINGLE, "lxmf", "delivery"
        )
        fields = {
            LXMF.FIELD_TELEMETRY: fc.pack_sideband_telemetry(),
            LXMF.FIELD_ICON_APPEARANCE: [
                "quadcopter",
                bytes([255, 255, 255]),
                bytes([0, 120, 215]),
            ],
        }
        # Fit within single opportunistically-encrypted packet if small, or direct link
        method = LXMF.LXMessage.OPPORTUNISTIC if len(text) < 160 else LXMF.LXMessage.DIRECT
        lxm = LXMF.LXMessage(
            out_dest,
            drone_dest,
            text,
            title="TMotorH743",
            desired_method=method,
            fields=fields,
        )
        print(f"[DRONE] Sending LXMF reply ({method}) to {RNS.prettyhexrep(source_hash)}: {text.splitlines()[0]}")
        router.handle_outbound(lxm)

    def on_lxmf_delivery(lxm: LXMF.LXMessage) -> None:
        src_hex = RNS.prettyhexrep(lxm.source_hash)
        content = ""
        try:
            content = lxm.content_as_string() or ""
        except Exception:
            pass
        print(
            f"[DRONE] RX LXMF from {src_hex} (RSSI={lxm.rssi} dBm, SNR={lxm.snr} dB): "
            f"content={content!r} fields={list(lxm.fields.keys()) if lxm.fields else []}"
        )

        # Check for Sideband TELEMETRY_REQUEST (0x01) or PING (0x02) in FIELD_COMMANDS
        if lxm.fields and LXMF.FIELD_COMMANDS in lxm.fields and not content.strip():
            for cmd_dict in lxm.fields[LXMF.FIELD_COMMANDS]:
                if isinstance(cmd_dict, dict) and 0x01 in cmd_dict:
                    print(f"[DRONE] Answering Sideband TELEMETRY_REQUEST from {src_hex}")
                    threading.Thread(
                        target=send_reply,
                        args=(lxm.source_hash, fc.format_summary("Telemetry Update")),
                        daemon=True,
                    ).start()
                    return

        cmdline = content.strip()
        parts = cmdline.split()
        verb = parts[0].lower() if parts else "status"

        if verb in ("status", "tlm", "telemetry", "ping", "help"):
            reply = fc.format_summary("TMotorH743 Status")
        elif verb == "loiter":
            t0 = time.time()
            fc.send_command_long(17, 0, 0, 0, 0, 0, 0, 0)  # MAV_CMD_NAV_LOITER_UNLIM = 17
            time.sleep(0.35)
            ack = fc.state["last_ack_res"] if fc.state["last_ack_time"] >= t0 else None
            reply = f"EXEC LOITER -> ACK={ack}\n" + fc.format_summary("After LOITER")
        elif verb == "rtl":
            t0 = time.time()
            fc.send_command_long(20, 0, 0, 0, 0, 0, 0, 0)  # MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
            time.sleep(0.35)
            ack = fc.state["last_ack_res"] if fc.state["last_ack_time"] >= t0 else None
            reply = f"EXEC RTL -> ACK={ack}\n" + fc.format_summary("After RTL")
        elif verb == "land":
            t0 = time.time()
            fc.send_command_long(21, 0, 0, 0, 0, 0, 0, 0)  # MAV_CMD_NAV_LAND = 21
            time.sleep(0.35)
            ack = fc.state["last_ack_res"] if fc.state["last_ack_time"] >= t0 else None
            reply = f"EXEC LAND -> ACK={ack}\n" + fc.format_summary("After LAND")
        elif verb == "mode" and len(parts) >= 2:
            arg = parts[1].lower()
            mnum = int(arg) if arg.isdigit() else MODE_NAME_TO_NUM.get(arg, 0)
            t0 = time.time()
            fc.send_command_long(176, 1.0, float(mnum), 0, 0, 0, 0, 0)
            time.sleep(0.35)
            ack = fc.state["last_ack_res"] if fc.state["last_ack_time"] >= t0 else None
            reply = f"EXEC MODE {mnum} -> ACK={ack}\n" + fc.format_summary(f"After MODE {mnum}")
        elif verb == "goto" and len(parts) >= 4:
            lat = float(parts[1])
            lon = float(parts[2])
            alt = float(parts[3])
            t0 = time.time()
            fc.send_guided_goto(lat, lon, alt)
            time.sleep(0.35)
            mack = fc.state["last_mission_ack"] if fc.state["last_mission_ack_time"] >= t0 else None
            reply = (
                f"EXEC GOTO ({lat:.5f}, {lon:.5f}, {alt:.1f}m) -> MISSION_ACK={mack}\n"
                + fc.format_summary("After GOTO")
            )
        elif verb == "rc" and len(parts) >= 5:
            c1, c2, c3, c4 = (int(parts[1]), int(parts[2]), int(parts[3]), int(parts[4]))
            fc.send_rc_override(c1, c2, c3, c4)
            time.sleep(0.35)
            reply = (
                f"EXEC RC {c1} {c2} {c3} {c4}\n"
                + fc.format_summary("After RC Override")
            )
        else:
            reply = (
                f"Received: {cmdline}\n"
                f"Commands: status | loiter | rtl | land | mode <n> | goto <lat> <lon> <alt> | rc <c1> <c2> <c3> <c4>\n"
                + fc.format_summary("Current Telemetry")
            )

        threading.Thread(target=send_reply, args=(lxm.source_hash, reply), daemon=True).start()

    router.register_delivery_callback(on_lxmf_delivery)

    last_ann = 0.0
    start = time.time()
    try:
        while duration <= 0 or (time.time() - start) < duration:
            now = time.time()
            if announce_interval > 0 and (now - last_ann) >= announce_interval:
                print(f"[DRONE] Announcing 'LR2021 Drone (TMotorH743)' <{drone_hash_hex}> over 867.2 MHz LoRa RF...")
                router.announce(drone_dest.hash)
                last_ann = now
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        fc.close()
    return 0


def run_rnode_tcp_server(serial_port: str, baud: int, listen_port: int) -> int:
    """Expose a local RNode serial port over TCP (default port 7633) for Sideband Hardware -> RNode."""
    ser = serial.Serial(serial_port, baud, timeout=0.02)
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", listen_port))
    srv.listen(1)
    print(f"[RNODE-TCP] Listening on 0.0.0.0:{listen_port} <-> {serial_port} @ {baud}")
    try:
        while True:
            conn, addr = srv.accept()
            print(f"[RNODE-TCP] Client connected from {addr}")
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            conn.settimeout(0.02)
            try:
                while True:
                    data = ser.read(512)
                    if data:
                        conn.sendall(data)
                    try:
                        net_in = conn.recv(512)
                        if not net_in:
                            break
                        ser.write(net_in)
                        ser.flush()
                    except socket.timeout:
                        pass
            except Exception as exc:
                print(f"[RNODE-TCP] Connection closed ({exc})")
            finally:
                conn.close()
    finally:
        srv.close()
        ser.close()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Reticulum / Sideband Live Drone Bridge")
    sub = parser.add_subparsers(dest="mode", required=True)

    p_gnd = sub.add_parser("ground", help="Run Ground Station RNode <-> LAN Transport")
    p_gnd.add_argument("--rnode-port", default="/dev/ttyACM5")
    p_gnd.add_argument("--net-dev", default="enp11s0")
    p_gnd.add_argument("--tcp-port", type=int, default=4242)
    p_gnd.add_argument("--duration", type=float, default=0.0)

    p_drn = sub.add_parser("drone", help="Run Airborne Drone RNode + TMotorH743 MAVLink Companion")
    p_drn.add_argument("--rnode-port", default="/dev/ttyACM7")
    p_drn.add_argument("--fc-port", default="/dev/ttyACM4")
    p_drn.add_argument("--announce-interval", type=float, default=15.0)
    p_drn.add_argument("--duration", type=float, default=0.0)

    p_both = sub.add_parser("both", help="Spawn both Ground Station Transport and Airborne Drone Companion")
    p_both.add_argument("--ground-port", default="/dev/ttyACM5")
    p_both.add_argument("--drone-port", default="/dev/ttyACM7")
    p_both.add_argument("--fc-port", default="/dev/ttyACM4")
    p_both.add_argument("--net-dev", default="enp11s0")
    p_both.add_argument("--announce-interval", type=float, default=12.0)
    p_both.add_argument("--duration", type=float, default=60.0)

    p_tcp = sub.add_parser("rnode-tcp", help="Expose local RNode serial port on TCP port 7633 for Sideband")
    p_tcp.add_argument("--rnode-port", default="/dev/ttyACM5")
    p_tcp.add_argument("--baud", type=int, default=115200)
    p_tcp.add_argument("--listen-port", type=int, default=7633)

    args = parser.parse_args()
    if args.mode == "ground":
        return run_ground_station(args.rnode_port, args.net_dev, args.tcp_port, args.duration)
    if args.mode == "drone":
        return run_airborne_drone(args.rnode_port, args.fc_port, args.announce_interval, args.duration)
    if args.mode == "rnode-tcp":
        return run_rnode_tcp_server(args.rnode_port, args.baud, args.listen_port)
    if args.mode == "both":
        p1 = subprocess.Popen(
            [
                sys.executable,
                __file__,
                "ground",
                "--rnode-port",
                args.ground_port,
                "--net-dev",
                args.net_dev,
                "--duration",
                str(args.duration),
            ]
        )
        time.sleep(5.0)
        p2 = subprocess.Popen(
            [
                sys.executable,
                __file__,
                "drone",
                "--rnode-port",
                args.drone_port,
                "--fc-port",
                args.fc_port,
                "--announce-interval",
                str(args.announce_interval),
                "--duration",
                str(max(1.0, args.duration - 3.5) if args.duration > 0 else 0.0),
            ]
        )
        try:
            p2.wait()
            p1.wait()
        except KeyboardInterrupt:
            p1.terminate()
            p2.terminate()
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main())
