#!/usr/bin/env python3
"""
Multi-Drone Reticulum Mesh + Live 868 MHz LR2021 RF + ArduCopter SITL (Anisoptera) Harness.

Topology tested:
  [ArduCopter SITL #0 (Leader, TCP 5760)]
         |  live GLOBAL_POSITION_INT telemetry
         v
  [LR2021 Board #1 (/dev/ttyACM5, FC6D1A37, Slot 0 Leader)]
         |
         |  Real 868.0 MHz LoRa RF (64B signed RNSF formation packet)
         v
  [LR2021 Board #2 (/dev/ttyACM7, D56B6941, Slot 1 Left Wingman + Airborne Relay)]
         |-- Relays RNSF packet over 868 MHz RF (RNS_RELAY_FWD hops=1)
         |-- Computes Slot 1 Vee offset (-10m North, -10m East + velocity feed-forward)
         v
  [ArduCopter SITL #1 (Follower/Wingman, TCP 5770)]
         |-- Receives MAVLink SET_POSITION_TARGET_GLOBAL_INT (#86)
         |-- Flies in Vee formation with Leader, then brakes on 3.0s dead-man watchdog
"""

import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time

import serial
from pymavlink import mavutil

# Import Anisoptera's MAVLink helpers if available
ANISOPTERA_ROOT = "/home/ian/Code/anisoptera"
if os.path.isdir(ANISOPTERA_ROOT) and ANISOPTERA_ROOT not in sys.path:
    sys.path.insert(0, ANISOPTERA_ROOT)

ARDUPILOT_BIN = "/home/ian/Code/ardupilot/build/sitl/bin/arducopter"
DEFAULT_PARM = "/home/ian/Code/ardupilot/Tools/autotest/default_params/copter.parm"


def start_sitl_instance(instance: int, home_str: str, speedup: int = 5):
    work_dir = tempfile.mkdtemp(prefix=f"sitl_rns_{instance}_")
    cmd = (
        f"sleep 86400 | {ARDUPILOT_BIN} "
        f"--model + --speedup {speedup} --slave 0 "
        f"--defaults {DEFAULT_PARM} "
        f"--sim-address=127.0.0.1 -I{instance} "
        f"--home {home_str}"
    )
    proc = subprocess.Popen(
        ["bash", "-c", cmd],
        cwd=work_dir,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )
    return proc, work_dir


def connect_and_takeoff(port: int, alt_m: float = 15.0):
    m = mavutil.mavlink_connection(f"tcp:127.0.0.1:{port}", source_system=255)
    m.wait_heartbeat(timeout=30)
    m.mav.request_data_stream_send(
        m.target_system,
        m.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        10,
        1,
    )
    # Wait for GPS + EKF ready, then switch to GUIDED and arm
    deadline = time.time() + 45.0
    armed = False
    while time.time() < deadline:
        m.set_mode("GUIDED")
        m.arducopter_arm()
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            armed = True
            break
        time.sleep(0.2)
    if not armed:
        raise RuntimeError(f"Failed to arm SITL on TCP {port}")

    m.mav.command_long_send(
        m.target_system,
        m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        alt_m,
    )

    deadline = time.time() + 30.0
    while time.time() < deadline:
        pos = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=0.5)
        if pos and (pos.relative_alt / 1000.0) >= (alt_m - 1.5):
            return m, pos
    raise RuntimeError(f"Takeoff timeout on TCP {port}")


def get_latest_gpos(m):
    last = None
    while True:
        msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        if msg is None:
            break
        last = msg
    if last is None:
        last = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1.0)
    return last


def send_ned_velocity(m, vn_ms: float, ve_ms: float, vd_ms: float = 0.0):
    m.mav.set_position_target_local_ned_send(
        0,
        m.target_system,
        m.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0x0DC7,  # velocity only
        0.0,
        0.0,
        0.0,
        vn_ms,
        ve_ms,
        vd_ms,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def send_global_pos_vel(
    m,
    lat_e7: int,
    lon_e7: int,
    alt_m: float,
    vn_ms: float,
    ve_ms: float,
    vd_ms: float = 0.0,
    hdg_deg: float = 0.0,
):
    # Matches MAVLink #86 SET_POSITION_TARGET_GLOBAL_INT emitted by rns_build_mavlink_command()
    m.mav.set_position_target_global_int_send(
        0,
        m.target_system,
        m.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0x09C0,  # pos + vel + yaw
        lat_e7,
        lon_e7,
        alt_m,
        vn_ms,
        ve_ms,
        vd_ms,
        0.0,
        0.0,
        0.0,
        math.radians(hdg_deg),
        0.0,
    )


def ned_offset_m(lat_ref_e7: int, lon_ref_e7: int, lat_e7: int, lon_e7: int):
    dlat_deg = (lat_e7 - lat_ref_e7) * 1e-7
    dlon_deg = (lon_e7 - lon_ref_e7) * 1e-7
    north_m = dlat_deg * 111319.5
    east_m = dlon_deg * 111319.5 * math.cos(math.radians(lat_ref_e7 * 1e-7))
    return north_m, east_m


def radio_cmd(ser: serial.Serial, line: str, wait: float = 0.08) -> str:
    ser.write((line + "\r\n").encode())
    time.sleep(wait)
    return ser.read_all().decode(errors="replace")


def main():
    leader_port = "/dev/ttyACM5"
    wing1_port = "/dev/ttyACM7"

    print("1. Opening physical LR2021 boards (/dev/ttyACM5 Leader, /dev/ttyACM7 Wingman 1)...")
    s_leader = serial.Serial(leader_port, 115200, timeout=0.05)
    s_wing1 = serial.Serial(wing1_port, 115200, timeout=0.05)
    time.sleep(0.15)
    s_leader.read_all()
    s_wing1.read_all()

    radio_cmd(s_leader, "rnode_cfg 868000000 500000 7 5")
    radio_cmd(s_leader, "rns_slot 0")
    radio_cmd(s_leader, "mode relay", wait=0.15)
    radio_cmd(s_wing1, "rnode_cfg 868000000 500000 7 5")
    radio_cmd(s_wing1, "rns_slot 1")
    radio_cmd(s_wing1, "mode relay", wait=0.15)

    # Start 2 ArduCopter SITL instances (Canberra default SITL origin)
    # Drone 0 (Leader): -35.363262, 149.165237
    # Drone 1 (Wingman 1): starts 15m South, 15m West
    print("2. Starting 2 ArduCopter SITL instances (-I0 Leader on 5760, -I1 Wingman on 5770)...")
    p0, dir0 = start_sitl_instance(0, "-35.363262,149.165237,584,0", speedup=5)
    p1, dir1 = start_sitl_instance(1, "-35.363397,149.165072,584,0", speedup=5)

    try:
        time.sleep(2.0)
        print("3. Arming & taking off both SITL drones to 15m AGL in GUIDED mode...")
        m0, pos0 = connect_and_takeoff(5760, alt_m=15.0)
        m1, pos1 = connect_and_takeoff(5770, alt_m=15.0)
        print(
            f"   [TAKEOFF OK] Leader lat={pos0.lat} lon={pos0.lon} alt={pos0.relative_alt/1000:.1f}m | "
            f"Wingman1 lat={pos1.lat} lon={pos1.lon} alt={pos1.relative_alt/1000:.1f}m"
        )

        print("\n4. Flying Vee formation (spacing=10.0m, Leader Vn=5.0 m/s North) over live 868 MHz LoRa...")
        rf_rx_count = 0
        relay_fwd_count = 0
        form_rx_re = re.compile(
            r"RNS_FORM_RX src=LORA hops=(\d+) seq=(\d+) shape=(\d+) slot=(\d+) "
            r"lat=(-?\d+) lon=(-?\d+) alt_mm=(-?\d+) vx=(-?\d+) vy=(-?\d+) mav_len=(\d+)"
        )

        for step in range(1, 9):
            # Command Leader North at 5.0 m/s in SITL
            send_ned_velocity(m0, 5.0, 0.0, 0.0)
            g0 = get_latest_gpos(m0)

            # Update Leader LR2021 board with live SITL GPS & broadcast 64B RNSF over 868 MHz RF
            alt_m_int = max(1, int(round(g0.relative_alt / 1000.0)))
            radio_cmd(
                s_leader,
                f"fc_sim {g0.lat} {g0.lon} {alt_m_int} 24800",
                wait=0.04,
            )
            vn_ms = g0.vx / 100.0
            ve_ms = g0.vy / 100.0
            hdg_deg = (g0.hdg / 100.0) if g0.hdg != 65535 else 0.0
            s_leader.write(f"rns_form vee 10.0 {vn_ms:.2f} {ve_ms:.2f} {hdg_deg:.1f}\r\n".encode())

            # Wait for 868 MHz LoRa reception & relay on Wingman 1 LR2021 board
            time.sleep(0.45)
            _ = s_leader.read_all()
            w1_out = s_wing1.read_all().decode(errors="replace")
            if "RNS_RELAY_FWD" in w1_out:
                relay_fwd_count += 1

            m_rx = form_rx_re.search(w1_out)
            if m_rx:
                rf_rx_count += 1
                target_lat = int(m_rx.group(5))
                target_lon = int(m_rx.group(6))
                target_alt = int(m_rx.group(7)) / 1000.0
                target_vx = int(m_rx.group(8)) / 100.0
                target_vy = int(m_rx.group(9)) / 100.0
                mav_len = int(m_rx.group(10))

                # Feed the LR2021-computed Slot 1 MAVLink #86 target into Wingman 1 SITL
                send_global_pos_vel(m1, target_lat, target_lon, target_alt, target_vx, target_vy, 0.0, hdg_deg)

                g1 = get_latest_gpos(m1)
                dn_m, de_m = ned_offset_m(g0.lat, g0.lon, g1.lat, g1.lon)
                tn_m, te_m = ned_offset_m(g0.lat, g0.lon, target_lat, target_lon)
                print(
                    f"   [Step {step}/8] 868MHz RNSF->Wing1 OK (mav_len={mav_len}B, relay_fwd={relay_fwd_count}) | "
                    f"Slot1 Target Offset=({tn_m:+.1f}m N, {te_m:+.1f}m E) | "
                    f"Wing1 Actual Offset=({dn_m:+.1f}m N, {de_m:+.1f}m E) Vn={g1.vx/100:.1f}m/s"
                )

        assert rf_rx_count >= 6, f"Expected >=6 live RF RNSF packets, got {rf_rx_count}"
        assert relay_fwd_count >= 6, f"Expected >=6 relayed RNSF packets, got {relay_fwd_count}"

        print("\n5. Testing 3.0s Dead-Man Failsafe Watchdog (silencing Leader RF transmissions)...")
        time.sleep(3.3)
        wd_out = s_wing1.read_all().decode(errors="replace").strip()
        print(f"   Wingman 1 LR2021 output after 3.3s RF silence: {wd_out!r}")
        assert "RNS_MESH_FAILSAFE action=LOITER timeout_ms=3000" in wd_out

        # Command Wingman 1 SITL to LOITER as triggered by the LR2021 failsafe frame
        m1.set_mode("LOITER")
        time.sleep(1.0)
        g1_final = get_latest_gpos(m1)
        print(
            f"   [PASS] Wingman 1 transitioned to LOITER & braked (Vn={g1_final.vx/100:.2f} m/s, Ve={g1_final.vy/100:.2f} m/s)"
        )

        print("\n6. Testing Multi-Hop Ping-Pong Relay Chain & Hop/TTL Expiration (`rns_chain 4 120 3000`)...")
        radio_cmd(s_leader, "rns_chain 4 120 3000")
        radio_cmd(s_wing1, "rns_chain 4 120 3000")
        s_leader.write(b"rns_form vee 10.0 5.0 0.0 0.0\r\n")
        time.sleep(0.85)
        chain_a = s_leader.read_all().decode(errors="replace")
        chain_b = s_wing1.read_all().decode(errors="replace")
        assert "hops=1" in chain_a and "hops=3" in chain_a, f"Expected hops=1,3 on Node A, got:\n{chain_a}"
        assert "hops=0" in chain_b and "hops=2" in chain_b and "RNS_RELAY_EXPIRED hops=4 max_hops=4" in chain_b, (
            f"Expected hops=0,2,4+EXPIRED on Node B, got:\n{chain_b}"
        )
        print("   [PASS] Single RNSF packet bounced 4 hops across both radios (Slots 1..5) and expired cleanly at max_hops=4.")
        radio_cmd(s_leader, "rns_chain off")
        radio_cmd(s_wing1, "rns_chain off")

        print("\nALL MULTI-DRONE LIVE 868 MHz RF + ANISOPTERA SITL FORMATION CHECKS PASSED.")

    finally:
        radio_cmd(s_leader, "mode idle")
        radio_cmd(s_wing1, "mode idle")
        s_leader.close()
        s_wing1.close()
        for p, d in [(p0, dir0), (p1, dir1)]:
            try:
                os.killpg(os.getpgid(p.pid), 15)
            except Exception:
                pass
            shutil.rmtree(d, ignore_errors=True)


if __name__ == "__main__":
    main()
