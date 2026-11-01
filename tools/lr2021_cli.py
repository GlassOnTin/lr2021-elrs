#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-or-later
# AP_FLAKE8_CLEAN
"""Host CLI and bench control utility for Seeed Wio-LR2021 + XIAO nRF54L15 ELRS nodes."""

import argparse
import sys
import time

import serial


class LR2021Node:
    """Serial client for the bare-metal LR2021 ELRS firmware CLI."""

    def __init__(self, port: str, baud: int = 115200, timeout: float = 1.0) -> None:
        self.port = port
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(0.05)
        self.ser.reset_input_buffer()

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()

    def send_cmd(self, cmd: str, wait_s: float = 0.25) -> str:
        self.ser.reset_input_buffer()
        self.ser.write((cmd.strip() + "\r\n").encode("ascii"))
        self.ser.flush()
        deadline = time.monotonic() + wait_s
        chunks = []
        while time.monotonic() < deadline:
            avail = self.ser.in_waiting
            if avail:
                chunks.append(self.ser.read(avail).decode("utf-8", errors="replace"))
            else:
                time.sleep(0.01)
        return "".join(chunks).strip()


def control_sh_ur04a_relay(port: str, channel: int, state: str, cycle_off_s: float = 1.5) -> str:
    """Control a DSD Tech SH-UR04A 4-channel USB relay (9600 8N1, AT+CHn=0|1|?)."""
    if channel < 1 or channel > 4:
        raise ValueError(f"Invalid relay channel {channel}; must be 1..4")
    with serial.Serial(port, 9600, timeout=1.0) as ser:
        time.sleep(0.25)
        ser.reset_input_buffer()

        def at(cmd: str) -> str:
            ser.write((cmd + "\r\n").encode("ascii"))
            ser.flush()
            time.sleep(0.15)
            raw = ser.read(256).replace(b"\x00", b"").decode("ascii", errors="replace").strip()
            return raw

        state_norm = state.lower()
        if state_norm == "status":
            return at(f"AT+CH{channel}=?")
        if state_norm in ("on", "1"):
            return at(f"AT+CH{channel}=1")
        if state_norm in ("off", "0"):
            return at(f"AT+CH{channel}=0")
        if state_norm == "cycle":
            r_off = at(f"AT+CH{channel}=0")
            time.sleep(cycle_off_s)
            r_on = at(f"AT+CH{channel}=1")
            return f"{r_off} -> {r_on}"
        raise ValueError(f"Unknown relay state '{state}'")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Host CLI for LR2021 ELRS TX/RX/Monitor/Bridge firmware and bench power relay"
    )
    parser.add_argument(
        "-p",
        "--port",
        default="/dev/serial/by-id/usb-Seeed_Studio_Seeed_Studio_XIAO_nrf54_CMSIS-DAP_FC6D1A37-if02",
        help="Serial port for LR2021 EVK node",
    )
    parser.add_argument("-b", "--baud", type=int, default=115200, help="Serial baud rate")

    sub = parser.add_subparsers(dest="subcmd", required=True)

    sub.add_parser("status", help="Query radio, FHSS, link, and RC channel status")
    sub.add_parser("rates", help="List all 17 supported 900 MHz, 2.4 GHz, and dual-band air rates")

    p_cfg = sub.add_parser("configure", help="Configure domain, air rate, UID/phrase, power, and mode")
    p_cfg.add_argument("--domain", choices=["fcc915", "au915", "eu868", "ism2g4", "ce2g4"])
    p_cfg.add_argument("--rate", help="Rate index (0..16) or name (e.g. 900_200HZ, 2G4_500HZ)")
    p_cfg.add_argument("--phrase", help="ExpressLRS binding phrase")
    p_cfg.add_argument("--uid", help="Explicit 6-byte UID in XX:XX:XX:XX:XX:XX format")
    p_cfg.add_argument("--power", type=int, help="TX power in dBm (-9..22 LF, -19..12 HF)")
    p_cfg.add_argument("--mode", choices=["idle", "tx", "rx", "monitor", "bridge"])

    p_ping = sub.add_parser("ping", help="Run active RF ping test on 868/900 MHz or 2.4 GHz")
    p_ping.add_argument("--freq", type=int, default=868000000, help="Frequency in Hz")
    p_ping.add_argument("--mod", choices=["lora", "flrc"], default="lora", help="Modulation")
    p_ping.add_argument("--count", type=int, default=5, help="Number of ping packets")

    p_pong = sub.add_parser("pong", help="Enter active RF pong responder mode")
    p_pong.add_argument("--freq", type=int, default=868000000, help="Frequency in Hz")
    p_pong.add_argument("--mod", choices=["lora", "flrc"], default="lora", help="Modulation")

    p_scan = sub.add_parser("scan", help="Run RSSI spectral scan across a frequency band")
    p_scan.add_argument("--start", type=int, default=863000000, help="Start frequency in Hz")
    p_scan.add_argument("--stop", type=int, default=870000000, help="Stop frequency in Hz")
    p_scan.add_argument("--step", type=int, default=250000, help="Frequency step in Hz")

    p_mon = sub.add_parser("monitor", help="Stream live OTA packets in MONITOR/RX/TX mode")
    p_mon.add_argument("--duration", type=float, default=10.0, help="Duration in seconds")

    p_rc = sub.add_parser("rc", help="Set RC channel values in microseconds (1000..2000)")
    p_rc.add_argument("channels", nargs="+", type=int, help="1 to 16 channel values in us")

    p_relay = sub.add_parser("relay", help="Control DSD Tech SH-UR04A USB relay for drone power")
    p_relay.add_argument("--relay-port", default="/dev/ttyUSB0", help="Serial port for SH-UR04A")
    p_relay.add_argument("--ch", type=int, default=1, help="Relay channel (1..4)")
    p_relay.add_argument(
        "--state",
        choices=["on", "off", "cycle", "status"],
        default="status",
        help="Desired relay state",
    )
    p_relay.add_argument("--off-time", type=float, default=1.5, help="Off duration in seconds for cycle")

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    if args.subcmd == "relay":
        out = control_sh_ur04a_relay(args.relay_port, args.ch, args.state, args.off_time)
        print(out)
        return 0

    node = LR2021Node(args.port, args.baud)
    try:
        if args.subcmd == "status":
            print(node.send_cmd("status", wait_s=0.25))
        elif args.subcmd == "rates":
            print(node.send_cmd("rates", wait_s=0.35))
        elif args.subcmd == "configure":
            if args.phrase is not None:
                print(node.send_cmd(f"phrase {args.phrase}", wait_s=0.25))
            if args.uid is not None:
                print(node.send_cmd(f"uid {args.uid}", wait_s=0.25))
            if args.domain is not None:
                print(node.send_cmd(f"domain {args.domain}", wait_s=0.25))
            if args.rate is not None:
                print(node.send_cmd(f"rate {args.rate}", wait_s=0.3))
            if args.power is not None:
                print(node.send_cmd(f"power {args.power}", wait_s=0.2))
            if args.mode is not None:
                print(node.send_cmd(f"mode {args.mode}", wait_s=0.35))
            print(node.send_cmd("status", wait_s=0.25))
        elif args.subcmd == "ping":
            wait_s = max(1.0, args.count * 0.25 + 0.5)
            print(node.send_cmd(f"ping {args.freq} {args.mod} {args.count}", wait_s=wait_s))
        elif args.subcmd == "pong":
            print(node.send_cmd(f"pong {args.freq} {args.mod}", wait_s=0.35))
        elif args.subcmd == "scan":
            steps = max(1, (args.stop - args.start) // max(1, args.step))
            wait_s = max(1.0, steps * 0.03 + 0.5)
            print(node.send_cmd(f"scan {args.start} {args.stop} {args.step}", wait_s=wait_s))
        elif args.subcmd == "rc":
            ch_str = " ".join(str(v) for v in args.channels[:16])
            print(node.send_cmd(f"rc {ch_str}", wait_s=0.2))
        elif args.subcmd == "monitor":
            print(node.send_cmd("mode monitor", wait_s=0.3))
            end_t = time.monotonic() + args.duration
            while time.monotonic() < end_t:
                line = node.ser.readline().decode("utf-8", errors="replace").strip()
                if line:
                    print(line)
    finally:
        node.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
