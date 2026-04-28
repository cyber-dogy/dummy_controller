#!/usr/bin/env python3
"""
在线调整 Dummy V2 关节 DCE 参数。

说明：
1. 当前主控 ASCII 协议支持在线 `#SET_DCE_*`，但不支持把现有值读回。
2. 因此本脚本只负责“按 profile 下发”，不会声称能读出当前 EEPROM 中的 PID。
3. J2/J3 负载较大，若出现静止抖动，优先降低 Kp，其次再调 Ki/Kd。
4. 当前实机验证较稳的默认档位是 `j23_damped`。

示例：
    python3 scripts/tune_pid.py --port /dev/ttyACM1 --profile j23_stable
    python3 scripts/tune_pid.py --port /dev/ttyACM1 --profile repo_reference --dry-run
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Dict

import serial


PidMap = Dict[int, Dict[str, int]]


PROFILES: Dict[str, PidMap] = {
    # 原脚本的高刚度风格，保留作对照。
    "legacy_aggressive": {
        1: {"kp": 2000, "kv": 120, "ki": 300, "kd": 300},
        2: {"kp": 2000, "kv": 120, "ki": 300, "kd": 300},
        3: {"kp": 2000, "kv": 120, "ki": 300, "kd": 300},
    },
    # 仓库里 dummy_robot.h 给出的参考值，更接近官方整机配置。
    "repo_reference": {
        2: {"kp": 1000, "kv": 80, "ki": 200, "kd": 200},
        3: {"kp": 1500, "kv": 80, "ki": 200, "kd": 250},
    },
    # 面向“J2/J3 静止抖动明显”的保守起步值。
    "j23_stable": {
        2: {"kp": 850, "kv": 80, "ki": 160, "kd": 220},
        3: {"kp": 1200, "kv": 80, "ki": 180, "kd": 240},
    },
    # 在 j23_stable 基础上增加一点阻尼，抑制起步/到位轻抖。
    "j23_damped": {
        2: {"kp": 850, "kv": 80, "ki": 140, "kd": 235},
        3: {"kp": 1200, "kv": 80, "ki": 150, "kd": 265},
    },
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Tune Dummy V2 DCE gains")
    parser.add_argument("--port", default="/dev/ttyACM1", help="主控串口，默认 /dev/ttyACM1")
    parser.add_argument(
        "--profile",
        choices=sorted(PROFILES.keys()),
        default="j23_damped",
        help="参数档位，默认 j23_damped",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只打印准备下发的参数，不实际写入",
    )
    parser.add_argument(
        "--no-reboot",
        action="store_true",
        help="写入后不重启电机板",
    )
    return parser.parse_args()


def send_and_expect(ser: serial.Serial, cmd: str, delay: float = 0.2) -> str:
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode())
    time.sleep(delay)
    return ser.read(ser.in_waiting).decode(errors="ignore").strip()


def apply_profile(port: str, profile_name: str, dry_run: bool, reboot: bool) -> int:
    profile = PROFILES[profile_name]

    print(f"[*] profile = {profile_name}")
    for joint, gains in profile.items():
        print(f"    J{joint}: Kp={gains['kp']} Kv={gains['kv']} Ki={gains['ki']} Kd={gains['kd']}")

    if dry_run:
        print("[+] dry-run 完成，未写入设备。")
        return 0

    ser = serial.Serial(port, 115200, timeout=1)
    time.sleep(0.3)
    print(f"[*] 已连接串口: {port}")

    try:
        print("[*] 发送 !START，确保主控处于可通信状态")
        resp = send_and_expect(ser, "!START", delay=0.4)
        print(f"    -> {resp or '[no response]'}")

        for joint, gains in profile.items():
            for key in ("kp", "kv", "ki", "kd"):
                cmd = f"#SET_DCE_{key.upper()} {joint} {gains[key]}"
                resp = send_and_expect(ser, cmd)
                print(f"    {cmd} -> {resp or '[no response]'}")

        if reboot:
            for joint in profile:
                cmd = f"#REBOOT {joint}"
                resp = send_and_expect(ser, cmd, delay=1.4)
                print(f"    {cmd} -> {resp or '[no response]'}")

        print("[+] 参数写入完成。")
        return 0
    finally:
        ser.close()


def main() -> int:
    args = parse_args()
    reboot = not args.no_reboot
    return apply_profile(args.port, args.profile, args.dry_run, reboot)


if __name__ == "__main__":
    sys.exit(main())
