#!/usr/bin/env python3
"""
Dummy V2 机械臂零点校准脚本

校准原理：
    1. 先让电机失能，手动将机械臂摆到 L-Pose
    2. 通过 odrivetool 调用主控的 calibrate_home_offset()
    3. 主控自动执行：apply_home_offset -> 回 Resting -> 再次 apply_home_offset -> 重启

用法：
    python3 scripts/calibrate_home.py /dev/ttyACM2
"""

import sys
import serial
import time


def disable_motors(port: str):
    """通过串口禁用电机，便于手动拖动"""
    ser = serial.Serial(port, 115200, timeout=1)
    time.sleep(0.2)
    ser.write(b'!DISABLE\n')
    time.sleep(0.3)
    resp = ser.read(ser.in_waiting).decode()
    ser.close()
    print(f"  电机已失能，响应: {resp.strip() or '无'}")


def calibrate_via_odrive():
    """通过 odrivetool 调用校准函数"""
    import odrive
    from odrive.utils import dump_errors

    print("[*] 正在通过 odrivetool 连接 Dummy V2 主控...")
    print("    (请确保 USB 线已连接，且没有被其他程序占用)")

    try:
        odrv = odrive.find_any(timeout=10)
        print("[+] 已连接到主控板")
    except Exception as e:
        print(f"[-] 连接失败: {e}")
        return False

    print("[*] 开始执行 calibrate_home_offset()...")
    print("    主控将自动：使能 -> 降电流 -> 第一次应用零点 -> 回 Resting -> 第二次应用零点 -> 重启")

    try:
        odrv.calibrate_home_offset()
    except Exception as e:
        print(f"[-] 调用校准函数失败: {e}")
        return False

    print("[+] 校准指令已发送，主控正在自动处理并重启...")
    print("    请等待约 5~10 秒，直到主控板重新上线")
    return True


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM2"

    print("=" * 60)
    print("Dummy V2 机械臂零点校准")
    print("=" * 60)

    print(f"\n[1/4] 通过串口 {port} 失能电机，以便手动拖动...")
    disable_motors(port)

    print("\n[2/4] 请手动将机械臂摆到 L-Pose：")
    print("      J1 = 0°")
    print("      J2 = 0°")
    print("      J3 = 90°")
    print("      J4 = 0°")
    print("      J5 = 0°")
    print("      J6 = 0°")
    input("\n    摆好后按 Enter 键继续...")

    print("\n[3/4] 通过 odrivetool 发送校准指令...")
    success = calibrate_via_odrive()

    if not success:
        print("\n[-] 自动校准失败。请尝试手动方式：")
        print("    1. 确保机械臂在 L-Pose")
        print("    2. 在终端运行: odrivetool")
        print("    3. 在 odrivetool 中执行: odrv0.calibrate_home_offset()")
        sys.exit(1)

    print("\n[4/4] 等待主控重启...")
    for i in range(10, 0, -1):
        print(f"    剩余 {i} 秒...", end="\r")
        time.sleep(1)
    print()

    # 尝试重新连接串口验证
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(0.5)
        ser.write(b'#GETJPOS\n')
        time.sleep(0.3)
        resp = ser.read(ser.in_waiting).decode().strip()
        ser.close()
        if resp.startswith("ok"):
            print(f"[+] 串口已恢复，当前位置: {resp}")
        else:
            print(f"[!] 串口有响应但可能尚未完全初始化: {resp}")
    except Exception as e:
        print(f"[!] 串口验证失败（可能还在重启中）: {e}")

    print("\n[+] 校准流程已完成！")
    print("    建议重新上电或执行 !START 使能电机后再操作。")


if __name__ == "__main__":
    main()
