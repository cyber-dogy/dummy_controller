#!/usr/bin/env python3
"""
一键在线调整 Dummy V2 关节 DCE PID 参数
通过串口向主控或各关节驱动板发送命令

用法:
    python3 scripts/tune_pid.py /dev/ttyACM2
"""

import sys
import serial
import time


def tune_joints(port: str):
    ser = serial.Serial(port, 115200, timeout=2)
    time.sleep(0.3)

    print(f"[*] 连接串口: {port}")

    # 使能电机（确保主控处于可通信状态）
    ser.write(b'!START\n')
    time.sleep(0.5)
    ser.read(ser.in_waiting)

    # J1/J2/J3 参数
    joints = [1, 2, 3]
    kp, kd, kv = 2000, 300, 120

    for j in joints:
        print(f"[*] 设置 J{j}: Kp={kp}, Kd={kd}, Kv={kv}")
        ser.write(f'#SET_DCE_KP {j} {kp}\n'.encode())
        time.sleep(0.15)
        ser.read(ser.in_waiting)

        ser.write(f'#SET_DCE_KD {j} {kd}\n'.encode())
        time.sleep(0.15)
        ser.read(ser.in_waiting)

        ser.write(f'#SET_DCE_KV {j} {kv}\n'.encode())
        time.sleep(0.15)
        ser.read(ser.in_waiting)

    # 重启各关节驱动板使参数生效
    for j in joints:
        print(f"[*] 重启 J{j} 驱动板...")
        ser.write(f'#REBOOT {j}\n'.encode())
        time.sleep(1.5)  # 等待驱动板重启完成
        ser.read(ser.in_waiting)

    ser.close()
    print("[+] 完成！J1/J2/J3 PID 已更新并重启生效。")


if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM2"
    tune_joints(port)
