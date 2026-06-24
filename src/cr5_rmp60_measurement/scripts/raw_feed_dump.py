#!/usr/bin/env python3
"""直连 CR5 30004 端口，dump 原始二进制数据结构。"""
import argparse
import socket
import struct
import time

parser = argparse.ArgumentParser()
parser.add_argument("--ip", default="192.168.5.1", help="CR5 controller IP")
args = parser.parse_args()

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.settimeout(5)
s.connect((args.ip, 30004))
print(f"已连接 {args.ip}:30004")

raw = b""
start = time.time()
while len(raw) < 1440 and time.time() - start < 3:
    try:
        chunk = s.recv(1440 - len(raw))
        if chunk:
            raw += chunk
        else:
            break
    except socket.timeout:
        continue

print(f"收到 {len(raw)} 字节")
print(f"前 64 字节 hex:\n{raw[:64].hex()}")
frame_len = struct.unpack("<H", raw[0:2])[0] if len(raw) >= 2 else None
print(f"\nlen 字段 (uint16 offset 0): {frame_len if frame_len is not None else 'N/A'}")
if len(raw) != 1440 or frame_len != 1440:
    print("警告：未收到完整 1440 字节 feedback 帧，下面字段可能无效")
print(f"digital_input_bits (uint64 offset 8): {struct.unpack('<Q', raw[8:16])[0] if len(raw) >= 16 else 'N/A'}")
print(f"robot_mode (uint64 offset 24): {struct.unpack('<Q', raw[24:32])[0] if len(raw) >= 32 else 'N/A'}")
print(f"controller_timer (uint64 offset 32): {struct.unpack('<Q', raw[32:40])[0] if len(raw) >= 40 else 'N/A'}")
print(f"q_actual[0] (double offset 432): {struct.unpack('<d', raw[432:440])[0] if len(raw) >= 440 else 'N/A'}")

s.close()
