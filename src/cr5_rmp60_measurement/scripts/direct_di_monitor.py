#!/usr/bin/env python3
"""直连 CR5 30004 feedback 口监控 DI1。

默认不连接 29999 命令口，避免和 ROS2 驱动/DobotStudio 抢占 dashboard。
"""
import argparse
import re
import socket
import struct
import time


def dashboard_cmd(ip, cmd):
    """Dashboard cmd."""
    with socket.create_connection((ip, 29999), timeout=2.0) as sock:
        sock.sendall((cmd + "\n").encode("ascii"))
        return sock.recv(1024).decode("ascii", errors="replace").strip()


def parse_dashboard_value(reply):
    """Parse dashboard value."""
    match = re.search(r"\{([^}]*)\}", reply)
    if not match:
        return None
    value = match.group(1).split(",", 1)[0].strip()
    try:
        return int(value)
    except ValueError:
        return None


def read_feedback_di1(ip):
    """Read feedback di1."""
    with socket.create_connection((ip, 30004), timeout=2.0) as sock:
        raw = b""
        while len(raw) < 1440:
            chunk = sock.recv(1440 - len(raw))
            if not chunk:
                break
            raw += chunk
    if len(raw) != 1440:
        return None, len(raw), None
    frame_len = struct.unpack("<H", raw[0:2])[0]
    if frame_len != 1440:
        return None, len(raw), frame_len
    di_bits = struct.unpack("<Q", raw[8:16])[0]
    return bool(di_bits & 0x01), len(raw), frame_len


def main():
    """Main."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default="192.168.5.1", help="CR5 controller IP")
    parser.add_argument("--interval", type=float, default=0.2, help="poll interval seconds")
    parser.add_argument("--dashboard-check", action="store_true", help="also query DI(1) on port 29999")
    args = parser.parse_args()

    print(f"monitoring DI1 on {args.ip}; press Ctrl-C to stop")
    last = None
    try:
        while True:
            try:
                fb_di1, fb_bytes, fb_len = read_feedback_di1(args.ip)
                fb_state = "INVALID" if fb_di1 is None else ("ON" if fb_di1 else "OFF")
                parts = [f"feedback={fb_state}", f"bytes={fb_bytes}", f"len={fb_len}"]
                if args.dashboard_check:
                    di_reply = dashboard_cmd(args.ip, "DI(1)")
                    di_value = parse_dashboard_value(di_reply)
                    di_state = "INVALID" if di_value is None else ("ON" if di_value else "OFF")
                    parts.insert(0, f"DI(1)={di_state}")
                    parts.append(f"reply={di_reply}")
                line = " ".join(parts)
            except OSError as exc:
                line = f"socket error: {exc}"
            if line != last:
                print(line, flush=True)
                last = line
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
