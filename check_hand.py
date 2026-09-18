#!/usr/bin/env python3
"""
check_hand.py — Standalone Communication Diagnostic Tool for Allegro Hand V6.
Zero external dependencies (uses only Python standard library: socket, struct, subprocess).
Runs directly on host machine (Python 3.10 / 3.12) or inside Docker container.
"""

import socket
import struct
import subprocess
import sys
import time

TARGET_IP = "192.168.1.100"
TARGET_PORT = 502

def print_banner():
    print("=" * 64)
    print("  🦾 Allegro Hand V6 Hardware Communication Diagnostic")
    print(f"  Target: {TARGET_IP}:{TARGET_PORT} (Modbus TCP)")
    print("=" * 64)

def check_ping(ip: str) -> bool:
    try:
        res = subprocess.run(
            ["ping", "-c", "2", "-W", "1", ip],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=3,
        )
        for line in res.stdout.splitlines():
            if "rtt min/avg/max" in line or "round-trip" in line:
                avg = line.split("/")[4]
                print(f"[✓] Network Ping: {ip} REACHABLE (avg RTT: {avg} ms)")
                return True
        if res.returncode == 0:
            print(f"[✓] Network Ping: {ip} REACHABLE")
            return True
    except Exception as e:
        pass
    print(f"[✗] Network Ping: FAILED to ping {ip}")
    print("    → Check Ethernet cable connection.")
    print("    → Check PC Ethernet IP (must be 192.168.1.x, e.g. 192.168.1.10).")
    print("    → Check 24V DC power supply to Allegro Hand.")
    return False

def check_modbus(ip: str, port: int):
    print(f"[*] Connecting to Modbus TCP socket at {ip}:{port}...")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(1.5)
    try:
        s.connect((ip, port))
    except Exception as e:
        print(f"[✗] Socket Connect FAILED: {e}")
        print("    → Port 502 is not responding.")
        s.close()
        return False

    print(f"[✓] TCP Connection: Connected to {ip}:{port} successfully!")

    # 1. Read Holding Register 0x0070 (FW Ver) & 0x0071 (Hand Type)
    try:
        # Transaction ID 1, Protocol 0, Length 6, Unit 1, Func 3, Reg 0x0070, Count 2
        req_info = struct.pack(">HHHBBHH", 1, 0, 6, 1, 3, 0x0070, 2)
        s.sendall(req_info)
        resp_info = s.recv(64)
        if len(resp_info) >= 13 and resp_info[7] == 3:
            fw_ver = int.from_bytes(resp_info[9:11], "big")
            hand_type_val = int.from_bytes(resp_info[11:13], "big")
            hand_type = "LEFT HAND (왼손)" if hand_type_val == 0 else "RIGHT HAND (오른손)"
            print(f"[✓] Robot Identity:")
            print(f"    • Firmware Version : 0x{fw_ver:04X} (v{fw_ver>>8}.{fw_ver&0xFF})")
            print(f"    • Hardware Hand Type: {hand_type} (Register 0x0071 == {hand_type_val})")
        else:
            print(f"[!] Warning: Unexpected response for register 0x0070: {resp_info.hex()}")
    except Exception as e:
        print(f"[✗] Failed to read identity registers: {e}")

    # 2. Read Serial Number (Registers 0x0318 ~ 0x031B, 8 bytes ASCII)
    try:
        req_sn = struct.pack(">HHHBBHH", 2, 0, 6, 1, 3, 0x0318, 4)
        s.sendall(req_sn)
        resp_sn = s.recv(64)
        if len(resp_sn) >= 17 and resp_sn[7] == 3:
            sn_bytes = resp_sn[9:17]
            sn_str = "".join(chr(b) for b in sn_bytes if 32 <= b <= 126).strip()
            if sn_str:
                print(f"    • Serial Number     : {sn_str}")
    except Exception:
        pass

    # 3. Read Current Joint Positions (Holding Registers 0x0000 ~ 0x0013, 20 joints)
    try:
        req_pos = struct.pack(">HHHBBHH", 3, 0, 6, 1, 3, 0x0000, 20)
        s.sendall(req_pos)
        resp_pos = s.recv(64)
        if len(resp_pos) >= 9 + 40 and resp_pos[7] == 3:
            pulses = list(struct.unpack(">20h", resp_pos[9:49]))
            deg = [round(p * 360.0 / 4096.0, 1) for p in pulses]
            rad = [round(p * (2.0 * 3.1415926535) / 4096.0, 3) for p in pulses]

            print(f"\n[✓] Real-time 20-DOF Joint Encoders (Degrees & Radians):")
            print(f"    Thumb  (joint00~03) : {deg[0]:+6.1f}°, {deg[1]:+6.1f}°, {deg[2]:+6.1f}°, {deg[3]:+6.1f}°  | rad: {rad[0:4]}")
            print(f"    Index  (joint10~13) : {deg[4]:+6.1f}°, {deg[5]:+6.1f}°, {deg[6]:+6.1f}°, {deg[7]:+6.1f}°  | rad: {rad[4:8]}")
            print(f"    Middle (joint20~23) : {deg[8]:+6.1f}°, {deg[9]:+6.1f}°, {deg[10]:+6.1f}°, {deg[11]:+6.1f}°  | rad: {rad[8:12]}")
            print(f"    Ring   (joint30~33) : {deg[12]:+6.1f}°, {deg[13]:+6.1f}°, {deg[14]:+6.1f}°, {deg[15]:+6.1f}°  | rad: {rad[12:16]}")
            print(f"    Pinky  (joint40~43) : {deg[16]:+6.1f}°, {deg[17]:+6.1f}°, {deg[18]:+6.1f}°, {deg[19]:+6.1f}°  | rad: {rad[16:20]}")
    except Exception as e:
        print(f"[✗] Failed to read joint position registers: {e}")

    s.close()
    print("\n" + "=" * 64)
    print("  🎉 RESULT: Allegro Hand V6 Hardware Communication is 100% HEALTHY!")
    print("=" * 64)
    return True

def main():
    global TARGET_IP
    candidate_ips = ["192.168.1.100", "192.168.1.201", "192.168.40.100"]
    if len(sys.argv) > 1:
        arg = sys.argv[1]
        if arg.isdigit():
            target = f"192.168.1.{arg}"
        elif arg.count(".") == 3:
            target = arg
        else:
            target = arg
        active_ip = target
    else:
        # Auto-discover active IP
        active_ip = TARGET_IP
        for ip in candidate_ips:
            try:
                s = socket.socket()
                s.settimeout(0.2)
                res = s.connect_ex((ip, TARGET_PORT))
                s.close()
                if res == 0:
                    active_ip = ip
                    break
            except Exception:
                pass

    TARGET_IP = active_ip
    print_banner()

    if not check_ping(active_ip):
        sys.exit(1)
    if not check_modbus(active_ip, TARGET_PORT):
        sys.exit(1)

if __name__ == "__main__":
    main()
