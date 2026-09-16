#!/usr/bin/env python3
"""
demo_thumb_poses.py — Standalone Diagnostic & Motion Demo for Allegro Hand V6 Thumb (Left Hand).
Tests multiple calibrated poses demonstrating full thumb opposition, elevation, and curling.
"""
import socket
import struct
import time
import sys

def main():
    print("=========================================================")
    print("   Allegro Hand V6 (Left Hand) Thumb Motion Demo         ")
    print("   Hardware Target: 192.168.1.100:502 (Modbus TCP)       ")
    print("=========================================================")

    ip = "192.168.1.100"
    port = 502

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(3.0)
    try:
        s.connect((ip, port))
    except Exception as e:
        print(f"[ERROR] Connection failed: {e}")
        return 1

    def write_coil(addr, val):
        s.sendall(struct.pack(">HHHBBHH", 1, 0, 6, 1, 5, addr, 0xFF00 if val else 0x0000))
        s.recv(1024)

    def write_regs(addr, vals):
        count = len(vals)
        byte_cnt = count * 2
        header = struct.pack(">HHHBBHHB", 1, 0, 7 + byte_cnt, 1, 16, addr, count, byte_cnt)
        payload = struct.pack(f">{count}h", *vals)
        s.sendall(header + payload)
        s.recv(1024)

    def read_feedback():
        s.sendall(struct.pack(">HHHBBHH", 1, 0, 6, 1, 3, 0x0000, 40))
        resp = s.recv(1024)
        if len(resp) >= 9 + 80:
            pos = list(struct.unpack(">20h", resp[9:49]))
            cur = list(struct.unpack(">20h", resp[49:89]))
            return pos, cur
        return None, None

    def deg2pul(deg_list):
        return [int(d * 4096.0 / 360.0) for d in deg_list]

    p0, c0 = read_feedback()
    print(f"Initial Thumb Pos: {[round(x*360/4096, 1) for x in p0[0:4]]}°")

    # Step 1: Position Mode
    write_regs(0x0128, [5])
    time.sleep(0.05)

    # Step 2: Torque ON
    print("[1] Torque ON...")
    write_coil(0x0000, True)
    time.sleep(0.1)

    # Step 3: Apply Current Limits (250 mA)
    print("[2] Applying Current Limits (250 mA)...")
    write_regs(0x0114, [250] * 20)
    time.sleep(0.1)

    # Poses designed for Left Hand kinematics:
    # [J00 (Opposition), J01 (Elevation), J02 (MCP Flex), J03 (IP Curl)]
    poses = [
        ("1. Zero Reference (Open Flat)",      [  0,   0,   0,   0], 1.0),
        ("2. Thumb Elevation (Swing Forward)", [  0,  40,   0,   0], 1.2),
        ("3. Thumb Tip Flexion (MCP+IP Curl)", [  0,  40, -35, -40], 1.5),
        ("4. Opposition Sweep (Across Palm)",  [-55,  35, -35, -40], 1.5),
        ("5. Deep Opposition Grip",            [-70,  20, -50, -50], 1.5),
        ("6. Return to Open Flat",             [  0,   0,   0,   0], 1.2),
    ]

    cmd = [0] * 20
    # Hold 4 other fingers in a gentle open posture
    for f in range(1, 5):
        cmd[f*4 + 0] = 0
        cmd[f*4 + 1] = int(20 * 4096 / 360)
        cmd[f*4 + 2] = int(20 * 4096 / 360)
        cmd[f*4 + 3] = int(20 * 4096 / 360)

    print("\n--- Starting Thumb Motion Demonstration ---")
    for name, thumb_deg, duration in poses:
        pulses = deg2pul(thumb_deg)
        cmd[0:4] = pulses
        write_regs(0x0100, cmd)
        time.sleep(duration)
        p, c = read_feedback()
        actual_deg = [round(x * 360 / 4096.0, 1) for x in p[0:4]]
        err = [round(actual_deg[i] - thumb_deg[i], 1) for i in range(4)]
        print(f"[{name}]")
        print(f"   Target: {thumb_deg} deg")
        print(f"   Actual: {actual_deg} deg (err={err}) | Cur: {c[0:4]} mA\n")

    # Step 4: Graceful torque off
    print("[3] Turning Torque OFF...")
    write_coil(0x0000, False)
    s.close()
    print("[✓] Thumb motion demonstration completed successfully!")
    return 0

if __name__ == "__main__":
    sys.exit(main())
