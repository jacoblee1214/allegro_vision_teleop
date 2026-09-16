#!/usr/bin/env python3
import socket
import struct
import time
import sys

def main():
    print("==================================================")
    print("  Direct Thumb Joint Control & Encoder Test       ")
    print("==================================================")

    ip = "192.168.1.100"
    port = 502

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(3.0)
    s.connect((ip, port))

    def write_coil(addr, val):
        val_bytes = 0xFF00 if val else 0x0000
        req = struct.pack(">HHHBBHH", 1, 0, 6, 1, 5, addr, val_bytes)
        s.sendall(req)
        s.recv(1024)

    def write_regs(addr, vals):
        count = len(vals)
        byte_cnt = count * 2
        header = struct.pack(">HHHBBHHB", 1, 0, 7 + byte_cnt, 1, 16, addr, count, byte_cnt)
        payload = struct.pack(f">{count}h", *vals)
        s.sendall(header + payload)
        s.recv(1024)

    def read_feedback():
        req = struct.pack(">HHHBBHH", 1, 0, 6, 1, 3, 0x0000, 40)
        s.sendall(req)
        resp = s.recv(1024)
        if len(resp) >= 9 + 80:
            pos = list(struct.unpack(">20h", resp[9:49]))
            cur = list(struct.unpack(">20h", resp[49:89]))
            return pos, cur
        return None, None

    # 1. Read initial state
    pos0, cur0 = read_feedback()
    print(f"Initial Thumb Pos (pulses): {pos0[0:4]}")
    print(f"Initial Thumb Pos (deg)   : {[round(x*360/4096, 1) for x in pos0[0:4]]}")

    # 2. Configure Operating Mode = 5 (Position Control)
    write_regs(0x0128, [5])
    time.sleep(0.05)

    # 3. Set Current Limits to 250 mA across all 20 joints
    write_regs(0x0114, [250] * 20)
    time.sleep(0.05)

    # 4. Initialize goal_pos to current positions
    cmd_pos = list(pos0)
    write_regs(0x0100, cmd_pos)
    time.sleep(0.05)

    # 5. Enable Torque
    print("[1] Enabling Torque...")
    write_coil(0x0000, True)
    time.sleep(0.2)

    # Verify hold
    for _ in range(5):
        write_regs(0x0100, cmd_pos)
        p, c = read_feedback()
        time.sleep(0.05)
    print(f"Holding Pose: {p[0:4]} | Cur: {c[0:4]} mA")

    # 6. Test Joint 01 (Elevation/Swing) - move +200 pulses
    print("\n[2] Testing Joint 01 (Swing +200 pulses)...")
    start_j01 = cmd_pos[1]
    for step in range(20):
        cmd_pos[1] = int(start_j01 + (step + 1) * 10)
        write_regs(0x0100, cmd_pos)
        time.sleep(0.05)
    p, c = read_feedback()
    print(f"  Target: {cmd_pos[1]} pulses ({cmd_pos[1]*360/4096:.1f}°), Actual: {p[1]} pulses ({p[1]*360/4096:.1f}°), Cur: {c[1]} mA")

    # Return Joint 01
    for step in range(20):
        cmd_pos[1] = int(cmd_pos[1] - 10)
        write_regs(0x0100, cmd_pos)
        time.sleep(0.05)
    cmd_pos[1] = start_j01

    # 7. Test Joint 02 (Flexion) - move +150 pulses
    print("\n[3] Testing Joint 02 (MCP Flexion +150 pulses)...")
    start_j02 = cmd_pos[2]
    for step in range(20):
        cmd_pos[2] = int(start_j02 + (step + 1) * 7.5)
        write_regs(0x0100, cmd_pos)
        time.sleep(0.05)
    p, c = read_feedback()
    print(f"  Target: {cmd_pos[2]} pulses ({cmd_pos[2]*360/4096:.1f}°), Actual: {p[2]} pulses ({p[2]*360/4096:.1f}°), Cur: {c[2]} mA")

    # Return Joint 02
    for step in range(20):
        cmd_pos[2] = int(cmd_pos[2] - 7.5)
        write_regs(0x0100, cmd_pos)
        time.sleep(0.05)
    cmd_pos[2] = start_j02

    # 8. Test Joint 03 (Tip Flexion) - move +150 pulses
    print("\n[4] Testing Joint 03 (Tip Flexion +150 pulses)...")
    start_j03 = cmd_pos[3]
    for step in range(20):
        cmd_pos[3] = int(start_j03 + (step + 1) * 7.5)
        write_regs(0x0100, cmd_pos)
        time.sleep(0.05)
    p, c = read_feedback()
    print(f"  Target: {cmd_pos[3]} pulses ({cmd_pos[3]*360/4096:.1f}°), Actual: {p[3]} pulses ({p[3]*360/4096:.1f}°), Cur: {c[3]} mA")

    # Return Joint 03
    for step in range(20):
        cmd_pos[3] = int(cmd_pos[3] - 7.5)
        write_regs(0x0100, cmd_pos)
        time.sleep(0.05)
    cmd_pos[3] = start_j03

    # 9. Test Joint 00 (Base Opposition) - move towards 0 (+200 pulses from -895)
    print("\n[5] Testing Joint 00 (Base Opposition +200 pulses towards zero)...")
    start_j00 = cmd_pos[0]
    for step in range(20):
        cmd_pos[0] = int(start_j00 + (step + 1) * 10)
        write_regs(0x0100, cmd_pos)
        time.sleep(0.05)
    p, c = read_feedback()
    print(f"  Target: {cmd_pos[0]} pulses ({cmd_pos[0]*360/4096:.1f}°), Actual: {p[0]} pulses ({p[0]*360/4096:.1f}°), Cur: {c[0]} mA")

    # Return Joint 00
    for step in range(20):
        cmd_pos[0] = int(cmd_pos[0] - 10)
        write_regs(0x0100, cmd_pos)
        time.sleep(0.05)

    # 10. Disable torque & close
    print("\n[6] Disabling Torque...")
    write_coil(0x0000, False)
    s.close()
    print("[Done] Test completed safely.")

if __name__ == "__main__":
    main()
