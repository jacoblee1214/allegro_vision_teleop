#!/usr/bin/env python3
import time
import sys
import numpy as np

import allegro_hand_v6_sdk as sdk

def main():
    print("==================================================")
    print("  Allegro Hand V6 Thumb Joint Diagnostic Test     ")
    print("==================================================")

    opts = sdk.Options()
    opts.comm_type = "tcp"
    opts.ip_address = "192.168.1.100"
    opts.tcp_port = 502
    opts.slave_id = 1
    # Safe torque limit: 0.23 Nm (~200mA)
    opts.torque_limit_table = [0.23] * sdk.JOINT_COUNT
    opts.k_p_table = [1.0] * sdk.JOINT_COUNT
    opts.k_d_table = [0.05] * sdk.JOINT_COUNT

    hand = sdk.HandV6ModBus(opts)
    if not hand.is_ready():
        print("[ERROR] Failed to connect to hand!")
        return 1

    dev_info = hand.get_device_info()
    print(f"[Device] FW: 0x{dev_info.firmware_version:04X}, Type: {dev_info.hand_type}, SN: {dev_info.serial_number}")

    # Read initial positions
    init_pos, _, _, _, _ = hand.read_states()
    thumb_init_rad = init_pos[0:4].copy()
    thumb_init_deg = np.rad2deg(thumb_init_rad)
    print(f"\n[Initial Thumb Positions]")
    for i in range(4):
        print(f"  joint0{i}: {thumb_init_rad[i]:.4f} rad ({thumb_init_deg[i]:.2f}°)")

    # Read holding registers for gains & torque limits
    p_gains = hand.read_gains("p_gain")
    print(f"P Gains (Thumb 0..3): {p_gains[0:4]}")

    print("\n[Step 1] Activating Position Control Mode & Torque ON...")
    hand.set_control_mode(True)
    time.sleep(0.3)

    # Hold initial position for 1 second
    cmd = init_pos.copy()
    print("\n[Step 2] Holding initial position for 1 sec...")
    for _ in range(20):
        hand.write_commands(position=cmd)
        time.sleep(0.05)
    
    pos, vel, tau, _, _ = hand.read_states()
    print(f"Held Pose: {np.round(np.rad2deg(pos[0:4]), 2)} deg, Tau: {np.round(tau[0:4], 3)} Nm")

    # Test joint 1 (known working)
    print("\n[Step 3] Testing Joint 01 (Elevation)...")
    target_j01 = thumb_init_rad[1] + np.deg2rad(15.0)
    cmd[1] = target_j01
    for _ in range(30):
        hand.write_commands(position=cmd)
        time.sleep(0.05)
    pos, _, tau, _, _ = hand.read_states()
    print(f"  Target: {np.rad2deg(target_j01):.2f}°, Actual: {np.rad2deg(pos[1]):.2f}°, Tau: {tau[1]:.3f} Nm")

    # Return joint 1
    cmd[1] = thumb_init_rad[1]
    for _ in range(20):
        hand.write_commands(position=cmd)
        time.sleep(0.05)

    # Test joint 00 (+10 deg and -10 deg)
    print("\n[Step 4] Testing Joint 00 (+10 deg relative)...")
    target_j00_p = thumb_init_rad[0] + np.deg2rad(10.0)
    cmd[0] = target_j00_p
    for _ in range(30):
        hand.write_commands(position=cmd)
        time.sleep(0.05)
    pos, _, tau, _, _ = hand.read_states()
    print(f"  Target: {np.rad2deg(target_j00_p):.2f}°, Actual: {np.rad2deg(pos[0]):.2f}°, Tau: {tau[0]:.3f} Nm")

    print("\n[Step 5] Testing Joint 00 (-10 deg relative)...")
    target_j00_m = thumb_init_rad[0] - np.deg2rad(10.0)
    cmd[0] = target_j00_m
    for _ in range(30):
        hand.write_commands(position=cmd)
        time.sleep(0.05)
    pos, _, tau, _, _ = hand.read_states()
    print(f"  Target: {np.rad2deg(target_j00_m):.2f}°, Actual: {np.rad2deg(pos[0]):.2f}°, Tau: {tau[0]:.3f} Nm")
    cmd[0] = thumb_init_rad[0]
    for _ in range(20):
        hand.write_commands(position=cmd)
        time.sleep(0.05)

    # Test joint 02 (+15 deg and -15 deg)
    print("\n[Step 6] Testing Joint 02 (+15 deg relative)...")
    target_j02_p = thumb_init_rad[2] + np.deg2rad(15.0)
    cmd[2] = target_j02_p
    for _ in range(30):
        hand.write_commands(position=cmd)
        time.sleep(0.05)
    pos, _, tau, _, _ = hand.read_states()
    print(f"  Target: {np.rad2deg(target_j02_p):.2f}°, Actual: {np.rad2deg(pos[2]):.2f}°, Tau: {tau[2]:.3f} Nm")

    print("\n[Step 7] Testing Joint 02 (-15 deg relative)...")
    target_j02_m = thumb_init_rad[2] - np.deg2rad(15.0)
    cmd[2] = target_j02_m
    for _ in range(30):
        hand.write_commands(position=cmd)
        time.sleep(0.05)
    pos, _, tau, _, _ = hand.read_states()
    print(f"  Target: {np.rad2deg(target_j02_m):.2f}°, Actual: {np.rad2deg(pos[2]):.2f}°, Tau: {tau[2]:.3f} Nm")
    cmd[2] = thumb_init_rad[2]
    for _ in range(20):
        hand.write_commands(position=cmd)
        time.sleep(0.05)

    # Test joint 03 (+15 deg and -15 deg)
    print("\n[Step 8] Testing Joint 03 (+15 deg relative)...")
    target_j03_p = thumb_init_rad[3] + np.deg2rad(15.0)
    cmd[3] = target_j03_p
    for _ in range(30):
        hand.write_commands(position=cmd)
        time.sleep(0.05)
    pos, _, tau, _, _ = hand.read_states()
    print(f"  Target: {np.rad2deg(target_j03_p):.2f}°, Actual: {np.rad2deg(pos[3]):.2f}°, Tau: {tau[3]:.3f} Nm")

    print("\n[Step 9] Testing Joint 03 (-15 deg relative)...")
    target_j03_m = thumb_init_rad[3] - np.deg2rad(15.0)
    cmd[3] = target_j03_m
    for _ in range(30):
        hand.write_commands(position=cmd)
        time.sleep(0.05)
    pos, _, tau, _, _ = hand.read_states()
    print(f"  Target: {np.rad2deg(target_j03_m):.2f}°, Actual: {np.rad2deg(pos[3]):.2f}°, Tau: {tau[3]:.3f} Nm")
    cmd[3] = thumb_init_rad[3]
    for _ in range(20):
        hand.write_commands(position=cmd)
        time.sleep(0.05)

    print("\n[Step 10] Disabling Torque...")
    hand.set_torque_enabled(False)
    print("[Done] Safe shutdown completed.")
    return 0

if __name__ == "__main__":
    sys.exit(main())
