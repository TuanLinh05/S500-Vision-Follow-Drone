"""Arm mot PX4 SITL qua MAVLink command - CHI dung cho mo phong.

Khac voi phan cung that (nguoi van hanh PHAI tu arm bang RC, follow.app
khong bao gio tu arm), trong SITL viec arm bang lenh MAVLink la cach lam
chuan cho automated test harness (khong co RC that de cam).

Chay:
    python -m tools.sitl_arm udp:127.0.0.1:14540
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


def main():
    url = sys.argv[1] if len(sys.argv) > 1 else "udp:127.0.0.1:14540"
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(url)
    hb = m.wait_heartbeat(timeout=10)
    if hb is None:
        print("Khong nhan duoc heartbeat")
        sys.exit(1)
    print(f"[+] heartbeat: sysid={m.target_system} compid={m.target_component}")

    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0)

    t_end = time.monotonic() + 5.0
    while time.monotonic() < t_end:
        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if msg is None:
            continue
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        print(f"    armed={armed}")
        if armed:
            print("[+] ARMED thanh cong")
            m.close()
            return
    print("[!] Khong xac nhan duoc arm trong 5s")
    m.close()
    sys.exit(1)


if __name__ == "__main__":
    main()
