"""Doc-only, theo doi ESTIMATOR_STATUS (MAVLink) theo thoi gian thuc.

Khac voi tools/px4_estimator_probe.py (tom tat SAU khi da doc xong `--seconds`
giay va thoat), script nay IN LIEN TUC tung dong, danh cho luc dang can quan
sat truc tiep trong khi thao tac vat ly (vd be drone di chuyen bang tay luc
DISARM de kiem tra optical flow co "song lai" khi co chuyen dong that hay
khong - xem HANDOFF_CLAUDE_UP7000_2026-08-28.md muc 18.2).

An toan: chi doc telemetry, khong bao gio goi set_mode()/arm/setpoint. Khong
can arm, khong can canh quat, dung duoc ca khi disarm hoan toan.

ESTIMATOR_STATUS.flags la bitmask THO qua MAVLink (khong chi tiet bang cac
co uORB estimator_status_flags trong ULog nhu cs_opt_flow/
cs_inertial_dead_reckoning), nhung ESTIMATOR_CONST_POS_MODE va
ESTIMATOR_POS_HORIZ_REL la hai bit gan nhat de suy ra tu xa: dang co nguon
sua vi tri ngang tuong doi (flow/GPS) hay dang "dead reckoning" thuan IMU.

Chay:
    python -m tools.estimator_status_probe /dev/serial/by-id/usb-...-if00
    python -m tools.estimator_status_probe COM5 --hz 5
"""

from __future__ import annotations

import argparse
import os
import time

os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil  # noqa: E402

# Ten bit -> gia tri, theo dung thu tu MAV_ESTIMATOR_STATUS_FLAGS.
_FLAG_BITS = (
    ("ATTITUDE", 1), ("VEL_HORIZ", 2), ("VEL_VERT", 4),
    ("POS_HORIZ_REL", 8), ("POS_HORIZ_ABS", 16), ("POS_VERT_ABS", 32),
    ("POS_VERT_AGL", 64), ("CONST_POS_MODE", 128),
    ("PRED_POS_HORIZ_REL", 256), ("PRED_POS_HORIZ_ABS", 512),
    ("GPS_GLITCH", 1024), ("ACCEL_ERROR", 2048),
)


def _decode_flags(flags: int) -> str:
    names = [name for name, bit in _FLAG_BITS if flags & bit]
    return "|".join(names) if names else "(khong co co nao)"


def _wait_autopilot_heartbeat(link, timeout: float):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = link.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if msg is None:
            continue
        if getattr(msg, "type", None) == mavutil.mavlink.MAV_TYPE_GCS:
            continue
        autopilot = getattr(
            msg, "autopilot", mavutil.mavlink.MAV_AUTOPILOT_INVALID)
        if autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            continue
        return msg
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument("--hz", type=float, default=5.0,
                        help="Tan so yeu cau PX4 phat ESTIMATOR_STATUS")
    args = parser.parse_args()

    print(f"[+] Dang ket noi {args.port} @ {args.baud} baud ...")
    print("    (script nay CHI DOC - khong gui lenh doi mode/setpoint/arm)")
    link = mavutil.mavlink_connection(
        args.port, baud=args.baud, source_system=245,
        source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)
    hb = _wait_autopilot_heartbeat(link, 10.0)
    if hb is None:
        print("[!] Khong nhan duoc heartbeat autopilot")
        return 1
    link.target_system = hb.get_srcSystem()
    link.target_component = hb.get_srcComponent()
    print(f"[+] Bat tay thanh cong: autopilot sysid={link.target_system} "
          f"compid={link.target_component}\n")

    link.mav.command_long_send(
        link.target_system, link.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_ESTIMATOR_STATUS,
        int(1e6 / max(args.hz, 0.1)), 0, 0, 0, 0, 0)
    link.mav.command_long_send(
        link.target_system, link.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        int(1e6 / max(args.hz, 0.1)), 0, 0, 0, 0, 0)

    pos = None
    armed = None
    t_end = time.monotonic() + args.seconds
    try:
        while time.monotonic() < t_end:
            msg = link.recv_match(blocking=True, timeout=0.5)
            if msg is None:
                continue
            if msg.get_srcSystem() != link.target_system:
                continue
            kind = msg.get_type()
            if kind == "HEARTBEAT":
                armed = bool(
                    msg.base_mode
                    & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif kind == "LOCAL_POSITION_NED":
                pos = (msg.x, msg.y, msg.z)
            elif kind == "ESTIMATOR_STATUS":
                flags = int(msg.flags)
                flow_ok = bool(flags & 8) and not bool(flags & 128)
                pos_str = (f"({pos[0]:+.2f},{pos[1]:+.2f},{pos[2]:+.2f})"
                          if pos else "(chua co)")
                print(
                    f"armed={'YES' if armed else 'no ':<3}  "
                    f"pos={pos_str:<24}  "
                    f"co_ho_tro_ngang={'CO' if flow_ok else 'KHONG':<5}  "
                    f"pos_h_ratio={msg.pos_horiz_ratio:5.2f}  "
                    f"vel_ratio={msg.vel_ratio:5.2f}  "
                    f"hagl_ratio={msg.hagl_ratio:5.2f}  "
                    f"flags={_decode_flags(flags)}")
    except KeyboardInterrupt:
        pass
    finally:
        link.close()

    print("\n[+] Da dong ket noi. Khong co lenh dieu khien nao duoc gui.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
