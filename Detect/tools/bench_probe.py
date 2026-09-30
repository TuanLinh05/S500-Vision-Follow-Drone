"""Doc-only ket noi Pixhawk that qua USB - KHONG gui bat ky lenh doi mode
hay setpoint nao. Dung de kiem chung lop MAVLink truoc khi chay follow.app.

An toan: khong can --enable-control, khong can arm, khong can thao canh
quat de chay duoc script nay (nhung van nen thao canh quat theo quy trinh
bench test). Script chi doc heartbeat/RC/pin/vi tri/mode va IN RA - khong
bao gio goi set_mode(), khong bao gio tao SetpointStreamer.

Chay:
    python -m tools.bench_probe COM5
    python -m tools.bench_probe /dev/ttyACM0 --baud 57600 --seconds 15
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from follow.px4 import PX4Link, px4_mode_name  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("port", help="vd COM5 hoac /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--rc-chan", type=int, default=0,
                     help="Kenh RC muon theo doi rieng, 0 = khong")
    ap.add_argument("--all-rc", action="store_true",
                    help="In tat ca kenh RC co gia tri hop le")
    args = ap.parse_args()

    print(f"[+] Dang ket noi {args.port} @ {args.baud} baud ...")
    print("    (script nay CHI DOC - khong gui lenh doi mode/setpoint)")
    link = PX4Link(args.port, args.baud, hb_timeout=10.0)
    print(f"[+] Bat tay thanh cong: autopilot sysid={link.ap_sys} "
          f"compid={link.ap_comp}\n")

    t_end = time.monotonic() + args.seconds
    try:
        while time.monotonic() < t_end:
            time.sleep(1.0)
            rc = link.rc.value or {}
            pos = link.pos.value
            att = link.att.value
            line = (
                f"hb_age={link.hb.age:4.2f}s  "
                f"armed={'YES' if link.armed else 'no ':<3}  "
                f"mode={link.mode:<12}  "
                f"failsafe={'!!' if link.failsafe else 'ok'}  "
                f"batt={link.batt.value:5.2f}V "
                f"({link.batt_pct.value:.0f}%)  "
                f"pos={'valid' if (pos and link.pos.age < 1.0) else 'KHONG CO':<8}"
            )
            if pos:
                line += f" xyz=({pos[0]:+.2f},{pos[1]:+.2f},{pos[2]:+.2f})"
            if att:
                import math
                line += f"  yawspeed={math.degrees(att[3]):+5.1f}do/s"
            if args.rc_chan:
                v = rc.get(args.rc_chan, "?")
                line += f"  RCch{args.rc_chan}={v}"
            if args.all_rc:
                line += "  RC={" + ", ".join(
                    f"{ch}:{rc[ch]}" for ch in sorted(rc)) + "}"
            print(line)
    except KeyboardInterrupt:
        pass
    finally:
        link.close()

    print("\n[+] Da dong ket noi. Khong co lenh dieu khien nao duoc gui.")
    if not link.armed:
        print("    Luu y: 'armed=no' trong suot qua trinh la BINH THUONG "
              "neu ban chua arm bang RC.")


if __name__ == "__main__":
    main()
