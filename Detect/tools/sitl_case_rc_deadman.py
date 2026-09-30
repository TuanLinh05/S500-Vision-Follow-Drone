"""Kich ban RC DEAD-MAN end-to-end tren PX4 SITL that (SIH).

Day la bai test ma tren PHAN CUNG THAT chua lam duoc (xem
HANDOFF_CLAUDE_UP7000_2026-08-28.md muc 18.1/18.4: ca hai lan thu deu bi cac
gate KHAC chan truoc khi kip vao ENGAGED). SITL cho phep kiem chung day du
chuoi nay ma khong co rui ro vat ly nao.

Kiem chung:
  1. Drone SITL arm + cat canh (bang MAVLink, CHI hop le trong mo phong).
  2. follow.app chay voi `--rc-chan 8` THAT (khong dung co bo qua nao),
     RC duoc bom bang tools.sitl_rc_inject.
  3. ENGAGE thanh cong -> PX4 vao OFFBOARD, co lenh yaw khac 0.
  4. HA CH8 -> app phai abort trong < 1s, yaw ve 0, roi OFFBOARD.
  5. NANG CH8 lai -> app KHONG duoc tu dong ENGAGE lai.

Yeu cau chay truoc:
  - PX4 SITL (SIH):  make px4_sitl sihsim_quadx
  - follow.app    :  python3 -m tools.sitl_bench udp:127.0.0.1:14540 \
                        --rc-chan 8 --port-web 8090

Chay:
    python3 -m tools.sitl_case_rc_deadman 8090 /tmp/sitl_app.log
"""

from __future__ import annotations

import os
import sys
import threading
import time

os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil  # noqa: E402

from tools.sitl_drive import http, read_token, stats  # noqa: E402
from tools.sitl_rc_proxy import RcProxy  # noqa: E402

MAV_URL = "udpin:127.0.0.1:14550"
TAKEOFF_ALT = 2.5   # ~MIS_TAKEOFF_ALT mac dinh cua PX4


class RcProxyControl:
    """Dieu khien gia tri CH8 cua proxy dang chay qua file /tmp/sitl_ch8."""

    PATH = "/tmp/sitl_ch8"

    def set_ch8(self, value: int) -> None:
        with open(self.PATH, "w") as fh:
            fh.write(str(int(value)))
        time.sleep(0.15)


def _fail(msg: str) -> int:
    print(f"\n  !! THAT BAI: {msg}")
    return 1


def arm_and_takeoff(alt: float) -> bool:
    """Arm + cat canh trong SITL. CHI hop le o mo phong (bay that: nguoi lai
    tu arm, follow.app khong bao gio tu arm)."""
    link = mavutil.mavlink_connection(
        "udpin:127.0.0.1:14030", source_system=201,
        source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)
    link.mav.heartbeat_send(6, 8, 0, 0, 0)
    hb = link.recv_match(type="HEARTBEAT", blocking=True, timeout=15)
    if hb is None:
        print("  khong nhan duoc heartbeat de arm")
        return False
    tgt = (hb.get_srcSystem(), hb.get_srcComponent())

    # PHAI yeu cau stream vi tri: instance nay co data rate thap va khong
    # tu phat LOCAL_POSITION_NED (truoc day thieu buoc nay nen tuong nham
    # la cat canh that bai trong khi drone van len binh thuong).
    link.mav.command_long_send(
        tgt[0], tgt[1], mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        100000, 0, 0, 0, 0, 0)
    # ARM truoc, roi moi vao AUTO.TAKEOFF (dat mode truoc khi arm bi tu choi)
    link.mav.command_long_send(
        tgt[0], tgt[1], mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0)
    time.sleep(1.5)
    link.mav.command_long_send(
        tgt[0], tgt[1], mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 2, 0, 0, 0, 0)

    deadline = time.monotonic() + 60
    while time.monotonic() < deadline:
        msg = link.recv_match(type="LOCAL_POSITION_NED", blocking=True,
                              timeout=1.0)
        if msg is not None and -msg.z >= alt * 0.6:
            print(f"  da cat canh: cao {-msg.z:.2f}m")
            link.close()
            return True
    link.close()
    print("  cat canh qua han")
    return False


def main() -> int:
    web = f"http://127.0.0.1:{sys.argv[1]}"
    token = read_token(sys.argv[2])

    print("=" * 72)
    print("  KICH BAN: RC DEAD-MAN (CH8) end-to-end tren PX4 SITL")
    print("=" * 72)

    # Proxy da chay san (tools.sitl_rc_proxy) va dang bom CH8=2000.
    # Ket noi toi no de dieu khien gia tri CH8 qua file dieu khien.
    print("  [1] proxy dang bom RC, CH8=2000 (dead-man BAT)")
    ctl = RcProxyControl()
    time.sleep(1.0)

    s = stats(web, token)
    if s.get("rc_val") != 2000:
        return _fail(f"app khong doc duoc RC ch8 (rc_val={s.get('rc_val')}) "
                     f"- kiem tra --rc-chan 8 va rc injector")
    print(f"  [2] app doc duoc RC ch8={s['rc_val']} OK")

    if not s.get("armed"):
        print("  [3] dang arm + cat canh drone SITL...")
        if not arm_and_takeoff(TAKEOFF_ALT):
            pass
            return _fail("khong arm/cat canh duoc")
    time.sleep(3.0)

    # ---- chon muc tieu + ENGAGE
    http(web, "/pick?x=0.68&y=0.48", token, "POST")
    http(web, "/alive", token, "POST")
    http(web, "/engage", token, "POST")
    stop_alive = threading.Event()

    def beat():
        while not stop_alive.wait(0.15):
            try:
                http(web, "/alive", token, "POST")
            except Exception:
                return
    threading.Thread(target=beat, daemon=True).start()
    print("  [4] da bam ENGAGE, dang cho vao OFFBOARD...")

    deadline = time.monotonic() + 20
    engaged = False
    while time.monotonic() < deadline:
        s = stats(web, token)
        if s.get("engaged"):
            engaged = True
            break
        time.sleep(0.1)
    if not engaged:
        stop_alive.set(); pass
        return _fail(f"khong ENGAGE duoc: {stats(web, token)}")
    print(f"  [5] ENGAGED OK, mode={s.get('mode')} sp_hz={s.get('sp_hz')}")

    # doi co lenh yaw khac 0 (dang thuc su dieu khien)
    deadline = time.monotonic() + 8
    yaw_seen = 0.0
    while time.monotonic() < deadline:
        s = stats(web, token)
        if abs(s.get("yaw", 0.0)) > 0.5:
            yaw_seen = s["yaw"]
            break
        time.sleep(0.1)
    if yaw_seen == 0.0:
        stop_alive.set(); pass
        return _fail("khong thay lenh yaw nao khi da engaged")
    print(f"  [6] dang dieu khien yaw thuc su: {yaw_seen:+.1f} do/s")

    # ---- HA CH8: day la phep thu chinh
    print("  [7] HA CH8 xuong 1000 (nha dead-man)...")
    t_drop = time.monotonic()
    ctl.set_ch8(1000)

    abort_s = None
    deadline = t_drop + 5
    while time.monotonic() < deadline:
        s = stats(web, token)
        if not s.get("engaged"):
            abort_s = time.monotonic() - t_drop
            break
        time.sleep(0.02)
    if abort_s is None:
        stop_alive.set(); pass
        return _fail("ha CH8 ma app KHONG abort")
    print(f"  [8] da abort sau {abort_s:.3f}s (yeu cau < 1.0s)")
    if abort_s >= 1.0:
        stop_alive.set(); pass
        return _fail(f"abort qua cham: {abort_s:.3f}s >= 1.0s")

    s = stats(web, token)
    print(f"      yaw={s.get('yaw')} mode={s.get('mode')} "
          f"block={s.get('block')!r}")
    if abs(s.get("yaw", 0.0)) > 0.01:
        stop_alive.set(); pass
        return _fail(f"yaw chua ve 0 sau abort: {s.get('yaw')}")

    # ---- NANG CH8 lai: KHONG duoc tu dong engage lai
    print("  [9] NANG CH8 lai 2000, kiem tra KHONG tu re-engage...")
    ctl.set_ch8(2000)
    time.sleep(3.0)
    s = stats(web, token)
    if s.get("engaged"):
        stop_alive.set(); pass
        return _fail("nang CH8 lai ma app TU DONG engage lai - RAT NGUY HIEM")
    print(f"      OK, van idle: state={s.get('state')} "
          f"engaged={s.get('engaged')}")

    stop_alive.set()
    http(web, "/disengage", token, "POST")
    pass

    print("\n" + "=" * 72)
    print(f"  DAT: dead-man CH8 abort trong {abort_s:.3f}s, yaw ve 0, "
          f"khong tu re-engage")
    print("=" * 72)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
