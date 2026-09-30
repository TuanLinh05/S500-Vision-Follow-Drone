"""Bom RC_CHANNELS_OVERRIDE vao PX4 SITL - CHI dung cho mo phong.

Ly do ton tai: SIH SITL khong co RC that, nen `tools/sitl_bench.py` truoc day
buoc phai chay voi `--rc-chan 0 --allow-no-rc-deadman`. Nghia la lop dead-man
RC - mot trong nhung lop an toan quan trong nhat - CHUA BAO GIO duoc kiem thu
end-to-end voi PX4 that (chi moi qua FakePX4 trong tests/).

Module nay bom RC_CHANNELS_OVERRIDE; PX4 phat lai thanh RC_CHANNELS, nen
`follow.app` doc duoc y het khi bay that va co the chay voi `--rc-chan 8`
KHONG can bat ky co bo qua nao.

An toan: CHI dung cho SITL. Khong bao gio chay script nay nham vao Pixhawk
that - no se ghi de RC that cua phi cong.

Dung nhu thu vien (khuyen nghi):
    inj = RcInjector("udpin:127.0.0.1:14550")
    inj.start()
    inj.set_channel(8, 2000)     # dead-man BAT
    ...
    inj.set_channel(8, 1000)     # dead-man TAT -> app phai abort
    inj.stop()

Hoac chay doc lap:
    python -m tools.sitl_rc_inject udpin:127.0.0.1:14550 --ch8 2000
"""

from __future__ import annotations

import argparse
import os
import threading
import time

os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil  # noqa: E402

# Mac dinh: stick giua, ga thap, cac cong tac o vi tri thap.
DEFAULT_CHANNELS = {1: 1500, 2: 1500, 3: 1000, 4: 1500,
                    5: 1000, 6: 1000, 7: 1000, 8: 1000}


class RcInjector(threading.Thread):
    """Phat RC_CHANNELS_OVERRIDE deu dan tren mot luong rieng."""

    def __init__(self, url: str, rate_hz: float = 20.0, hb_timeout: float = 15.0):
        super().__init__(daemon=True, name="sitl-rc-inject")
        self.url = url
        self.dt = 1.0 / rate_hz
        self.hb_timeout = hb_timeout
        self._lk = threading.Lock()
        self._chan = dict(DEFAULT_CHANNELS)
        self._stop = threading.Event()
        self._ready = threading.Event()
        self.link = None
        self.target = (1, 1)
        self.sent = 0

    def set_channel(self, chan: int, value: int) -> None:
        with self._lk:
            self._chan[int(chan)] = int(value)

    def channels(self) -> dict:
        with self._lk:
            return dict(self._chan)

    def wait_ready(self, timeout: float = 20.0) -> bool:
        return self._ready.wait(timeout)

    def stop(self) -> None:
        self._stop.set()

    def run(self) -> None:
        self.link = mavutil.mavlink_connection(
            self.url, source_system=200,
            source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)
        self.link.mav.heartbeat_send(6, 8, 0, 0, 0)
        hb = self.link.recv_match(type="HEARTBEAT", blocking=True,
                                  timeout=self.hb_timeout)
        if hb is None:
            print("[rc-inject] KHONG nhan duoc heartbeat PX4")
            return
        self.target = (hb.get_srcSystem(), hb.get_srcComponent())
        # yeu cau PX4 phat lai RC_CHANNELS de app doc duoc
        self.link.mav.command_long_send(
            self.target[0], self.target[1],
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, 50000, 0, 0, 0, 0, 0)
        self._ready.set()

        next_t = time.monotonic()
        while not self._stop.is_set():
            c = self.channels()
            try:
                self.link.mav.rc_channels_override_send(
                    self.target[0], self.target[1],
                    c.get(1, 1500), c.get(2, 1500), c.get(3, 1000),
                    c.get(4, 1500), c.get(5, 1000), c.get(6, 1000),
                    c.get(7, 1000), c.get(8, 1000))
                self.sent += 1
            except Exception as exc:
                print(f"[rc-inject] loi gui: {exc}")
            # tieu thu ban tin den de socket khong day
            while self.link.recv_match(blocking=False) is not None:
                pass
            next_t += self.dt
            time.sleep(max(0.0, next_t - time.monotonic()))
        try:
            self.link.close()
        except Exception:
            pass


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("url", nargs="?", default="udpin:127.0.0.1:14550")
    parser.add_argument("--ch8", type=int, default=2000)
    parser.add_argument("--seconds", type=float, default=60.0)
    args = parser.parse_args()

    inj = RcInjector(args.url)
    inj.start()
    if not inj.wait_ready(20.0):
        print("KHONG ket noi duoc PX4 SITL")
        return 1
    inj.set_channel(8, args.ch8)
    print(f"[rc-inject] dang bom RC (ch8={args.ch8}) trong {args.seconds:.0f}s")
    try:
        time.sleep(args.seconds)
    except KeyboardInterrupt:
        pass
    inj.stop()
    print(f"[rc-inject] da gui {inj.sent} goi")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
