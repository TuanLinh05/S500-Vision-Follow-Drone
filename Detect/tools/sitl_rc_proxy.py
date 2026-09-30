"""Proxy MAVLink cho SITL: chuyen tiep PX4 <-> follow.app VA bom RC tren
CUNG mot ket noi. CHI dung cho mo phong.

Vi sao can:
    PX4 SITL chi phat lai `RC_CHANNELS` tren DUNG ket noi MAVLink da gui
    `RC_CHANNELS_OVERRIDE` toi no. Neu bom RC tu mot ket noi khac (vd
    tools.sitl_rc_inject tren cong rieng) thi `follow.app` khong bao gio
    thay RC, va khong test duoc lop dead-man.

    Day la dac thu cua SITL. Tren PHAN CUNG THAT khong co van de nay: RC do
    driver cua PX4 phat len uORB va moi ket noi MAVLink deu nhan duoc.

    Proxy nay dat minh o giua: app noi toi proxy, proxy noi toi PX4, va
    proxy tu chen RC_CHANNELS_OVERRIDE vao luong di toi PX4. Nho vay PX4
    coi RC va traffic cua app den tu cung mot ket noi, nen RC_CHANNELS quay
    ve dung duong va app doc duoc y het khi bay that.

So do:
    follow.app  <--udp-->  proxy(app_port)  ...  proxy  <--udp-->  PX4(14580)
                              ^ bom RC_CHANNELS_OVERRIDE vao day

Dung nhu thu vien:
    px = RcProxy(app_port=14546)
    px.start(); px.wait_ready()
    px.set_channel(8, 2000)
    ...
    px.set_channel(8, 1000)

App chay voi:  --mavlink udpout:127.0.0.1:14546
"""

from __future__ import annotations

import argparse
import os
import select
import socket
import threading
import time

os.environ.setdefault("MAVLINK20", "1")

from pymavlink.dialects.v20 import common as mavlink2  # noqa: E402

DEFAULT_CHANNELS = {1: 1500, 2: 1500, 3: 1000, 4: 1500,
                    5: 1000, 6: 1000, 7: 1000, 8: 1000}


class _Buf:
    """File-like de mavlink2.MAVLink ghi byte ra."""

    def __init__(self):
        self.data = b""

    def write(self, b):
        self.data += b


class RcProxy(threading.Thread):
    def __init__(self, app_port: int = 14546,
                 px4_addr=("127.0.0.1", 14580),
                 px4_listen_port: int = 14540,
                 rate_hz: float = 20.0):
        super().__init__(daemon=True, name="sitl-rc-proxy")
        self.app_port = app_port
        self.px4_addr = px4_addr
        self.px4_listen_port = px4_listen_port
        self.dt = 1.0 / rate_hz
        self._lk = threading.Lock()
        self._chan = dict(DEFAULT_CHANNELS)
        self._stop = threading.Event()
        self._ready = threading.Event()
        self.n_to_app = 0
        self.n_to_px4 = 0
        self.n_rc = 0

    def set_channel(self, chan: int, value: int) -> None:
        with self._lk:
            self._chan[int(chan)] = int(value)

    def channels(self) -> dict:
        with self._lk:
            return dict(self._chan)

    # File dieu khien de kich ban test doi CH8 tu tien trinh khac.
    # CHI dung cho SITL; tren phan cung that RC den tu tay dieu khien.
    CONTROL_FILE = "/tmp/sitl_ch8"

    def _poll_control_file(self) -> None:
        try:
            with open(self.CONTROL_FILE) as fh:
                value = int(fh.read().strip())
        except (OSError, ValueError):
            return
        if self.channels().get(8) != value:
            self.set_channel(8, value)
            print(f"[rc-proxy] CH8 -> {value}", flush=True)

    def wait_ready(self, timeout: float = 20.0) -> bool:
        return self._ready.wait(timeout)

    def stop(self) -> None:
        self._stop.set()

    def run(self) -> None:
        # socket nhan tu PX4 (PX4 gui toi cong 14540) va gui nguoc lai PX4
        s_px4 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s_px4.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s_px4.bind(("127.0.0.1", self.px4_listen_port))
        s_px4.setblocking(False)

        # socket noi chuyen voi app
        s_app = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s_app.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s_app.bind(("127.0.0.1", 0))
        s_app.setblocking(False)
        app_addr = ("127.0.0.1", self.app_port)

        mav = mavlink2.MAVLink(_Buf(), srcSystem=200,
                               srcComponent=mavlink2.MAV_COMP_ID_ONBOARD_COMPUTER)
        self._ready.set()
        next_rc = time.monotonic()

        while not self._stop.is_set():
            # PHAI hut CAN socket moi vong: PX4 onboard link phat rat nhieu
            # ban tin: neu chi doc 1 goi/vong thi buffer tran va mat gan het
            # (trieu chung: app bao "mat heartbeat" du van co du lieu khac).
            ready, _, _ = select.select([s_px4, s_app], [], [], 0.005)
            for sock in ready:
                for _ in range(256):
                    try:
                        data, _addr = sock.recvfrom(4096)
                    except (socket.timeout, BlockingIOError, OSError):
                        break
                    if not data:
                        break
                    try:
                        if sock is s_px4:
                            s_app.sendto(data, app_addr)
                            self.n_to_app += 1
                        else:
                            s_px4.sendto(data, self.px4_addr)
                            self.n_to_px4 += 1
                    except OSError:
                        break
            # bom RC vao luong toi PX4 (cung ket noi voi traffic cua app)
            now = time.monotonic()
            if now >= next_rc:
                next_rc = now + self.dt
                self._poll_control_file()
                c = self.channels()
                buf = _Buf()
                mav.file = buf
                try:
                    mav.rc_channels_override_send(
                        1, 1,
                        c.get(1, 1500), c.get(2, 1500), c.get(3, 1000),
                        c.get(4, 1500), c.get(5, 1000), c.get(6, 1000),
                        c.get(7, 1000), c.get(8, 1000))
                    s_px4.sendto(buf.data, self.px4_addr)
                    self.n_rc += 1
                except Exception as exc:
                    print(f"[rc-proxy] loi bom RC: {exc}")
        s_px4.close()
        s_app.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--app-port", type=int, default=14546)
    parser.add_argument("--ch8", type=int, default=2000)
    parser.add_argument("--seconds", type=float, default=120.0)
    args = parser.parse_args()

    px = RcProxy(app_port=args.app_port)
    px.start()
    px.wait_ready(10.0)
    px.set_channel(8, args.ch8)
    print(f"[rc-proxy] app_port={args.app_port} ch8={args.ch8}")
    try:
        time.sleep(args.seconds)
    except KeyboardInterrupt:
        pass
    px.stop()
    print(f"[rc-proxy] to_app={px.n_to_app} to_px4={px.n_to_px4} rc={px.n_rc}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
