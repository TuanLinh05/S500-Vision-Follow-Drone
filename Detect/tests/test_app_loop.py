"""Kiem thu VONG LAP CHINH end-to-end voi PX4 gia + model gia + video gia.

Day la ban thu nho cua cac kich ban SITL trong Plan/DroneFollowPX4.md muc 7.2:
  - chon muc tieu -> ENGAGE -> PX4 that su vao OFFBOARD -> co lenh yaw
  - vuot hang rao mem -> tu dong ngat va tro ve AUTO.LOITER
  - tha nut ENGAGE (mat nhip /alive) -> ngat trong duoi 1.5s

Khong can OpenVINO: `load_model` duoc thay bang model gia.
"""

import json
import math
import queue
import socket
import threading
import time
import urllib.request
import uuid
from pathlib import Path

import cv2
import numpy as np
import pytest

from follow import app as app_mod
from follow.px4 import (MODE_LOITER, MODE_OFFBOARD, PX4_MAIN_AUTO,
                        PX4_SUB_LOITER)
from tests.fake_px4 import FakePX4, free_udp_port, mv

FW, FH = 640, 480
IMGSZ = 416
BOX = (400.0, 100.0, 460.0, 400.0)      # nguoi lech PHAI so voi tam khung


# ------------------------------------------------------------ model gia
class _Tensor:
    def __init__(self, data):
        self.data = data


class _Req:
    """Luon tra ve dung mot nguoi tai BOX (toa do khung goc)."""

    def __init__(self):
        r = min(IMGSZ / FH, IMGSZ / FW)
        dw, dh = (IMGSZ - int(round(FW * r))) // 2, (IMGSZ - int(round(FH * r))) // 2
        x1, y1, x2, y2 = BOX
        self._det = np.array([[[x1 * r + dw, y1 * r + dh,
                                x2 * r + dw, y2 * r + dh, 0.95, 0.0]]],
                             np.float32)

    def infer(self, _inp):
        return None

    def start_async(self, _inp):
        return None

    def wait(self):
        return None

    def get_tensor(self, _op):
        return _Tensor(self._det)


class _Compiled:
    def create_infer_request(self):
        return _Req()

    def output(self, _i=0):
        return "out0"


def free_tcp_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


@pytest.fixture
def video():
    """Video gia: nen xam + mot hinh chu nhat dung yen."""
    # Khong dung pytest.tmp_path: tren may Windows bi sandbox, pytest co the
    # bi cam scandir TEMP truoc ca khi app test duoc chay. Thu muc nay nam
    # trong workspace va moi test co prefix UUID rieng, nen khong dung chung.
    root = Path(__file__).resolve().parent.parent / "test_artifacts"
    root.mkdir(exist_ok=True)
    path = root / f"gia-{uuid.uuid4().hex}.avi"
    vw = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"),
                         30.0, (FW, FH))
    assert vw.isOpened(), "khong tao duoc video test"
    for _ in range(45):
        f = np.full((FH, FW, 3), 90, np.uint8)
        cv2.rectangle(f, (int(BOX[0]), int(BOX[1])),
                      (int(BOX[2]), int(BOX[3])), (30, 60, 200), -1)
        vw.write(f)
    vw.release()
    try:
        yield str(path)
    finally:
        # App tao CSV ben canh video; chi xoa cac file cua dung fixture nay.
        for artifact in path.parent.glob(path.name + "*"):
            try:
                artifact.unlink()
            except FileNotFoundError:
                pass


class Driver:
    """Gia lap trinh duyet cua nguoi van hanh."""

    def __init__(self, port, token):
        self.base = f"http://127.0.0.1:{port}"
        self.token = token
        self._alive = None

    def _call(self, path, method="POST"):
        sep = '&' if '?' in path else '?'
        req = urllib.request.Request(
            f"{self.base}{path}{sep}t={self.token}", method=method)
        with urllib.request.urlopen(req, timeout=2.0) as r:
            return r.read()

    def stats(self):
        return json.loads(self._call("/stats", "GET"))

    def pick(self, xn, yn):
        self._call(f"/pick?x={xn}&y={yn}")

    def hold_engage(self):
        self._call("/alive")
        self._call("/engage")
        stop = threading.Event()

        def beat():
            while not stop.wait(0.15):
                try:
                    self._call("/alive")
                except Exception:
                    return
        t = threading.Thread(target=beat, daemon=True)
        t.start()
        self._alive = stop

    def release_engage(self):
        if self._alive:
            self._alive.set()
            self._alive = None


def wait_for(cond, timeout=6.0, poll=0.05):
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        try:
            if cond():
                return True
        except Exception:
            pass
        time.sleep(poll)
    return False


@pytest.fixture
def run_app(monkeypatch, video):
    """Chay main() tren luong nen; tra ve (fake_px4, driver)."""
    udp = free_udp_port()
    web = free_tcp_port()
    fake = FakePX4(udp)
    fake.start()

    monkeypatch.setattr(app_mod, "load_model",
                        lambda *a, **k: (_Compiled(), "gia.xml"))
    monkeypatch.setattr(app_mod.signal, "signal", lambda *a, **k: None)

    holder = {}
    real_op = app_mod.OperatorLink

    def make_op():
        op = real_op()
        holder["op"] = op
        return op
    monkeypatch.setattr(app_mod, "OperatorLink", make_op)

    monkeypatch.setattr(app_mod.sys, "argv", [
        "app", "--source", video, "--imgsz", str(IMGSZ),
        "--mavlink", f"udpin:127.0.0.1:{udp}", "--enable-control",
        "--rc-chan", "7", "--rc-min", "1500",
        "--batt-min", "14.0",
        "--sp-rate", "20", "--prestream", "0.3", "--ramp", "0.0",
        "--max-yaw-rate", "15", "--gain", "0.6", "--deadzone", "4",
        "--port", str(web), "--jpeg-hz", "4",
        "--fence-radius", "15", "--max-drift", "25",
        "--fence-alt-max", "20",
        "--fence-alt-min", "1.0", "--fence-max-speed", "1.5",
        "--max-engage-s", "0", "--loop-timeout", "2.0",
        "--log-csv", str(video) + ".csv",
    ])
    app_mod._stop.clear()

    err = {}

    def runner():
        try:
            app_mod.main()
        except BaseException as e:      # noqa: BLE001 - bao lai cho test
            err["e"] = e
    t = threading.Thread(target=runner, daemon=True, name="app-main")
    t.start()

    assert wait_for(lambda: "op" in holder, 15.0), f"app khong khoi dong: {err}"
    drv = Driver(web, holder["op"].token)
    assert wait_for(lambda: drv.stats()["fps"] >= 0, 15.0), \
        f"web khong len: {err}"
    assert wait_for(lambda: drv.stats()["xy_pos_healthy"] is True, 5.0), \
        f"chua nhan duoc SYS_STATUS XY healthy: {drv.stats()}"

    yield fake, drv, err

    app_mod._stop.set()
    drv.release_engage()
    t.join(timeout=10.0)
    fake.halt()
    time.sleep(0.05)


def _boot_app(monkeypatch, video, udp, web, extra_args, log_suffix):
    """Khoi dong follow.app tren luong nen voi model/OperatorLink gia, KHONG
    dung fixture run_app - dung khi test can chuan bi `fake` (vd fake.params)
    TRUOC luc app ket noi. Tra ve (drv, err, thread); goi da start() fake."""
    monkeypatch.setattr(app_mod, "load_model",
                        lambda *a, **k: (_Compiled(), "gia.xml"))
    monkeypatch.setattr(app_mod.signal, "signal", lambda *a, **k: None)
    holder = {}
    real_op = app_mod.OperatorLink
    monkeypatch.setattr(app_mod, "OperatorLink",
                        lambda: holder.setdefault("op", real_op()))
    monkeypatch.setattr(app_mod.sys, "argv", [
        "app", "--source", video, "--imgsz", str(IMGSZ),
        "--mavlink", f"udpin:127.0.0.1:{udp}", "--enable-control",
        "--rc-chan", "7", "--rc-min", "1500", "--batt-min", "14.0",
        "--prestream", "0.3", "--ramp", "0.0", "--max-engage-s", "0",
        "--port", str(web), "--jpeg-hz", "4", "--loop-timeout", "2.0",
        "--log-csv", str(video) + log_suffix,
    ] + list(extra_args))
    app_mod._stop.clear()
    err = {}

    def runner():
        try:
            app_mod.main()
        except BaseException as e:      # noqa: BLE001 - bao lai cho test
            err["e"] = e
    t = threading.Thread(target=runner, daemon=True, name="app-main")
    t.start()
    assert wait_for(lambda: "op" in holder, 15.0), f"app khong khoi dong: {err}"
    drv = Driver(web, holder["op"].token)
    assert wait_for(lambda: drv.stats()["fps"] >= 0, 15.0), f"web khong len: {err}"
    return drv, err, t


def yaw_deg(sp):
    return math.degrees(sp.yaw_rate)


def assert_stream_paused_with_zero(fake, drv):
    assert wait_for(lambda: drv.stats().get("sp_paused") is True, 4.0), \
        f"roi Offboard xong phai pause setpoint: {drv.stats()}"
    assert wait_for(lambda: drv.stats()["sp_hz"] == 0.0, 3.0)
    with fake._lk:
        assert fake.setpoints, "phai gui yaw=0 truoc khi pause"
        assert abs(yaw_deg(fake.setpoints[-1])) < 0.01
        fake.setpoints.clear()
    time.sleep(0.35)
    with fake._lk:
        assert not fake.setpoints, "idle sau LOITER khong duoc tiep tuc prime"


def test_chuoi_engage_va_ngat_khi_vuot_fence(run_app):
    fake, drv, err = run_app

    # ---- 1. chua chon muc tieu thi TU CHOI ENGAGE
    assert wait_for(lambda: drv.stats()["mode"] == "POSCTL")
    drv.hold_engage()
    assert wait_for(lambda: "muc tieu chua san sang" in drv.stats()["note"], 4.0), \
        f"phai tu choi ENGAGE khi chua co muc tieu: {drv.stats()}"
    assert fake.main_mode != 6, "khong duoc vao OFFBOARD khi chua co muc tieu"
    drv.release_engage()
    assert fake.last_setpoint(timeout=0.5) is None, \
        "khong duoc phat setpoint truoc khi toan bo preflight PASS"

    # ---- 2. chon muc tieu roi ENGAGE
    cx = (BOX[0] + BOX[2]) / 2 / FW
    cy = (BOX[1] + BOX[3]) / 2 / FH
    assert wait_for(lambda: drv.stats()["locked"] or drv.pick(cx, cy) or True,
                    2.0)
    drv.pick(cx, cy)
    assert wait_for(lambda: len(drv.stats()["locked"]) == 1, 5.0), \
        f"khong khoa duoc muc tieu: {drv.stats()}"

    drv.hold_engage()
    assert wait_for(lambda: drv.stats()["engaged"] is True, 8.0), \
        f"khong ENGAGE duoc: {drv.stats()} err={err}"
    assert fake.main_mode == 6, "PX4 phai o OFFBOARD"
    assert drv.stats()["mode"] == MODE_OFFBOARD

    # ---- 3. co lenh yaw thuc su, dung chieu (nguoi lech PHAI -> yaw duong)
    assert wait_for(lambda: abs(yaw_deg(fake.fresh_setpoint())) > 1.0, 4.0), \
        "khong co lenh yaw nao duoc gui"
    sp = fake.fresh_setpoint()
    assert yaw_deg(sp) > 0, "nguoi ben phai -> phai quay phai"
    assert abs(yaw_deg(sp)) <= 15.0 + 1e-6, "vuot tran yaw"
    assert sp.type_mask == 1528, "phai la setpoint GIU VI TRI"

    # ---- 4. vuot hang rao mem -> phai tu ngat va tra ve AUTO.LOITER
    fake.pos = (20.0, 0.0, -5.0)   # >15m fence, <25m luoi du phong
    assert wait_for(lambda: drv.stats()["engaged"] is False, 5.0), \
        f"vuot fence ma khong ngat: {drv.stats()}"
    # sau khi ngat, diem chot bi xoa nen `fence` tro lai "ok";
    # ly do that su nam trong `note` va trong CSV
    assert "hang rao" in drv.stats()["note"], drv.stats()["note"]
    assert wait_for(lambda: fake.main_mode == 4 and fake.sub_mode == 3, 6.0), \
        "khong tra ve AUTO.LOITER sau khi vuot fence"
    assert wait_for(lambda: drv.stats()["mode"] == MODE_LOITER, 4.0)
    assert_stream_paused_with_zero(fake, drv)

    # ---- 5. ENGAGE LAI sau pause: phai resume ve toc do day, khong con bi
    # dong bang o 0 Hz mai mai (muc 11.4 HANDOFF_CLAUDE_UP7000_2026-08-28.md)
    drv.hold_engage()
    assert wait_for(lambda: drv.stats()["engaged"] is True, 8.0), \
        f"khong ENGAGE lai duoc sau khi da pause: {drv.stats()} err={err}"
    assert fake.main_mode == 6, "PX4 phai vao lai OFFBOARD"
    assert wait_for(lambda: drv.stats()["sp_hz"] >= 18.0, 4.0), \
        f"khong resume ve toc do day sau ENGAGE lan hai: {drv.stats()}"
    assert wait_for(lambda: abs(yaw_deg(fake.fresh_setpoint())) > 1.0, 4.0), \
        "ENGAGE lan hai khong co lenh yaw nao"
    drv.release_engage()


def test_tha_nut_engage_thi_ngat(run_app):
    fake, drv, err = run_app
    assert wait_for(lambda: drv.stats()["mode"] == "POSCTL")

    cx = (BOX[0] + BOX[2]) / 2 / FW
    cy = (BOX[1] + BOX[3]) / 2 / FH
    drv.pick(cx, cy)
    assert wait_for(lambda: len(drv.stats()["locked"]) == 1, 5.0)

    drv.hold_engage()
    assert wait_for(lambda: drv.stats()["engaged"] is True, 8.0), \
        f"khong ENGAGE duoc: {drv.stats()} err={err}"

    # tha tay: khong con /alive -> gate phai chan trong duoi 1.5s
    drv.release_engage()
    t0 = time.monotonic()
    assert wait_for(lambda: drv.stats()["engaged"] is False, 3.0), \
        "tha nut ENGAGE ma khong ngat"
    assert (time.monotonic() - t0) < 2.5
    assert_stream_paused_with_zero(fake, drv)


def test_EKF_reset_khi_dang_engage_thi_ngat(run_app):
    """v2.3: ODOMETRY.reset_counter doi trong khi giu ENGAGE (EKF dat lai vi
    tri/van toc/huong) -> phai disengage + tra ve LOITER, khong duoc coi day
    la tiep tuc bay binh thuong (HANDOFF muc 10/11.1)."""
    fake, drv, err = run_app
    assert wait_for(lambda: drv.stats()["mode"] == "POSCTL")

    cx = (BOX[0] + BOX[2]) / 2 / FW
    cy = (BOX[1] + BOX[3]) / 2 / FH
    drv.pick(cx, cy)
    assert wait_for(lambda: len(drv.stats()["locked"]) == 1, 5.0)

    drv.hold_engage()
    assert wait_for(lambda: drv.stats()["engaged"] is True, 8.0), \
        f"khong ENGAGE duoc: {drv.stats()} err={err}"

    fake.odom_reset_counter += 1        # mo phong EKF reset giua chung
    assert wait_for(lambda: drv.stats()["engaged"] is False, 4.0), \
        f"EKF reset trong khi ENGAGE ma khong ngat: {drv.stats()}"
    assert "EKF" in drv.stats()["note"], drv.stats()["note"]
    drv.release_engage()
    assert_stream_paused_with_zero(fake, drv)


def test_loiter_cham_khong_pause_engage_moi(run_app, monkeypatch):
    """v2.1 (da co trong code, chua co test): callback enter_loiter_async
    cua LAN DISENGAGE TRUOC toi TRE, sau khi mot ENGAGE MOI da bat dau, thi
    KHONG duoc pause luong setpoint dang phuc vu ENGAGE moi (HANDOFF 11.5)."""
    fake, drv, err = run_app
    assert wait_for(lambda: drv.stats()["mode"] == "POSCTL")

    cx = (BOX[0] + BOX[2]) / 2 / FW
    cy = (BOX[1] + BOX[3]) / 2 / FH
    drv.pick(cx, cy)
    assert wait_for(lambda: len(drv.stats()["locked"]) == 1, 5.0)

    drv.hold_engage()
    assert wait_for(lambda: drv.stats()["engaged"] is True, 8.0), \
        f"khong ENGAGE duoc: {drv.stats()} err={err}"

    # Chan enter_loiter_async: van doi mode THAT (giong code that) nhung
    # GIU LAI on_done thay vi goi ngay, de test tu quyet dinh khi nao goi.
    pending = queue.Queue()
    real_link_cls = app_mod.PX4Link

    def fake_enter_loiter_async(self, on_done=None):
        ok = self.set_mode(PX4_MAIN_AUTO, PX4_SUB_LOITER, expect=MODE_LOITER)
        if on_done is not None:
            pending.put((on_done, ok))
        return True
    monkeypatch.setattr(real_link_cls, "enter_loiter_async",
                        fake_enter_loiter_async)

    # Tha nut ENGAGE -> app goi enter_loiter_async, callback bi giu lai
    drv.release_engage()
    assert wait_for(lambda: drv.stats()["engaged"] is False, 4.0), \
        f"tha ENGAGE ma khong ngat: {drv.stats()}"
    cb, ok = pending.get(timeout=5.0)
    assert ok is True

    # ENGAGE LAI truoc khi callback cu (tre) duoc goi
    drv.hold_engage()
    assert wait_for(lambda: drv.stats()["engaged"] is True, 8.0), \
        f"khong ENGAGE lai duoc: {drv.stats()} err={err}"

    # Callback CU bay gio moi toi -> KHONG duoc pause ENGAGE MOI nay
    cb(ok)
    time.sleep(0.3)
    assert drv.stats()["engaged"] is True, \
        "callback LOITER tre da pause nham mot ENGAGE moi"
    assert drv.stats()["sp_hz"] >= 15.0, \
        f"callback tre lam giam toc do setpoint cua ENGAGE moi: {drv.stats()}"
    drv.release_engage()


def test_RC_deadman_xung_dot_voi_RC_MAP_thi_tu_choi_engage(monkeypatch, video):
    """v2.3, tinh huong thuc te tren S500 (HANDOFF muc 9): CH7 vua la dead-man
    cua app vua la RC_MAP_FLTMODE cua PX4 -> KHONG con la mot lop doc lap,
    app phai TU CHOI ENGAGE thay vi im lang chap nhan."""
    udp, web = free_udp_port(), free_tcp_port()
    fake = FakePX4(udp)
    fake.params["RC_MAP_FLTMODE"] = (7, mv.MAV_PARAM_TYPE_INT32)
    fake.start()
    t = None
    try:
        drv, err, t = _boot_app(
            monkeypatch, video, udp, web, [], "-conflict.csv")
        assert wait_for(lambda: drv.stats()["xy_pos_healthy"] is True, 5.0)
        cx = (BOX[0] + BOX[2]) / 2 / FW
        cy = (BOX[1] + BOX[3]) / 2 / FH
        drv.pick(cx, cy)
        assert wait_for(lambda: len(drv.stats()["locked"]) == 1, 5.0)

        drv.hold_engage()
        assert wait_for(lambda: "xung dot" in drv.stats()["note"], 4.0), \
            (f"phai tu choi ENGAGE khi RC dead-man xung dot RC_MAP_FLTMODE: "
             f"{drv.stats()}")
        assert fake.main_mode != 6, \
            "khong duoc vao OFFBOARD khi dead-man xung dot"
        drv.release_engage()
    finally:
        app_mod._stop.set()
        if t is not None:
            t.join(timeout=10.0)
        fake.halt()


def test_bench_allow_conflicting_deadman_bo_qua_xung_dot(monkeypatch, video):
    """--bench-allow-conflicting-deadman: operator DA XAC NHAN chap nhan
    dead-man khong doc lap (CHI SITL/bench) -> van ENGAGE duoc."""
    udp, web = free_udp_port(), free_tcp_port()
    fake = FakePX4(udp)
    fake.params["RC_MAP_FLTMODE"] = (7, mv.MAV_PARAM_TYPE_INT32)
    fake.start()
    t = None
    try:
        drv, err, t = _boot_app(
            monkeypatch, video, udp, web,
            ["--bench-allow-conflicting-deadman"], "-conflict-ok.csv")
        assert wait_for(lambda: drv.stats()["xy_pos_healthy"] is True, 5.0)
        cx = (BOX[0] + BOX[2]) / 2 / FW
        cy = (BOX[1] + BOX[3]) / 2 / FH
        drv.pick(cx, cy)
        assert wait_for(lambda: len(drv.stats()["locked"]) == 1, 5.0)

        drv.hold_engage()
        assert wait_for(lambda: drv.stats()["engaged"] is True, 8.0), (
            f"khong ENGAGE duoc du da co --bench-allow-conflicting-deadman: "
            f"{drv.stats()} err={err}")
        assert fake.main_mode == 6
        drv.release_engage()
    finally:
        app_mod._stop.set()
        if t is not None:
            t.join(timeout=10.0)
        fake.halt()


def test_cong_tac_RC_dead_man(monkeypatch, video):
    """--rc-chan: ha cong tac xuong duoi nguong -> ngat ngay."""
    udp = free_udp_port()
    web = free_tcp_port()
    fake = FakePX4(udp)
    fake.start()
    monkeypatch.setattr(app_mod, "load_model",
                        lambda *a, **k: (_Compiled(), "gia.xml"))
    monkeypatch.setattr(app_mod.signal, "signal", lambda *a, **k: None)
    holder = {}
    real_op = app_mod.OperatorLink
    monkeypatch.setattr(app_mod, "OperatorLink",
                        lambda: holder.setdefault("op", real_op()))
    monkeypatch.setattr(app_mod.sys, "argv", [
        "app", "--source", video, "--imgsz", str(IMGSZ),
        "--mavlink", f"udpin:127.0.0.1:{udp}", "--enable-control",
        "--rc-chan", "7", "--rc-min", "1500",
        "--batt-min", "14.0",
        "--prestream", "0.3", "--ramp", "0.0", "--max-engage-s", "0",
        "--port", str(web), "--jpeg-hz", "4", "--loop-timeout", "2.0",
        "--log-csv", str(video) + "2.csv",
    ])
    app_mod._stop.clear()
    t = threading.Thread(target=app_mod.main, daemon=True)
    t.start()
    try:
        assert wait_for(lambda: "op" in holder, 15.0)
        drv = Driver(web, holder["op"].token)
        assert wait_for(lambda: drv.stats()["mode"] == "POSCTL", 10.0)
        assert wait_for(lambda: drv.stats()["xy_pos_healthy"] is True, 5.0)
        cx = (BOX[0] + BOX[2]) / 2 / FW
        cy = (BOX[1] + BOX[3]) / 2 / FH
        drv.pick(cx, cy)
        assert wait_for(lambda: len(drv.stats()["locked"]) == 1, 5.0)
        drv.hold_engage()
        assert wait_for(lambda: drv.stats()["engaged"] is True, 8.0)

        fake.rc = {**fake.rc, 7: 1100}       # ha cong tac
        t0 = time.monotonic()
        assert wait_for(lambda: drv.stats()["engaged"] is False, 3.0), \
            "ha cong tac RC ma khong ngat"
        assert (time.monotonic() - t0) < 1.0, "dead-man phai phan hoi < 1s"
        drv.release_engage()
    finally:
        app_mod._stop.set()
        t.join(timeout=10.0)
        fake.halt()


def test_web_moi_endpoint_deu_yeu_cau_token(run_app):
    """A5: khong co token thi khong lam duoc gi, ke ca xem hinh."""
    import urllib.error
    _fake, drv, _err = run_app
    endpoints = [("/", "GET"), ("/stats", "GET"), ("/stream.mjpg", "GET"),
                 ("/engage", "POST"), ("/disengage", "POST"),
                 ("/alive", "POST"), ("/unlock", "POST"),
                 ("/pick?x=0.5&y=0.5", "POST")]
    for path, method in endpoints:
        req = urllib.request.Request(drv.base + path, method=method)
        with pytest.raises(urllib.error.HTTPError) as ex:
            urllib.request.urlopen(req, timeout=2.0)
        assert ex.value.code == 403, f"{path} khong doi token"


def test_web_trang_goc_co_token_moi_hien_giao_dien(run_app):
    _fake, drv, _err = run_app
    req = urllib.request.Request(f"{drv.base}/?t={drv.token}", method="GET")
    with urllib.request.urlopen(req, timeout=2.0) as response:
        html = response.read().decode("utf-8")
    assert "Drone Follow Control" in html
    assert drv.token in html


def test_web_chan_request_tu_trang_la(run_app):
    """CSRF: co token nhung Origin la -> tu choi."""
    import urllib.error
    _fake, drv, _err = run_app
    req = urllib.request.Request(
        f"{drv.base}/engage?t={drv.token}", method="POST",
        headers={"Origin": "http://ke-tan-cong.example"})
    with pytest.raises(urllib.error.HTTPError) as ex:
        urllib.request.urlopen(req, timeout=2.0)
    assert ex.value.code == 403


def test_web_khong_nhan_GET_cho_lenh_doi_trang_thai(run_app):
    """/engage bang GET (vd <img src=...>) phai khong lam gi."""
    import urllib.error
    _fake, drv, _err = run_app
    req = urllib.request.Request(f"{drv.base}/engage?t={drv.token}",
                                 method="GET")
    with pytest.raises(urllib.error.HTTPError) as ex:
        urllib.request.urlopen(req, timeout=2.0)
    assert ex.value.code == 404
    assert drv.stats()["state"] == "idle"
