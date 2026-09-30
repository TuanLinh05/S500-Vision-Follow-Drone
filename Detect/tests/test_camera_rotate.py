"""Kiem thu xoay khung hinh trong Camera (bu camera lap xoay tren drone).

Quan trong ve AN TOAN: YawController tinh sai so goc tu toa do NGANG (x) cua
muc tieu. Neu camera lap xoay 90 do ma khong bu lai thi nguoi di sang
trai/phai THAT se hien ra la di len/xuong trong anh -> khong ra lenh xoay;
dong thoi thu gi lam doi vi tri DOC lai bi hieu nham thanh lech ngang -> ra
lenh yaw SAI. Da gap that tren UP7000 ngay 2026-09-04.
"""

import uuid
from pathlib import Path

import cv2
import numpy as np
import pytest

from follow.camera import ROTATIONS, Camera

W, H = 160, 120


@pytest.fixture
def video():
    """Video co MOT dam sang lech PHAI va len TREN, de nhan biet chieu xoay."""
    root = Path(__file__).resolve().parent.parent / "test_artifacts"
    root.mkdir(exist_ok=True)
    path = root / f"rot-{uuid.uuid4().hex}.avi"
    vw = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"),
                         30.0, (W, H))
    assert vw.isOpened()
    for _ in range(20):
        f = np.zeros((H, W, 3), np.uint8)
        # dam sang o goc PHAI-TREN cua khung goc
        f[10:30, W - 40:W - 10] = 255
        vw.write(f)
    vw.release()
    try:
        yield str(path)
    finally:
        for art in path.parent.glob(path.name + "*"):
            try:
                art.unlink()
            except FileNotFoundError:
                pass


def _grab(cam, timeout=5.0):
    import time
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        frame, seq, _ = cam.read()
        if frame is not None and seq > 0:
            return frame
        time.sleep(0.02)
    raise AssertionError("khong doc duoc khung nao")


def _bright_center(frame):
    """Tam cua vung sang, tra ve (x, y) chuan hoa 0..1."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ys, xs = np.nonzero(gray > 128)
    assert len(xs) > 0, "khong tim thay vung sang"
    h, w = gray.shape
    return xs.mean() / w, ys.mean() / h


def test_khong_xoay_giu_nguyen_kich_thuoc(video):
    cam = Camera(source=video, rotate=0)
    try:
        frame = _grab(cam)
        assert (cam.width, cam.height) == (W, H)
        assert frame.shape[:2] == (H, W)
        x, y = _bright_center(frame)
        assert x > 0.6 and y < 0.4, "dam sang phai o PHAI-TREN khi khong xoay"
    finally:
        cam.release()


def test_xoay_90_va_270_doi_cho_rong_cao(video):
    """Xoay 90/270 lam doi cho rong<->cao. cam.width/height PHAI phan anh
    kich thuoc SAU xoay, vi YawController dung cam.width lam tam khung va
    LoggingSink dung no lam kich thuoc video."""
    for rot in (90, 270):
        cam = Camera(source=video, rotate=rot)
        try:
            frame = _grab(cam)
            assert (cam.width, cam.height) == (H, W), f"rot={rot}"
            assert frame.shape[:2] == (W, H), f"rot={rot}"
        finally:
            cam.release()


def test_xoay_180_giu_kich_thuoc_nhung_lat_vi_tri(video):
    cam = Camera(source=video, rotate=180)
    try:
        frame = _grab(cam)
        assert (cam.width, cam.height) == (W, H)
        x, y = _bright_center(frame)
        assert x < 0.4 and y > 0.6, "xoay 180 phai dua dam sang ve TRAI-DUOI"
    finally:
        cam.release()


def test_xoay_270_dua_dam_sang_dung_cho():
    """Kiem tra CHIEU xoay bang ma tran truc tiep (khong phu thuoc codec).

    Anh goc co dam sang o PHAI-TREN. Xoay 270 (= 90 nguoc chieu kim dong ho)
    phai dua no ve TRAI-TREN.
    """
    img = np.zeros((H, W, 3), np.uint8)
    img[10:30, W - 40:W - 10] = 255
    out = cv2.rotate(img, ROTATIONS[270])
    assert out.shape[:2] == (W, H)
    x, y = _bright_center(out)
    assert x < 0.4, f"xoay 270 phai dua ve ben TRAI, dang o x={x:.2f}"
    assert y < 0.4, f"xoay 270 phai giu o phia TREN, dang o y={y:.2f}"


def test_gia_tri_xoay_khong_hop_le_bi_tu_choi(video):
    with pytest.raises(SystemExit):
        Camera(source=video, rotate=45)


def test_xoay_360_tuong_duong_khong_xoay(video):
    cam = Camera(source=video, rotate=360)
    try:
        assert cam.rotate == 0
        assert (cam.width, cam.height) == (W, H)
    finally:
        cam.release()
