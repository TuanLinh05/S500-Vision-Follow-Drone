"""Test YawController — cong thuc goc, deadzone mem, mat muc tieu."""
import math
import numpy as np
import pytest
from follow.control import YawController, YawConfig


def test_bearing_dung_cong_thuc_pinhole():
    """B1: dung atan2 thay vi tuyen tinh."""
    ctl = YawController(YawConfig(hfov_deg=90.0), width_px=1280)
    # bien phai: atan2(640, 640) = 45 do
    assert abs(ctl.bearing_deg(1280) - 45.0) < 0.01
    # 3/4 phai: atan2(480, 640) ≈ 36.87°, KHONG phai 33.75°
    assert abs(ctl.bearing_deg(1120) - 36.87) < 0.1
    # 1/2 phai: atan2(320, 640) ≈ 26.57°, KHONG phai 22.5° (B1)
    assert abs(ctl.bearing_deg(960) - 26.57) < 0.05
    # tam
    assert abs(ctl.bearing_deg(640)) < 1e-6


def test_mat_muc_tieu_ve_0_ngay_bo_qua_slew():
    """Mat muc tieu -> yaw ve 0 ngay lap tuc, ke ca khi slew rat nho."""
    ctl = YawController(YawConfig(slew=5.0), 1280)
    ctl.yaw = 35.0
    assert ctl.update(None, dt=0.05, allow=True) == 0.0


def test_gate_chan_thi_yaw_0():
    """Gate chan (allow=False) -> yaw ve 0."""
    ctl = YawController(YawConfig(), 1280)
    ctl.yaw = 20.0
    box = np.array([900., 0., 1000., 400.])
    assert ctl.update(box, 0.05, allow=False) == 0.0


def test_deadzone_mem():
    """B7: trong vung chet -> yaw 0. Ngoai vung chet -> bat dau tu 0."""
    cfg = YawConfig(deadzone_deg=4.0, gain=1.0, slew=1000.0)
    ctl = YawController(cfg, 1280)
    # nguoi o tam -> trong vung chet
    box_center = np.array([630., 100., 650., 500.])
    y = ctl.update(box_center, dt=0.05, allow=True)
    assert abs(y) < 0.2  # trong vung chet


def test_box_cham_bien_co_toc_do_toi_thieu():
    """B2: box sat bien phai -> lien tuc quay voi toc do toi thieu."""
    cfg = YawConfig(min_edge_rate=8.0, slew=1000.0, gain=0.1)
    ctl = YawController(cfg, 1280)
    box = np.array([1200., 100., 1280., 600.])  # cat o bien phai
    y = ctl.update(box, dt=0.05, allow=True)
    assert y > 0  # quay phai
    assert ctl.at_edge


def test_slew_gioi_han_buoc_nhay():
    """Slew rate gioi han buoc nhay moi chu ky."""
    cfg = YawConfig(slew=20.0, max_rate=40.0, gain=10.0)
    ctl = YawController(cfg, 1280)
    # nguoi rat lech -> raw cao, nhung slew chan
    box = np.array([1100., 100., 1200., 500.])
    y = ctl.update(box, dt=0.05, allow=True)
    # slew=20 do/s^2 * 0.05s = 1.0 do max thay doi
    assert abs(y) <= 1.1  # cho phep sai so nho


def test_invert_yaw():
    """Invert dao dau lech."""
    cfg = YawConfig(invert=True, gain=1.0, deadzone_deg=0.0, slew=1000.0)
    ctl = YawController(cfg, 1280)
    # nguoi ben phai (x > 640) -> binh thuong la quay phai (duong)
    # nhung invert -> quay trai (am)
    box = np.array([900., 100., 1000., 500.])
    y = ctl.update(box, dt=0.05, allow=True)
    assert y < 0


def test_max_rate_gioi_han():
    """Tran cung max_rate."""
    cfg = YawConfig(max_rate=10.0, gain=100.0, deadzone_deg=0.0, slew=10000.0)
    ctl = YawController(cfg, 1280)
    box = np.array([1200., 0., 1280., 400.])
    y = ctl.update(box, dt=1.0, allow=True)
    assert abs(y) <= 10.1


# ==================================================================
# v2.1
# ==================================================================

def test_invert_doi_dau_ca_uoc_luong_omega():
    """v2.0 chi doi dau bearing ma khong doi dau yawspeed thuc te ->
    uoc luong toc do goc muc tieu sai dau khi bat --invert-yaw."""
    box = np.array([800.0, 100.0, 900.0, 500.0])

    thuan = YawController(YawConfig(invert=False, k_ff=1.0, gain=0.0,
                                    deadzone_deg=0.0, slew=1e6), 1280)
    nghich = YawController(YawConfig(invert=True, k_ff=1.0, gain=0.0,
                                     deadzone_deg=0.0, slew=1e6), 1280)
    for _ in range(6):
        a = thuan.update(box, 0.05, True, 20.0)
        b = nghich.update(box, 0.05, True, 20.0)
    assert a == pytest.approx(-b, abs=1e-6), "invert phai doi dau ca hai ve"


def test_box_NaN_duoc_coi_nhu_mat_muc_tieu():
    ctl = YawController(YawConfig(), 1280)
    ctl.yaw = 25.0
    assert ctl.update(np.array([np.nan, 0.0, 100.0, 400.0]), 0.05, True) == 0.0


def test_dt_NaN_khong_lam_hong_bo_dieu_khien():
    ctl = YawController(YawConfig(), 1280)
    y = ctl.update(np.array([900.0, 0.0, 1000.0, 400.0]),
                   float("nan"), True)
    assert math.isfinite(y)


def test_yawspeed_NaN_bi_bo_qua():
    ctl = YawController(YawConfig(k_ff=1.0), 1280)
    for _ in range(5):
        y = ctl.update(np.array([900.0, 0.0, 1000.0, 400.0]), 0.05, True,
                       float("nan"))
    assert math.isfinite(y)


def test_dau_ra_luon_huu_han():
    ctl = YawController(YawConfig(gain=1e9, k_ff=1e9, slew=1e9), 1280)
    for _ in range(10):
        y = ctl.update(np.array([1279.0, 0.0, 1280.0, 400.0]), 0.05, True,
                       1e6)
        assert math.isfinite(y)
        assert abs(y) <= ctl.cfg.max_rate + 1e-6
