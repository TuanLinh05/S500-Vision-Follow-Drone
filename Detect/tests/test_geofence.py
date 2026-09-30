"""Test hang rao mem — lop chinh chong bay mat kiem soat.

Toa do LOCAL_NED: z huong XUONG, nen do cao = -z.
Diem chot (anchor) la vi tri luc ENGAGE.
"""
import pytest

from follow.geofence import (BREACH, OK, WARN, FenceConfig, evaluate,
                             preflight)

CFG = FenceConfig(radius_m=15.0, home_radius_m=0.0, alt_dev_m=3.0,
                  alt_max_m=10.0, alt_min_m=1.5, max_speed_ms=1.5,
                  warn_ratio=0.75, predict_s=1.0)

ANCHOR = (0.0, 0.0, -5.0)          # cao 5 m


def test_tam_vung_thi_ok():
    st = evaluate(CFG, ANCHOR, (1.0, 1.0, -5.0), (0.0, 0.0, 0.0))
    assert st.level == OK
    assert st.dist_m == pytest.approx(1.414, abs=0.01)
    assert st.alt_m == pytest.approx(5.0)


def test_vuot_ban_kinh_thi_breach():
    st = evaluate(CFG, ANCHOR, (16.0, 0.0, -5.0), (0.0, 0.0, 0.0))
    assert st.level == BREACH
    assert "vuot fence" in st.reason


def test_gan_bien_thi_warn():
    st = evaluate(CFG, ANCHOR, (12.0, 0.0, -5.0), (0.0, 0.0, 0.0))
    assert st.level == WARN
    assert "gan bien" in st.reason


def test_du_doan_vuot_bien_thi_warn_som():
    """Con trong vung nhung voi van toc hien tai se vuot trong 1s."""
    st = evaluate(CFG, ANCHOR, (10.0, 0.0, -5.0), (6.0, 0.0, 0.0))
    assert st.level in (WARN, BREACH)
    assert st.dist_pred_m == pytest.approx(16.0, abs=0.01)


def test_troi_do_cao_thi_breach():
    st = evaluate(CFG, ANCHOR, (0.0, 0.0, -9.0), (0.0, 0.0, 0.0))
    assert st.level == BREACH
    assert "lech do cao" in st.reason
    assert st.alt_dev_m == pytest.approx(4.0)


def test_vuot_tran_tuyet_doi():
    cfg = FenceConfig(radius_m=0.0, alt_dev_m=0.0, alt_max_m=10.0,
                      alt_min_m=0.0, max_speed_ms=0.0)
    st = evaluate(cfg, None, (0.0, 0.0, -12.0), None)
    assert st.level == BREACH
    assert "vuot tran" in st.reason


def test_duoi_san_tuyet_doi():
    cfg = FenceConfig(radius_m=0.0, alt_dev_m=0.0, alt_max_m=0.0,
                      alt_min_m=1.5, max_speed_ms=0.0)
    st = evaluate(cfg, None, (0.0, 0.0, -0.8), None)
    assert st.level == BREACH
    assert "duoi san" in st.reason


def test_troi_ngang_qua_nhanh_thi_breach():
    """Che do yaw-only: drone khong duoc dich chuyen."""
    st = evaluate(CFG, ANCHOR, (1.0, 0.0, -5.0), (2.5, 0.0, 0.0))
    assert st.level == BREACH
    assert "dang troi" in st.reason
    assert st.speed_ms == pytest.approx(2.5)


def test_ban_kinh_tu_diem_cat_canh():
    cfg = FenceConfig(radius_m=0.0, home_radius_m=30.0, alt_dev_m=0.0,
                      alt_max_m=0.0, alt_min_m=0.0, max_speed_ms=0.0)
    st = evaluate(cfg, None, (40.0, 0.0, -5.0), None)
    assert st.level == BREACH
    assert "diem cat canh" in st.reason
    assert st.home_m == pytest.approx(40.0)


def test_tat_fence_thi_luon_ok():
    cfg = FenceConfig(enabled=False)
    st = evaluate(cfg, ANCHOR, (999.0, 999.0, -999.0), (50.0, 50.0, 0.0))
    assert st.level == OK


def test_khong_co_vi_tri_thi_khong_ket_luan():
    """Gate co lop `pos_valid` rieng; fence khong duoc doan bua."""
    assert evaluate(CFG, ANCHOR, None, None).level == OK


def test_NaN_khong_lam_do_fence():
    nan = float("nan")
    st = evaluate(CFG, ANCHOR, (nan, 0.0, -5.0), (nan, nan, nan))
    assert st.level == OK          # coi nhu khong co du lieu
    st2 = evaluate(CFG, ANCHOR, (1.0, 1.0, -5.0), (nan, 0.0, 0.0))
    assert st2.speed_ms == 0.0     # van toc ban bi bo qua


def test_muc_do_uu_tien_breach_hon_warn():
    """Vua gan bien vua vuot tran -> phai bao BREACH."""
    st = evaluate(CFG, ANCHOR, (12.0, 0.0, -12.0), (0.0, 0.0, 0.0))
    assert st.level == BREACH


# ------------------------------------------------------------- preflight
def test_preflight_khong_co_vi_tri():
    assert "chua co vi tri" in preflight(CFG, None)


def test_preflight_dat_khi_trong_vung():
    assert preflight(CFG, (0.0, 0.0, -5.0)) == ""


def test_preflight_tu_choi_khi_qua_cao():
    why = preflight(CFG, (0.0, 0.0, -14.0))
    assert why and "ngoai vung" in why


def test_preflight_tu_choi_khi_qua_thap():
    why = preflight(CFG, (0.0, 0.0, -0.5))
    assert why and "ngoai vung" in why


def test_preflight_bo_qua_khi_tat_fence():
    assert preflight(FenceConfig(enabled=False), None) == ""
