"""Test hau xu ly detection — loc kich thuoc / ti le / bien khung.

Trong tam: mot nguoi dung nua trong nua ngoai khung PHAI duoc giu lai.
Truoc v2.1 box bi clip vao khung roi moi tinh ti le -> nguoi o ria bi coi
la "qua beo" va bi loai, dung tinh huong nguy hiem nhat (sap ra khoi khung).
"""
import numpy as np
import pytest

from follow.detect import letterbox, postprocess

W, H = 1280, 720
IMGSZ = 416


def lb_params(w=W, h=H, n=IMGSZ):
    """Tra ve (r, dw, dh) giong letterbox()."""
    img = np.zeros((h, w, 3), np.uint8)
    _, r, dw, dh = letterbox(img, n)
    return r, dw, dh


def det_from_box(x1, y1, x2, y2, conf=0.9, cls=0, r=None, dw=0, dh=0):
    """Tao mot hang detection o TOA DO MODEL tu box o toa do khung goc."""
    return [x1 * r + dw, y1 * r + dh, x2 * r + dw, y2 * r + dh, conf, cls]


def run(boxes_goc, conf=0.4, **kw):
    r, dw, dh = lb_params()
    det = np.array([det_from_box(*b, r=r, dw=dw, dh=dh)
                    for b in boxes_goc], np.float32)
    return postprocess(det, r, dw, dh, H, W, conf, **kw)


# ------------------------------------------------------------ co ban
def test_letterbox_giu_ti_le():
    img = np.zeros((720, 1280, 3), np.uint8)
    out, r, dw, dh = letterbox(img, 416)
    assert out.shape == (416, 416, 3)
    assert r == pytest.approx(416 / 1280)
    assert dh > 0 and dw == 0


def test_nguoi_binh_thuong_duoc_giu():
    b, s = run([(600, 200, 680, 500)])          # 80x300, ti le 3.75
    assert len(b) == 1
    assert s[0] == pytest.approx(0.9, abs=0.02)


def test_loai_theo_confidence():
    b, _ = run([(600, 200, 680, 500, 0.2)], conf=0.4)
    assert len(b) == 0


def test_loai_class_khac_nguoi():
    b, _ = run([(600, 200, 680, 500, 0.9, 2)])   # class 2 = xe hoi
    assert len(b) == 0


def test_loai_box_qua_nho():
    b, _ = run([(600, 300, 610, 318)])           # cao 18 px < 24
    assert len(b) == 0


def test_loai_box_qua_beo_o_giua_khung():
    """Vat the rong bet o GIUA khung khong phai nguoi -> loai."""
    b, _ = run([(500, 300, 800, 400)])           # 300x100, ti le 0.33
    assert len(b) == 0


def test_loai_box_qua_cao_gay():
    b, _ = run([(600, 100, 615, 600)])           # 15x500, ti le 33
    assert len(b) == 0


# -------------------------------------------- nguoi o ria khung (fix chinh)
def test_nguoi_bi_cat_o_ria_TRAI_van_duoc_giu():
    """Model bao box vuot ra ngoai khung (x1 am). Sau khi clip, box chi con
    40 px rong / 300 px cao -> ti le van ok. Nhung neu nguoi lech them nua:"""
    b, _ = run([(-60, 200, 40, 500)])            # 100x300 that, clip con 40x300
    assert len(b) == 1
    assert b[0][0] == pytest.approx(0.0, abs=0.5)   # da clip ve 0


def test_nguoi_bi_cat_nhieu_o_ria_PHAI_van_duoc_giu():
    """Chi con 25 px trong khung: sau clip ti le = 300/25 = 12 > MAX_ASPECT.
    Truoc v2.1 se bi loai. Nay box cham bien duoc mien kiem tra ti le."""
    b, _ = run([(1255, 200, 1355, 500)])
    assert len(b) == 1, "nguoi sap ra khoi khung bi loai oan"
    assert b[0][2] == pytest.approx(W, abs=0.5)


def test_nguoi_bi_cat_o_ria_DUOI_van_duoc_giu():
    """Chi thay dau va vai: sau clip thanh box bet -> phai duoc mien."""
    b, _ = run([(600, 640, 700, 900)])
    assert len(b) == 1


def test_vat_the_bet_cham_bien_van_qua_duoc_nhung_can_du_cao():
    """Mien ti le KHONG co nghia la mien chieu cao."""
    b, _ = run([(1250, 300, 1350, 315)])         # cao 15 px
    assert len(b) == 0


# ------------------------------------------------------------ do ben
def test_bo_hang_co_NaN():
    r, dw, dh = lb_params()
    det = np.array([
        det_from_box(600, 200, 680, 500, r=r, dw=dw, dh=dh),
        [np.nan, 0, 10, 10, 0.9, 0],
    ], np.float32)
    b, _ = postprocess(det, r, dw, dh, H, W, 0.4)
    assert len(b) == 1


def test_det_rong_khong_gay_loi():
    r, dw, dh = lb_params()
    b, s = postprocess(np.zeros((0, 6), np.float32), r, dw, dh, H, W)
    assert len(b) == 0 and len(s) == 0
    b, s = postprocess(None, r, dw, dh, H, W)
    assert len(b) == 0


def test_nguong_co_the_chinh_tu_CLI():
    b, _ = run([(600, 300, 680, 340)], min_aspect=0.4, max_aspect=6.0,
               min_box_h=20)
    assert len(b) == 1
