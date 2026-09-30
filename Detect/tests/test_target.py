"""Test TargetManager — reacquire nhap nhang, histogram ngoai hinh."""
import numpy as np
import pytest
from follow.target import Appearance, TargetManager
from follow.track import _Track


def make_track(tid, x1, y1, x2, y2, conf=0.9):
    return _Track(tid, np.array([x1, y1, x2, y2], np.float32), conf)


def test_appearance_embed_not_none():
    """Embed phai tra ve histogram khong rong tu anh hop le."""
    frame = np.random.randint(0, 255, (720, 1280, 3), np.uint8)
    box = np.array([100, 100, 200, 500])
    emb = Appearance.embed(frame, box)
    assert emb is not None
    assert emb.shape == (256,)  # 16*16


def test_appearance_dist_cung_nguoi():
    """Khoang cach giua hai crop giong nhau phai nho."""
    frame = np.random.randint(0, 255, (720, 1280, 3), np.uint8)
    box = np.array([100, 100, 200, 500])
    a = Appearance.embed(frame, box)
    b = Appearance.embed(frame, box)
    assert Appearance.dist(a, b) < 0.01


def test_appearance_dist_none():
    """None -> khoang cach = 1.0."""
    assert Appearance.dist(None, None) == 1.0
    frame = np.random.randint(0, 255, (720, 1280, 3), np.uint8)
    emb = Appearance.embed(frame, np.array([10, 10, 50, 200]))
    assert Appearance.dist(emb, None) == 1.0


def test_pick_at_khoa_muc_tieu():
    """Bam vao nguoi -> khoa thanh cong."""
    tm = TargetManager()
    t1 = make_track(1, 100, 100, 200, 500)
    ok = tm.pick_at(0.12, 0.42, [t1], 1280, 720)
    assert ok
    assert tm.primary == "M1"
    assert len(tm.locked) == 1


def test_pick_at_khong_trung_ai():
    """Bam xa nguoi -> khong khoa."""
    tm = TargetManager()
    t1 = make_track(1, 100, 100, 200, 500)
    ok = tm.pick_at(0.9, 0.9, [t1], 1280, 720)
    assert not ok


def test_unlock_all_xoa_het():
    tm = TargetManager()
    t1 = make_track(1, 100, 100, 200, 500)
    tm.pick_at(0.12, 0.42, [t1], 1280, 720)
    tm.unlock_all()
    assert len(tm.locked) == 0
    assert tm.primary is None


def test_next_primary_xoay_vong():
    tm = TargetManager()
    t1 = make_track(1, 100, 100, 200, 500)
    t2 = make_track(2, 500, 100, 600, 500)
    tm.pick_at(0.12, 0.42, [t1, t2], 1280, 720)
    tm.pick_at(0.43, 0.42, [t1, t2], 1280, 720)
    assert len(tm.locked) == 2
    first = tm.primary
    tm.next_primary()
    assert tm.primary != first


# ==================================================================
# v2.1 — timeout reacquire, tam dung sau khi tim lai, hoc ngoai hinh
# ==================================================================

def solid_frame(color=(40, 200, 90), h=720, w=1280):
    f = np.zeros((h, w, 3), np.uint8)
    f[:, :] = color
    return f


def test_reacquire_timeout_bo_han_muc_tieu():
    """v2.0 bo quen tham so `reacquire`: muc tieu mat dau nam lai vinh vien."""
    tm = TargetManager(reacquire=2.0)
    t1 = make_track(1, 100, 100, 200, 500)
    tm.pick_at(0.12, 0.42, [t1], 1280, 720)
    assert len(tm.locked) == 1

    tm.update([], 1280, 720, now=100.0)          # bat dau mat dau
    assert len(tm.locked) == 1
    assert tm.locked[0].seen is False

    tm.update([], 1280, 720, now=101.5)          # chua qua 2s
    assert len(tm.locked) == 1

    tm.update([], 1280, 720, now=103.0)          # qua 2s -> bo
    assert len(tm.locked) == 0
    assert tm.primary is None
    assert "mat qua" in tm.msg


def test_reacquire_timeout_tat_duoc():
    tm = TargetManager(reacquire=0.0)
    t1 = make_track(1, 100, 100, 200, 500)
    tm.pick_at(0.12, 0.42, [t1], 1280, 720)
    tm.update([], 1280, 720, now=1e6)
    assert len(tm.locked) == 1


def test_mat_muc_tieu_chinh_khong_duoc_tu_doi_sang_nguoi_khac():
    """M1 mat han khong bao gio duoc phep tu dong bam M2.

    Day tung la hanh vi co test bao ve, nhung trai voi yeu cau nguoi van hanh
    chon dung nguoi can theo. Guidance phai ve 0 cho toi khi ho click M2.
    """
    tm = TargetManager(reacquire=1.0)
    t1 = make_track(1, 100, 100, 200, 500)
    t2 = make_track(2, 900, 100, 1000, 500)
    tm.pick_at(0.12, 0.42, [t1, t2], 1280, 720)
    tm.pick_at(0.74, 0.42, [t1, t2], 1280, 720)
    tm.set_primary("M1")
    # M1 mat han, M2 van thay
    tm.update([t2], 1280, 720, now=100.0)
    tm.update([t2], 1280, 720, now=102.0)
    assert [m.nhan for m in tm.locked] == ["M2"]
    assert tm.primary is None
    assert tm.update([t2], 1280, 720, now=102.1) is None


def test_tam_dung_yaw_sau_khi_tim_lai():
    """Re-ID chi tao ung vien; phai click xac nhan truoc khi gan lai M1."""
    frame = solid_frame()
    tm = TargetManager(reacquire=5.0, reacq_hold_s=0.5)
    t1 = make_track(1, 500, 100, 600, 500)
    tm.pick_at(500 / 1280, 300 / 720, [t1], 1280, 720, frame)
    assert not tm.just_reacquired(now=100.0)

    tm.update([], 1280, 720, frame, now=100.0)      # mat dau
    t2 = make_track(2, 510, 100, 610, 500)          # nguoi cu, ID moi
    assert tm.update([t2], 1280, 720, frame, now=100.2) is None
    assert tm.primary_target.tid == 1, "khong duoc tu dong doi ID"
    assert tm.primary_target.pending_tid == 2
    assert tm.primary_target.needs_confirm is True
    assert not tm.just_reacquired(now=100.3)

    # Click vao ung vien = xac nhan co chu y cua nguoi van hanh.
    assert tm.pick_at(560 / 1280, 300 / 720, [t2], 1280, 720, frame,
                      now=100.2)
    assert tm.primary_target.tid == 2
    assert tm.primary_target.needs_confirm is False
    assert tm.just_reacquired(now=100.3) is True
    assert tm.just_reacquired(now=100.9) is False   # het cua so 0.5s


def test_reacquire_nhap_nhang_phai_cho_click_du_lieu_sau_khong_tu_xoa_co():
    """Khong duoc thu lai den khi mot frame sau chi con mot ung vien.

    Neu lam vay, he thong se tu doi nguoi chi vi nguoi thu hai vua khuất —
    dung luc operator dang duoc yeu cau xac nhan.
    """
    frame = solid_frame()
    tm = TargetManager(reacquire=5.0)
    t1 = make_track(1, 500, 100, 600, 500)
    tm.pick_at(550 / 1280, 300 / 720, [t1], 1280, 720, frame)
    tm.update([], 1280, 720, frame, now=100.0)
    t2 = make_track(2, 510, 100, 610, 500)
    t3 = make_track(3, 515, 100, 615, 500)
    assert tm.update([t2, t3], 1280, 720, frame, now=100.2) is None
    m = tm.primary_target
    assert m.needs_confirm is True
    assert m.tid == 1
    assert m.pending_tid is None

    # Sau do chi con t2: van phai dung cho click, khong duoc auto-reacquire.
    assert tm.update([t2], 1280, 720, frame, now=100.3) is None
    assert m.needs_confirm is True and m.tid == 1
    assert tm.pick_at(560 / 1280, 300 / 720, [t2], 1280, 720, frame)
    assert m.tid == 2 and not m.needs_confirm


def test_khong_hoc_ngoai_hinh_tu_box_cham_bien():
    """Box bi cat o ria chi con mot phan nguoi -> hoc vao se lam hong mau."""
    tm = TargetManager()
    assert tm._emb_updatable(np.array([100., 100., 200., 500.]), 1280, 720)
    assert not tm._emb_updatable(np.array([0., 100., 90., 500.]), 1280, 720)
    assert not tm._emb_updatable(np.array([1200., 100., 1280., 500.]),
                                 1280, 720)
    assert not tm._emb_updatable(np.array([100., 100., 115., 500.]),
                                 1280, 720)   # qua hep


def test_pick_at_lai_thi_xoa_co_can_xac_nhan():
    """Nguoi van hanh bam lai dung nguoi = da xac nhan -> gate thoi chan."""
    tm = TargetManager()
    t1 = make_track(1, 100, 100, 200, 500)
    tm.pick_at(0.12, 0.42, [t1], 1280, 720)
    tm.locked[0].needs_confirm = True
    tm.pick_at(0.12, 0.42, [t1], 1280, 720)
    assert tm.locked[0].needs_confirm is False


def test_embed_dung_vung_than_tren():
    """Doi mau rieng phan chan: dac trung than tren KHONG duoc doi theo."""
    f1 = solid_frame((40, 200, 90))
    f2 = f1.copy()
    f2[350:500, 100:200] = (200, 40, 40)        # doi mau nua duoi cua box
    box = np.array([100, 100, 200, 500])
    d = Appearance.dist(Appearance.embed(f1, box), Appearance.embed(f2, box))
    assert d < 0.2, "vung lay dac trung bi lan xuong chan"


def test_embed_bat_duoc_doi_mau_ao():
    f1 = solid_frame((40, 200, 90))
    f2 = f1.copy()
    f2[115:320, 100:200] = (200, 40, 40)        # doi mau than tren
    box = np.array([100, 100, 200, 500])
    d = Appearance.dist(Appearance.embed(f1, box), Appearance.embed(f2, box))
    assert d > 0.5


def test_embed_anh_qua_toi_khong_gay_loi():
    f = np.zeros((720, 1280, 3), np.uint8)      # den hoan toan
    e = Appearance.embed(f, np.array([100, 100, 200, 500]))
    assert e is None or e.shape == (256,)


# ------------------------------------------------------- ByteTrack coasting
def test_track_mat_dau_van_truot_theo_van_toc():
    """v2.0: track mat dau dung yen -> diem neo reacquire lech han so voi
    cho nguoi that su dang di toi."""
    from follow.track import ByteTrack

    tr = ByteTrack(min_hits=1)
    for i in range(4):                          # di deu sang phai 20 px/frame
        x = 100 + 20 * i
        tr.update(np.array([[x, 100, x + 80, 400]], np.float32),
                  np.array([0.9], np.float32))
    x_truoc = tr.tracks[0].box[0]
    tr.update(np.zeros((0, 4), np.float32),
              np.zeros((0,), np.float32))       # mat detection
    assert tr.tracks[0].lost == 1
    assert tr.tracks[0].box[0] > x_truoc + 5, "track khong truot theo van toc"
