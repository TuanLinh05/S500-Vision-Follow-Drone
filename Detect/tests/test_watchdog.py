"""Test LoopWatchdog / CommandAudit / StickMonitor."""
import time

import pytest

from follow.watchdog import (AuditConfig, CommandAudit, LoopWatchdog,
                             StickMonitor)


# ------------------------------------------------------------ LoopWatchdog
def test_watchdog_khong_trip_khi_dap_nhip():
    trips = []
    wd = LoopWatchdog(timeout=0.3, on_trip=lambda age: trips.append(age),
                      poll=0.02)
    wd.start()
    try:
        for _ in range(15):
            wd.beat()
            time.sleep(0.03)
    finally:
        wd.halt()
    assert trips == []
    assert wd.trips == 0


def test_watchdog_trip_khi_main_loop_treo():
    trips = []
    wd = LoopWatchdog(timeout=0.2, on_trip=lambda age: trips.append(age),
                      poll=0.02)
    wd.start()
    try:
        wd.beat()
        time.sleep(0.6)              # mo phong main loop treo
    finally:
        wd.halt()
    assert len(trips) == 1, "phai trip DUNG mot lan cho moi lan treo"
    assert trips[0] >= 0.2
    assert wd.trips == 1


def test_watchdog_trip_lai_sau_khi_hoi_phuc():
    trips = []
    wd = LoopWatchdog(timeout=0.15, on_trip=lambda age: trips.append(age),
                      poll=0.02)
    wd.start()
    try:
        wd.beat()
        time.sleep(0.35)
        wd.beat()                    # hoi phuc
        time.sleep(0.1)
        time.sleep(0.35)             # treo lan hai
    finally:
        wd.halt()
    assert wd.trips == 2


def test_watchdog_khong_chet_khi_on_trip_nem_loi():
    def boom(age):
        raise RuntimeError("loi co y")
    wd = LoopWatchdog(timeout=0.1, on_trip=boom, poll=0.02)
    wd.start()
    try:
        time.sleep(0.3)
        assert wd.is_alive()
    finally:
        wd.halt()


# ------------------------------------------------------------ CommandAudit
def test_audit_lenh_khop_thuc_te_thi_khong_bao():
    a = CommandAudit()
    for _ in range(60):
        assert a.update(0.05, 20.0, 19.0, True) is False


def test_audit_bat_drone_tu_xoay():
    """Lenh ~0 nhung drone van xoay -> co lenh cu song o dau do."""
    a = CommandAudit(AuditConfig(idle_limit_s=1.0))
    bad = False
    for _ in range(40):
        bad = a.update(0.05, 0.0, 40.0, True)
    assert bad
    assert "tu xoay" in a.reason


def test_audit_bat_drone_khong_theo_lenh():
    a = CommandAudit(AuditConfig(limit_s=1.0))
    bad = False
    for _ in range(60):
        bad = a.update(0.05, 30.0, 0.0, True)
    assert bad
    assert "khong theo lenh" in a.reason


def test_audit_khong_bao_khi_chua_engage():
    a = CommandAudit()
    for _ in range(100):
        assert a.update(0.05, 0.0, 90.0, False) is False
    assert a.spin_s == 0.0


def test_audit_bo_dem_giam_khi_binh_thuong_tro_lai():
    a = CommandAudit(AuditConfig(idle_limit_s=1.0))
    for _ in range(10):
        a.update(0.05, 0.0, 40.0, True)     # tich 0.5s
    assert a.spin_s > 0.4
    for _ in range(20):
        a.update(0.05, 0.0, 0.0, True)      # binh thuong tro lai
    assert a.spin_s == 0.0


def test_audit_chan_dt_bat_thuong():
    """dt khong lo (vd sau khi treo) khong duoc lam trip ngay lap tuc."""
    a = CommandAudit(AuditConfig(idle_limit_s=1.0))
    assert a.update(30.0, 0.0, 40.0, True) is False     # dt bi chan ve 0.5


def test_audit_tat_duoc():
    a = CommandAudit(AuditConfig(enabled=False))
    for _ in range(100):
        assert a.update(0.05, 0.0, 90.0, True) is False


# ------------------------------------------------------------ StickMonitor
def test_stick_chua_chot_thi_khong_bao():
    s = StickMonitor()
    assert s.check({1: 1900}) == (False, "")
    assert not s.armed


def test_stick_giu_nguyen_thi_khong_bao():
    s = StickMonitor(threshold=120)
    assert s.arm({1: 1500, 2: 1500, 3: 1200, 4: 1500})
    assert s.check({1: 1560, 2: 1440, 3: 1200, 4: 1500}) == (False, "")


def test_stick_bi_day_thi_bao():
    s = StickMonitor(threshold=120)
    s.arm({1: 1500, 2: 1500, 3: 1200, 4: 1500})
    ov, why = s.check({1: 1500, 2: 1500, 3: 1200, 4: 1750})
    assert ov
    assert "ch4" in why


def test_stick_chot_theo_vi_tri_ENGAGE_khong_phai_1500():
    """Ga o 1200 luc ENGAGE — giu nguyen 1200 KHONG duoc coi la can thiep."""
    s = StickMonitor(chans=(3,), threshold=120)
    s.arm({3: 1200})
    assert s.check({3: 1200})[0] is False
    assert s.check({3: 1400})[0] is True


def test_stick_disarm_xoa_moc():
    s = StickMonitor()
    s.arm({1: 1500, 2: 1500, 3: 1200, 4: 1500})
    s.disarm()
    assert not s.armed
    assert s.check({1: 1900}) == (False, "")


def test_stick_khong_co_du_lieu_RC_thi_khong_bao_oan():
    s = StickMonitor()
    assert s.arm({}) is False
    assert s.check(None) == (False, "")
