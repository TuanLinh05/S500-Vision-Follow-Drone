"""Test SafetyGate — moi dieu kien chan phai duoc kiem tra doc lap."""
import pytest
from follow.safety import check, GateConfig, GateInputs


def ok_inputs(**kw):
    """Tao GateInputs hop le, ghi de bang kw."""
    base = dict(
        engaged=True, operator_age=0.1,
        link_connected=True, hb_age=0.2, armed=True,
        mode="OFFBOARD", pos_valid=True, drift_m=0.3,
        rc_value=1800, rc_age=0.1,
        batt_v=15.2, batt_age=0.5,
        cam_age=0.05, has_target=True, target_age=0.1,
        target_needs_confirm=False,
        loop_age=0.02, tx_watchdog_trips=0,
    )
    base.update(kw)
    return GateInputs(**base)


CFG = GateConfig(enable_control=True, rc_chan=7, rc_min=1500, batt_min=14.0)


def test_duong_co_ban():
    """Tat ca dieu kien tot -> cho phep."""
    assert check(CFG, ok_inputs()).allow


def test_khong_enable_control():
    cfg2 = GateConfig(enable_control=False)
    r = check(cfg2, ok_inputs())
    assert not r.allow
    assert "enable-control" in r.reason


@pytest.mark.parametrize("kw,frag,fatal", [
    ({"link_connected": False},       "chua ket noi MAVLink",   True),
    ({"hb_age": 9.0},                 "mat heartbeat",          True),
    ({"armed": False},                "chua arm",               True),
    ({"pos_valid": False},            "uoc luong vi tri",       True),   # A7
    ({"xy_pos_healthy": False},       "vi tri XY",              True),
    ({"estimator_age": 3.0},          "estimator cu",           True),
    ({"mode": "POSCTL"},              "khong o OFFBOARD",       True),   # A3
    ({"drift_m": 5.0},                "troi",                   True),   # A7
    ({"rc_value": 1200},              "cong tac RC",            True),
    ({"rc_age": 3.0},                 "du lieu RC cu",          True),   # A4
    ({"batt_v": 13.1},                "pin thap",               True),
    ({"batt_v": 0.0},                 "dien ap pin",             True),
    ({"batt_age": 30.0},              "du lieu pin cu",         True),
    ({"cam_age": 2.0},                "mat camera",             True),
    ({"loop_age": 1.5},               "vong lap treo",          True),   # A1
    ({"engaged": True, "operator_age": 3.0}, "nhip nguoi van hanh", True),  # A5
    ({"has_target": False},           "chua khoa",              False),
    ({"target_age": 2.0},             "muc tieu cu",            False),
    ({"target_needs_confirm": True},  "xac nhan lai",           False),  # C3
])
def test_tung_dieu_kien_chan_doc_lap(kw, frag, fatal):
    r = check(CFG, ok_inputs(**kw))
    assert not r.allow
    assert frag in r.reason or any(frag in x for x in r.all_reasons)
    assert r.fatal is fatal


def test_nhieu_dieu_kien_chan():
    """Nhieu dieu kien hong -> ghi tat ca, fatal neu co bat ky fatal."""
    r = check(CFG, ok_inputs(armed=False, has_target=False))
    assert not r.allow
    assert len(r.all_reasons) >= 2
    assert r.fatal  # armed=False la fatal


def test_rc_chan_0_khong_kiem_tra_rc():
    """SITL/bench co acknowledge rieng moi duoc tat dead-man RC."""
    cfg2 = GateConfig(enable_control=True, rc_chan=0,
                      require_rc_deadman=False)
    r = check(cfg2, ok_inputs(rc_value=0, rc_age=999.0))
    # chi bi chan boi cac dieu kien khac, khong phai RC
    assert all("RC" not in reason for reason in r.all_reasons)


def test_control_that_buoc_phai_co_rc_deadman():
    r = check(GateConfig(enable_control=True, rc_chan=0), ok_inputs())
    assert not r.allow and r.fatal
    assert "RC dead-man" in " ".join(r.all_reasons)


def test_batt_min_0_khong_kiem_tra_pin():
    """batt_min=0 la tat kiem tra pin."""
    cfg2 = GateConfig(enable_control=True, rc_chan=7, rc_min=1500, batt_min=0.0)
    r = check(cfg2, ok_inputs(batt_v=5.0))
    assert all("pin" not in reason for reason in r.all_reasons)


def test_mode_check_chi_khi_engaged():
    """Khi chua engaged, khong kiem tra mode OFFBOARD."""
    r = check(CFG, ok_inputs(engaged=False, mode="POSCTL"))
    assert all("OFFBOARD" not in reason for reason in r.all_reasons)


def test_operator_age_chi_khi_engaged():
    """Khi chua engaged, khong kiem tra operator age."""
    r = check(CFG, ok_inputs(engaged=False, operator_age=999.0))
    assert all("nguoi van hanh" not in reason for reason in r.all_reasons)


# ==================================================================
# v2.1 — cac lop chong "bay mat kiem soat"
# ==================================================================

CFG21 = GateConfig(enable_control=True, rc_chan=7, rc_min=1500,
                   batt_min=14.0, batt_min_pct=20.0, max_engage_s=20.0,
                   max_tx_trips=1, pos_max_age=1.0)


@pytest.mark.parametrize("kw,frag,fatal", [
    # tuoi du lieu vi tri — truoc day chi kiem tra "co hay khong"
    ({"pos_age": 3.0},                 "du lieu vi tri cu",     True),
    ({"vel_age": 3.0},                 "du lieu van toc cu",    True),
    ({"att_age": 3.0},                 "du lieu attitude cu",   True),
    # hang rao mem
    ({"fence_level": "breach",
      "fence_reason": "vuot fence 18.2m"}, "vuot hang rao mem", True),
    ({"fence_level": "warn",
      "fence_reason": "gan bien fence"},   "canh bao hang rao",  False),
    # phi cong gianh quyen
    ({"rc_override": True},            "dong stick",            True),
    # PX4 tu bao failsafe
    ({"px4_failsafe": True},           "failsafe",              True),
    # pin theo phan tram
    ({"batt_pct": 12.0},               "pin con",               True),
    # het thoi luong engage
    ({"engage_s": 25.0},               "het thoi luong engage",  True),
    # TX watchdog da trip
    ({"tx_watchdog_trips": 1},         "TX watchdog",           True),
    # lenh khong huu han
    ({"cmd_finite": False},            "khong huu han",         True),
    # doi chieu lenh voi chuyen dong that
    ({"audit_bad": True,
      "audit_reason": "drone tu xoay +40 do/s"}, "tu xoay",      True),
    ({"emergency_stop": True,
      "emergency_reason": "watchdog cat stream"}, "watchdog cat", True),
    # v2.3: EKF reset vi tri/van toc giua chung ENGAGE (ODOMETRY.reset_counter)
    ({"reset_counter_delta": 1},        "EKF dat lai",           True),
])
def test_lop_moi_chan_doc_lap(kw, frag, fatal):
    r = check(CFG21, ok_inputs(**kw))
    assert not r.allow
    assert frag in r.reason or any(frag in x for x in r.all_reasons)
    assert r.fatal is fatal


def test_fence_warn_khong_disengage():
    """Canh bao hang rao: dung yaw nhung GIU engaged de phi cong kip xu ly."""
    r = check(CFG21, ok_inputs(fence_level="warn", fence_reason="gan bien"))
    assert not r.allow
    assert not r.fatal


def test_fence_breach_thi_fatal():
    r = check(CFG21, ok_inputs(fence_level="breach", fence_reason="vuot"))
    assert not r.allow
    assert r.fatal


def test_pin_phan_tram_tat_khi_bang_0():
    cfg = GateConfig(enable_control=True, rc_chan=7, batt_min=14.0,
                     batt_min_pct=0.0)
    assert check(cfg, ok_inputs(batt_pct=5.0)).allow


def test_pin_phan_tram_khong_biet_thi_bo_qua():
    """batt_pct = -1 nghia la PX4 khong bao -> khong duoc chan oan."""
    assert check(CFG21, ok_inputs(batt_pct=-1.0)).allow


def test_pin_phan_tram_cu_phai_chan_du_biet_gia_tri():
    r = check(CFG21, ok_inputs(batt_pct=88.0, batt_pct_age=9.0))
    assert not r.allow and r.fatal
    assert "phan tram pin cu" in " ".join(r.all_reasons)


def test_thoi_luong_engage_chi_ap_khi_dang_engage():
    r = check(CFG21, ok_inputs(engaged=False, engage_s=99.0))
    assert "het thoi luong" not in " ".join(r.all_reasons)


def test_tx_trips_tat_khi_bang_0():
    cfg = GateConfig(enable_control=True, rc_chan=7, batt_min=14.0,
                     max_tx_trips=0)
    assert check(cfg, ok_inputs(tx_watchdog_trips=5)).allow


def test_reset_counter_delta_0_khong_chan():
    """Mac dinh (chua doi hoac chua co du lieu ODOMETRY) -> khong chan."""
    assert check(CFG21, ok_inputs(reset_counter_delta=0)).allow


def test_nhieu_lop_cung_hong_van_bao_het():
    r = check(CFG21, ok_inputs(fence_level="breach", fence_reason="vuot",
                               rc_override=True, px4_failsafe=True))
    assert not r.allow and r.fatal
    assert len(r.all_reasons) >= 3
