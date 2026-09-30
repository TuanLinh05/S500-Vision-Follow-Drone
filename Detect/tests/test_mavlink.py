"""Kiem thu px4.py voi PX4 gia qua UDP loopback (khong can phan cung).

Bao phu: A1 (TX watchdog), A3 (xac minh mode), A6 (loc heartbeat),
A7 (type_mask vi tri), B5 (time_boot_ms), va cac lop v2.1
(clamp tran, chan NaN, ramp mem, idle-follow).
"""

import math
import time

import pytest

from follow.px4 import (MODE_LOITER, MODE_OFFBOARD, PX4_MAIN_AUTO,
                        PX4_MAIN_OFFBOARD, PX4_SUB_LOITER,
                        TYPE_MASK_POS_YAWRATE, TYPE_MASK_VEL_YAWRATE,
                        PX4Link, SetpointStreamer, check_rc_map_deadman_conflict,
                        px4_mode_name)
from follow.watchdog import LoopWatchdog
from tests.fake_px4 import CUSTOM_ENABLED, AUTO_FLAGS, FakePX4, free_udp_port


@pytest.fixture
def px4():
    port = free_udp_port()
    fake = FakePX4(port)
    fake.start()
    link = PX4Link(f"udpin:127.0.0.1:{port}", hb_timeout=8.0)
    yield fake, link
    link.close()
    fake.halt()
    time.sleep(0.05)


def yaw_deg(sp):
    return math.degrees(sp.yaw_rate)


def wait_for(cond, timeout=3.0):
    """Doi den khi cond() dung. Tra ve ket qua cuoi cung."""
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        if cond():
            return True
        time.sleep(0.02)
    return cond()


# ------------------------------------------------------------ giai ma mode
def test_ten_mode_px4_on_dinh():
    """Gate so sanh mode bang chuoi nen ten phai dung, khong phu thuoc
    phien ban pymavlink (mode_string_v10 tra 'LOITER' chu khong 'AUTO.LOITER')."""
    base = CUSTOM_ENABLED | AUTO_FLAGS
    assert px4_mode_name(base, PX4_MAIN_OFFBOARD << 16) == MODE_OFFBOARD
    assert px4_mode_name(
        base, (PX4_MAIN_AUTO << 16) | (PX4_SUB_LOITER << 24)) == MODE_LOITER
    assert px4_mode_name(base, 3 << 16) == "POSCTL"
    assert px4_mode_name(0, 0) == "?"


# ------------------------------------------------------------ ket noi
def test_doc_duoc_telemetry(px4):
    fake, link = px4
    assert wait_for(lambda: link.pos.value is not None
                    and link.mode != "?" and link.rc.value)
    assert link.connected
    assert link.armed is True
    assert link.mode == "POSCTL"
    assert link.pos.value == pytest.approx((0.0, 0.0, -5.0), abs=0.01)
    assert wait_for(lambda: link.xy_pos_health.value is True)
    assert link.hb.age < 1.0
    assert (link.rc.value or {}).get(7) == 1800


def test_doc_suc_khoe_uoc_luong_XY(px4):
    fake, link = px4
    assert wait_for(lambda: link.xy_pos_health.value is True)
    fake.xy_pos_healthy = False
    assert wait_for(lambda: link.xy_pos_health.value is False, timeout=2.0)


def test_bo_qua_heartbeat_cua_GCS(px4):
    """A6: mot QGC tren cung link khong duoc lam sai mode/armed."""
    fake, link = px4
    assert wait_for(lambda: link.mode == "POSCTL")
    fake.send_gcs_heartbeat = True
    time.sleep(1.0)
    assert link.mode == "POSCTL"      # khong bi GCS ghi de
    assert link.armed is True


def test_failsafe_tu_MAV_STATE(px4):
    fake, link = px4
    assert wait_for(lambda: link.mode == "POSCTL")
    assert link.failsafe is False
    fake.sys_state = 6                 # MAV_STATE_EMERGENCY
    time.sleep(0.5)
    assert link.failsafe is True


# ------------------------------------------------------------ EKF reset (v2.3)
def test_doc_duoc_odometry_reset_counter(px4):
    """Can MAVLink2 (id > 255) - da ep o follow/__init__.py."""
    fake, link = px4
    assert wait_for(lambda: link.reset_counter.value is not None)
    assert link.reset_counter.value == 0
    fake.odom_reset_counter = 7
    assert wait_for(lambda: link.reset_counter.value == 7, timeout=2.0)


def test_reset_counter_wraparound_uint8(px4):
    fake, link = px4
    assert wait_for(lambda: link.reset_counter.value is not None)
    fake.odom_reset_counter = 257       # 257 & 0xFF == 1
    assert wait_for(lambda: link.reset_counter.value == 1, timeout=2.0)


# ------------------------------------------------------------ parameter (v2.3)
def test_read_param_giai_ma_INT32_byte_wise(px4):
    """RC_MAP_* la INT32 - PX4 ma hoa byte-wise trong truong float."""
    fake, link = px4
    assert wait_for(lambda: link.mode == "POSCTL")
    assert link.read_param("RC_MAP_ARM_SW", timeout=2.0) == 5
    assert link.read_param("RC_MAP_FLTMODE", timeout=2.0) == 9


def test_read_param_khong_ton_tai_tra_ve_None_khong_treo(px4):
    fake, link = px4
    t0 = time.monotonic()
    assert link.read_param("KHONG_TON_TAI", timeout=0.6) is None
    assert (time.monotonic() - t0) < 2.0


def test_khong_xung_dot_deadman_voi_mapping_mac_dinh(px4):
    """Mapping mac dinh cua FakePX4 khong trung kenh dead-man 7."""
    fake, link = px4
    assert wait_for(lambda: link.mode == "POSCTL")
    conflicts, unverified = check_rc_map_deadman_conflict(
        link, 7, timeout_each=1.0)
    assert conflicts == []
    assert unverified == []


def test_phat_hien_xung_dot_RC_MAP_FLTMODE(px4):
    """Tinh huong thuc te da gap tren phan cung: CH7 vua la dead-man vua la
    RC_MAP_FLTMODE -> ha dead-man se dong thoi doi flight mode."""
    fake, link = px4
    assert wait_for(lambda: link.mode == "POSCTL")
    fake.params["RC_MAP_FLTMODE"] = (7, 6)      # MAV_PARAM_TYPE_INT32
    conflicts, unverified = check_rc_map_deadman_conflict(
        link, 7, timeout_each=1.0)
    assert len(conflicts) == 1
    assert "RC_MAP_FLTMODE" in conflicts[0]
    assert "7" in conflicts[0]


def test_xung_dot_deadman_khong_kiem_tra_khi_rc_chan_0():
    """rc_chan=0 (khong dung dead-man) -> khong can doc parameter nao."""
    conflicts, unverified = check_rc_map_deadman_conflict(None, 0)
    assert conflicts == [] and unverified == []


def test_param_khong_doc_duoc_bao_la_unverified_khong_phai_sach(px4):
    """Khong the phan biet 'da kiem tra sach' voi 'khong doc duoc' - phai
    tra ve rieng, KHONG duoc lang le coi la khong xung dot."""
    fake, link = px4
    assert wait_for(lambda: link.mode == "POSCTL")
    del fake.params["RC_MAP_KILL_SW"]           # PX4 se khong tra loi
    conflicts, unverified = check_rc_map_deadman_conflict(
        link, 7, timeout_each=0.6)
    assert conflicts == []
    assert "RC_MAP_KILL_SW" in unverified


# ------------------------------------------------------------ doi mode
def test_vao_offboard_duoc_xac_nhan(px4):
    fake, link = px4
    assert wait_for(lambda: link.mode == "POSCTL")
    assert link.enter_offboard() is True
    assert link.mode == MODE_OFFBOARD


def test_offboard_bi_tu_choi_thi_tra_False(px4):
    """A3: PX4 tra TEMPORARILY_REJECTED -> KHONG duoc bao la da engaged."""
    fake, link = px4
    assert wait_for(lambda: link.mode == "POSCTL")
    fake.reject_mode = True
    assert link.enter_offboard() is False
    assert link.mode != MODE_OFFBOARD


def test_loiter_duoc_xac_nhan_bang_heartbeat(px4):
    """A2: enter_loiter phai xac minh mode that su doi, khong chi tin ACK."""
    fake, link = px4
    assert wait_for(lambda: link.mode == "POSCTL")
    assert link.enter_offboard() is True
    assert link.enter_loiter() is True
    assert link.mode == MODE_LOITER


def test_loiter_khong_giat_quyen_khi_khong_o_offboard(px4):
    """A2: dang o POSCTL (phi cong da gianh quyen) -> KHONG gui doi mode."""
    fake, link = px4
    assert wait_for(lambda: link.mode == "POSCTL")
    n_truoc = len(fake.mode_cmds)
    assert link.enter_loiter() is True
    time.sleep(0.3)
    assert len(fake.mode_cmds) == n_truoc
    assert link.mode == "POSCTL"


def test_doi_mode_async_khong_chan(px4):
    fake, link = px4
    assert wait_for(lambda: link.mode == "POSCTL")
    t0 = time.monotonic()
    assert link.enter_offboard_async() is True
    assert (time.monotonic() - t0) < 0.2      # tra ve ngay
    t_end = time.monotonic() + 4.0
    while time.monotonic() < t_end and link.mode != MODE_OFFBOARD:
        time.sleep(0.05)
    assert link.mode == MODE_OFFBOARD


# ------------------------------------------------------------ streamer
@pytest.fixture
def streamer(px4):
    fake, link = px4
    s = SetpointStreamer(link, rate_hz=20.0, max_rate=40.0, ramp_s=0.0)
    s.start()
    yield fake, link, s
    s.halt()
    time.sleep(0.1)


def test_A1_watchdog_ep_yaw_ve_0(streamer):
    """A1: khong nuoi lenh -> setpoint phai ve 0 trong ~0.25s."""
    fake, link, s = streamer
    s.arm_hold((1.0, 2.0, -5.0))
    for _ in range(10):
        s.command(30.0)
        time.sleep(0.03)
    assert yaw_deg(fake.fresh_setpoint()) == pytest.approx(30.0, abs=0.5)

    time.sleep(0.6)                      # mo phong main loop treo
    sps = fake.next_setpoints(5)
    assert sps, "khong nhan duoc setpoint nao"
    assert all(abs(yaw_deg(sp)) < 0.01 for sp in sps)
    assert s.watchdog_trips >= 1


def test_A7_type_mask_vi_tri_yawrate(streamer):
    """A7: co diem chot -> setpoint VI TRI, khong phai van toc 0."""
    fake, link, s = streamer
    s.arm_hold((10.0, -4.0, -25.0))
    s.command(0.0)
    sp = fake.fresh_setpoint()
    assert sp.type_mask == TYPE_MASK_POS_YAWRATE
    assert (sp.x, sp.y, sp.z) == pytest.approx((10.0, -4.0, -25.0), abs=0.01)


def test_idle_follow_giu_vi_tri_hien_tai(streamer):
    """v2.1: chua ENGAGE van phat setpoint GIU VI TRI hien tai.
    Neu PX4 bat ngo vao Offboard thi drone dung yen chu khong troi."""
    fake, link, s = streamer
    fake.pos = (7.0, -3.0, -12.0)
    assert wait_for(lambda: link.pos.value == pytest.approx(
        (7.0, -3.0, -12.0), abs=0.01))
    s.command(0.0)
    sp = fake.fresh_setpoint()
    assert sp.type_mask == TYPE_MASK_POS_YAWRATE
    assert (sp.x, sp.y, sp.z) == pytest.approx((7.0, -3.0, -12.0), abs=0.01)


def test_khong_idle_follow_thi_dung_van_toc(px4):
    fake, link = px4
    s = SetpointStreamer(link, rate_hz=20.0, idle_follow=False, ramp_s=0.0)
    s.start()
    try:
        s.command(0.0)
        sp = fake.fresh_setpoint()
        assert sp.type_mask == TYPE_MASK_VEL_YAWRATE
    finally:
        s.halt()


def test_B5_time_boot_ms_khong_reset(streamer):
    """B5: timestamp phai tang don dieu qua moc 1 giay."""
    fake, link, s = streamer
    s.command(0.0)
    time.sleep(1.4)
    sps = fake.next_setpoints(10)
    ts = [sp.time_boot_ms for sp in sps]
    assert all(b >= a for a, b in zip(ts, ts[1:]))
    assert ts[-1] > 1000


def test_clamp_tran_yaw(streamer):
    """v2.1: lenh vuot tran bi cat NGAY TRUOC pymavlink (lop cuoi cung)."""
    fake, link, s = streamer
    s.max_rate = 15.0
    s.arm_hold((0.0, 0.0, -5.0))
    for _ in range(6):
        s.command(500.0)
        time.sleep(0.03)
    sp = fake.fresh_setpoint()
    assert abs(yaw_deg(sp)) <= 15.0 + 1e-6
    assert s.clamp_hits > 0


def test_chan_NaN(streamer):
    """v2.1: NaN khong bao gio duoc phep ra khoi may bay."""
    fake, link, s = streamer
    s.arm_hold((0.0, 0.0, -5.0))
    for _ in range(6):
        s.command(float("nan"))
        time.sleep(0.03)
    sp = fake.fresh_setpoint()
    assert math.isfinite(sp.yaw_rate)
    assert yaw_deg(sp) == pytest.approx(0.0, abs=1e-6)
    assert s.nan_blocks > 0


def test_ramp_mem_sau_engage(px4):
    """v2.1: ngay sau ENGAGE, tran yaw tang dan tu 0 chu khong nhay len max."""
    fake, link = px4
    s = SetpointStreamer(link, rate_hz=20.0, max_rate=40.0, ramp_s=1.0)
    s.start()
    try:
        s.arm_hold((0.0, 0.0, -5.0))     # bat dau ramp
        s.command(40.0)
        time.sleep(0.15)
        sp = fake.fresh_setpoint()
        assert abs(yaw_deg(sp)) < 20.0, "ramp khong hoat dong"
        t_end = time.monotonic() + 1.5
        while time.monotonic() < t_end:
            s.command(40.0)
            time.sleep(0.03)
        assert abs(yaw_deg(fake.fresh_setpoint())) == pytest.approx(40.0, abs=1.0)
    finally:
        s.halt()


def test_halt_dung_han_luong_setpoint(streamer):
    """--on-abort stream-off: PX4 phai ngung nhan setpoint."""
    fake, link, s = streamer
    s.command(0.0)
    assert fake.fresh_setpoint() is not None
    s.halt()
    time.sleep(0.3)
    with fake._lk:
        fake.setpoints.clear()
    time.sleep(0.4)
    with fake._lk:
        assert len(fake.setpoints) == 0


def test_pause_ngung_TX_va_resume_duoc(streamer):
    """Roi Offboard thanh cong thi idle phai 0 Hz, ENGAGE sau van dung duoc."""
    fake, link, s = streamer
    s.arm_hold((1.0, 2.0, -5.0))
    s.command(10.0)
    assert fake.fresh_setpoint() is not None

    assert s.pause() is True
    assert s.paused is True
    time.sleep(0.15)
    with fake._lk:
        fake.setpoints.clear()
    time.sleep(0.35)
    with fake._lk:
        assert len(fake.setpoints) == 0
    assert s.sp_hz == 0.0
    assert s.command(10.0) is False

    assert s.resume() is True
    assert s.paused is False
    s.arm_hold((3.0, 4.0, -5.0))
    s.command(5.0)
    sp = fake.fresh_setpoint()
    assert sp is not None
    assert yaw_deg(sp) == pytest.approx(5.0, abs=0.5)


def test_emergency_stop_gui_zero_roi_cat_stream(streamer):
    """Watchdog ngoai main loop phai co duong cat stream doc lap.

    Khong chi dem watchdog hay dat mot latch cho app: luong TX phai tu gui
    yaw=0 mot lan, dung han, va tu choi moi lenh sau do.
    """
    fake, link, s = streamer
    s.arm_hold((1.0, 2.0, -5.0))
    for _ in range(8):
        s.command(25.0)
        time.sleep(0.03)
    assert abs(yaw_deg(fake.fresh_setpoint())) > 20.0

    with fake._lk:
        fake.setpoints.clear()
    assert s.emergency_stop("test watchdog") is True
    assert wait_for(lambda: s.halted, timeout=1.0)
    assert s.emergency_requested
    assert s.emergency_reason == "test watchdog"
    # UDP receiver xu ly packet o luong rieng; `halted` la trang thai cua TX
    # sender, khong dong nghia receiver da kip dua packet vao deque.
    def emergency_zero_seen():
        with fake._lk:
            return bool(fake.setpoints) and abs(yaw_deg(fake.setpoints[-1])) < 0.01
    assert wait_for(emergency_zero_seen, timeout=0.5)
    with fake._lk:
        sent = list(fake.setpoints)
    assert sent, "emergency stop phai gui mot setpoint yaw=0 truoc khi cat"
    assert abs(yaw_deg(sent[-1])) < 0.01
    assert s.command(10.0) is False

    with fake._lk:
        fake.setpoints.clear()
    time.sleep(0.25)
    with fake._lk:
        assert not fake.setpoints, "da emergency thi khong duoc gui them setpoint"


def test_loop_watchdog_cat_stream_khi_main_khong_the_hoi_phuc(streamer):
    """Fault-injection gan voi app: watchdog va TX la hai luong doc lap."""
    fake, link, s = streamer
    s.arm_hold((0.0, 0.0, -5.0))
    s.command(20.0)
    assert abs(yaw_deg(fake.fresh_setpoint())) > 15.0
    with fake._lk:
        fake.setpoints.clear()

    wd = LoopWatchdog(
        timeout=0.12,
        on_trip=lambda age: s.emergency_stop(f"loop treo {age:.2f}s"),
        poll=0.01)
    wd.start()
    try:
        # Khong beat nua: mo phong req.wait()/cv2 bi ket vo han. Khong can
        # main loop quay lai de luong streamer phai tu dung.
        assert wait_for(lambda: s.halted, timeout=1.0)
        assert wd.trips == 1
        assert s.emergency_requested
        def emergency_zero_seen():
            with fake._lk:
                return (bool(fake.setpoints)
                        and abs(yaw_deg(fake.setpoints[-1])) < 0.01)
        assert wait_for(emergency_zero_seen, timeout=0.5)
        with fake._lk:
            sent = list(fake.setpoints)
        assert sent and abs(yaw_deg(sent[-1])) < 0.01
    finally:
        wd.halt()
