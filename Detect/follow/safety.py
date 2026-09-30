"""Gate an toan - THUAN TUY: khong I/O, khong doc dong ho he thong.
Moi dau vao truyen qua GateInputs de kiem thu duoc bang bang du lieu.

v2.1 bo sung cac lop chong "bay mat kiem soat" theo Plan muc 7:
  - tuoi cua uoc luong vi tri (khong chi "co hay khong")
  - hang rao mem (geofence.py): warn -> zero yaw, breach -> disengage
  - phi cong dong stick (RC override) -> tra quyen ngay
  - gioi han thoi luong moi lan engage
  - PX4 bao failsafe / trang thai nguy cap
  - pin theo phan tram, khong chi theo volt
  - doi chieu lenh voi chuyen dong that (watchdog.CommandAudit)
  - lenh khong huu han (NaN/Inf)
  - so lan TX watchdog trip
"""

from dataclasses import dataclass, field
from typing import List

# muc do hang rao - dong bo voi geofence.py
FENCE_OK, FENCE_WARN, FENCE_BREACH = "ok", "warn", "breach"


@dataclass(frozen=True)
class GateInputs:
    # y dinh cua nguoi van hanh
    engaged: bool
    operator_age: float          # giay ke tu nhip /alive cuoi cung

    # lien ket & trang thai PX4
    link_connected: bool
    hb_age: float
    armed: bool
    mode: str                    # vd "OFFBOARD", "POSCTL"
    pos_valid: bool              # co LOCAL_POSITION_NED moi
    drift_m: float               # lech so voi diem chot khi engage

    # RC / pin (KEM tuoi du lieu)
    rc_value: int
    rc_age: float
    batt_v: float
    batt_age: float

    # vision
    cam_age: float
    has_target: bool
    target_age: float
    target_needs_confirm: bool

    # suc khoe he thong
    loop_age: float              # giay ke tu nhip main loop cuoi
    tx_watchdog_trips: int

    # ---- v2.1: cac truong moi deu co mac dinh "an toan" de tuong thich nguoc
    pos_age: float = 0.0         # tuoi cua LOCAL_POSITION_NED
    vel_age: float = 0.0         # tuoi cua van toc LOCAL_POSITION_NED
    att_age: float = 0.0         # tuoi cua ATTITUDE / yawspeed
    xy_pos_healthy: bool = True  # SYS_STATUS: XY position control healthy
    estimator_age: float = 0.0   # tuoi cua co health noi tren
    fence_level: str = FENCE_OK  # ok / warn / breach
    fence_reason: str = ""
    rc_override: bool = False    # phi cong dang dong stick
    engage_s: float = 0.0        # da engage bao lau
    px4_failsafe: bool = False   # MAV_STATE critical/emergency
    batt_pct: float = -1.0       # -1 = khong biet
    batt_pct_age: float = 0.0    # tuoi cua phan tram pin
    cmd_finite: bool = True      # lenh yaw la so huu han
    audit_bad: bool = False      # CommandAudit phat hien bat thuong
    audit_reason: str = ""
    emergency_stop: bool = False # watchdog da cat stream, bat buoc restart
    emergency_reason: str = ""
    # v2.3: ODOMETRY.reset_counter da doi so voi luc chot ENGAGE (EKF reset
    # vi tri/van toc/huong giua chung). 0 = chua doi hoac chua co du lieu.
    reset_counter_delta: int = 0


@dataclass(frozen=True)
class GateConfig:
    enable_control: bool
    rc_chan: int = 0
    rc_min: int = 1500
    rc_max_age: float = 0.5
    batt_min: float = 0.0
    batt_min_pct: float = 0.0    # 0 = khong kiem tra
    batt_max_age: float = 5.0
    hb_timeout: float = 2.0
    frame_timeout: float = 1.0
    vision_timeout: float = 0.6
    operator_timeout: float = 1.0
    loop_timeout: float = 0.5
    pos_max_age: float = 1.0
    vel_max_age: float = 1.0
    att_max_age: float = 0.5
    estimator_max_age: float = 2.0
    max_drift_m: float = 3.0
    max_engage_s: float = 0.0    # 0 = khong gioi han
    max_tx_trips: int = 1        # >= so nay -> fatal. 0 = khong kiem tra
    require_offboard: bool = True
    require_rc_deadman: bool = True


@dataclass
class GateResult:
    allow: bool
    reason: str = ""             # ly do dau tien, hien tren web
    all_reasons: List[str] = field(default_factory=list)
    fatal: bool = False          # True -> disengage, khong chi zero yaw


def check(cfg: GateConfig, s: GateInputs) -> GateResult:
    """Tra ve GateResult. `fatal` phan biet hai loai chan:
   - khong fatal (vd mat muc tieu tam thoi) -> yaw ve 0, GIU engaged
   - fatal (vd mat heartbeat)               -> disengage + thoat offboard
    """
    reasons: List[str] = []
    fatal = False

    def add(msg: str, is_fatal: bool = False):
        nonlocal fatal
        reasons.append(msg)
        fatal = fatal or is_fatal

    # --- lop 1: co bat
    if not cfg.enable_control:
        add("chua bat --enable-control")
    elif cfg.require_rc_deadman and not cfg.rc_chan:
        # Khong cho phep bo qua dead-man do quen CLI. SITL/bench phai dung co
        # acknowledge rieng, khong duoc vo tinh mang len may bay that.
        add("chua cau hinh RC dead-man", True)

    # --- lop 2: lien ket
    if not s.link_connected:
        add("chua ket noi MAVLink", True)
    if s.hb_age > cfg.hb_timeout:
        add(f"mat heartbeat {s.hb_age:.1f}s", True)

    # --- lop 3: trang thai bay
    if not s.armed:
        add("drone chua arm", True)
    if not s.pos_valid:
        add("chua co uoc luong vi tri", True)
    elif s.pos_age > cfg.pos_max_age:
        add(f"du lieu vi tri cu {s.pos_age:.1f}s", True)
    if not s.xy_pos_healthy:
        add("uoc luong vi tri XY khong healthy", True)
    elif s.estimator_age > cfg.estimator_max_age:
        add(f"du lieu suc khoe estimator cu {s.estimator_age:.1f}s", True)
    if s.engaged and s.vel_age > cfg.vel_max_age:
        add(f"du lieu van toc cu {s.vel_age:.1f}s", True)
    if s.engaged and s.att_age > cfg.att_max_age:
        add(f"du lieu attitude cu {s.att_age:.1f}s", True)
    if s.px4_failsafe:
        add("PX4 dang bao failsafe", True)
    if cfg.require_offboard and s.engaged and s.mode != "OFFBOARD":
        add(f"PX4 khong o OFFBOARD (dang: {s.mode})", True)
    if s.drift_m > cfg.max_drift_m:
        add(f"troi {s.drift_m:.1f}m khoi diem chot", True)
    if s.reset_counter_delta:
        add(f"EKF dat lai vi tri/van toc ({s.reset_counter_delta} lan) "
            f"trong khi giu ENGAGE", True)

    # --- lop 4: dead-man RC (KIEM TRA TUOI TRUOC)
    if cfg.rc_chan:
        if s.rc_age > cfg.rc_max_age:
            add(f"du lieu RC cu {s.rc_age:.1f}s", True)
        elif s.rc_value < cfg.rc_min:
            add(f"cong tac RC ch{cfg.rc_chan}={s.rc_value} "
                f"(can >={cfg.rc_min})", True)

    # --- lop 4b: phi cong gianh quyen bang stick -> tra quyen NGAY
    if s.rc_override:
        add("phi cong dang dong stick", True)

    # --- lop 5: pin
    if cfg.batt_min > 0:
        if s.batt_age > cfg.batt_max_age:
            add(f"du lieu pin cu {s.batt_age:.1f}s", True)
        elif s.batt_v <= 0:
            add("du lieu dien ap pin khong hop le", True)
        elif s.batt_v < cfg.batt_min:
            add(f"pin thap {s.batt_v:.1f}V", True)
    if cfg.batt_min_pct > 0 and 0 <= s.batt_pct < cfg.batt_min_pct:
        if s.batt_pct_age > cfg.batt_max_age:
            add(f"du lieu phan tram pin cu {s.batt_pct_age:.1f}s", True)
        else:
            add(f"pin con {s.batt_pct:.0f}% "
                f"(can >={cfg.batt_min_pct:.0f}%)", True)
    elif cfg.batt_min_pct > 0 and s.batt_pct >= 0 \
            and s.batt_pct_age > cfg.batt_max_age:
        add(f"du lieu phan tram pin cu {s.batt_pct_age:.1f}s", True)

    # --- lop 6: hang rao mem (chong bay mat)
    if s.fence_level == FENCE_BREACH:
        add(f"vuot hang rao mem: {s.fence_reason}", True)
    elif s.fence_level == FENCE_WARN:
        # canh bao: dung yaw nhung GIU engaged de phi cong kip xu ly
        add(f"canh bao hang rao: {s.fence_reason}")

    # --- lop 7: suc khoe phan mem
    if s.cam_age > cfg.frame_timeout:
        add(f"mat camera {s.cam_age:.1f}s", True)
    if s.loop_age > cfg.loop_timeout:
        add(f"vong lap treo {s.loop_age:.1f}s", True)
    if cfg.max_tx_trips > 0 and s.tx_watchdog_trips >= cfg.max_tx_trips:
        add(f"TX watchdog da trip {s.tx_watchdog_trips} lan", True)
    if not s.cmd_finite:
        add("lenh yaw khong huu han (NaN/Inf)", True)
    if s.audit_bad:
        add(s.audit_reason or "lenh khong khop chuyen dong that", True)
    if s.emergency_stop:
        add(s.emergency_reason or "watchdog da dung khan cap stream setpoint",
            True)

    # --- lop 8: nguoi van hanh
    if not s.engaged:
        add("chua bam ENGAGE")
    if s.engaged and s.operator_age > cfg.operator_timeout:
        add(f"mat nhip nguoi van hanh {s.operator_age:.1f}s", True)
    if s.engaged and cfg.max_engage_s > 0 and s.engage_s > cfg.max_engage_s:
        add(f"het thoi luong engage {cfg.max_engage_s:.0f}s", True)

    # --- lop 9: muc tieu (KHONG fatal - chi zero yaw)
    if not s.has_target:
        add("chua khoa muc tieu")
    elif s.target_needs_confirm:
        add("can xac nhan lai muc tieu")
    elif s.target_age > cfg.vision_timeout:
        add(f"muc tieu cu {s.target_age:.1f}s")

    return GateResult(allow=not reasons,
                      reason=reasons[0] if reasons else "",
                      all_reasons=reasons,
                      fatal=fatal)
