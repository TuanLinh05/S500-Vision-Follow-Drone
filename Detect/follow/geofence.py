"""Hang rao mem (soft fence) cua companion - THUAN TUY, khong I/O.

Muc dich: chan drone bay ra khoi vung thu truoc khi cham hard fence cua PX4.
Theo Plan/ke_hoach_dieu_khien_up7000_pixhawk_usb.md muc 7.4:
  - soft fence companion : ban kinh ~15 m, tran AI 8-10 m
  - hard fence PX4       : ban kinh ~40 m, tran ~25 m (co margin)

Ba muc canh bao:
  OK     - trong vung an toan
  WARN   - sap cham bien (hoac du doan se cham trong `predict_s` giay)
           -> gate CHAN (yaw = 0) nhung KHONG disengage
  BREACH - da vuot bien -> gate coi la FATAL -> disengage + thoat Offboard

He toa do: LOCAL_NED cua PX4. z huong XUONG, nen do cao = -z.
"""

import math
from dataclasses import dataclass
from typing import Optional, Sequence

OK, WARN, BREACH = "ok", "warn", "breach"
_LEVEL_RANK = {OK: 0, WARN: 1, BREACH: 2}


@dataclass(frozen=True)
class FenceConfig:
    """Moi gioi han deu co the tat bang cach dat 0 (tru warn_ratio)."""
    enabled: bool = True

    # ban kinh ngang tinh tu diem CHOT luc ENGAGE
    radius_m: float = 15.0
    # ban kinh ngang tinh tu goc LOCAL_NED (~ diem cat canh). 0 = tat
    home_radius_m: float = 0.0

    # do lech do cao cho phep so voi luc ENGAGE (met, mot chieu)
    alt_dev_m: float = 3.0
    # tran/san tuyet doi so voi goc LOCAL_NED (~ mat dat luc cat canh)
    alt_max_m: float = 10.0
    alt_min_m: float = 1.5

    # toc do ngang toi da chap nhan trong che do yaw-only. 0 = tat
    max_speed_ms: float = 1.5

    # ti le canh bao som: dist > radius * warn_ratio -> WARN
    warn_ratio: float = 0.75
    # du doan vi tri sau `predict_s` giay theo van toc hien tai
    predict_s: float = 1.0


@dataclass(frozen=True)
class FenceState:
    level: str = OK
    reason: str = ""
    dist_m: float = 0.0          # khoang cach ngang toi diem chot
    home_m: float = 0.0          # khoang cach ngang toi goc LOCAL_NED
    alt_dev_m: float = 0.0       # chenh do cao so voi luc engage (+ = cao hon)
    alt_m: float = 0.0           # do cao so voi goc LOCAL_NED
    speed_ms: float = 0.0        # toc do ngang
    dist_pred_m: float = 0.0     # khoang cach du doan sau predict_s

    @property
    def breached(self) -> bool:
        return self.level == BREACH

    @property
    def warned(self) -> bool:
        return self.level != OK


def _finite(v) -> bool:
    try:
        return math.isfinite(float(v))
    except (TypeError, ValueError):
        return False


def _vec_ok(v, n=3) -> bool:
    return (v is not None and len(v) >= n
            and all(_finite(x) for x in list(v)[:n]))


def evaluate(cfg: FenceConfig,
             anchor: Optional[Sequence[float]],
             cur: Optional[Sequence[float]],
             vel: Optional[Sequence[float]] = None) -> FenceState:
    """anchor/cur: (x, y, z) LOCAL_NED. vel: (vx, vy, vz) m/s hoac None.

    anchor = None nghia la chua ENGAGE -> chi kiem tra gioi han tuyet doi
    (home_radius, alt_max, alt_min) neu co du lieu vi tri.
    """
    if not cfg.enabled:
        return FenceState()

    if not _vec_ok(cur):
        # khong co vi tri -> khong ket luan duoc; gate co lop `pos_valid` rieng
        return FenceState()

    x, y, z = float(cur[0]), float(cur[1]), float(cur[2])
    alt = -z                                   # z huong xuong
    home = math.hypot(x, y)

    if _vec_ok(vel, 2):
        vx, vy = float(vel[0]), float(vel[1])
    else:
        vx = vy = 0.0
    speed = math.hypot(vx, vy)

    if _vec_ok(anchor):
        ax, ay, az = float(anchor[0]), float(anchor[1]), float(anchor[2])
        dist = math.hypot(x - ax, y - ay)
        alt_dev = (-z) - (-az)
        dist_pred = math.hypot(x + vx * cfg.predict_s - ax,
                               y + vy * cfg.predict_s - ay)
    else:
        dist = alt_dev = dist_pred = 0.0

    level, reason = OK, ""

    def raise_to(lv: str, msg: str):
        nonlocal level, reason
        if _LEVEL_RANK[lv] > _LEVEL_RANK[level]:
            level, reason = lv, msg

    # --- ban kinh tinh tu diem chot ENGAGE
    if _vec_ok(anchor) and cfg.radius_m > 0:
        if dist > cfg.radius_m:
            raise_to(BREACH, f"vuot fence {dist:.1f}m > {cfg.radius_m:.0f}m")
        elif dist_pred > cfg.radius_m:
            raise_to(WARN, f"du doan vuot fence trong {cfg.predict_s:.0f}s "
                           f"({dist_pred:.1f}m)")
        elif dist > cfg.radius_m * cfg.warn_ratio:
            raise_to(WARN, f"gan bien fence {dist:.1f}m")

    # --- ban kinh tinh tu goc LOCAL_NED (~ diem cat canh)
    if cfg.home_radius_m > 0:
        if home > cfg.home_radius_m:
            raise_to(BREACH, f"cach diem cat canh {home:.1f}m "
                             f"> {cfg.home_radius_m:.0f}m")
        elif home > cfg.home_radius_m * cfg.warn_ratio:
            raise_to(WARN, f"gan bien vung bay {home:.1f}m")

    # --- do lech do cao so voi luc ENGAGE
    if _vec_ok(anchor) and cfg.alt_dev_m > 0:
        if abs(alt_dev) > cfg.alt_dev_m:
            raise_to(BREACH, f"lech do cao {alt_dev:+.1f}m "
                             f"(gioi han {cfg.alt_dev_m:.0f}m)")
        elif abs(alt_dev) > cfg.alt_dev_m * cfg.warn_ratio:
            raise_to(WARN, f"do cao dang troi {alt_dev:+.1f}m")

    # --- tran / san tuyet doi
    if cfg.alt_max_m > 0:
        if alt > cfg.alt_max_m:
            raise_to(BREACH, f"vuot tran {alt:.1f}m > {cfg.alt_max_m:.0f}m")
        elif alt > cfg.alt_max_m * cfg.warn_ratio:
            raise_to(WARN, f"gan tran {alt:.1f}m")
    if cfg.alt_min_m > 0 and alt < cfg.alt_min_m:
        raise_to(BREACH, f"duoi san {alt:.1f}m < {cfg.alt_min_m:.1f}m")

    # --- toc do ngang: yaw-only thi drone khong duoc dich chuyen
    if cfg.max_speed_ms > 0:
        if speed > cfg.max_speed_ms:
            raise_to(BREACH, f"dang troi {speed:.1f}m/s "
                             f"> {cfg.max_speed_ms:.1f}m/s")
        elif speed > cfg.max_speed_ms * cfg.warn_ratio:
            raise_to(WARN, f"toc do ngang {speed:.1f}m/s")

    return FenceState(level=level, reason=reason, dist_m=dist, home_m=home,
                      alt_dev_m=alt_dev, alt_m=alt, speed_ms=speed,
                      dist_pred_m=dist_pred)


def preflight(cfg: FenceConfig, cur: Optional[Sequence[float]]) -> str:
    """Kiem tra TRUOC khi cho ENGAGE. Tra ve "" neu dat, hoac ly do tu choi.

    Diem chot se la `cur`, nen chi can kiem tra cac gioi han tuyet doi.
    """
    if not cfg.enabled:
        return ""
    if not _vec_ok(cur):
        return "chua co vi tri LOCAL_NED de chot fence"
    st = evaluate(cfg, None, cur, None)
    if st.level == BREACH:
        return f"vi tri hien tai da ngoai vung: {st.reason}"
    if st.level == WARN:
        return f"vi tri hien tai sat bien: {st.reason}"
    return ""
