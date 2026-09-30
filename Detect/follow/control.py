import math
from dataclasses import dataclass


def _finite(*vals) -> bool:
    for v in vals:
        try:
            if not math.isfinite(float(v)):
                return False
        except (TypeError, ValueError):
            return False
    return True


@dataclass
class YawConfig:
    hfov_deg: float = 90.0
    gain: float = 0.6           # 1/s
    k_ff: float = 0.0           # he so feed-forward (0 = tat, bat dau 0.6)
    deadzone_deg: float = 4.0
    max_rate: float = 40.0      # do/s
    slew: float = 80.0          # do/s^2
    lead_s: float = 0.0         # bu tre, dat = do tre do duoc
    min_edge_rate: float = 8.0  # toc do toi thieu khi muc tieu cham bien
    invert: bool = False
    fx_px: float = 0.0          # neu >0 thi dung thay cho hfov (da hieu chuan)


class YawController:
    """Thuan tuy: nhan so, tra so. Khong I/O, khong doc dong ho."""

    EDGE_PX = 4

    def __init__(self, cfg: YawConfig, width_px: int):
        self.cfg = cfg
        self.w = width_px
        if cfg.fx_px > 0:
            self.fx = cfg.fx_px
        else:
            self.fx = (width_px / 2.0) / math.tan(math.radians(cfg.hfov_deg / 2.0))
        self.yaw = 0.0
        self.bearing = 0.0
        self._prev_bearing = None
        self._omega = 0.0           # toc do goc tuong doi da loc
        self.at_edge = False
        self.note = ""

    # pixel -> goc (fix B1: dung atan2 thay vi tuyen tinh)
    def bearing_deg(self, x_px: float) -> float:
        return math.degrees(math.atan2(x_px - self.w / 2.0, self.fx))

    def _measure(self, box):
        """Tra (bearing_deg, at_edge). Xu ly box bi cat o bien (fix B2).

        Khi box bi cat o TRAI (left=True): tam that nam TRAI hon ta thay
        -> dung x1 (=0) lam reference de bearing AM (quay trai).
        Khi box bi cat o PHAI (right=True): tam that nam PHAI hon ta thay
        -> dung x2 (=w) lam reference de bearing DUONG (quay phai).
        """
        x1, x2 = float(box[0]), float(box[2])
        left  = x1 <= self.EDGE_PX
        right = x2 >= self.w - self.EDGE_PX
        if left ^ right:
            ref = x1 if left else x2     # dung canh BIEN de chi dung huong
            return self.bearing_deg(ref), True
        return self.bearing_deg((x1 + x2) / 2.0), False

    def update(self, box, dt: float, allow: bool, yawspeed_actual_deg_s: float = 0.0) -> float:
        """box: ndarray[4] hoac None. allow: ket qua tu SafetyGate."""
        c = self.cfg

        # chan du lieu ban: NaN/Inf tu detector hoac dt bat thuong
        if box is not None and not _finite(*tuple(box)[:4]):
            box = None
        if not _finite(dt):
            dt = 0.0
        if not _finite(yawspeed_actual_deg_s):
            yawspeed_actual_deg_s = 0.0

        if box is None or not allow:
            # AN TOAN: ve 0 NGAY, bo qua slew (giu nguyen thiet ke goc)
            self.yaw = 0.0
            self.bearing = 0.0
            self._prev_bearing = None
            self._omega = 0.0
            self.at_edge = False
            self.note = "khong co muc tieu" if box is None else "bi chan"
            return 0.0

        b, self.at_edge = self._measure(box)
        if c.invert:
            # doi dau CA HAI: neu chi doi bearing thi uoc luong omega sai dau
            b = -b
            yawspeed_actual_deg_s = -yawspeed_actual_deg_s
        self.bearing = b

        # uoc luong toc do goc cua MUC TIEU trong khung quan tinh
        if self._prev_bearing is not None and dt > 1e-3:
            raw_omega = (b - self._prev_bearing) / dt + yawspeed_actual_deg_s
            alpha = min(1.0, dt / 0.3)            # loc bac nhat tau = 0.3 s
            self._omega += alpha * (raw_omega - self._omega)
        self._prev_bearing = b

        # bu tre (fix B3)
        e = b + self._omega * c.lead_s

        # deadzone mem (fix B7)
        if abs(e) <= c.deadzone_deg:
            e_eff = 0.0
            self.note = "trong vung chet"
        else:
            e_eff = math.copysign(abs(e) - c.deadzone_deg, e)
            self.note = "quay phai" if e_eff > 0 else "quay trai"

        # P + feed-forward (fix B4)
        raw = c.gain * e_eff + c.k_ff * self._omega

        # toc do toi thieu khi muc tieu sap ra khoi khung (fix B2)
        if self.at_edge and abs(raw) < c.min_edge_rate:
            raw = math.copysign(c.min_edge_rate, e if e != 0 else raw)
            self.note += " (bien khung)"

        raw = max(-c.max_rate, min(c.max_rate, raw))

        # gioi han toc do thay doi (slew rate)
        step = c.slew * max(dt, 1e-3)
        self.yaw += max(-step, min(step, raw - self.yaw))
        if abs(self.yaw) < 0.15:
            self.yaw = 0.0
        if not _finite(self.yaw):        # khong bao gio de NaN ra khoi day
            self.yaw = 0.0
        return self.yaw
