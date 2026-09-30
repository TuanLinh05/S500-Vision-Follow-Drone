"""Hai lop giam sat doc lap voi vong dieu khien.

1) LoopWatchdog  - luong rieng, dem nhip cua main loop. Main loop treo qua
   `timeout` giay thi goi `on_trip` (thuong la op.request_stop()).
   Day la LOP THU HAI cua fix A1: SetpointStreamer.WATCHDOG_S chi ep yaw ve 0,
   con LoopWatchdog moi thuc su NGAT dieu khien va thoat Offboard.

2) CommandAudit  - thuan tuy, so lenh yaw voi yawspeed thuc te tu ATTITUDE.
   Bat hai kieu hong ma link van "khoe":
     - drone KHONG lam theo lenh (Offboard bi tu choi ngam, bao hoa)
     - drone TU XOAY trong khi lenh gan 0 (lenh cu con song o dau do)
   Theo Plan muc 7.7: "So sanh lenh voi chuyen dong that ... roi Offboard".
"""

import threading
import time
from dataclasses import dataclass


class LoopWatchdog(threading.Thread):
    """Giam sat nhip main loop. `on_trip(age)` phai KHONG chan (chi set co)."""

    def __init__(self, timeout: float = 1.0, on_trip=None,
                 poll: float = 0.05):
        super().__init__(daemon=True, name="loop-watchdog")
        self.timeout = float(timeout)
        self.poll = float(poll)
        self._on_trip = on_trip
        self._t_beat = time.monotonic()
        self._lk = threading.Lock()
        # `_stop` la method noi bo cua threading.Thread; ghi de no lam
        # join()/is_alive() co the nem TypeError luc watchdog da dung.
        self._stop_evt = threading.Event()
        self._tripped = False
        self.trips = 0

    def beat(self):
        with self._lk:
            self._t_beat = time.monotonic()

    @property
    def age(self) -> float:
        with self._lk:
            return time.monotonic() - self._t_beat

    def halt(self):
        self._stop_evt.set()

    def run(self):
        while not self._stop_evt.wait(self.poll):
            age = self.age
            if age > self.timeout:
                if not self._tripped:
                    self._tripped = True
                    self.trips += 1
                    print(f"\n  !! LOOP WATCHDOG: main loop im {age:.2f}s "
                          f"-> yeu cau ngat dieu khien")
                    if self._on_trip is not None:
                        try:
                            self._on_trip(age)
                        except Exception as e:      # khong duoc chet o day
                            print(f"  !! loi on_trip: {e}")
            else:
                self._tripped = False


@dataclass(frozen=True)
class AuditConfig:
    tol_deg_s: float = 12.0        # sai lech lenh/thuc te chap nhan duoc
    limit_s: float = 1.5           # sai lech lien tuc bao lau thi bao dong
    idle_cmd_deg_s: float = 2.0    # duoi muc nay coi nhu "lenh = 0"
    idle_spin_deg_s: float = 15.0  # lenh ~0 nhung drone xoay nhanh hon nay
    idle_limit_s: float = 1.0
    enabled: bool = True


class CommandAudit:
    """THUAN TUY: nhan dt tu ngoai, khong doc dong ho he thong."""

    def __init__(self, cfg: AuditConfig = None):
        self.cfg = cfg or AuditConfig()
        self.mismatch_s = 0.0
        self.spin_s = 0.0
        self.reason = ""

    def reset(self):
        self.mismatch_s = 0.0
        self.spin_s = 0.0
        self.reason = ""

    def update(self, dt: float, cmd_deg_s: float, actual_deg_s: float,
               active: bool) -> bool:
        """Tra ve True neu PHAT HIEN bat thuong (caller nen disengage).

        active=False (chua engage / gate dang chan) -> xoa bo dem.
        """
        c = self.cfg
        if not c.enabled or not active:
            self.reset()
            return False

        dt = max(0.0, min(float(dt), 0.5))     # chan dt bat thuong

        # --- lenh lon nhung drone khong lam theo
        if abs(cmd_deg_s) > c.idle_cmd_deg_s and \
                abs(cmd_deg_s - actual_deg_s) > c.tol_deg_s:
            self.mismatch_s += dt
        else:
            self.mismatch_s = max(0.0, self.mismatch_s - dt)

        # --- lenh ~0 nhung drone van xoay
        if abs(cmd_deg_s) <= c.idle_cmd_deg_s and \
                abs(actual_deg_s) > c.idle_spin_deg_s:
            self.spin_s += dt
        else:
            self.spin_s = max(0.0, self.spin_s - dt)

        if self.spin_s > c.idle_limit_s:
            self.reason = (f"drone tu xoay {actual_deg_s:+.0f} do/s "
                           f"trong khi lenh {cmd_deg_s:+.0f} do/s")
            return True
        if self.mismatch_s > c.limit_s:
            self.reason = (f"drone khong theo lenh yaw "
                           f"(lenh {cmd_deg_s:+.0f}, thuc {actual_deg_s:+.0f} do/s)")
            return True
        self.reason = ""
        return False


class StickMonitor:
    """Phat hien phi cong dong stick (RC override) - THUAN TUY.

    Chot vi tri stick tai thoi diem ENGAGE. Sau do neu bat ky kenh nao lech
    qua `threshold` micro-giay thi coi nhu phi cong dang gianh quyen ->
    gate tra quyen ngay, khong doi PX4 xu ly COM_RC_OVERRIDE.

    Ly do khong so voi 1500 co dinh: ga (throttle) va trim moi may bay khac
    nhau; so voi chinh vi tri luc ENGAGE moi phan biet duoc "dang giu" va
    "vua day".
    """

    def __init__(self, chans=(1, 2, 3, 4), threshold=120.0):
        self.chans = tuple(int(c) for c in chans)
        self.threshold = float(threshold)
        self._ref = None

    @property
    def armed(self) -> bool:
        return bool(self._ref)

    def arm(self, rc) -> bool:
        """Chot vi tri stick hien tai. Tra ve False neu khong co du lieu RC."""
        if not rc or not self.chans:
            self._ref = None
            return False
        self._ref = {c: int(rc[c]) for c in self.chans if c in rc}
        return bool(self._ref)

    def disarm(self):
        self._ref = None

    def check(self, rc):
        """Tra ve (override, ly_do)."""
        if not self._ref or not rc:
            return False, ""
        for c, v0 in self._ref.items():
            v = rc.get(c)
            if v is None:
                continue
            d = abs(int(v) - v0)
            if d > self.threshold:
                return True, f"phi cong dong stick ch{c} lech {d:.0f}us"
        return False, ""
