"""PX4 MAVLink link voi ACK, gia tri co tuoi (Stamped), va SetpointStreamer.

Fix A1: SetpointStreamer co watchdog 0.25s, qua han -> yaw ve 0
Fix A2: enter_loiter() thoat Offboard that su, CO XAC MINH mode
Fix A3: set_mode() cho ACK + heartbeat xac nhan, retry 3 lan
Fix A4: Moi gia tri MAVLink boc trong Stamped co timestamp
Fix A6: source_system=191, MAV_COMP_ID_ONBOARD_COMPUTER
Fix A7: Setpoint vi tri + yaw_rate (type_mask=1528) khi co hold
Fix B5: _t_boot CO DINH, _t_win rieng cho do tan so
Fix B6: Lap lich theo moc tuyet doi

v2.1 (chong bay mat kiem soat):
  - CHI nhan heartbeat tu autopilot (bo qua GCS / thanh phan khac)
  - doc them van toc NED, pin %, MAV_STATE (failsafe)
  - doi mode chay tren luong rieng, KHONG chan vong dieu khien
  - streamer: chan NaN/Inf va clamp tran yaw ngay truoc pymavlink
  - streamer: ramp mem sau ENGAGE
    - streamer: khi CHUA engage van phat setpoint GIU VI TRI HIEN TAI
      (idle-follow) de neu PX4 bat ngo vao Offboard thi drone dung yen
    - emergency-stop: watchdog ngoai main loop co the gui mot setpoint yaw=0
      va DUNG stream, de PX4 tu kich hoat Offboard-loss ngay ca khi main loop
      bi ket trong OpenVINO/cv2
"""

import math
import queue
import threading
import time
from dataclasses import dataclass
from typing import Any

# type_mask cho setpoint vi tri + yaw_rate (fix A7)
# bo qua van toc (8|16|32) + gia toc (64|128|256) + yaw (1024)
TYPE_MASK_POS_YAWRATE = (8 | 16 | 32) | (64 | 128 | 256) | 1024   # 1528

# type_mask cho setpoint van toc + yaw_rate (du phong / khi chua co vi tri)
# bo qua vi tri (1|2|4) + gia toc (64|128|256) + yaw (1024)
TYPE_MASK_VEL_YAWRATE = (1 | 2 | 4) | (64 | 128 | 256) | 1024     # 1479

PX4_MAIN_AUTO, PX4_MAIN_OFFBOARD = 4, 6
PX4_SUB_LOITER = 3
MODE_LOITER = "AUTO.LOITER"
MODE_OFFBOARD = "OFFBOARD"

# MAV_STATE bao hieu may bay dang trong tinh trang bat thuong
_BAD_STATES = (5, 6, 8)      # CRITICAL, EMERGENCY, FLIGHT_TERMINATION

# --- giai ma mode PX4 tu custom_mode.
# KHONG dung mavutil.mode_string_v10 vi:
#   1) no tra "LOITER" chu khong phai "AUTO.LOITER", va tra "UNKNOWN" cho
#      POSCTL khi base_mode khong co MANUAL_INPUT_ENABLED -> ten mode phu
#      thuoc phien ban pymavlink, khong the dem so sanh chuoi trong gate;
#   2) mavfile.flightmode lay theo sysid cua ban tin NHAN GAN NHAT, nen mot
#      GCS tren cung link co the lam sai mode cua autopilot.
_PX4_MAIN = {1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 4: "AUTO", 5: "ACRO",
             6: "OFFBOARD", 7: "STABILIZED", 8: "RATTITUDE"}
_PX4_AUTO_SUB = {1: "READY", 2: "TAKEOFF", 3: "LOITER", 4: "MISSION",
                 5: "RTL", 6: "LAND", 7: "RTGS", 8: "FOLLOW_TARGET",
                 9: "PRECLAND"}


def px4_mode_name(base_mode, custom_mode) -> str:
    """Ten mode PX4 on dinh giua cac phien ban pymavlink."""
    if not (int(base_mode) & 1):        # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        return "?"
    main = (int(custom_mode) >> 16) & 0xFF
    sub = (int(custom_mode) >> 24) & 0xFF
    if main == PX4_MAIN_AUTO:
        return "AUTO." + _PX4_AUTO_SUB.get(sub, str(sub))
    return _PX4_MAIN.get(main, f"MAIN{main}")


@dataclass
class Stamped:
    """Gia tri co timestamp. Coi du lieu chua duoc set la cu vinh vien."""
    value: Any = None
    t: float = 0.0

    @property
    def age(self) -> float:
        return time.monotonic() - self.t if self.t else 1e9

    def set(self, v):
        self.value, self.t = v, time.monotonic()


def is_finite(*vals) -> bool:
    """True neu MOI gia tri deu la so huu han. Dung o ranh gioi pymavlink."""
    for v in vals:
        try:
            if not math.isfinite(float(v)):
                return False
        except (TypeError, ValueError):
            return False
    return True


# PX4 gui INT32/UINT16/... trong truong float32 cua PARAM_VALUE/PARAM_SET
# bang cach dien lai BIT PATTERN (byte-wise), khong phai ep kieu so hoc.
# Giong het tools/px4_param.py::_decode_value - lap lai o day (thay vi
# import tu tools) de follow/ khong phu thuoc nguoc vao tools/.
def _decode_param_value(raw_value: float, param_type: int):
    import struct
    packed = struct.pack(">f", float(raw_value))
    formats = {
        1: ">xxxB", 2: ">xxxb", 3: ">xxH", 4: ">xxh", 5: ">I", 6: ">i",
    }  # MAV_PARAM_TYPE_{UINT8,INT8,UINT16,INT16,UINT32,INT32}
    if param_type == 9:              # MAV_PARAM_TYPE_REAL32
        return float(raw_value)
    if param_type not in formats:
        raise ValueError(f"MAV_PARAM_TYPE {param_type} chua ho tro giai ma")
    return struct.unpack(formats[param_type], packed)[0]


def _param_name(value) -> str:
    if isinstance(value, bytes):
        value = value.decode("ascii", errors="replace")
    return str(value).rstrip("\x00")


# Cac cong tac RC anh huong truc tiep den arm/kill/flight-mode/return. Neu
# --rc-chan (dead-man cua app) trung mot trong so nay, ha dead-man se DONG
# THOI doi flight mode / kich RTL / kill motor cua PX4 - khong con la mot
# lop dead-man DOC LAP nua (xem HANDOFF_CLAUDE_UP7000_2026-08-28.md muc 9).
RC_MAP_DEADMAN_CONFLICT_PARAMS = (
    "RC_MAP_ARM_SW", "RC_MAP_KILL_SW", "RC_MAP_FLTMODE", "RC_MAP_RETURN_SW",
    "RC_MAP_OFFB_SW",
)


class PX4Link:
    """RX: heartbeat, mode, armed, RC, pin, vi tri, thai do, ACK, STATUSTEXT.

    Fix A6: source_system=191 tranh trung voi autopilot (sysid=1).
    Fix A4: moi gia tri doc tu MAVLink boc trong Stamped co timestamp.
    v2.1 : chi tin du lieu den tu dung autopilot da bat tay luc khoi tao.
    """

    def __init__(self, url, baud=57600, sysid=191, hb_timeout=10.0):
        from pymavlink import mavutil
        self.mavutil, self.mv = mavutil, mavutil.mavlink
        self.m = mavutil.mavlink_connection(
            url, baud=baud, source_system=sysid,
            source_component=self.mv.MAV_COMP_ID_ONBOARD_COMPUTER)

        hb = self._wait_autopilot_heartbeat(hb_timeout)
        if hb is None:
            raise SystemExit(f"[!] Khong nhan duoc heartbeat autopilot tu {url}")

        # chot dung autopilot de GCS / thanh phan khac khong lam nhieu
        self.ap_sys = hb.get_srcSystem()
        self.ap_comp = hb.get_srcComponent()
        self.m.target_system = self.ap_sys
        self.m.target_component = self.ap_comp
        print(f"  [MAV] autopilot sysid={self.ap_sys} compid={self.ap_comp}")

        self.connected = True
        self.hb = Stamped()
        self.hb.set(True)
        self.armed = False
        self.mode = "?"
        self.sys_state = 0
        self.rc = Stamped({})          # dict {chan: value} (fix A4)
        self.batt = Stamped(0.0)       # volt
        self.batt_pct = Stamped(-1.0)  # phan tram (-1 = khong biet)
        self.pos = Stamped(None)       # (x, y, z) NED
        self.vel = Stamped(None)       # (vx, vy, vz) NED m/s
        self.att = Stamped(None)       # (roll, pitch, yaw, yawspeed)
        # SYS_STATUS xac nhan estimator/position controller XY dang healthy.
        # LOCAL_POSITION_NED co the van duoc phat khi EKF o constant-position
        # mode; chi "co ban tin" khong co nghia la giu vi tri Offboard an toan.
        self.xy_pos_health = Stamped(None)
        # ODOMETRY.reset_counter: PX4 tang gia tri nay MOI LAN EKF reset vi
        # tri/van toc/huong. Can MAVLink2 (id 331 > 255) - da ep o
        # follow/__init__.py. Neu van con None nghia la CHUA TUNG nhan duoc
        # ban tin nay (vd firmware/param khong phat) - goi la khong biet,
        # KHONG duoc suy dien la "khong co reset nao".
        self.reset_counter = Stamped(None)
        self.status_text = ""

        self._acks = queue.Queue(maxsize=32)
        self._params = queue.Queue(maxsize=64)
        self._done = threading.Event()
        self._mode_lk = threading.Lock()
        self._mode_busy = False
        self.mode_req = ""             # mode dang duoc yeu cau (de hien thi)
        threading.Thread(target=self._rx, daemon=True, name="mav-rx").start()

        self._request_streams()

    # ---------------------------------------------------------- bat tay
    def _wait_autopilot_heartbeat(self, timeout):
        """Bo qua heartbeat cua GCS / onboard computer khac tren cung link."""
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            msg = self.m.recv_match(type="HEARTBEAT", blocking=True,
                                    timeout=1.0)
            if msg is None:
                continue
            if getattr(msg, "type", None) == self.mv.MAV_TYPE_GCS:
                continue
            ap = getattr(msg, "autopilot", self.mv.MAV_AUTOPILOT_INVALID)
            if ap == self.mv.MAV_AUTOPILOT_INVALID:
                continue
            return msg
        return None

    @property
    def failsafe(self) -> bool:
        """PX4 dang bao trang thai nguy cap (failsafe da kich hoat)."""
        return self.sys_state in _BAD_STATES

    # ---------------------------------------------------------- streams
    def _request_streams(self):
        """Yeu cau PX4 gui cac stream can thiet (fix A4)."""
        want = [
            ("RC_CHANNELS",        self.mv.MAVLINK_MSG_ID_RC_CHANNELS,        20),
            ("LOCAL_POSITION_NED", self.mv.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10),
            ("ATTITUDE",           self.mv.MAVLINK_MSG_ID_ATTITUDE,           20),
            ("BATTERY_STATUS",     self.mv.MAVLINK_MSG_ID_BATTERY_STATUS,      1),
            ("SYS_STATUS",         self.mv.MAVLINK_MSG_ID_SYS_STATUS,           2),
            ("ODOMETRY",           self.mv.MAVLINK_MSG_ID_ODOMETRY,             5),
        ]
        for _name, mid, hz in want:
            self.m.mav.command_long_send(
                self.ap_sys, self.ap_comp,
                self.mv.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                mid, int(1e6 / hz), 0, 0, 0, 0, 0)

    # ---------------------------------------------------------- RX
    def _rx(self):
        while not self._done.is_set():
            try:
                msg = self.m.recv_match(blocking=True, timeout=1.0)
            except Exception:
                time.sleep(0.2)
                continue
            if msg is None:
                continue

            # chi tin du lieu tu dung autopilot da bat tay
            try:
                if msg.get_srcSystem() != self.ap_sys:
                    continue
            except Exception:
                continue

            t = msg.get_type()

            if t == "HEARTBEAT":
                if msg.get_srcComponent() != self.ap_comp:
                    continue
                self.hb.set(True)
                self.armed = bool(msg.base_mode
                                  & self.mv.MAV_MODE_FLAG_SAFETY_ARMED)
                self.sys_state = int(getattr(msg, "system_status", 0))
                try:
                    self.mode = px4_mode_name(msg.base_mode, msg.custom_mode)
                except Exception:
                    self.mode = str(msg.custom_mode)

            elif t in ("RC_CHANNELS", "RC_CHANNELS_RAW"):
                d = {}
                for i in range(1, 19):
                    v = getattr(msg, f"chan{i}_raw", None)
                    if v is not None and v != 65535:
                        d[i] = int(v)
                if d:
                    self.rc.set(d)       # fix A4: timestamp vao du lieu RC

            elif t == "LOCAL_POSITION_NED":
                if is_finite(msg.x, msg.y, msg.z):
                    self.pos.set((msg.x, msg.y, msg.z))
                if is_finite(msg.vx, msg.vy, msg.vz):
                    self.vel.set((msg.vx, msg.vy, msg.vz))

            elif t == "ATTITUDE":
                # Du lieu thai do hong/NaN phai de no tu cu di, de safety gate
                # phat hien attitude stale thay vi bien no thanh yaw rate = 0.
                if is_finite(msg.roll, msg.pitch, msg.yaw, msg.yawspeed):
                    self.att.set((msg.roll, msg.pitch, msg.yaw, msg.yawspeed))

            elif t in ("SYS_STATUS", "BATTERY_STATUS"):
                if t == "SYS_STATUS":
                    bit = getattr(
                        self.mv, "MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL", 0)
                    if bit:
                        present = int(getattr(
                            msg, "onboard_control_sensors_present", 0))
                        enabled = int(getattr(
                            msg, "onboard_control_sensors_enabled", 0))
                        healthy = int(getattr(
                            msg, "onboard_control_sensors_health", 0))
                        self.xy_pos_health.set(bool(
                            (present & bit) and (enabled & bit) and
                            (healthy & bit)))
                mvv = getattr(msg, "voltage_battery", None)
                if mvv is None:
                    vs = getattr(msg, "voltages", None)
                    mvv = vs[0] if vs else None
                if mvv and 0 < mvv < 65535:
                    self.batt.set(mvv / 1000.0)
                pct = getattr(msg, "battery_remaining", None)
                if pct is not None and 0 <= pct <= 100:
                    self.batt_pct.set(float(pct))

            elif t == "ODOMETRY":
                rc = getattr(msg, "reset_counter", None)
                if rc is not None:
                    self.reset_counter.set(int(rc) & 0xFF)

            elif t == "COMMAND_ACK":
                try:
                    self._acks.put_nowait(msg)
                except queue.Full:
                    pass

            elif t == "PARAM_VALUE":
                try:
                    self._params.put_nowait(msg)
                except queue.Full:
                    pass

            elif t == "STATUSTEXT":
                self.status_text = msg.text
                print(f"  [PX4] {msg.text}")

    # ---------------------------------------------------------- doi mode
    def _wait_ack(self, cmd_id, timeout):
        """Cho ACK cho lenh cu the. Tra ve msg hoac None."""
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            try:
                a = self._acks.get(
                    timeout=max(0.05, t_end - time.monotonic()))
            except queue.Empty:
                return None
            if a.command == cmd_id:
                return a
        return None

    # ---------------------------------------------------------- parameter
    def read_param(self, name, timeout=2.0):
        """Doc DONG BO mot PX4 parameter. Tra ve gia tri da giai ma hoac
        None neu qua han - KHONG bao gio nem loi vi day chi la kiem tra tu
        van/canh bao luc khoi dong, khong phai duong dieu khien.

        CHI goi truoc khi vong dieu khien bat dau: co the block toi
        `timeout` giay, giong het set_mode() dong bo.
        """
        deadline = time.monotonic() + timeout
        encoded = name.encode("ascii")
        next_req = 0.0
        while not self._done.is_set():
            now = time.monotonic()
            if now >= deadline:
                return None
            if now >= next_req:
                self.m.mav.param_request_read_send(
                    self.ap_sys, self.ap_comp, encoded, -1)
                next_req = now + 0.5
            try:
                msg = self._params.get(
                    timeout=max(0.05, min(0.5, deadline - now)))
            except queue.Empty:
                continue
            if _param_name(msg.param_id) != name:
                continue
            try:
                return _decode_param_value(
                    msg.param_value, int(msg.param_type))
            except ValueError:
                return None
        return None

    def set_mode(self, main, sub=0, expect=None, retries=3) -> bool:
        """Gui DO_SET_MODE, cho ACK, roi cho heartbeat XAC NHAN mode.
        Fix A3: xac minh Offboard duoc chap nhan.

        CHAN luong goi toi ~10s -> vong dieu khien phai dung set_mode_async.
        """
        cmd = self.mv.MAV_CMD_DO_SET_MODE
        for _ in range(retries):
            # Xoa ACK cu tranh nham ket qua tu lenh truoc
            while not self._acks.empty():
                try:
                    self._acks.get_nowait()
                except queue.Empty:
                    break
            self.m.mav.command_long_send(
                self.ap_sys, self.ap_comp, cmd, 0,
                self.mv.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                main, sub, 0, 0, 0, 0)
            ack = self._wait_ack(cmd, 1.5)
            if ack is None or ack.result != self.mv.MAV_RESULT_ACCEPTED:
                print(f"  [MAV] DO_SET_MODE bi tu choi: "
                      f"{ack.result if ack else 'khong co ACK'} "
                      f"| {self.status_text}")
                continue
            # cho heartbeat xac nhan mode thay doi
            t_end = time.monotonic() + 2.0
            while time.monotonic() < t_end:
                if expect is None or self.mode == expect:
                    return True
                time.sleep(0.05)
            print(f"  [MAV] ACK ok nhung mode van la {self.mode}")
        return False

    def set_mode_async(self, main, sub=0, expect=None, retries=3,
                       on_done=None) -> bool:
        """Doi mode tren luong rieng. Tra ve False neu dang co lenh khac.

        Vong dieu khien PHAI dung ham nay: set_mode() dong bo co the chan
        toi ~10s, du de TX watchdog trip va camera don khung.
        """
        with self._mode_lk:
            if self._mode_busy:
                return False
            self._mode_busy = True
            self.mode_req = expect or f"main={main}"

        def worker():
            ok = False
            try:
                ok = self.set_mode(main, sub, expect, retries)
            except Exception as e:
                print(f"  !! loi doi mode: {e}")
            finally:
                with self._mode_lk:
                    self._mode_busy = False
                    self.mode_req = ""
                if on_done is not None:
                    try:
                        on_done(ok)
                    except Exception as e:
                        print(f"  !! loi callback doi mode: {e}")

        threading.Thread(target=worker, daemon=True,
                         name="mav-setmode").start()
        return True

    @property
    def mode_busy(self) -> bool:
        with self._mode_lk:
            return self._mode_busy

    def enter_offboard(self) -> bool:
        return self.set_mode(PX4_MAIN_OFFBOARD, 0, expect=MODE_OFFBOARD)

    def enter_offboard_async(self, on_done=None) -> bool:
        return self.set_mode_async(PX4_MAIN_OFFBOARD, 0,
                                   expect=MODE_OFFBOARD, on_done=on_done)

    def enter_loiter(self) -> bool:
        """Fix A2: CHI goi khi dang o OFFBOARD - tranh giat quyen khoi phi cong.
        v2.1: xac minh bang heartbeat (expect) chu khong chi tin ACK."""
        if self.mode != MODE_OFFBOARD:
            return True
        return self.set_mode(PX4_MAIN_AUTO, PX4_SUB_LOITER,
                             expect=MODE_LOITER)

    def enter_loiter_async(self, on_done=None) -> bool:
        if self.mode != MODE_OFFBOARD:
            if on_done is not None:
                on_done(True)
            return True
        return self.set_mode_async(PX4_MAIN_AUTO, PX4_SUB_LOITER,
                                   expect=MODE_LOITER, on_done=on_done)

    def close(self):
        self._done.set()
        try:
            self.m.close()
        except Exception:
            pass


def check_rc_map_deadman_conflict(link: "PX4Link", rc_chan: int,
                                  timeout_each: float = 1.5):
    """Doc RC_MAP_* tren PX4 va so voi kenh dead-man cua app luc khoi dong.

    Tra ve (conflicts, unverified):
      conflicts  - danh sach chuoi mo ta cac RC_MAP_* TRUNG voi rc_chan.
                   Non-empty => dead-man KHONG DOC LAP, PX4 se doi flight
                   mode / kich RTL / kill motor cung luc voi ha dead-man.
      unverified - danh sach ten parameter KHONG doc duoc trong thoi gian
                   cho phep (PX4 khong tra loi / link qua cham). Khong duoc
                   suy dien "khong doc duoc" thanh "khong xung dot".

    CHI goi mot lan luc khoi dong (dong bo, co the block vai giay).
    """
    conflicts, unverified = [], []
    if not rc_chan:
        return conflicts, unverified
    for name in RC_MAP_DEADMAN_CONFLICT_PARAMS:
        value = link.read_param(name, timeout=timeout_each)
        if value is None:
            unverified.append(name)
            continue
        if int(round(value)) == int(rc_chan):
            conflicts.append(
                f"{name}={int(round(value))} trung voi --rc-chan {rc_chan}")
    return conflicts, unverified


class SetpointStreamer(threading.Thread):
    """Gui setpoint deu dan. Lenh yaw CO HAN SU DUNG.

    Fix A1: An toan cot loi - neu khong duoc command() nuoi trong
    WATCHDOG_S giay thi tu dong ve 0. Main loop treo => drone dung xoay.

    Fix A7: Dung setpoint vi tri + yaw_rate khi co hold.
    Fix B5: _t_boot CO DINH, _t_win rieng cho do tan so.
    Fix B6: Lap lich theo moc tuyet doi.

    v2.1:
      - chan NaN/Inf va clamp tran yaw ngay truoc khi goi pymavlink
      - ramp mem `ramp_s` giay sau arm_hold: tran yaw tang dan tu 0
      - idle-follow: chua arm_hold thi phat setpoint GIU VI TRI HIEN TAI
        thay vi van toc 0, de vao Offboard bat ngo cung khong troi
    """
    WATCHDOG_S = 0.25

    def __init__(self, link, rate_hz=20.0, use_position_hold=True,
                 max_rate=40.0, ramp_s=1.0, idle_follow=True):
        super().__init__(daemon=True, name="sp-streamer")
        self.link = link
        self.dt = 1.0 / rate_hz
        self.use_position_hold = use_position_hold
        self.max_rate = float(max_rate)
        self.ramp_s = float(ramp_s)
        self.idle_follow = idle_follow
        self._lk = threading.Lock()
        self._cmd_yaw = 0.0
        self._t_cmd = 0.0
        self._hold = None                    # (x, y, z) chot luc engage
        self._t_hold = 0.0                   # thoi diem arm_hold (cho ramp)
        # Khong dat ten `_stop`: do la method noi bo cua threading.Thread.
        # Ghi de no lam join()/is_alive() nem TypeError sau khi luong ket thuc.
        self._stop_evt = threading.Event()
        self._emergency = threading.Event()
        self._paused = threading.Event()
        self._emergency_reason = ""
        self._t_boot = time.monotonic()      # CO DINH - khong reset (fix B5)
        self.sp_sent = 0
        self.sp_hz = 0.0
        self.watchdog_trips = 0
        self.nan_blocks = 0
        self.clamp_hits = 0
        self.emergency_stops = 0
        self.last_yaw = 0.0
        self.halted = False

    # ------------------------------------------------------------ API
    def command(self, yaw_deg_s):
        """Nuoi lenh yaw. Phai goi LIEN TUC tu main loop (fix A1)."""
        if self.halted or self._emergency.is_set() or self._paused.is_set():
            return False
        if not is_finite(yaw_deg_s):
            self.nan_blocks += 1
            yaw_deg_s = 0.0
        with self._lk:
            self._cmd_yaw = float(yaw_deg_s)
            self._t_cmd = time.monotonic()
        return True

    def arm_hold(self, xyz):
        """Chot diem giu tai thoi diem ENGAGE (fix A7). Bat dau ramp."""
        if self.halted or self._emergency.is_set():
            return False
        if xyz is not None and not is_finite(*tuple(xyz)[:3]):
            xyz = None
        with self._lk:
            self._hold = tuple(xyz)[:3] if xyz is not None else None
            self._t_hold = time.monotonic() if xyz is not None else 0.0
        return True

    def clear_hold(self):
        with self._lk:
            self._hold = None
            self._t_hold = 0.0

    def start_ramp(self):
        """Bat dau lai dong ho ramp (goi dung luc OFFBOARD duoc xac nhan)."""
        with self._lk:
            self._t_hold = time.monotonic()

    @property
    def hold(self):
        with self._lk:
            return self._hold

    def drift_from_hold(self, cur_xyz):
        """Tinh khoang cach xy tu vi tri hien tai toi diem chot."""
        with self._lk:
            h = self._hold
        if h is None or cur_xyz is None:
            return 0.0
        if not is_finite(*tuple(cur_xyz)[:2]):
            return 0.0
        return math.dist(h[:2], tuple(cur_xyz)[:2])

    def ramp_scale(self) -> float:
        """He so 0..1 - tran yaw tang dan trong `ramp_s` giay sau ENGAGE."""
        with self._lk:
            t_hold = self._t_hold
        if t_hold == 0.0 or self.ramp_s <= 0:
            return 1.0
        return min(1.0, (time.monotonic() - t_hold) / self.ramp_s)

    def halt(self):
        """Dung han luong TX. PX4 kich hoat Offboard-loss theo COM_OF_LOSS_T."""
        self.halted = True
        self._stop_evt.set()

    def pause(self):
        """Ngung TX sau khi da xac nhan roi Offboard; co the resume lan sau."""
        if self.halted or self._emergency.is_set():
            return False
        with self._lk:
            self._cmd_yaw = 0.0
            self._t_cmd = time.monotonic()
            self._hold = None
            self._t_hold = 0.0
            self._paused.set()
        return True

    def resume(self):
        """Cho phep TX lai de pre-stream cho mot ENGAGE moi."""
        if self.halted or self._emergency.is_set():
            return False
        with self._lk:
            self._cmd_yaw = 0.0
            self._t_cmd = time.monotonic()
            self._hold = None
            self._t_hold = 0.0
            self._paused.clear()
        return True

    @property
    def paused(self):
        return self._paused.is_set()

    @property
    def emergency_requested(self):
        return self._emergency.is_set()

    @property
    def emergency_reason(self):
        with self._lk:
            return self._emergency_reason

    def emergency_stop(self, reason=""):
        """Cau chi khan cap, an toan khi goi tu watchdog thread.

        Ham nay KHONG goi MAVLink hay doi mode: cac thao tac do co the block
        dung luc main loop dang treo. Luong TX tu gui mot setpoint yaw=0, sau
        do dung han stream de PX4 xu ly Offboard-loss theo cau hinh FC.
        Tra ve True chi o lan yeu cau dau tien.
        """
        with self._lk:
            if self._emergency.is_set() or self.halted:
                return False
            self._cmd_yaw = 0.0
            self._t_cmd = time.monotonic()
            self._emergency_reason = str(reason)
            self.emergency_stops += 1
            self._emergency.set()
        return True

    # ------------------------------------------------------------ vong TX
    def run(self):
        next_t = time.monotonic()
        t_win, n_win = next_t, 0
        while not self._stop_evt.is_set():
            next_t += self.dt
            with self._lk:
                yaw = self._cmd_yaw
                t_cmd = self._t_cmd
                hold = self._hold

            # Duong dung khan cap phai nam trong chinh luong TX. Nhieu thread
            # cung gui MAVLink/mode tu watchdog se tao race va co the block.
            if self._emergency.is_set():
                self._send(hold, 0.0)     # dua yaw ve 0 ngay truoc khi cat
                self.last_yaw = 0.0
                self.halted = True
                self._stop_evt.set()
                break

            # Da xac nhan roi Offboard: khong tiep tuc "prime" PX4 khi app
            # idle. Thread van ton tai de co the resume cho ENGAGE ke tiep.
            if self._paused.is_set():
                self.sp_hz = 0.0
                if self._stop_evt.wait(min(self.dt, 0.05)):
                    break
                next_t = time.monotonic()
                t_win, n_win = next_t, self.sp_sent
                continue

            # WATCHDOG (fix A1): lenh qua han -> ep ve 0
            if (time.monotonic() - t_cmd) > self.WATCHDOG_S:
                if yaw != 0.0:
                    self.watchdog_trips += 1
                    print("\n  !! TX WATCHDOG: lenh yaw qua han, ep ve 0")
                yaw = 0.0

            # ramp mem sau ENGAGE + clamp tran (lop cuoi truoc pymavlink)
            lim = self.max_rate * self.ramp_scale()
            if abs(yaw) > lim:
                self.clamp_hits += 1
                yaw = math.copysign(lim, yaw)

            self._send(hold, yaw)
            self.last_yaw = yaw

            now = time.monotonic()
            if now - t_win >= 1.0:
                self.sp_hz = (self.sp_sent - n_win) / (now - t_win)
                t_win, n_win = now, self.sp_sent

            # Lap lich theo moc tuyet doi (fix B6)
            sleep = next_t - now
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()   # bo chu ky da tre

    def _send(self, hold, yaw_deg_s):
        mv, m = self.link.mv, self.link.m
        boot_ms = int((time.monotonic() - self._t_boot) * 1000) & 0xFFFFFFFF

        target = hold
        if target is None and self.idle_follow:
            # chua ENGAGE: giu chinh vi tri hien tai, khong phai van toc 0
            cur = self.link.pos.value
            if cur is not None and self.link.pos.age < 1.0:
                target = cur

        # Fix A7: dung setpoint vi tri khi co diem giu
        if target is not None and self.use_position_hold:
            mask = TYPE_MASK_POS_YAWRATE
            x, y, z = target
            vx = vy = vz = 0.0
        else:
            mask = TYPE_MASK_VEL_YAWRATE
            x = y = z = 0.0
            vx = vy = vz = 0.0

        if not is_finite(x, y, z, yaw_deg_s):
            self.nan_blocks += 1
            return
        try:
            m.mav.set_position_target_local_ned_send(
                boot_ms, self.link.ap_sys, self.link.ap_comp,
                mv.MAV_FRAME_LOCAL_NED, mask,
                x, y, z, vx, vy, vz, 0, 0, 0,
                0.0, math.radians(yaw_deg_s))
            self.sp_sent += 1
        except Exception as e:
            print(f"\n  !! loi gui setpoint: {e}")
