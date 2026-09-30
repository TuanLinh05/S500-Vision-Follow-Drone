"""PX4 gia qua UDP loopback de kiem thu px4.py ma khong can phan cung.

Dung cho tests/test_mavlink.py — kiem chung cac fix A1/A3/A6/A7/B5 va cac
lop an toan moi cua v2.1 (clamp, NaN, ramp, idle-follow, loc heartbeat).
"""

import socket
import struct
import threading
import time
from collections import deque

from pymavlink import mavutil

mv = mavutil.mavlink

_PARAM_ENCODE_FORMATS = {
    mv.MAV_PARAM_TYPE_UINT8: ">xxxB", mv.MAV_PARAM_TYPE_INT8: ">xxxb",
    mv.MAV_PARAM_TYPE_UINT16: ">xxH", mv.MAV_PARAM_TYPE_INT16: ">xxh",
    mv.MAV_PARAM_TYPE_UINT32: ">I", mv.MAV_PARAM_TYPE_INT32: ">i",
}


def _encode_param(value, param_type):
    """Ma hoa byte-wise giong PX4 that (tools/px4_param.py::_encode_value)."""
    if param_type == mv.MAV_PARAM_TYPE_REAL32:
        return float(value)
    packed = struct.pack(_PARAM_ENCODE_FORMATS[param_type], int(value))
    return struct.unpack(">f", packed)[0]


def _param_name(value) -> str:
    if isinstance(value, bytes):
        value = value.decode("ascii", errors="replace")
    return str(value).rstrip("\x00")

PX4_MAIN_AUTO, PX4_MAIN_OFFBOARD = 4, 6
PX4_SUB_LOITER = 3
AUTO_FLAGS = 28          # AUTO | GUIDED | STABILIZE
CUSTOM_ENABLED = mv.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED


def _mute_udp_connreset(conn):
    """Windows: gui UDP toi cong chua ai nghe -> recvfrom nem WSAECONNRESET.
    Tat hanh vi nay de luong PX4 gia khong chet luc dang cho ben kia bind."""
    port = getattr(conn, "port", None)
    if port is None or not hasattr(socket, "SIO_UDP_CONNRESET"):
        return
    try:
        port.ioctl(socket.SIO_UDP_CONNRESET, False)
    except (OSError, AttributeError):
        pass


def free_udp_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


class FakePX4(threading.Thread):
    """Phat heartbeat/telemetry, nhan setpoint, tra loi DO_SET_MODE."""

    def __init__(self, port, sysid=1, compid=1):
        super().__init__(daemon=True, name="fake-px4")
        self.m = mavutil.mavlink_connection(
            f"udpout:127.0.0.1:{port}", source_system=sysid,
            source_component=compid)
        self.sysid = sysid
        _mute_udp_connreset(self.m)

        # trang thai co the chinh tu test
        self.main_mode = 3               # POSCTL
        self.sub_mode = 0
        self.armed = True
        self.sys_state = 4               # MAV_STATE_ACTIVE
        self.pos = (0.0, 0.0, -5.0)
        self.vel = (0.0, 0.0, 0.0)
        self.yawspeed = 0.0
        self.rc = {1: 1500, 2: 1500, 3: 1400, 4: 1500, 7: 1800}
        self.batt_mv = 15400
        self.batt_pct = 88
        self.xy_pos_healthy = True
        self.reject_mode = False         # True -> tu choi moi DO_SET_MODE
        self.send_gcs_heartbeat = False  # gia lap QGC tren cung link
        self.odom_reset_counter = 0      # ODOMETRY.reset_counter (uint8)
        # RC_MAP_* / cac parameter khac PX4Link.read_param() co the doc.
        # Mac dinh la mot mapping "sach" (KHONG trung kenh 7 - dead-man dung
        # trong hau het test) de check_rc_map_deadman_conflict() khong tra
        # ve xung dot GIA cho cac test khong chu y kiem tra tinh nang nay.
        # Test muon mo phong xung dot thuc (vd RC_MAP_FLTMODE trung --rc-chan)
        # tu ghi de fake.params[...] truoc khi start().
        T = mv.MAV_PARAM_TYPE_INT32
        self.params = {
            "RC_MAP_ARM_SW": (5, T), "RC_MAP_KILL_SW": (6, T),
            "RC_MAP_FLTMODE": (9, T), "RC_MAP_RETURN_SW": (8, T),
            "RC_MAP_OFFB_SW": (0, T),      # 0 = chua gan (binh thuong o PX4)
        }

        self.setpoints = deque(maxlen=400)
        self.mode_cmds = deque(maxlen=50)
        self._lk = threading.Lock()
        self._stop = threading.Event()

    # ------------------------------------------------------------ helpers
    def last_setpoint(self, timeout=1.0):
        """Doi va tra ve setpoint moi nhat."""
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            with self._lk:
                if self.setpoints:
                    return self.setpoints[-1]
            time.sleep(0.01)
        return None

    def fresh_setpoint(self, timeout=2.0, n=3):
        """Xoa hang doi roi tra ve setpoint MOI (khong phai cai truoc do)."""
        sps = self.next_setpoints(n, timeout)
        return sps[-1] if sps else None

    def next_setpoints(self, n, timeout=2.0):
        """Xoa hang doi roi thu n setpoint TIEP THEO."""
        with self._lk:
            self.setpoints.clear()
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            with self._lk:
                if len(self.setpoints) >= n:
                    return list(self.setpoints)[:n]
            time.sleep(0.01)
        with self._lk:
            return list(self.setpoints)

    def set_mode_now(self, main, sub=0):
        self.main_mode, self.sub_mode = main, sub

    def halt(self):
        self._stop.set()

    # ------------------------------------------------------------ vong chay
    def run(self):
        t_hb = t_pos = t_att = t_rc = t_batt = t_sys = t_odom = 0.0
        while not self._stop.is_set():
            now = time.monotonic()
            try:
                if now - t_hb >= 0.1:
                    t_hb = now
                    self._send_hb()
                if now - t_pos >= 0.05:
                    t_pos = now
                    self.m.mav.local_position_ned_send(
                        int(now * 1000) & 0xFFFFFFFF,
                        self.pos[0], self.pos[1], self.pos[2],
                        self.vel[0], self.vel[1], self.vel[2])
                if now - t_att >= 0.05:
                    t_att = now
                    self.m.mav.attitude_send(
                        int(now * 1000) & 0xFFFFFFFF, 0.0, 0.0, 0.0,
                        0.0, 0.0, float(self.yawspeed))
                if now - t_rc >= 0.05:
                    t_rc = now
                    ch = [self.rc.get(i, 65535) for i in range(1, 19)]
                    self.m.mav.rc_channels_send(
                        int(now * 1000) & 0xFFFFFFFF, 18, *ch[:18], 100)
                if now - t_batt >= 1.0:
                    t_batt = now
                    volts = [self.batt_mv] + [65535] * 9
                    self.m.mav.battery_status_send(
                        0, 0, 0, 32767, volts, -1, -1, -1, self.batt_pct)
                if now - t_sys >= 0.5:
                    t_sys = now
                    xy = getattr(
                        mv, "MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL", 0)
                    health = xy if self.xy_pos_healthy else 0
                    self.m.mav.sys_status_send(
                        xy, xy, health, 350, self.batt_mv, -1,
                        self.batt_pct, 0, 0, 0, 0, 0, 0)
                if now - t_odom >= 0.1:
                    t_odom = now
                    # can MAVLink2 (id 331 > 255) - ep o tests/__init__.py.
                    self.m.mav.odometry_send(
                        int(now * 1e6) & 0xFFFFFFFFFFFFFFFF,
                        mv.MAV_FRAME_LOCAL_NED, mv.MAV_FRAME_BODY_FRD,
                        self.pos[0], self.pos[1], self.pos[2],
                        [1.0, 0.0, 0.0, 0.0],
                        self.vel[0], self.vel[1], self.vel[2],
                        0.0, 0.0, float(self.yawspeed),
                        [0.0] * 21, [0.0] * 21,
                        int(self.odom_reset_counter) & 0xFF)
                self._pump_rx()
            except OSError:
                pass          # ben kia chua bind / da dong - bo qua
            time.sleep(0.005)

    def _send_hb(self):
        base = CUSTOM_ENABLED | AUTO_FLAGS
        if self.armed:
            base |= mv.MAV_MODE_FLAG_SAFETY_ARMED
        custom = (self.main_mode << 16) | (self.sub_mode << 24)
        self.m.mav.heartbeat_send(
            mv.MAV_TYPE_QUADROTOR, mv.MAV_AUTOPILOT_PX4,
            base, custom, self.sys_state)
        if self.send_gcs_heartbeat:
            # QGC gia: sysid khac, KHONG phai autopilot, chua arm
            old_s, old_c = self.m.mav.srcSystem, self.m.mav.srcComponent
            self.m.mav.srcSystem, self.m.mav.srcComponent = 255, 190
            self.m.mav.heartbeat_send(
                mv.MAV_TYPE_GCS, mv.MAV_AUTOPILOT_INVALID, 0, 0, 3)
            self.m.mav.srcSystem, self.m.mav.srcComponent = old_s, old_c

    def _pump_rx(self):
        for _ in range(40):
            msg = self.m.recv_match(blocking=False)
            if msg is None:
                return
            t = msg.get_type()
            if t == "SET_POSITION_TARGET_LOCAL_NED":
                with self._lk:
                    self.setpoints.append(msg)
            elif t == "COMMAND_LONG":
                if msg.command == mv.MAV_CMD_DO_SET_MODE:
                    with self._lk:
                        self.mode_cmds.append((msg.param2, msg.param3))
                    if self.reject_mode:
                        res = mv.MAV_RESULT_TEMPORARILY_REJECTED
                    else:
                        res = mv.MAV_RESULT_ACCEPTED
                        self.main_mode = int(msg.param2)
                        self.sub_mode = int(msg.param3)
                    self.m.mav.command_ack_send(mv.MAV_CMD_DO_SET_MODE, res)
                elif msg.command == mv.MAV_CMD_SET_MESSAGE_INTERVAL:
                    self.m.mav.command_ack_send(
                        mv.MAV_CMD_SET_MESSAGE_INTERVAL,
                        mv.MAV_RESULT_ACCEPTED)
            elif t == "PARAM_REQUEST_READ":
                name = _param_name(msg.param_id)
                if name in self.params:
                    value, ptype = self.params[name]
                    self.m.mav.param_value_send(
                        name.encode("ascii"),
                        _encode_param(value, ptype), ptype, 1, 0)
