from __future__ import annotations

import copy
import time
from typing import Any

from .models import TelemetrySnapshot
from .serial_discovery import resolve_connection


class MissingDependencyError(RuntimeError):
    pass


def _load_mavutil() -> Any:
    try:
        from pymavlink import mavutil
    except ImportError as exc:  # pragma: no cover - depends on target image
        raise MissingDependencyError(
            "pymavlink is not installed. Run: python3 -m pip install -e ."
        ) from exc
    return mavutil


class MavlinkTelemetryReader:
    """Single-owner MAVLink reader. It sends no arm/mode/setpoint commands."""

    def __init__(
        self,
        connection: str = "auto",
        expected_system_id: int = 1,
        heartbeat_timeout_s: float = 5.0,
        poll_s: float = 0.05,
    ) -> None:
        self.connection_spec = resolve_connection(connection)
        self.expected_system_id = expected_system_id
        self.heartbeat_timeout_s = heartbeat_timeout_s
        self.poll_s = poll_s
        self._mavutil = _load_mavutil()
        self._connection: Any | None = None
        self._snapshot = TelemetrySnapshot()

    @property
    def connection(self) -> Any:
        if self._connection is None:
            raise RuntimeError("MAVLink connection is not open")
        return self._connection

    def open(self) -> TelemetrySnapshot:
        kwargs: dict[str, Any] = {
            "source_system": 245,
            "source_component": 190,
            "autoreconnect": True,
        }
        self._connection = self._mavutil.mavlink_connection(
            self.connection_spec, **kwargs
        )
        heartbeat = self.connection.wait_heartbeat(
            blocking=True, timeout=self.heartbeat_timeout_s
        )
        if heartbeat is None:
            raise TimeoutError(
                f"No MAVLink HEARTBEAT on {self.connection_spec} within "
                f"{self.heartbeat_timeout_s:.1f}s"
            )
        self._apply(heartbeat, time.monotonic())
        if self._snapshot.system_id != self.expected_system_id:
            raise RuntimeError(
                f"Unexpected MAVLink system id {self._snapshot.system_id}; "
                f"expected {self.expected_system_id}"
            )
        return self.snapshot()

    def close(self) -> None:
        if self._connection is not None:
            close = getattr(self._connection, "close", None)
            if close is not None:
                close()
        self._connection = None

    def __enter__(self) -> "MavlinkTelemetryReader":
        self.open()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def poll(self) -> TelemetrySnapshot:
        message = self.connection.recv_match(blocking=True, timeout=self.poll_s)
        now = time.monotonic()
        if message is not None:
            source_system = int(message.get_srcSystem())
            if source_system == self.expected_system_id:
                self._apply(message, now)
        self._refresh_derived(now)
        return self.snapshot()

    def snapshot(self) -> TelemetrySnapshot:
        return copy.deepcopy(self._snapshot)

    def _refresh_derived(self, now: float) -> None:
        value = self._snapshot
        value.monotonic_ts = now
        value.connected = value.heartbeat_age_s(now) <= self.heartbeat_timeout_s
        value.local_position_valid = (
            value.last_local_position_ts > 0
            and now - value.last_local_position_ts <= 1.0
        )
        value.global_position_valid = (
            value.last_global_position_ts > 0
            and now - value.last_global_position_ts <= 1.5
            and (value.gps_fix_type or 0) >= 3
        )
        value.rc_available = value.last_rc_ts > 0 and now - value.last_rc_ts <= 1.0

    def _apply(self, message: Any, now: float) -> None:
        value = self._snapshot
        name = message.get_type()
        if name == "BAD_DATA":
            return
        value.monotonic_ts = now
        value.last_rx_ts = now
        value.system_id = int(message.get_srcSystem())
        value.component_id = int(message.get_srcComponent())
        value.message_counts[name] = value.message_counts.get(name, 0) + 1

        if name == "HEARTBEAT":
            value.last_heartbeat_ts = now
            value.connected = True
            value.armed = bool(
                int(message.base_mode)
                & int(self._mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            )
            value.flight_mode = _normalize_px4_mode(
                self._mavutil.mode_string_v10(message)
            )
            value.system_status = int(message.system_status)
            critical_states = {
                int(getattr(self._mavutil.mavlink, name))
                for name in (
                    "MAV_STATE_CRITICAL",
                    "MAV_STATE_EMERGENCY",
                    "MAV_STATE_POWEROFF",
                    "MAV_STATE_FLIGHT_TERMINATION",
                )
                if hasattr(self._mavutil.mavlink, name)
            }
            value.failsafe = value.system_status in critical_states
        elif name == "LOCAL_POSITION_NED":
            value.last_local_position_ts = now
            value.x_m = float(message.x)
            value.y_m = float(message.y)
            value.z_m = float(message.z)
            value.vx_m_s = float(message.vx)
            value.vy_m_s = float(message.vy)
            value.vz_m_s = float(message.vz)
        elif name == "GLOBAL_POSITION_INT":
            value.last_global_position_ts = now
            value.latitude_deg = float(message.lat) / 1e7
            value.longitude_deg = float(message.lon) / 1e7
            value.relative_alt_m = float(message.relative_alt) / 1000.0
            value.vx_m_s = float(message.vx) / 100.0
            value.vy_m_s = float(message.vy) / 100.0
            value.vz_m_s = float(message.vz) / 100.0
        elif name == "ATTITUDE":
            value.roll_rad = float(message.roll)
            value.pitch_rad = float(message.pitch)
            value.yaw_rad = float(message.yaw)
            value.yaw_rate_rad_s = float(message.yawspeed)
        elif name == "GPS_RAW_INT":
            value.gps_fix_type = int(message.fix_type)
            value.satellites_visible = int(message.satellites_visible)
        elif name == "SYS_STATUS":
            voltage_mv = int(message.voltage_battery)
            current_ca = int(message.current_battery)
            remaining = int(message.battery_remaining)
            value.battery_voltage_v = None if voltage_mv == 65535 else voltage_mv / 1000.0
            value.battery_current_a = None if current_ca == -1 else current_ca / 100.0
            value.battery_remaining_pct = None if remaining == -1 else remaining
            if value.battery_remaining_pct is not None:
                value.battery_warning = value.battery_remaining_pct <= 30
        elif name == "BATTERY_STATUS":
            remaining = int(message.battery_remaining)
            value.battery_remaining_pct = None if remaining == -1 else remaining
            charge_state = int(getattr(message, "charge_state", 0))
            low = int(getattr(self._mavutil.mavlink, "MAV_BATTERY_CHARGE_STATE_LOW", 3))
            value.battery_warning = charge_state >= low or (
                value.battery_remaining_pct is not None
                and value.battery_remaining_pct <= 30
            )
        elif name == "RC_CHANNELS":
            value.last_rc_ts = now
            value.rc_rssi = int(message.rssi)
            count = max(0, min(18, int(message.chancount)))
            value.rc_channels_us = tuple(
                int(getattr(message, f"chan{index}_raw"))
                for index in range(1, count + 1)
            )
        elif name == "HOME_POSITION":
            value.home_position_valid = True
        elif name == "FENCE_STATUS":
            value.geofence_warning = int(message.breach_status) != 0
        elif name == "STATUSTEXT":
            text = message.text
            if isinstance(text, bytes):
                text = text.decode("utf-8", errors="replace")
            value.last_statustext = str(text).rstrip("\x00")
        elif name == "COMMAND_ACK":
            value.last_command_ack_ts = now
            value.last_command_ack_command = int(message.command)
            value.last_command_ack_result = int(message.result)


def _normalize_px4_mode(mode: str) -> str:
    value = mode.strip().upper()
    aliases = {
        "POSCTL": "POSITION",
        "POSITION": "POSITION",
        "ALTCTL": "ALTITUDE",
        "ALTITUDE": "ALTITUDE",
        "AUTO.LOITER": "HOLD",
        "LOITER": "HOLD",
        "HOLD": "HOLD",
        "AUTO.RTL": "RETURN",
    }
    return aliases.get(value, value)
