from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
from math import isfinite
from typing import Any


class SupervisorState(str, Enum):
    DISABLED = "DISABLED"
    OBSERVE = "OBSERVE"
    READY = "READY"
    PRESTREAM = "PRESTREAM"
    OFFBOARD_YAW = "OFFBOARD_YAW"
    EXITING = "EXITING"
    FAILSAFE = "FAILSAFE"


class GuardAction(str, Enum):
    NONE = "NONE"
    START_OFFBOARD = "START_OFFBOARD"
    STOP_TO_POSITION = "STOP_TO_POSITION"


@dataclass(slots=True)
class TelemetrySnapshot:
    monotonic_ts: float = 0.0
    last_rx_ts: float = 0.0
    last_heartbeat_ts: float = 0.0
    last_local_position_ts: float = 0.0
    last_global_position_ts: float = 0.0
    last_rc_ts: float = 0.0
    last_command_ack_ts: float = 0.0
    connected: bool = False
    system_id: int | None = None
    component_id: int | None = None
    armed: bool = False
    flight_mode: str = "UNKNOWN"
    system_status: int | None = None
    local_position_valid: bool = False
    global_position_valid: bool = False
    home_position_valid: bool = False
    rc_available: bool = False
    battery_warning: bool = False
    failsafe: bool = False
    geofence_warning: bool = False
    x_m: float | None = None
    y_m: float | None = None
    z_m: float | None = None
    vx_m_s: float | None = None
    vy_m_s: float | None = None
    vz_m_s: float | None = None
    roll_rad: float | None = None
    pitch_rad: float | None = None
    yaw_rad: float | None = None
    yaw_rate_rad_s: float | None = None
    latitude_deg: float | None = None
    longitude_deg: float | None = None
    relative_alt_m: float | None = None
    battery_voltage_v: float | None = None
    battery_current_a: float | None = None
    battery_remaining_pct: int | None = None
    gps_fix_type: int | None = None
    satellites_visible: int | None = None
    rc_rssi: int | None = None
    rc_channels_us: tuple[int, ...] = ()
    last_statustext: str = ""
    last_command_ack_command: int | None = None
    last_command_ack_result: int | None = None
    message_counts: dict[str, int] = field(default_factory=dict)

    def age_s(self, now: float) -> float:
        if self.last_rx_ts <= 0:
            return float("inf")
        return max(0.0, now - self.last_rx_ts)

    def heartbeat_age_s(self, now: float) -> float:
        if self.last_heartbeat_ts <= 0:
            return float("inf")
        return max(0.0, now - self.last_heartbeat_ts)

    def ground_speed_m_s(self) -> float | None:
        if self.vx_m_s is None or self.vy_m_s is None:
            return None
        return (self.vx_m_s**2 + self.vy_m_s**2) ** 0.5

    def horizontal_distance_m(self) -> float | None:
        if self.x_m is None or self.y_m is None:
            return None
        return (self.x_m**2 + self.y_m**2) ** 0.5

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class VisionStatus:
    camera_ok: bool = False
    detector_ok: bool = False
    tracker_ok: bool = False
    frame_ts: float = 0.0
    track_ts: float = 0.0
    selected_track_id: int | None = None
    track_state: str = "LOST"
    confidence: float = 0.0


@dataclass(slots=True)
class GuidanceProposal:
    created_ts: float = 0.0
    sequence: int = 0
    track_id: int | None = None
    vx_body_m_s: float = 0.0
    vy_body_m_s: float = 0.0
    vz_body_m_s: float = 0.0
    yaw_rate_deg_s: float = 0.0

    def finite(self) -> bool:
        return all(
            isfinite(value)
            for value in (
                self.created_ts,
                self.vx_body_m_s,
                self.vy_body_m_s,
                self.vz_body_m_s,
                self.yaw_rate_deg_s,
            )
        )


@dataclass(slots=True)
class OperatorInput:
    ai_enable: bool = False
    force_disable: bool = False


@dataclass(slots=True)
class SupervisorOutput:
    monotonic_ts: float
    state: SupervisorState
    action: GuardAction = GuardAction.NONE
    permit_setpoint: bool = False
    vx_body_m_s: float = 0.0
    vy_body_m_s: float = 0.0
    vz_body_m_s: float = 0.0
    yaw_rate_deg_s: float = 0.0
    reject_reason: str = ""

    def as_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["state"] = self.state.value
        data["action"] = self.action.value
        return data
