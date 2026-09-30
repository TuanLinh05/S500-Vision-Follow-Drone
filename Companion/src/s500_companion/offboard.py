from __future__ import annotations

import math
import time
from typing import Any

from .serial_discovery import is_serial_connection


class ControlAuthorizationError(RuntimeError):
    pass


class MavlinkOffboardTransport:
    """Explicit setpoint transport with no arm/takeoff/actuator API."""

    PX4_MAIN_MODE_POSITION = 3
    PX4_MAIN_MODE_OFFBOARD = 6

    def __init__(
        self,
        connection: Any,
        connection_spec: str,
        config_control_enabled: bool,
        cli_control_enabled: bool,
        allow_serial_control: bool,
    ) -> None:
        if not config_control_enabled or not cli_control_enabled:
            raise ControlAuthorizationError(
                "Control requires config control_enabled=true and explicit CLI consent"
            )
        if is_serial_connection(connection_spec) and not allow_serial_control:
            raise ControlAuthorizationError(
                "Real-hardware control is locked. Complete P4/P5 first and set "
                "allow_serial_control=true only for an approved test."
            )
        self.connection = connection
        self.target_system = int(connection.target_system or 1)
        self.target_component = int(connection.target_component or 1)

    def send_body_velocity_yaw_rate(
        self,
        vx_m_s: float,
        vy_m_s: float,
        vz_m_s: float,
        yaw_rate_deg_s: float,
    ) -> None:
        values = (vx_m_s, vy_m_s, vz_m_s, yaw_rate_deg_s)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Non-finite setpoint rejected")
        type_mask = (
            (1 << 0)
            | (1 << 1)
            | (1 << 2)
            | (1 << 6)
            | (1 << 7)
            | (1 << 8)
            | (1 << 10)
        )
        self.connection.mav.set_position_target_local_ned_send(
            int(time.monotonic() * 1000) & 0xFFFFFFFF,
            self.target_system,
            self.target_component,
            8,  # MAV_FRAME_BODY_NED
            type_mask,
            0.0,
            0.0,
            0.0,
            float(vx_m_s),
            float(vy_m_s),
            float(vz_m_s),
            0.0,
            0.0,
            0.0,
            0.0,
            math.radians(float(yaw_rate_deg_s)),
        )

    def request_offboard(self) -> None:
        self._request_px4_main_mode(self.PX4_MAIN_MODE_OFFBOARD)

    def request_position(self) -> None:
        self._request_px4_main_mode(self.PX4_MAIN_MODE_POSITION)

    def _request_px4_main_mode(self, main_mode: int) -> None:
        self.connection.mav.command_long_send(
            self.target_system,
            self.target_component,
            176,  # MAV_CMD_DO_SET_MODE
            0,
            1,  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            float(main_mode),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )

