from __future__ import annotations

from dataclasses import replace
from typing import Protocol

from .config import SafetyConfig
from .models import (
    GuardAction,
    GuidanceProposal,
    OperatorInput,
    SupervisorOutput,
    TelemetrySnapshot,
    VisionStatus,
)
from .safety import SafetySupervisor


class OffboardTransport(Protocol):
    def send_body_velocity_yaw_rate(
        self,
        vx_m_s: float,
        vy_m_s: float,
        vz_m_s: float,
        yaw_rate_deg_s: float,
    ) -> None: ...

    def request_offboard(self) -> None: ...

    def request_position(self) -> None: ...


class FlightGuard:
    """Sole control-output owner.

    With ``transport=None`` the guard is strictly dry-run and cannot send any
    MAVLink control message.
    """

    def __init__(
        self,
        config: SafetyConfig,
        transport: OffboardTransport | None = None,
    ) -> None:
        self.config = config
        self.supervisor = SafetySupervisor(config)
        self.transport = transport
        self._last_tick_ts = 0.0

    @property
    def dry_run(self) -> bool:
        return self.transport is None

    def tick(
        self,
        now: float,
        telemetry: TelemetrySnapshot,
        vision: VisionStatus,
        guidance: GuidanceProposal,
        operator: OperatorInput,
    ) -> SupervisorOutput:
        if (
            self._last_tick_ts > 0
            and now - self._last_tick_ts > self.config.command_gap_abort_s
        ):
            operator = replace(operator, force_disable=True)
        self._last_tick_ts = now
        output = self.supervisor.step(now, telemetry, vision, guidance, operator)
        if self.transport is not None:
            self._apply(output)
        return output

    def _apply(self, output: SupervisorOutput) -> None:
        assert self.transport is not None
        if output.permit_setpoint:
            self.transport.send_body_velocity_yaw_rate(
                output.vx_body_m_s,
                output.vy_body_m_s,
                output.vz_body_m_s,
                output.yaw_rate_deg_s,
            )
        if output.action == GuardAction.START_OFFBOARD:
            self.transport.request_offboard()
        elif output.action == GuardAction.STOP_TO_POSITION:
            self.transport.request_position()

