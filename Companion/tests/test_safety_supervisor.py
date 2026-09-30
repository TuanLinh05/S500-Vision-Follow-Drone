import math
import unittest

from s500_companion.config import SafetyConfig
from s500_companion.flight_guard import FlightGuard
from s500_companion.models import (
    GuardAction,
    GuidanceProposal,
    OperatorInput,
    SupervisorState,
    TelemetrySnapshot,
    VisionStatus,
)
from s500_companion.safety import SafetySupervisor


def healthy_telemetry(now: float, mode: str = "POSITION") -> TelemetrySnapshot:
    return TelemetrySnapshot(
        monotonic_ts=now,
        last_rx_ts=now,
        last_heartbeat_ts=now,
        connected=True,
        system_id=1,
        component_id=1,
        armed=True,
        flight_mode=mode,
        local_position_valid=True,
        global_position_valid=True,
        home_position_valid=True,
        rc_available=True,
        x_m=0.0,
        y_m=0.0,
        z_m=-4.0,
        vx_m_s=0.0,
        vy_m_s=0.0,
        vz_m_s=0.0,
        relative_alt_m=4.0,
        battery_remaining_pct=80,
    )


def healthy_vision(now: float) -> VisionStatus:
    return VisionStatus(
        camera_ok=True,
        detector_ok=True,
        tracker_ok=True,
        frame_ts=now,
        track_ts=now,
        selected_track_id=7,
        track_state="TRACKED",
        confidence=0.9,
    )


def guidance(now: float, yaw: float = 5.0) -> GuidanceProposal:
    return GuidanceProposal(
        created_ts=now,
        sequence=1,
        track_id=7,
        yaw_rate_deg_s=yaw,
    )


class FakeTransport:
    def __init__(self) -> None:
        self.setpoints: list[tuple[float, float, float, float]] = []
        self.offboard_requests = 0
        self.position_requests = 0

    def send_body_velocity_yaw_rate(
        self, vx_m_s: float, vy_m_s: float, vz_m_s: float, yaw_rate_deg_s: float
    ) -> None:
        self.setpoints.append((vx_m_s, vy_m_s, vz_m_s, yaw_rate_deg_s))

    def request_offboard(self) -> None:
        self.offboard_requests += 1

    def request_position(self) -> None:
        self.position_requests += 1


class SafetySupervisorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = SafetyConfig()
        self.supervisor = SafetySupervisor(self.config)

    def _ready(self, now: float = 100.0) -> None:
        result = self.supervisor.step(
            now,
            healthy_telemetry(now),
            healthy_vision(now),
            guidance(now),
            OperatorInput(ai_enable=False),
        )
        self.assertEqual(result.state, SupervisorState.READY)

    def _offboard(self, now: float = 100.0) -> float:
        self._ready(now)
        now += 0.01
        result = self.supervisor.step(
            now,
            healthy_telemetry(now),
            healthy_vision(now),
            guidance(now),
            OperatorInput(ai_enable=True),
        )
        self.assertEqual(result.state, SupervisorState.PRESTREAM)
        now += self.config.prestream_s + 0.01
        result = self.supervisor.step(
            now,
            healthy_telemetry(now),
            healthy_vision(now),
            guidance(now),
            OperatorInput(ai_enable=True),
        )
        self.assertEqual(result.action, GuardAction.START_OFFBOARD)
        now += 0.01
        result = self.supervisor.step(
            now,
            healthy_telemetry(now, "OFFBOARD"),
            healthy_vision(now),
            guidance(now),
            OperatorInput(ai_enable=True),
        )
        self.assertEqual(result.state, SupervisorState.OFFBOARD_YAW)
        return now

    def test_safe01_stale_guidance_exits_after_grace(self) -> None:
        now = self._offboard()
        stale_ts = now - 1.0
        result = self.supervisor.step(
            now + 0.1,
            healthy_telemetry(now + 0.1, "OFFBOARD"),
            healthy_vision(stale_ts),
            guidance(stale_ts),
            OperatorInput(ai_enable=True),
        )
        self.assertTrue(result.permit_setpoint)
        self.assertEqual(result.yaw_rate_deg_s, 0.0)
        result = self.supervisor.step(
            now + 0.7,
            healthy_telemetry(now + 0.7, "OFFBOARD"),
            healthy_vision(stale_ts),
            guidance(stale_ts),
            OperatorInput(ai_enable=True),
        )
        self.assertEqual(result.action, GuardAction.STOP_TO_POSITION)

    def test_safe05_nan_is_rejected(self) -> None:
        self._ready()
        now = 100.01
        bad = guidance(now)
        bad.yaw_rate_deg_s = math.nan
        result = self.supervisor.step(
            now,
            healthy_telemetry(now),
            healthy_vision(now),
            bad,
            OperatorInput(ai_enable=True),
        )
        self.assertFalse(result.permit_setpoint)
        self.assertEqual(result.reject_reason, "guidance_non_finite")

    def test_safe05_yaw_is_clamped_and_translation_is_zero(self) -> None:
        now = self._offboard()
        result = self.supervisor.step(
            now + 0.1,
            healthy_telemetry(now + 0.1, "OFFBOARD"),
            healthy_vision(now + 0.1),
            guidance(now + 0.1, yaw=100.0),
            OperatorInput(ai_enable=True),
        )
        self.assertLessEqual(abs(result.yaw_rate_deg_s), 15.0)
        self.assertEqual((result.vx_body_m_s, result.vy_body_m_s, result.vz_body_m_s), (0, 0, 0))

    def test_safe05_translation_proposal_is_rejected(self) -> None:
        self._ready()
        now = 100.01
        value = guidance(now)
        value.vx_body_m_s = 0.1
        result = self.supervisor.step(
            now,
            healthy_telemetry(now),
            healthy_vision(now),
            value,
            OperatorInput(ai_enable=True),
        )
        self.assertEqual(result.reject_reason, "translation_forbidden_in_yaw_only")

    def test_safe08_operator_off_requests_position(self) -> None:
        now = self._offboard()
        result = self.supervisor.step(
            now + 0.01,
            healthy_telemetry(now + 0.01, "OFFBOARD"),
            healthy_vision(now + 0.01),
            guidance(now + 0.01),
            OperatorInput(ai_enable=False),
        )
        self.assertEqual(result.action, GuardAction.STOP_TO_POSITION)

    def test_safe09_boot_with_ai_on_is_blocked(self) -> None:
        now = 100.0
        result = self.supervisor.step(
            now,
            healthy_telemetry(now),
            healthy_vision(now),
            guidance(now),
            OperatorInput(ai_enable=True),
        )
        self.assertEqual(result.state, SupervisorState.DISABLED)
        self.assertEqual(result.reject_reason, "ai_off_on_cycle_required")

    def test_safe11_soft_fence_blocks_entry(self) -> None:
        now = 100.0
        telemetry = healthy_telemetry(now)
        telemetry.x_m = self.config.soft_fence_radius_m
        result = self.supervisor.step(
            now,
            telemetry,
            healthy_vision(now),
            guidance(now),
            OperatorInput(ai_enable=False),
        )
        self.assertEqual(result.reject_reason, "soft_fence_horizontal")

    def test_safe13_invalid_local_position_blocks_entry(self) -> None:
        now = 100.0
        telemetry = healthy_telemetry(now)
        telemetry.local_position_valid = False
        result = self.supervisor.step(
            now,
            telemetry,
            healthy_vision(now),
            guidance(now),
            OperatorInput(ai_enable=False),
        )
        self.assertEqual(result.reject_reason, "local_position_invalid")

    def test_safe14_guard_gap_forces_exit(self) -> None:
        transport = FakeTransport()
        guard = FlightGuard(self.config, transport)
        now = 100.0
        guard.tick(
            now,
            healthy_telemetry(now),
            healthy_vision(now),
            guidance(now),
            OperatorInput(ai_enable=False),
        )
        now += 0.01
        guard.tick(
            now,
            healthy_telemetry(now),
            healthy_vision(now),
            guidance(now),
            OperatorInput(ai_enable=True),
        )
        result = guard.tick(
            now + self.config.command_gap_abort_s + 0.01,
            healthy_telemetry(now + self.config.command_gap_abort_s + 0.01),
            healthy_vision(now + self.config.command_gap_abort_s + 0.01),
            guidance(now + self.config.command_gap_abort_s + 0.01),
            OperatorInput(ai_enable=True),
        )
        self.assertEqual(result.action, GuardAction.STOP_TO_POSITION)
        self.assertGreaterEqual(transport.position_requests, 1)


if __name__ == "__main__":
    unittest.main()

