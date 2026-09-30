from __future__ import annotations

from dataclasses import dataclass
from math import copysign

from .config import SafetyConfig
from .models import (
    GuardAction,
    GuidanceProposal,
    OperatorInput,
    SupervisorOutput,
    SupervisorState,
    TelemetrySnapshot,
    VisionStatus,
)


@dataclass(frozen=True, slots=True)
class HealthResult:
    ok: bool
    reason: str = ""


class SafetySupervisor:
    """Deterministic Offboard permission state machine.

    It never arms, takes off, lands, kills motors, or emits direct actuator
    commands. The initial profile only permits zero body velocity plus a
    limited yaw rate.
    """

    def __init__(self, config: SafetyConfig) -> None:
        config.validate()
        self.config = config
        self.state = SupervisorState.DISABLED
        self._seen_ai_off = False
        self._last_ai_enable = False
        self._prestream_started_ts = 0.0
        self._offboard_requested_ts = 0.0
        self._lost_started_ts = 0.0
        self._last_step_ts = 0.0
        self._last_yaw_rate_deg_s = 0.0

    def reset(self) -> None:
        self.state = SupervisorState.DISABLED
        self._seen_ai_off = False
        self._last_ai_enable = False
        self._prestream_started_ts = 0.0
        self._offboard_requested_ts = 0.0
        self._lost_started_ts = 0.0
        self._last_step_ts = 0.0
        self._last_yaw_rate_deg_s = 0.0

    def step(
        self,
        now: float,
        telemetry: TelemetrySnapshot,
        vision: VisionStatus,
        guidance: GuidanceProposal,
        operator: OperatorInput,
    ) -> SupervisorOutput:
        if self._last_step_ts > 0 and now < self._last_step_ts:
            self.reset()
            return self._output(now, reason="monotonic_clock_reversed")

        dt = 0.0 if self._last_step_ts <= 0 else now - self._last_step_ts
        self._last_step_ts = now
        rising_ai = operator.ai_enable and not self._last_ai_enable
        self._last_ai_enable = operator.ai_enable
        if not operator.ai_enable:
            self._seen_ai_off = True

        if operator.force_disable:
            return self._begin_exit(now, telemetry, "operator_force_disable")

        common = self._check_common(now, telemetry)
        if not common.ok:
            if common.reason in {"telemetry_disconnected", "telemetry_stale"}:
                self.state = SupervisorState.FAILSAFE
                return self._output(now, reason=common.reason)
            return self._begin_exit(now, telemetry, common.reason)

        if not operator.ai_enable:
            if self.state in {
                SupervisorState.PRESTREAM,
                SupervisorState.OFFBOARD_YAW,
                SupervisorState.EXITING,
            }:
                return self._begin_exit(now, telemetry, "ai_enable_off")
            ready = self._check_entry(now, telemetry, vision, guidance)
            self.state = SupervisorState.READY if ready.ok else SupervisorState.OBSERVE
            return self._output(now, reason="" if ready.ok else ready.reason)

        if not self._seen_ai_off:
            self.state = SupervisorState.DISABLED
            return self._output(now, reason="ai_off_on_cycle_required")

        if self.state in {SupervisorState.DISABLED, SupervisorState.OBSERVE}:
            reason = "ai_enable_must_rise_from_ready" if rising_ai else "ai_cycle_required"
            return self._output(now, reason=reason)

        if self.state == SupervisorState.READY:
            if not rising_ai:
                return self._output(now, reason="ai_enable_edge_required")
            entry = self._check_entry(now, telemetry, vision, guidance)
            if not entry.ok:
                self.state = SupervisorState.OBSERVE
                return self._output(now, reason=entry.reason)
            self.state = SupervisorState.PRESTREAM
            self._prestream_started_ts = now
            self._offboard_requested_ts = 0.0
            return self._zero(now, permit=True)

        if self.state == SupervisorState.PRESTREAM:
            active = self._check_active(now, telemetry, vision, guidance)
            if not active.ok:
                return self._begin_exit(now, telemetry, active.reason)
            if telemetry.flight_mode == "OFFBOARD":
                self.state = SupervisorState.OFFBOARD_YAW
                self._lost_started_ts = 0.0
                return self._limited_guidance(now, guidance, dt)
            elapsed = now - self._prestream_started_ts
            if self._offboard_requested_ts <= 0 and elapsed >= self.config.prestream_s:
                self._offboard_requested_ts = now
                return self._zero(now, permit=True, action=GuardAction.START_OFFBOARD)
            if (
                self._offboard_requested_ts > 0
                and now - self._offboard_requested_ts
                > self.config.offboard_start_timeout_s
            ):
                return self._begin_exit(now, telemetry, "offboard_start_timeout")
            return self._zero(now, permit=True)

        if self.state == SupervisorState.OFFBOARD_YAW:
            if telemetry.flight_mode != "OFFBOARD":
                self.state = SupervisorState.OBSERVE
                self._last_yaw_rate_deg_s = 0.0
                return self._output(now, reason="pilot_or_px4_left_offboard")
            active = self._check_active(now, telemetry, vision, guidance)
            if not active.ok:
                if active.reason in {
                    "frame_stale",
                    "track_stale",
                    "target_lost",
                    "guidance_stale",
                }:
                    if self._lost_started_ts <= 0:
                        self._lost_started_ts = now
                    if now - self._lost_started_ts < self.config.lost_exit_s:
                        return self._zero(now, permit=True, reason=active.reason)
                return self._begin_exit(now, telemetry, active.reason)
            self._lost_started_ts = 0.0
            return self._limited_guidance(now, guidance, dt)

        if self.state == SupervisorState.EXITING:
            if telemetry.flight_mode == "OFFBOARD":
                return self._zero(
                    now,
                    permit=True,
                    action=GuardAction.STOP_TO_POSITION,
                    reason="exit_pending",
                )
            self.state = SupervisorState.OBSERVE
            self._last_yaw_rate_deg_s = 0.0
            return self._output(now, reason="offboard_exited")

        return self._output(now, reason="failsafe_latched_cycle_ai_off")

    def _check_common(self, now: float, telemetry: TelemetrySnapshot) -> HealthResult:
        if not telemetry.connected:
            return HealthResult(False, "telemetry_disconnected")
        if telemetry.age_s(now) > self.config.telemetry_stale_s:
            return HealthResult(False, "telemetry_stale")
        if telemetry.system_id != 1:
            return HealthResult(False, "unexpected_system_id")
        if not telemetry.rc_available:
            return HealthResult(False, "rc_unavailable")
        if telemetry.battery_warning:
            return HealthResult(False, "battery_warning")
        if telemetry.failsafe:
            return HealthResult(False, "px4_failsafe")
        if telemetry.geofence_warning:
            return HealthResult(False, "geofence_warning")
        if not telemetry.local_position_valid:
            return HealthResult(False, "local_position_invalid")
        if self.config.require_global_position and not telemetry.global_position_valid:
            return HealthResult(False, "global_position_invalid")
        if self.config.require_home_position and not telemetry.home_position_valid:
            return HealthResult(False, "home_position_invalid")
        distance = telemetry.horizontal_distance_m()
        if distance is not None and distance >= self.config.soft_fence_radius_m:
            return HealthResult(False, "soft_fence_horizontal")
        height = telemetry.relative_alt_m
        if height is None and telemetry.z_m is not None:
            height = max(0.0, -telemetry.z_m)
        if height is not None and height >= self.config.soft_fence_height_m:
            return HealthResult(False, "soft_fence_vertical")
        speed = telemetry.ground_speed_m_s()
        if speed is not None and speed > self.config.max_actual_ground_speed_m_s:
            return HealthResult(False, "unexpected_ground_speed")
        return HealthResult(True)

    def _check_entry(
        self,
        now: float,
        telemetry: TelemetrySnapshot,
        vision: VisionStatus,
        guidance: GuidanceProposal,
    ) -> HealthResult:
        if not telemetry.armed:
            return HealthResult(False, "vehicle_not_armed_by_pilot")
        if telemetry.flight_mode != self.config.required_entry_mode:
            return HealthResult(False, "entry_mode_not_position")
        return self._check_active(now, telemetry, vision, guidance)

    def _check_active(
        self,
        now: float,
        telemetry: TelemetrySnapshot,
        vision: VisionStatus,
        guidance: GuidanceProposal,
    ) -> HealthResult:
        if not telemetry.armed:
            return HealthResult(False, "vehicle_disarmed")
        if telemetry.flight_mode not in {
            self.config.required_entry_mode,
            "OFFBOARD",
        }:
            return HealthResult(False, "unexpected_flight_mode")
        if not vision.camera_ok:
            return HealthResult(False, "camera_unhealthy")
        if not vision.detector_ok:
            return HealthResult(False, "detector_unhealthy")
        if not vision.tracker_ok:
            return HealthResult(False, "tracker_unhealthy")
        if now - vision.frame_ts > self.config.frame_stale_s:
            return HealthResult(False, "frame_stale")
        if now - vision.track_ts > self.config.track_stale_s:
            return HealthResult(False, "track_stale")
        if vision.track_state != "TRACKED" or vision.selected_track_id is None:
            return HealthResult(False, "target_lost")
        if not guidance.finite():
            return HealthResult(False, "guidance_non_finite")
        if now - guidance.created_ts > self.config.guidance_stale_s:
            return HealthResult(False, "guidance_stale")
        if guidance.track_id != vision.selected_track_id:
            return HealthResult(False, "guidance_track_id_mismatch")
        translation = max(
            abs(guidance.vx_body_m_s),
            abs(guidance.vy_body_m_s),
            abs(guidance.vz_body_m_s),
        )
        if translation > self.config.max_translation_m_s + 1e-6:
            return HealthResult(False, "translation_forbidden_in_yaw_only")
        return HealthResult(True)

    def _limited_guidance(
        self, now: float, guidance: GuidanceProposal, dt: float
    ) -> SupervisorOutput:
        target = max(
            -self.config.max_yaw_rate_deg_s,
            min(self.config.max_yaw_rate_deg_s, guidance.yaw_rate_deg_s),
        )
        if dt > 0:
            max_delta = self.config.max_yaw_accel_deg_s2 * dt
            delta = target - self._last_yaw_rate_deg_s
            if abs(delta) > max_delta:
                target = self._last_yaw_rate_deg_s + copysign(max_delta, delta)
        self._last_yaw_rate_deg_s = target
        return SupervisorOutput(
            monotonic_ts=now,
            state=self.state,
            permit_setpoint=True,
            vx_body_m_s=0.0,
            vy_body_m_s=0.0,
            vz_body_m_s=0.0,
            yaw_rate_deg_s=target,
        )

    def _begin_exit(
        self, now: float, telemetry: TelemetrySnapshot, reason: str
    ) -> SupervisorOutput:
        was_active = self.state in {
            SupervisorState.PRESTREAM,
            SupervisorState.OFFBOARD_YAW,
            SupervisorState.EXITING,
        }
        self._last_yaw_rate_deg_s = 0.0
        if was_active and telemetry.connected:
            self.state = SupervisorState.EXITING
            return self._zero(
                now,
                permit=True,
                action=GuardAction.STOP_TO_POSITION,
                reason=reason,
            )
        self.state = SupervisorState.OBSERVE
        return self._output(now, reason=reason)

    def _zero(
        self,
        now: float,
        permit: bool,
        action: GuardAction = GuardAction.NONE,
        reason: str = "",
    ) -> SupervisorOutput:
        self._last_yaw_rate_deg_s = 0.0
        return SupervisorOutput(
            monotonic_ts=now,
            state=self.state,
            action=action,
            permit_setpoint=permit,
            reject_reason=reason,
        )

    def _output(self, now: float, reason: str = "") -> SupervisorOutput:
        return SupervisorOutput(
            monotonic_ts=now,
            state=self.state,
            reject_reason=reason,
        )
