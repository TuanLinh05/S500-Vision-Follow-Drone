from __future__ import annotations

import json
from dataclasses import dataclass, fields
from pathlib import Path
from typing import Any, TypeVar


@dataclass(frozen=True, slots=True)
class VehicleConfig:
    connection: str = "auto"
    expected_system_id: int = 1
    heartbeat_timeout_s: float = 5.0
    read_poll_s: float = 0.05
    ai_enable_rc_channel: int = 0
    ai_enable_low_us: int = 1300
    ai_enable_high_us: int = 1700
    vision_udp_host: str = "127.0.0.1"
    vision_udp_port: int = 5800
    control_enabled: bool = False
    allow_serial_control: bool = False


@dataclass(frozen=True, slots=True)
class SafetyConfig:
    command_hz: float = 20.0
    prestream_s: float = 1.0
    offboard_start_timeout_s: float = 2.0
    telemetry_stale_s: float = 0.30
    frame_stale_s: float = 0.25
    track_stale_s: float = 0.30
    guidance_stale_s: float = 0.30
    lost_exit_s: float = 0.50
    command_gap_abort_s: float = 0.15
    max_yaw_rate_deg_s: float = 15.0
    max_yaw_accel_deg_s2: float = 45.0
    max_translation_m_s: float = 0.0
    max_actual_ground_speed_m_s: float = 1.0
    soft_fence_radius_m: float = 15.0
    soft_fence_height_m: float = 10.0
    require_global_position: bool = True
    require_home_position: bool = True
    required_entry_mode: str = "POSITION"

    def validate(self) -> None:
        positive = (
            "command_hz",
            "prestream_s",
            "offboard_start_timeout_s",
            "telemetry_stale_s",
            "frame_stale_s",
            "track_stale_s",
            "guidance_stale_s",
            "lost_exit_s",
            "command_gap_abort_s",
            "max_yaw_rate_deg_s",
            "max_yaw_accel_deg_s2",
            "max_actual_ground_speed_m_s",
            "soft_fence_radius_m",
            "soft_fence_height_m",
        )
        for name in positive:
            if float(getattr(self, name)) <= 0:
                raise ValueError(f"{name} must be > 0")
        if self.max_translation_m_s < 0:
            raise ValueError("max_translation_m_s must be >= 0")
        if self.command_hz < 10:
            raise ValueError("command_hz must be >= 10 Hz for this project")


T = TypeVar("T", VehicleConfig, SafetyConfig)


def _load_dataclass(path: str | Path, cls: type[T]) -> T:
    raw: dict[str, Any] = json.loads(Path(path).read_text(encoding="utf-8"))
    allowed = {item.name for item in fields(cls)}
    unknown = sorted(set(raw) - allowed)
    if unknown:
        raise ValueError(f"Unknown keys in {path}: {', '.join(unknown)}")
    value = cls(**raw)
    validate = getattr(value, "validate", None)
    if validate is not None:
        validate()
    return value


def load_vehicle_config(path: str | Path) -> VehicleConfig:
    return _load_dataclass(path, VehicleConfig)


def load_safety_config(path: str | Path) -> SafetyConfig:
    return _load_dataclass(path, SafetyConfig)
