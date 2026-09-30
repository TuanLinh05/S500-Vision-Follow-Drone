"""Disarmed-only PX4 hardware MAVLink mode test.

This module deliberately does not share the Offboard transport: it can only
send MAV_CMD_DO_SET_MODE for a short allow-list of non-Offboard flight modes.
There is no arm, takeoff, land, actuator, or setpoint API here.
"""

from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .serial_discovery import is_serial_connection
from .telemetry import MavlinkTelemetryReader


MAV_CMD_DO_SET_MODE = 176
MAV_RESULT_ACCEPTED = 0
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1

# PX4 main mode, PX4 sub mode. Offboard is intentionally absent.
SAFE_MODE_COMMANDS: dict[str, tuple[int, int]] = {
    "STABILIZED": (7, 0),
    "ALTITUDE": (2, 0),
    "POSITION": (3, 0),
    "HOLD": (4, 3),  # AUTO.LOITER
}


class HardwareModeTestError(RuntimeError):
    pass


def normalize_safe_mode(value: str) -> str:
    aliases = {
        "POSCTL": "POSITION",
        "POSITION": "POSITION",
        "ALTCTL": "ALTITUDE",
        "ALTITUDE": "ALTITUDE",
        "AUTO.LOITER": "HOLD",
        "LOITER": "HOLD",
        "HOLD": "HOLD",
        "STABILIZED": "STABILIZED",
    }
    return aliases.get(value.strip().upper(), value.strip().upper())


@dataclass(slots=True)
class ModeTestReport:
    started_utc: str
    connection: str
    initial_mode: str = "UNKNOWN"
    target_mode: str = "POSITION"
    restored_mode: str = ""
    target_ack_result: int | None = None
    restore_ack_result: int | None = None
    status: str = "FAIL"
    reason: str = ""
    events: list[str] = field(default_factory=list)

    def write(self, output: Path) -> None:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(asdict(self), indent=2) + "\n", encoding="utf-8")


def _assert_disarmed(snapshot: Any) -> None:
    if not snapshot.connected:
        raise HardwareModeTestError("MAVLink disconnected")
    if snapshot.armed:
        raise HardwareModeTestError("Vehicle is armed; no mode command was sent")


def _wait_disarmed(reader: MavlinkTelemetryReader, duration_s: float) -> Any:
    deadline = time.monotonic() + duration_s
    last = reader.snapshot()
    while time.monotonic() < deadline:
        last = reader.poll()
        _assert_disarmed(last)
    return last


def _send_safe_mode(reader: MavlinkTelemetryReader, mode: str) -> float:
    normalized = normalize_safe_mode(mode)
    if normalized not in SAFE_MODE_COMMANDS:
        raise HardwareModeTestError(f"Mode is not allow-listed: {mode}")
    _assert_disarmed(reader.snapshot())
    main_mode, sub_mode = SAFE_MODE_COMMANDS[normalized]
    reader.connection.mav.command_long_send(
        int(reader.connection.target_system or 1),
        int(reader.connection.target_component or 1),
        MAV_CMD_DO_SET_MODE,
        0,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        float(main_mode),
        float(sub_mode),
        0.0,
        0.0,
        0.0,
        0.0,
    )
    return time.monotonic()


def _wait_for_mode_result(
    reader: MavlinkTelemetryReader,
    mode: str,
    sent_at: float,
    timeout_s: float,
) -> tuple[Any, int]:
    expected = normalize_safe_mode(mode)
    deadline = time.monotonic() + timeout_s
    ack_result: int | None = None
    while time.monotonic() < deadline:
        snapshot = reader.poll()
        _assert_disarmed(snapshot)
        if (
            snapshot.last_command_ack_ts >= sent_at
            and snapshot.last_command_ack_command == MAV_CMD_DO_SET_MODE
        ):
            ack_result = snapshot.last_command_ack_result
            if ack_result != MAV_RESULT_ACCEPTED:
                raise HardwareModeTestError(
                    f"PX4 rejected {expected} mode, MAV_RESULT={ack_result}"
                )
        if ack_result == MAV_RESULT_ACCEPTED and snapshot.flight_mode == expected:
            return snapshot, ack_result
    raise HardwareModeTestError(
        f"No accepted ACK plus HEARTBEAT confirmation for {expected} within "
        f"{timeout_s:.1f}s"
    )


def default_report_path() -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return Path("logs") / f"hardware_mode_test_{stamp}.json"


def run_hardware_mode_test(args: Any) -> int:
    if args.confirm != "DISARMED_MODE_TEST":
        raise HardwareModeTestError(
            "Refusing hardware test: use --confirm DISARMED_MODE_TEST"
        )
    if args.props_removed != "YES":
        raise HardwareModeTestError("Refusing hardware test: use --props-removed YES")
    if args.disarmed_observe_s < 1.0:
        raise HardwareModeTestError("disarmed observation must be at least 1 second")
    if args.timeout_s <= 0 or args.poll_s <= 0:
        raise HardwareModeTestError("timeout and poll interval must be positive")

    reader = MavlinkTelemetryReader(
        connection=args.connection,
        expected_system_id=args.expected_system_id,
        heartbeat_timeout_s=args.heartbeat_timeout_s,
        poll_s=args.poll_s,
    )
    if not is_serial_connection(reader.connection_spec):
        raise HardwareModeTestError("Hardware mode test requires a serial Pixhawk link")

    output = Path(args.output) if args.output else default_report_path()
    report = ModeTestReport(
        started_utc=datetime.now(timezone.utc).isoformat(),
        connection=reader.connection_spec,
        target_mode=normalize_safe_mode(args.target_mode),
    )
    try:
        with reader:
            initial = _wait_disarmed(reader, args.disarmed_observe_s)
            report.initial_mode = normalize_safe_mode(initial.flight_mode)
            if report.initial_mode not in SAFE_MODE_COMMANDS:
                raise HardwareModeTestError(
                    f"Initial mode is not allow-listed for restoration: {initial.flight_mode}"
                )
            report.events.append("disarmed_observation_passed")

            sent_at = _send_safe_mode(reader, report.target_mode)
            report.events.append(f"requested_{report.target_mode}")
            _, report.target_ack_result = _wait_for_mode_result(
                reader, report.target_mode, sent_at, args.timeout_s
            )
            report.events.append(f"confirmed_{report.target_mode}")

            if report.target_mode != report.initial_mode:
                sent_at = _send_safe_mode(reader, report.initial_mode)
                report.events.append(f"requested_restore_{report.initial_mode}")
                restored, report.restore_ack_result = _wait_for_mode_result(
                    reader, report.initial_mode, sent_at, args.timeout_s
                )
                report.restored_mode = restored.flight_mode
                report.events.append(f"confirmed_restore_{report.initial_mode}")
            else:
                report.restored_mode = report.initial_mode

        report.status = "PASS"
        report.reason = "Disarmed mode command ACKed and mode restored"
        return 0
    except HardwareModeTestError as exc:
        report.reason = str(exc)
        raise
    finally:
        report.write(output)
        print(json.dumps(asdict(report), ensure_ascii=False))
