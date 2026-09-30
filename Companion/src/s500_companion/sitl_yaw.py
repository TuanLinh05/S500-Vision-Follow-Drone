from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from .config import load_safety_config, load_vehicle_config
from .flight_guard import FlightGuard
from .models import GuidanceProposal, OperatorInput, SupervisorState, VisionStatus
from .offboard import MavlinkOffboardTransport
from .serial_discovery import is_serial_connection
from .telemetry import MavlinkTelemetryReader


def run(args: argparse.Namespace) -> int:
    vehicle_config = load_vehicle_config(args.vehicle_config)
    safety_config = load_safety_config(args.safety_config)
    connection_spec = args.connection or vehicle_config.connection
    if is_serial_connection(connection_spec):
        raise RuntimeError("This runner is SITL-only and refuses serial hardware")
    if not args.enable_control or args.confirm != "SITL_ONLY":
        raise RuntimeError(
            "SITL control needs both --enable-control and --confirm SITL_ONLY"
        )

    reader = MavlinkTelemetryReader(
        connection=connection_spec,
        expected_system_id=vehicle_config.expected_system_id,
        heartbeat_timeout_s=vehicle_config.heartbeat_timeout_s,
        poll_s=vehicle_config.read_poll_s,
    )
    with reader:
        transport = MavlinkOffboardTransport(
            reader.connection,
            reader.connection_spec,
            config_control_enabled=vehicle_config.control_enabled,
            cli_control_enabled=True,
            allow_serial_control=False,
        )
        guard = FlightGuard(safety_config, transport)
        period = 1.0 / safety_config.command_hz
        started = time.monotonic()
        ready_since = 0.0
        enable_since = 0.0
        last_state = None
        last_action = None
        last_reason = None
        while time.monotonic() - started < args.wait_ready_s + args.active_s + 10.0:
            loop_started = time.monotonic()
            telemetry = reader.poll()
            now = time.monotonic()
            if args.assume_sitl_rc:
                telemetry.rc_available = True
            vision = VisionStatus(
                camera_ok=True,
                detector_ok=True,
                tracker_ok=True,
                frame_ts=now,
                track_ts=now,
                selected_track_id=1,
                track_state="TRACKED",
                confidence=1.0,
            )
            proposal = GuidanceProposal(
                created_ts=now,
                sequence=int((now - started) * safety_config.command_hz),
                track_id=1,
                yaw_rate_deg_s=args.yaw_rate,
            )
            ai_enable = ready_since > 0
            if guard.supervisor.state == SupervisorState.READY and ready_since <= 0:
                ready_since = now
                ai_enable = False
            elif ready_since > 0 and now - ready_since >= 1.0:
                ai_enable = True
                if enable_since <= 0:
                    enable_since = now
            force_disable = enable_since > 0 and now - enable_since >= args.active_s
            output = guard.tick(
                now,
                telemetry,
                vision,
                proposal,
                OperatorInput(ai_enable=ai_enable, force_disable=force_disable),
            )
            if (
                output.state != last_state
                or output.action != last_action
                or output.reject_reason != last_reason
            ):
                print(json.dumps(output.as_dict(), ensure_ascii=False))
                last_state = output.state
                last_action = output.action
                last_reason = output.reject_reason
            if force_disable and telemetry.flight_mode != "OFFBOARD":
                print("SITL yaw test completed and PX4 left Offboard.")
                return 0
            if ready_since <= 0 and now - started > args.wait_ready_s:
                raise TimeoutError(
                    "SITL never reached READY. Arm/take off manually in Position and "
                    "check local/global/home/RC telemetry."
                )
            sleep_s = period - (time.monotonic() - loop_started)
            if sleep_s > 0:
                time.sleep(sleep_s)
    return 3


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description="Explicitly gated yaw-only PX4 SITL test")
    parser.add_argument("--vehicle-config", type=Path, default=root / "config/sitl_vehicle.json")
    parser.add_argument("--safety-config", type=Path, default=root / "config/safety.json")
    parser.add_argument("--connection")
    parser.add_argument("--yaw-rate", type=float, default=5.0)
    parser.add_argument("--active-s", type=float, default=5.0)
    parser.add_argument("--wait-ready-s", type=float, default=60.0)
    parser.add_argument("--assume-sitl-rc", action="store_true")
    parser.add_argument("--enable-control", action="store_true")
    parser.add_argument("--confirm", default="")
    try:
        return run(parser.parse_args())
    except (RuntimeError, TimeoutError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
