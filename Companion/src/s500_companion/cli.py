from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Sequence

from .config import load_vehicle_config
from .recorder import TelemetryRecorder
from .param_audit import audit_file, write_audit_json
from .serial_discovery import list_serial_candidates
from .telemetry import MavlinkTelemetryReader, MissingDependencyError
from .validation import validate_p1_csv, write_validation_json
from .hardware_mode_test import SAFE_MODE_COMMANDS, run_hardware_mode_test


def _default_config() -> Path:
    return Path(__file__).resolve().parents[2] / "config" / "vehicle.json"


def _reader(args: argparse.Namespace) -> MavlinkTelemetryReader:
    config = load_vehicle_config(args.vehicle_config)
    connection = args.connection or config.connection
    return MavlinkTelemetryReader(
        connection=connection,
        expected_system_id=config.expected_system_id,
        heartbeat_timeout_s=config.heartbeat_timeout_s,
        poll_s=config.read_poll_s,
    )


def _status(snapshot: object) -> str:
    value = snapshot
    return json.dumps(
        {
            "connected": value.connected,
            "system_id": value.system_id,
            "armed": value.armed,
            "mode": value.flight_mode,
            "local_position": value.local_position_valid,
            "global_position": value.global_position_valid,
            "home": value.home_position_valid,
            "rc": value.rc_available,
            "battery_pct": value.battery_remaining_pct,
            "gps_fix": value.gps_fix_type,
            "satellites": value.satellites_visible,
            "last_text": value.last_statustext,
        },
        ensure_ascii=False,
    )


def cmd_discover(args: argparse.Namespace) -> int:
    candidates = list_serial_candidates(args.by_id_root)
    if not candidates:
        print(f"No devices found under {args.by_id_root}", file=sys.stderr)
        return 2
    for item in candidates:
        print(f"{item.path} -> {item.resolved_path} score={item.score}")
    return 0


def cmd_probe(args: argparse.Namespace) -> int:
    reader = _reader(args)
    print(f"Opening read-only MAVLink: {reader.connection_spec}")
    started = time.monotonic()
    next_print = started
    try:
        with reader:
            while time.monotonic() - started < args.duration:
                snapshot = reader.poll()
                now = time.monotonic()
                if now >= next_print:
                    print(_status(snapshot))
                    next_print = now + 1.0
    except KeyboardInterrupt:
        return 130
    print("Probe complete; no control command was sent.")
    return 0


def cmd_record(args: argparse.Namespace) -> int:
    reader = _reader(args)
    output = Path(args.output) if args.output else _default_log_path()
    started = time.monotonic()
    next_write = started
    interval = 1.0 / args.rate
    print(f"Recording read-only MAVLink from {reader.connection_spec} to {output}")
    try:
        with reader, TelemetryRecorder(output) as recorder:
            while time.monotonic() - started < args.duration:
                snapshot = reader.poll()
                now = time.monotonic()
                if now >= next_write:
                    recorder.write(snapshot)
                    next_write += interval
    except KeyboardInterrupt:
        print("Recording stopped by operator.")
        return 130
    print(f"Recording complete: {output}")
    return 0


def _default_log_path() -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return Path("logs") / f"telemetry_{stamp}.csv"


def cmd_validate_log(args: argparse.Namespace) -> int:
    result = validate_p1_csv(
        args.input,
        expected_system_id=args.expected_system_id,
        minimum_duration_s=args.minimum_duration,
    )
    print(json.dumps(result.as_dict(), indent=2, ensure_ascii=False))
    if args.output:
        write_validation_json(result, args.output)
    return 0 if result.passed else 3


def cmd_audit_params(args: argparse.Namespace) -> int:
    result = audit_file(args.input)
    print(json.dumps(result, indent=2, ensure_ascii=False))
    if args.output:
        write_audit_json(result, args.output)
    return 0 if result["ready_for_p7"] else 4


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="s500-companion",
        description="Safe USB MAVLink tools for S500/Pixhawk 6C",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    discover = sub.add_parser("discover", help="List persistent USB serial paths")
    discover.add_argument("--by-id-root", default="/dev/serial/by-id")
    discover.set_defaults(func=cmd_discover)

    validate = sub.add_parser("validate-log", help="Evaluate a P1 telemetry CSV")
    validate.add_argument("--input", required=True)
    validate.add_argument("--output")
    validate.add_argument("--expected-system-id", type=int, default=1)
    validate.add_argument("--minimum-duration", type=float, default=1740.0)
    validate.set_defaults(func=cmd_validate_log)

    audit = sub.add_parser("audit-params", help="Audit PX4 params for P7 yaw-only")
    audit.add_argument("--input", required=True)
    audit.add_argument("--output")
    audit.set_defaults(func=cmd_audit_params)

    mode_test = sub.add_parser(
        "hardware-mode-test",
        help="Disarmed-only real Pixhawk mode command/restore test",
    )
    mode_test.add_argument("--connection", default="auto")
    mode_test.add_argument("--expected-system-id", type=int, default=1)
    mode_test.add_argument("--heartbeat-timeout-s", type=float, default=5.0)
    mode_test.add_argument("--poll-s", type=float, default=0.05)
    mode_test.add_argument("--disarmed-observe-s", type=float, default=2.0)
    mode_test.add_argument("--timeout-s", type=float, default=5.0)
    mode_test.add_argument(
        "--target-mode", choices=sorted(SAFE_MODE_COMMANDS), default="POSITION"
    )
    mode_test.add_argument("--props-removed", default="")
    mode_test.add_argument("--confirm", default="")
    mode_test.add_argument("--output")
    mode_test.set_defaults(func=run_hardware_mode_test)

    for name, function in (("probe", cmd_probe), ("record", cmd_record)):
        item = sub.add_parser(name)
        item.add_argument("--vehicle-config", type=Path, default=_default_config())
        item.add_argument("--connection", help="Override auto/serial/UDP connection")
        item.add_argument(
            "--duration",
            type=float,
            default=10.0 if name == "probe" else 1800.0,
        )
        if name == "record":
            item.add_argument("--output")
            item.add_argument("--rate", type=float, default=5.0)
        item.set_defaults(func=function)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return int(args.func(args))
    except (MissingDependencyError, FileNotFoundError, PermissionError, RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
