from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from .config import load_safety_config, load_vehicle_config
from .flight_guard import FlightGuard
from .models import GuidanceProposal, VisionStatus
from .operator_consent import RcConsentDecoder
from .telemetry import MavlinkTelemetryReader
from .vision_socket import LocalVisionReceiver


def run(args: argparse.Namespace) -> int:
    vehicle = load_vehicle_config(args.vehicle_config)
    safety = load_safety_config(args.safety_config)
    reader = MavlinkTelemetryReader(
        connection=args.connection or vehicle.connection,
        expected_system_id=vehicle.expected_system_id,
        heartbeat_timeout_s=vehicle.heartbeat_timeout_s,
        poll_s=vehicle.read_poll_s,
    )
    consent = RcConsentDecoder(
        channel=vehicle.ai_enable_rc_channel,
        low_us=vehicle.ai_enable_low_us,
        high_us=vehicle.ai_enable_high_us,
    )
    guard = FlightGuard(safety, transport=None)
    vision = VisionStatus()
    guidance = GuidanceProposal()
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    period = 1.0 / safety.command_hz
    started = time.monotonic()
    with reader, LocalVisionReceiver(
        vehicle.vision_udp_host, vehicle.vision_udp_port
    ) as receiver, output_path.open("w", encoding="utf-8") as handle:
        print(
            f"Shadow mode: USB={reader.connection_spec}, vision="
            f"{receiver.host}:{receiver.port}, output={output_path}"
        )
        while time.monotonic() - started < args.duration:
            loop_started = time.monotonic()
            telemetry = reader.poll()
            packet = receiver.receive_latest()
            if packet is not None:
                vision, guidance = packet
            now = time.monotonic()
            output = guard.tick(now, telemetry, vision, guidance, consent.decode(telemetry))
            handle.write(
                json.dumps(
                    {
                        "telemetry": telemetry.as_dict(),
                        "supervisor": output.as_dict(),
                        "dry_run": True,
                    },
                    ensure_ascii=False,
                )
                + "\n"
            )
            handle.flush()
            sleep_s = period - (time.monotonic() - loop_started)
            if sleep_s > 0:
                time.sleep(sleep_s)
    print("Shadow complete; no MAVLink control transport existed.")
    return 0


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description="Read-only S500 shadow-mode runner")
    parser.add_argument("--vehicle-config", type=Path, default=root / "config/vehicle.json")
    parser.add_argument("--safety-config", type=Path, default=root / "config/safety.json")
    parser.add_argument("--connection")
    parser.add_argument("--duration", type=float, default=600.0)
    parser.add_argument("--output", default="logs/shadow.jsonl")
    try:
        return run(parser.parse_args())
    except (RuntimeError, TimeoutError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

