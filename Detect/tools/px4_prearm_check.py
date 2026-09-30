"""Request PX4 pre-arm checks and print the resulting MAVLink diagnostics.

This utility never sends an arm command, mode command, or setpoint.
"""

from __future__ import annotations

import argparse
import os
import time

# Ep MAVLink2 phong khi script chay truc tiep (khong qua `python -m`). Xem
# follow/__init__.py.
os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil

from .px4_param import _close, _wait_autopilot_heartbeat


# Older pymavlink dialects bundled by some SBC distributions omit the name,
# while PX4/MAVLink define MAV_CMD_RUN_PREARM_CHECK as command 401.
MAV_CMD_RUN_PREARM_CHECK = getattr(
    mavutil.mavlink, "MAV_CMD_RUN_PREARM_CHECK", 401)


def _text(msg) -> str:
    value = msg.text
    if isinstance(value, bytes):
        value = value.decode("utf-8", errors="replace")
    return str(value).rstrip("\x00")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--seconds", type=float, default=8.0)
    args = parser.parse_args()

    link = mavutil.mavlink_connection(
        args.port, baud=args.baud, source_system=245,
        source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)
    try:
        heartbeat = _wait_autopilot_heartbeat(link, 8.0)
        if heartbeat is None:
            raise TimeoutError("no PX4 heartbeat")
        armed = bool(
            heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        print(f"BEFORE armed={armed} custom_mode={heartbeat.custom_mode}")

        link.mav.command_long_send(
            link.target_system,
            link.target_component,
            MAV_CMD_RUN_PREARM_CHECK,
            0,
            0, 0, 0, 0, 0, 0, 0,
        )
        print("REQUESTED MAV_CMD_RUN_PREARM_CHECK (does not arm)")

        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            msg = link.recv_match(blocking=True, timeout=0.25)
            if msg is None:
                continue
            kind = msg.get_type()
            if kind == "STATUSTEXT":
                print(f"STATUSTEXT severity={msg.severity}: {_text(msg)}")
            elif (kind == "COMMAND_ACK" and
                  msg.command == MAV_CMD_RUN_PREARM_CHECK):
                print(f"COMMAND_ACK result={msg.result}")
            elif kind == "HEARTBEAT" and msg.get_srcSystem() == link.target_system:
                armed = bool(
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        print(f"AFTER armed={armed}")
        return 0
    finally:
        _close(link)


if __name__ == "__main__":
    raise SystemExit(main())
