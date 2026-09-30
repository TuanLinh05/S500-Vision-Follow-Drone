"""Read a filtered PX4 parameter list without changing any parameter."""

from __future__ import annotations

import argparse
import os
import re
import time

# Ep MAVLink2 phong khi script chay truc tiep (khong qua `python -m`). Xem
# follow/__init__.py.
os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil

from .px4_param import (
    _close, _decode_value, _param_name, _wait_autopilot_heartbeat,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port")
    parser.add_argument("--match", default="ARM|RC_MAP|SAFETY|MAN_.*GEST")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--seconds", type=float, default=35.0)
    args = parser.parse_args()

    pattern = re.compile(args.match, re.IGNORECASE)
    link = mavutil.mavlink_connection(
        args.port, baud=args.baud, source_system=245,
        source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)
    try:
        heartbeat = _wait_autopilot_heartbeat(link, 8.0)
        if heartbeat is None:
            raise TimeoutError("no PX4 heartbeat")
        link.mav.param_request_list_send(
            link.target_system, link.target_component)

        deadline = time.monotonic() + args.seconds
        last_new = time.monotonic()
        seen = set()
        expected = None
        selected = {}
        while time.monotonic() < deadline:
            msg = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if msg is None:
                if seen and time.monotonic() - last_new > 3.0:
                    break
                continue
            name = _param_name(msg.param_id)
            expected = int(msg.param_count)
            if name not in seen:
                seen.add(name)
                last_new = time.monotonic()
            if pattern.search(name):
                selected[name] = (
                    _decode_value(msg.param_value, int(msg.param_type)),
                    int(msg.param_type),
                )
            if expected and len(seen) >= expected:
                break

        for name in sorted(selected):
            value, param_type = selected[name]
            print(f"{name}={value:g} type={param_type}")
        print(f"SEEN {len(seen)}/{expected if expected is not None else '?'}")
        return 0 if seen else 2
    finally:
        _close(link)


if __name__ == "__main__":
    raise SystemExit(main())
