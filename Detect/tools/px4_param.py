"""Read or safely change one PX4 parameter over a direct MAVLink link.

The ``set`` operation reads the current value first, can require an expected
old value, writes the parameter, and waits for PX4 to echo the new value.
This is intended for controlled bench maintenance, not flight-time tuning.

Examples::

    python -m tools.px4_param /dev/ttyACM0 get COM_ARM_WO_GPS
    python -m tools.px4_param /dev/ttyACM0 set COM_ARM_WO_GPS 1 --expect-old 0
"""

from __future__ import annotations

import argparse
import math
import os
import struct
import time

# Ep MAVLink2 truoc khi import mavutil, phong khi script nay chay truc tiep
# (khong qua `python -m`, tools/__init__.py se khong duoc nap). Xem
# follow/__init__.py de biet ly do (ODOMETRY/PARAM id > 255).
os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil


def _param_name(value) -> str:
    if isinstance(value, bytes):
        value = value.decode("ascii", errors="replace")
    return str(value).rstrip("\x00")


def _wait_autopilot_heartbeat(link, timeout: float):
    """Ignore routed GCS/onboard heartbeats and target the real autopilot."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = link.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if msg is None:
            continue
        if getattr(msg, "type", None) == mavutil.mavlink.MAV_TYPE_GCS:
            continue
        autopilot = getattr(
            msg, "autopilot", mavutil.mavlink.MAV_AUTOPILOT_INVALID)
        if autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            continue
        link.target_system = msg.get_srcSystem()
        link.target_component = msg.get_srcComponent()
        return msg
    return None


def _decode_value(raw_value: float, param_type: int):
    """Decode MAVLink's byte-wise integer representation used by PX4."""
    packed = struct.pack(">f", float(raw_value))
    formats = {
        mavutil.mavlink.MAV_PARAM_TYPE_UINT8: ">xxxB",
        mavutil.mavlink.MAV_PARAM_TYPE_INT8: ">xxxb",
        mavutil.mavlink.MAV_PARAM_TYPE_UINT16: ">xxH",
        mavutil.mavlink.MAV_PARAM_TYPE_INT16: ">xxh",
        mavutil.mavlink.MAV_PARAM_TYPE_UINT32: ">I",
        mavutil.mavlink.MAV_PARAM_TYPE_INT32: ">i",
    }
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_REAL32:
        return float(raw_value)
    if param_type not in formats:
        raise ValueError(f"unsupported MAV_PARAM_TYPE {param_type}")
    return struct.unpack(formats[param_type], packed)[0]


def _encode_value(value: float, param_type: int) -> float:
    """Encode a typed value into the PARAM_SET float field byte-for-byte."""
    formats = {
        mavutil.mavlink.MAV_PARAM_TYPE_UINT8: ">xxxB",
        mavutil.mavlink.MAV_PARAM_TYPE_INT8: ">xxxb",
        mavutil.mavlink.MAV_PARAM_TYPE_UINT16: ">xxH",
        mavutil.mavlink.MAV_PARAM_TYPE_INT16: ">xxh",
        mavutil.mavlink.MAV_PARAM_TYPE_UINT32: ">I",
        mavutil.mavlink.MAV_PARAM_TYPE_INT32: ">i",
    }
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_REAL32:
        return float(value)
    if param_type not in formats:
        raise ValueError(f"unsupported MAV_PARAM_TYPE {param_type}")
    packed = struct.pack(formats[param_type], int(value))
    return struct.unpack(">f", packed)[0]


def _read_param(link, target_system: int, target_component: int,
                name: str, timeout: float):
    deadline = time.monotonic() + timeout
    encoded_name = name.encode("ascii")
    next_request = 0.0
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_request:
            link.mav.param_request_read_send(
                target_system, target_component, encoded_name, -1)
            next_request = now + 0.5
        msg = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.2)
        if msg is not None and _param_name(msg.param_id) == name:
            return msg
    raise TimeoutError(f"PX4 did not return parameter {name!r}")


def _close(link) -> None:
    port = getattr(link, "port", None)
    if port is not None:
        port.close()
    elif hasattr(link, "close"):
        link.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port")
    parser.add_argument("action", choices=("get", "set"))
    parser.add_argument("name")
    parser.add_argument("value", type=float, nargs="?")
    parser.add_argument("--expect-old", type=float)
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--timeout", type=float, default=8.0)
    args = parser.parse_args()

    if args.action == "set" and args.value is None:
        parser.error("set requires VALUE")
    if args.action == "get" and args.value is not None:
        parser.error("get does not accept VALUE")

    link = mavutil.mavlink_connection(
        args.port, baud=args.baud, source_system=245,
        source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)
    try:
        heartbeat = _wait_autopilot_heartbeat(link, args.timeout)
        if heartbeat is None:
            raise TimeoutError("no PX4 heartbeat")
        target_system = link.target_system
        target_component = link.target_component

        current = _read_param(
            link, target_system, target_component, args.name, args.timeout)
        param_type = int(current.param_type)
        old_value = _decode_value(current.param_value, param_type)
        print(f"{args.name}={old_value:g} type={param_type}")

        if args.action == "get":
            return 0
        if args.expect_old is not None and not math.isclose(
                old_value, args.expect_old, rel_tol=0.0, abs_tol=1e-5):
            raise RuntimeError(
                f"refusing write: expected old value {args.expect_old:g}, "
                f"got {old_value:g}")

        encoded_name = args.name.encode("ascii")
        encoded_value = _encode_value(args.value, param_type)
        for _ in range(3):
            link.mav.param_set_send(
                target_system, target_component, encoded_name,
                encoded_value, param_type)
            echoed = _read_param(
                link, target_system, target_component, args.name, 2.0)
            actual = _decode_value(echoed.param_value, int(echoed.param_type))
            if math.isclose(actual, args.value, rel_tol=0.0, abs_tol=1e-5):
                print(f"VERIFIED {args.name}={actual:g}")
                return 0
        raise RuntimeError(
            f"PX4 did not verify {args.name}={args.value:g}; "
            f"last value was {actual:g}")
    finally:
        _close(link)


if __name__ == "__main__":
    raise SystemExit(main())
