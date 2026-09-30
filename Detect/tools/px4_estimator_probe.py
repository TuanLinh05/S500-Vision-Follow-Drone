"""Passively summarize PX4 local-position, optical-flow and range data."""

from __future__ import annotations

import argparse
import math
import os
import statistics
import time

# Ep MAVLink2 phong khi script chay truc tiep (khong qua `python -m`). Day
# la ly do chinh khien script nay tu truoc gio khong bao gio thay ODOMETRY:
# dialect MAVLink1 mac dinh khong co class cho id > 255. Xem follow/__init__.py.
os.environ.setdefault("MAVLINK20", "1")

from pymavlink import mavutil

from .px4_param import _close, _wait_autopilot_heartbeat


def _range(values):
    if not values:
        return "n/a"
    return f"{min(values):.3f}..{max(values):.3f}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--seconds", type=float, default=12.0)
    args = parser.parse_args()

    link = mavutil.mavlink_connection(
        args.port, baud=args.baud, source_system=245,
        source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)
    positions = []
    odometry = []
    flow_quality = []
    flow_distance = []
    ranges = []
    resets = set()
    estimator_flags = set()
    estimator_ratios = []
    counts = {}
    try:
        heartbeat = _wait_autopilot_heartbeat(link, 8.0)
        if heartbeat is None:
            raise TimeoutError("no PX4 heartbeat")
        armed = bool(
            heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        print(f"START armed={armed} custom_mode={heartbeat.custom_mode}")
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            msg = link.recv_match(blocking=True, timeout=0.25)
            if msg is None:
                continue
            kind = msg.get_type()
            counts[kind] = counts.get(kind, 0) + 1
            if kind == "LOCAL_POSITION_NED":
                positions.append((float(msg.x), float(msg.y), float(msg.z)))
            elif kind == "ODOMETRY":
                odometry.append((float(msg.x), float(msg.y), float(msg.z)))
                resets.add(int(getattr(msg, "reset_counter", 0)))
            elif kind == "OPTICAL_FLOW_RAD":
                flow_quality.append(int(msg.quality))
                distance = float(getattr(msg, "distance", math.nan))
                if math.isfinite(distance):
                    flow_distance.append(distance)
            elif kind == "DISTANCE_SENSOR":
                ranges.append(float(msg.current_distance) / 100.0)
            elif kind == "ESTIMATOR_STATUS":
                estimator_flags.add(int(msg.flags))
                estimator_ratios.append(max(
                    float(msg.vel_ratio), float(msg.pos_horiz_ratio),
                    float(msg.pos_vert_ratio), float(msg.hagl_ratio)))

        if positions:
            xs, ys, zs = zip(*positions)
            print(
                f"LPNED n={len(positions)} x={_range(xs)} y={_range(ys)} "
                f"z={_range(zs)} final={positions[-1]}")
        else:
            print("LPNED n=0")
        if odometry:
            xs, ys, zs = zip(*odometry)
            print(
                f"ODOMETRY n={len(odometry)} x={_range(xs)} "
                f"y={_range(ys)} z={_range(zs)} resets={sorted(resets)}")
        else:
            print("ODOMETRY n=0")
        if flow_quality:
            print(
                f"FLOW n={len(flow_quality)} quality="
                f"{min(flow_quality)}..{max(flow_quality)} "
                f"mean={statistics.fmean(flow_quality):.1f} "
                f"distance_m={_range(flow_distance)}")
        else:
            print("FLOW n=0")
        print(f"RANGE n={len(ranges)} distance_m={_range(ranges)}")
        print(
            f"ESTIMATOR flags={sorted(estimator_flags)} "
            f"max_test_ratio="
            f"{max(estimator_ratios) if estimator_ratios else 'n/a'}")
        print("COUNTS " + " ".join(
            f"{name}={counts.get(name, 0)}" for name in (
                "LOCAL_POSITION_NED", "ODOMETRY", "OPTICAL_FLOW_RAD",
                "DISTANCE_SENSOR", "ESTIMATOR_STATUS")))
        return 0
    finally:
        _close(link)


if __name__ == "__main__":
    raise SystemExit(main())
