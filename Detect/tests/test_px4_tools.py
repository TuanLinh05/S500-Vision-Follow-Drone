"""Regression tests for the hardware-bench PX4 maintenance utilities."""

from pymavlink import mavutil
import pytest

from tools.px4_param import (
    _decode_value, _encode_value, _wait_autopilot_heartbeat,
)


@pytest.mark.parametrize("param_type,value", [
    (mavutil.mavlink.MAV_PARAM_TYPE_UINT8, 200),
    (mavutil.mavlink.MAV_PARAM_TYPE_INT8, -100),
    (mavutil.mavlink.MAV_PARAM_TYPE_UINT16, 50000),
    (mavutil.mavlink.MAV_PARAM_TYPE_INT16, -22000),
    (mavutil.mavlink.MAV_PARAM_TYPE_UINT32, 22027),
    (mavutil.mavlink.MAV_PARAM_TYPE_INT32, 22027),
    (mavutil.mavlink.MAV_PARAM_TYPE_INT32, 1),
    (mavutil.mavlink.MAV_PARAM_TYPE_REAL32, 14.25),
])
def test_param_bytewise_roundtrip(param_type, value):
    encoded = _encode_value(value, param_type)
    assert _decode_value(encoded, param_type) == pytest.approx(value)


class _Heartbeat:
    def __init__(self, vehicle_type, autopilot, system, component):
        self.type = vehicle_type
        self.autopilot = autopilot
        self._system = system
        self._component = component

    def get_srcSystem(self):
        return self._system

    def get_srcComponent(self):
        return self._component


class _Link:
    def __init__(self, messages):
        self.messages = list(messages)
        self.target_system = 0
        self.target_component = 0

    def recv_match(self, **_kwargs):
        return self.messages.pop(0) if self.messages else None


def test_tool_bo_qua_heartbeat_qgroundcontrol():
    gcs = _Heartbeat(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        255, 190,
    )
    autopilot = _Heartbeat(
        mavutil.mavlink.MAV_TYPE_QUADROTOR,
        mavutil.mavlink.MAV_AUTOPILOT_PX4,
        1, 1,
    )
    link = _Link([gcs, autopilot])

    assert _wait_autopilot_heartbeat(link, 0.5) is autopilot
    assert (link.target_system, link.target_component) == (1, 1)
