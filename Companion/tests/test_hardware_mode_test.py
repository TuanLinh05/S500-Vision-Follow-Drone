import unittest

from s500_companion.hardware_mode_test import (
    HardwareModeTestError,
    MAV_CMD_DO_SET_MODE,
    _send_safe_mode,
    normalize_safe_mode,
)
from s500_companion.models import TelemetrySnapshot


class DummyMav:
    def __init__(self) -> None:
        self.calls = []

    def command_long_send(self, *args: object) -> None:
        self.calls.append(args)


class DummyConnection:
    target_system = 1
    target_component = 1

    def __init__(self) -> None:
        self.mav = DummyMav()


class DummyReader:
    def __init__(self, armed: bool = False) -> None:
        self.connection = DummyConnection()
        self._snapshot = TelemetrySnapshot(connected=True, armed=armed)

    def snapshot(self) -> TelemetrySnapshot:
        return self._snapshot


class HardwareModeTestTests(unittest.TestCase):
    def test_normalizes_safe_mode_aliases(self) -> None:
        self.assertEqual(normalize_safe_mode("posctl"), "POSITION")
        self.assertEqual(normalize_safe_mode("AUTO.LOITER"), "HOLD")
        self.assertEqual(normalize_safe_mode("offboard"), "OFFBOARD")

    def test_only_allowlisted_disarmed_mode_is_sent(self) -> None:
        reader = DummyReader()
        _send_safe_mode(reader, "POSITION")
        call = reader.connection.mav.calls[0]
        self.assertEqual(call[2], MAV_CMD_DO_SET_MODE)
        self.assertEqual(call[5], 3.0)
        self.assertEqual(call[6], 0.0)

    def test_armed_vehicle_blocks_before_sending(self) -> None:
        reader = DummyReader(armed=True)
        with self.assertRaisesRegex(HardwareModeTestError, "armed"):
            _send_safe_mode(reader, "POSITION")
        self.assertEqual(reader.connection.mav.calls, [])

    def test_offboard_is_not_allowlisted(self) -> None:
        reader = DummyReader()
        with self.assertRaisesRegex(HardwareModeTestError, "allow-listed"):
            _send_safe_mode(reader, "OFFBOARD")
        self.assertEqual(reader.connection.mav.calls, [])


if __name__ == "__main__":
    unittest.main()
