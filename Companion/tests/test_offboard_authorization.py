import unittest

from s500_companion.offboard import ControlAuthorizationError, MavlinkOffboardTransport


class DummyConnection:
    target_system = 1
    target_component = 1


class OffboardAuthorizationTests(unittest.TestCase):
    def test_two_key_authorization_is_required(self) -> None:
        with self.assertRaises(ControlAuthorizationError):
            MavlinkOffboardTransport(
                DummyConnection(),
                "udp:127.0.0.1:14540",
                config_control_enabled=False,
                cli_control_enabled=True,
                allow_serial_control=False,
            )

    def test_real_serial_is_locked_separately(self) -> None:
        with self.assertRaisesRegex(ControlAuthorizationError, "Real-hardware"):
            MavlinkOffboardTransport(
                DummyConnection(),
                "/dev/serial/by-id/pixhawk",
                config_control_enabled=True,
                cli_control_enabled=True,
                allow_serial_control=False,
            )


if __name__ == "__main__":
    unittest.main()

