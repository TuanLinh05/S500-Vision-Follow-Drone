import unittest

from s500_companion.models import TelemetrySnapshot
from s500_companion.operator_consent import RcConsentDecoder


class OperatorConsentTests(unittest.TestCase):
    def test_unconfigured_channel_is_always_off(self) -> None:
        decoder = RcConsentDecoder(channel=0)
        telemetry = TelemetrySnapshot(rc_available=True, rc_channels_us=(2000,))
        self.assertFalse(decoder.decode(telemetry).ai_enable)

    def test_hysteresis_and_rc_loss(self) -> None:
        decoder = RcConsentDecoder(channel=3)
        telemetry = TelemetrySnapshot(
            rc_available=True,
            rc_channels_us=(1500, 1500, 1800),
        )
        self.assertTrue(decoder.decode(telemetry).ai_enable)
        telemetry.rc_channels_us = (1500, 1500, 1500)
        self.assertTrue(decoder.decode(telemetry).ai_enable)
        telemetry.rc_channels_us = (1500, 1500, 1200)
        self.assertFalse(decoder.decode(telemetry).ai_enable)
        telemetry.rc_channels_us = (1500, 1500, 1800)
        telemetry.rc_available = False
        self.assertFalse(decoder.decode(telemetry).ai_enable)


if __name__ == "__main__":
    unittest.main()

