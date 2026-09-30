from __future__ import annotations

from dataclasses import dataclass

from .models import OperatorInput, TelemetrySnapshot


@dataclass(slots=True)
class RcConsentDecoder:
    """Hysteretic AI-enable decoder for one RC channel.

    Channel numbering is one-based. Channel 0 means unconfigured and always
    returns AI disabled.
    """

    channel: int = 0
    low_us: int = 1300
    high_us: int = 1700
    _enabled: bool = False

    def __post_init__(self) -> None:
        if self.channel < 0 or self.channel > 18:
            raise ValueError("RC AI-enable channel must be 0..18")
        if not 800 <= self.low_us < self.high_us <= 2200:
            raise ValueError("Invalid RC hysteresis thresholds")

    def decode(self, telemetry: TelemetrySnapshot) -> OperatorInput:
        if self.channel == 0 or not telemetry.rc_available:
            self._enabled = False
            return OperatorInput(ai_enable=False)
        index = self.channel - 1
        if index >= len(telemetry.rc_channels_us):
            self._enabled = False
            return OperatorInput(ai_enable=False)
        pulse = telemetry.rc_channels_us[index]
        if pulse <= self.low_us:
            self._enabled = False
        elif pulse >= self.high_us:
            self._enabled = True
        return OperatorInput(ai_enable=self._enabled)

