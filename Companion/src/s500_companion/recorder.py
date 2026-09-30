from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import TextIO

from .models import TelemetrySnapshot


_CSV_FIELDS = (
    "monotonic_ts",
    "last_rx_ts",
    "last_heartbeat_ts",
    "connected",
    "system_id",
    "component_id",
    "armed",
    "flight_mode",
    "local_position_valid",
    "global_position_valid",
    "home_position_valid",
    "rc_available",
    "battery_warning",
    "failsafe",
    "geofence_warning",
    "x_m",
    "y_m",
    "z_m",
    "vx_m_s",
    "vy_m_s",
    "vz_m_s",
    "roll_rad",
    "pitch_rad",
    "yaw_rad",
    "yaw_rate_rad_s",
    "latitude_deg",
    "longitude_deg",
    "relative_alt_m",
    "battery_voltage_v",
    "battery_current_a",
    "battery_remaining_pct",
    "gps_fix_type",
    "satellites_visible",
    "rc_rssi",
    "last_statustext",
)


class TelemetryRecorder:
    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._handle: TextIO | None = None
        self._writer: csv.DictWriter[str] | None = None
        self._jsonl = self.path.suffix.lower() == ".jsonl"

    def __enter__(self) -> "TelemetryRecorder":
        self._handle = self.path.open("w", encoding="utf-8", newline="")
        if not self._jsonl:
            self._writer = csv.DictWriter(self._handle, fieldnames=_CSV_FIELDS)
            self._writer.writeheader()
        return self

    def write(self, snapshot: TelemetrySnapshot) -> None:
        if self._handle is None:
            raise RuntimeError("Recorder is not open")
        payload = snapshot.as_dict()
        if self._jsonl:
            self._handle.write(json.dumps(payload, ensure_ascii=False) + "\n")
        else:
            assert self._writer is not None
            self._writer.writerow({name: payload.get(name) for name in _CSV_FIELDS})
        self._handle.flush()

    def __exit__(self, *_: object) -> None:
        if self._handle is not None:
            self._handle.close()
        self._handle = None
        self._writer = None

