import csv
import unittest
from pathlib import Path

from s500_companion.models import TelemetrySnapshot
from s500_companion.recorder import TelemetryRecorder


class RecorderTests(unittest.TestCase):
    def test_csv_is_written(self) -> None:
        path = Path(__file__).with_name("_runtime_telemetry.csv")
        try:
            with TelemetryRecorder(path) as recorder:
                recorder.write(
                    TelemetrySnapshot(
                        monotonic_ts=10.0,
                        connected=True,
                        system_id=1,
                        flight_mode="POSITION",
                    )
                )
            with path.open(encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["system_id"], "1")
            self.assertEqual(rows[0]["flight_mode"], "POSITION")
        finally:
            path.unlink(missing_ok=True)


if __name__ == "__main__":
    unittest.main()
