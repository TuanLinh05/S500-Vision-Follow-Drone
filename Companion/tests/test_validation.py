import csv
import unittest
from pathlib import Path

from s500_companion.validation import validate_p1_csv


class ValidationTests(unittest.TestCase):
    def test_short_but_otherwise_clean_log_reports_duration_only(self) -> None:
        path = Path(__file__).with_name("_runtime_validation.csv")
        fieldnames = (
            "monotonic_ts",
            "last_rx_ts",
            "last_heartbeat_ts",
            "connected",
            "system_id",
            "local_position_valid",
            "global_position_valid",
            "rc_available",
            "battery_remaining_pct",
        )
        try:
            with path.open("w", encoding="utf-8", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=fieldnames)
                writer.writeheader()
                for now in (100.0, 100.2, 100.4):
                    writer.writerow(
                        {
                            "monotonic_ts": now,
                            "last_rx_ts": now,
                            "last_heartbeat_ts": 100.0,
                            "connected": True,
                            "system_id": 1,
                            "local_position_valid": True,
                            "global_position_valid": True,
                            "rc_available": True,
                            "battery_remaining_pct": 80,
                        }
                    )
            result = validate_p1_csv(path)
            self.assertFalse(result.passed)
            self.assertEqual(result.reasons, ("duration_below_29_minutes",))
        finally:
            path.unlink(missing_ok=True)


if __name__ == "__main__":
    unittest.main()

