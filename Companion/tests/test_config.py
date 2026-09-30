import json
import unittest
from pathlib import Path

from s500_companion.config import SafetyConfig, load_safety_config


class ConfigTests(unittest.TestCase):
    def test_default_safety_config_is_valid(self) -> None:
        SafetyConfig().validate()

    def test_unknown_config_key_is_rejected(self) -> None:
        path = Path(__file__).with_name("_runtime_safety.json")
        try:
            path.write_text(json.dumps({"not_a_parameter": 1}), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "Unknown keys"):
                load_safety_config(path)
        finally:
            path.unlink(missing_ok=True)

    def test_unsafe_low_command_rate_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "command_hz"):
            SafetyConfig(command_hz=2).validate()


if __name__ == "__main__":
    unittest.main()
