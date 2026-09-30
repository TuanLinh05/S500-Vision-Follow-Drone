import unittest
from pathlib import Path

from s500_companion.serial_discovery import discover_pixhawk, list_serial_candidates


class SerialDiscoveryTests(unittest.TestCase):
    def setUp(self) -> None:
        self.root = Path(__file__).with_name("_serial_fixture")
        self.root.mkdir(exist_ok=True)
        for item in self.root.iterdir():
            if item.is_file():
                item.unlink()

    def tearDown(self) -> None:
        for item in self.root.iterdir():
            if item.is_file():
                item.unlink()
        self.root.rmdir()

    def test_prefers_pixhawk_named_device(self) -> None:
        (self.root / "usb-generic-modem").touch()
        pixhawk = self.root / "usb-Holybro_Pixhawk6C_PX4"
        pixhawk.touch()
        candidates = list_serial_candidates(str(self.root))
        self.assertEqual(candidates[0].path, str(pixhawk))
        self.assertEqual(discover_pixhawk(str(self.root)), str(pixhawk))

    def test_empty_directory_is_rejected(self) -> None:
        with self.assertRaises(FileNotFoundError):
            discover_pixhawk(str(self.root))


if __name__ == "__main__":
    unittest.main()
