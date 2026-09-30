import json
import unittest

from s500_companion.vision_protocol import VisionProtocolError, parse_vision_packet


def packet() -> dict[str, object]:
    return {
        "schema": "s500.vision.v1",
        "sequence": 1,
        "frame_ts": 100.0,
        "track_ts": 100.0,
        "guidance_ts": 100.0,
        "track_id": 7,
        "track_state": "TRACKED",
        "confidence": 0.9,
        "camera_ok": True,
        "detector_ok": True,
        "tracker_ok": True,
        "yaw_rate_deg_s": 5.0,
    }


class VisionProtocolTests(unittest.TestCase):
    def test_valid_packet(self) -> None:
        vision, guidance = parse_vision_packet(json.dumps(packet()))
        self.assertEqual(vision.selected_track_id, 7)
        self.assertEqual(guidance.track_id, 7)

    def test_nan_is_rejected(self) -> None:
        value = packet()
        value["yaw_rate_deg_s"] = float("nan")
        with self.assertRaisesRegex(VisionProtocolError, "NaN"):
            parse_vision_packet(json.dumps(value))

    def test_wrong_schema_is_rejected(self) -> None:
        value = packet()
        value["schema"] = "unknown"
        with self.assertRaisesRegex(VisionProtocolError, "schema"):
            parse_vision_packet(json.dumps(value))


if __name__ == "__main__":
    unittest.main()

