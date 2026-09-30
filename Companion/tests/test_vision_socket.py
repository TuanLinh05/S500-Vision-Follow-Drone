import json
import socket
import unittest

from s500_companion.vision_socket import LocalVisionReceiver


class VisionSocketTests(unittest.TestCase):
    def test_non_loopback_bind_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "loopback"):
            LocalVisionReceiver("0.0.0.0", 5800)

    def test_latest_loopback_packet_is_received(self) -> None:
        packet = {
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
        with LocalVisionReceiver("127.0.0.1", 0) as receiver:
            sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                sender.sendto(json.dumps(packet).encode(), (receiver.host, receiver.port))
                result = receiver.receive_latest()
            finally:
                sender.close()
        self.assertIsNotNone(result)
        assert result is not None
        self.assertEqual(result[0].selected_track_id, 7)


if __name__ == "__main__":
    unittest.main()

