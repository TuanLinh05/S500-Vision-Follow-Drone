"""Chay follow.app that (--enable-control) nham vao PX4 SITL, voi muc tieu
gia lap co dinh (chua co model OpenVINO). Dung video file gia lap, khong
can camera that (WSL khong co camera).

Chay:
    python -m tools.sitl_bench udp:127.0.0.1:14540
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from follow import app as app_mod  # noqa: E402

FW, FH = 640, 480
BOX = (400.0, 100.0, 460.0, 400.0)   # lech PHAI so voi tam khung


class _Tensor:
    def __init__(self, data):
        self.data = data


class _FakeReq:
    def __init__(self, imgsz, w, h):
        r = min(imgsz / h, imgsz / w)
        dw = (imgsz - int(round(w * r))) // 2
        dh = (imgsz - int(round(h * r))) // 2
        x1, y1, x2, y2 = BOX
        self._det = np.array(
            [[[x1 * r + dw, y1 * r + dh, x2 * r + dw, y2 * r + dh,
               0.99, 0.0]]], np.float32)

    def infer(self, _inp):
        return None

    def start_async(self, _inp):
        return None

    def wait(self):
        return None

    def get_tensor(self, _op):
        return _Tensor(self._det)


class _FakeCompiled:
    def __init__(self, imgsz, w, h):
        self._req = _FakeReq(imgsz, w, h)

    def create_infer_request(self):
        return self._req

    def output(self, _i=0):
        return "out0"


def make_video(path, w, h, n=200):
    vw = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"),
                         30.0, (w, h))
    for _ in range(n):
        f = np.full((h, w, 3), 90, np.uint8)
        cv2.rectangle(f, (int(BOX[0]), int(BOX[1])),
                      (int(BOX[2]), int(BOX[3])), (30, 60, 200), -1)
        vw.write(f)
    vw.release()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mavlink")
    ap.add_argument("--max-yaw-rate", type=float, default=15.0)
    ap.add_argument("--max-engage-s", type=float, default=8.0)
    ap.add_argument("--imgsz", type=int, default=416)
    ap.add_argument("--port-web", type=int, default=8090)
    ap.add_argument("--fence-radius", type=float, default=10.0)
    ap.add_argument("--fence-alt-max", type=float, default=30.0)
    ap.add_argument("--rc-chan", type=int, default=0,
                    help="Kenh RC dead-man. 0 = tat (mac dinh, vi SIH SITL "
                         "khong tu phat RC_CHANNELS). Dat >0 khi co bom RC "
                         "bang tools.sitl_rc_inject de test dead-man that.")
    ap.add_argument("--rc-min", type=int, default=1500)
    args = ap.parse_args()

    video_path = Path("/tmp/sitl_bench_video.avi")
    make_video(video_path, FW, FH)

    imgsz = args.imgsz
    app_mod.load_model = lambda *a, **k: (
        _FakeCompiled(imgsz, FW, FH), "GIA_LAP.xml")

    print("=" * 72)
    print("  SITL BENCH TEST - MUC TIEU GIA LAP - DIEU KHIEN THAT")
    print(f"  mavlink={args.mavlink}  max-yaw-rate={args.max_yaw_rate}")
    print("=" * 72)

    sys.argv = [
        "app",
        "--source", str(video_path),
        "--width", str(FW), "--height", str(FH),
        "--imgsz", str(imgsz),
        "--mavlink", args.mavlink,
        "--enable-control",
        # SIH SITL khong TU phat RC_CHANNELS. Khi --rc-chan 0 thi dung co
        # acknowledge ro rang cho moi truong mo phong, de code bay that van
        # bat buoc dead-man RC. Khi co bom RC (tools.sitl_rc_inject) thi
        # dat --rc-chan > 0 de test dead-man THAT su, khong bo qua lop nao.
        "--rc-chan", str(args.rc_chan), "--rc-min", str(args.rc_min),
        *([] if args.rc_chan else ["--allow-no-rc-deadman"]),
        "--allow-no-battery-check",
        "--max-yaw-rate", str(args.max_yaw_rate),
        "--max-engage-s", str(args.max_engage_s),
        "--ramp", "0.5", "--prestream", "0.5",
        "--fence-radius", str(args.fence_radius),
        "--fence-alt-max", str(args.fence_alt_max),
        "--fence-alt-min", "0",
        "--fence-max-speed", "3.0",
        "--on-abort", "loiter",
        "--port", str(args.port_web),
        "--jpeg-hz", "4",
        "--log-csv", "/tmp/sitl_bench.csv",
    ]
    app_mod.main()


if __name__ == "__main__":
    main()
