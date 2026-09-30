"""Bench test co dieu khien that (--enable-control) voi MUC TIEU GIA LAP.

Dung khi CHUA co model OpenVINO nhung muon kiem chung chuoi PX4 that:
ENGAGE -> OFFBOARD -> lenh yaw -> disengage/LOITER, tren Pixhawk that.

"Mo hinh" o day la mot bo phat hien GIA: luon tra ve DUNG MOT box co dinh,
lech nhe sang PHAI so voi tam khung. Khong co nhan dien nguoi that nao xay
ra. Muc dich CHI la kiem tra lop dieu khien/an toan/MAVLink, KHONG kiem tra
vision.

AN TOAN — DOC TRUOC KHI CHAY:
  - CANH QUAT PHAI DA THAO HET.
  - Khung phai duoc co dinh / dat vung, khong roi khoi ban.
  - Tay va vat la phai TRANH XA dong co - dong co SE QUAY khi arm (binh
    thuong, vi khong co canh quat nen khong nguy hiem, nhung van quay that).
  - Nguoi dung PHAI tu arm bang RC (script KHONG bao gio tu arm).
  - --max-engage-s va --max-yaw-rate duoc dat rat thap cho lan dau.

Chay:
    python -m tools.bench_synthetic_target COM30 --rc-chan 8
"""

import argparse
import sys
import threading
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np  # noqa: E402

from follow import app as app_mod  # noqa: E402


class _Tensor:
    def __init__(self, data):
        self.data = data


class _FakeReq:
    """Luon tra ve DUNG MOT nguoi gia, lech PHAI so voi tam khung."""

    def __init__(self, imgsz, frame_w, frame_h, offset_ratio=0.18):
        r = min(imgsz / frame_h, imgsz / frame_w)
        dw = (imgsz - int(round(frame_w * r))) // 2
        dh = (imgsz - int(round(frame_h * r))) // 2
        cx = frame_w * (0.5 + offset_ratio)
        x1, y1 = cx - frame_w * 0.06, frame_h * 0.20
        x2, y2 = cx + frame_w * 0.06, frame_h * 0.75
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


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("port")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--rc-chan", type=int, default=0)
    ap.add_argument("--rc-min", type=int, default=1500)
    ap.add_argument("--batt-min", type=float, default=14.0)
    ap.add_argument("--max-yaw-rate", type=float, default=8.0)
    ap.add_argument("--max-engage-s", type=float, default=5.0)
    ap.add_argument("--imgsz", type=int, default=416)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--port-web", type=int, default=8080)
    args = ap.parse_args()

    print("=" * 72)
    print("  BENCH TEST - MUC TIEU GIA LAP - DIEU KHIEN THAT")
    print("  CANH QUAT PHAI DA THAO. DONG CO SE QUAY KHI ARM.")
    print("  Box 'nguoi' tren video la GIA - khong phai AI nhan dien that.")
    print(f"  max-yaw-rate={args.max_yaw_rate} do/s  "
          f"max-engage-s={args.max_engage_s}s  rc-chan={args.rc_chan or 'TAT'}")
    print("=" * 72)

    imgsz = args.imgsz
    app_mod.load_model = lambda *a, **k: (
        _FakeCompiled(imgsz, args.width, args.height), "GIA_LAP.xml")

    sys.argv = [
        "app",
        "--camera", str(args.camera),
        "--width", str(args.width), "--height", str(args.height),
        "--imgsz", str(imgsz),
        "--mavlink", args.port, "--baud", str(args.baud),
        "--enable-control",
        "--rc-chan", str(args.rc_chan), "--rc-min", str(args.rc_min),
        "--batt-min", str(args.batt_min),
        "--max-yaw-rate", str(args.max_yaw_rate),
        "--max-engage-s", str(args.max_engage_s),
        "--ramp", "0.5", "--prestream", "0.5",
        "--loop-watchdog", "1.0",
        "--no-fence",           # bench trong nha, khong GPS - xem ghi chu
        "--on-abort", "loiter",
        "--port", str(args.port_web),
        "--jpeg-hz", "8",
        "--log-csv", "bench_synthetic.csv",
    ]
    app_mod.main()


if __name__ == "__main__":
    main()
