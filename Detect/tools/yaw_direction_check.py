"""Kiem chung CHIEU xoay yaw ma KHONG can MAVLink, KHONG can arm, KHONG bay.

Vi sao can rieng cong cu nay: chay `follow.app` che do kho khong kiem tra duoc
chieu yaw, vi khi SafetyGate chan (chua bat --enable-control) thi
YawController CHU DONG dat bearing = 0 va yaw = 0 (thiet ke an toan dung).
Nen `lech`/`yaw` tren giao dien web luon bang 0 du da khoa muc tieu.

Cong cu nay goi thang YawController voi allow=True de tinh xem lenh yaw SE la
bao nhieu, nhung KHONG gui di dau ca - khong mo MAVLink, khong tao streamer.

Vi sao QUAN TRONG: neu chieu yaw bi nguoc (camera lap quay nguoc so voi mui
drone, hoac quen --invert-yaw), drone se xoay RA XA muc tieu, sai so cang
lon, thanh phan hoi duong -> mat kiem soat. Day la che do hong nguy hiem
nhat con lai chua duoc kiem chung truoc khi bay.

Quy uoc can thay:
    nguoi o NUA PHAI khung hinh -> lech DUONG -> yaw DUONG (xoay phai)
    nguoi o NUA TRAI khung hinh -> lech AM    -> yaw AM    (xoay trai)

Neu thay nguoc lai -> them co --invert-yaw cho follow.app.

Chay:
    python3 -m tools.yaw_direction_check --model <duong dan> --cam-rot 180
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np  # noqa: E402

from follow.camera import Camera  # noqa: E402
from follow.control import YawConfig, YawController  # noqa: E402
from follow.detect import letterbox, load_model, postprocess  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--model", required=True)
    ap.add_argument("--device", default="GPU")
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--cam-rot", type=int, default=0, choices=[0, 90, 180, 270])
    ap.add_argument("--imgsz", type=int, default=416)
    ap.add_argument("--conf", type=float, default=0.4)
    ap.add_argument("--hfov", type=float, default=90.0)
    ap.add_argument("--gain", type=float, default=0.6)
    ap.add_argument("--deadzone", type=float, default=4.0)
    ap.add_argument("--max-yaw-rate", type=float, default=15.0)
    ap.add_argument("--invert-yaw", action="store_true")
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--hz", type=float, default=2.0, help="Tan so in ra")
    args = ap.parse_args()

    print("=" * 72)
    print("  KIEM CHUNG CHIEU XOAY YAW - KHONG gui lenh nao ra ngoai")
    print("  Dung o NUA PHAI khung hinh -> phai thay lech DUONG, yaw DUONG")
    print("=" * 72)

    compiled, xml = load_model(args.model, args.device, args.imgsz)
    req = compiled.create_infer_request()
    op = compiled.output(0)
    for _ in range(5):
        req.infer({0: np.zeros((1, args.imgsz, args.imgsz, 3), np.uint8)})
    print(f"[+] Model {xml}")

    cam = Camera(args.camera, args.width, args.height, "MJPG",
                 rotate=args.cam_rot)
    print(f"[+] Camera {cam.width}x{cam.height} (xoay {args.cam_rot} do)\n")

    ctl = YawController(
        YawConfig(hfov_deg=args.hfov, gain=args.gain,
                  deadzone_deg=args.deadzone, max_rate=args.max_yaw_rate,
                  invert=args.invert_yaw),
        cam.width)

    t_end = time.monotonic() + args.seconds
    t_prev = time.monotonic()
    next_print = 0.0
    last_seq = -1
    n_seen = 0
    while time.monotonic() < t_end:
        frame, seq, _ = cam.read()
        if frame is None or seq == last_seq:
            time.sleep(0.005)
            continue
        last_seq = seq

        lb, r, dw, dh = letterbox(frame, args.imgsz)
        req.start_async({0: lb[None]})
        req.wait()
        det = req.get_tensor(op).data[0]
        H, W = frame.shape[:2]
        boxes, scores = postprocess(det, r, dw, dh, H, W, args.conf)

        now = time.monotonic()
        dt, t_prev = now - t_prev, now

        box = None
        if len(boxes):
            # chon box LON NHAT (nguoi gan camera nhat)
            areas = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
            box = boxes[int(np.argmax(areas))]

        # allow=True: tinh xem lenh SE la bao nhieu (khong gui di dau)
        yaw = ctl.update(box, dt, allow=True)

        if now >= next_print:
            next_print = now + 1.0 / max(args.hz, 0.1)
            if box is None:
                print("  khong thay nguoi nao")
                continue
            n_seen += 1
            cx = (box[0] + box[2]) / 2.0
            frac = cx / cam.width
            if frac < 0.45:
                phia = "TRAI "
            elif frac > 0.55:
                phia = "PHAI "
            else:
                phia = "GIUA "
            dung = ("?" if phia == "GIUA " else
                    ("OK " if (ctl.bearing > 0) == (phia == "PHAI ")
                     else "SAI"))
            print(f"  nguoi o {phia}(x={frac*100:4.0f}% khung) | "
                  f"lech={ctl.bearing:+6.1f} do | yaw={yaw:+6.1f} do/s | "
                  f"chieu: {dung}")

    cam.release()
    print(f"\n[+] Xong. Da danh gia {n_seen} lan.")
    print("    Neu cot 'chieu' bao SAI -> phai them --invert-yaw cho follow.app")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
