"""Kiem tra camera that qua dung class follow.camera.Camera (khong phai
code rieng le) - do FPS thuc, xac nhan do phan giai/fourcc, luu 1 anh chup
de xem lai bang mat.

Chay tren may co camera that (vd UP7000):
    python -m tools.camera_probe
    python -m tools.camera_probe --camera 0 --width 1280 --height 720
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2  # noqa: E402

from follow.camera import Camera  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fourcc", default="MJPG")
    ap.add_argument("--buffer-size", type=int, default=2,
                    help="CAP_PROP_BUFFERSIZE. 0 = de mac dinh cua driver. "
                         "Xem follow/camera.py de biet ly do mac dinh la 2")
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--cam-rot", type=int, default=0,
                    choices=[0, 90, 180, 270],
                    help="Xoay khung hinh de bu camera lap xoay "
                         "(giong --cam-rot cua follow.app)")
    ap.add_argument("--out", default="camera_probe_snapshot.jpg")
    args = ap.parse_args()

    print(f"[+] Dang mo camera {args.camera} @ {args.width}x{args.height} "
          f"fourcc={args.fourcc} buffersize={args.buffer_size} ...")
    cam = Camera(args.camera, args.width, args.height, args.fourcc,
                buffersize=args.buffer_size, rotate=args.cam_rot)
    print(f"[+] Da mo: {cam.width}x{cam.height}  fourcc thuc te={cam.fourcc}")

    t_end = time.monotonic() + args.seconds
    n_frames = 0
    last_seq = -1
    n_dup_skip = 0
    frame = None
    while time.monotonic() < t_end:
        frame, seq, t_cap = cam.read()
        if frame is None:
            time.sleep(0.005)
            continue
        if seq == last_seq:
            n_dup_skip += 1
            time.sleep(0.002)
            continue
        last_seq = seq
        n_frames += 1

    fps = n_frames / args.seconds
    print(f"\n[+] Ket qua sau {args.seconds:.0f}s:")
    print(f"    khung MOI nhan duoc : {n_frames}  (~{fps:.1f} FPS)")
    print(f"    khung trung bi bo qua: {n_dup_skip}")
    print(f"    loi doc camera (errors): {cam.errors}")
    print(f"    tuoi khung hien tai  : {cam.age * 1000:.1f} ms")

    if frame is not None:
        ok = cv2.imwrite(args.out, frame)
        print(f"    da luu anh chup vao : {args.out} "
              f"({'OK' if ok else 'THAT BAI'})")
        print(f"    kich thuoc anh      : {frame.shape[1]}x{frame.shape[0]}, "
              f"gia tri pixel trung binh (BGR) = {frame.mean(axis=(0,1))}")
    else:
        print("    [!] KHONG lay duoc khung hinh nao ca!")

    cam.release()

    if n_frames == 0:
        print("\n[FAIL] Camera khong tra ve khung hinh nao. Kiem tra "
              "/dev/video*, quyen truy cap (nhom 'video'), hoac dang bi "
              "chuong trinh khac chiem dung.")
        sys.exit(1)
    if fps < 5:
        print(f"\n[CANH BAO] FPS rat thap ({fps:.1f}). Kiem tra fourcc/do "
              f"phan giai, hoac cong USB (USB2 vs USB3).")
    print("\n[PASS] Camera hoat dong.")


if __name__ == "__main__":
    main()
