"""Chan doan chi tiet detect.py: in ra TUNG khung - model thay gi (RAW,
truoc loc) va sau khi loc (postprocess that) khac nhau the nao. Dung khi
nghi ngo bo loc aspect/kich thuoc loai oan detection dung.

Chay (dung nguoi trong camera trong luc chay):
    python3 -m tools.detect_diag --model <duong dan model> --seconds 8
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np  # noqa: E402

from follow.camera import Camera  # noqa: E402
from follow.detect import MIN_BOX_H, PERSON, letterbox, load_model  # noqa: E402


def raw_boxes(det, r, dw, dh, frame_h, frame_w, conf_thr,
             min_aspect, max_aspect, min_box_h):
    """Giong postprocess() nhung KHONG loc aspect/kich thuoc - de so sanh."""
    det = np.asarray(det, np.float32)
    det = det[np.isfinite(det[:, :6]).all(axis=1)]
    det = det[det[:, 4] >= conf_thr]
    det = det[det[:, 5].astype(int) == PERSON]
    if len(det) == 0:
        return []
    b = det[:, :4].copy()
    b[:, [0, 2]] -= dw
    b[:, [1, 3]] -= dh
    b /= max(r, 1e-6)
    out = []
    for i in range(len(b)):
        x1, y1, x2, y2 = b[i]
        w, h = x2 - x1, y2 - y1
        aspect = h / max(w, 1.0)
        at_edge = (x1 <= 4 or x2 >= frame_w - 4 or y1 <= 4 or y2 >= frame_h - 4)
        ok_h = h >= min_box_h
        ok_asp = min_aspect <= aspect <= max_aspect
        passed = ok_h and (ok_asp or at_edge)
        reason = []
        if not ok_h:
            reason.append(f"qua thap(h={h:.0f}<{min_box_h})")
        if not ok_asp and not at_edge:
            reason.append(f"aspect={aspect:.2f} ngoai [{min_aspect},{max_aspect}]")
        out.append({
            "box": (int(x1), int(y1), int(x2), int(y2)),
            "w": int(w), "h": int(h), "aspect": round(aspect, 2),
            "conf": round(float(det[i, 4]), 2),
            "passed": passed, "reason": "; ".join(reason) or "OK",
        })
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--model", required=True)
    ap.add_argument("--device", default="GPU")
    ap.add_argument("--imgsz", type=int, default=416)
    ap.add_argument("--conf", type=float, default=0.25,
                    help="Thap hon binh thuong de xem het ca nhung box "
                         "gan nguong, khong chi >=0.4")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--cam-rot", type=int, default=0,
                    choices=[0, 90, 180, 270],
                    help="Xoay khung hinh de bu camera lap xoay "
                         "(giong --cam-rot cua follow.app)")
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--min-aspect", type=float, default=1.2,
                    help="Mac dinh follow/detect.py=1.2 (nguoi DUNG tu xa)")
    ap.add_argument("--max-aspect", type=float, default=5.0)
    ap.add_argument("--min-box-h", type=float, default=24.0)
    ap.add_argument("--quiet", action="store_true",
                    help="Chi in tom tat, khong in tung khung")
    args = ap.parse_args()

    compiled, xml = load_model(args.model, args.device, args.imgsz)
    req = compiled.create_infer_request()
    op = compiled.output(0)
    print(f"[+] Model {xml} tren {args.device}\n")

    cam = Camera(args.camera, args.width, args.height, "MJPG",
                 rotate=args.cam_rot)
    time.sleep(0.5)

    t_end = time.monotonic() + args.seconds
    n_frame = 0
    n_with_raw = 0
    n_with_pass = 0
    last_seq = -1
    while time.monotonic() < t_end:
        frame, seq, t = cam.read()
        if frame is None or seq == last_seq:
            time.sleep(0.01)
            continue
        last_seq = seq
        n_frame += 1

        lb, r, dw, dh = letterbox(frame, args.imgsz)
        req.infer({0: lb[None]})
        det = req.get_tensor(op).data[0]
        boxes = raw_boxes(det, r, dw, dh, frame.shape[0], frame.shape[1],
                          args.conf, args.min_aspect, args.max_aspect,
                          args.min_box_h)

        if boxes:
            n_with_raw += 1
            any_pass = any(b["passed"] for b in boxes)
            if any_pass:
                n_with_pass += 1
            if not args.quiet:
                tag = "PASS" if any_pass else "BI LOC HET"
                print(f"[khung {n_frame:3d}] {tag}")
                for b in boxes:
                    mark = "OK " if b["passed"] else "LOAI"
                    print(f"    {mark} box={b['box']} w={b['w']} h={b['h']} "
                          f"aspect={b['aspect']} conf={b['conf']} - {b['reason']}")
        elif not args.quiet:
            print(f"[khung {n_frame:3d}] KHONG co detection nao "
                  f"(conf>={args.conf})")

    cam.release()
    print(f"\n=== Tong ket {n_frame} khung trong {args.seconds:.0f}s ===")
    print(f"  co it nhat 1 detection RAW (truoc loc) : {n_with_raw} "
          f"({100*n_with_raw/max(n_frame,1):.0f}%)")
    print(f"  co it nhat 1 detection QUA DUOC bo loc  : {n_with_pass} "
          f"({100*n_with_pass/max(n_frame,1):.0f}%)")
    if n_with_raw > n_with_pass:
        print(f"\n  => Model THAY nguoi nhieu hon so lan hien thi len man "
              f"hinh. Bo loc aspect/kich thuoc dang loai oan "
              f"{n_with_raw - n_with_pass} khung.")


if __name__ == "__main__":
    main()
