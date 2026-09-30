"""Tai model OpenVINO va hau xu ly detection.
Fix C1: loc kich thuoc + ti le khung nguoi.

v2.1: LOC TRUOC KHI CLIP. Truoc day box bi clip vao khung roi moi tinh ti le,
nen nguoi dung nua trong nua ngoai khung bi coi la "qua beo" va bi loai —
dung dung tinh huong ma control.py (fix B2) da co gang xu ly. Nay ti le duoc
tinh tren box GOC, va box cham bien duoc mien kiem tra ti le.
"""

from pathlib import Path

import cv2
import numpy as np

# openvino duoc import BEN TRONG load_model: letterbox/postprocess la numpy
# thuan tuy nen kiem thu duoc tren may khong cai OpenVINO.

PERSON = 0
MIN_BOX_H = 24       # nguoi < 24 px cao thi khong tin
MIN_ASPECT = 1.2     # cao/rong toi thieu
MAX_ASPECT = 5.0     # cao/rong toi da
EDGE_PX = 4          # sat bien khung hinh bao nhieu px thi coi la bi cat

_EMPTY = (np.zeros((0, 4), np.float32), np.zeros((0,), np.float32))


def load_model(path, device, imgsz, cache="ov_cache"):
    """Tra ve (compiled_model, xml_path)."""
    import openvino as ov
    from openvino import Layout, Type
    from openvino.preprocess import ColorFormat, PrePostProcessor

    core = ov.Core()
    if cache:
        Path(cache).mkdir(exist_ok=True)
        core.set_property({"CACHE_DIR": cache})
    p = Path(path)
    if p.suffix == ".xml":
        xml = p
    else:
        found = sorted(p.glob("*.xml"))
        if not found:
            raise SystemExit(f"[!] Khong thay .xml trong '{p}'")
        xml = found[0]
    model = core.read_model(xml)
    ppp = PrePostProcessor(model)
    ppp.input().tensor().set_element_type(Type.u8) \
        .set_layout(Layout("NHWC")).set_color_format(ColorFormat.BGR)
    ppp.input().model().set_layout(Layout("NCHW"))
    ppp.input().preprocess().convert_element_type(Type.f32) \
        .convert_color(ColorFormat.RGB).scale(255.0)
    cfg = {"PERFORMANCE_HINT": "LATENCY"}
    if device.upper() == "CPU":
        cfg["NUM_STREAMS"] = "1"
    c = core.compile_model(ppp.build(), device.upper(), cfg)
    shape = c.output(0).shape
    if not (len(shape) == 3 and shape[2] == 6):
        raise SystemExit(
            f"[!] Can model NMS-free (output [batch, N, 6]), dang co {shape}.")
    return c, xml


def letterbox(img, n):
    """Resize va pad anh ve n x n. Tra ve (img_padded, scale, dw, dh)."""
    h, w = img.shape[:2]
    r = min(n / h, n / w)
    nh, nw = int(round(h * r)), int(round(w * r))
    if (nw, nh) != (w, h):
        img = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    dw, dh = (n - nw) // 2, (n - nh) // 2
    out = np.full((n, n, 3), 114, np.uint8)
    out[dh:dh + nh, dw:dw + nw] = img
    return out, r, dw, dh


def postprocess(det, r, dw, dh, frame_h, frame_w, conf_thr=0.4,
                min_box_h=MIN_BOX_H, min_aspect=MIN_ASPECT,
                max_aspect=MAX_ASPECT, edge_px=EDGE_PX):
    """Loc detection: confidence, class person, kich thuoc va ti le.

    Thu tu QUAN TRONG: quy doi ve toa do khung goc -> loc ti le tren box
    CHUA CLIP -> moi clip. Nguoc lai se loai nham nguoi dang o ria khung.

    Tra ve (boxes [N,4] da clip, scores [N]).
    """
    if det is None or len(det) == 0:
        return _EMPTY
    det = np.asarray(det, np.float32)
    if det.ndim != 2 or det.shape[1] < 6:
        return _EMPTY

    # bo cac hang chua NaN/Inf truoc moi so sanh
    det = det[np.isfinite(det[:, :6]).all(axis=1)]
    if len(det) == 0:
        return _EMPTY

    det = det[det[:, 4] >= conf_thr]
    if len(det) == 0:
        return _EMPTY
    det = det[det[:, 5].astype(int) == PERSON]
    if len(det) == 0:
        return _EMPTY

    # quy doi ve khung goc, CHUA clip
    b = det[:, :4].copy()
    b[:, [0, 2]] -= dw
    b[:, [1, 3]] -= dh
    b /= max(r, 1e-6)

    w_raw = b[:, 2] - b[:, 0]
    h_raw = b[:, 3] - b[:, 1]
    valid = (w_raw > 1.0) & (h_raw > 1.0)

    # box cham bien = nguoi bi cat -> ti le khong con y nghia, mien kiem tra
    at_edge = ((b[:, 0] <= edge_px) | (b[:, 2] >= frame_w - edge_px) |
               (b[:, 1] <= edge_px) | (b[:, 3] >= frame_h - edge_px))

    aspect = h_raw / np.maximum(w_raw, 1.0)
    ok_aspect = (aspect >= min_aspect) & (aspect <= max_aspect)

    # chieu cao van phai du: nguoi qua nho thi model khong dang tin
    ok = valid & (h_raw >= min_box_h) & (ok_aspect | at_edge)

    b = b[ok]
    sc = det[:, 4][ok]
    if len(b) == 0:
        return _EMPTY

    # clip SAU CUNG, chi de ve/track cho gon trong khung.
    # LUU Y: phai clip theo TUNG COT. `b[:, [0, 2]]` la advanced indexing ->
    # tra ve BAN SAO, nen np.clip(..., out=b[:, [0, 2]]) ghi vao ban sao roi
    # vut di: box thuc te khong he bi clip. Day la loi co san trong v2.0.
    b[:, 0] = np.clip(b[:, 0], 0.0, frame_w)
    b[:, 2] = np.clip(b[:, 2], 0.0, frame_w)
    b[:, 1] = np.clip(b[:, 1], 0.0, frame_h)
    b[:, 3] = np.clip(b[:, 3], 0.0, frame_h)

    return b.astype(np.float32), sc.astype(np.float32)
