"""GMC (Global Motion Compensation) va ByteTrack tracker."""

import cv2
import numpy as np


class GMC:
    """Phase correlation de uoc luong chuyen dong camera."""

    def __init__(self, width=160, max_ratio=0.25):
        self.w = width
        self.max_ratio = max_ratio
        self.prev = None

    def estimate(self, frame):
        h, w = frame.shape[:2]
        nh = max(2, int(round(h * self.w / w)))
        small = cv2.cvtColor(
            cv2.resize(frame, (self.w, nh), interpolation=cv2.INTER_AREA),
            cv2.COLOR_BGR2GRAY).astype(np.float32)
        if self.prev is None or self.prev.shape != small.shape:
            self.prev = small
            return 0.0, 0.0
        try:
            (sx, sy), resp = cv2.phaseCorrelate(self.prev, small)
        except cv2.error:
            self.prev = small
            return 0.0, 0.0
        self.prev = small
        k = w / float(self.w)
        dx, dy = sx * k, sy * k
        if (abs(dx) > self.max_ratio * w or
                abs(dy) > self.max_ratio * h or resp < 0.03):
            return 0.0, 0.0
        return dx, dy


def _iou(a, b):
    """IoU matrix [len(a), len(b)]."""
    if len(a) == 0 or len(b) == 0:
        return np.zeros((len(a), len(b)), np.float32)
    ax1, ay1, ax2, ay2 = a[:, 0:1], a[:, 1:2], a[:, 2:3], a[:, 3:4]
    bx1, by1, bx2, by2 = b[:, 0], b[:, 1], b[:, 2], b[:, 3]
    iw = np.clip(np.minimum(ax2, bx2) - np.maximum(ax1, bx1), 0, None)
    ih = np.clip(np.minimum(ay2, by2) - np.maximum(ay1, by1), 0, None)
    inter = iw * ih
    return (inter / np.maximum(
        (ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - inter,
        1e-6)).astype(np.float32)


class _Track:
    __slots__ = ("id", "box", "vel", "conf", "hits", "lost")

    def __init__(self, tid, box, conf):
        self.id = tid
        self.box = box.astype(np.float32)
        self.vel = np.zeros(4, np.float32)
        self.conf = float(conf)
        self.hits = 1
        self.lost = 0

    def predict(self, shift=None):
        p = self.box + self.vel
        if shift is not None:
            p = p + np.array([shift[0], shift[1], shift[0], shift[1]],
                             np.float32)
        return p

    def update(self, box, conf):
        box = box.astype(np.float32)
        self.vel = 0.5 * self.vel + 0.5 * (box - self.box)
        self.box = box
        self.conf = float(conf)
        self.hits += 1
        self.lost = 0

    def shift_by(self, dx, dy):
        self.box = self.box + np.array([dx, dy, dx, dy], np.float32)

    def coast(self, shift=None, decay=0.85):
        """Track mat dau: van truot theo van toc cuoi (giam dan) + GMC.

        Truoc day box dung yen khi mat dau, nen diem neo cho reacquire lech
        han so voi cho nguoi that su dang di toi."""
        self.box = self.box + self.vel
        if shift is not None:
            self.box = self.box + np.array(
                [shift[0], shift[1], shift[0], shift[1]], np.float32)
        self.vel = self.vel * decay


class ByteTrack:
    """ByteTrack: nguong cao + thap, greedy matching bang IoU."""

    def __init__(self, high=0.5, low=0.1, match=0.2, low_match=0.5,
                 max_lost=45, min_hits=3):
        self.high = high
        self.low = low
        self.match = match
        self.low_match = low_match
        self.max_lost = max_lost
        self.min_hits = min_hits
        self.tracks = []
        self._next = 1

    @staticmethod
    def _greedy(m, thr):
        pairs = []
        if m.size:
            ur, uc = set(), set()
            idx = np.dstack(
                np.unravel_index(np.argsort(-m, axis=None), m.shape))[0]
            for r, c in idx:
                if m[r, c] < thr:
                    break
                if r in ur or c in uc:
                    continue
                ur.add(int(r))
                uc.add(int(c))
                pairs.append((int(r), int(c)))
        mr = {r for r, _ in pairs}
        mc = {c for _, c in pairs}
        return (pairs,
                [i for i in range(m.shape[0]) if i not in mr],
                [j for j in range(m.shape[1]) if j not in mc])

    def update(self, boxes, scores, shift=None):
        """Tra ve danh sach track VISIBLE (lost==0 va hits>=min_hits)."""
        hi = scores >= self.high
        lo = (scores >= self.low) & ~hi
        pred = (np.array([t.predict(shift) for t in self.tracks], np.float32)
                if self.tracks else np.zeros((0, 4), np.float32))

        # buoc 1: ghep detection cao voi tat ca track
        pairs, un_t, un_d = self._greedy(
            _iou(pred, boxes[hi]), self.match)
        hidx = np.flatnonzero(hi)
        for ti, di in pairs:
            self.tracks[ti].update(boxes[hidx[di]], scores[hidx[di]])

        # buoc 2: ghep detection thap voi track chua duoc ghep
        lodx = np.flatnonzero(lo)
        if un_t and len(lodx):
            rest = np.array(
                [self.tracks[i].predict(shift) for i in un_t], np.float32)
            p2, un_t2, _ = self._greedy(
                _iou(rest, boxes[lodx]), self.low_match)
            for ri, di in p2:
                self.tracks[un_t[ri]].update(
                    boxes[lodx[di]], scores[lodx[di]])
            un_t = [un_t[i] for i in un_t2]

        # cap nhat track khong duoc ghep
        for ti in un_t:
            self.tracks[ti].lost += 1
            self.tracks[ti].coast(shift)

        # tao track moi tu detection cao chua duoc ghep
        for di in un_d:
            self.tracks.append(
                _Track(self._next, boxes[hidx[di]], scores[hidx[di]]))
            self._next += 1

        # xoa track cu
        self.tracks = [t for t in self.tracks if t.lost <= self.max_lost]
        return [t for t in self.tracks
                if t.lost == 0 and t.hits >= self.min_hits]
