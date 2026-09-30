"""Quan ly muc tieu (Target Management) voi nhan dien dac diem (appearance).
Fix C2: them dac trung HSV histogram cho moi muc tieu.
Fix C3: reacquire tu choi khi nhap nhang (2 ung cu vien qua giong nhau).

v2.1:
  - dac trung lay o VUNG THAN TREN (15-55% chieu cao) thay vi ca box:
    bo chan/nen co/be tong nen it bi lan mau nen
  - bo qua diem qua toi / bao hoa thap khi dung histogram mau
  - CHI cap nhat embedding khi track dang thay ro va khong cham bien khung
  - co t_lost: ban kinh tim lai NO RA theo thoi gian mat (plan muc 6.4)
  - AP DUNG `reacquire` timeout — truoc day tham so nay bi bo quen, muc tieu
    mat dau nam trong danh sach vinh vien
  - sau khi tim lai thanh cong: bat co `just_reacquired` de vong dieu khien
    ep yaw = 0 trong `reacq_hold_s` giay cho nguoi van hanh kip huy
"""

import time

import cv2
import numpy as np

MAU = [(0, 255, 0), (255, 140, 0), (255, 80, 255), (0, 220, 255),
       (120, 200, 255), (200, 255, 120), (255, 200, 80), (150, 150, 255)]


class Appearance:
    """Rut trich dac trung mau sac (HSV histogram) — fix C2."""
    H_BINS, S_BINS = 16, 16
    TORSO_TOP, TORSO_BOT = 0.15, 0.55    # ti le chieu cao dung lam ROI
    SAT_MIN, VAL_MIN = 40, 40            # bo diem qua nhat / qua toi

    @staticmethod
    def embed(img, box, torso=True):
        """Tra ve histogram phang 256 chieu hoac None."""
        if img is None or box is None:
            return None
        x1, y1, x2, y2 = (int(v) for v in box[:4])
        h_img, w_img = img.shape[:2]

        if torso:
            bh = y2 - y1
            if bh > 30:
                y1 = int(y1 + Appearance.TORSO_TOP * bh)
                y2 = int(y2 - (1.0 - Appearance.TORSO_BOT) * bh)

        x1, x2 = max(0, x1), min(w_img, x2)
        y1, y2 = max(0, y1), min(h_img, y2)
        if x2 - x1 < 10 or y2 - y1 < 10:
            return None

        crop = img[y1:y2, x1:x2]
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, Appearance.SAT_MIN, Appearance.VAL_MIN),
                           (180, 255, 255))
        # anh qua toi/xam -> mask gan rong, dung ca crop con hon khong co gi
        if cv2.countNonZero(mask) < 0.15 * mask.size:
            mask = None

        hist = cv2.calcHist([hsv], [0, 1], mask,
                            [Appearance.H_BINS, Appearance.S_BINS],
                            [0, 180, 0, 256])
        if float(hist.sum()) <= 0.0:
            return None
        cv2.normalize(hist, hist, alpha=0, beta=1,
                      norm_type=cv2.NORM_MINMAX)
        return hist.flatten().astype(np.float32)

    @staticmethod
    def dist(emb1, emb2):
        """Bhattacharyya. 0 = giong het, 1 = khac hoan toan."""
        if emb1 is None or emb2 is None:
            return 1.0
        d = float(cv2.compareHist(
            emb1, emb2, cv2.HISTCMP_BHATTACHARYYA))
        return d if np.isfinite(d) else 1.0


class LockedTarget:
    """Luu tru muc tieu da khoa — mau, dac trung, trang thai."""

    def __init__(self, tid, nhan, box, mau, emb=None, now=None):
        self.tid = tid                        # track ID
        self.nhan = nhan                      # nhan hien thi, vd "M1"
        self.mau = mau                        # (B, G, R)
        self.box = box.copy() if hasattr(box, "copy") else np.array(box, np.float32)
        self.last_box = self.box.copy()
        self.emb = emb                        # histogram
        self.seen = False                     # True = co mat trong frame nay
        self.needs_confirm = False            # True = reacquire nhap nhang
        # Track moi tim thay sau khi mat ID. Day chi la UNG VIEN: tuyet doi
        # khong doi tid hay phat yaw cho toi khi nguoi van hanh bam xac nhan.
        self.pending_tid = None
        self.pending_box = None
        self.t_lost = None                    # thoi diem mat dau (monotonic)
        self.t_reacq = None                   # lan tim lai thanh cong gan nhat
        self.n_reacq = 0

    @property
    def cx(self):
        return float(self.box[0] + self.box[2]) / 2.0

    @property
    def cy(self):
        return float(self.box[1] + self.box[3]) / 2.0

    def lost_for(self, now):
        return 0.0 if self.t_lost is None else max(0.0, now - self.t_lost)


class TargetManager:
    """Quan ly nhieu muc tieu khoa, chon muc tieu chinh, reacquire.

    API tuong thich voi code goc:
      pick_at(xn, yn, tracks, W, H) -> bool
      update(tracks, W, H, frame=None) -> target_box hoac None
      unlock_all(), unlock_primary(), set_primary(nhan), next_primary()
      primary (str|None), locked (list[LockedTarget]), primary_target, msg
    """
    ACCEPT = 0.45        # nguong Bhattacharyya chap nhan
    MARGIN = 0.12        # khoang cach giua 2 ung cu vien toi thieu
    EMA = 0.9            # trong so embedding cu khi cap nhat
    EDGE_PX = 4          # box cham bien -> khong cap nhat embedding

    def __init__(self, reacquire=4.0, r_base=120, reacq_hold_s=0.5):
        self.locked: list = []       # danh sach LockedTarget
        self.primary: str = None     # nhan cua muc tieu chinh, vd "M1"
        self.msg = ""                # thong bao cuoi cung
        self.reacquire = reacquire   # giay truoc khi BO HAN muc tieu
        self.reacq_hold_s = reacq_hold_s
        self._r_base = r_base
        self._cnt = 0                # dem de tao nhan moi

    # ------------------------------------------------------- properties
    @property
    def primary_target(self):
        for m in self.locked:
            if m.nhan == self.primary:
                return m
        return None

    @property
    def primary_ready(self):
        """Muc tieu chinh du dieu kien de bat dau ENGAGE hay chua.

        Co ``LockedTarget`` khong dong nghia la duoc phep theo: muc tieu co
        the da LOST hoac dang cho nguoi van hanh xac nhan lai identity.
        """
        m = self.primary_target
        return bool(m is not None and m.seen and not m.needs_confirm)

    def just_reacquired(self, now=None) -> bool:
        """True trong `reacq_hold_s` giay sau khi muc tieu chinh duoc tim lai.
        Vong dieu khien dung co nay de ep yaw = 0 cho nguoi van hanh kip nhin."""
        m = self.primary_target
        if m is None or m.t_reacq is None:
            return False
        now = time.monotonic() if now is None else now
        return (now - m.t_reacq) < self.reacq_hold_s

    # ------------------------------------------------------- pick / lock
    def pick_at(self, xn, yn, tracks, W, H, frame=None, now=None):
        """Khoa muc tieu tai toa do chuot (normalized 0-1)."""
        px, py = xn * W, yn * H
        best, bd = None, 1e9
        for t in tracks:
            cx = (t.box[0] + t.box[2]) / 2.0
            cy = (t.box[1] + t.box[3]) / 2.0
            d = ((cx - px) ** 2 + (cy - py) ** 2) ** 0.5
            if d < bd and d < max(W, H) * 0.15:
                bd, best = d, t
        if best is None:
            self.msg = "khong trung ai"
            return False
        now = time.monotonic() if now is None else now
        # Kiem tra da khoa chua, hoac dang la ung vien reacquire. Click cua
        # nguoi van hanh la cach DUY NHAT de chuyen ung vien thanh target that.
        for m in self.locked:
            if m.tid == best.id or m.pending_tid == best.id:
                was_pending = m.pending_tid == best.id
                if was_pending:
                    m.tid = best.id
                    m.box = best.box.copy()
                    m.last_box = m.box.copy()
                    m.seen = True
                    m.t_lost = None
                    m.t_reacq = now
                    m.n_reacq += 1
                    m.pending_tid = None
                    m.pending_box = None
                self.primary = m.nhan
                # nguoi van hanh vua bam vao dung nguoi nay = da xac nhan.
                # Khong duoc tu clear co nay o update(): do se cho phep mot
                # reacquire nhap nhang tu dong chay lai o frame ke tiep.
                m.needs_confirm = False
                m.t_lost = None
                if not was_pending:
                    m.t_reacq = None
                if frame is not None:
                    e = Appearance.embed(frame, best.box)
                    if e is not None:
                        m.emb = e
                self.msg = f"da chon {m.nhan} lam chinh"
                return True

        # Neu M1 bi LOST/nhap nhang, click vao mot track khac la xac nhan
        # minh bach cua nguoi van hanh rang track do chinh la M1. Phai xu ly
        # truoc khi tao M moi, neu khong M1 bi treo va click khong co tac dung.
        pm = self.primary_target
        if pm is not None and not pm.seen and pm.needs_confirm:
            pm.tid = best.id
            pm.box = best.box.copy()
            pm.last_box = pm.box.copy()
            pm.seen = True
            pm.t_lost = None
            pm.t_reacq = now
            pm.n_reacq += 1
            pm.pending_tid = None
            pm.pending_box = None
            pm.needs_confirm = False
            if frame is not None:
                e = Appearance.embed(frame, best.box)
                if e is not None:
                    pm.emb = e
            self.msg = (f"da xac nhan lai {pm.nhan} (ID {best.id}) - "
                        f"yaw tam dung {self.reacq_hold_s:.1f}s")
            return True
        # tao moi
        self._cnt += 1
        nhan = f"M{self._cnt}"
        mau = MAU[(self._cnt - 1) % len(MAU)]
        emb = None
        if frame is not None:
            emb = Appearance.embed(frame, best.box)
        m = LockedTarget(best.id, nhan, best.box, mau, emb)
        m.seen = True
        self.locked.append(m)
        if self.primary is None:
            self.primary = nhan
        self.msg = f"da khoa {nhan} (ID {best.id})"
        return True

    # ------------------------------------------------------- unlock
    def unlock_all(self):
        self.locked.clear()
        self.primary = None
        self._cnt = 0
        self.msg = "da bo khoa tat ca"

    def unlock_primary(self):
        if self.primary is None:
            self.msg = "khong co muc tieu chinh"
            return
        self.locked = [m for m in self.locked if m.nhan != self.primary]
        self.msg = f"da bo khoa {self.primary}"
        # TUYET DOI khong tu dong chuyen sang nguoi khac. Nguoi van hanh da
        # chon M1 thi bo M1 phai lam guidance ve 0 cho den khi ho click M2.
        self.primary = None

    # ------------------------------------------------------- switch
    def set_primary(self, nhan):
        for m in self.locked:
            if m.nhan == nhan:
                self.primary = nhan
                self.msg = f"da chon {nhan} lam chinh"
                return
        self.msg = f"khong tim thay {nhan}"

    def next_primary(self):
        if len(self.locked) < 2:
            self.msg = "chi co 1 muc tieu"
            return
        names = [m.nhan for m in self.locked]
        idx = names.index(self.primary) if self.primary in names else -1
        self.primary = names[(idx + 1) % len(names)]
        self.msg = f"chuyen sang {self.primary}"

    # ------------------------------------------------------- update
    def update(self, tracks, W, H, frame=None, now=None):
        """Cap nhat moi frame. Tra ve box cua muc tieu chinh hoac None."""
        if not self.locked:
            return None
        now = time.monotonic() if now is None else now
        tid2t = {t.id: t for t in tracks}
        bo_di = []

        for m in self.locked:
            t = tid2t.get(m.tid)
            if t is not None:
                m.box = t.box.copy()
                m.last_box = m.box.copy()
                m.seen = True
                m.t_lost = None
                if frame is not None and self._emb_updatable(t.box, W, H):
                    new_emb = Appearance.embed(frame, t.box)
                    if new_emb is not None:
                        if m.emb is None:
                            m.emb = new_emb
                        else:
                            m.emb = (self.EMA * m.emb
                                     + (1.0 - self.EMA) * new_emb)
                continue

            # --- khong thay: dem thoi gian mat dau
            m.seen = False
            if m.t_lost is None:
                m.t_lost = now
            if self.reacquire > 0 and m.lost_for(now) > self.reacquire:
                bo_di.append(m)          # AP DUNG timeout (truoc day bi quen)
                continue

            # Sau mot lan identity bi nhap nhang, chi click cua nguoi van hanh
            # moi duoc phep tiep tuc. Khong duoc "thu lai" den khi mot frame
            # tinh co tro nen ro rang roi tu dong doi nguoi.
            if m.needs_confirm:
                continue

            found = self._try_reacquire(m, tracks, frame, W, now)
            if found is not None:
                # Re-ID bang HSV + khoang cach van chi la xac suat. Luu ung
                # vien de ve HUD, chan yaw, va bat buoc operator click xac nhan.
                m.pending_tid = found.id
                m.pending_box = found.box.copy()
                m.needs_confirm = True
                self.msg = (f"{m.nhan}: tim thay ung vien ID {found.id} "
                            "-> bam xac nhan lai, yaw dang dung")

        if bo_di:
            names = ", ".join(m.nhan for m in bo_di)
            self.locked = [m for m in self.locked if m not in bo_di]
            self.msg = f"bo khoa {names}: mat qua {self.reacquire:.0f}s"
            if self.primary not in [m.nhan for m in self.locked]:
                # Khong bao gio auto-promote M2 thanh primary khi M1 mat.
                self.primary = None

        pm = self.primary_target
        if pm is not None and pm.seen:
            return pm.box
        return None

    def _emb_updatable(self, box, W, H) -> bool:
        """Chi hoc ngoai hinh tu box nam TRON trong khung, du lon."""
        e = self.EDGE_PX
        if box[0] <= e or box[1] <= e or box[2] >= W - e or box[3] >= H - e:
            return False
        return (box[2] - box[0]) >= 20 and (box[3] - box[1]) >= 40

    def _try_reacquire(self, m, tracks, frame=None, W=1280, now=None):
        """Tim lai muc tieu da mat. Tu choi khi nhap nhang (fix C3)."""
        if not tracks:
            return None
        # khong reacquire nhung track da thuoc muc tieu khac
        used_ids = {x.tid for x in self.locked}
        avail = [t for t in tracks if t.id not in used_ids]
        if not avail:
            return None

        # ban kinh NO RA theo thoi gian mat, chan tren 0.30*W (plan 6.4)
        now = time.monotonic() if now is None else now
        dt_lost = m.lost_for(now)
        r_max = min(0.12 * W + 120.0 * dt_lost, 0.30 * W)
        r_max = max(r_max, float(self._r_base))

        cx, cy = m.cx, m.cy
        cands = []
        for t in avail:
            tcx = (t.box[0] + t.box[2]) / 2.0
            tcy = (t.box[1] + t.box[3]) / 2.0
            d = ((tcx - cx) ** 2 + (tcy - cy) ** 2) ** 0.5
            if d > r_max:
                continue
            emb = None
            if frame is not None:
                emb = Appearance.embed(frame, t.box)
            app_d = Appearance.dist(m.emb, emb)
            score = 0.35 * (d / r_max) + 0.65 * app_d
            cands.append((score, app_d, t))

        if not cands:
            return None
        cands.sort(key=lambda x: x[0])
        best_score, best_app, best_t = cands[0]

        # tu choi neu ngoai hinh khac qua
        if best_app > self.ACCEPT:
            self.msg = f"{m.nhan}: ung vien khong khop ngoai hinh"
            return None
        # tu choi neu nhap nhang — hai ung cu vien qua giong nhau (fix C3)
        if len(cands) >= 2:
            second_score = cands[1][0]
            if second_score - best_score < self.MARGIN:
                m.needs_confirm = True
                self.msg = (f"{m.nhan}: reacquire NHAP NHANG "
                            f"({len(cands)} ung vien) -> can bam xac nhan lai")
                return None
        return best_t
