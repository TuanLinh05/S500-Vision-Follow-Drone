"""Camera capture voi seq + timestamp.
Fix C5: them _seq de phat hien khung trung.

Fix v2.1 (phat hien qua bench that tren UP7000 + Rapoo C280 / chip
Microdia): CAP_PROP_BUFFERSIZE=1 lam giam gan MOT NUA thong luong thuc te
tren driver V4L2 nay (11.5 fps thay vi ~23 fps o 1280x720 MJPG, do bang
v4l2-ctl truc tiep xac nhan camera/driver hoan toan lam duoc 30fps that -
nghen nam o cach OpenCV dequeue/requeue buffer khi hang doi chi co 1 cho).
buffersize=2 lay duoc gan het loi ich thong luong (~21fps, so voi 23fps
khong gioi han) ma van gioi han do tre o muc thap (toi da 2 khung ~ 100ms
o toc do nay). Da do thuc te, khong doan."""

import threading
import time

import cv2

DEFAULT_BUFFERSIZE = 2   # xem ghi chu o tren - do thuc nghiem tren UP7000

# Xoay khung hinh NGAY TAI NGUON (truoc detect/track/control/HUD/video), de
# moi thanh phan phia sau deu lam viec tren anh da dung chieu. Rat quan trong:
# YawController tinh sai so goc tu toa do NGANG (truc x) cua muc tieu. Neu
# camera lap xoay 90 do ma khong bu lai, nguoi di sang trai/phai THAT se hien
# ra la di len/xuong trong anh -> app khong thay lech ngang (khong xoay), dong
# thoi thu gi lam doi vi tri DOC trong anh lai bi hieu nham thanh lech ngang
# -> ra lenh xoay SAI. Da gap that tren UP7000 ngay 2026-09-04.
ROTATIONS = {
    0: None,
    90: cv2.ROTATE_90_CLOCKWISE,          # anh bi xoay CCW -> bu lai bang CW
    180: cv2.ROTATE_180,
    270: cv2.ROTATE_90_COUNTERCLOCKWISE,  # anh bi xoay CW -> bu lai bang CCW
}


class Camera:
    def __init__(self, index=0, width=1280, height=720, fourcc="MJPG",
                 source=None, buffersize=DEFAULT_BUFFERSIZE, rotate=0):
        rotate = int(rotate) % 360
        if rotate not in ROTATIONS:
            raise SystemExit(
                f"[!] --cam-rot khong hop le: {rotate} "
                f"(chi nhan {sorted(ROTATIONS)})")
        self.rotate = rotate
        self._rot_code = ROTATIONS[rotate]
        if source:
            self.cap = cv2.VideoCapture(str(source))
            self.is_file = True
        else:
            self.is_file = False
            self.cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
            if not self.cap.isOpened():
                self.cap = cv2.VideoCapture(index)
            if fourcc and fourcc.upper() != "AUTO":
                self.cap.set(cv2.CAP_PROP_FOURCC,
                             cv2.VideoWriter_fourcc(*fourcc.upper()))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            if buffersize and buffersize > 0:
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, buffersize)
        if not self.cap.isOpened():
            raise SystemExit(f"[!] Khong mo duoc nguon ({source or index})")
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        # Xoay 90/270 lam DOI CHO rong<->cao. width/height phai la kich thuoc
        # SAU xoay vi YawController dung cam.width lam tam khung, va
        # LoggingSink dung (width,height) lam kich thuoc video.
        if rotate in (90, 270):
            self.width, self.height = self.height, self.width
        v = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        self.fourcc = "".join(chr((v >> 8 * i) & 0xFF) for i in range(4)).strip() or "?"

        self._f = None
        self._seq = 0          # tang moi khi co khung MOI (fix C5)
        self._t_cap = 0.0      # timestamp monotonic cua khung
        self._lk = threading.Lock()
        self._done = False
        self.errors = 0
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while not self._done:
            ok, img = self.cap.read()
            if not ok:
                if self.is_file:
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue
                self.errors += 1
                time.sleep(0.2)
                continue
            if self._rot_code is not None:
                img = cv2.rotate(img, self._rot_code)
            with self._lk:
                self._f = img
                self._seq += 1
                self._t_cap = time.monotonic()
            if self.is_file:
                time.sleep(0.033)

    def read(self):
        """Tra ve (frame, seq, t_capture). Dung seq de phat hien khung trung."""
        with self._lk:
            return self._f, self._seq, self._t_cap

    @property
    def age(self):
        with self._lk:
            return time.monotonic() - self._t_cap if self._t_cap else 999.0

    def release(self):
        self._done = True
        time.sleep(0.1)
        self.cap.release()
