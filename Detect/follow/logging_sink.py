"""Ghi CSV va video tren luong rieng, khong chan vong dieu khien.
Fix D2: flush CSV sau moi dong.
Fix D3: tach I/O hoan toan khoi vong lap chinh."""

import csv
import queue
import threading
import time
from datetime import datetime

import cv2


class LoggingSink:
    """Nhan du lieu tu vong dieu khien qua queue, ghi CSV va video tren luong rieng."""

    CSV_HEADER = [
        "timestamp", "state", "engaged", "block", "muc_tieu", "n_khoa",
        "lech_goc_do", "yaw_cmd_deg_s", "yaw_cmd_rad_s", "yaw_thuc_deg_s",
        "mode", "armed", "batt_v", "batt_pct", "rc_val", "rc_override",
        # --- v2.1: cac cot phuc vu dieu tra su co "bay mat kiem soat"
        "fence", "fence_why", "dist_m", "home_m", "alt_m", "speed_ms",
        "drift_m", "engage_s",
        "sp_hz", "fps", "infer_ms", "latency_ms", "at_edge",
        "watchdog_trips", "emergency_stops", "loop_wd_trips", "nan_blocks",
        "clamp_hits", "log_drops", "log_errors",
    ]

    def __init__(self, csv_path, video_path=None, video_fps=20.0,
                 video_size=None, max_queue=64):
        self._q = queue.Queue(maxsize=max_queue)
        self._stop = threading.Event()
        self._csv_path = csv_path
        self._video_path = video_path
        self._video_fps = video_fps
        self._video_size = video_size
        self.dropped = 0
        self.errors = 0
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def push(self, row_dict, frame=None):
        """Day du lieu vao queue. Bo qua neu queue day (khong chan vong dieu khien)."""
        try:
            self._q.put_nowait((row_dict, frame))
        except queue.Full:
            # Khong chan control loop, NHUNG phai dem de post-flight khong
            # nham log thieu la du lieu day du.
            self.dropped += 1

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=3.0)

    def _run(self):
        cf = open(self._csv_path, "w", newline="", encoding="utf-8")
        cw = csv.DictWriter(cf, fieldnames=self.CSV_HEADER,
                            extrasaction="ignore")
        cw.writeheader()
        cf.flush()

        writer = None
        if self._video_path and self._video_size:
            writer = cv2.VideoWriter(
                self._video_path,
                cv2.VideoWriter_fourcc(*"mp4v"),
                self._video_fps, self._video_size)

        try:
            while True:
                try:
                    row, frame = self._q.get(timeout=0.5)
                except queue.Empty:
                    if self._stop.is_set():
                        break        # da dung VA queue rong -> thoat
                    continue
                try:
                    if row:
                        row["timestamp"] = datetime.now().isoformat(
                            timespec="milliseconds")
                        cw.writerow(row)
                        cf.flush()           # fix D2: flush moi dong
                    if frame is not None and writer is not None:
                        writer.write(frame)
                except Exception:
                    # Loi disk/codec khong duoc lam chet luong logger va an
                    # mat bang chung dieu tra. Control loop van uu tien, con
                    # status nay phai duoc ghi vao Web/CSV o cac chu ky sau.
                    self.errors += 1
        finally:
            cf.close()
            if writer is not None:
                writer.release()
