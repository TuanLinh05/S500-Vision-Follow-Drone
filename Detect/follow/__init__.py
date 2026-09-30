"""follow — Drone PX4 xoay bam theo nguoi duoc chon.

QUAN TRONG: ep MAVLink2 truoc khi bat ky module con nao import pymavlink.
PX4 phat mot so ban tin co id > 255 (vd ODOMETRY, dung de doc reset_counter
cho tinh nang phat hien EKF reset) - dialect MAVLink1 mac dinh cua pymavlink
KHONG co class cho cac ban tin nay (id khong ma hoa duoc trong khung
MAVLink1), nen viec doc se LANG LE khong bao gio nhan duoc gi, khong loi,
khong canh bao. Bien moi truong nay phai duoc dat truoc lan import
`pymavlink.mavutil` DAU TIEN trong ca tien trinh - dat o day la noi bao dam
chay truoc vi Python luon nap __init__.py cua package truoc bat ky module
con nao (follow.px4, ...).

Module layout:
  camera.py       Camera capture (seq + timestamp)
  detect.py       OpenVINO model + letterbox + postprocess
  track.py        GMC + ByteTrack
  target.py       LockedTarget + TargetManager + Appearance
  control.py      YawController (thuan tuy, khong I/O)
  safety.py       SafetyGate (thuan tuy, khong I/O)
  geofence.py     Hang rao mem (thuan tuy, khong I/O)
  watchdog.py     LoopWatchdog + CommandAudit + StickMonitor
  px4.py          PX4Link + SetpointStreamer (MAVLink)
  webui.py        HTTP server + token + dead-man
  logging_sink.py Ghi CSV + video tren luong rieng
  app.py          Vong lap chinh — diem vao chuong trinh
"""
import os

os.environ.setdefault("MAVLINK20", "1")

__version__ = "2.4.0"
