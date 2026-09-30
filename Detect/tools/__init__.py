"""tools — Cac tien ich bench/maintenance doc lap voi follow.app.

Ep MAVLink2 truoc khi bat ky script con nao import pymavlink.mavutil, cung
ly do nhu follow/__init__.py: PARAM_VALUE/ODOMETRY va cac ban tin id > 255
can dialect MAVLink2 moi co class de nhan. Package nay chay truoc khi bat ky
tools.xxx nao duoc import (vd `python -m tools.px4_param`).
"""
import os

os.environ.setdefault("MAVLINK20", "1")
