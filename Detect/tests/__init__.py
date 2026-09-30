# Ep MAVLink2 truoc khi bat ky test module nao import pymavlink.mavutil (xem
# follow/__init__.py). Package nay luon duoc nap truoc tests.fake_px4 /
# tests.test_* vi Python chay __init__.py cua package truoc submodule.
import os

os.environ.setdefault("MAVLINK20", "1")
