"""Dieu khien tu dong: arm (MAVLink) -> pick muc tieu -> giu ENGAGE -> theo
doi /stats -> nha ENGAGE -> xac nhan LOITER. Chi dung cho SITL (khong co
nguoi that cam RC).

Doc token TRUC TIEP tu file log cua follow.app - tranh truyen qua bien
shell xuyen nhieu lop (git-bash -> wsl.exe -> bash -c) de khong bi rong.

Chay:
    python -m tools.sitl_drive udp:127.0.0.1:14540 8090 <duong_dan_log> <engage_s>
"""
import json
import re
import sys
import threading
import time
import urllib.error
import urllib.request


def read_token(log_path, timeout=10.0):
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        try:
            text = open(log_path, encoding="utf-8", errors="replace").read()
            m = re.search(r"WEBUI TOKEN: (\S+)", text)
            if m:
                return m.group(1)
        except FileNotFoundError:
            pass
        time.sleep(0.3)
    raise RuntimeError(f"Khong tim thay WEBUI TOKEN trong {log_path}")


def http(base, path, token, method="GET"):
    sep = "&" if "?" in path else "?"
    req = urllib.request.Request(f"{base}{path}{sep}t={token}", method=method)
    try:
        with urllib.request.urlopen(req, timeout=3.0) as r:
            return r.read()
    except urllib.error.HTTPError as e:
        body = e.read().decode(errors="replace")
        print(f"    !! HTTP {e.code} tren {path}: {body}")
        raise


def stats(base, token):
    return json.loads(http(base, "/stats", token, "GET"))


def arm(url):
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(url)
    m.wait_heartbeat(timeout=8)
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0)
    t_end = time.monotonic() + 4.0
    while time.monotonic() < t_end:
        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            m.close()
            return True
    m.close()
    return False


def main():
    url = sys.argv[1]
    web = f"http://127.0.0.1:{sys.argv[2]}"
    log_path = sys.argv[3]
    engage_s = float(sys.argv[4]) if len(sys.argv) > 4 else 6.0

    token = read_token(log_path)
    print(f"[0] token doc tu log: {token[:6]}...")

    print("[1] Arm qua MAVLink...")
    if not arm(url):
        print("    THAT BAI arm"); sys.exit(1)
    print("    armed=True")

    # arm() da dong ket noi rieng - doi 1.5s de follow.app het bi tranh
    # chap cong UDP 14540 va bat lai vai heartbeat sach
    print("    doi ket noi MAVLink cua app on dinh lai...")
    time.sleep(1.5)

    print("[2] Pick muc tieu gia lap (giua box)...")
    # box gia lap trong sitl_bench.py: (400,100,460,400) tren khung 640x480
    cx, cy = (400 + 460) / 2 / 640, (100 + 400) / 2 / 480
    http(web, f"/pick?x={cx}&y={cy}", token, "POST")
    time.sleep(0.5)
    st = stats(web, token)
    print(f"    locked={st['locked']}")

    print("[3] Giu ENGAGE (tu gui lai /engage neu con idle)...")
    stop = threading.Event()

    def beat():
        while not stop.wait(0.15):
            try:
                http(web, "/alive", token, "POST")
            except Exception:
                return
    t = threading.Thread(target=beat, daemon=True)
    http(web, "/alive", token, "POST")
    http(web, "/engage", token, "POST")
    t.start()

    t_end = time.monotonic() + engage_s
    last_state = None
    t_next_retry = time.monotonic() + 1.0
    while time.monotonic() < t_end:
        time.sleep(0.3)
        st = stats(web, token)
        if st["state"] != last_state or True:
            print(f"    t={engage_s-(t_end-time.monotonic()):4.1f}s  "
                  f"state={st['state']:<10} engaged={st['engaged']!s:<5} "
                  f"mode={st['mode']:<12} armed={st['armed']!s:<5} "
                  f"yaw={st['yaw']:+6.2f} bearing={st['bearing']:+6.1f} "
                  f"block={st['block']}")
            last_state = st["state"]
        # neu van idle (co the lan dau bi tu choi vi ly do thoang qua),
        # thu gui lai /engage - dung nhu nguoi van hanh se bam lai
        if st["state"] == "idle" and time.monotonic() > t_next_retry:
            http(web, "/engage", token, "POST")
            t_next_retry = time.monotonic() + 1.0

    print("[4] Nha ENGAGE...")
    stop.set()
    http(web, "/disengage", token, "POST")
    time.sleep(2.0)
    st = stats(web, token)
    print(f"    sau disengage: state={st['state']} mode={st['mode']} "
          f"engaged={st['engaged']}")


if __name__ == "__main__":
    main()
