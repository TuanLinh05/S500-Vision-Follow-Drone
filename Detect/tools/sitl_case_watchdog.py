"""Kich ban 3 (DroneFollowPX4.md muc 7.2):
"Dang bam, kill -STOP tien trinh 2 giay roi kill -CONT -> drone phai da
dung xoay (watchdog TX)."

Yeu cau: da arm san, chua engage (script tu pick + engage).

Chay:
    python -m tools.sitl_case_watchdog 8090 <log> <pid_python>
"""
import os
import signal
import sys
import time

from tools.sitl_drive import http, read_token, stats


def main():
    web = f"http://127.0.0.1:{sys.argv[1]}"
    token = read_token(sys.argv[2])
    pid = int(sys.argv[3])

    print("    doi ket noi MAVLink cua app on dinh lai sau arm...")
    time.sleep(1.5)

    cx, cy = (400 + 460) / 2 / 640, (100 + 400) / 2 / 480
    http(web, f"/pick?x={cx}&y={cy}", token, "POST")
    time.sleep(0.3)

    print("[1] ENGAGE (tu gui lai neu con idle)...")
    http(web, "/alive", token, "POST")
    http(web, "/engage", token, "POST")

    engaged = False
    t_next_retry = time.monotonic() + 1.0
    for _ in range(30):
        http(web, "/alive", token, "POST")
        time.sleep(0.2)
        st = stats(web, token)
        if st["engaged"]:
            engaged = True
            break
        if st["state"] == "idle" and time.monotonic() > t_next_retry:
            http(web, "/engage", token, "POST")
            t_next_retry = time.monotonic() + 1.0
    if not engaged:
        print(f"[!] Khong ENGAGE duoc: {st}")
        sys.exit(2)
    print(f"    engaged=True mode={st['mode']} yaw={st['yaw']:+.2f} "
          f"wd_trips_truoc={st['wd_trips']}")
    wd_before = st["wd_trips"]

    print(f"[2] kill -STOP pid={pid} trong 2.0s (khong /alive trong luc nay)...")
    os.kill(pid, signal.SIGSTOP)
    t_stop = time.monotonic()
    time.sleep(2.0)
    os.kill(pid, signal.SIGCONT)
    print(f"    kill -CONT sau {time.monotonic()-t_stop:.2f}s")

    print("[3] Theo doi ngay sau khi hoi phuc (khong gui /alive nua - "
          "operator cung coi nhu da mat nhip trong luc STOP)...")
    for i in range(15):
        time.sleep(0.3)
        try:
            st = stats(web, token)
        except Exception as e:
            print(f"    loi doc stats: {e}")
            continue
        print(f"    t={i*0.3:3.1f}s state={st['state']:<10} "
              f"engaged={st['engaged']!s:<5} mode={st['mode']:<12} "
              f"yaw={st['yaw']:+6.2f} wd_trips={st['wd_trips']} "
              f"block={st['block']}")

    ok_yaw = abs(st["yaw"]) < 0.5
    ok_wd = st["wd_trips"] > wd_before
    ok_disengaged = not st["engaged"]
    print()
    print(f"  yaw ve 0?              {'PASS' if ok_yaw else 'FAIL'} "
          f"(yaw cuoi={st['yaw']:+.2f})")
    print(f"  watchdog_trips tang?   {'PASS' if ok_wd else 'FAIL'} "
          f"({wd_before} -> {st['wd_trips']})")
    print(f"  da disengage?          {'PASS' if ok_disengaged else 'FAIL'}")
    if ok_yaw and ok_wd:
        print("\n[PASS] TX watchdog hoat dong dung: yaw ve 0 sau khi "
              "main loop 'treo'.")
    else:
        print("\n[FAIL] Kiem tra lai.")
        sys.exit(1)


if __name__ == "__main__":
    main()
