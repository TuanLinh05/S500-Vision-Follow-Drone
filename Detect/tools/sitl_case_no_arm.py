"""Kich ban 2 (DroneFollowPX4.md muc 7.2): ENGAGE khi CHUA arm phai bi tu
choi, PX4 khong duoc vao OFFBOARD.

Chay:
    python -m tools.sitl_case_no_arm 8090 <duong_dan_log>
"""
import sys
import time

from tools.sitl_drive import http, read_token, stats


def main():
    web = f"http://127.0.0.1:{sys.argv[1]}"
    token = read_token(sys.argv[2])

    st0 = stats(web, token)
    print(f"[0] truoc test: armed={st0['armed']} mode={st0['mode']}")
    if st0["armed"]:
        print("[!] Drone dang armed - kich ban nay can armed=False. Dung lai.")
        sys.exit(2)

    cx, cy = (400 + 460) / 2 / 640, (100 + 400) / 2 / 480
    http(web, f"/pick?x={cx}&y={cy}", token, "POST")
    time.sleep(0.3)

    print("[1] Bam ENGAGE ma KHONG arm...")
    http(web, "/alive", token, "POST")
    http(web, "/engage", token, "POST")

    bad = False
    for i in range(10):
        time.sleep(0.3)
        http(web, "/alive", token, "POST")
        st = stats(web, token)
        print(f"    t={i*0.3:3.1f}s state={st['state']:<10} "
              f"engaged={st['engaged']!s:<5} mode={st['mode']:<10} "
              f"block={st['block']}")
        if st["mode"] == "OFFBOARD" or st["engaged"]:
            bad = True

    if bad:
        print("\n[FAIL] PX4 da vao OFFBOARD hoac engaged=True ma KHONG arm!")
        sys.exit(1)
    else:
        print("\n[PASS] ENGAGE bi tu choi dung nhu ky vong khi chua arm.")


if __name__ == "__main__":
    main()
