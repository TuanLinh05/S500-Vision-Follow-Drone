"""Kich ban 6 (DroneFollowPX4.md muc 7.2):
"Dang bam, dong tab trinh duyet -> phai disengage trong < 1.5s."

Mo phong: ENGAGE binh thuong roi NGUNG HAN goi /alive (khong goi
/disengage) - dung nhu trinh duyet bi dong dot ngot. Do thoi gian tu
luc ngung /alive toi luc engaged=False.

Chay:
    python -m tools.sitl_case_operator_loss 8090 <log>
"""
import sys
import time

from tools.sitl_drive import http, read_token, stats


def main():
    web = f"http://127.0.0.1:{sys.argv[1]}"
    token = read_token(sys.argv[2])

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
    print(f"    engaged=True mode={st['mode']} yaw={st['yaw']:+.2f}")

    print("[2] NGUNG HAN goi /alive tu bay gio (mo phong dong tab)...")
    t0 = time.monotonic()
    t_disengaged = None
    for i in range(20):
        time.sleep(0.15)
        st = stats(web, token)
        el = time.monotonic() - t0
        print(f"    t={el:4.2f}s engaged={st['engaged']!s:<5} "
              f"mode={st['mode']:<12} yaw={st['yaw']:+6.2f} "
              f"block={st['block']}")
        if not st["engaged"] and t_disengaged is None:
            t_disengaged = el

    print()
    if t_disengaged is not None:
        ok = t_disengaged < 1.5
        print(f"  Disengage sau {t_disengaged:.2f}s "
              f"(yeu cau < 1.5s): {'PASS' if ok else 'FAIL'}")
        print(f"  Mode cuoi: {st['mode']} "
              f"({'PASS - da ve LOITER' if st['mode'] == 'AUTO.LOITER' else 'FAIL'})")
        sys.exit(0 if (ok and st["mode"] == "AUTO.LOITER") else 1)
    else:
        print("[FAIL] Khong bao gio disengage!")
        sys.exit(1)


if __name__ == "__main__":
    main()
