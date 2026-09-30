"""Vong lap chinh cua ung dung drone follow PX4.

Lap rap tat ca module va dieu phoi:
  camera -> detect -> track -> target -> control -> safety -> px4
  web commands -> cmd queue -> vong dieu khien
  logging -> luong rieng (khong chan dieu khien)

KHONG tu arm, KHONG tu cat canh, chi dieu khien truc yaw.

v2.1 - cac lop chong bay mat kiem soat:
  - LoopWatchdog: luong doc lap, main loop treo -> ngat dieu khien that su
  - geofence mem: ban kinh / tran / san / toc do; canh bao -> zero yaw,
    vuot bien -> disengage
  - chuoi ENGAGE co tien kiem tra + phat zero setpoint truoc khi vao Offboard
  - doi mode chay tren luong rieng (khong chan vong dieu khien)
  - phat hien phi cong dong stick -> tra quyen ngay
  - gioi han thoi luong moi lan engage
  - CommandAudit: lenh yaw khong khop chuyen dong that -> disengage
  - disengage: loiter that bai -> tu dong rot xuong stream-off
"""

import argparse
import math
import queue
import signal
import sys
import threading
import time
from collections import deque

import cv2
import numpy as np

from .camera import Camera
from .control import YawConfig, YawController
from .detect import letterbox, load_model, postprocess
from .geofence import FenceConfig, evaluate as fence_eval, preflight as fence_pre
from .logging_sink import LoggingSink
from .px4 import PX4Link, SetpointStreamer, check_rc_map_deadman_conflict
from .safety import GateConfig, GateInputs, check as gate_check
from .target import TargetManager
from .track import ByteTrack, GMC
from .watchdog import AuditConfig, CommandAudit, LoopWatchdog, StickMonitor
from .webui import OperatorLink, start_web

_stop = threading.Event()

# trang thai chuoi ENGAGE
S_IDLE, S_PRESTREAM, S_SWITCHING, S_ENGAGED = "idle", "prestream", "switching", "engaged"


# ============================================================ draw HUD
def draw(img, tracks, tm, ctl, engaged, block, fps, fence=None, eng_left=None):
    """Ve thong tin len khung hinh de hien thi / stream."""
    h, w = img.shape[:2]
    cx = w // 2

    # deadzone
    if ctl.cfg.hfov_deg > 0:
        dzx = int((ctl.cfg.deadzone_deg / (ctl.cfg.hfov_deg / 2)) * (w / 2))
    else:
        dzx = 20
    cv2.line(img, (cx, 0), (cx, h), (70, 70, 70), 1)
    cv2.rectangle(img, (cx - dzx, 0), (cx + dzx, h), (60, 60, 60), 1)

    id2m = {m.tid: m for m in tm.locked if m.seen}
    pending2m = {m.pending_tid: m for m in tm.locked
                 if m.pending_tid is not None}
    for t in tracks:
        x1, y1, x2, y2 = t.box.astype(int)
        m = id2m.get(t.id)
        pending = m is None and t.id in pending2m
        if pending:
            m = pending2m[t.id]
        if m is None:
            cv2.rectangle(img, (x1, y1), (x2, y2), (105, 105, 105), 1)
            cv2.putText(img, f"ID {t.id}", (x1, max(y1 - 6, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (105, 105, 105), 1)
            continue
        pri = m.nhan == tm.primary and not pending
        color = (0, 220, 255) if pending else m.mau
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 4 if pri else 2)
        lb = m.nhan + (" [CHINH]" if pri else "")
        if pending or m.needs_confirm:
            lb += " ?XAC NHAN"
        cv2.putText(img, lb, (x1, max(y1 - 6, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        if pri:
            tx = int((t.box[0] + t.box[2]) / 2)
            cv2.line(img, (cx, h - 46), (tx, h - 46), (0, 255, 255), 2)
            cv2.circle(img, (tx, h - 46), 6, (0, 255, 255), -1)

    # muc tieu bi mat (last_box)
    for m in tm.locked:
        if not m.seen and m.last_box is not None:
            x1, y1, x2, y2 = m.last_box.astype(int)
            cv2.rectangle(img, (x1, y1), (x2, y2), m.mau, 1, cv2.LINE_4)

    # mui ten yaw
    if abs(ctl.yaw) > 0.15:
        L = int(min(abs(ctl.yaw) / max(ctl.cfg.max_rate, 1), 1.0) * (w * 0.28))
        y_arrow = h - 92
        cv2.arrowedLine(img, (cx, y_arrow),
                        (cx + (L if ctl.yaw > 0 else -L), y_arrow),
                        (0, 165, 255), 6, tipLength=0.3)

    # thanh trang thai (tren)
    band = (0, 0, 160) if engaged else (40, 40, 40)
    if fence is not None and fence.level == "breach":
        band = (0, 0, 255)
    elif fence is not None and fence.level == "warn":
        band = (0, 110, 190)
    cv2.rectangle(img, (0, 0), (w, 40), band, -1)
    txt = (f"DANG DIEU KHIEN   yaw {ctl.yaw:+.1f} do/s" if engaged
           else f"KHONG DIEU KHIEN   {block[:52]}")
    if engaged and eng_left is not None:
        txt += f"   con {eng_left:.0f}s"
    cv2.putText(img, txt, (12, 27), cv2.FONT_HERSHEY_DUPLEX, 0.7,
                (255, 255, 255) if engaged else (170, 170, 170), 2)

    # thanh thong tin (duoi)
    ds = " ".join(
        f"{m.nhan}{'*' if m.nhan == tm.primary else ''}"
        f"{'(mat)' if not m.seen else ''}" for m in tm.locked) or "chua khoa"
    line = f"{ds}   lech {ctl.bearing:+.0f} do   FPS {fps:.0f}"
    if fence is not None:
        line += (f"   fence {fence.dist_m:.1f}m  cao {fence.alt_m:.1f}m"
                 f"  v {fence.speed_ms:.1f}m/s")
    cv2.putText(img, line, (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.52,
                (175, 175, 175), 1)


# ============================================================ argparse
def build_args():
    ap = argparse.ArgumentParser(
        description="Drone PX4 xoay bam theo nguoi duoc chon")
    # nguon hinh
    g = ap.add_argument_group("Camera")
    g.add_argument("--camera", type=int, default=0)
    g.add_argument("--source", default=None)
    g.add_argument("--width", type=int, default=1280)
    g.add_argument("--height", type=int, default=720)
    g.add_argument("--fourcc", default="MJPG")
    g.add_argument("--cam-rot", type=int, default=0,
                   choices=[0, 90, 180, 270],
                   help="Xoay khung hinh (do, nguoc chieu kim dong ho) de bu "
                        "camera lap xoay. Vi du camera lap xoay 90 do theo "
                        "chieu kim dong ho -> dung --cam-rot 270. BAT BUOC "
                        "dat dung: YawController tinh lech tu truc NGANG, "
                        "camera xoay ma khong bu se ra lenh yaw SAI.")
    g.add_argument("--cam-buffer", type=int, default=2,
                   help="CAP_PROP_BUFFERSIZE. Mac dinh 2 - do thuc nghiem "
                        "tren UP7000/Rapoo C280: buffersize=1 giam gan mot "
                        "nua thong luong thuc te tren mot so driver V4L2. "
                        "0 = de OpenCV/driver tu chon (co the tang do tre)")
    # model
    g = ap.add_argument_group("Model")
    g.add_argument("--model", default="yolo26n_int8_openvino_model")
    g.add_argument("--device", default="GPU")
    g.add_argument("--imgsz", type=int, default=416)
    g.add_argument("--conf", type=float, default=0.4)
    g.add_argument("--min-box-h", type=float, default=24.0,
                   help="Chieu cao toi thieu cua box nguoi (px)")
    g.add_argument("--min-aspect", type=float, default=1.2)
    g.add_argument("--max-aspect", type=float, default=5.0)
    # dieu khien
    g = ap.add_argument_group("Dieu khien")
    g.add_argument("--hfov", type=float, default=90.0,
                   help="Goc nhin ngang camera (do)")
    g.add_argument("--fx-px", type=float, default=0.0,
                   help="Tieu cu px da hieu chuan. >0 se dung thay --hfov")
    g.add_argument("--gain", type=float, default=0.6,
                   help="He so ti le: yaw_rate = gain * sai_so_goc")
    g.add_argument("--k-ff", type=float, default=0.0,
                   help="He so feed-forward (0=tat)")
    g.add_argument("--deadzone", type=float, default=4.0, help="do")
    g.add_argument("--max-yaw-rate", type=float, default=15.0, help="do/s")
    g.add_argument("--slew", type=float, default=80.0, help="do/s^2")
    g.add_argument("--lead", type=float, default=0.0, help="Bu tre (giay)")
    g.add_argument("--min-edge-rate", type=float, default=8.0, help="do/s")
    g.add_argument("--invert-yaw", action="store_true")
    g.add_argument("--reacquire", type=float, default=4.0,
                   help="Giay mat dau truoc khi BO HAN muc tieu")
    g.add_argument("--ramp", type=float, default=1.0,
                   help="Giay tang dan tran yaw tu 0 sau khi ENGAGE")
    # MAVLink
    g = ap.add_argument_group("MAVLink")
    g.add_argument("--mavlink", default=None,
                   help="vd udp:127.0.0.1:14540 hoac /dev/ttyACM0. "
                        "Bo trong = khong ket noi (chay kho)")
    g.add_argument("--baud", type=int, default=57600)
    g.add_argument("--sp-rate", type=float, default=20.0, help="Hz setpoint")
    g.add_argument("--prestream", type=float, default=1.0,
                   help="Giay phat setpoint zero truoc khi xin Offboard")
    # an toan
    g = ap.add_argument_group("An toan")
    g.add_argument("--enable-control", action="store_true",
                   help="BAT dieu khien that. Khong co co nay -> chay kho.")
    g.add_argument("--rc-chan", type=int, default=0,
                   help="Kenh RC dead-man. Bat buoc khi --enable-control, "
                        "tru khi dung --allow-no-rc-deadman cho SITL/bench")
    g.add_argument("--rc-min", type=int, default=1500)
    g.add_argument("--allow-no-rc-deadman", action="store_true",
                   help="CHI SITL/bench: cho phep --enable-control ma khong "
                        "co RC dead-man. KHONG dung khi bay that.")
    g.add_argument("--bench-allow-conflicting-deadman", action="store_true",
                   help="CHI SITL/bench: cho phep --rc-chan trung mot "
                        "RC_MAP_* cua PX4 (vd RC_MAP_FLTMODE/RC_MAP_ARM_SW) "
                        "da biet la KHONG DOC LAP - ha dead-man se dong thoi "
                        "doi flight mode/kill/RTL. KHONG dung khi bay that.")
    g.add_argument("--batt-min", type=float, default=0.0,
                   help="Volt, 0 = khong kiem tra")
    g.add_argument("--allow-no-battery-check", action="store_true",
                   help="CHI SITL/bench: cho phep --enable-control ma khong "
                        "co --batt-min. KHONG dung khi bay that.")
    g.add_argument("--batt-min-pct", type=float, default=0.0,
                   help="Phan tram pin toi thieu, 0 = khong kiem tra")
    g.add_argument("--vision-timeout", type=float, default=0.6)
    g.add_argument("--loop-timeout", type=float, default=0.5,
                   help="Vong lap cham hon nguong nay -> chan dieu khien")
    g.add_argument("--loop-watchdog", type=float, default=1.0,
                   help="Main loop im lang lau hon nguong nay -> NGAT")
    g.add_argument("--pos-max-age", type=float, default=1.0,
                   help="Tuoi toi da cua LOCAL_POSITION_NED")
    g.add_argument("--max-drift", type=float, default=5.0,
                   help="Met lech khoi diem chot -> ngat. Day la LUOI DU "
                        "PHONG cho truong hop chay --no-fence; binh thuong "
                        "--fence-radius (co dai canh bao) se chan truoc")
    g.add_argument("--max-engage-s", type=float, default=20.0,
                   help="Thoi luong toi da moi lan ENGAGE. 0 = khong gioi han")
    g.add_argument("--max-tx-trips", type=int, default=1,
                   help="So lan TX watchdog trip truoc khi ngat. 0 = bo qua")
    g.add_argument("--on-abort", default="loiter",
                   choices=["loiter", "stream-off", "hold-only"],
                   help="Hanh dong khi disengage")
    g.add_argument("--position-hold", action="store_true", default=True,
                   help="Dung setpoint vi tri (fix A7)")
    g.add_argument("--no-idle-follow", action="store_true",
                   help="Khong phat setpoint giu vi tri khi chua ENGAGE")
    # hang rao mem
    g = ap.add_argument_group("Hang rao mem (soft fence)")
    g.add_argument("--no-fence", action="store_true",
                   help="TAT hang rao mem (khong khuyen nghi)")
    g.add_argument("--fence-radius", type=float, default=3.0,
                   help="Met, tinh tu diem chot luc ENGAGE. 0 = tat. "
                        "Bay yaw-only thi drone KHONG duoc dich chuyen, nen "
                        "de chat. Toi 0.75*R se canh bao (yaw ve 0) truoc "
                        "khi vuot han thi moi ngat")
    g.add_argument("--fence-home-radius", type=float, default=0.0,
                   help="Met, tinh tu goc LOCAL_NED (~diem cat canh). 0 = tat. "
                        "Dat theo ban do khu thu, vd 15-40m. Day la thu duy "
                        "nhat chan duoc viec ENGAGE LAI o cho moi rat xa")
    g.add_argument("--fence-alt-dev", type=float, default=3.0,
                   help="Met lech do cao cho phep so voi luc ENGAGE")
    g.add_argument("--fence-alt-max", type=float, default=10.0,
                   help="Tran tuyet doi so voi goc LOCAL_NED. 0 = tat")
    g.add_argument("--fence-alt-min", type=float, default=1.5,
                   help="San tuyet doi so voi goc LOCAL_NED. 0 = tat")
    g.add_argument("--fence-max-speed", type=float, default=1.5,
                   help="Toc do ngang toi da o che do yaw-only (m/s). 0 = tat")
    g.add_argument("--fence-predict", type=float, default=1.0,
                   help="Giay du doan vi tri de canh bao som")
    # phi cong gianh quyen + doi chieu lenh
    g = ap.add_argument_group("Phi cong / doi chieu lenh")
    g.add_argument("--stick-chans", default="1,2,3,4",
                   help="Kenh stick theo doi RC override. Rong = tat")
    g.add_argument("--stick-th", type=float, default=120.0,
                   help="Lech us so voi luc ENGAGE thi coi la phi cong can thiep")
    g.add_argument("--no-audit", action="store_true",
                   help="Tat doi chieu lenh yaw voi yawspeed thuc te")
    g.add_argument("--audit-tol", type=float, default=12.0, help="do/s")
    # dau ra
    g = ap.add_argument_group("Dau ra")
    g.add_argument("--log-csv", default="follow_px4.csv")
    g.add_argument("--save-video", default=None)
    g.add_argument("--port", type=int, default=8080)
    g.add_argument("--bind", default="127.0.0.1")
    g.add_argument("--jpeg-hz", type=float, default=12.0,
                   help="Tan so ma hoa JPEG cho web. Thap = vong lap nhanh hon")
    g.add_argument("--no-web", action="store_true")
    g.add_argument("--quiet-status", action="store_true",
                   help="Giam tan so dong trang thai console con "
                        "1 dong/giay (co ket thuc bang xuong dong, khong "
                        "dung \\r). Mac dinh TU DONG bat khi stdout khong "
                        "phai TTY (vd chay nen/ghi log) de tranh spam file "
                        "log o toc do vong lap.")
    return ap.parse_args()


# ============================================================ main
def main():
    args = build_args()

    print("=" * 72)
    print("  DRONE PX4 - XOAY BAM THEO NGUOI   v2.1")
    print(f"  Dieu khien that : "
          f"{'BAT' if args.enable_control else 'TAT (chay kho)'}")
    print(f"  MAVLink         : {args.mavlink or 'khong ket noi'}")
    print(f"  Dead-man RC     : "
          f"{('kenh ' + str(args.rc_chan) + ' >= ' + str(args.rc_min)) if args.rc_chan else 'TAT'}")
    print(f"  Gioi han yaw    : {args.max_yaw_rate} do/s, "
          f"slew {args.slew} do/s^2, ramp {args.ramp}s")
    print(f"  Vung chet       : {args.deadzone} do")
    if args.no_fence:
        print("  Hang rao mem    : TAT  <-- khong khuyen nghi")
    else:
        print(f"  Hang rao mem    : R={args.fence_radius}m"
              f"{' (canh bao tu ' + format(args.fence_radius * 0.75, '.1f') + 'm)' if args.fence_radius else ''}"
              f"  home R={args.fence_home_radius or 'tat'}  "
              f"cao {args.fence_alt_min}-{args.fence_alt_max}m  "
              f"lech {args.fence_alt_dev}m  v<{args.fence_max_speed}m/s")
        print(f"  Luoi du phong   : troi > {args.max_drift}m -> ngat")
    print("  Thoi luong engage: " +
          (f"{args.max_engage_s:.0f}s moi lan" if args.max_engage_s > 0
           else "khong gioi han"))
    print("  KHONG tu arm, KHONG tu cat canh, chi dieu khien truc yaw.")
    print("=" * 72)

    if args.enable_control and not args.rc_chan:
        if args.allow_no_rc_deadman:
            print("  [!] CANH BAO: bo RC dead-man theo co --allow-no-rc-"
                  "deadman (CHI SITL/bench, khong bay that)")
        else:
            print("  [!] SE TU CHOI ENGAGE: --enable-control can --rc-chan "
                  "hoac co acknowledge --allow-no-rc-deadman cho SITL")
    if args.enable_control and args.batt_min <= 0:
        if args.allow_no_battery_check:
            print("  [!] CANH BAO: bo kiem tra pin theo co "
                  "--allow-no-battery-check (CHI SITL/bench)")
        else:
            print("  [!] SE TU CHOI ENGAGE: --enable-control can "
                  "--batt-min > 0")
    if args.bind not in ("127.0.0.1", "localhost"):
        print(f"  [!] CANH BAO: web bind {args.bind} - chi dung tren mang tin cay")

    # ---- model
    compiled, xml = load_model(args.model, args.device, args.imgsz)
    req = compiled.create_infer_request()
    op = compiled.output(0)
    for _ in range(5):
        req.infer({0: np.zeros((1, args.imgsz, args.imgsz, 3), np.uint8)})
    print(f"[+] Model {xml} tren {args.device}")

    # ---- camera
    cam = Camera(args.camera, args.width, args.height,
                 args.fourcc, args.source, buffersize=args.cam_buffer,
                 rotate=args.cam_rot)
    print(f"[+] Camera {cam.width}x{cam.height} fourcc={cam.fourcc}"
          f"{f' (xoay {args.cam_rot} do)' if args.cam_rot else ''}")

    # ---- MAVLink (None = chay kho)
    link = None
    streamer = None
    streamer_started = False
    deadman_conflict = ""    # non-rong => preflight() se tu choi ENGAGE
    if args.mavlink:
        link = PX4Link(args.mavlink, args.baud)
        # Che do monitor (--mavlink nhung khong --enable-control) phai la
        # receive-only: khong tao/khong khoi dong bat ky luong TX setpoint nao.
        # Khi co cho phep dieu khien, van doi toan bo preflight PASS va operator
        # ENGAGE roi moi bat dau pre-stream. Nhu vay chi ket noi app khong the
        # vo tinh "prime" PX4 Offboard.
        if args.enable_control:
            streamer = SetpointStreamer(
                link, rate_hz=args.sp_rate,
                use_position_hold=args.position_hold,
                max_rate=args.max_yaw_rate, ramp_s=args.ramp,
                idle_follow=not args.no_idle_follow)
        print(f"[+] MAVLink {args.mavlink}")

        # Kiem tra MOT LAN luc khoi dong: --rc-chan co trung mot RC_MAP_*
        # cua PX4 khong (vd flight-mode switch). Neu trung, ha dead-man se
        # dong thoi lam PX4 doi mode/kill/RTL - khong con la mot lop DOC LAP
        # (xem HANDOFF_CLAUDE_UP7000_2026-08-28.md muc 9). Chi chay khi that
        # su can dead-man; block toi vai giay la chap nhan duoc vi day la
        # luc khoi dong, khong phai vong dieu khien.
        if args.enable_control and args.rc_chan:
            print(f"  [+] Dang doc RC_MAP_* tren PX4 de kiem tra xung dot "
                  f"voi --rc-chan {args.rc_chan} (vai giay)...")
            conflicts, unverified = check_rc_map_deadman_conflict(
                link, args.rc_chan)
            for c in conflicts:
                print(f"  [!] XUNG DOT DEAD-MAN: {c}")
            if conflicts and args.bench_allow_conflicting_deadman:
                print("  [!] CANH BAO: bo qua xung dot theo co "
                      "--bench-allow-conflicting-deadman (CHI SITL/bench, "
                      "KHONG dung khi bay that)")
            elif conflicts:
                deadman_conflict = "; ".join(conflicts)
                print("  [!] SE TU CHOI ENGAGE: RC dead-man khong doc lap "
                      "voi PX4 (dung --bench-allow-conflicting-deadman CHI "
                      "cho SITL/bench de bo qua)")
            if unverified:
                print(f"  [!] CANH BAO: khong doc duoc "
                      f"{', '.join(unverified)} trong thoi gian cho - PX4 "
                      f"co the khong ho tro hoac link cham. TU KIEM TRA "
                      f"RC_MAP_* thu cong truoc khi bay that.")

    # ---- operator link + web
    op_link = OperatorLink()
    cmd_q = queue.Queue(maxsize=32)

    # ---- logging
    log_sink = LoggingSink(
        args.log_csv,
        video_path=args.save_video,
        video_size=(cam.width, cam.height) if args.save_video else None)

    # ---- tracker / target / control
    tracker = ByteTrack()
    gmc = GMC()
    tm = TargetManager(reacquire=args.reacquire)
    ycfg = YawConfig(
        hfov_deg=args.hfov, fx_px=args.fx_px, gain=args.gain, k_ff=args.k_ff,
        deadzone_deg=args.deadzone, max_rate=args.max_yaw_rate,
        slew=args.slew, lead_s=args.lead, invert=args.invert_yaw,
        min_edge_rate=args.min_edge_rate)
    ctl = YawController(ycfg, cam.width)

    gcfg = GateConfig(
        enable_control=args.enable_control,
        rc_chan=args.rc_chan, rc_min=args.rc_min,
        batt_min=args.batt_min, batt_min_pct=args.batt_min_pct,
        vision_timeout=args.vision_timeout,
        loop_timeout=args.loop_timeout,
        pos_max_age=args.pos_max_age,
        max_drift_m=args.max_drift,
        max_engage_s=args.max_engage_s,
        max_tx_trips=args.max_tx_trips,
        require_rc_deadman=not args.allow_no_rc_deadman)

    fcfg = FenceConfig(
        enabled=not args.no_fence,
        radius_m=args.fence_radius,
        home_radius_m=args.fence_home_radius,
        alt_dev_m=args.fence_alt_dev,
        alt_max_m=args.fence_alt_max,
        alt_min_m=args.fence_alt_min,
        max_speed_ms=args.fence_max_speed,
        predict_s=args.fence_predict)

    audit = CommandAudit(AuditConfig(enabled=not args.no_audit,
                                     tol_deg_s=args.audit_tol))
    stick_chans = tuple(int(c) for c in args.stick_chans.split(",")
                        if c.strip().isdigit())
    sticks = StickMonitor(chans=stick_chans, threshold=args.stick_th)

    # ---- trang thai
    engaged = False
    eng_state = S_IDLE
    t_state = time.monotonic()
    t_engage = 0.0
    eng_result = queue.Queue()          # ket qua doi mode tu luong rieng
    sp_halted = False                   # luong setpoint da bi dung han
    reset_baseline = None               # ODOMETRY.reset_counter luc chot ENGAGE
    control_active = threading.Event()  # dong bo voi watchdog doc lap
    t_prev = time.monotonic()
    t_target = 0.0
    fps_win = deque(maxlen=30)
    block_truoc = None
    last_seq = -1
    web_jpeg = b""
    web_stats = {}
    t_jpeg = 0.0
    jpeg_dt = 1.0 / max(args.jpeg_hz, 0.1)
    fence = fence_eval(fcfg, None, None, None)
    # Non-TTY (chay nen, ghi log ra file/systemd) khong huong loi gi tu "\r"
    # o toc do vong lap (~20Hz) - moi lan ghi thanh mot dong rieng trong file
    # log, spam rat lon. Tu dong giam con 1 dong/giay khi phat hien khong
    # phai TTY, hoac ep bang --quiet-status.
    status_quiet = args.quiet_status or not sys.stdout.isatty()
    status_dt = 1.0 if status_quiet else 0.0
    t_status = 0.0

    # ---- ham dieu khien
    def on_loiter_done(ok):
        nonlocal sp_halted
        if ok and streamer and not streamer.halted:
            # Callback cu khong duoc pause mot ENGAGE moi da bat dau.
            if not engaged and eng_state == S_IDLE:
                streamer.pause()
        elif streamer and not streamer.halted:
            print("\n  !! KHONG vao duoc AUTO.LOITER -> DUNG phat setpoint, "
                  "de PX4 kich hoat Offboard-loss")
            streamer.halt()
            sp_halted = True

    def disengage(ly_do, force_abort=False):
        nonlocal engaged, eng_state, sp_halted, reset_baseline
        was = engaged or eng_state != S_IDLE
        if was:
            print(f"\n  !! NGAT DIEU KHIEN: {ly_do}")
        op_link.note = f"da ngat: {ly_do}" if was else op_link.note
        engaged = False
        eng_state = S_IDLE
        control_active.clear()
        ctl.yaw = 0.0
        audit.reset()
        sticks.disarm()
        reset_baseline = None
        if streamer:
            # Loop watchdog da yeu cau cat stream doc lap. Khong duoc gui
            # DO_SET_MODE o day: neu main vua hoi phuc sau khi bi ket, luong
            # MAVLink/mode co the dang khong khoe. PX4 se xu ly Offboard-loss.
            if streamer.emergency_requested:
                sp_halted = True
                op_link.note = ("da ngat khan cap: "
                                f"{streamer.emergency_reason or ly_do}")
                return
            streamer.command(0.0)
            abort_action = args.on_abort
            # `hold-only` chi la tuy chon co chu y cua operator. Khong bao gio
            # duoc giu Offboard sau mot loi FATAL (fence, RC, sensor, audit...).
            if force_abort and abort_action == "hold-only":
                abort_action = "loiter"
            if was and abort_action == "loiter" and link:
                if not link.enter_loiter_async(on_done=on_loiter_done):
                    # luong doi mode dang ban -> khong doi duoc, dung stream
                    print("\n  !! doi mode dang ban -> DUNG phat setpoint")
                    streamer.halt()
                    sp_halted = True
            elif was and abort_action == "stream-off":
                streamer.halt()
                sp_halted = True
            streamer.clear_hold()

    def preflight() -> str:
        """Tra ve "" neu du dieu kien ENGAGE, hoac ly do tu choi."""
        if not args.enable_control:
            return "chua bat --enable-control"
        if gcfg.require_rc_deadman and not args.rc_chan:
            return "chua cau hinh RC dead-man"
        if deadman_conflict:
            return f"RC dead-man xung dot voi PX4: {deadman_conflict}"
        if args.batt_min <= 0 and not args.allow_no_battery_check:
            return "chua cau hinh nguong dien ap pin --batt-min"
        if sp_halted or (streamer and streamer.halted):
            return "luong setpoint da dung - khoi dong lai chuong trinh"
        if streamer_started and not streamer.is_alive():
            return "luong setpoint khong chay - khoi dong lai chuong trinh"
        if link is None or streamer is None:
            return "chua ket noi MAVLink"
        if not link.connected or link.hb.age > gcfg.hb_timeout:
            return "mat heartbeat PX4"
        if not link.armed:
            return "drone chua arm"
        if link.failsafe:
            return "PX4 dang bao failsafe"
        if link.pos.value is None or link.pos.age > gcfg.pos_max_age:
            return "chua co vi tri LOCAL_NED moi"
        if link.xy_pos_health.value is not True:
            return "uoc luong vi tri XY khong healthy"
        if link.xy_pos_health.age > gcfg.estimator_max_age:
            return "du lieu suc khoe estimator cu"
        if link.vel.value is None or link.vel.age > gcfg.vel_max_age:
            return "chua co van toc LOCAL_NED moi"
        if link.att.value is None or link.att.age > gcfg.att_max_age:
            return "chua co ATTITUDE/yawspeed moi"
        if args.rc_chan:
            if link.rc.age > gcfg.rc_max_age:
                return "du lieu RC cu"
            if (link.rc.value or {}).get(args.rc_chan, 0) < args.rc_min:
                return f"cong tac RC ch{args.rc_chan} chua bat"
        if args.batt_min > 0:
            if link.batt.age > gcfg.batt_max_age:
                return "chua co du lieu pin moi"
            if (link.batt.value or 0) <= 0:
                return "du lieu dien ap pin khong hop le"
            if link.batt.value < args.batt_min:
                return f"pin thap {link.batt.value:.1f}V"
        if not tm.primary_ready:
            return "muc tieu chua san sang (can thay ro va da xac nhan)"
        why = fence_pre(fcfg, link.pos.value)
        if why:
            return why
        return ""

    def start_engage():
        """Buoc 1: chot diem giu, phat zero setpoint mot khoang truoc Offboard."""
        nonlocal eng_state, t_state, streamer_started, sp_halted, reset_baseline
        why = preflight()
        if why:
            print(f"\n  >>> TU CHOI ENGAGE: {why}")
            op_link.note = f"tu choi ENGAGE: {why}"
            return
        if not streamer_started:
            try:
                streamer.start()
                streamer_started = True
            except Exception as exc:
                sp_halted = True
                op_link.note = f"khong khoi dong duoc luong setpoint: {exc}"
                print(f"\n  >>> TU CHOI ENGAGE: {op_link.note}")
                return
        elif streamer.paused and not streamer.resume():
            sp_halted = True
            op_link.note = "khong resume duoc luong setpoint"
            print(f"\n  >>> TU CHOI ENGAGE: {op_link.note}")
            return
        streamer.command(0.0)
        streamer.arm_hold(link.pos.value)
        reset_baseline = link.reset_counter.value  # co the la None - xem below
        control_active.set()
        sticks.arm(link.rc.value or {})
        audit.reset()
        eng_state = S_PRESTREAM
        t_state = time.monotonic()
        op_link.note = "dang phat setpoint zero truoc khi vao Offboard"
        print(f"\n  >>> ENGAGE buoc 1: giu {tuple(round(v, 1) for v in link.pos.value)}"
              f", phat zero {args.prestream:.1f}s")

    def abort_engage(ly_do):
        nonlocal eng_state
        if eng_state in (S_PRESTREAM, S_SWITCHING):
            print(f"\n  >>> HUY ENGAGE: {ly_do}")
            op_link.note = f"huy ENGAGE: {ly_do}"
        disengage(ly_do)

    # ---- signal: chi dat co, KHONG in / KHONG lay lock (fix D1)
    def sig_h(*_):
        _stop.set()
    signal.signal(signal.SIGINT, sig_h)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, sig_h)

    # ---- watchdog vong lap (lop thu hai cua fix A1)
    def on_loop_watchdog(age):
        """Duong abort khan cap khong phu thuoc main loop co hoi phuc."""
        if not control_active.is_set():
            return
        op_link.request_stop()  # de main ghi thong bao neu no hoi phuc
        if streamer is not None:
            streamer.emergency_stop(
                f"loop watchdog: main loop im {age:.2f}s")

    loop_wd = LoopWatchdog(timeout=args.loop_watchdog,
                           on_trip=on_loop_watchdog)
    loop_wd.start()

    # ---- web stats/jpeg callbacks
    def get_jpeg():
        return web_jpeg

    def get_stats():
        return web_stats

    srv = None
    if not args.no_web:
        srv = start_web(args.port, op_link, get_stats, get_jpeg,
                        cmd_q, bind_host=args.bind)
        print(f"[+] Web: http://{args.bind}:{args.port}/?t={op_link.token}")

    print("[+] San sang. Ctrl+C de dung.\n")

    # ============================================================ loop
    try:
        while not _stop.is_set():
            loop_wd.beat()

            # ---- uu tien: stop latch (fix B8)
            if op_link.stop_latch.is_set():
                op_link.stop_latch.clear()
                while not cmd_q.empty():        # bo moi lenh dang cho
                    try:
                        cmd_q.get_nowait()
                    except queue.Empty:
                        break
                disengage("nguoi van hanh dung")

            # ---- camera + skip frame trung (fix C5)
            frame, seq, t_cap = cam.read()
            if frame is None or seq == last_seq:
                # CO Y KHONG goi streamer.command() o day: khong co khung moi
                # thi khong co so do moi. De lenh cu tu het han qua TX
                # watchdog (0.25s) thay vi nuoi lai gia tri cu.
                time.sleep(0.002)
                if eng_state != S_IDLE and cam.age > gcfg.frame_timeout:
                    disengage("mat camera", force_abort=True)
                continue
            last_seq = seq

            # ---- GMC + inference
            shift = gmc.estimate(frame)
            lb, r, dw, dh = letterbox(frame, args.imgsz)
            t_inf = time.monotonic()
            req.start_async({0: lb[None]})
            req.wait()
            infer_ms = (time.monotonic() - t_inf) * 1000.0
            det = req.get_tensor(op).data[0]
            H, W = frame.shape[:2]
            boxes, scores = postprocess(
                det, r, dw, dh, H, W, args.conf,
                min_box_h=args.min_box_h, min_aspect=args.min_aspect,
                max_aspect=args.max_aspect)
            tracks = tracker.update(boxes, scores, shift)

            # ---- lenh tu web
            while not cmd_q.empty():
                try:
                    cmd = cmd_q.get_nowait()
                except queue.Empty:
                    break
                ct = cmd.get("type", "")
                if ct == "engage":
                    if eng_state == S_IDLE:
                        start_engage()
                elif ct == "pick":
                    tm.pick_at(cmd["x"], cmd["y"], tracks, W, H, frame)
                    op_link.note = tm.msg
                    print(f"\n  >>> {tm.msg}")
                elif ct == "unlock":
                    tm.unlock_all()
                    op_link.note = tm.msg
                    if eng_state != S_IDLE:
                        disengage("bo khoa muc tieu")

            # ---- cap nhat muc tieu
            now = time.monotonic()
            target = tm.update(tracks, W, H, frame, now=now)
            if target is not None:
                t_target = now
            target_age = (now - t_target) if t_target else 999.0

            dt = now - t_prev
            t_prev = now

            # ---- hang rao mem
            cur_pos = link.pos.value if link else None
            cur_vel = link.vel.value if link else None
            anchor = streamer.hold if streamer else None
            fence = fence_eval(fcfg, anchor, cur_pos, cur_vel)

            # ---- phi cong dong stick?
            override, ov_why = sticks.check(link.rc.value if link else None)

            # ---- doi chieu lenh voi chuyen dong that
            yawspeed_deg = 0.0
            if link and link.att.value:
                yawspeed_deg = math.degrees(link.att.value[3])
            audit_bad = audit.update(dt, ctl.yaw, yawspeed_deg, engaged)

            # ---- chuoi ENGAGE khong chan vong lap
            if eng_state == S_PRESTREAM:
                streamer.command(0.0)
                if op_link.age > gcfg.operator_timeout:
                    abort_engage("mat nhip nguoi van hanh")
                elif (why := preflight()):
                    abort_engage(why)
                elif now - t_state >= args.prestream:
                    print("  >>> ENGAGE buoc 2: xin vao OFFBOARD")
                    op_link.note = "dang xin vao OFFBOARD"
                    if link.enter_offboard_async(
                            on_done=lambda ok: eng_result.put(ok)):
                        eng_state = S_SWITCHING
                        t_state = now
                    else:
                        abort_engage("luong doi mode dang ban")
            elif eng_state == S_SWITCHING:
                streamer.command(0.0)
                if op_link.age > gcfg.operator_timeout:
                    abort_engage("mat nhip nguoi van hanh")
                else:
                    try:
                        ok = eng_result.get_nowait()
                    except queue.Empty:
                        ok = None
                    if ok is True:
                        engaged = True
                        eng_state = S_ENGAGED
                        t_engage = now
                        streamer.start_ramp()
                        op_link.note = "DANG DIEU KHIEN"
                        print(f"  >>> ENGAGE buoc 3: OFFBOARD xac nhan, "
                              f"ramp {args.ramp:.1f}s")
                    elif ok is False:
                        abort_engage(f"PX4 tu choi OFFBOARD | "
                                     f"{link.status_text}")

            engage_s = (now - t_engage) if engaged else 0.0

            # ---- safety gate
            drift = (streamer.drift_from_hold(cur_pos)
                     if (streamer and cur_pos) else 0.0)
            # EKF reset trong khi giu ENGAGE: reset_baseline chi duoc chot
            # trong start_engage(); neu PX4 chua tung phat ODOMETRY thi ca
            # hai gia tri deu None va reset_delta o lai 0 (khong suy dien).
            # % 256 xu ly wraparound cua reset_counter (uint8).
            reset_delta = 0
            if (reset_baseline is not None and link is not None
                    and link.reset_counter.value is not None):
                reset_delta = (int(link.reset_counter.value)
                               - int(reset_baseline)) % 256
            pm = tm.primary_target
            gate_in = GateInputs(
                engaged=engaged,
                operator_age=op_link.age,
                link_connected=(link is not None and link.connected),
                hb_age=link.hb.age if link else 1e9,
                armed=link.armed if link else False,
                mode=link.mode if link else "?",
                pos_valid=(link is not None and link.pos.value is not None),
                pos_age=link.pos.age if link else 1e9,
                vel_age=link.vel.age if link else 1e9,
                att_age=link.att.age if link else 1e9,
                xy_pos_healthy=(link is not None and
                                link.xy_pos_health.value is True),
                estimator_age=(link.xy_pos_health.age if link else 1e9),
                drift_m=drift,
                reset_counter_delta=reset_delta,
                rc_value=(link.rc.value.get(args.rc_chan, 0)
                          if (link and link.rc.value) else 0),
                rc_age=link.rc.age if link else 1e9,
                batt_v=(link.batt.value if (link and link.batt.value) else 0.0),
                batt_age=link.batt.age if link else 1e9,
                batt_pct=(link.batt_pct.value if link else -1.0),
                batt_pct_age=link.batt_pct.age if link else 1e9,
                cam_age=now - t_cap,
                has_target=(pm is not None),
                target_age=target_age,
                target_needs_confirm=any(m.needs_confirm for m in tm.locked),
                loop_age=dt,
                tx_watchdog_trips=(streamer.watchdog_trips if streamer else 0),
                fence_level=fence.level,
                fence_reason=fence.reason,
                rc_override=override,
                engage_s=engage_s,
                px4_failsafe=(link.failsafe if link else False),
                # lenh cua chu ky TRUOC: neu NaN lot qua duoc thi phai ngat
                cmd_finite=math.isfinite(ctl.yaw),
                audit_bad=audit_bad,
                audit_reason=audit.reason,
                emergency_stop=(streamer.emergency_requested
                                if streamer else False),
                emergency_reason=(streamer.emergency_reason
                                  if streamer else ""))
            gr = gate_check(gcfg, gate_in)

            if not gr.allow and gr.fatal:
                if engaged:
                    disengage(gr.reason, force_abort=True)
                elif eng_state in (S_PRESTREAM, S_SWITCHING):
                    abort_engage(gr.reason)
            if override and eng_state == S_IDLE and ov_why:
                op_link.note = ov_why
            block = gr.reason

            # ---- yaw controller
            hold_yaw = tm.just_reacquired(now)   # vua tim lai -> tam dung
            allow_yaw = gr.allow and engaged and not hold_yaw
            yaw = ctl.update(target, dt, allow_yaw, yawspeed_deg)
            if hold_yaw and block == "":
                block = "vua tim lai muc tieu - tam dung yaw"
            if streamer:
                streamer.command(yaw if allow_yaw else 0.0)

            # ---- FPS
            fps_win.append(now)
            fps = ((len(fps_win) - 1) / (fps_win[-1] - fps_win[0])
                   if len(fps_win) > 1 else 0.0)

            # ---- thong bao chan moi
            if block != block_truoc:
                block_truoc = block
                if block:
                    print(f"\n  [chan] {block}")

            # ---- visualization (chi khi can) + web + log
            eng_left = (args.max_engage_s - engage_s
                        if (engaged and args.max_engage_s > 0) else None)
            need_video = bool(args.save_video)
            need_jpeg = (not args.no_web) and (now - t_jpeg >= jpeg_dt)
            vis = None
            if need_video or need_jpeg:
                vis = frame.copy()
                draw(vis, tracks, tm, ctl, engaged, block, fps, fence, eng_left)
            if need_jpeg and vis is not None:
                ok_e, buf = cv2.imencode(
                    ".jpg", vis, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
                if ok_e:
                    web_jpeg = buf.tobytes()
                t_jpeg = now

            web_stats = {
                "state": eng_state,
                "engaged": engaged,
                "can_control": args.enable_control,
                "block": block,
                "note": op_link.note,
                "yaw": round(yaw, 2),
                "bearing": round(ctl.bearing, 1),
                "target": pm.nhan if pm else "chua khoa ai",
                "locked": [{
                    "nhan": m.nhan, "id": int(m.tid),
                    "pri": m.nhan == tm.primary,
                "seen": m.seen, "confirm": m.needs_confirm,
                "pending_id": (int(m.pending_tid)
                               if m.pending_tid is not None else None),
                    "rgb": [m.mau[2], m.mau[1], m.mau[0]]
                } for m in tm.locked],
                "mav": (args.mavlink or "-") if link else "-",
                "mode": link.mode if link else "-",
                "mode_req": link.mode_req if link else "",
                "xy_pos_healthy": (link.xy_pos_health.value is True
                                   if link else False),
                "estimator_age": (round(link.xy_pos_health.age, 2)
                                  if link else 1e9),
                "armed": link.armed if link else False,
                "failsafe": (link.failsafe if link else False),
                "batt": round(link.batt.value, 2) if link else 0.0,
                "batt_pct": (link.batt_pct.value if link else -1.0),
                "rc_chan": args.rc_chan,
                "rc_val": (link.rc.value.get(args.rc_chan, 0)
                           if (link and link.rc.value) else 0),
                "fence": fence.level,
                "fence_why": fence.reason,
                "dist_m": round(fence.dist_m, 1),
                "alt_m": round(fence.alt_m, 1),
                "speed_ms": round(fence.speed_ms, 2),
                "drift_m": round(drift, 2),
                "ekf_reset_cnt": (link.reset_counter.value if link else None),
                "ekf_reset_delta": reset_delta,
                "engage_left": (round(eng_left, 1)
                                if eng_left is not None else None),
                "sp_hz": round(streamer.sp_hz, 1) if streamer else 0.0,
                "sp_paused": bool(streamer.paused) if streamer else False,
                "sp_halted": sp_halted,
                "wd_trips": streamer.watchdog_trips if streamer else 0,
                "emergency_stops": streamer.emergency_stops if streamer else 0,
                "emergency": (streamer.emergency_reason if streamer and
                              streamer.emergency_requested else ""),
                "loop_wd": loop_wd.trips,
                "log_drops": log_sink.dropped,
                "log_errors": log_sink.errors,
                "infer_ms": round(infer_ms, 1),
                "fps": round(fps, 1)}

            latency = (time.monotonic() - t_cap) * 1000
            log_sink.push({
                "state": eng_state,
                "engaged": int(engaged),
                "block": block,
                "muc_tieu": pm.nhan if pm else "",
                "n_khoa": len(tm.locked),
                "lech_goc_do": round(ctl.bearing, 2),
                "yaw_cmd_deg_s": round(yaw, 2),
                "yaw_cmd_rad_s": round(math.radians(yaw), 4),
                "yaw_thuc_deg_s": round(yawspeed_deg, 2),
                "mode": link.mode if link else "",
                "armed": int(link.armed) if link else "",
                "batt_v": (round(link.batt.value, 2) if link else ""),
                "batt_pct": (link.batt_pct.value if link else ""),
                "rc_val": (link.rc.value.get(args.rc_chan, 0)
                           if (link and args.rc_chan and link.rc.value) else ""),
                "rc_override": int(override),
                "fence": fence.level,
                "fence_why": fence.reason,
                "dist_m": round(fence.dist_m, 2),
                "home_m": round(fence.home_m, 2),
                "alt_m": round(fence.alt_m, 2),
                "speed_ms": round(fence.speed_ms, 2),
                "drift_m": round(drift, 2),
                "ekf_reset_cnt": (link.reset_counter.value if link else ""),
                "ekf_reset_delta": reset_delta,
                "engage_s": round(engage_s, 1),
                "sp_hz": (round(streamer.sp_hz, 1) if streamer else ""),
                "fps": round(fps, 1),
                "infer_ms": round(infer_ms, 1),
                "latency_ms": round(latency, 1),
                "at_edge": int(ctl.at_edge),
                "watchdog_trips": (streamer.watchdog_trips if streamer else 0),
                "emergency_stops": (streamer.emergency_stops if streamer else 0),
                "loop_wd_trips": loop_wd.trips,
                "nan_blocks": (streamer.nan_blocks if streamer else 0),
                "clamp_hits": (streamer.clamp_hits if streamer else 0),
                "log_drops": log_sink.dropped,
                "log_errors": log_sink.errors,
            }, vis if need_video else None)

            # ---- console status (giam tan so neu --quiet-status / non-TTY)
            if now - t_status >= status_dt:
                t_status = now
                line = (f"{eng_state:<9} | "
                        f"{(pm.nhan if pm else '--'):>3} | "
                        f"lech {ctl.bearing:+6.1f} do | "
                        f"yaw {yaw:+6.1f} do/s | "
                        f"{fence.level:<6} | {block[:32]:<32}")
                if status_quiet:
                    print("  " + line)
                else:
                    sys.stdout.write("\r  " + line)
                    sys.stdout.flush()

    except KeyboardInterrupt:
        pass
    finally:
        # Thoat DONG BO, khong dung disengage() (vi no doi mode tren luong
        # rieng - se dua den hai lenh DO_SET_MODE chay chong nhau luc thoat).
        _stop.set()
        loop_wd.halt()
        control_active.clear()
        engaged = False
        eng_state = S_IDLE
        ctl.yaw = 0.0
        if streamer and streamer_started and not streamer.halted:
            streamer.command(0.0)
            streamer.clear_hold()
            time.sleep(0.3)          # de PX4 nhan vai setpoint yaw = 0
            if link and args.on_abort == "loiter":
                try:
                    if not link.enter_loiter():
                        print("\n  !! khong vao duoc LOITER khi thoat - "
                              "dung phat setpoint de PX4 tu failsafe")
                except Exception:
                    pass
            streamer.halt()
        elif streamer and not streamer.halted:
            # Chua ENGAGE => thread chua tung start, khong gui zero setpoint
            # va khong doi mode cua PX4 khi app thoat.
            streamer.halt()
        if link:
            link.close()
        if srv is not None:
            try:
                srv.shutdown()
            except Exception:
                pass
        cam.release()
        log_sink.stop()
        print(f"\n\n  Da dung. Log: {args.log_csv}")
        print("  Kiem tra cot 'block' cho biet vi sao bi chan,")
        print("  cot 'sp_hz' phai luon >18 Hz trong suot engaged,")
        print("  cot 'fence' phai la 'ok' trong suot chuyen bay.")


if __name__ == "__main__":
    main()
