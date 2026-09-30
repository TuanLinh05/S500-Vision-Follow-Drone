# 🎯 S500 Vision Follow Drone

![PX4](https://img.shields.io/badge/PX4-v1.17-0B3D91)
![Pixhawk](https://img.shields.io/badge/FC-Pixhawk%206C-455A64)
![Companion](https://img.shields.io/badge/Companion-UP%207000-6A1B9A)
![OpenVINO](https://img.shields.io/badge/Vision-YOLO%20%2B%20OpenVINO-0071C5)
![Gazebo](https://img.shields.io/badge/Sim-Gazebo%20%2B%20PX4%20SITL-F58113)
![Python](https://img.shields.io/badge/Python-3.10%2B-3776AB?logo=python&logoColor=white)
![Tests](https://img.shields.io/badge/tests-217%20passing-2E7D32)

<a id="english"></a>**🇬🇧 English** · [🇻🇳 Tiếng Việt](#tieng-viet)

An **S500 quadcopter** that detects a selected person with an onboard camera and **rotates (yaw) to keep them in view**.
A **UP 7000** companion computer runs YOLO person detection with **OpenVINO**, tracks the chosen target, and sends yaw-only setpoints to a **Pixhawk 6C / PX4** over MAVLink Offboard. Every step is wrapped in independent safety layers.
The repository also contains a telemetry/safety companion package and a **Gazebo + PX4 SITL** model of the S500 for testing without hardware.

> [!WARNING]
> The software **never arms and never takes off by itself**, and controls **only the yaw axis**. The pilot must take off and hover in Position mode before enabling the AI.
> The Position/RTL switch on the RC transmitter is always the **last safety layer**. Test with propellers removed first.

---

## ✨ Highlights

- **Person detection & tracking:** OpenVINO YOLO with pre-filtering, GMC + ByteTrack, and HSV upper-body appearance matching to keep the selected person.
- **Yaw-only follow control:** a pure `YawController` (no I/O) with ramp, deadzone, feed-forward and rate clamp. Camera mounting rotation (`--cam-rot`) and FOV (`--hfov` / `--fx-px`) are compensated.
- **9 independent anti-flyaway layers:** TX watchdog, loop watchdog with emergency stream-off, soft geofence, pilot stick detection, command audit (commanded vs actual yaw rate), engage time limit, NaN/clamp guard, identity confirmation, and EKF-reset detection.
- **Preflight gates:** refuses to ENGAGE unless armed, position is fresh, no failsafe, RC dead-man is on, battery is OK, a target is selected and the drone is inside the fence. It also detects RC dead-man channel conflicts with PX4 `RC_MAP_*`.
- **Web UI with token + hold-to-engage**, and CSV/video logging of every flight for post-flight review.
- **Companion package (`s500-companion`):** USB Pixhawk discovery, read-only MAVLink telemetry recorder and validator, parameter audit, safety supervisor / flight guard, and a disarmed-only mode-switch test on the real Pixhawk.
- **Gazebo S500 model:** CAD-based visual mesh, MTF-01 optical-flow camera and rangefinder, IMU/GPS/baro/mag, attached to PX4 SITL (airframe 4001).
- **217 automated tests** (186 Detect + 31 Companion), none of which need hardware.

## 🔧 Hardware

| Block | Part |
|---|---|
| Frame | S500 |
| Flight controller | Pixhawk 6C, PX4 v1.17 |
| Companion computer | UP 7000 (32 GB / 4 GB) |
| Camera | Rapoo C280 (USB) |
| Motors | SunnySky X2216 950 kV |
| RC | FlySky FS-i6 + FS-iA6B |
| Power | PM07, Ovonic 4S 6200 mAh |
| Optical flow | MTF-01 (flow + rangefinder) |

## 🏗️ Architecture

```text
┌──────────────┐  frames  ┌─────────────────────────── UP 7000 ───────────────────────────┐
│ USB camera   │ ───────▶ │ detect (YOLO/OpenVINO) → track (ByteTrack) → target lock      │
└──────────────┘          │        → YawController → SafetyGate (9 layers) → SetpointStreamer │
                          │ Web UI (token, hold-to-engage)  ·  CSV / video logging        │
                          └───────────────────────────────┬───────────────────────────────┘
                                                          │ MAVLink (USB), yaw-only Offboard
                                                          ▼
┌──────────────┐   RC (Position/RTL, dead-man)   ┌──────────────────────┐
│ FS-i6 + iA6B │ ──────────────────────────────▶ │ Pixhawk 6C · PX4     │ ──▶ S500 motors
└──────────────┘                                 └──────────────────────┘
```

## 📂 Repository structure

```text
├── Detect/                  # Person-follow app v2.4 (follow/ package, 186 tests, SITL/bench tools)
├── Companion/               # s500-companion: telemetry, safety supervisor, SITL yaw runner (31 tests)
├── Simulation/gazebo/       # S500 Gazebo model + world, PX4 SITL launchers (WSL)
├── Plan/                    # Design plans, reviews, benchmark requirements (Vietnamese)
└── OpticalFlow_Hover_Test.params   # PX4 param profile: slow, low-altitude optical-flow hover
```

## 🚀 Getting started

### Detect – person follow

```bash
cd Detect
pip install -r requirements.txt

# Dry run: camera + detection only, no MAVLink, no control
python -m follow.app --camera 0 --model yolo26n_int8_openvino_model

# PX4 SITL
python -m follow.app --mavlink udp:127.0.0.1:14540 --enable-control --rc-chan 7

# Tests (no hardware, no OpenVINO needed)
python -m pytest tests/ -v
```

The OpenVINO model folder is not included; export it locally with Ultralytics. Real-flight command lines, all parameters, the CSV columns to review after each flight and the camera-rotation notes are in [`Detect/README.md`](Detect/README.md).

### Companion – telemetry & safety

```bash
cd Companion
bash deploy/install_up7000.sh           # on the UP 7000 (Ubuntu 22.04, Python 3.10+)
. .venv/bin/activate
s500-companion discover
s500-companion probe --duration 10      # read-only MAVLink check
python -m unittest discover -s tests -v
```

Control on the real Pixhawk is locked by default (`control_enabled=false`, `allow_serial_control=false`). The step-by-step unlock sequence is in [`Companion/README.md`](Companion/README.md).

### Simulation – Gazebo + PX4 SITL (WSL)

```bash
bash Simulation/gazebo/run_s500_gazebo_wsl.sh     # terminal 1: Gazebo with the S500 model
bash Simulation/gazebo/run_s500_px4_sitl_wsl.sh   # terminal 2: PX4 SITL (airframe 4001), stays disarmed
```

Home is set to **HCM University of Technology, 268 Lý Thường Kiệt, District 10, Ho Chi Minh City** (`10.772100, 106.657900`). Details and QGroundControl setup: [`Simulation/gazebo/README.md`](Simulation/gazebo/README.md).

## 🛡️ Before flying the real drone

1. Unit tests pass on the actual UP 7000.
2. SITL fault tests are logged (process kill, stream loss, stale target, RC dead-man).
3. Bench test on the real Pixhawk with **propellers removed**: Position / RTL / RC override / geofence confirmed.
4. PX4 Offboard-loss (`COM_OF_LOSS_T`), hard geofence (`GF_*`) and the RC Position/RTL switch are configured. These three layers are outside this code and are mandatory.
5. First flights: `--max-yaw-rate 15`, small fence, short `--max-engage-s`.

## 📚 Credits

- [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot), [pymavlink](https://github.com/ArduPilot/pymavlink), [OpenVINO](https://github.com/openvinotoolkit/openvino), Ultralytics YOLO, ByteTrack.
- The S500 visual mesh (`Simulation/gazebo/models/s500_quad_x/meshes/s500_frame_full.stl`) was exported from a third-party S500 frame CAD assembly, which is not included in this repository.

---

<a id="tieng-viet"></a>

## 🇻🇳 Tiếng Việt

[🇬🇧 English](#english) · **🇻🇳 Tiếng Việt**

Drone **S500** nhận diện một người được chọn qua camera và **xoay (yaw) để luôn giữ người đó trong khung hình**.
Máy tính đồng hành **UP 7000** chạy nhận diện người bằng YOLO trên **OpenVINO**, bám mục tiêu đã chọn và gửi setpoint chỉ-yaw tới **Pixhawk 6C / PX4** qua MAVLink Offboard. Mọi bước đều được bọc bởi các lớp an toàn độc lập.
Repo còn có gói companion cho telemetry/an toàn và mô hình **Gazebo + PX4 SITL** của S500 để thử nghiệm không cần phần cứng.

> [!WARNING]
> Phần mềm **không tự arm, không tự cất cánh**, chỉ điều khiển **trục yaw**. Phi công phải cất cánh và hover Position trước khi bật AI.
> Công tắc Position/RTL trên tay điều khiển luôn là **lớp an toàn cuối cùng**. Luôn thử với cánh quạt đã tháo trước.

### ✨ Điểm nổi bật

- **Nhận diện và bám người:** YOLO trên OpenVINO có lọc trước, GMC + ByteTrack, so khớp ngoại hình thân trên bằng histogram HSV để giữ đúng người đã chọn.
- **Điều khiển chỉ-yaw:** `YawController` thuần (không I/O) có ramp, deadzone, feed-forward và giới hạn tốc độ. Có bù camera lắp xoay (`--cam-rot`) và góc nhìn (`--hfov` / `--fx-px`).
- **9 lớp chống bay mất kiểm soát độc lập:** TX watchdog, loop watchdog kèm cắt stream khẩn cấp, hàng rào mềm, phát hiện phi công đánh stick, đối chiếu lệnh yaw với yaw thật, giới hạn thời gian engage, chặn NaN/clamp, xác nhận danh tính mục tiêu, phát hiện EKF reset.
- **Tiền kiểm tra:** từ chối ENGAGE nếu chưa arm, vị trí cũ, PX4 báo failsafe, chưa bật RC dead-man, pin thấp, chưa chọn mục tiêu hoặc đã ra ngoài hàng rào. Có kiểm tra kênh RC dead-man trùng với `RC_MAP_*` của PX4.
- **Web UI có token và giữ-để-engage**, ghi CSV/video mỗi chuyến bay để xem lại.
- **Gói companion (`s500-companion`):** tự tìm Pixhawk qua USB, ghi và kiểm tra telemetry MAVLink chỉ-đọc, audit parameter, safety supervisor / flight guard, test đổi mode trên Pixhawk thật khi disarmed.
- **Mô hình Gazebo S500:** mesh lấy từ CAD, camera optical-flow và rangefinder MTF-01, IMU/GPS/baro/mag, gắn với PX4 SITL (airframe 4001).
- **217 test tự động** (186 Detect + 31 Companion), không cần phần cứng.

Bảng phần cứng, sơ đồ kiến trúc và cấu trúc thư mục: xem phần tiếng Anh ở trên.

### 🚀 Hướng dẫn sử dụng

- **Detect:** `cd Detect && pip install -r requirements.txt`, chạy khô bằng `python -m follow.app --camera 0 --model yolo26n_int8_openvino_model`, chạy test bằng `python -m pytest tests/ -v`. Thư mục model OpenVINO không kèm theo repo, cần tự export bằng Ultralytics. Chi tiết tham số, lệnh bay thật và các cột CSV cần soi: [`Detect/README.md`](Detect/README.md).
- **Companion:** trên UP 7000 chạy `bash deploy/install_up7000.sh`, sau đó `s500-companion discover` và `s500-companion probe --duration 10`. Điều khiển trên Pixhawk thật bị khóa mặc định; trình tự mở khóa xem [`Companion/README.md`](Companion/README.md).
- **Mô phỏng:** trong WSL chạy `bash Simulation/gazebo/run_s500_gazebo_wsl.sh` và `bash Simulation/gazebo/run_s500_px4_sitl_wsl.sh` ở hai terminal. Home đặt tại **Trường Đại học Bách khoa – ĐHQG TP.HCM, 268 Lý Thường Kiệt, Quận 10, TP.HCM** (`10.772100, 106.657900`). Cách nối QGroundControl: [`Simulation/gazebo/README.md`](Simulation/gazebo/README.md).

### 🛡️ Trước khi bay thật

1. Unit test đạt trên đúng máy UP 7000.
2. Có log các test lỗi trên SITL (kill process, mất stream, mục tiêu cũ, RC dead-man).
3. Bench trên Pixhawk thật đã **tháo cánh**: xác nhận Position / RTL / RC override / geofence.
4. Đã cấu hình Offboard-loss (`COM_OF_LOSS_T`), geofence cứng (`GF_*`) và công tắc Position/RTL trên RC. Ba lớp này nằm ngoài code và là bắt buộc.
5. Chuyến đầu: `--max-yaw-rate 15`, hàng rào nhỏ, `--max-engage-s` ngắn.

---

<p align="center">Made by <a href="https://github.com/TuanLinh05">Vu Tuan Linh</a> · HCMUT</p>
