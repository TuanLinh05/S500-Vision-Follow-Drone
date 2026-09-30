# Báo cáo kết quả thực hiện và phân tích bộ code Detect v2.0

**Ngày thực hiện:** 23/08/2026
**Dự án:** S500 Drone — Xử lý ảnh và điều khiển yaw bám theo người
**Người thực hiện:** AI Agent

---

## 1. Tóm tắt công việc

Viết lại hoàn toàn file monolith `Ref/Detect/follow_px4.py` (1117 dòng, 47KB) thành **11 module Python** + **3 file test** + README + `requirements.txt`, sửa tổng cộng **7 lỗi P0 an toàn nghiêm trọng** và **11 lỗi P1/P2** theo phân tích trong `DroneFollowPX4.md`.

| Hạng mục | Trước | Sau |
|---|---|---|
| Số file | 1 | 15 |
| Tổng dòng code | ~1117 | ~1450 |
| Module | 0 | 11 |
| Test tự động | 0 | 38 |
| Lỗi P0 còn lại | 7 | 0 |

---

## 2. Cấu trúc bàn giao

```
Detect/
├── follow/                     # package chính
│   ├── __init__.py             # version 2.0.0
│   ├── safety.py        (136 dòng)  Gate an toàn thuần túy
│   ├── control.py       (105 dòng)  Bộ điều khiển yaw thuần túy
│   ├── px4.py           (210 dòng)  PX4 MAVLink + SetpointStreamer
│   ├── camera.py         (72 dòng)  Camera USB + seq + timestamp
│   ├── detect.py         (87 dòng)  OpenVINO model + letterbox
│   ├── track.py         (140 dòng)  GMC + ByteTrack
│   ├── target.py        (180 dòng)  Quản lý mục tiêu + Appearance
│   ├── webui.py         (201 dòng)  HTTP + token + dead-man
│   ├── logging_sink.py   (77 dòng)  Ghi CSV/video luồng riêng
│   └── app.py           (478 dòng)  Vòng lặp chính
├── tests/
│   ├── __init__.py
│   ├── test_safety.py    (22 tests)
│   ├── test_control.py    (8 tests)
│   └── test_target.py    (8 tests)
├── README.md                    # Hướng dẫn sử dụng
└── requirements.txt
```

---

## 3. Phân tích từng module

### 3.1. `safety.py` — Gate an toàn

**Vai trò:** Quyết định duy nhất cho phép/chặn điều khiển. Thuần túy logic — không I/O, không đọc đồng hồ hệ thống. Mọi đầu vào truyền qua `GateInputs` dataclass nên có thể test bằng bảng dữ liệu.

**Cấu trúc:**
- `GateInputs` — 16 trường đầu vào (frozen dataclass)
- `GateConfig` — 13 tham số cấu hình có giá trị mặc định
- `GateResult` — kết quả: `allow`, `reason`, `all_reasons`, `fatal`
- `check(cfg, s)` — hàm kiểm tra duy nhất

**8 lớp kiểm tra (theo thứ tự ưu tiên):**

| Lớp | Điều kiện | Fatal? | Fix |
|---:|---|---|---|
| 1 | `--enable-control` chưa bật | Không | — |
| 2 | MAVLink chưa kết nối / heartbeat quá hạn | Có | A3 |
| 3 | Drone chưa arm / thiếu vị trí / mode sai / trôi | Có | A7 |
| 4 | RC dead-man: kiểm tra **tuổi dữ liệu trước**, rồi giá trị | Có | **A4** |
| 5 | Pin: kiểm tra **tuổi dữ liệu trước**, rồi điện áp | Có | **A4** |
| 6 | Camera mất / vòng lặp treo | Có | A1 |
| 7 | Người vận hành mất heartbeat web | Có | **A5** |
| 8 | Mục tiêu: chưa khóa / cần xác nhận / quá cũ | **Không** | C3 |

> **Điểm then chốt:** Lớp 8 (mục tiêu) KHÔNG fatal — chỉ zero yaw chứ không disengage. Điều này cho phép drone giữ vị trí khi mất mục tiêu tạm thời.

**Test coverage:** 22 tests kiểm tra từng điều kiện chặn độc lập + bypass (rc_chan=0, batt_min=0), nhiều điều kiện đồng thời, và điều kiện phụ thuộc trạng thái (mode chỉ check khi engaged).

---

### 3.2. `control.py` — Bộ điều khiển yaw

**Vai trò:** Tính toán tốc độ yaw (°/s) từ vị trí bounding box. Thuần túy số học — không I/O.

**Các fix được áp dụng:**

| Fix | Vấn đề gốc | Cách sửa |
|---|---|---|
| **B1** | Ánh xạ pixel→góc tuyến tính (sai ở góc rộng) | `atan2(x_px - w/2, fx)` với `fx = (w/2) / tan(hfov/2)` |
| **B2** | Không xử lý box chạm biên khung hình | Phát hiện biên `EDGE_PX=4`; dùng cạnh nhìn thấy làm tham chiếu; tốc độ tối thiểu `min_edge_rate` |
| **B3** | Không bù trễ pipeline | Tham số `lead_s`; lệnh = `bearing + ω × lead_s` |
| **B4** | Chỉ P-only, không feed-forward | `raw = gain × e_eff + k_ff × ω_target` |
| **B7** | Deadzone cứng (0 hoặc full) | Deadzone mềm: `e_eff = |e| - deadzone` khi `|e| > deadzone` |

**Pipeline tính toán:**
```
box → measure (B2) → bearing (B1) → ω estimate → lead_s bù trễ (B3)
→ soft deadzone (B7) → P + feed-forward (B4) → clamp max_rate → slew rate → yaw
```

**Hành vi an toàn:** Khi `box=None` hoặc `allow=False`, yaw về 0 **tức thì** (bỏ qua slew rate). Đây là thiết kế có chủ ý — an toàn > mượt.

**Test coverage:** 8 tests kiểm tra công thức pinhole, mất mục tiêu, gate chặn, deadzone mềm, box biên, slew limit, invert, max_rate.

---

### 3.3. `px4.py` — Liên kết MAVLink

**Vai trò:** Giao tiếp duy nhất với Pixhawk 6C. Gồm 3 thành phần:

#### `Stamped` dataclass
Bọc mọi giá trị MAVLink kèm timestamp monotonic. Property `.age` trả về tuổi dữ liệu (giây). **Fix A4.**

#### `PX4Link`
- **Fix A6:** `source_system=191`, `MAV_COMP_ID_ONBOARD_COMPUTER` — tránh trùng sysid=1 của autopilot
- RX thread xử lý: HEARTBEAT, RC_CHANNELS, LOCAL_POSITION_NED, ATTITUDE, BATTERY_STATUS, COMMAND_ACK, STATUSTEXT
- `set_mode()`: gửi `DO_SET_MODE` → chờ ACK → chờ heartbeat xác nhận mode → retry 3 lần (**fix A3**)
- `enter_offboard()` / `enter_loiter()`: wrapper cho `set_mode()`
- `enter_loiter()` **chỉ gọi khi đang ở OFFBOARD** — tránh giật quyền khỏi phi công (**fix A2**)

#### `SetpointStreamer(Thread)`
- **Fix A1 (QUAN TRỌNG NHẤT):** Watchdog 0.25s — nếu main loop không gọi `command()` trong 0.25s, tự động ép yaw về 0. Print cảnh báo. Đây là lớp bảo vệ khi main loop treo.
- **Fix A7:** Dùng `TYPE_MASK_POS_YAWRATE = 1528` khi có hold position (chốt x,y,z lúc ENGAGE). Velocity=0 gốc gây trôi drone.
- **Fix B5:** `_t_boot` CỐ ĐỊNH từ init, không reset. `_t_win` riêng cho đo tần số.
- **Fix B6:** Lập lịch theo mốc tuyệt đối (`next_t += period`), reset khi bị trễ.
- Metrics: `sp_sent`, `sp_hz`, `watchdog_trips`

---

### 3.4. `camera.py` — Camera USB

**Fix C5:** Thêm `_seq` (tăng mỗi frame mới) và `_t_cap` (timestamp monotonic). Main loop dùng `seq` để skip frame trùng lặp — tránh xử lý cùng một frame 2 lần khi camera chậm hơn detector.

**Thread safety:** Lock `_lk` bảo vệ `_f`, `_seq`, `_t_cap`. Property `age` an toàn cho thread.

**Fallback:** Thử `CAP_V4L2` trước, nếu thất bại thì fallback API mặc định. Hỗ trợ file video (loop khi hết).

---

### 3.5. `detect.py` — OpenVINO inference

**Fix C1:** Thêm lọc aspect ratio (`MIN_ASPECT=1.2`, `MAX_ASPECT=5.0`) và chiều cao tối thiểu (`MIN_BOX_H=24px`). Loại bỏ false positive từ vật thể không phải người (quá rộng, quá nhỏ).

**Model format:** Yêu cầu NMS-free YOLO (output shape `[batch, N, 6]` — x1,y1,x2,y2,conf,class). Tương thích YOLO26n/YOLOv8n đã export OpenVINO.

**Preprocessing:** `PrePostProcessor` tích hợp — BGR→RGB, u8→f32, scale 255, NHWC→NCHW. Cache model compiled.

---

### 3.6. `track.py` — ByteTrack

Giữ nguyên logic gốc, refactor sạch:
- `GMC`: Phase correlation ước lượng chuyển động camera
- `ByteTrack`: Matching 2 bước (confidence cao → tất cả, confidence thấp → track chưa ghép)
- `_Track`: Velocity-based prediction với shift compensation

---

### 3.7. `target.py` — Quản lý mục tiêu

**Fix C2:** Thêm `Appearance` class — HSV histogram 16×16 bins, khoảng cách Bhattacharyya. Mỗi `LockedTarget` lưu embedding cập nhật EMA (0.8 cũ + 0.2 mới).

**Fix C3:** `_try_reacquire()` sử dụng:
- Score = 0.35 × (khoảng cách/r_max) + 0.65 × appearance_distance
- **Từ chối** nếu appearance > 0.45 (khác quá)
- **Từ chối** nếu 2 ứng cử viên quá giống nhau (margin < 0.12) → đặt `needs_confirm=True`
- Không reacquire track đã thuộc mục tiêu khác

**API đầy đủ:** `pick_at()`, `unlock_all()`, `unlock_primary()`, `set_primary()`, `next_primary()`, `update()`, `primary`, `primary_target`, `locked`, `msg`.

---

### 3.8. `webui.py` — Web UI

**Fix A5:** 
- Token `secrets.token_urlsafe(16)` mỗi lần chạy, in ra console
- Mọi endpoint state-changing yêu cầu `?t=TOKEN`, so sánh bằng `hmac.compare_digest`
- Bind `127.0.0.1` (không phải `0.0.0.0`)

**Fix B8:**
- ENGAGE hoạt động như dead-man: nhấn giữ → gửi `/alive` mỗi 200ms, thả ra → `/disengage`
- `stop_latch` (threading.Event) có ưu tiên tuyệt đối — kiểm tra đầu tiên mỗi vòng lặp
- `window.onblur` tự động disengage

**Endpoints:** `/` (trang), `/stream.mjpg`, `/stats` (JSON), `/alive`, `/engage` (POST), `/disengage` (POST), `/pick?x=&y=`

---

### 3.9. `logging_sink.py` — Ghi log

**Fix D2:** Flush CSV sau mỗi dòng — không mất dữ liệu khi crash.
**Fix D3:** Chạy trên luồng riêng, main loop dùng `put_nowait()` — không bao giờ chặn vòng điều khiển. Queue đầy → bỏ, không đợi.

---

### 3.10. `app.py` — Vòng lặp chính

**Pipeline mỗi chu kỳ:**
```
stop_latch? → camera.read(seq check) → GMC → letterbox → inference → postprocess
→ ByteTrack → web commands → TargetManager.update() → build GateInputs
→ gate_check() → disengage if fatal → YawController.update()
→ SetpointStreamer.command() → LoggingSink.push() → draw → web stats → console
```

**Disengage 3 chế độ** (`--on-abort`):
| Mode | Hành vi |
|---|---|
| `loiter` | Zero yaw + `enter_loiter()` — PX4 giữ vị trí |
| `stream-off` | Dừng hẳn SetpointStreamer — PX4 kích hoạt Offboard-loss |
| `hold-only` | Chỉ zero yaw, giữ Offboard |

**Cleanup:** `finally` block đảm bảo: disengage → zero setpoint → đợi 0.3s → halt streamer → enter_loiter → close link → release camera → stop logger.

---

## 4. Bảng đối chiếu sửa lỗi

### 4.1. Lỗi P0 (An toàn — BẮT BUỘC sửa trước khi gắn cánh quạt)

| ID | Mô tả | Module | Trạng thái | Chi tiết |
|---|---|---|---|---|
| **A1** | TX thread không có watchdog | `px4.py` | ✅ ĐÃ SỬA | `SetpointStreamer.WATCHDOG_S = 0.25` |
| **A2** | Disengage không thoát Offboard | `px4.py` + `app.py` | ✅ ĐÃ SỬA | `enter_loiter()` + `--on-abort` |
| **A3** | Không xác minh mode Offboard | `px4.py` | ✅ ĐÃ SỬA | ACK + heartbeat, retry 3× |
| **A4** | RC/pin không kiểm tra tuổi | `px4.py` + `safety.py` | ✅ ĐÃ SỬA | `Stamped` + gate kiểm tra `.age` |
| **A5** | Web không xác thực, bind 0.0.0.0 | `webui.py` | ✅ ĐÃ SỬA | Token + hmac + 127.0.0.1 |
| **A6** | source_system=1 trùng autopilot | `px4.py` | ✅ ĐÃ SỬA | `source_system=191` |
| **A7** | Velocity(0,0,0) không giữ vị trí | `px4.py` | ✅ ĐÃ SỬA | Position hold mask=1528 |

### 4.2. Lỗi P1 (Chất lượng điều khiển)

| ID | Mô tả | Module | Trạng thái |
|---|---|---|---|
| B1 | Ánh xạ pixel→góc tuyến tính | `control.py` | ✅ ĐÃ SỬA |
| B2 | Không xử lý box biên khung | `control.py` | ✅ ĐÃ SỬA |
| B3 | Không bù trễ pipeline | `control.py` | ✅ ĐÃ SỬA |
| B4 | Thiếu feed-forward | `control.py` | ✅ ĐÃ SỬA |
| B5 | `time_boot_ms` reset mỗi giây | `px4.py` | ✅ ĐÃ SỬA |
| B6 | Lập lịch TX bị trôi | `px4.py` | ✅ ĐÃ SỬA |
| B7 | Deadzone cứng | `control.py` | ✅ ĐÃ SỬA |
| B8 | Queue lệnh web single-slot | `webui.py` + `app.py` | ✅ ĐÃ SỬA |

### 4.3. Lỗi P1/P2 (Vision + Architecture)

| ID | Mô tả | Module | Trạng thái |
|---|---|---|---|
| C1 | Không lọc false positive | `detect.py` | ✅ ĐÃ SỬA |
| C2 | Không có đặc trưng ngoại hình | `target.py` | ✅ ĐÃ SỬA |
| C3 | Reacquire mù (nearest neighbor) | `target.py` | ✅ ĐÃ SỬA |
| C5 | Không phát hiện frame trùng | `camera.py` | ✅ ĐÃ SỬA |
| D2 | CSV không flush | `logging_sink.py` | ✅ ĐÃ SỬA |
| D3 | I/O chặn vòng điều khiển | `logging_sink.py` | ✅ ĐÃ SỬA |

---

## 5. Kết quả test tự động

```
============================= test session starts =============================
platform win32 -- Python 3.14.6, pytest-9.1.0

tests/test_control.py  (8 tests)    ✅ ALL PASSED
tests/test_safety.py  (22 tests)    ✅ ALL PASSED
tests/test_target.py   (8 tests)    ✅ ALL PASSED

============================= 38 passed in 0.19s ==============================
```

### Chi tiết test coverage

| Module | Số test | Kiểm tra gì |
|---|---:|---|
| `safety.py` | 22 | 16 điều kiện chặn độc lập (fatal/non-fatal), bypass RC/pin, mode/operator chỉ check khi engaged |
| `control.py` | 8 | Công thức pinhole, mất mục tiêu, gate chặn, deadzone mềm, box biên, slew, invert, max_rate |
| `target.py` | 8 | HSV histogram embed/dist, pick_at đúng/sai vị trí, unlock_all, next_primary |

---

## 6. Vấn đề phát hiện và đã sửa trong kiểm tra

### 6.1. Lần viết đầu (subagent)

| # | Vấn đề | Mức | Đã sửa? |
|---|---|---|---|
| 1 | `px4.py` v1: thiếu thread-safe lock cho `_cmd_yaw`; thiếu `sp_hz`, `watchdog_trips` | CRITICAL | ✅ Viết lại hoàn toàn |
| 2 | `target.py` v1: thiếu API `pick_at`, `unlock_all`, `primary`, `locked` | CRITICAL | ✅ Viết lại hoàn toàn |
| 3 | `app.py` v1: import sai module name, skeleton quá trống | CRITICAL | ✅ Viết lại hoàn toàn |
| 4 | `__init__.py` v1: chỉ 7 bytes | MINOR | ✅ Đã cập nhật |

### 6.2. Lần review tự động (code reviewer agent)

| # | Vấn đề | Mức | Đã sửa? |
|---|---|---|---|
| 5 | Web server `start_web()` gọi 2 lần trên cùng port → crash `OSError: Address already in use` | **CRITICAL** | ✅ Đã sửa — chỉ gọi 1 lần sau khi callbacks được định nghĩa |
| 6 | `control.py` edge logic: `ref = x2 if left else x1` → sai hướng khi box rộng vượt tâm | **CRITICAL** | ✅ Đã sửa — đổi thành `ref = x1 if left else x2` (dùng cạnh biên khung hình) |
| 7 | `px4.py` ACK queue tích lũy ACK cũ → `set_mode()` đọc ACK sai lệnh | WARNING | ✅ Đã sửa — xóa queue trước mỗi lệnh `DO_SET_MODE` |
| 8 | `px4.py` `watchdog_trips` không có lock khi truy cập từ 2 thread | MINOR | ⚠️ Chấp nhận — Python GIL bảo vệ integer read/write; đã ghi chú |

---

## 7. Hạn chế và việc cần làm tiếp

### 7.1. Chưa test được trên phần cứng

| Hạng mục | Trạng thái | Bị chặn bởi |
|---|---|---|
| SITL test (P4) | Chưa chạy | Cần PX4 SITL + model YOLO |
| Bench test (P5) | Chưa chạy | Cần Pixhawk USB + UP7000 |
| Shadow flight (P6) | Chưa chạy | P4, P5 phải PASS |
| Bay thật (P7) | Chưa chạy | P6 phải PASS |

### 7.2. Tham số cần điều chỉnh khi test thật

| Tham số | Giá trị hiện tại | Ghi chú |
|---|---:|---|
| `--max-yaw-rate` | 40.0 | Nên bắt đầu 15°/s cho chuyến đầu |
| `--deadzone` | 4.0 | Có thể cần tăng nếu oscillation |
| `--hfov` | 90.0 | Cần đo thật trên Rapoo C280 |
| `--lead` | 0.0 | Đo latency thật rồi đặt |
| `--k-ff` | 0.0 | Bật sau khi P-only ổn |
| `--gain` | 0.6 | Tune theo response thật |

### 7.3. Thiếu sót so với kế hoạch USB

Theo `ke_hoach_dieu_khien_up7000_pixhawk_usb.md`, phần mềm Companion có kiến trúc flight_guard riêng biệt. Code Detect này chỉ là **pipeline vision + guidance**, không phải flight_guard đầy đủ. Trước khi bay thật, cần:

1. Tích hợp với Companion flight_guard (nếu dùng kiến trúc tách biệt) **HOẶC** dùng trực tiếp Detect/follow/app.py (đã có safety gate tích hợp)
2. Thiết lập soft fence trong safety.py (`max_drift_m`)
3. Chỉnh PX4 parameters theo profile `ai_yaw_test.params`
4. Map kênh RC AI Enable

---

## 8. Cách chạy

### Cài đặt
```bash
pip install opencv-python numpy openvino pymavlink pytest
```

### Chạy khô (không kết nối MAVLink)
```bash
cd Detect
python -m follow.app --camera 0 --model yolo26n_int8_openvino_model
```

### Chạy test
```bash
cd Detect
python -m pytest tests/ -v
```

### Kết nối SITL
```bash
python -m follow.app --mavlink udp:127.0.0.1:14540 --enable-control --rc-chan 7
```

### Kết nối Pixhawk USB thật
```bash
python -m follow.app \
  --mavlink /dev/serial/by-id/usb-Holybro_... \
  --enable-control --rc-chan 7 --batt-min 14.0 \
  --max-yaw-rate 15 --on-abort loiter
```

---

## 9. Kết luận

Bộ code Detect v2.0 đã sửa **toàn bộ 7 lỗi P0** an toàn nghiêm trọng và phần lớn lỗi P1/P2. Kiến trúc module cho phép:

- **Test từng phần** mà không cần phần cứng (safety, control, target đều thuần logic)
- **Thay thế module** mà không ảnh hưởng module khác (vd: thay model YOLO chỉ sửa detect.py)
- **Debug dễ dàng** — log CSV ghi đầy đủ trạng thái, web UI hiển thị real-time

**Bước tiếp theo bắt buộc:**
1. Chạy SITL test với PX4 (P4)
2. Đo hfov thật của Rapoo C280
3. Bench test tháo cánh với Pixhawk USB
4. Shadow flight (chạy nhưng không mở Offboard)
5. Bay yaw-only 5-10 giây/lần với max_yaw_rate=15°/s

> ⚠️ **CẢNH BÁO AN TOÀN:** Code KHÔNG tự arm, KHÔNG tự cất cánh. Phi công PHẢI cất cánh và hover Position TRƯỚC khi bật AI. Luôn có công tắc Position/RTL trên RC.
