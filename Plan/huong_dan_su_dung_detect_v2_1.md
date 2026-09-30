# Hướng dẫn cài đặt & sử dụng `Detect` (v2.1)

> **Cập nhật an toàn v2.2 (27/08/2026):** Code hiện tại bắt buộc RC dead-man
> khi có `--enable-control` (trừ `--allow-no-rc-deadman` dành riêng cho
> SITL/bench), không tự đổi sang người khác khi target LOST/re-ID, và watchdog
> có thể cắt stream setpoint độc lập với main loop. Đọc `Detect/README.md` là
> hướng dẫn vận hành mới nhất.

**Ngày viết:** 25/08/2026
**Đối tượng đọc:** người sẽ cài, chạy, và vận hành bộ xử lý ảnh cho dự án S500.
**Vị trí code:** `Detect/follow/` — package Python, điểm vào `python -m follow.app`.

Tài liệu này gộp lại "đã làm gì" + "dùng thế nào" thành một chỗ. Để đọc chi tiết
từng lỗi đã sửa và cơ sở lý luận an toàn, xem
[`bao_cao_kiem_tra_va_nang_cap_detect_v2_1.md`](bao_cao_kiem_tra_va_nang_cap_detect_v2_1.md)
và [`DroneFollowPX4.md`](DroneFollowPX4.md).

---

## 1. Đã làm được gì

`Detect` là pipeline **xử lý ảnh + điều khiển yaw** cho drone S500: nhận diện
người bằng YOLO chạy trên OpenVINO, bám một người do người vận hành chọn qua
web, và gửi lệnh tốc độ xoay (yaw-rate) xuống PX4 qua MAVLink. Đây **không**
phải bộ điều khiển bay đầy đủ — nó chỉ đề xuất một trục yaw, mọi thứ khác
(attitude, rate, motor, arm, cất/hạ cánh, RTL, geofence cứng) vẫn hoàn toàn
do PX4 và phi công quyết định.

Quá trình thực hiện có ba giai đoạn:

1. **Viết lại từ file monolith** `Ref/Detect/follow_px4.py` (1117 dòng) thành
   package 11 module + test, sửa 7 lỗi an toàn nghiêm trọng (P0) theo phân
   tích `DroneFollowPX4.md`. Kết quả ghi trong
   [`bao_cao_ket_qua_viet_lai_detect_v2.md`](bao_cao_ket_qua_viet_lai_detect_v2.md).
2. **Kiểm tra lại từng dòng (v2.1)**: tìm ra 10 lỗi còn sót trong bản viết
   lại — trong đó có 4 lỗi làm hỏng đúng lớp an toàn mà tài liệu nói là đã
   có, và 2 lỗi vision làm mất mục tiêu ở đúng lúc nguy hiểm nhất (người ở
   rìa khung hình).
3. **Bổ sung 7 lớp chống "bay mất kiểm soát"** theo yêu cầu an toàn trong
   `ke_hoach_dieu_khien_up7000_pixhawk_usb.md` — hàng rào mềm (geofence),
   watchdog vòng lặp độc lập, phát hiện phi công giành quyền, đối chiếu lệnh
   với chuyển động thật, giới hạn thời lượng mỗi lần bật AI.
4. **Kiểm chứng bằng PX4 v1.17 SITL thật** (không phải PX4 giả tự viết):
   chuỗi ENGAGE→OFFBOARD→yaw đúng chiều→tự ngắt đúng giới hạn→LOITER, cộng
   4 trong 8 kịch bản SITL của `DroneFollowPX4.md` §7.2 (ENGAGE khi chưa
   arm bị từ chối, main loop treo, mất nhịp operator...).
5. **Chạy thật trên Pixhawk 6C + khung S500** (bench, cánh quạt tháo) và
   trên chính **UP7000** (companion computer thật) — tìm và sửa 2 lỗi chỉ
   lộ ra trên phần cứng đích: `opencv-python` thiếu `libGL` trên máy
   headless, và `CAP_PROP_BUFFERSIZE=1` làm giảm một nửa FPS camera thật.

Chi tiết từng lỗi + cách sửa nằm trong
[`bao_cao_kiem_tra_va_nang_cap_detect_v2_1.md`](bao_cao_kiem_tra_va_nang_cap_detect_v2_1.md).

**Trạng thái kiểm thử:** 143 test tự động PASS — trên máy dev, trên PX4
SITL thật, và trên chính UP7000 + camera thật. Bench Pixhawk thật đã xác
nhận kết nối/RC/pin đúng nhưng **chưa ENGAGE được** (PX4 từ chối arm vì
thiếu GPS/optical-flow trong nhà — hành vi an toàn đúng, không phải lỗi).
**Chưa bay.**

---

## 2. Detect làm được gì ngay bây giờ

### 2.1. Luồng xử lý một khung hình

```
Camera USB (seq + timestamp)
  → GMC (bù chuyển động camera bằng phase correlation)
  → letterbox → suy luận OpenVINO (YOLO NMS-free)
  → hậu xử lý: lọc confidence, lọc class người, lọc tỉ lệ/kích thước khung người
  → ByteTrack (ghép 2 ngưỡng: confidence cao trước, thấp sau)
  → TargetManager: cập nhật box mục tiêu đã khoá, thử tìm lại nếu mất dấu
  → YawController: box → góc lệch → tốc độ yaw (°/s), có bù trễ + feed-forward
  → SafetyGate: 9 lớp kiểm tra, quyết định CHO PHÉP hay CHẶN
  → SetpointStreamer: gửi setpoint MAVLink 20 Hz xuống PX4 (có watchdog riêng)
  → LoggingSink: ghi CSV + video trên luồng riêng, không chặn vòng điều khiển
  → Web UI: xem hình trực tiếp, chọn mục tiêu, bấm ENGAGE/DISENGAGE
```

### 2.2. Nhận diện & bám mục tiêu

- Model YOLO dạng NMS-free (output `[batch, N, 6]`), chạy qua OpenVINO trên
  CPU hoặc GPU (UP7000 dùng iGPU).
- Lọc bỏ vật không phải người bằng tỉ lệ cao/rộng và chiều cao tối thiểu
  (`--min-aspect`, `--max-aspect`, `--min-box-h`) — lọc **trước khi** cắt
  box vào khung, để người đứng ở rìa khung không bị loại oan.
- Người vận hành **bấm chuột vào một người trên video** để khoá mục tiêu
  (không tự chọn). Mỗi mục tiêu có nhãn `M1`, `M2`... và một màu riêng.
- Có thể khoá nhiều người cùng lúc, chọn 1 người làm "mục tiêu chính"
  (`primary`) — chỉ mục tiêu chính mới sinh ra lệnh yaw.
- Khi mục tiêu bị che khuất tạm thời, hệ thống thử **tìm lại** bằng khoảng
  cách + đặc trưng màu sắc (histogram HSV vùng thân trên). Nếu hai người
  trông giống nhau đến mức không phân biệt được, hệ thống **từ chối đoán**
  và yêu cầu người vận hành xác nhận lại — không tự tráo nhãn.
- Mục tiêu mất dấu quá `--reacquire` giây (mặc định 4s) thì bị bỏ khoá hẳn.

### 2.3. Điều khiển yaw

- Sai số góc tính bằng công thức pinhole (`atan2`), không phải xấp xỉ tuyến
  tính — chính xác cả ở rìa khung góc rộng.
- Vùng chết (deadzone) mềm — trừ dần thay vì cắt cứng, không giật khi mục
  tiêu ra vào vùng chết.
- Có bù trễ pipeline (`--lead`) và feed-forward theo vận tốc góc ước lượng
  của mục tiêu (`--k-ff`) — giảm sai số bám thường trực khi người di chuyển.
- Box chạm biên khung hình được xử lý riêng: dùng cạnh nhìn thấy làm tham
  chiếu, đặt tốc độ tối thiểu để kéo người sắp ra khỏi khung về giữa.
- Giới hạn tốc độ tối đa (`--max-yaw-rate`) và giới hạn gia tốc góc (slew
  rate) để chuyển động mượt và an toàn.
- **Mất mục tiêu hoặc bị chặn → yaw về 0 ngay lập tức**, bỏ qua slew rate.
  Đây là thiết kế có chủ ý: an toàn quan trọng hơn mượt mà.

### 2.4. Kết nối PX4 (MAVLink)

- Dùng `source_system=191` (không trùng sysid=1 của autopilot).
- Bắt tay đúng heartbeat của autopilot, bỏ qua heartbeat của GCS hoặc thiết
  bị khác trên cùng đường truyền.
- Đổi mode (vào/ra Offboard) chạy trên **luồng riêng**, không chặn vòng điều
  khiển; xác minh bằng cả `COMMAND_ACK` lẫn heartbeat xác nhận mode thật sự
  đổi — không tin ACK suông.
- Ra khỏi Offboard **chỉ khi đang ở Offboard** — không giật quyền khỏi phi
  công nếu họ đã tự giành lại bằng RC.
- Setpoint dùng **vị trí + yaw-rate** (không phải vận tốc 0), có phản hồi vị
  trí thật nên không trôi theo gió khi giữ chỗ.
- Khi **chưa** ENGAGE, vẫn phát setpoint giữ nguyên vị trí hiện tại (không
  phải vận tốc 0) — nếu PX4 bất ngờ vào Offboard vì lý do khác, drone đứng
  yên thay vì trôi.
- Luồng gửi setpoint có **watchdog 0.25s** riêng: nếu vòng điều khiển chính
  ngừng nuôi lệnh (treo, crash logic...), yaw tự động về 0 dù luồng gửi vẫn
  sống.

### 2.5. Bảy lớp chống "bay mất kiểm soát"

| # | Lớp | Chặn được gì |
|---|---|---|
| 1 | TX watchdog 0.25s trong luồng gửi setpoint | Vòng điều khiển treo → yaw tự về 0 |
| 2 | Watchdog vòng lặp độc lập (luồng riêng) | Vòng điều khiển treo → **thật sự ngắt** + thoát Offboard |
| 3 | Hàng rào mềm (bán kính / trần / sàn / tốc độ, có cảnh báo sớm) | Drone trôi ra khỏi vùng thử |
| 4 | Phát hiện phi công đóng cần lái (RC stick) | Người lái giành quyền mà phần mềm không biết |
| 5 | Đối chiếu lệnh yaw với tốc độ xoay thật đo từ PX4 | Drone tự xoay hoặc không theo lệnh dù kết nối vẫn khoẻ |
| 6 | Giới hạn thời lượng mỗi lần bật AI | Người vận hành quên buông nút |
| 7 | Chặn NaN/Inf + giới hạn cứng ngay trước khi gửi xuống PX4 | Lệnh sai đơn vị, giá trị bất thường |

Ba lớp **không** nằm trong code này, vẫn bắt buộc phải có riêng: PX4
Offboard-loss (tham số `COM_OF_LOSS_T`), geofence cứng của PX4
(`GF_MAX_HOR_DIST`/`GF_MAX_VER_DIST` — **hiện đang tắt, cần bật trước khi
bay thật**), và công tắc Position/RTL vật lý trên tay điều khiển.

### 2.6. Giao diện web

- Chạy trên `127.0.0.1:8080` mặc định, có token ngẫu nhiên sinh mỗi lần
  chạy (in ra console) — mọi thao tác đều cần token, chống truy cập trái
  phép và giả mạo request (CSRF).
- Xem hình trực tiếp (MJPEG), bấm chuột vào người để khoá mục tiêu.
- Nút **ENGAGE phải giữ** (dead-man): thả tay ra là tự động ngắt trong dưới
  1 giây. Chuyển tab hoặc mất focus cửa sổ cũng tự ngắt.
- Bảng trạng thái thời gian thực: mode PX4, pin, khoảng cách tới hàng rào,
  độ cao, tốc độ, tần số setpoint, số lần watchdog trip...

### 2.7. Ghi log

- CSV ghi mỗi khung hình, flush ngay (không mất dữ liệu khi crash), chạy
  trên luồng riêng nên không làm chậm vòng điều khiển.
- Cột quan trọng nhất khi điều tra sự cố: `block` (lý do bị chặn), `fence`,
  `sp_hz`, `watchdog_trips`, `loop_wd_trips`, `yaw_cmd_deg_s` so với
  `yaw_thuc_deg_s`.
- Có thể ghi cả video kèm hình vẽ debug (`--save-video`).

### 2.8. Giới hạn hiện tại (chưa làm)

- Chưa hiệu chuẩn nội tại camera Rapoo C280 — `--hfov 90` là số phỏng đoán,
  chưa đo `--fx-px` thật.
- Chưa đo trễ vòng kín thật để đặt `--lead`.
- Model INT8 chưa được hiệu chỉnh lại bằng ảnh chụp từ đúng camera/góc nhìn
  trên không — vẫn dùng bộ hiệu chỉnh gốc.
- Tầm phát hiện thực tế ước tính ~15-20m với `--imgsz 416`; chưa làm suy
  luận hai mức để tăng tầm.
- Chưa chạy SITL với PX4 thật, chưa bench, chưa bay.

---

## 3. Cài đặt

### 3.1. Yêu cầu

- Python 3.10+ (đã test với 3.12).
- Máy tính đồng hành: UP7000 (Ubuntu) khi chạy thật; máy Windows/Linux bất
  kỳ khi chạy khô hoặc chạy test.
- Camera USB (Rapoo C280 hoặc tương đương) khi chạy thật.
- Pixhawk 6C chạy PX4 khi kết nối MAVLink thật; hoặc PX4 SITL qua UDP khi
  thử nghiệm.

### 3.2. Cài thư viện

```bash
cd Detect
pip install -r requirements.txt
```

`requirements.txt` gồm: `opencv-python-headless`, `numpy`, `openvino`,
`pymavlink`, `pyserial`, `pytest`. Trên UP7000 (Ubuntu), cùng lệnh trên là
đủ.

> Dùng bản **headless** vì UP7000 không có desktop/GUI — code trong
> `follow/` không gọi hàm GUI nào của OpenCV (`imshow`...), chỉ vẽ lên
> buffer ảnh rồi encode JPEG cho web. Nếu máy đã lỡ cài `opencv-python`
> (bản đầy đủ) thì phải gỡ trước khi cài `requirements.txt`, vì hai gói
> cùng cung cấp module `cv2` và xung đột nhau:
> ```bash
> pip uninstall opencv-python -y
> pip install -r requirements.txt
> ```
> Dùng nhầm bản đầy đủ trên máy headless sẽ báo lỗi
> `ImportError: libGL.so.1: cannot open shared object file` — đã gặp thật
> khi chạy trên UP7000, không phải giả định.

### 3.3. Chuẩn bị model

Cần một model YOLO dạng **NMS-free**, xuất sang định dạng OpenVINO
(`.xml` + `.bin`), output shape `[batch, N, 6]` (x1, y1, x2, y2, conf,
class). Ví dụ YOLO26n hoặc YOLOv8n đã export. Đặt thư mục chứa file `.xml`
và trỏ `--model` vào đó (mặc định tìm `yolo26n_int8_openvino_model/`).

### 3.4. Kiểm tra cài đặt bằng test

```bash
cd Detect
python -m pytest tests/ -v
```

Bộ test **không cần OpenVINO thật, không cần camera, không cần Pixhawk** —
dùng model giả, video giả, và PX4 giả qua UDP loopback
(`tests/fake_px4.py`). Nếu 143 test đều PASS, môi trường Python + các
module logic đã sẵn sàng. (OpenVINO chỉ được import khi thực sự gọi
`load_model()`, nên thiếu OpenVINO không làm hỏng test.)

---

## 4. Cách sử dụng

### 4.1. Chạy khô — xem pipeline vision hoạt động, không cần Pixhawk

```bash
cd Detect
python -m follow.app --camera 0 --model yolo26n_int8_openvino_model
```

Mở trình duyệt tới địa chỉ in ra console (`http://127.0.0.1:8080`), sẽ thấy
token trong log. Bấm vào một người trên hình để khoá mục tiêu, xem sai số
góc và (khi có `--mavlink`) tốc độ yaw đề xuất — nhưng ở chế độ này
**không có lệnh nào được gửi xuống PX4** vì `--enable-control` chưa bật.

Dùng file video có sẵn thay camera:

```bash
python -m follow.app --source video_test.mp4 --model yolo26n_int8_openvino_model
```

### 4.2. Kết nối PX4 SITL (mô phỏng, an toàn tuyệt đối)

```bash
python -m follow.app \
  --mavlink udp:127.0.0.1:14540 \
  --enable-control --rc-chan 7 --rc-min 1500 \
  --batt-min 14.0 \
  --max-yaw-rate 15 --max-engage-s 10
```

Khi đó web sẽ cho phép ENGAGE thật (PX4 sẽ thật sự vào chế độ Offboard
trong SITL). Đây là bước bắt buộc trước khi chạm vào phần cứng thật.

### 4.3. Kết nối Pixhawk USB thật (đã tháo cánh quạt / trên bàn)

```bash
python -m follow.app \
  --mavlink /dev/serial/by-id/usb-Holybro_Pixhawk6C_xxx \
  --enable-control --rc-chan 7 --rc-min 1500 --batt-min 14.0 \
  --max-yaw-rate 15 --max-engage-s 10 \
  --fence-home-radius 30 \
  --on-abort loiter
```

> ⚠️ **Bắt buộc đọc trước khi chạy với phần cứng thật:** drone phải được
> arm và giữ Position bởi phi công **trước** khi bật AI. Code không tự arm,
> không tự cất cánh. Luôn có công tắc Position/RTL sẵn sàng trên tay điều
> khiển — đây là lớp an toàn cuối cùng, không phải dead-man qua MAVLink.

### 4.4. Sử dụng web UI

1. Mở `http://<bind>:<port>` (mặc định `127.0.0.1:8080`) — token in trong
   console lúc khởi động, đã được nhúng sẵn vào trang.
2. Bấm chuột vào người trên video để khoá làm mục tiêu (nhãn `M1`, `M2`...).
3. Bấm và **giữ** nút ENGAGE để bắt đầu điều khiển — thả tay là ngắt ngay.
4. Theo dõi bảng trạng thái: `state` (idle/prestream/switching/engaged),
   `fence` (ok/warn/breach), `mode` PX4, pin, tần số setpoint (`sp_hz` phải
   ≥ 18 khi đang engaged).
5. Nút "BỎ KHOÁ" để huỷ mục tiêu đang chọn.

### 4.5. Các tham số dòng lệnh chính

Chạy `python -m follow.app --help` để xem đầy đủ. Nhóm quan trọng nhất:

**Camera & model**

| Tham số | Mặc định | Ý nghĩa |
|---|---:|---|
| `--camera` | 0 | Chỉ số camera USB |
| `--source` | — | Dùng file video thay camera |
| `--model` | `yolo26n_int8_openvino_model` | Thư mục chứa `.xml` OpenVINO |
| `--device` | GPU | `GPU` hoặc `CPU` |
| `--imgsz` | 416 | Kích thước ảnh vào model |
| `--conf` | 0.4 | Ngưỡng confidence |

**Điều khiển yaw**

| Tham số | Mặc định | Ý nghĩa |
|---|---:|---|
| `--hfov` | 90.0 | Góc nhìn ngang camera (độ) — nên đo thật |
| `--gain` | 0.6 | Hệ số tỉ lệ P |
| `--deadzone` | 4.0 | Vùng chết (độ) |
| `--max-yaw-rate` | **15** | Trần tốc độ yaw (°/s) — giữ 15 cho chuyến đầu |
| `--ramp` | 1.0 | Giây tăng dần trần yaw từ 0 sau ENGAGE |
| `--reacquire` | 4.0 | Giây mất dấu trước khi bỏ khoá hẳn |

**An toàn**

| Tham số | Mặc định | Ý nghĩa |
|---|---:|---|
| `--enable-control` | tắt | **Bắt buộc** để điều khiển thật, không có cờ này = chạy khô |
| `--rc-chan` | 0 | Kênh RC dead-man — **bắt buộc** cùng `--enable-control`; chỉ SITL/bench được dùng `--allow-no-rc-deadman` để bỏ qua |
| `--max-engage-s` | 20 | Tự ngắt sau N giây mỗi lần engage |
| `--loop-watchdog` | 1.0 | Vòng lặp im lặng quá N giây → ngắt |
| `--on-abort` | loiter | Hành động khi ngắt: `loiter` / `stream-off` / `hold-only` |

**Hàng rào mềm**

| Tham số | Mặc định | Ý nghĩa |
|---|---:|---|
| `--fence-radius` | 3.0 | Bán kính (m) từ điểm chốt lúc ENGAGE |
| `--fence-home-radius` | 0 (tắt) | Bán kính (m) từ điểm cất cánh — **nên đặt theo bãi thử** |
| `--fence-alt-max` / `--fence-alt-min` | 10 / 1.5 | Trần / sàn độ cao tuyệt đối (m) |
| `--fence-max-speed` | 1.5 | Tốc độ ngang tối đa (m/s) — yaw-only không được dịch chuyển |

**Đầu ra**

| Tham số | Mặc định | Ý nghĩa |
|---|---:|---|
| `--log-csv` | `follow_px4.csv` | File log |
| `--save-video` | — | Ghi video kèm hình debug |
| `--port` / `--bind` | 8080 / 127.0.0.1 | Địa chỉ web UI |
| `--no-web` | tắt | Tắt hẳn web UI (chạy hoàn toàn qua console) |

### 4.6. Đọc log sau mỗi lần chạy

Kiểm tra file CSV (`--log-csv`), đặc biệt các cột:

- `block` — lý do bị chặn tại mỗi khung hình, cột quan trọng nhất khi có
  sự cố.
- `fence`, `dist_m`, `alt_m`, `speed_ms` — phải là `ok` suốt thời gian bay.
- `sp_hz` — phải ≥ 18 trong suốt lúc `engaged=1`.
- `watchdog_trips`, `loop_wd_trips` — phải là **0**. Có giá trị khác 0 là
  phải điều tra ngay trước lần chạy tiếp theo.
- `yaw_cmd_deg_s` so với `yaw_thuc_deg_s` — phải bám sát nhau.

---

## 5. Việc cần làm trước khi bay thật

Theo thứ tự ưu tiên (xem chi tiết mục 8 trong báo cáo kiểm tra v2.1):

1. Đặt geofence cứng trên PX4 (`GF_MAX_HOR_DIST`, `GF_MAX_VER_DIST`,
   `GF_ACTION=Hold`) — hiện đang tắt hoàn toàn.
2. Chạy nốt kịch bản 7 (bay thật trong SITL, gió vs setpoint vị trí) và 8
   (bền 60 phút) trong `DroneFollowPX4.md` §7.2 — 5/8 kịch bản khác đã
   PASS thật với PX4 SITL. Kịch bản mất RC cần chuyển sang bench thật vì
   SITL không mô phỏng được (xem báo cáo kiểm tra v2.1 mục 6b–6c).
3. Đo `--fence-home-radius` theo bản đồ khu thử rồi bật lên.
4. **Bench tháo cánh với vị trí hợp lệ**: cần GPS lock (mang ra ngoài
   trời) hoặc module optical-flow MTF-01 (trong nhà) — lần bench trước bị
   PX4 từ chối arm đúng ở đây. Sau khi arm được: đo thời gian dead-man
   thật từ công tắc SWD (ch8) tới `yaw_cmd_deg_s = 0` (mục tiêu < 0.5s).
5. Hiệu chuẩn camera (`--fx-px`), đo trễ (`--lead`), rồi mới tune `--gain`.
   Nếu FPS camera thật thấp hơn kỳ vọng, thử `tools/camera_probe.py
   --buffer-size 1|2|3` trước khi nghi ngờ driver/USB — trên Rapoo C280 +
   UP7000, `buffersize=1` từng làm giảm một nửa FPS (đã sửa, mặc định
   hiện là 2, xem `--cam-buffer`).
6. Chuyến bay đầu: `--max-yaw-rate 15 --max-engage-s 10 --rc-chan 8
   --rc-min 1500` với công tắc AI Enable trên RC và `--fence-home-radius`
   đã đặt theo bãi thật.
