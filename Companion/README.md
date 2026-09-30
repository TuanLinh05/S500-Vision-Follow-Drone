# S500 Companion — UP 7000 ↔ Pixhawk 6C qua USB

Project này hiện thực các pha đầu của kế hoạch điều khiển:

- P1: tự tìm Pixhawk dưới `/dev/serial/by-id`, đọc MAVLink USB và ghi telemetry 30 phút.
- P2: telemetry snapshot có timestamp, CSV/JSONL recorder và validator PASS/FAIL.
- P3: safety supervisor/flight guard, yaw-only limiter, soft fence và fault tests.
- P4: transport Offboard + runner yaw-only dành riêng cho PX4 SITL.

Control trên Pixhawk thật đang khóa bằng hai cấu hình độc lập. Mặc định `config/vehicle.json` có `control_enabled=false` và `allow_serial_control=false`. Không có API arm, takeoff, land, kill hoặc actuator trong project.

## 1. Cài trên UP 7000

Yêu cầu Ubuntu 22.04 và Python 3.10 trở lên:

```bash
cd Companion
bash deploy/install_up7000.sh
```

Nếu user chưa có quyền đọc `/dev/ttyACM*`:

```bash
sudo usermod -aG dialout "$USER"
```

Đăng xuất/đăng nhập lại sau khi thêm group. Không chạy flight application thường xuyên bằng `sudo`.

## 2. P1 — USB read-only, bắt đầu tại đây

Tháo toàn bộ cánh, bật Pixhawk từ PM07, bật UP 7000 từ nguồn 12 V riêng rồi chạy:

```bash
cd Companion
. .venv/bin/activate
s500-companion discover
s500-companion probe --duration 10
```

`probe` chỉ nghe MAVLink, không gửi mode/setpoint/arm. Kết quả mong đợi có `system_id=1`, `connected=true`, mode hiện tại và dữ liệu GPS/pin.

Chạy bộ thu 30 phút và validator tự động:

```bash
export S500_PROPS_REMOVED=YES
bash deploy/run_p1_usb.sh
```

Script tạo bốn artifact trong `logs/`: danh sách USB, `dmesg`, CSV telemetry và JSON validation. Gửi các file đó để review trước khi chuyển P4/P5.

Audit một file parameter trước profile bay yaw-only:

```bash
s500-companion audit-params --input test.params --output logs/param_audit.json
```

Audit chỉ báo lỗi/cảnh báo; nó không sửa hoặc upload parameter lên Pixhawk.

## 2.5. P5a — kiểm tra lệnh mode trên Pixhawk thật, disarmed-only

Đây là bài test control đầu tiên trên Pixhawk thật. Nó chỉ gửi
`MAV_CMD_DO_SET_MODE` để chuyển sang `POSITION`, chờ `COMMAND_ACK` và
HEARTBEAT xác nhận, rồi trả về mode ban đầu. Công cụ không có arm, takeoff,
land, actuator, Offboard hoặc setpoint API; nó từ chối chạy nếu telemetry báo
`armed=true` tại bất cứ thời điểm nào.

Tháo toàn bộ cánh, để transmitter sẵn sàng override, và xác nhận drone đang
disarmed trong QGroundControl. Không thay đổi hai cờ `control_enabled` hoặc
`allow_serial_control` cho bài này. Trên UP 7000, sau khi cập nhật source:

```bash
cd ~/Companion
. .venv/bin/activate
python -m pip install -e .
s500-companion hardware-mode-test \
  --props-removed YES \
  --confirm DISARMED_MODE_TEST \
  --target-mode POSITION \
  --output logs/p5a_mode_test.json
```

Kết quả PASS chỉ có nghĩa USB MAVLink nhận lệnh mode, nhận ACK và khôi phục
mode thành công khi disarmed. Nó không cấp phép arm hay bay. Gửi file
`logs/p5a_mode_test.json` để review trước khi test tiếp.

## 3. Chạy test phần mềm P3

```bash
cd Companion
. .venv/bin/activate
python -m unittest discover -s tests -v
```

Test bao phủ: boot khi AI đang ON, frame/track stale, NaN, dịch chuyển bị cấm, yaw clamp, soft fence, mất local position, operator disable, scheduler gap và khóa control phần cứng.

## 4. Hợp đồng với detector/tracker

Detector gửi packet theo `docs/vision_packet_example.json`. Các timestamp phải dùng `time.monotonic()` trên cùng UP 7000. `flight_guard` từ chối packet:

- sai schema hoặc thiếu field;
- có NaN/Infinity;
- target ID khác target được chọn;
- frame/track/guidance quá hạn;
- chứa `vx/vy/vz` khác zero trong profile yaw-only.

Detector/tracker không được mở thiết bị MAVLink hoặc import module Offboard.

Chạy tích hợp shadow hoàn toàn read-only:

```bash
python tools/shadow_run.py --duration 600 --output logs/shadow.jsonl
```

Receiver chỉ bind `127.0.0.1:5800`; packet điều khiển từ mạng/4G không được chấp nhận. `ai_enable_rc_channel=0` mặc định khóa AI Enable cho tới khi xác định một kênh RC riêng và kiểm tra pulse trong log.

## 5. P4 — PX4 SITL yaw-only

Runner này từ chối đường serial và không arm/takeoff. Khởi động PX4 SITL, dùng QGroundControl đưa model cất cánh/hover Position, sau đó chạy trong cùng môi trường Linux:

```bash
cd Companion
. .venv/bin/activate
python tools/sitl_yaw_test.py \
  --enable-control \
  --confirm SITL_ONLY \
  --assume-sitl-rc \
  --yaw-rate 5 \
  --active-s 5
```

Không đổi `config/sitl_vehicle.json` sang một đường `/dev/serial/...`. Class transport có khóa riêng và sẽ từ chối serial thật khi `allow_serial_control=false`.

## 6. Trình tự để mở control trên drone thật

Không mở chỉ vì unit tests PASS. Cần đủ:

1. P1 USB 30 phút PASS và LR24/QGC không mất link.
2. P3 test suite PASS trên đúng UP 7000.
3. P4 SITL fault tests có log, đặc biệt kill process/rút stream/target stale.
4. P5 bench Pixhawk thật, tháo cánh; Position/RTL/RC override/geofence đã được xác nhận.
5. Hai shadow flight P6 không có output control.
6. Parameter diff an toàn đã review; PIC duyệt P7 yaw-only.

Tại P7 vẫn chỉ mở yaw, ép `vx=vy=vz=0` ngay trong supervisor. Việc chỉnh `control_enabled`/`allow_serial_control` không được thực hiện tại bãi bay nếu chưa có release và log thử tương ứng.

## 7. Cấu trúc chính

```text
src/s500_companion/
├── serial_discovery.py   # USB by-id
├── telemetry.py          # MAVLink read-only decoder
├── recorder.py           # CSV/JSONL
├── validation.py         # gate P1
├── models.py             # data contracts
├── vision_protocol.py    # input contract từ detector
├── safety.py             # state machine/interlocks
├── flight_guard.py       # owner duy nhất của output
├── offboard.py           # transport không có arm/takeoff
└── sitl_yaw.py           # runner SITL-only
```
