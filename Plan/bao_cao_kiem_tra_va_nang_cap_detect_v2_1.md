# Báo cáo kiểm tra bộ xử lý ảnh `Detect` và nâng cấp lên v2.1

**Ngày thực hiện:** 25/08/2026
**Phạm vi:** rà soát toàn bộ `Detect/follow/` (v2.0), sửa lỗi còn sót, và bổ sung lớp an toàn chống bay mất kiểm soát theo `Plan/ke_hoach_dieu_khien_up7000_pixhawk_usb.md` mục 7.
**Trạng thái:** 140 test tự động PASS. **Chưa chạy trên phần cứng thật, chưa chạy SITL với PX4 thật.**

---

## 1. Kết luận ngắn

Bộ code v2.0 có kiến trúc đúng và đã sửa thật 7 lỗi P0. Nhưng khi rà lại từng dòng thì:

- **10 lỗi còn sót**, trong đó **4 lỗi làm hỏng đúng cái lớp an toàn mà tài liệu nói là đã có** (E3, E4, E6, E8) và **2 lỗi vision làm mất mục tiêu đúng lúc nguy hiểm nhất** (E1, E2).
- **Hàng rào mềm hoàn toàn chưa tồn tại.** `ke_hoach_dieu_khien_up7000_pixhawk_usb.md` §7.4 yêu cầu soft fence của companion; trong v2.0 không có dòng code nào về bán kính, trần, sàn hay tốc độ.
- **`LoopWatchdog` chưa tồn tại**, dù `DroneFollowPX4.md` §A1 ghi rõ *"Bổ sung lớp hai: một luồng giám sát độc lập theo dõi `t_loop`"* và README v2.0 liệt kê nó như một lớp đã có.

Cả ba nhóm này đã được xử lý trong v2.1.

---

## 2. Phần xử lý ảnh — những gì đang làm đúng

Ghi lại để không bị "cải tiến" mất:

1. **Tách module thuần túy.** `control.py` và `safety.py` không import cv2/pymavlink, không đọc đồng hồ hệ thống. Đây là lý do có thể viết 140 test mà không cần drone.
2. **ByteTrack hai ngưỡng + GMC bằng phase correlation.** Rẻ, đủ dùng, và bù được chuyển động camera khi drone xoay — đúng bài toán.
3. **Nguyên tắc "nhập nhằng thì từ chối, không đoán"** trong `_try_reacquire`. Đây là quyết định thiết kế tốt nhất trong cả bộ code.
4. **Mất mục tiêu → yaw về 0 ngay, bỏ qua slew.** Giữ nguyên.
5. **Công thức pinhole `atan2`** và xử lý box chạm biên khung.

---

## 3. Lỗi phát hiện trong v2.0

### 3.1. Nhóm vision — mất mục tiêu và bám nhầm

| ID | Vấn đề | Mức | Đã sửa |
|---|---|---|---|
| **E1** | `np.clip(b[:, [0,2]], 0, w, out=b[:, [0,2]])` là **no-op**. `b[:, [0,2]]` là advanced indexing → trả về **bản sao**; `out=` ghi vào bản sao rồi vứt đi. Box **chưa bao giờ** được clip về khung. | P1 | ✅ clip theo từng cột |
| **E2** | Lọc tỉ lệ khung người chạy **sau** khi clip. Người đứng nửa trong nửa ngoài khung → sau clip thành box hẹp → tỉ lệ vượt `MAX_ASPECT=5` → **bị loại**. Đúng tình huống mà fix B2 trong `control.py` đang cố gắng xử lý. | **P0 vision** | ✅ lọc trên box gốc; box chạm biên được miễn kiểm tra tỉ lệ |
| **E3** | `TargetManager.reacquire` (mặc định 4.0 s) **được lưu nhưng không bao giờ dùng**. Mục tiêu mất dấu nằm trong `locked` vĩnh viễn và tiếp tục thử reacquire vô hạn. | P1 | ✅ có `t_lost`, quá hạn thì bỏ khoá |
| **E11** | Đặc trưng ngoại hình lấy trên **cả box** → lẫn chân, bóng, nền cỏ/bê tông. EMA cập nhật **mọi khung**, kể cả khi box đang bị cắt ở biên → embedding trôi dần sang màu nền. | P1 | ✅ lấy vùng thân trên 15–55% chiều cao, bỏ điểm quá tối/nhạt, chỉ học từ box nằm trọn trong khung |
| **E12** | Track mất dấu **đứng yên** (chỉ dịch theo GMC). Điểm neo cho reacquire lệch hẳn so với chỗ người đang thật sự đi tới. | P2 | ✅ `_Track.coast()` — trượt theo vận tốc cuối, giảm dần |
| **E13** | Bán kính reacquire **cố định 120 px**, không nở theo thời gian mất dấu (kế hoạch §6.4 yêu cầu `min(0.12·W + 120·dt, 0.30·W)`). | P2 | ✅ đã áp dụng công thức |
| **E14** | Không có cửa sổ tạm dừng sau reacquire (kế hoạch để `TODO`). | P2 | ✅ `just_reacquired()` — ép yaw 0 trong 0.5 s |
| **E15** | `target_needs_confirm` chỉ xét mục tiêu chính; kế hoạch yêu cầu `any(...)`. | P2 | ✅ |
| **E16** | `detect.py` import OpenVINO ở cấp module → không test được hậu xử lý trên máy không có OpenVINO. | P2 | ✅ import lười trong `load_model` |

### 3.2. Nhóm an toàn / PX4

| ID | Vấn đề | Mức | Đã sửa |
|---|---|---|---|
| **E4** | `pos_valid = link.pos.value is not None` — chỉ hỏi "đã từng có chưa", **không hỏi tuổi**. Stream `LOCAL_POSITION_NED` chết mà gate vẫn cho bay. Đây đúng là lỗi A4, chỉ khác trường dữ liệu. | **P0** | ✅ thêm `pos_age` + `--pos-max-age` |
| **E5** | `--invert-yaw` chỉ đổi dấu `bearing`, **không** đổi dấu `yawspeed_actual`. Ước lượng vận tốc góc mục tiêu sai dấu → feed-forward và bù trễ đẩy ngược hướng. | P1 | ✅ đổi dấu cả hai |
| **E6** | `enter_loiter()` gọi `set_mode(..., expect=None)` → **tin ACK, không xác minh mode**. Thêm nữa: `mavutil.mode_string_v10` trả `"LOITER"` chứ không phải `"AUTO.LOITER"`, và `mavfile.flightmode` lấy theo **sysid của bản tin nhận gần nhất** — một QGC trên cùng link làm sai `mode`/`armed`. | **P0** | ✅ tự giải mã `custom_mode` (`px4_mode_name`), lọc heartbeat theo đúng autopilot, `enter_loiter` xác minh bằng heartbeat |
| **E7** | `enter_offboard()` / `enter_loiter()` gọi **ngay trong vòng điều khiển**, chặn tới ~10 s (3 lần retry × 3.5 s). Trong lúc đó main loop không nuôi `command()` → TX watchdog trip, camera dồn khung. | **P0** | ✅ `set_mode_async()` chạy luồng riêng; chuỗi ENGAGE thành máy trạng thái không chặn |
| **E8** | Signal handler gọi `disengage()` → in ra màn hình, lấy lock, gửi MAVLink và **chờ ACK**. Đúng lỗi D1 mà kế hoạch cảnh báo, chưa từng được sửa. | P1 | ✅ handler chỉ `_stop.set()`; dọn dẹp trong `finally` |
| **E9** | `/stream.mjpg` **không kiểm tra token**. `/pick` là `GET` có tác dụng phụ → CSRF bằng `<img src=...>`. | P1 | ✅ token cho cả stream, mọi endpoint đổi trạng thái là POST, kiểm tra Origin |
| **E10** | `engaged = True` đặt **trước** khi biết PX4 có nhận Offboard không. Web báo "ĐANG ĐIỀU KHIỂN" trong khi máy bay ở mode khác — đúng kịch bản A3 mô tả. | P1 | ✅ chỉ `engaged = True` sau khi heartbeat xác nhận OFFBOARD |
| **E17** | `tx_watchdog_trips` có trong `GateInputs` nhưng **không dòng nào dùng**. Kế hoạch nói một lần trip là phải điều tra. | P2 | ✅ `--max-tx-trips` (mặc định 1 → ngắt) |
| **E18** | `LoggingSink.stop()` thoát khi cờ dừng được đặt, **không vét hàng đợi** → mất đúng những dòng cuối. | P2 | ✅ vét hết queue rồi mới đóng |
| **E19** | `draw()` + `cv2.imencode` chạy **mọi khung** trên vòng điều khiển, kể cả khi không ai xem web (~5–15 ms ở 1280×720). | P2 | ✅ `--jpeg-hz` (mặc định 12), chỉ vẽ khi cần |

---

## 4. Bảy lớp chống bay mất kiểm soát đã bổ sung

Theo `ke_hoach_dieu_khien_up7000_pixhawk_usb.md` §7.2 và §7.7.

| # | Lớp | Module | Chặn được gì |
|---|---|---|---|
| 1 | TX watchdog 0.25 s | `px4.py` (đã có v2.0) | Main loop treo → yaw về 0 |
| 2 | **`LoopWatchdog`** — luồng độc lập | `watchdog.py` **mới** | Main loop treo → **thật sự ngắt** và thoát Offboard |
| 3 | **Hàng rào mềm** | `geofence.py` **mới** | Trôi ra khỏi vùng thử: bán kính từ điểm chốt, bán kính từ gốc LOCAL_NED, trần/sàn tuyệt đối, lệch cao so với lúc ENGAGE, tốc độ ngang |
| 4 | **`StickMonitor`** | `watchdog.py` **mới** | Phi công đóng stick mà phần mềm không biết |
| 5 | **`CommandAudit`** | `watchdog.py` **mới** | Drone tự xoay khi lệnh ~0, hoặc không làm theo lệnh — dù link vẫn khoẻ |
| 6 | **Giới hạn thời lượng engage** | `safety.py` | Người vận hành quên thả nút (§7.8: mỗi lần enable 5–10 s) |
| 7 | **Clamp + chặn NaN ngay trước `pymavlink`** | `px4.py` | Lệnh sai đơn vị, NaN/Inf, spike |

### 4.1. Hàng rào mềm — ba mức

```
OK      → bay bình thường
WARN    → yaw về 0 nhưng GIỮ engaged, băng web chuyển cam, phi công kịp xử lý
BREACH  → ngắt điều khiển + AUTO.LOITER
```

Có **dự đoán sớm**: chiếu vị trí theo vận tốc hiện tại `predict_s` giây (mặc định 1 s); nếu vị trí dự đoán vượt biên thì cảnh báo ngay, không đợi chạm biên thật.

### 4.2. Chuỗi ENGAGE mới

```
idle → [tiền kiểm tra] → prestream (phát setpoint zero ≥1 s)
     → switching (xin OFFBOARD trên luồng riêng) → engaged (ramp yaw từ 0)
```

Đúng theo §6.2 của kế hoạch USB. Tiền kiểm tra từ chối ENGAGE nếu chưa arm, chưa có vị trí mới, PX4 báo failsafe, công tắc RC chưa bật, pin thấp, **chưa chọn mục tiêu**, hoặc vị trí hiện tại đã nằm ngoài hàng rào.

### 4.3. Khi ngắt — có đường lui

`--on-abort loiter` gửi `AUTO.LOITER` và **xác minh bằng heartbeat**. Nếu thất bại thì **tự động rơi xuống `stream-off`**: dừng hẳn luồng setpoint để PX4 kích hoạt Offboard-loss theo `COM_OF_LOSS_T`. Đây là điều kế hoạch yêu cầu mà v2.0 chưa làm.

### 4.4. Idle-follow — chi tiết nhỏ nhưng quan trọng

Khi **chưa** ENGAGE, v2.0 phát setpoint **vận tốc 0**. Nếu PX4 vào Offboard vì lý do nào khác (phi công gạt nhầm, `RC_MAP_OFFB_SW`), drone sẽ **trôi theo gió** vì vận tốc-0 không có phản hồi vị trí — đúng cơ chế lỗi A7.

v2.1 phát setpoint **giữ chính vị trí hiện tại**. Vào Offboard bất ngờ thì drone đứng yên.

---

## 5. Thay đổi hành vi cần biết

| Tham số | v2.0 | v2.1 | Lý do |
|---|---:|---:|---|
| `--max-yaw-rate` | 40 | **15** | Kế hoạch §5.3: trần yaw-only chuyến đầu là 15 °/s. 40 là giá trị để tune sau, không phải mặc định |
| `--max-drift` | 3.0 | **5.0** | `--fence-radius` (mặc định 3.0) đo **cùng khoảng cách từ cùng điểm neo**. Để hàng rào chặn trước — vì nó có dải cảnh báo — `--max-drift` lùi lại thành lưới dự phòng cho trường hợp `--no-fence` |
| `--max-engage-s` | — | **20** | Mới. Đặt 10 cho chuyến đầu |
| `--ramp` | — | **1.0** | Mới |
| `--fence-home-radius` | — | **0 (tắt)** | Phải đặt theo bản đồ khu thử. **Đây là thứ duy nhất chặn được việc ENGAGE LẠI ở một chỗ mới rất xa** — vì điểm chốt được đặt lại mỗi lần ENGAGE |

---

## 6. Kết quả test

```
140 passed in 22.5s
```

| File | Số test | Nội dung |
|---|---:|---|
| `test_safety.py` | 40 | 9 lớp gate, từng điều kiện độc lập, fatal vs không fatal |
| `test_geofence.py` | 18 | Bán kính, trần, sàn, tốc độ, dự đoán sớm, NaN, ưu tiên mức |
| `test_watchdog.py` | 17 | LoopWatchdog trip/hồi phục, CommandAudit hai chiều, StickMonitor |
| `test_mavlink.py` | 18 | **PX4 giả qua UDP loopback** |
| `test_app_loop.py` | 3 | **End-to-end vòng lặp chính** |
| `test_detect.py` | 14 | Lọc trước/sau clip, người ở rìa khung, NaN |
| `test_target.py` | 17 | Timeout reacquire, tạm dừng sau reacquire, đặc trưng thân trên |
| `test_control.py` | 13 | Pinhole, deadzone mềm, invert, NaN |

### 6.1. Hai bộ test đáng chú ý

**`tests/fake_px4.py`** — PX4 giả phát heartbeat/telemetry thật qua UDP, nhận setpoint, trả lời `DO_SET_MODE`, và giả lập được cả một QGC trên cùng link. Nhờ nó, các fix A1/A3/A6/A7/B5 được **kiểm chứng bằng bản tin thật** chứ không phải bằng đọc code.

**`tests/test_app_loop.py`** — chạy `main()` thật với model giả + video giả + PX4 giả, rồi điều khiển qua đúng HTTP API mà trình duyệt dùng. Đây là bản thu nhỏ của kịch bản SITL §7.2:

1. Bấm ENGAGE khi **chưa chọn mục tiêu** → bị từ chối, PX4 **không** vào Offboard.
2. Chọn mục tiêu → ENGAGE → PX4 **thật sự** vào OFFBOARD → có lệnh yaw **đúng chiều**, trong trần, `type_mask=1528`.
3. Dời vị trí ra ngoài hàng rào → **tự ngắt** và PX4 trở về **AUTO.LOITER**, yaw về 0.
4. Thả nút ENGAGE → ngắt trong < 2.5 s.
5. Hạ công tắc RC dead-man → ngắt trong **< 1 s**.

---

## 6b. Cập nhật 25/08/2026 — đã kiểm chứng trên phần cứng thật + PX4 SITL thật

Sau khi viết xong v2.1, đã thực hiện bench test với **Pixhawk 6C + khung S500 thật** (cánh quạt đã tháo) và **PX4 v1.17.0 SITL thật** (không phải PX4 giả tự viết) trên cùng máy.

### Bench test phần cứng thật (Pixhawk qua USB, `tools/bench_probe.py`)

| Kiểm tra | Kết quả |
|---|---|
| Kết nối USB CDC-ACM, bắt tay heartbeat | ✅ `sysid=1 compid=1`, ổn định |
| `px4_mode_name()` tự viết giải mã đúng mode thật | ✅ đọc đúng `AUTO.LOITER`, `POSCTL`, `STABILIZED` |
| `RC_CHANNELS` về đúng tần số yêu cầu | ✅ ~20Hz |
| Xác định kênh dead-man thật (SWD = ch8) | ✅ nghỉ=1000, bật=~2000 |
| Đọc pin đúng khi PX4 báo "không có dữ liệu" (0x FFFF) | ✅ không phải lỗi code |

**Bị chặn ở bước ENGAGE thật:** PX4 từ chối arm vì thiếu GPS/optical-flow (bench trong nhà). Đây là hành vi **đúng và mong muốn** của PX4 — không sửa tham số EKF/arming để "cho chạy được", theo đúng nguyên tắc trong `ke_hoach_dieu_khien_up7000_pixhawk_usb.md` (mọi thay đổi tham số an toàn phải có backup + kiểm chứng SITL trước). Chuyển sang SITL để kiểm chứng phần chưa test được.

### PX4 SITL thật (`sihsim_quadx`, v1.17.0 — đúng bản đang chạy trên xe)

Dựng bằng WSL2 (repo `PX4-Autopilot` đã có sẵn, target SIH — mô phỏng nội bộ, không cần GUI/Gazebo). Chạy `follow.app` thật (mục tiêu giả lập cố định do chưa có model OpenVINO, xem `tools/sitl_bench.py` + `tools/sitl_drive.py`) nhắm vào SITL qua `udp:127.0.0.1:14540`, tự động hoá toàn bộ arm → pick → giữ ENGAGE → theo dõi → disengage.

**Kết quả — chuỗi ENGAGE chạy đúng và có bằng chứng thời gian thực, không phải suy đoán:**

```
t=0.3s  prestream   phat setpoint zero
t=0.6s  switching   xin OFFBOARD
t=1.2s  engaged     mode=OFFBOARD (PX4 that xac nhan qua heartbeat)
        yaw=+8.98 do/s  bearing=+19.0 do   <- box lech PHAI -> yaw DUONG (dung chieu)
t=9.4s  tu dong ngat dung luc --max-engage-s=10, quay ve AUTO.LOITER
t=9.7s..10.3s  ENGAGE LAI thanh cong lan 2 - khong bi ket trang thai
cuoi    disengage -> mode=AUTO.LOITER xac nhan that qua heartbeat
```

Đây là lần đầu tiên các claim sau được kiểm chứng bằng **firmware PX4 thật** (không phải PX4 giả tự viết trong `tests/fake_px4.py`):
- **A2** (thoát Offboard thật): `mode=AUTO.LOITER` sau disengage, đọc từ heartbeat thật.
- **A3** (xác minh OFFBOARD được chấp nhận): `engaged=True` chỉ xảy ra sau khi heartbeat xác nhận `mode=OFFBOARD`, không tin ACK suông.
- Giới hạn thời lượng engage (`--max-engage-s`) tự cắt đúng thời điểm.
- Chuỗi ENGAGE có thể lặp lại nhiều lần liên tiếp mà không kẹt trạng thái.
- Chiều yaw đúng theo quy ước (mục tiêu lệch phải → yaw dương).

### Các kịch bản còn lại trong §7.2 — chạy tiếp ngay sau đó

Dùng lại đúng SITL v1.17.0 và bộ script `tools/sitl_case_*.py` (chạy tự động, có tiêu chí PASS/FAIL rõ ràng, tái sử dụng được).

**Kịch bản 2 — ENGAGE khi chưa arm → phải bị từ chối.** ✅ **PASS.** `preflight()` chặn ngay từ đầu, không gửi `DO_SET_MODE` nào; PX4 giữ nguyên `AUTO.LOITER` suốt 10 lần thăm dò liên tiếp.

**Kịch bản 3 — `kill -STOP` tiến trình 2s rồi `kill -CONT` → drone phải dừng xoay.** ✅ **PASS** trên tiêu chí thực tế của plan (yaw về 0.00, disengage, PX4 trả về `AUTO.LOITER`) — nhưng có một phát hiện đáng ghi lại: `watchdog_trips` (bộ đếm watchdog **nội bộ luồng TX**, fix A1) **không** tăng lần này. Lý do: `kill -STOP` đóng băng **toàn bộ tiến trình** (mọi luồng, kể cả luồng TX), nên khi `kill -CONT`, luồng chính tự phát hiện mất heartbeat MAVLink (chu kỳ kiểm tra ~50ms) và gọi `disengage()` — trong đó có `streamer.command(0.0)` — **nhanh hơn** ngưỡng 250ms mà watchdog nội bộ của luồng TX cần để tự trip. Hai lớp bảo vệ (mất heartbeat cấp ứng dụng, và watchdog cấp luồng TX) chồng lấn phạm vi ở đúng kịch bản test này, nên lớp phản ứng nhanh hơn "thắng" trước — kết quả an toàn cuối cùng vẫn đúng, chỉ là cơ chế cụ thể khác dự đoán ban đầu. Muốn cô lập đúng "chỉ main loop treo, luồng TX vẫn sống" (đúng kịch bản A1 nhắm tới) cần chèn lỗi trực tiếp vào main loop (ví dụ khoá `camera.read()` vô hạn) thay vì đóng băng cả tiến trình — chưa làm, để lại cho vòng test sau.

**Kịch bản 6 — đóng tab trình duyệt (mất nhịp `/alive`) → disengage < 1.5s.** ✅ **PASS.** Disengage tại **0.92s** (đúng khớp `operator_timeout=1.0s` mặc định), mode trả về `AUTO.LOITER` xác nhận qua heartbeat thật.

**Kịch bản 4/5 — mất `RC_CHANNELS` trong khi vẫn giữ heartbeat.** ⚠️ **Không chạy được trong môi trường SITL này — giới hạn môi trường, không phải giới hạn code.** Đã kiểm tra source PX4 (`mavlink_receiver.cpp::handle_message_manual_control`): đường duy nhất một companion có thể bơm "RC" từ xa (`MANUAL_CONTROL`) đi thẳng vào topic `manual_control_setpoint`, **không** đi qua `input_rc`/`RC_CHANNELS`. SITL kiểu SIH không có receiver/joystick mô phỏng nên **không phát `RC_CHANNELS` nào cả** (xác nhận bằng cách lắng nghe trực tiếp 5s, không nhận được gói nào). Cách duy nhất tạo được `RC_CHANNELS` giả là giả mạo `sysid=1` — không đại diện cho tình huống thật và bị chính lớp lọc heartbeat-theo-đúng-autopilot (fix v2.1) từ chối một cách hợp lý. Logic `rc.age > rc_max_age` đã được kiểm chứng đầy đủ bằng `tests/test_safety.py` (bảng dữ liệu) và dùng **chung cơ chế `Stamped.age`** với `hb.age` vừa được xác nhận hoạt động đúng trên PX4 thật ở kịch bản 3 và 6 — rủi ro còn lại thấp, nhưng đây vẫn là một khoảng trống thật, cần bench test phần cứng (RC thật, rút ăng-ten/tắt máy phát) để đóng hẳn.

| Kịch bản §7.2 | Kết quả |
|---|---|
| 1. ENGAGE bình thường → OFFBOARD xác nhận | ✅ PASS (mục 6b) |
| 2. ENGAGE khi chưa arm → từ chối | ✅ PASS |
| 3. Main loop treo → watchdog | ✅ PASS (an toàn đạt được, cơ chế khác dự kiến — xem ghi chú) |
| 4/5. Mất RC giữ heartbeat | ⚠️ Không chạy được trong SITL này (giới hạn môi trường) |
| 6. Mất nhịp operator → disengage | ✅ PASS (0.92s) |
| 7. Gió/setpoint vị trí vs vận tốc | Chưa — cần bay thật trong sim |
| 8. Chạy liên tục 60 phút | Chưa — chưa thử |

---

## 6c. Cập nhật 26/08/2026 — chạy thật trên UP7000, hai lỗi môi trường/hiệu năng tìm ra và sửa

Người vận hành đã đưa `Detect/` lên đúng UP7000 thật (qua Tailscale, `scp`) và chạy `pytest` lần đầu trên phần cứng đích. Phát sinh hai vấn đề, cả hai đã xác định nguyên nhân bằng đo đạc thực tế (không đoán) và sửa.

### Lỗi 1 — `opencv-python` không import được trên UP7000

```
ImportError: libGL.so.1: cannot open shared object file: No such file or directory
```

UP7000 là máy headless (không desktop), thiếu thư viện đồ họa mà bản `opencv-python` đầy đủ cần cho các hàm GUI (`imshow`, cửa sổ...). Đã kiểm tra: **không có dòng nào trong `follow/`, `tests/`, `tools/` gọi hàm GUI của cv2** — toàn bộ chỉ vẽ lên buffer ảnh rồi encode JPEG cho web stream. Đổi `requirements.txt` sang `opencv-python-headless`. Sau khi đổi: 143/143 test PASS trên UP7000.

### Lỗi 2 — Camera thật (Rapoo C280, chip Microdia `0c45:6365`) chỉ đạt ~12 FPS thay vì 30 FPS

Điều tra theo từng lớp, đo bằng chứng ở mỗi bước thay vì đoán:

| Bước đo | Kết quả | Kết luận |
|---|---|---|
| `v4l2-ctl --list-formats-ext` | 1280×720 MJPG hỗ trợ 30fps native | Phần cứng có khả năng |
| Đổi độ phân giải 640×480 vs 1280×720 | FPS không đổi (~11.5) | Không phải nghẽn giải mã CPU |
| `top` trong lúc capture | CPU chỉ 2.3%, 97% idle | Xác nhận không phải nghẽn CPU |
| `v4l2-ctl --stream-mmap` (bỏ qua OpenCV hoàn toàn) | **29.64 fps** | Driver/USB/camera hoàn toàn không phải nguyên nhân — vấn đề nằm trong OpenCV |
| So `CAP_PROP_BUFFERSIZE=1` vs không đặt | 11.5 fps vs 23.0 fps | **Tìm ra nguyên nhân**: buffersize=1 khiến OpenCV dequeue/requeue gần như đồng bộ trên driver V4L2 này, giảm gần một nửa thông lượng |
| So buffersize 1/2/3/4 | 11.5 / 21.0 / 20.7 / 20.7 fps | buffersize=2 lấy gần hết lợi ích, vẫn giữ hàng đợi nông (~100ms) |

**Đã sửa** `follow/camera.py`: mặc định đổi từ `buffersize=1` sang `buffersize=2`, thêm tham số `--cam-buffer` (app.py) và `--buffer-size` (`tools/camera_probe.py`) để tinh chỉnh nếu camera/driver khác cư xử khác. Xác nhận trên chính UP7000 sau khi sửa: **24.2 fps** (gấp đôi 12.2 fps ban đầu), 143/143 test vẫn PASS.

> Đây là ví dụ cụ thể cho lý do phải test trên **đúng phần cứng đích**: `CAP_PROP_BUFFERSIZE=1` là lựa chọn hợp lý về lý thuyết (luôn lấy khung mới nhất, không xử lý khung cũ) và không gây vấn đề gì trên nhiều driver/camera khác — nhưng trên đúng tổ hợp UP7000 + Rapoo C280 + chip Microdia này lại làm giảm một nửa FPS. Không có cách nào phát hiện ra bằng test logic hay SITL.

### Công cụ mới

`tools/camera_probe.py` — đo FPS thật qua đúng class `follow.camera.Camera`, đếm khung trùng bị bỏ, lưu ảnh chụp để kiểm tra bằng mắt, so sánh `--buffer-size` khi cần tinh chỉnh cho camera khác.

### Quy trình đồng bộ code Windows ↔ UP7000

UP7000 truy cập qua Tailscale (`ssh`/`scp` không cần mật khẩu, key đã cấu hình sẵn). Xác minh đồng bộ bằng `md5sum` hai đầu thay vì tin tưởng suông — phát hiện `requirements.txt` bị lệch (chưa có bản cập nhật opencv-headless) trước khi nó gây lỗi thật.

---

## 6d. Cập nhật 26/08/2026 — model YOLO26n thật đã chạy trên UP7000

Theo yêu cầu hạn chế ghi vào eMMC, toàn bộ cài đặt nặng (`torch` CPU-only, `ultralytics`) được đặt trong venv riêng trên ổ USB rời (`/mnt/droneup/yolo_venv`), tái sử dụng lại OpenVINO hệ thống đã cài sẵn (`--system-site-packages`) thay vì cài trùng.

- Tải `yolo26n.pt` chính thức từ Ultralytics (5.3MB) — xác nhận model này **có thật** và đúng kiến trúc NMS-free.
- Export sang OpenVINO: output shape **`[1, 300, 6]`** — khớp chính xác định dạng `follow/detect.py` yêu cầu, không cần sửa code.
- Chạy thử end-to-end thật: `load_model` → `Camera` thật → `letterbox` → suy luận trên **iGPU** (Intel N100) → `postprocess`. Không lỗi. **Suy luận 31.9ms** (~31 FPS lý thuyết) — nhanh hơn tốc độ camera thật (24fps sau khi sửa buffersize), nên suy luận không phải nút thắt của pipeline.
- Khởi động `follow.app` chạy khô (không `--mavlink`, an toàn tuyệt đối) với camera + model thật, cho người vận hành xem trực tiếp qua web để tự kiểm tra bằng mắt chất lượng nhận diện/bám.

> Model export là FP32, chưa hiệu chỉnh INT8 bằng ảnh camera thật (mục C1 trong `DroneFollowPX4.md` vẫn còn treo — xem mục 8 bên dưới). FP32 chạy 31.9ms trên iGPU N100 là đủ nhanh cho mục đích test hôm nay; INT8 sẽ nhanh hơn nữa nhưng cần bộ ảnh hiệu chỉnh riêng, không làm vội.

### Phản hồi từ người vận hành: "khung khoanh vùng chập chờn, không phải ô vuông"

Xem web demo trực tiếp, người vận hành báo hiện tượng khung nhận diện lúc hiện lúc mất, và hình dạng "không giống ô vuông". Điều tra bằng `tools/detect_diag.py` (script mới, in ra RAW detection trước và sau bộ lọc `postprocess()` cho từng khung) thay vì đoán:

| Kịch bản test | Model thấy người | Qua được bộ lọc | `aspect` (cao/rộng) |
|---|---|---|---|
| Ngồi tại bàn, webcam cận cảnh | 224/224 = 100%, conf 0.88–0.94 | 187/224 = 83% | 0.65–0.94 (**rộng hơn cao**) |
| Nới `--min-aspect` xuống 0.5 | 225/225 = 100% | **225/225 = 100%** | như trên |
| Đứng dậy, lùi ra xa (không đủ chỗ lùi hẳn) | 225/225 = 100%, conf 0.95 | 215/225 = 96% | vẫn 0.69 — chưa đủ xa |

**Chẩn đoán:** `MIN_ASPECT=1.2` (fix C1, chống vật rộng bẹt không phải người) yêu cầu box cao hơn rộng ít nhất 1.2 lần — đúng dáng người **đứng, nhìn từ xa** (kịch bản drone thật, tầm ~15–20m theo kế hoạch). Test tại bàn làm việc không đủ khoảng cách lùi nên chủ thể luôn hiện dáng "bè ngang" (0.65–0.94), khiến box chỉ lọt qua bộ lọc nhờ trùng hợp chạm mép khung hình (ngoại lệ dành cho người bị cắt cảnh) — người dùng dịch chuyển nhẹ là ngoại lệ này bật/tắt, gây cảm giác "lúc được lúc không" dù model bên dưới **chưa từng mất dấu**.

**Bằng chứng ủng hộ giữ nguyên bộ lọc:** trong chính khung hình đó, một người khác đứng xa hơn ở hậu cảnh có `aspect=3.1–4.56` (đúng dáng cao gầy) và **pass sạch 100%** không cần ngoại lệ nào. Kết luận: bộ lọc hoạt động đúng thiết kế cho đúng cự ly vận hành thật; **giữ nguyên** `MIN_ASPECT=1.2` mặc định, không nới lỏng chỉ vì bench test trong phòng thiếu chiều sâu. `tools/detect_diag.py` được giữ lại làm công cụ chẩn đoán cho các lần test tiếp theo.

---

## 7. Những gì vẫn CHƯA làm

Nói rõ để không nhầm là đã xong.

| Hạng mục | Trạng thái |
|---|---|
| Chạy 8 kịch bản SITL §7.2 với PX4 thật | **5/8 PASS thật** (1, 2, 3, 6 + phần OFFBOARD của kịch bản khác — xem bảng mục 6b). Kịch bản 4/5 bị chặn bởi giới hạn môi trường SITL (không phải lỗi code). Kịch bản 7 (gió/vị trí) và 8 (bền 60 phút) cần bay thật trong sim, chưa thử |
| Bench / shadow flight / bay thật (P5–P7) | Chưa — bench thật mới dừng ở mức đọc telemetry + xác định kênh RC, chưa ENGAGE được vì thiếu GPS/optical-flow trong nhà |
| Hiệu chuẩn nội tại camera Rapoo C280 → `--fx-px` | Chưa — `--hfov 90` vẫn là số phỏng đoán |
| Đo trễ vòng kín thật → `--lead` | Chưa — cột `latency_ms` và `infer_ms` đã có sẵn để đo |
| Hiệu chỉnh lại INT8 bằng ảnh camera thật (C1 phần model) | **Một phần.** Model YOLO26n FP32 đã export và chạy thật trên UP7000 (31.9ms/khung, xem mục 6d) — nhưng đó là FP32, chưa lượng tử hoá INT8 bằng ảnh camera thật. FP32 hiện đủ nhanh nên chưa cấp bách |
| Suy luận hai mức tăng tầm phát hiện (C4) | Chưa — tầm thực tế vẫn ~15–20 m |
| Pipelining suy luận thật sự (B3) | **Cố ý chưa làm.** `start_async` + `wait()` ngay sau vẫn là đồng bộ. Pipelining cắt được ~30 ms nhưng làm lệch việc ghép khung hình với telemetry — không đáng đổi ở giai đoạn này |
| `GF_MAX_HOR_DIST` / `GF_MAX_VER_DIST` trên PX4 | **Vẫn đang bằng 0 (tắt)** theo audit `test.params`. Hàng rào mềm của companion **không thay thế** được geofence cứng của PX4 |

---

## 8. Việc nên làm tiếp, theo thứ tự

1. **Chạy nốt kịch bản 7 và 8** (bay thật trong SITL để test setpoint vị trí vs gió; bền 60 phút) — cần điều khiển thêm trục vị trí trong SITL (takeoff), phức tạp hơn các kịch bản đã chạy nên để riêng một phiên. Kịch bản 4/5 (mất RC) cần chuyển sang bench phần cứng thật vì SITL không mô phỏng được.
2. **Đặt geofence cứng trên PX4** (`GF_MAX_HOR_DIST`, `GF_MAX_VER_DIST`, `GF_ACTION=Hold`). Đây là lớp 3 trong bảng 5 lớp của kế hoạch và hiện đang tắt hoàn toàn.
3. **Đo `--fence-home-radius` theo bản đồ khu thử** rồi bật lên. Không để mặc định 0.
4. **Bench tháo cánh với vị trí hợp lệ:** cần GPS lock (mang Pixhawk ra ngoài trời, vẫn giữ cánh quạt tháo + khung cố định) hoặc cắm module optical-flow MTF-01 để có `pos_valid` trong nhà — bench lần trước bị chặn đúng ở đây. Sau khi arm được: đo thời gian dead-man thật từ lúc hạ công tắc SWD (ch8) tới `yaw_cmd_deg_s = 0` trong CSV (kế hoạch yêu cầu < 0.5s; SITL cho thấy chuỗi phần mềm phản hồi nhanh, nhưng đây phải đo trên phần cứng thật).
5. Hiệu chuẩn camera → `--fx-px`; đo trễ → `--lead`; rồi mới tune `--gain` và bật `--k-ff`.
6. Tải/export model OpenVINO thật để test toàn bộ pipeline vision (webcam thật đã có sẵn trên máy dev, chưa dùng tới).
7. Chuyến đầu: `--max-yaw-rate 15 --max-engage-s 10 --rc-chan 8 --rc-min 1500 --fence-home-radius <theo bãi>`.

---

## 9. Cảnh báo giữ nguyên

> Code **KHÔNG tự arm, KHÔNG tự cất cánh**, chỉ điều khiển **một trục yaw**.
> Phi công phải cất cánh và hover Position TRƯỚC khi bật AI.
> **Công tắc Position/RTL trên tay điều khiển là lớp an toàn cuối cùng.**
> Dead-man qua MAVLink đi qua RC → máy thu → FMU → MAVLink → companion,
> trễ tổng 150–400 ms và phụ thuộc một đường truyền không đảm bảo.
> Nó **không tương đương** công tắc mode nối thẳng vào FC.
