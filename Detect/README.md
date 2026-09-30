# Drone PX4 — Xoay bám theo người được chọn v2.4

Hệ thống xử lý ảnh + điều khiển yaw cho drone S500 qua MAVLink/PX4.

- **v2.0** sửa 7 lỗi P0 + phần lớn P1/P2 theo `Plan/DroneFollowPX4.md`.
- **v2.1** bổ sung các lớp chống bay mất kiểm soát và sửa lỗi vision/PX4 còn sót.
- **v2.2** bổ sung đường **emergency stream-off độc lập với main loop**, bắt buộc RC dead-man khi bật control, và cấm tự đổi người sau khi target bị mất/re-ID.
- **v2.3** ép MAVLink2 (sửa lỗi âm thầm khiến `ODOMETRY` không bao giờ tới),
  thêm phát hiện **EKF reset** (`reset_counter`) trong khi ENGAGE, thêm kiểm
  tra **xung đột kênh RC dead-man với `RC_MAP_*` của PX4** lúc khởi động, và
  `--quiet-status` để giảm spam console khi chạy nền/ghi log.
- **v2.4** thêm `--cam-rot` bù camera lắp xoay trên khung drone (và cảnh báo
  `--hfov` phải đổi theo), cùng bộ công cụ SITL kiểm chứng dead-man RC
  end-to-end với PX4 thật.

> ⚠️ Code **KHÔNG tự arm, KHÔNG tự cất cánh**, chỉ điều khiển **một trục yaw**.
> Phi công phải cất cánh và hover Position TRƯỚC khi bật AI.
> Công tắc Position/RTL trên tay điều khiển là **lớp an toàn cuối cùng** —
> dead-man qua MAVLink chỉ là lớp tiện lợi (trễ 150–400 ms).

## Phần cứng

| Khối | Chi tiết |
|---|---|
| Khung | S500 |
| FC | Pixhawk 6C, PX4 v1.17 |
| Companion | UP7000 32GB/4GB |
| Camera | Rapoo C280 (USB) |
| Motor | SunnySky X2216 950kV |
| RC | FS-iA6B |
| Nguồn | PM07, Ovonic 4S 6200mAh |

## Cấu trúc

```
Detect/
├── follow/
│   ├── camera.py          Camera USB: seq + timestamp
│   ├── detect.py          OpenVINO YOLO + lọc trước khi clip
│   ├── track.py           GMC + ByteTrack (track mất dấu vẫn trượt theo vận tốc)
│   ├── target.py          LockedTarget + Appearance HSV thân trên + timeout
│   ├── control.py         YawController (thuần túy, không I/O)
│   ├── safety.py          SafetyGate 9 lớp (thuần túy, không I/O)
│   ├── geofence.py        Hàng rào mềm (thuần túy, không I/O)      ← v2.1
│   ├── watchdog.py        LoopWatchdog + CommandAudit + StickMonitor ← v2.2
│   ├── px4.py             PX4Link + SetpointStreamer (MAVLink)
│   ├── webui.py           HTTP + token + dead-man + bảng an toàn
│   ├── logging_sink.py    Ghi CSV/video tách luồng
│   └── app.py             Vòng lặp chính
└── tests/                 186 test, không cần phần cứng
    ├── fake_px4.py        PX4 giả qua UDP loopback
    ├── test_app_loop.py   End-to-end: pick → engage → fence breach → LOITER
    └── ...
```

## Cài đặt

```bash
pip install -r requirements.txt
```

## Chạy

### Chạy khô (không MAVLink, không điều khiển)

```bash
python -m follow.app --camera 0 --model yolo26n_int8_openvino_model
```

### Kết nối SITL

```bash
python -m follow.app --mavlink udp:127.0.0.1:14540 --enable-control --rc-chan 7
```

### Kết nối Pixhawk USB thật

```bash
python -m follow.app --mavlink /dev/serial/by-id/usb-Holybro_Pixhawk6C_xxx --enable-control --rc-chan 7 --rc-min 1500 --batt-min 14.0 --max-yaw-rate 15 --max-engage-s 10 --fence-home-radius 30 --on-abort loiter
```

### Chạy test

```bash
python -m pytest tests/ -v
```

## Chín lớp chống bay mất kiểm soát (v2.3)

Mỗi lớp độc lập; **không được bỏ lớp nào vì lớp khác đã có**.

| # | Lớp | Chặn được gì | Tham số |
|---|---|---|---|
| 1 | **TX watchdog** 0.25 s trong `SetpointStreamer` | Main loop treo → drone xoay mãi | cố định |
| 2 | **LoopWatchdog + emergency stream-off** | Main loop kẹt → TX tự gửi yaw=0 một lần rồi dừng stream; PX4 xử lý Offboard-loss, không chờ main loop hồi phục | `--loop-watchdog` |
| 3 | **Hàng rào mềm** — bán kính / trần / sàn / tốc độ, có dải cảnh báo và dự đoán | Drone trôi ra khỏi vùng thử | `--fence-*` |
| 4 | **Phi công đóng stick** | Người lái giành quyền mà phần mềm không biết | `--stick-chans`, `--stick-th` |
| 5 | **CommandAudit** — đối chiếu lệnh yaw với yawspeed thật | Drone tự xoay / không làm theo lệnh dù link vẫn khỏe | `--audit-tol` |
| 6 | **Giới hạn thời lượng engage** | Người vận hành quên thả nút | `--max-engage-s` |
| 7 | **Clamp + chặn NaN ngay trước `pymavlink`** | Lệnh sai đơn vị, NaN, spike | `--max-yaw-rate` |
| 8 | **Identity confirmation** | Mục tiêu mất ID/re-ID hoặc nhập nhằng → yaw=0, phải click xác nhận; không tự chuyển M1 sang M2 | — |
| 9 | **EKF reset detection** (`ODOMETRY.reset_counter`) | EKF đặt lại vị trí/vận tốc/hướng giữa chừng ENGAGE (đổi gốc tọa độ) mà các lớp khác không thấy vì `pos_valid`/`xy_pos_healthy` vẫn "đúng" | tự động khi có MAVLink; xem mục "EKF reset" bên dưới |

Ba lớp nữa **không** nằm trong code này và vẫn bắt buộc phải có:
PX4 Offboard-loss (`COM_OF_LOSS_T`), geofence cứng của PX4 (`GF_*`),
và công tắc Position/RTL trên RC.

## Camera lắp xoay — `--cam-rot` và `--hfov` (v2.4)

Nếu camera được lắp xoay trên khung drone, **bắt buộc** phải bù lại bằng
`--cam-rot`, vì `YawController` tính sai số góc từ toạ độ **ngang (trục x)**
của mục tiêu. Camera xoay 90° mà không bù:

- Người đi **sang trái/phải** thật → trong ảnh là đi **lên/xuống** → app
  không thấy lệch ngang, **không ra lệnh xoay**.
- Thứ gì làm đổi vị trí **dọc** trong ảnh (drone chúi/ngẩng, người tiến lại
  gần) → bị hiểu nhầm thành lệch ngang → **ra lệnh yaw sai**.
- Bộ lọc `--min-aspect`/`--max-aspect` (mặc định 1.2–5.0, dành cho người
  đứng thẳng) loại oan vì người nằm ngang cho aspect < 1.

`--cam-rot` xoay ảnh **ngay tại nguồn** (trong `follow/camera.py`), nên
detect/track/control/HUD/video đều làm việc trên ảnh đã đúng chiều.
`cam.width`/`cam.height` tự đổi chỗ khi xoay 90/270.

### Cấu hình thực tế của S500 hiện tại: `--cam-rot 180`

Đo trên phần cứng thật ngày 2026-09-04 (drone đặt đúng tư thế cất cánh, 4
chân chạm đất): camera lắp **lộn ngược 180°** — ảnh thô cho người đầu ở dưới,
chân ở trên.

```
--cam-rot 180        # --hfov giữ nguyên 90, KHÔNG cần đổi
```

Xoay 180° **không đổi chỗ rộng/cao**, khung vẫn 1280×720 ngang, nên góc nhìn
ngang vẫn là 90° và `--hfov` không phải chỉnh.

Hiệu quả đo được (`tools.detect_diag`, 12 s, cùng một người đứng yên):

| Cấu hình | Nhận diện thô | Qua bộ lọc | Độ tin cậy |
|---|---:|---:|---:|
| Sai chiều (ảnh xoay 90°) | 9% | 9% | 0.27–0.50 |
| **Đúng chiều (`--cam-rot 180`)** | **100%** | **100%** | **0.95–0.96** |

Chênh lệch hơn 10 lần — YOLO được huấn luyện với người đứng thẳng.

### ⚠️ Nếu camera lắp xoay 90/270 thì PHẢI đổi `--hfov`

Xoay không làm đổi góc nhìn vật lý của camera — nó chỉ đổi **trục nào nằm
ngang**. Với Rapoo C280 1280×720 và `--hfov 90` (đo trên trục 1280 px):

| | Trục ngang | Góc nhìn ngang |
|---|---:|---:|
| Không xoay / xoay 180 | 1280 px | 90° |
| Xoay 90/270 | 720 px | **58.7°** |

Để nguyên `--hfov 90` khi xoay 90/270 khiến app **tính vượt sai số góc
~1.5–1.8 lần** → ra lệnh yaw quá tay → vọt lố và dao động. Khi đó phải dùng
`--hfov 58.7`, và chấp nhận mất ~35% góc nhìn ngang (điều rất bất lợi cho
bài toán bám người theo trục yaw).

Cách tốt nhất vẫn là **hiệu chuẩn `--fx-px`** thay cho `--hfov` (khi đó
`--hfov` bị bỏ qua hoàn toàn, không phụ thuộc suy luận trên).

## Kiểm tra xung đột RC dead-man lúc khởi động (v2.3)

Khi `--enable-control` và `--rc-chan` được đặt, app đọc một lần (lúc khởi
động, có thể mất vài giây) các parameter `RC_MAP_ARM_SW`, `RC_MAP_KILL_SW`,
`RC_MAP_FLTMODE`, `RC_MAP_RETURN_SW`, `RC_MAP_OFFB_SW` trên PX4 và so với
`--rc-chan`. Lý do: nếu kênh dead-man của app trùng một `RC_MAP_*`, hạ dead-man
sẽ **đồng thời** đổi flight mode / kill motor / RTL trên PX4 — dead-man không
còn là một lớp độc lập nữa (đã xảy ra thật trên S500: CH7 vừa là dead-man tạm
thời vừa là `RC_MAP_FLTMODE`, xem `HANDOFF_CLAUDE_UP7000_2026-08-28.md` mục 9).

- Có xung đột xác nhận → app **từ chối mọi ENGAGE** (`preflight()` trả lý do
  rõ ràng), trừ khi có `--bench-allow-conflicting-deadman` (CHỈ SITL/bench).
- Không đọc được parameter trong thời gian chờ (PX4 không hỗ trợ / link
  chậm) → chỉ in cảnh báo, **không chặn** — vì "không đọc được" khác với
  "đã xác nhận sạch"; tự kiểm tra `RC_MAP_*` thủ công trước khi bay thật.

## EKF reset trong khi ENGAGE (v2.3)

`PX4Link` đăng ký thêm `ODOMETRY` (cần MAVLink2 — app tự ép `MAVLINK20=1`
trước khi import `pymavlink`, xem `follow/__init__.py`). `reset_counter` của
PX4 tăng mỗi lần EKF đặt lại vị trí/vận tốc/hướng. App chốt giá trị này tại
lúc ENGAGE (`arm_hold`); nếu nó đổi trong khi đang giữ ENGAGE, gate coi là
**fatal** → yaw về 0, disengage, PX4 rời Offboard (giống lỗi drift/fence).

Nếu PX4 chưa từng phát `ODOMETRY` (firmware/param không hỗ trợ), giá trị này
là `None` mãi mãi và lớp này **không chặn được gì** — không suy diễn ngầm
"không có dữ liệu" thành "không có reset". Cột CSV `ekf_reset_cnt` cho biết
có đang nhận được dữ liệu hay không; `ekf_reset_delta` phải luôn là 0.

### Chuỗi ENGAGE (không còn chặn vòng điều khiển)

```
idle → [tiền kiểm tra] → prestream (phát setpoint zero) → switching (xin OFFBOARD
       trên luồng riêng) → engaged (ramp yaw từ 0)
```

Tiền kiểm tra từ chối ENGAGE nếu: chưa arm, chưa có vị trí mới, PX4 báo failsafe,
công tắc RC chưa bật, pin thấp, **chưa chọn mục tiêu**, hoặc vị trí hiện tại đã
nằm ngoài hàng rào.

### Khi ngắt

`--on-abort loiter` (mặc định) gửi `AUTO.LOITER` và **xác minh bằng heartbeat**.
Nếu không vào được LOITER thì **tự động rơi xuống `stream-off`**: dừng hẳn luồng
setpoint để PX4 kích hoạt Offboard-loss theo `COM_OF_LOSS_T`.

## Tham số quan trọng

| Tham số | Mặc định | Ý nghĩa |
|---|---:|---|
| `--enable-control` | tắt | **BẮT BUỘC** để điều khiển thật |
| `--rc-chan` | 0 | **Bắt buộc** cùng `--enable-control`; chỉ SITL/bench mới được thêm `--allow-no-rc-deadman` |
| `--batt-min` | 0 | Khi `--enable-control`, phải đặt >0; chỉ SITL/bench mới được dùng `--allow-no-battery-check` |
| `--mavlink` | — | URL MAVLink (bỏ trống = chạy khô) |
| `--max-yaw-rate` | **15** | Trần tốc độ yaw (°/s) — chuyến đầu giữ 15 |
| `--ramp` | 1.0 | Giây tăng dần trần yaw từ 0 sau khi ENGAGE |
| `--max-engage-s` | 20 | Tự ngắt sau N giây mỗi lần engage (0 = tắt) |
| `--fence-radius` | 3.0 | Bán kính từ điểm chốt ENGAGE (cảnh báo từ 2.25 m) |
| `--fence-home-radius` | 0 | Bán kính từ gốc LOCAL_NED — **đặt theo bản đồ khu thử** |
| `--fence-alt-max` / `--fence-alt-min` | 10 / 1.5 | Trần và sàn tuyệt đối (m) |
| `--fence-alt-dev` | 3.0 | Độ lệch cao cho phép so với lúc ENGAGE |
| `--fence-max-speed` | 1.5 | Tốc độ ngang tối đa — yaw-only thì drone không được dịch chuyển |
| `--max-drift` | 5.0 | Lưới dự phòng khi chạy `--no-fence` |
| `--on-abort` | loiter | loiter / stream-off / hold-only |
| `--jpeg-hz` | 12 | Hạ xuống để vòng điều khiển nhanh hơn |
| `--cam-rot` | 0 | Xoay khung hình (0/90/180/270) bù camera lắp xoay. **Đổi giá trị này thì PHẢI đổi `--hfov`** — xem mục dưới |
| `--bench-allow-conflicting-deadman` | tắt | CHỈ SITL/bench: bỏ qua xung đột `--rc-chan` với `RC_MAP_*` đã xác nhận |
| `--quiet-status` | tắt (tự bật khi non-TTY) | Dòng trạng thái console còn 1 dòng/giây thay vì mỗi frame |

> `--fence-radius` và `--max-drift` đo **cùng một khoảng cách**. Hàng rào chặn
> trước vì có dải cảnh báo (yaw về 0 mà vẫn giữ engaged); `--max-drift` chỉ là
> lưới dự phòng cho trường hợp `--no-fence`.

Sau khi DISENGAGE và xác nhận PX4 đã rời OFFBOARD, luồng setpoint tự pause về
0 Hz. Nó chỉ resume để pre-stream khi một yêu cầu ENGAGE mới vượt đầy đủ
preflight; ứng dụng idle không duy trì tín hiệu OFFBOARD ngầm.

> Khi target mất dấu hoặc ByteTrack đổi ID, hệ thống **không tự bám người gần
> nhất và không tự chuyển sang target đang khóa khác**. HUD hiện ứng viên vàng
> `?XAC NHAN`; click trực tiếp vào ứng viên mới cho phép tiếp tục sau 0.5 s hold.

## Cột CSV cần soi sau mỗi chuyến

| Cột | Phải như thế nào |
|---|---|
| `block` | Lý do bị chặn — cột quan trọng nhất khi điều tra sự cố |
| `fence`, `dist_m`, `alt_m`, `speed_ms` | `fence` phải là `ok` suốt chuyến |
| `sp_hz` | ≥ 18 trong suốt lúc engaged |
| `watchdog_trips`, `loop_wd_trips` | **0**. Một lần trip là phải điều tra |
| `emergency_stops` | **0**. >0 nghĩa là stream đã bị cắt khẩn cấp; phải restart app và điều tra trước khi bay lại |
| `nan_blocks`, `clamp_hits` | 0 (clamp > 0 nghĩa là gain đang quá cao) |
| `yaw_cmd_deg_s` vs `yaw_thuc_deg_s` | Phải bám nhau, lệch < ~12 °/s |
| `latency_ms`, `infer_ms` | Dùng để đặt `--lead` |
| `rc_override` | 0 trừ lúc phi công cố ý giành quyền |
| `ekf_reset_delta` | Phải là **0** suốt chuyến. >0 nghĩa là EKF đã reset khi đang ENGAGE — app đã tự ngắt, nhưng phải điều tra nguyên nhân (xem mục 10 trong HANDOFF) trước khi bay lại |
| `ekf_reset_cnt` | Rỗng/không đổi suốt file → PX4 chưa từng phát `ODOMETRY`, lớp EKF-reset không có tác dụng thật (không phải "an toàn") |

## Lỗi đã sửa

### v2.0 — P0 an toàn

**A1** TX watchdog · **A2** thoát Offboard thật · **A3** xác minh ACK + heartbeat ·
**A4** dữ liệu RC/pin có tuổi · **A5** web có token + dead-man · **A6** `source_system=191` ·
**A7** setpoint giữ vị trí (`type_mask=1528`)

### v2.0 — P1/P2

**B1** atan2 · **B2** box chạm biên · **B3** `lead_s` · **B4** feed-forward ·
**B5** `time_boot_ms` · **B6** lịch tuyệt đối · **B7** deadzone mềm · **B8** stop latch ·
**C1** lọc tỉ lệ · **C2** HSV histogram · **C3** từ chối reacquire nhập nhằng · **C5** seq

### v2.1 — lỗi còn sót trong v2.0

| ID | Vấn đề | Hậu quả |
|---|---|---|
| E1 | `np.clip(b[:, [0,2]], ..., out=b[:, [0,2]])` là **no-op** (advanced indexing trả bản sao) | Box chưa bao giờ được clip về khung |
| E2 | Lọc tỉ lệ chạy **sau** khi clip | Người ở rìa khung bị loại oan — đúng lúc nguy hiểm nhất |
| E3 | `TargetManager.reacquire` **không được dùng** | Mục tiêu mất dấu nằm trong danh sách vĩnh viễn |
| E4 | `pos_valid` chỉ kiểm tra "có hay không", không kiểm tra tuổi | Stream vị trí chết mà gate vẫn cho bay — đúng lỗi A4 nhưng ở trường khác |
| E5 | `--invert-yaw` chỉ đổi dấu bearing, không đổi dấu `yawspeed` | Ước lượng vận tốc góc sai dấu khi bật invert |
| E6 | `enter_loiter()` không xác minh mode; `mavutil.mode_string_v10` trả `"LOITER"` chứ không phải `"AUTO.LOITER"`, và `mavfile.flightmode` lấy theo sysid **nhận gần nhất** | Ngắt điều khiển tưởng thành công; một GCS trên cùng link có thể làm sai mode/armed |
| E7 | `enter_offboard()` / `enter_loiter()` gọi **trong** vòng điều khiển, chặn tới ~10 s | TX watchdog trip, camera dồn khung |
| E8 | Signal handler in ra màn hình, lấy lock, gọi MAVLink | Nguy cơ deadlock khi Ctrl+C (lỗi D1 chưa sửa) |
| E9 | `/stream.mjpg` không kiểm tra token | Ai vào được cổng đều xem được hình |
| E10 | `engaged = True` đặt trước khi biết PX4 có nhận Offboard không | Web báo "ĐANG ĐIỀU KHIỂN" khi PX4 ở mode khác |

## Chưa làm (cần trước khi bay thật)

- Chạy đủ 8 kịch bản SITL trong `Plan/DroneFollowPX4.md` §7.2 với PX4 thật
- Hiệu chuẩn nội tại camera Rapoo C280 → `--fx-px`
- Đo trễ vòng kín thật → `--lead`
- Hiệu chỉnh lại INT8 bằng ảnh camera thật (C1 phần model)
- Suy luận hai mức để tăng tầm phát hiện (C4) — hiện ~15–20 m
- Đặt `GF_MAX_HOR_DIST` / `GF_MAX_VER_DIST` khác 0 trên PX4 (hiện đang tắt)
- **Chưa lắp cánh, chưa bay**: local-position/EKF từng đổi gốc giữa các lần
  arm/mode trên bàn (chưa rõ nguyên nhân) và CH7 trên RC hiện vừa là ứng viên
  dead-man vừa là `RC_MAP_FLTMODE` (không độc lập) — cả hai phải được xử lý
  xong trước lần active-test tiếp theo. Chi tiết đầy đủ + quy trình điều tra ở
  `HANDOFF_CLAUDE_UP7000_2026-08-28.md` mục 9, 10, 12.
