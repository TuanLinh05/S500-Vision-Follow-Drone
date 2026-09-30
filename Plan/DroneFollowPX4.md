# Phân tích & lộ trình hoàn thiện `follow_px4.py`

**Đối tượng đọc:** người tiếp nhận dự án **Trạng thái hệ thống:** đã chạy được với PX4 giả (UDP loopback), **chưa từng bay thật** **Kết luận ngắn:** kiến trúc và triết lý an toàn là đúng hướng, nhưng có **7 lỗi mức P0** khiến hệ thống hiện tại **chưa được phép bật `--enable-control` với cánh quạt lắp trên máy**. Ba trong số đó là lỗi có thể gây mất kiểm soát thật sự, không phải lỗi hình thức.

---

## 0. Bảng tổng hợp vấn đề

| ID | Vấn đề | Mức | Hậu quả xấu nhất |
| --- | --- | --- | --- |
| A1 | Luồng TX không có watchdog riêng | **P0** | Main loop treo → drone xoay mãi ở tốc độ cuối cùng |
| A2 | Ngắt điều khiển **không** thoát Offboard | **P0** | Buông dead-man nhưng drone vẫn ở Offboard, treo vô thời hạn |
| A3 | Không xác minh lệnh vào Offboard được chấp nhận | **P0** | Web báo "ĐANG ĐIỀU KHIỂN" trong khi PX4 ở mode khác |
| A4 | Giá trị RC/pin không kiểm tra độ cũ | **P0** | Mất stream RC → gate đọc giá trị cũ → dead-man coi như đang giữ |
| A5 | Web không xác thực, bind `0.0.0.0` | **P0** | Bất kỳ ai trong mạng đều ENGAGE được drone |
| A6 | `source_system=1` trùng ID với autopilot | **P0** | Xung đột định tuyến MAVLink, hành vi không xác định |
| A7 | Setpoint vận tốc 0 **không** giữ vị trí | **P0** | Drone trôi theo gió; phi công không giành lại được cần |
| B1 | Quy đổi pixel → góc dùng công thức tuyến tính | P1 | Sai tới ~4.5° ở HFOV 90° |
| B2 | Bbox bị cắt ở biên khung → tâm lệch vào trong | P1 | Người ở rìa khung không bao giờ được kéo về giữa |
| B3 | Không đo/không bù độ trễ vòng kín | P1 | Vọt lố, dao động khi tăng gain |
| B4 | Chỉ có khâu P → sai số bám thường trực | P1 | Người đi ngang 10°/s → lệch thường trực ~17° |
| B5 | `time_boot_ms` bị reset mỗi giây (dùng chung biến `t0`) | P1 | Timestamp bản tin sai, khó đối chiếu ulog |
| B6 | `time.sleep(dt)` cố định → trôi tần số | P1 | 19.7 Hz thay vì 20 Hz; xấu đi khi tải cao |
| B7 | Deadzone cứng gây bước nhảy 2.4°/s | P1 | Giật nhẹ khi vào/ra vùng chết |
| B8 | Lệnh web dùng ô nhớ đơn, có thể mất lệnh | P1 | DISENGAGE bị ENGAGE ghi đè trong cùng vòng lặp |
| C1 | INT8 hiệu chỉnh bằng 4 ảnh `coco8` | P1 | False positive → drone bám vật không phải người |
| C2 | Tracker không có đặc trưng ngoại hình | P1 | Tráo nhãn khi hai người cắt qua nhau |
| C3 | Reacquire chọn "gần nhất" không kiểm tra nhập nhằng | P1 | Âm thầm chuyển sang bám nhầm người |
| C4 | `imgsz=416` giới hạn tầm phát hiện ~20–25 m | P2 | Mất mục tiêu ở khoảng cách vận hành |
| C5 | Đọc trùng khung hình khi loop nhanh hơn camera | P2 | Nhiễu vận tốc track, GMC sai |
| C6 | Không có ngưỡng tin cậy trễ (hysteresis) cho mục tiêu đã khoá | P2 | Nhấp nháy mất/nhận mục tiêu |
| D1 | Signal handler in ra màn hình & lấy lock | P2 | Nguy cơ deadlock khi Ctrl+C |
| D2 | CSV không flush | P2 | Mất đúng những dòng log cần nhất sau sự cố |
| D3 | I/O (CSV, JPEG, print) nằm trên vòng điều khiển | P2 | FPS tụt, dt giật |
| D4 | Không có test tự động trong repo bàn giao | P2 | Không hồi quy được sau mỗi lần sửa |
| D5 | README mô tả 6 hành vi mà code không thực hiện | P2 | Người tiếp nhận tin nhầm vào cơ chế không tồn tại |

---

## 1. Những thứ đang làm đúng — giữ nguyên

Ghi rõ để người tiếp nhận không "cải tiến" mất:
1. **Thu hẹp phạm vi về một trục yaw.** Quyết định đúng. Mọi đề xuất mở rộng sang trục tiến/lùi phải được coi là một dự án mới với mô hình an toàn mới.
2. **`SafetyGate` tập trung, trả về lý do bằng chữ.** Kiểu thiết kế này hiếm và rất đáng giá khi debug sau sự cố. Cần mở rộng chứ không thay thế.
3. **Mặc định chạy khô.** `--enable-control` phải bật tường minh — đúng.
4. **Mất mục tiêu thì yaw về 0 ngay, bỏ qua slew.** Lập luận trong docstring chính xác.
5. **Cột `block` và `sp_hz` trong CSV.** Đây là hai cột phân tích sự cố quan trọng nhất.
6. **Không tự arm, không tự cất cánh.** Giữ nguyên vĩnh viễn.

---

## 2. Nhóm P0 — phải sửa trước khi lắp cánh quạt

### A1. Luồng TX không có watchdog riêng — nghiêm trọng nhất

**Hiện trạng.** `PX4Link._tx()` đọc `self._want_yaw` và gửi đi 20 Hz. Giá trị này chỉ được cập nhật bởi main loop, ở tốc độ ~15–25 Hz.

```python
def _tx(self):
    while ...:
        with self._lk:
            yaw = self._want_yaw      # <-- khong bao gio het han
        self.m.mav.set_position_target_local_ned_send(..., math.radians(yaw))
```

Nếu main loop treo — OpenVINO block, `cam.read()` kẹt, ổ đĩa đầy khi ghi CSV, GIL bị luồng web giữ — thì `_want_yaw` **đóng băng ở giá trị cuối cùng** và luồng TX tiếp tục gửi nó 20 Hz mãi mãi. PX4 thấy setpoint hợp lệ đều đặn nên **không** kích hoạt failsafe. Drone xoay liên tục ở 40°/s cho tới khi hết pin hoặc phi công can thiệp.

README mục 2 liệt kê "vòng lặp treo" là điều kiện watchdog. **Không có dòng code nào thực hiện việc này.**

**Hướng giải quyết.** Setpoint phải là giá trị *có hạn sử dụng*. Luồng TX tự quyết định về 0 nếu không được nuôi.

```python
WATCHDOG_S = 0.25   # ~5 chu ky setpoint

def command(self, yaw_deg_s):
    with self._lk:
        self._cmd_yaw = float(yaw_deg_s)
        self._t_cmd   = time.monotonic()

# trong vong lap TX:
with self._lk:
    yaw   = self._cmd_yaw
    stale = (time.monotonic() - self._t_cmd) > WATCHDOG_S
if stale:
    yaw = 0.0
    self.watchdog_trips += 1
```

Đồng thời đưa `watchdog_trips` lên web và vào CSV. Một lần trip trong chuyến bay là tín hiệu phải điều tra.

**Bổ sung lớp hai:** một luồng giám sát độc lập theo dõi `t_loop` của main loop; quá 1.0 s không thấy nhịp thì gọi thẳng `disengage(reason="main loop treo")`.

---

### A2. Ngắt điều khiển không thoát Offboard

**Hiện trạng.** `dung_khan()` chỉ làm ba việc:

```python
engaged = False
ctl.yaw = 0.0
link.set_yaw_rate(0.0)
```

Luồng TX **vẫn chạy**, vẫn gửi setpoint vận tốc 0 ở 20 Hz. PX4 thấy dòng setpoint liên tục và hợp lệ → **ở nguyên trong Offboard**.

README mục 2 viết: *"Buông tay là chặn ngay, drone thoát offboard và PX4 chuyển sang failsafe theo cấu hình của nó."* Câu này **sai**. Buông dead-man chỉ làm drone ngừng xoay; nó vẫn ở Offboard, vẫn do companion computer điều khiển, và phi công vẫn **không** có quyền cần (xem A7).

Đây là loại nhầm lẫn nguy hiểm nhất: người vận hành tin rằng mình có một nút thoát mà nút đó không tồn tại.

**Hướng giải quyết.** Định nghĩa rõ ba trạng thái và ba loại thoát:

| Trạng thái | Setpoint gửi đi | Mode PX4 |
| --- | --- | --- |
| `IDLE` | vị trí giữ + yaw_rate 0 | bất kỳ (chưa vào Offboard) |
| `ACTIVE` | vị trí giữ + yaw_rate tính toán | OFFBOARD |
| `HANDOFF` | **ngừng gửi** hoặc lệnh AUTO.LOITER | thoát Offboard |

Và ba loại thoát, chọn bằng cờ dòng lệnh:

```
--on-abort loiter   # chu dong gui AUTO.LOITER, xac minh bang heartbeat (MAC DINH)
--on-abort stream-off  # ngung setpoint, de PX4 tu failsafe theo COM_OBL_ACT
--on-abort hold-only   # yaw 0 nhung o lai Offboard (chi dung khi test tren ban)
```

`loiter` nên là mặc định vì nó **xác định được**: gửi lệnh, chờ `COMMAND_ACK`, chờ heartbeat xác nhận mode đã đổi, retry 3 lần, nếu thất bại thì rơi xuống `stream-off`.

**Lưu ý ngược lại, quan trọng:** `request_hold()` hiện tại gửi AUTO.LOITER **bất kể mode hiện tại là gì**. Nếu phi công đã tự giành lại quyền và đang bay Position mode thì lệnh này giật máy bay khỏi tay họ. Phải kiểm tra: chỉ gửi lệnh đổi mode khi `link.mode == "OFFBOARD"`.

---

### A3. Không xác minh Offboard được chấp nhận

**Hiện trạng.**

```python
def request_offboard(self):
    self.m.mav.command_long_send(..., MAV_CMD_DO_SET_MODE, 0, 1, 6, 0,0,0,0,0)
    self._want_offboard = True     # <-- "mong muon", khong phai "thuc te"
```

Không đọc `COMMAND_ACK`, không đối chiếu `custom_mode` trong heartbeat kế tiếp, `_want_offboard` không được ai dùng để kiểm tra lại. `SafetyGate` **không hề kiểm tra `link.mode`**.

Hệ quả cụ thể:
- PX4 từ chối vào Offboard (thiếu ước lượng vị trí, chưa arm, EKF chưa hội tụ) → web hiển thị nền đỏ **"DANG DIEU KHIEN DRONE — yaw 18.3 do/s"** trong khi máy bay đứng yên ở Position mode. Người vận hành mất niềm tin vào toàn bộ giao diện.
- Phi công gạt công tắc mode để giành quyền → script **không biết**, `engaged` vẫn `True`, vẫn tính toán và gửi yaw_rate. Nếu vì lý do nào đó máy bay quay lại Offboard, nó lập tức xoay theo lệnh cũ mà không ai bấm ENGAGE lần nữa.

**Hướng giải quyết.**
1. RX thread bắt `COMMAND_ACK` và đẩy vào hàng đợi.
2. `set_mode()` đồng bộ: gửi → chờ ACK (1.5 s) → chờ heartbeat xác nhận mode (2.0 s) → retry tối đa 3 lần → trả về `bool`.
3. **`SafetyGate` thêm điều kiện**: khi `engaged=True` thì bắt buộc `mode == "OFFBOARD"`, nếu không → block với lý do `"PX4 khong o OFFBOARD (dang: POSCTL)"` và tự động disengage.
4. Bắt `STATUSTEXT` và hiển thị lên web. PX4 nói rất rõ lý do từ chối (`"Offboard: no setpoint"`, `"Not arming: ..."`). Hiện script vứt bỏ toàn bộ thông tin này.

---

### A4. RC và pin không kiểm tra độ cũ

**Hiện trạng.**

```python
self.rc[i] = int(v)          # ghi de, khong luu thoi diem
...
v = link.rc.get(self.rc_chan, 0)
if v < self.rc_min: return False, "..."
```

`link.rc` là một dict giá trị thuần, **không có timestamp**. Nếu stream `RC_CHANNELS` ngừng (link telemetry nghẽn, PX4 giảm rate khi bận, radio nhiễu) nhưng HEARTBEAT vẫn về, thì gate tiếp tục đọc giá trị **1800 từ 8 giây trước** và kết luận phi công vẫn đang giữ dead-man.

Đây là lỗi phá vỡ chính xác cái lớp an toàn được README mô tả là quan trọng nhất.

Cùng vấn đề với `batt_v`.

**Hướng giải quyết.** Bọc mọi giá trị đọc từ MAVLink trong một kiểu có timestamp và **coi dữ liệu cũ là dữ liệu xấu**:

```python
@dataclass
class Stamped:
    value: object = None
    t: float = 0.0
    @property
    def age(self) -> float:
        return time.monotonic() - self.t if self.t else 1e9
```

Gate mới:

```python
if self.rc_chan:
    if link.rc_age > self.rc_max_age:          # mac dinh 0.5 s
        return False, f"du lieu RC cu {link.rc_age:.1f}s"
    if link.rc_value(self.rc_chan) < self.rc_min:
        return False, f"cong tac RC ch{self.rc_chan}=..."
```

**Đồng thời:** chủ động yêu cầu tần số bằng `MAV_CMD_SET_MESSAGE_INTERVAL` thay vì trông chờ stream mặc định:

| Bản tin | Tần số yêu cầu | Lý do |
| --- | --- | --- |
| `RC_CHANNELS` | 20 Hz | dead-man switch |
| `HEARTBEAT` | 2 Hz (mặc định) | mode + armed |
| `LOCAL_POSITION_NED` | 10 Hz | chốt điểm giữ (A7) |
| `ATTITUDE` | 20 Hz | yawspeed thực tế, bù nghiêng |
| `BATTERY_STATUS` | 1 Hz | pin |

**Cảnh báo về bản chất của dead-man qua MAVLink.** Đường đi là: công tắc → máy phát RC → máy thu → FMU → MAVLink → radio telemetry → companion → gate. Trễ tổng thường 150–400 ms và phụ thuộc vào một đường truyền không đảm bảo. Nó **không tương đương** với công tắc mode nối trực tiếp vào FC. Tài liệu vận hành phải ghi rõ: *công tắc mode trên tay điều khiển là lớp an toàn cuối cùng; dead-man qua MAVLink chỉ là lớp tiện lợi.*

---

### A5. Giao diện web không có xác thực

**Hiện trạng.** `ThreadingHTTPServer(("0.0.0.0", port), H)`, không token, không TLS, không CSRF. `GET /engage?on=1` là một request không tham số bí mật nào.

Bất cứ ai truy cập được cổng 8080 — cùng Wi-Fi bãi bay, cùng tailnet, hoặc một trang web bất kỳ mà người vận hành mở trong tab khác (`<img src="http://192.168.1.50:8080/engage?on=1">` là đủ, vì đây là GET) — đều bật được điều khiển drone.

**Hướng giải quyết.**
1. Sinh token ngẫu nhiên mỗi lần chạy, in ra console; mọi endpoint thay đổi trạng thái phải kèm token, so sánh bằng `hmac.compare_digest`.
2. Bind vào interface Tailscale cụ thể, không phải `0.0.0.0`. Cho phép ghi đè bằng `--bind`.
3. Đổi các endpoint điều khiển sang `POST`, chặn request có header `Origin`/`Referer` lạ.
4. **Thêm nhịp sống của người vận hành.** Hiện `engaged` là một chốt (latch) tồn tại mãi; nếu trình duyệt của người vận hành đơ hoặc mất Wi-Fi, drone **vẫn engaged**. Trang web nên gửi `POST /alive` mỗi 200 ms và gate chặn nếu `operator_age > 1.0 s`.
5. Thay `keydown → DISENGAGE` (không bắn trên mobile) bằng **nút giữ**: phải giữ ngón tay trên nút thì trang mới gửi `/alive`. Nhả tay = mất nhịp = disengage sau 1 s. Đây mới là dead-man đúng nghĩa ở phía web.

---

### A6. Trùng system ID với autopilot

```python
def __init__(self, url, baud=57600, rate_hz=20.0, source_system=1):
```

Companion đang phát bản tin dưới **sysid 1**, đúng bằng sysid của PX4. Trên một mạng MAVLink có thêm QGroundControl hoặc bộ định tuyến (mavlink-router, MAVSDK), việc trùng ID gây định tuyến sai, log lẫn lộn, và một số bộ lọc sẽ bỏ qua bản tin.

**Sửa:**

```python
mavutil.mavlink_connection(url, baud=baud,
                           source_system=191,      # companion rieng biet
                           source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)
```

Cùng lúc: xác nhận `target_system` lấy từ heartbeat của autopilot chứ không hardcode, và bỏ qua heartbeat từ các thành phần không phải autopilot (kiểm tra `msg.type != MAV_TYPE_GCS`).

---

### A7. Setpoint vận tốc 0 không giữ vị trí

**Hiện trạng.** `type_mask = 1479` → bỏ qua vị trí, dùng **vận tốc** `(0,0,0)` + yaw_rate.

Vận tốc 0 trong khung LOCAL_NED nghĩa là: *"bộ điều khiển hãy giữ vận tốc bằng 0"*. Đây là điều khiển **không có phản hồi vị trí**. Sai lệch ước lượng vận tốc, gió giật, hoặc bias EKF đều tích luỹ thành trôi vị trí mà không có gì kéo về. Trục Z cũng vậy — không có vòng giữ độ cao, chỉ có vòng giữ vận tốc đứng bằng 0, nên độ cao trôi chậm.

README mục 1 ghi "Giữ hoặc đổi độ cao — Phi công / chế độ bay của PX4". **Trong Offboard, phi công không giữ độ cao được.** Cần điều khiển đã do companion chiếm, trừ khi `COM_RC_OVERRIDE` bật đúng bit cho Offboard (theo tài liệu PX4 đây là bitmask, bit dành cho Auto và bit dành cho Offboard tách riêng — **phải kiểm tra lại trên đúng phiên bản firmware đang dùng**, mặc định thường chỉ bật cho Auto).

**Hướng giải quyết.** Chuyển sang **setpoint vị trí + yaw_rate**:

```python
# bo qua van toc (8|16|32) + gia toc (64|128|256) + yaw (1024)
TYPE_MASK_POS_YAWRATE = (8 | 16 | 32) | (64 | 128 | 256) | 1024   # = 1528
```

Tại thời điểm ENGAGE, chốt `(x, y, z)` hiện tại từ `LOCAL_POSITION_NED` và gửi đúng điểm đó suốt thời gian bám. Drone giữ vị trí có phản hồi, chỉ xoay quanh trục đứng — đúng như README mô tả.

> **Cần xác minh trong SITL trước:** tổ hợp position + yawspeed phải được firmware đang dùng chấp nhận. Nếu bị từ chối, phương án dự phòng là giữ nguyên setpoint vận tốc nhưng thêm một vòng P vị trí ở phía companion (chốt điểm, tính `v = k·(p_hold − p_hiện tại)`, giới hạn ±0.5 m/s).

Thêm hai điều kiện gate mới:
- **Ước lượng vị trí hợp lệ** trước khi cho ENGAGE (có `LOCAL_POSITION_NED` mới, EKF ổn định). Không có vị trí thì Offboard sẽ bị từ chối hoặc tệ hơn.
- **Giới hạn trôi**: nếu vị trí hiện tại lệch quá `--max-drift` (mặc định 3 m) so với điểm chốt → disengage. Đây là dấu hiệu bộ điều khiển vị trí không theo kịp.

---

## 3. Nhóm P1 — độ đúng và chất lượng điều khiển

### B1. Công thức pixel → góc sai

```python
self.ang_x = ((tx - w / 2.0) / (w / 2.0)) * (self.hfov / 2.0)
```

Đây là ánh xạ tuyến tính. Mô hình pinhole đúng là:

```
f      = (w/2) / tan(HFOV/2)
góc    = atan( (tx − w/2) / f )
```

Sai số ở HFOV 90°, khung 1280 px:

| Vị trí mục tiêu (px từ tâm) | Công thức tuyến tính | Công thức đúng | Sai lệch |
| --- | --- | --- | --- |
| 160 | 11.25° | 9.09° | 2.16° |
| 320 | 22.50° | 17.55° | 4.95° |
| 480 | 33.75° | 24.78° | 8.97° |
| 640 (biên) | 45.00° | 45.00° | 0° |

Sai lệch 5–9° trên một hệ có deadzone 4° là sai lệch có ý nghĩa: gain hiệu dụng phụ thuộc vị trí mục tiêu trong khung, cao hơn ở rìa. Không gây mất ổn định (hệ đang quá thận trọng) nhưng làm việc tinh chỉnh gain trở nên vô nghĩa.

**Sửa:** dùng `atan2`. Chi phí bằng không.

**Nâng cao:** ống kính góc rộng có méo thùng đáng kể. Nên hiệu chuẩn camera một lần (`cv2.calibrateCamera`, bàn cờ), lưu `fx, cx, dist` ra JSON, và undistort riêng toạ độ tâm mục tiêu (`cv2.undistortPoints` — rẻ hơn undistort cả ảnh nhiều lần). Khi đó `--hfov` trở thành tham số dự phòng chứ không phải nguồn chân lý.

### B2. Bbox bị cắt ở biên khung

```python
np.clip(b[:, [0, 2]], 0, frame.shape[1], out=b[:, [0, 2]])
```

Người đứng nửa trong nửa ngoài khung: box bị cắt, tâm box dịch **vào trong** so với tâm người thật. Sai số góc bị báo nhỏ hơn thực tế → drone quay chậm hơn cần thiết → người trôi ra khỏi khung hoàn toàn.

**Sửa.** Phát hiện box chạm biên và xử lý riêng:

```python
touch_left  = box[0] <= EDGE_PX
touch_right = box[2] >= w - EDGE_PX
if touch_left ^ touch_right:
    # dung canh NHIN THAY lam moc, khong dung tam
    ref_x = box[2] if touch_left else box[0]
    # va dat san toc do toi thieu de keo ve
    min_rate = MIN_EDGE_RATE      # vd 8 do/s
```

Ghi cờ `at_edge` vào CSV — đây là tình huống rủi ro cao cần soi lại sau bay.

### B3. Không đo độ trễ vòng kín

Ước lượng ngân sách trễ hiện tại:

| Khâu | Trễ điển hình |
| --- | --- |
| Phơi sáng + truyền USB MJPEG + giải mã | 20–40 ms |
| Chờ trong luồng camera (tối đa 1 khung) | 0–33 ms |
| Letterbox + suy luận OpenVINO @416 | 25–60 ms |
| Phần còn lại của vòng lặp (CSV, JPEG, print) | 10–25 ms |
| Lượng tử hoá TX 20 Hz | 0–50 ms |
| Serial + xử lý PX4 | 10–25 ms |
| **Tổng** | **~65–230 ms** |

Ở 40°/s, 200 ms trễ tương đương 8° vọt lố — bằng đúng hai lần deadzone. Đây là lý do hệ phải để gain rất thấp mới không dao động.

**Hướng giải quyết.**
1. **Đo, đừng đoán.** Gắn timestamp vào từng khung ngay khi `cap.read()` trả về, mang timestamp đó đi suốt pipeline, và log `t_now − t_capture` vào CSV thành cột `latency_ms`.
2. **Bù trễ bằng dự đoán.** Dùng vận tốc góc tương đối ước lượng được để ngoại suy: `error_dùng = error_đo + ω_tương_đối × latency`
3. **Cắt bỏ trễ thừa:** tách JPEG encode và ghi CSV sang luồng riêng (xem D3); giảm `--sp-rate` không giúp gì, nhưng dùng `req.start_async` **thật sự bất đồng bộ** (xử lý khung N trong khi lấy khung N+1) cắt được ~30 ms.

### B4. Chỉ có khâu P — sai số bám thường trực

Vòng kín hiện tại là bậc nhất: `dθ/dt = −gain·θ`. Với mục tiêu **đứng yên**, hằng số thời gian là `1/gain = 1.67 s`.

Với mục tiêu **chuyển động góc** ω, sai số xác lập là `θ_ss = ω / gain`:

| Người di chuyển | ω tại 15 m | Sai số xác lập (gain 0.6) |
| --- | --- | --- |
| Đi bộ 1.4 m/s ngang | 5.3°/s | 8.9° |
| Chạy nhẹ 3 m/s ngang | 11.5°/s | 19.1° |
| Chạy 5 m/s ngang | 19.1°/s | 31.8° |

Ở 32° lệch với HFOV 90°, người đã gần rìa khung. Đây là lý do thật sự khiến hệ mất mục tiêu, không phải do model.

**Ghi chú sửa tài liệu:** README mục 7 viết *"gain = 0.6 nghĩa là mỗi giây sửa được 60% sai số góc còn lại"*. Đúng phải là `1 − e^(−0.6) = 45%` sau một giây. Con số 60% chỉ đúng theo nghĩa "tốc độ sửa tức thời".

**Hướng giải quyết.** Thêm khâu **feed-forward** thay vì khâu tích phân (I dễ gây wind-up khi mục tiêu bị che):

```
ω_mục_tiêu ≈ d(bearing)/dt + yawspeed_thực_tế_từ_ATTITUDE
lệnh       = gain · error + k_ff · ω_mục_tiêu
```

`k_ff = 1.0` về lý thuyết triệt tiêu hoàn toàn sai số bám. Bắt đầu từ `k_ff = 0.6`, lọc `ω` bằng bộ lọc bậc nhất τ≈0.3 s để không khuếch đại nhiễu box.

Sau khi có bù trễ (B3) và công thức góc đúng (B1), gain hoàn toàn có thể nâng lên 1.2–1.8 mà vẫn dư biên độ ổn định (biên pha ở gain 1.5, trễ 0.2 s vẫn trên 70°). **Nhưng chỉ nâng gain sau khi đã đo trễ thật, trên máy bay có dây buộc.**

### B5. `time_boot_ms` bị reset mỗi giây

```python
n0 = 0; t0 = time.monotonic()
while ...:
    self.m.mav.set_position_target_local_ned_send(
        int((time.monotonic() - t0) * 1000) & 0xFFFFFFFF, ...)   # (1)
    ...
    if now - t0 >= 1.0:
        self.sp_hz = (self.sp_sent - n0) / (now - t0)
        n0 = self.sp_sent; t0 = now                              # (2)
```

`t0` vừa là gốc thời gian của trường `time_boot_ms`, vừa là mốc cửa sổ đo tần số — và dòng (2) reset nó mỗi giây. Trường timestamp gửi đi chạy 0→1000 rồi về 0, lặp vô hạn.

PX4 phần lớn bỏ qua trường này với offboard setpoint, nên chưa gây sự cố, nhưng nó phá hoại khả năng đối chiếu bản tin với ulog sau bay.

**Sửa:** tách hai biến — `self._t_boot` cố định tại lúc khởi tạo, `self._t_win` cho cửa sổ đo tần số.

### B6. `time.sleep(dt)` cố định

`time.sleep(0.05)` cộng với thời gian gửi bản tin → chu kỳ thực > 50 ms → 19.7 Hz như đã đo. Khi CPU tải cao, con số này tụt thêm.

**Sửa:** lập lịch theo mốc tuyệt đối:

```python
next_t = time.monotonic()
while ...:
    next_t += self.dt
    ...send...
    sleep = next_t - time.monotonic()
    if sleep > 0:
        time.sleep(sleep)
    else:
        next_t = time.monotonic()   # bo chu ky da tre, khong don no
```

### B7. Deadzone cứng

Khi `|error|` vượt 4°, lệnh nhảy từ 0 lên `0.6 × 4 = 2.4°/s`. Slew làm mượt phần nào, nhưng vẫn tạo chu kỳ giới hạn nhỏ quanh mép vùng chết.

**Sửa:** deadzone mềm — trừ đi thay vì cắt bỏ:

```python
e = self.bearing_deg(...)
if abs(e) <= self.deadzone:
    e_eff = 0.0
else:
    e_eff = math.copysign(abs(e) - self.deadzone, e)   # lien tuc tai bien
```

### B8. Lệnh web dùng ô nhớ đơn

```python
_cmd = {"engage": None, ...}
```

Hai lệnh đến trong cùng một chu kỳ vòng lặp (~50 ms — hoàn toàn có thể khi người dùng bấm nhầm rồi sửa) thì lệnh đầu bị mất. Trường hợp nguy hiểm: DISENGAGE rồi ENGAGE → chỉ ENGAGE được xử lý.

**Sửa:** dùng `queue.Queue()` cho lệnh thường, và **một chốt dừng riêng, có ưu tiên tuyệt đối**:

```python
_stop_latch = threading.Event()     # bat boi DISENGAGE, /alive het han, phim bat ky

# dau moi vong lap:
if _stop_latch.is_set():
    disengage("chot dung"); _stop_latch.clear()
    drain(_cmd_queue)               # bo moi lenh dang cho, ke ca ENGAGE
```

---

## 4. Nhóm vision

### C1. Hiệu chỉnh INT8 bằng 4 ảnh `coco8`

Bộ hiệu chỉnh 4 ảnh chung chung không đại diện cho phân bố kích hoạt của camera thật ở góc nhìn từ trên cao, ngược sáng, nền cỏ/bê tông. Kết quả điển hình: mAP tụt và **false positive tăng ở những vùng ảnh có texture lặp** — bụi cây, cột, vạch kẻ, bóng người.

Một false positive được tracker duy trì 3 khung (`min_hits=3`) là đủ để người vận hành khoá nhầm, hoặc để reacquire nhảy sang nó.

**Kế hoạch cụ thể:**
1. Thu 300–500 khung từ **đúng camera, đúng ống kính, đúng độ phân giải** ở 3 điều kiện sáng (sáng gắt, râm, chiều muộn) và 2 độ cao bay dự kiến. Bao gồm cả khung **không có người** — rất quan trọng cho hiệu chỉnh.
2. Hiệu chỉnh lại INT8 với tập này.
3. Đo trên tập kiểm tra tách riêng (~200 khung có nhãn tay): so sánh Recall/Precision của INT8 mới với FP32 gốc. Tiêu chí chấp nhận: recall giảm không quá 3 điểm, precision **không giảm**.
4. Chạy **bài soát false positive**: 20 phút video hoàn toàn không có người, đếm số detection có `conf ≥ 0.4`. Tiêu chí: **< 1 FP kéo dài ≥ 3 khung trên toàn bài**.

Bổ sung lọc rẻ tiền, làm ngay được:

```python
# ti le khung nguoi hop ly
ok_ratio = 1.2 <= (y2 - y1) / max(x2 - x1, 1) <= 5.0
# kich thuoc toi thieu (nguoi < 24 px cao thi khong tin)
ok_size  = (y2 - y1) >= MIN_BOX_H
```

### C2 & C3. Tracker tráo nhãn và reacquire mù

Hiện tại tracker chỉ dùng IoU; `TargetManager.update()` khi mất dấu thì chọn **track gần nhất trong bán kính `0.25 × w` = 320 px** — một vùng rất rộng — và âm thầm gán lại nhãn.

Hai kịch bản hỏng:
- Hai người mặc áo giống nhau đi cắt qua nhau → IoU ghép chéo → `M1` chuyển sang người kia. Drone bám nhầm, không có cảnh báo nào.
- Mục tiêu bị che sau cột 2 giây; một người khác đi ngang chỗ đó → reacquire gán nhầm.

**Hướng giải quyết — hai phần.**

**Phần 1: thêm đặc trưng ngoại hình rẻ.** Không cần mạng re-ID. Histogram HSV của vùng thân trên (30–60% chiều cao box) đã đủ phân biệt phần lớn trường hợp thực tế:

```python
def embed(frame, box):
    x1, y1, x2, y2 = box.astype(int)
    h = y2 - y1
    roi = frame[y1 + int(0.30*h) : y1 + int(0.60*h), x1:x2]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0, 1], None, [16, 16], [0, 180, 0, 256])
    return cv2.normalize(hist, hist).flatten()

# khoang cach: cv2.compareHist(a, b, cv2.HISTCMP_BHATTACHARYYA)
```

Giữ EMA histogram cho mỗi mục tiêu đã khoá (`α = 0.9`), chỉ cập nhật khi track đang `seen` và conf cao.

**Phần 2 — quan trọng hơn: khi nhập nhằng thì từ chối, không đoán.**

```python
cands = sorted(candidates, key=score)      # score thap = tot
if not cands or cands[0].score > ACCEPT_THR:
    return None                            # khong nhan lai
if len(cands) > 1 and (cands[1].score - cands[0].score) < MARGIN:
    self.msg = "reacquire nhap nhang -> tu choi, can xac nhan lai"
    self.needs_confirm = True              # gate se CHAN
    return None
```

Và thêm điều kiện gate: `needs_confirm == True` → block với lý do `"can xac nhan lai muc tieu"`. Người vận hành phải bấm lại vào đúng người. Nguyên tắc: **dừng luôn tốt hơn đoán sai** — cùng nguyên tắc mà `YawController` đã áp dụng đúng khi mất mục tiêu.

Bổ sung: sau mỗi lần reacquire thành công, ép yaw = 0 trong 0.5 s và nháy cảnh báo trên web, để người vận hành kịp nhìn và huỷ nếu sai.

### C4. Ngân sách tầm phát hiện

Với HFOV 90°, khung 1280×720 (VFOV ≈ 58.7°), `imgsz=416` (hệ số letterbox `r = 0.325`):

| Khoảng cách | Chiều cao người trong khung gốc | Trong ảnh vào model |
| --- | --- | --- |
| 10 m | ~120 px | ~39 px |
| 15 m | ~80 px | ~26 px |
| 20 m | ~60 px | ~19 px |
| 30 m | ~40 px | ~13 px |

YOLO nano ở 13 px chiều cao là không đáng tin. **Tầm vận hành thực tế hiện nay khoảng 15–20 m**, cần ghi vào tài liệu vận hành.

**Cách mở rộng tầm mà không giết FPS — suy luận hai mức:**
- Mỗi 5 khung: chạy toàn khung ở 416 để phát hiện người mới.
- Các khung còn lại: crop vùng 2× quanh box mục tiêu đã khoá, resize về 256, suy luận. Độ phân giải hiệu dụng trên mục tiêu tăng 3–4×, thời gian suy luận **giảm**.

Đây là cải tiến giải quyết đồng thời C4 và B3.

### C5. Đọc trùng khung hình

`Camera.read()` trả về khung mới nhất; nếu vòng lặp nhanh hơn camera thì cùng một khung được xử lý hai lần. GMC trả về shift 0, tracker cập nhật `vel = 0.5·vel + 0` → hãm vận tốc dự đoán một cách giả tạo.

**Sửa:** thêm bộ đếm khung, bỏ qua nếu chưa có khung mới.

```python
def read(self):
    with self._lk:
        return self._f, self._seq, self._t_cap
```

---

## 5. Kiến trúc và khả năng kiểm thử

### D3/D4. Tách vòng điều khiển ra khỏi I/O

Vấn đề cấu trúc lớn nhất: `main()` dài ~200 dòng, trộn lẫn suy luận, điều khiển, an toàn, ghi log, mã hoá JPEG và in ra terminal. **Không có phần nào kiểm thử tự động được** ngoài việc chạy cả chương trình.

Đề xuất tách file:

```
follow/
  __init__.py
  camera.py        Camera (them seq + timestamp)
  detect.py        load_model, letterbox, hau xu ly, loc kich thuoc/ti le
  track.py         GMC, ByteTrack, _Track
  target.py        LockedTarget, TargetManager, Appearance
  control.py       YawController (thuan tuy, khong I/O)
  safety.py        SafetyGate, GateInputs, GateResult  (thuan tuy)
  px4.py           PX4Link, SetpointStreamer
  webui.py         HTTP server, token, nhip song nguoi van hanh
  logging_sink.py  luong ghi CSV + video
  app.py           lap rap, vong lap chinh mong
tests/
  test_safety.py
  test_control.py
  test_mavlink.py
  test_target.py
```

Nguyên tắc: `control.py` và `safety.py` **không import cv2, không import pymavlink, không đọc thời gian hệ thống** (nhận `now` làm tham số). Khi đó chúng kiểm thử được bằng bảng dữ liệu thuần.

---

## 6. Sườn code

Chú thích trong code giữ nguyên phong cách không dấu như codebase hiện tại.

### 6.1 `safety.py` — gate mở rộng

```python
"""Gate an toan - THUAN TUY: khong I/O, khong doc dong ho he thong.
Moi dau vao truyen qua GateInputs de kiem thu duoc bang bang du lieu."""

from dataclasses import dataclass, field
from typing import List, Optional


@dataclass(frozen=True)
class GateInputs:
    # y dinh cua nguoi van hanh
    engaged: bool
    operator_age: float          # giay ke tu nhip /alive cuoi cung

    # lien ket & trang thai PX4
    link_connected: bool
    hb_age: float
    armed: bool
    mode: str                    # vd "OFFBOARD", "POSCTL"
    pos_valid: bool              # co LOCAL_POSITION_NED moi
    drift_m: float               # lech so voi diem chot khi engage

    # RC / pin (KEM tuoi du lieu)
    rc_value: int
    rc_age: float
    batt_v: float
    batt_age: float

    # vision
    cam_age: float
    has_target: bool
    target_age: float
    target_needs_confirm: bool

    # suc khoe he thong
    loop_age: float              # giay ke tu nhip main loop cuoi
    tx_watchdog_trips: int


@dataclass(frozen=True)
class GateConfig:
    enable_control: bool
    rc_chan: int = 0
    rc_min: int = 1500
    rc_max_age: float = 0.5
    batt_min: float = 0.0
    batt_max_age: float = 5.0
    hb_timeout: float = 2.0
    frame_timeout: float = 1.0
    vision_timeout: float = 0.6
    operator_timeout: float = 1.0
    loop_timeout: float = 0.5
    max_drift_m: float = 3.0
    require_offboard: bool = True


@dataclass
class GateResult:
    allow: bool
    reason: str = ""             # ly do dau tien, hien tren web
    all_reasons: List[str] = field(default_factory=list)
    fatal: bool = False          # True -> disengage, khong chi zero yaw


def check(cfg: GateConfig, s: GateInputs) -> GateResult:
    """Tra ve GateResult. `fatal` phan biet hai loai chan:
   - khong fatal (vd mat muc tieu tam thoi) -> yaw ve 0, GIU engaged
   - fatal (vd mat heartbeat)               -> disengage + thoat offboard
    """
    reasons: List[str] = []
    fatal = False

    def add(msg: str, is_fatal: bool = False):
        nonlocal fatal
        reasons.append(msg)
        fatal = fatal or is_fatal

    # --- lop 1: co bat
    if not cfg.enable_control:
        add("chua bat --enable-control")

    # --- lop 2: lien ket
    if not s.link_connected:
        add("chua ket noi MAVLink", True)
    if s.hb_age > cfg.hb_timeout:
        add(f"mat heartbeat {s.hb_age:.1f}s", True)

    # --- lop 3: trang thai bay
    if not s.armed:
        add("drone chua arm", True)
    if not s.pos_valid:
        add("chua co uoc luong vi tri", True)
    if cfg.require_offboard and s.engaged and s.mode != "OFFBOARD":
        add(f"PX4 khong o OFFBOARD (dang: {s.mode})", True)
    if s.drift_m > cfg.max_drift_m:
        add(f"troi {s.drift_m:.1f}m khoi diem chot", True)

    # --- lop 4: dead-man RC (KIEM TRA TUOI TRUOC)
    if cfg.rc_chan:
        if s.rc_age > cfg.rc_max_age:
            add(f"du lieu RC cu {s.rc_age:.1f}s", True)
        elif s.rc_value < cfg.rc_min:
            add(f"cong tac RC ch{cfg.rc_chan}={s.rc_value} "
                f"(can >={cfg.rc_min})", True)

    # --- lop 5: pin
    if cfg.batt_min > 0:
        if s.batt_age > cfg.batt_max_age:
            add(f"du lieu pin cu {s.batt_age:.1f}s", True)
        elif 0 < s.batt_v < cfg.batt_min:
            add(f"pin thap {s.batt_v:.1f}V", True)

    # --- lop 6: suc khoe phan mem
    if s.cam_age > cfg.frame_timeout:
        add(f"mat camera {s.cam_age:.1f}s", True)
    if s.loop_age > cfg.loop_timeout:
        add(f"vong lap treo {s.loop_age:.1f}s", True)

    # --- lop 7: nguoi van hanh
    if not s.engaged:
        add("chua bam ENGAGE")
    if s.engaged and s.operator_age > cfg.operator_timeout:
        add(f"mat nhip nguoi van hanh {s.operator_age:.1f}s", True)

    # --- lop 8: muc tieu (KHONG fatal - chi zero yaw)
    if not s.has_target:
        add("chua khoa muc tieu")
    elif s.target_needs_confirm:
        add("can xac nhan lai muc tieu")
    elif s.target_age > cfg.vision_timeout:
        add(f"muc tieu cu {s.target_age:.1f}s")

    return GateResult(allow=not reasons,
                      reason=reasons[0] if reasons else "",
                      all_reasons=reasons,
                      fatal=fatal)
```

### 6.2 `px4.py` — link có ACK, giá trị có tuổi, TX có watchdog

```python
import math, threading, time, queue
from dataclasses import dataclass

TYPE_MASK_POS_YAWRATE = (8 | 16 | 32) | (64 | 128 | 256) | 1024   # 1528
TYPE_MASK_VEL_YAWRATE = (1 | 2 | 4) | (64 | 128 | 256) | 1024     # 1479

PX4_MAIN_AUTO, PX4_MAIN_OFFBOARD = 4, 6
PX4_SUB_LOITER = 3


@dataclass
class Stamped:
    value: object = None
    t: float = 0.0

    @property
    def age(self) -> float:
        return time.monotonic() - self.t if self.t else 1e9

    def set(self, v):
        self.value, self.t = v, time.monotonic()


class PX4Link:
    """RX: heartbeat, mode, armed, RC, pin, vi tri, thai do, ACK, STATUSTEXT."""

    def __init__(self, url, baud=57600, sysid=191):
        from pymavlink import mavutil
        self.mavutil, self.mv = mavutil, mavutil.mavlink
        self.m = mavutil.mavlink_connection(
            url, baud=baud, source_system=sysid,
            source_component=self.mv.MAV_COMP_ID_ONBOARD_COMPUTER)

        hb = self.m.wait_heartbeat(timeout=10)
        if hb is None:
            raise SystemExit(f"[!] Khong nhan duoc heartbeat tu {url}")

        self.connected = True
        self.hb    = Stamped(); self.hb.set(True)
        self.armed = False
        self.mode  = "?"
        self.rc    = Stamped({})          # dict {chan: value}
        self.batt  = Stamped(0.0)
        self.pos   = Stamped(None)        # (x, y, z) NED
        self.att   = Stamped(None)        # (roll, pitch, yaw, yawspeed)
        self.status_text = ""

        self._acks = queue.Queue(maxsize=32)
        self._done = threading.Event()
        threading.Thread(target=self._rx, daemon=True).start()

        self._request_streams()

    # ---------------------------------------------------------- streams
    def _request_streams(self):
        want = [("RC_CHANNELS",        self.mv.MAVLINK_MSG_ID_RC_CHANNELS,        20),
                ("LOCAL_POSITION_NED", self.mv.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10),
                ("ATTITUDE",           self.mv.MAVLINK_MSG_ID_ATTITUDE,           20),
                ("BATTERY_STATUS",     self.mv.MAVLINK_MSG_ID_BATTERY_STATUS,      1)]
        for name, mid, hz in want:
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component,
                self.mv.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                mid, int(1e6 / hz), 0, 0, 0, 0, 0)
            # TODO: doc ACK, canh bao neu PX4 tu choi

    # ---------------------------------------------------------- RX
    def _rx(self):
        while not self._done.is_set():
            try:
                msg = self.m.recv_match(blocking=True, timeout=1.0)
            except Exception:
                time.sleep(0.2); continue
            if msg is None:
                continue
            t = msg.get_type()

            if t == "HEARTBEAT":
                # TODO: bo qua heartbeat khong phai tu autopilot
                self.hb.set(True)
                self.armed = bool(msg.base_mode
                                  & self.mv.MAV_MODE_FLAG_SAFETY_ARMED)
                try:    self.mode = self.m.flightmode
                except Exception: self.mode = str(msg.custom_mode)

            elif t in ("RC_CHANNELS", "RC_CHANNELS_RAW"):
                d = {}
                for i in range(1, 19):
                    v = getattr(msg, f"chan{i}_raw", None)
                    if v is not None and v != 65535:
                        d[i] = int(v)
                if d:
                    self.rc.set(d)

            elif t == "LOCAL_POSITION_NED":
                self.pos.set((msg.x, msg.y, msg.z))

            elif t == "ATTITUDE":
                self.att.set((msg.roll, msg.pitch, msg.yaw, msg.yawspeed))

            elif t in ("SYS_STATUS", "BATTERY_STATUS"):
                mvv = getattr(msg, "voltage_battery", None)
                if mvv is None:
                    vs = getattr(msg, "voltages", None)
                    mvv = vs[0] if vs else None
                if mvv and 0 < mvv < 65535:
                    self.batt.set(mvv / 1000.0)

            elif t == "COMMAND_ACK":
                try:    self._acks.put_nowait(msg)
                except queue.Full: pass

            elif t == "STATUSTEXT":
                # RAT HUU ICH: PX4 noi ro ly do tu choi offboard
                self.status_text = msg.text
                print(f"  [PX4] {msg.text}")

    # ---------------------------------------------------------- doi mode
    def _wait_ack(self, cmd_id, timeout):
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            try:
                a = self._acks.get(timeout=max(0.05, t_end - time.monotonic()))
            except queue.Empty:
                return None
            if a.command == cmd_id:
                return a
        return None

    def set_mode(self, main, sub=0, expect=None, retries=3) -> bool:
        """Gui DO_SET_MODE, cho ACK, roi cho heartbeat XAC NHAN mode."""
        cmd = self.mv.MAV_CMD_DO_SET_MODE
        for _ in range(retries):
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component, cmd, 0,
                self.mv.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                main, sub, 0, 0, 0, 0)
            ack = self._wait_ack(cmd, 1.5)
            if ack is None or ack.result != self.mv.MAV_RESULT_ACCEPTED:
                print(f"  [MAV] DO_SET_MODE bi tu choi: "
                      f"{ack.result if ack else 'khong co ACK'} "
                      f"| {self.status_text}")
                continue
            t_end = time.monotonic() + 2.0
            while time.monotonic() < t_end:
                if expect is None or self.mode == expect:
                    return True
                time.sleep(0.05)
            print(f"  [MAV] ACK ok nhung mode van la {self.mode}")
        return False

    def enter_offboard(self) -> bool:
        return self.set_mode(PX4_MAIN_OFFBOARD, 0, expect="OFFBOARD")

    def enter_loiter(self) -> bool:
        # CHI goi khi dang o OFFBOARD - tranh giat quyen khoi phi cong
        if self.mode != "OFFBOARD":
            return True
        return self.set_mode(PX4_MAIN_AUTO, PX4_SUB_LOITER)

    def close(self):
        self._done.set()
        try: self.m.close()
        except Exception: pass
```

```python
class SetpointStreamer(threading.Thread):
    """Gui setpoint deu dan. Lenh yaw CO HAN SU DUNG.

    An toan cot loi: neu khong duoc `command()` nuoi trong WATCHDOG_S giay
    thi tu dong ve 0. Main loop treo => drone dung xoay, khong xoay mai.
    """
    WATCHDOG_S = 0.25

    def __init__(self, link, rate_hz=20.0, use_position_hold=True):
        super().__init__(daemon=True)
        self.link, self.dt = link, 1.0 / rate_hz
        self.use_position_hold = use_position_hold
        self._lk = threading.Lock()
        self._cmd_yaw, self._t_cmd = 0.0, 0.0
        self._hold = None                    # (x, y, z) chot luc engage
        self._stop = threading.Event()
        self._t_boot = time.monotonic()      # CO DINH - khong reset (loi B5)
        self.sp_sent = 0
        self.sp_hz = 0.0
        self.watchdog_trips = 0

    # ------------------------------------------------------------ API
    def command(self, yaw_deg_s):
        with self._lk:
            self._cmd_yaw = float(yaw_deg_s)
            self._t_cmd = time.monotonic()

    def arm_hold(self, xyz):
        """Chot diem giu tai thoi diem ENGAGE."""
        with self._lk:
            self._hold = tuple(xyz) if xyz else None

    def clear_hold(self):
        with self._lk:
            self._hold = None

    def drift_from_hold(self, cur_xyz):
        with self._lk: h = self._hold
        if h is None or cur_xyz is None:
            return 0.0
        return math.dist(h[:2], cur_xyz[:2])

    def stop(self):
        self._stop.set()

    # ------------------------------------------------------------ vong TX
    def run(self):
        next_t = time.monotonic()
        t_win, n_win = next_t, 0
        while not self._stop.is_set():
            next_t += self.dt
            with self._lk:
                yaw, t_cmd, hold = self._cmd_yaw, self._t_cmd, self._hold
            if (time.monotonic() - t_cmd) > self.WATCHDOG_S:
                if yaw != 0.0:
                    self.watchdog_trips += 1
                    print(f"\n  !! TX WATCHDOG: lenh yaw qua han, ep ve 0")
                yaw = 0.0
            self._send(hold, yaw)

            now = time.monotonic()
            if now - t_win >= 1.0:
                self.sp_hz = (self.sp_sent - n_win) / (now - t_win)
                t_win, n_win = now, self.sp_sent

            sleep = next_t - now
            if sleep > 0: time.sleep(sleep)
            else:         next_t = time.monotonic()   # bo chu ky da tre

    def _send(self, hold, yaw_deg_s):
        mv, m = self.link.mv, self.link.m
        boot_ms = int((time.monotonic() - self._t_boot) * 1000) & 0xFFFFFFFF
        if hold is not None and self.use_position_hold:
            mask = TYPE_MASK_POS_YAWRATE
            x, y, z = hold
            vx = vy = vz = 0.0
        else:
            mask = TYPE_MASK_VEL_YAWRATE
            x = y = z = 0.0
            vx = vy = vz = 0.0
        try:
            m.mav.set_position_target_local_ned_send(
                boot_ms, m.target_system, m.target_component,
                mv.MAV_FRAME_LOCAL_NED, mask,
                x, y, z, vx, vy, vz, 0, 0, 0,
                0.0, math.radians(yaw_deg_s))
            self.sp_sent += 1
        except Exception as e:
            print(f"\n  !! loi gui setpoint: {e}")
```

### 6.3 `control.py` — bộ điều khiển yaw viết lại

```python
import math
from dataclasses import dataclass


@dataclass
class YawConfig:
    hfov_deg: float = 90.0
    gain: float = 0.6           # 1/s
    k_ff: float = 0.0           # he so feed-forward (0 = tat, bat dau 0.6)
    deadzone_deg: float = 4.0
    max_rate: float = 40.0      # do/s
    slew: float = 80.0          # do/s^2
    lead_s: float = 0.0         # bu tre, dat = do tre do duoc
    min_edge_rate: float = 8.0  # toc do toi thieu khi muc tieu cham bien
    invert: bool = False
    fx_px: float = 0.0          # neu >0 thi dung thay cho hfov (da hieu chuan)


class YawController:
    """Thuan tuy: nhan so, tra so. Khong I/O, khong doc dong ho."""

    EDGE_PX = 4

    def __init__(self, cfg: YawConfig, width_px: int):
        self.cfg = cfg
        self.w = width_px
        self.fx = cfg.fx_px or (width_px / 2.0) /
                   math.tan(math.radians(cfg.hfov_deg) / 2.0)
        self.yaw = 0.0
        self.bearing = 0.0
        self._prev_bearing = None
        self._omega = 0.0           # toc do goc tuong doi da loc
        self.at_edge = False
        self.note = ""

    # -------------------------------------------------- pixel -> goc (loi B1)
    def bearing_deg(self, x_px: float) -> float:
        return math.degrees(math.atan2(x_px - self.w / 2.0, self.fx))

    def _measure(self, box):
        """Tra (bearing_deg, at_edge). Xu ly box bi cat o bien (loi B2)."""
        x1, x2 = float(box[0]), float(box[2])
        left  = x1 <= self.EDGE_PX
        right = x2 >= self.w - self.EDGE_PX
        if left ^ right:
            ref = x2 if left else x1     # dung canh NHIN THAY
            return self.bearing_deg(ref), True
        return self.bearing_deg((x1 + x2) / 2.0), False

    # -------------------------------------------------- vong dieu khien
    def update(self, box, dt, allow, yawspeed_actual_deg_s=0.0):
        """box: ndarray[4] hoac None. allow: ket qua tu SafetyGate."""
        c = self.cfg

        if box is None or not allow:
            # AN TOAN: ve 0 NGAY, bo qua slew (giu nguyen thiet ke goc)
            self.yaw = 0.0
            self.bearing = 0.0
            self._prev_bearing = None
            self._omega = 0.0
            self.at_edge = False
            self.note = "khong co muc tieu" if box is None else "bi chan"
            return 0.0

        b, self.at_edge = self._measure(box)
        if c.invert:
            b = -b
        self.bearing = b

        # --- uoc luong toc do goc cua MUC TIEU trong khung quan tinh
        if self._prev_bearing is not None and dt > 1e-3:
            raw_omega = (b - self._prev_bearing) / dt + yawspeed_actual_deg_s
            alpha = min(1.0, dt / 0.3)            # loc bac nhat tau = 0.3 s
            self._omega += alpha * (raw_omega - self._omega)
        self._prev_bearing = b

        # --- bu tre (loi B3)
        e = b + self._omega * c.lead_s

        # --- deadzone mem (loi B7)
        if abs(e) <= c.deadzone_deg:
            e_eff = 0.0
            self.note = "trong vung chet"
        else:
            e_eff = math.copysign(abs(e) - c.deadzone_deg, e)
            self.note = "quay phai" if e_eff > 0 else "quay trai"

        # --- P + feed-forward (loi B4)
        raw = c.gain * e_eff + c.k_ff * self._omega

        # --- toc do toi thieu khi muc tieu sap ra khoi khung (loi B2)
        if self.at_edge and abs(raw) < c.min_edge_rate:
            raw = math.copysign(c.min_edge_rate, e if e != 0 else raw)
            self.note += " (bien khung)"

        raw = max(-c.max_rate, min(c.max_rate, raw))

        # --- gioi han toc do thay doi
        step = c.slew * max(dt, 1e-3)
        self.yaw += max(-step, min(step, raw - self.yaw))
        if abs(self.yaw) < 0.15:
            self.yaw = 0.0
        return self.yaw
```

### 6.4 `target.py` — cổng ngoại hình và từ chối khi nhập nhằng

```python
import cv2, numpy as np, time


class Appearance:
    """Histogram HSV vung than tren. Re, du de chan phan lon vu trao nhan."""

    ACCEPT = 0.45      # Bhattacharyya; > nguong nay = khac nguoi
    MARGIN = 0.12      # hai ung vien cach nhau duoi nguong nay = nhap nhang

    @staticmethod
    def embed(frame, box):
        x1, y1, x2, y2 = [int(v) for v in box]
        h = max(y2 - y1, 1)
        ys, ye = y1 + int(0.30 * h), y1 + int(0.60 * h)
        roi = frame[max(ys, 0):max(ye, ys + 1), max(x1, 0):max(x2, x1 + 1)]
        if roi.size == 0:
            return None
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv], [0, 1], None, [16, 16], [0, 180, 0, 256])
        cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)
        return hist.flatten().astype(np.float32)

    @staticmethod
    def dist(a, b):
        if a is None or b is None:
            return 1.0
        return float(cv2.compareHist(a.reshape(16, 16), b.reshape(16, 16),
                                     cv2.HISTCMP_BHATTACHARYYA))


class LockedTarget:
    def __init__(self, nhan, track, mau, emb):
        self.nhan, self.tid = nhan, track.id
        self.last_box = track.box.copy()
        self.mau, self.emb = mau, emb
        self.t_lost, self.seen, self.n_reacq = None, True, 0
        self.needs_confirm = False        # -> SafetyGate se CHAN

    def update_embedding(self, emb, alpha=0.9):
        if emb is None:
            return
        self.emb = emb if self.emb is None else alpha * self.emb + (1 - alpha) * emb


class TargetManager:
    # ... phan pick_at / set_primary / next_primary giu nguyen ...

    def _try_reacquire(self, m, cands, frame, w):
        """Tra ve track duoc chon, hoac None.
        NGUYEN TAC: nhap nhang thi TU CHOI, khong doan."""
        if not cands:
            return None

        lx = float(m.last_box[0] + m.last_box[2]) / 2
        ly = float(m.last_box[1] + m.last_box[3]) / 2
        dt_lost = time.monotonic() - (m.t_lost or time.monotonic())
        # ban kinh nhan lai NO ra theo thoi gian mat, khong co dinh 0.25*w
        r_max = min(0.12 * w + 120.0 * dt_lost, 0.30 * w)

        scored = []
        for t in cands:
            cx = (t.box[0] + t.box[2]) / 2
            cy = (t.box[1] + t.box[3]) / 2
            d = ((cx - lx) ** 2 + (cy - ly) ** 2) ** 0.5
            if d > r_max:
                continue
            d_app = Appearance.dist(m.emb, Appearance.embed(frame, t.box))
            # ti trong: ngoai hinh quan trong hon vi tri
            scored.append((0.35 * (d / r_max) + 0.65 * d_app, d_app, t))

        if not scored:
            return None
        scored.sort(key=lambda s: s[0])

        if scored[0][1] > Appearance.ACCEPT:
            self.msg = f"{m.nhan}: ung vien khong khop ngoai hinh"
            return None

        if len(scored) > 1 and (scored[1][0] - scored[0][0]) < Appearance.MARGIN:
            m.needs_confirm = True
            self.msg = (f"{m.nhan}: reacquire NHAP NHANG "
                        f"({len(scored)} ung vien) -> can bam xac nhan lai")
            return None

        return scored[0][2]

    # TODO: sau khi reacquire thanh cong, dat co `just_reacquired`
    #       de app.py ep yaw = 0 trong 0.5 s va nhay canh bao tren web
```

### 6.5 `webui.py` — token + nhịp sống người vận hành

```python
import hmac, secrets, threading, time


class OperatorLink:
    """Nguoi van hanh phai GIU nut tren trang web. Trang gui /alive moi 200 ms.
    Mat nhip > 1 s => SafetyGate chan."""

    def __init__(self):
        self.token = secrets.token_urlsafe(16)
        self._t_alive = 0.0
        self._lk = threading.Lock()
        self.stop_latch = threading.Event()   # uu tien tuyet doi (loi B8)

    def authed(self, q) -> bool:
        return hmac.compare_digest(q.get("t", [""])[0], self.token)

    def beat(self):
        with self._lk:
            self._t_alive = time.monotonic()

    @property
    def age(self) -> float:
        with self._lk:
            return time.monotonic() - self._t_alive if self._t_alive else 1e9

    def request_stop(self):
        self.stop_latch.set()


# --- trong handler:
#   moi endpoint doi trang thai:  if not op.authed(q): return 403
#   GET /alive?t=...   -> op.beat()
#   POST /engage       -> day vao queue
#   POST /disengage    -> op.request_stop()   (KHONG qua queue)
#
# --- ben trinh duyet: nut ENGAGE la nut GIU
#   btn.onpointerdown = () => alive = setInterval(()=>fetch('/alive?t='+T), 200)
#   btn.onpointerup   = () => { clearInterval(alive); fetch('/disengage?t='+T) }
#   window.onblur     = () => fetch('/disengage?t='+T)
```

### 6.6 `app.py` — bộ khung vòng lặp chính

```python
def run(cfg):
    link     = PX4Link(cfg.mavlink, cfg.baud) if cfg.mavlink else None
    streamer = None
    if link:
        streamer = SetpointStreamer(link, cfg.sp_rate,
                                    use_position_hold=cfg.position_hold)
        streamer.start()

    op   = OperatorLink()
    gate = GateConfig(enable_control=cfg.enable_control, rc_chan=cfg.rc_chan, ...)
    ctl  = YawController(YawConfig(...), width_px=cam.width)
    sink = LoggingSink(cfg.log_csv, cfg.save_video)   # luong rieng (loi D3)

    engaged = False
    t_loop  = Stamped(); t_loop.set(True)
    watchdog = LoopWatchdog(t_loop, timeout=1.0,
                            on_trip=lambda: op.request_stop())
    watchdog.start()

    def disengage(reason, hard=True):
        nonlocal engaged
        if engaged:
            print(f"\n  !! NGAT DIEU KHIEN: {reason}")
        engaged = False
        ctl.yaw = 0.0
        if streamer:
            streamer.command(0.0)
        if hard and link:
            # A2: THUC SU thoat offboard, khong chi dat yaw ve 0
            if cfg.on_abort == "loiter":
                if not link.enter_loiter():
                    print("  !! khong vao duoc LOITER -> ngung stream setpoint")
                    streamer.stop()
            elif cfg.on_abort == "stream-off":
                streamer.stop()
            streamer.clear_hold()

    while not _stop.is_set():
        t_loop.set(True)                       # nhip cho watchdog

        # 0. chot dung co uu tien tuyet doi (loi B8)
        if op.stop_latch.is_set():
            op.stop_latch.clear()
            drain(cmd_queue)
            disengage("nguoi van hanh dung")

        # 1. lay khung MOI (loi C5)
        frame, seq, t_cap = cam.read()
        if frame is None or seq == last_seq:
            time.sleep(0.002); continue
        last_seq = seq

        # 2. vision
        tracks = detect_and_track(frame)
        target = tm.update(tracks, frame)

        # 3. lenh tu web (khong bao gom lenh dung)
        handle_commands(cmd_queue, tm, ...)

        # 4. gate
        s = GateInputs(
            engaged=engaged,
            operator_age=op.age,
            link_connected=bool(link) and link.connected,
            hb_age=link.hb.age if link else 1e9,
            armed=link.armed if link else False,
            mode=link.mode if link else "?",
            pos_valid=bool(link) and link.pos.age < 1.0,
            drift_m=streamer.drift_from_hold(link.pos.value) if streamer else 0.0,
            rc_value=(link.rc.value or {}).get(cfg.rc_chan, 0) if link else 0,
            rc_age=link.rc.age if link else 1e9,
            batt_v=link.batt.value if link else 0.0,
            batt_age=link.batt.age if link else 1e9,
            cam_age=time.monotonic() - t_cap,   # tuoi THUC cua khung dang dung
            has_target=bool(tm.primary_target),
            target_age=...,
            target_needs_confirm=any(m.needs_confirm for m in tm.locked),
            loop_age=0.0,
            tx_watchdog_trips=streamer.watchdog_trips if streamer else 0)

        res = check(gate, s)
        if engaged and not res.allow and res.fatal:
            disengage(res.reason)

        # 5. dieu khien
        yawspeed = math.degrees(link.att.value[3]) if (link and link.att.value) else 0.0
        yaw = ctl.update(target.box if target else None, dt, res.allow, yawspeed)
        if streamer:
            streamer.command(yaw if res.allow else 0.0)   # co han su dung

        # 6. log + web (day sang luong khac, KHONG chan vong dieu khien)
        sink.push(build_row(...), frame if cfg.need_video else None)
```

### 6.7 `tests/` — bộ kiểm thử tối thiểu

```python
# test_safety.py -------------------------------------------------------
import pytest
from follow.safety import check, GateConfig, GateInputs

def ok_inputs(**kw):
    base = dict(engaged=True, operator_age=0.1, link_connected=True,
                hb_age=0.2, armed=True, mode="OFFBOARD", pos_valid=True,
                drift_m=0.3, rc_value=1800, rc_age=0.1, batt_v=15.2,
                batt_age=0.5, cam_age=0.05, has_target=True, target_age=0.1,
                target_needs_confirm=False, loop_age=0.02, tx_watchdog_trips=0)
    base.update(kw)
    return GateInputs(**base)

CFG = GateConfig(enable_control=True, rc_chan=7, rc_min=1500, batt_min=14.0)

def test_duong_co_ban():
    assert check(CFG, ok_inputs()).allow

@pytest.mark.parametrize("kw,frag,fatal", [
    ({"rc_value": 1200},              "cong tac RC",        True),
    ({"rc_age": 3.0},                 "du lieu RC cu",      True),   # A4
    ({"mode": "POSCTL"},              "khong o OFFBOARD",   True),   # A3
    ({"hb_age": 9.0},                 "mat heartbeat",      True),
    ({"armed": False},                "chua arm",           True),
    ({"batt_v": 13.1},                "pin thap",           True),
    ({"batt_age": 30.0},              "du lieu pin cu",     True),
    ({"cam_age": 2.0},                "mat camera",         True),
    ({"loop_age": 1.5},               "vong lap treo",      True),   # A1
    ({"operator_age": 3.0},           "nhip nguoi van hanh",True),   # A5
    ({"pos_valid": False},            "uoc luong vi tri",   True),   # A7
    ({"drift_m": 5.0},                "troi",               True),   # A7
    ({"has_target": False},           "chua khoa",          False),
    ({"target_age": 2.0},             "muc tieu cu",        False),
    ({"target_needs_confirm": True},  "xac nhan lai",       False),  # C3
])
def test_tung_dieu_kien_chan_doc_lap(kw, frag, fatal):
    r = check(CFG, ok_inputs(**kw))
    assert not r.allow
    assert frag in r.reason or any(frag in x for x in r.all_reasons)
    assert r.fatal is fatal


# test_control.py ------------------------------------------------------
def test_bearing_dung_cong_thuc_pinhole():
    ctl = YawController(YawConfig(hfov_deg=90.0), width_px=1280)
    assert abs(ctl.bearing_deg(1280) - 45.0) < 0.01     # bien phai
    assert abs(ctl.bearing_deg(960)  - 26.57) < 0.05    # KHONG phai 22.5  (B1)
    assert abs(ctl.bearing_deg(640)) < 1e-6

def test_box_cham_bien_van_lenh_toi_thieu():
    ctl = YawController(YawConfig(min_edge_rate=8.0), 1280)
    box = np.array([1100., 100., 1280., 600.])          # cat o bien phai
    y = ctl.update(box, dt=0.05, allow=True)
    assert y >= 0.4          # slew 80 do/s^2 * 0.05 s -> bat dau tang, dung dau
    assert ctl.at_edge

def test_mat_muc_tieu_ve_0_ngay_bo_qua_slew():
    ctl = YawController(YawConfig(slew=5.0), 1280)
    ctl.yaw = 35.0
    assert ctl.update(None, dt=0.05, allow=True) == 0.0

def test_gate_chan_thi_yaw_0():
    ctl = YawController(YawConfig(), 1280)
    ctl.yaw = 20.0
    assert ctl.update(np.array([900., 0., 1000., 400.]), 0.05, allow=False) == 0.0


# test_mavlink.py — chay voi PX4 gia qua UDP loopback ------------------
def test_watchdog_TX_ep_yaw_ve_0(fake_px4):
    """A1: khong nuoi lenh -> setpoint phai ve 0 trong ~0.25 s."""
    streamer.command(30.0)
    assert_yaw_rate_gan(fake_px4.next_setpoint(), 30.0)
    time.sleep(0.6)                                  # mo phong main loop treo
    assert_yaw_rate_gan(fake_px4.next_setpoint(), 0.0)
    assert streamer.watchdog_trips >= 1

def test_type_mask_vi_tri_yawrate(fake_px4):
    """A7: dung setpoint vi tri, khong phai van toc."""
    streamer.arm_hold((10.0, -4.0, -25.0))
    sp = fake_px4.next_setpoint()
    assert sp.type_mask == 1528
    assert (sp.x, sp.y, sp.z) == pytest.approx((10.0, -4.0, -25.0))

def test_khong_engage_khi_offboard_bi_tu_choi(fake_px4):
    """A3: PX4 tra ve TEMPORARILY_REJECTED -> khong duoc bao la dang engaged."""
    fake_px4.reject_next_mode_change()
    assert link.enter_offboard() is False

def test_time_boot_ms_khong_reset(fake_px4):
    """B5: timestamp phai tang don dieu qua moc 1 giay."""
    ts = [fake_px4.next_setpoint().time_boot_ms for _ in range(60)]
    assert all(b >= a for a, b in zip(ts, ts[1:]))
    assert ts[-1] > 1000
```

---

## 7. Kế hoạch kiểm thử

### 7.1 Năm mức, không bỏ mức nào

| Mức | Môi trường | Mục đích | Điều kiện qua |
| --- | --- | --- | --- |
| **T0** | pytest, không phần cứng | Logic gate + điều khiển | 100% test ở §6.7 xanh |
| **T1** | PX4 SITL + video ghi sẵn | Vòng đời Offboard thật | Vào/thoát Offboard 20/20 lần; watchdog trip đúng |
| **T2** | Bench: FMU + camera, **không có sải cánh** | Đường đi dữ liệu thật | RC age < 0.3 s; `sp_hz` ≥ 19; trễ đo được |
| **T3** | Drone tháo cánh quạt, trên bàn | Chiều quay, dead-man, hiển thị | 3 kiểm tra ở §7.3 |
| **T4** | Buộc dây, ngoài trời | Động lực học thật | Không dao động; trôi < 1 m/phút |
| **T5** | Bãi trống, bay tự do | Nghiệm thu | Bảng §7.4 |

### 7.2 SITL — bắt buộc trước mọi thứ khác

Tất cả 7 lỗi P0 đều tái hiện và kiểm chứng được trong SITL, **miễn phí và không rủi ro**. Không có lý do gì để bỏ bước này.

Kịch bản SITL cần chạy:
1. ENGAGE bình thường → xác nhận heartbeat báo OFFBOARD.
2. ENGAGE khi chưa arm → phải bị từ chối, `STATUSTEXT` hiển thị lý do.
3. Đang bám, `kill -STOP` tiến trình 2 giây rồi `kill -CONT` → drone phải đã dừng xoay (watchdog TX).
4. Đang bám, hạ RC ch7 xuống 1200 → phải thoát Offboard, mode chuyển AUTO.LOITER trong < 0.5 s.
5. Đang bám, ngắt stream `RC_CHANNELS` (nhưng giữ heartbeat) → phải chặn vì "du lieu RC cu".
6. Đang bám, đóng tab trình duyệt → phải disengage trong < 1.5 s.
7. Thổi gió trong sim → xác nhận vị trí giữ được (setpoint vị trí), so với setpoint vận tốc để thấy khác biệt.
8. Chạy liên tục 60 phút → không rò bộ nhớ, `sp_hz` không tụt, không có watchdog trip.

### 7.3 Ba kiểm tra bắt buộc ở T3 (tháo cánh quạt)

Giữ nguyên từ README gốc, bổ sung tiêu chí số:
1. **Chiều quay.** Người đứng bên phải khung → `yaw_cmd_deg_s > 0` → thân drone xoay sang phải (nhìn từ trên xuống, cùng chiều kim đồng hồ). Nếu sai → dùng `--invert-yaw`, **không** sửa dấu trong code.
2. **Dead-man.** Buông ch7: đo thời gian từ lúc buông tới lúc `yaw_cmd_deg_s = 0` trong CSV. **Phải < 0.5 s.** Ghi con số này vào hồ sơ.
3. **Tần số setpoint.** `sp_hz` ≥ 19.0 trong suốt 10 phút chạy, kể cả khi có 2 trình duyệt đang xem stream.

Thêm hai kiểm tra mới:
4. **Trễ vòng kín.** Vẫy tay đột ngột từ giữa ra rìa khung, đo từ khung hình có chuyển động tới bản tin MAVLink tương ứng. Ghi vào hồ sơ, dùng làm giá trị `--lead`.
5. **Watchdog.** `kill -STOP <pid>` 1 giây; xác nhận log PX4 cho thấy yaw_rate về 0.

### 7.4 Tiêu chí nghiệm thu T5

| Hạng mục | Ngưỡng |
| --- | --- |
| Thời gian giữ mục tiêu trong ±15° | ≥ 90% thời lượng bay, người đi bộ |
| Vọt lố sau khi mục tiêu dừng | ≤ 8° |
| Dao động (chu kỳ giới hạn) | Không quan sát được |
| Số lần tráo nhãn ngoài ý muốn | 0 |
| Số lần watchdog TX trip | 0 |
| Trôi vị trí | < 2 m trong 5 phút, gió < 5 m/s |
| Thời gian phản hồi dead-man | < 0.5 s, đo lại trên thực địa |
| `sp_hz` tối thiểu trong suốt chuyến | ≥ 18 |

---

## 8. Thứ tự công việc đề xuất

| Đợt | Nội dung | Ước lượng | Chặn cái gì |
| --- | --- | --- | --- |
| **1** | A1, A2, A3, A4, A6 + bộ test T0 | 3–4 ngày | Mọi thử nghiệm có cánh quạt |
| **2** | A5, A7, B5, B6, B8 | 2–3 ngày | Bay ngoài trời |
| **3** | Chạy toàn bộ kịch bản SITL (§7.2) | 2 ngày | T3 |
| **4** | B1, B2, B7 + hiệu chuẩn camera | 2 ngày | Tinh chỉnh gain |
| **5** | C1 (hiệu chỉnh lại INT8) + bài soát false positive | 3–4 ngày | Gate P2 |
| **6** | B3, B4 (đo trễ, feed-forward, tinh chỉnh gain) | 3 ngày | Bám người chạy |
| **7** | C2, C3 (ngoại hình + từ chối nhập nhằng) | 3 ngày | Bay có nhiều người |
| **8** | Tái cấu trúc module, C4, C5, D1–D3 | 4–5 ngày | Bảo trì dài hạn |

Đợt 1 và 2 là **điều kiện cần tuyệt đối**. Đợt 3 phát hiện những gì hai đợt đầu bỏ sót.

---

## 9. Sai lệch giữa README và code — phải sửa tài liệu

Ghi riêng vì người tiếp nhận sẽ đọc README trước khi đọc code.

| Mục README | Nội dung viết | Thực tế trong code |
| --- | --- | --- |
| §2, lớp 4 | "Watchdog… vòng lặp treo" | **Không tồn tại.** `SafetyGate` không có điều kiện nào về main loop |
| §2, dead-man | "Buông tay là drone thoát offboard và PX4 chuyển sang failsafe" | Sai. Setpoint tiếp tục chạy, drone ở nguyên Offboard |
| §1, bảng phạm vi | "Giữ độ cao: Phi công / chế độ bay PX4" | Trong Offboard phi công không có quyền cần trừ khi `COM_RC_OVERRIDE` bật đúng bit |
| §1, bảng phạm vi | "Bay tiến/lùi/ngang: Phi công" | Như trên |
| §7 | "gain 0.6 = sửa 60% sai số mỗi giây" | Đúng là 45% (`1 − e^−0.6`) |
| §2, dừng khẩn | "DISENGAGE trên web — trong một chu kỳ vòng lặp" | Đúng khi vòng lặp khoẻ; mất hiệu lực nếu loop treo, và lệnh có thể bị ghi đè (B8) |
| §5 | `--rc-chan` mặc định `0` (tắt) | Nên **bắt buộc** khi có `--enable-control`; hiện chỉ là khuyến nghị bằng lời |
| §10 | "Đã kiểm thử với PX4 giả" | Không thấy file test trong bàn giao — phải commit bộ harness này |
| §4 | Tên tham số PX4 | Tên và ngữ nghĩa (`COM_OBL_ACT`, `MPC_YAWRAUTO_MAX`) thay đổi theo phiên bản firmware — **xác minh lại trên đúng bản đang chạy**, không chép từ tài liệu này |

---

## 10. Điều kiện mở Pha 3

Spec Gate P2 chưa đóng. Danh sách còn thiếu, cộng với các mục mới phát sinh từ phân tích này:

**Từ hồ sơ P2 gốc**
- [ ]  Bài endurance 60 phút (ghi nhiệt độ CPU, FPS, RAM, số watchdog trip)
- [ ]  Bản chạy sạch không preview (`--no-web`, xác nhận FPS và ổn định)
- [ ]  Soát false positive (§C1, bước 4)

**Bổ sung bắt buộc từ tài liệu này**
- [ ]  Toàn bộ 7 lỗi P0 đã sửa và có test hồi quy
- [ ]  Bộ test T0 chạy trong CI, xanh
- [ ]  8 kịch bản SITL (§7.2) chạy qua, có log lưu lại
- [ ]  Hiệu chỉnh lại INT8 bằng ảnh camera thật, có báo cáo so sánh với FP32
- [ ]  Camera đã hiệu chuẩn nội tại, `fx` lưu ra file
- [ ]  Ba kiểm tra T3 hoàn tất, có số đo cụ thể (không chỉ "đạt")
- [ ]  README đã sửa theo §9
- [ ]  Danh sách tham số PX4 đã xác minh trên firmware đang dùng, có ảnh chụp màn hình QGC

Trong lúc chờ, `--enable-control` chỉ được bật ở T0–T3. Không lắp cánh quạt.

---

## 11. Bảng tóm tắt

| Nhóm | Số vấn đề | Rủi ro chính | Đợt xử lý |
| --- | --- | --- | --- |
| An toàn / failsafe (A1–A7) | 7 (P0) | Mất kiểm soát thật sự, không có đường thoát | 1–3 |
| Điều khiển (B1–B8) | 8 (P1) | Bám kém, sai số thường trực, timestamp sai | 4, 6 |
| Vision (C1–C6) | 6 (P1/P2) | Bám nhầm vật, tráo nhãn, tầm ngắn | 5, 7 |
| Kỹ thuật phần mềm (D1–D5) | 5 (P2) | Không hồi quy được, mất log, tài liệu sai | 8 |
| **Tổng** | **26** | — | ~22–26 ngày công |

**Ba việc làm đầu tiên, theo đúng thứ tự:** watchdog cho luồng TX (A1) → thoát Offboard thật sự khi ngắt (A2) → xác minh mode bằng ACK và heartbeat (A3). Ba việc này sửa hết trong một ngày và loại bỏ phần lớn rủi ro mất kiểm soát.