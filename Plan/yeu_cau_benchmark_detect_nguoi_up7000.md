# Yêu cầu bàn giao: Detect người trên UP7000 cho S500

## 1. Mục tiêu bàn giao

Xây dựng và đo kiểm một pipeline **phát hiện người chạy trực tiếp trên UP7000** với camera Rapoo C280/UVC. Đầu ra cần đủ số liệu để quyết định có qua Gate P2 của dự án trước khi phát triển chọn mục tiêu và tracker.

Phạm vi hiện tại chỉ là:

- Thu hình từ camera, phát hiện lớp `person`, hiển thị hoặc ghi kết quả.
- Ghi video và dữ liệu benchmark đồng bộ thời gian.
- Chạy ổn định trên UP7000 thật.

Không nằm trong phạm vi bàn giao:

- Không gửi lệnh MAVLink/PX4, không Offboard, không điều khiển drone.
- Không tự chọn người để theo dõi, không tracker, không nhận diện danh tính.
- Không cần truyền 4G hoặc xây UI từ xa ở giai đoạn này.

## 2. Phần cứng và bối cảnh

| Hạng mục | Cấu hình |
|---|---|
| Máy tính nhúng | UP7000, RAM 4 GB, eMMC 32 GB |
| Camera | Rapoo C280, kết nối USB/UVC |
| Hệ điều hành dự kiến | Ubuntu 22.04 hoặc bản đang dùng thực tế |
| Nền tảng ưu tiên | OpenCV/V4L2 hoặc GStreamer + OpenVINO |
| Đối tượng cần detect | `person` (người), không cần nhận dạng danh tính |

Nếu cấu hình thực tế khác bảng trên, ghi rõ trong báo cáo thay vì tự giả định.

## 3. Việc cần thực hiện

### 3.1. Xác nhận camera

1. Ghi đầu ra của `v4l2-ctl --all --list-formats-ext`.
2. Thử tối thiểu hai mode: `1280×720@30` và `640×480@30` (hoặc mode gần nhất camera thực sự hỗ trợ).
3. Ghi rõ camera có autofocus/exposure tự động hay không; nếu driver hỗ trợ, nêu các control đã khóa/chỉnh.
4. Xác nhận capture liên tục, không tự tăng frame queue và không mất camera trong bài test dài.

### 3.2. Detector

1. Dùng một detector gọn, ưu tiên YOLOv8n/YOLO11n đã chuyển OpenVINO hoặc model tương đương.
2. Chạy thử trên các device sẵn có: `CPU`, `GPU` và `AUTO` (nếu UP7000 không có device tương ứng, ghi `N/A`).
3. Thử input 640×640 trước; chỉ giảm input khi không đạt latency/RAM.
4. Chốt **một cấu hình đề xuất** cho các pha sau, dựa trên P95 latency và độ ổn định, không chỉ dựa vào FPS trung bình.
5. Pipeline phải dùng timestamp monotonic và sequence number tăng dần cho frame/detection.
6. Nếu xử lý chậm hơn camera, chỉ giữ frame mới nhất; không để backlog tích lũy. Mục tiêu frame queue ≤2.

### 3.3. Benchmark bắt buộc

Thực hiện các bài test sau ở đúng UP7000:

| Bài test | Điều kiện tối thiểu |
|---|---|
| Smoke test | 2–5 phút, xác nhận có box `person` và không crash |
| So sánh cấu hình | Các device/input size đã thử, mỗi cấu hình ít nhất 2 phút |
| Live endurance | Cấu hình đề xuất chạy liên tục 60 phút |
| Bối cảnh | Trong nhà, ngoài trời, ngược sáng/nắng, người gần/xa |
| Khó khăn | Người đi ngang, hai người xuất hiện, che khuất ngắn, camera rung nhẹ |
| Replay | Chạy lại ít nhất một video đã thu để kết quả có thể tái lập |

Không cần gắn camera lên drone hoặc bay trong pha này. Nếu đã gắn camera cố định đúng góc dự kiến thì ghi rõ góc và vị trí.

## 4. Dữ liệu phải ghi

Mỗi detection hoặc mỗi inference ghi một dòng JSONL/CSV. Các trường bắt buộc:

```json
{
  "frame_id": 12345,
  "t_capture_monotonic_ns": 0,
  "t_done_monotonic_ns": 0,
  "source": "live|replay",
  "width": 1280,
  "height": 720,
  "queue_depth": 0,
  "capture_ms": 0.0,
  "preprocess_ms": 0.0,
  "inference_ms": 0.0,
  "postprocess_ms": 0.0,
  "end_to_end_ms": 0.0,
  "detections": [
    {"class": "person", "confidence": 0.0, "x1": 0, "y1": 0, "x2": 0, "y2": 0}
  ]
}
```

Mỗi 1–5 giây ghi thêm telemetry hệ thống: CPU load, RAM used/available, nhiệt độ CPU/board nếu đọc được, FPS capture, FPS detector, dropped frames và lỗi camera/model. Không cần sửa đúng tên trường, nhưng phải giải thích rõ đơn vị.

## 5. Tiêu chí nghiệm thu Gate P2

| Hạng mục | Ngưỡng đạt |
|---|---:|
| Detector trên UP7000 | ≥10 inference/giây ở cấu hình chọn |
| Luồng camera/hiển thị | Mục tiêu ≥20 FPS nếu camera cho phép |
| Độ trễ P95 capture → detection | ≤200 ms |
| Frame queue | ≤2, không tăng dần theo thời gian |
| Ổn định | 60 phút không crash, reboot hoặc mất camera |
| RAM | Tổng dùng ≤ khoảng 3,2/4 GB |
| Nhiệt | Không thermal throttling nguy hiểm; phải báo nhiệt tối đa đo được |
| Chất lượng detect | Không có false positive người lặp lại nghiêm trọng ở bối cảnh dự kiến |

Nếu không đạt một ngưỡng, vẫn bàn giao đầy đủ số liệu và đề xuất nguyên nhân/cách tối ưu. Không che bỏ kết quả xấu.

## 6. Gói bàn giao bắt buộc

```text
detector_delivery/
├── README.md                 # cách cài, cách chạy, cách replay
├── requirements.txt          # hoặc môi trường/phiên bản package
├── config.yaml               # model, device, input size, threshold, camera mode
├── src/                      # mã nguồn hoặc link commit/repository
├── benchmark_summary.md      # bảng kết quả và cấu hình khuyến nghị
├── logs/
│   ├── live_60min.jsonl      # hoặc CSV tương đương
│   ├── replay.jsonl
│   └── system_metrics.csv
└── samples/
    ├── ảnh/clip minh họa có box
    └── danh sách false positive/false negative đáng chú ý
```

Không gửi video có người ra ngoài phạm vi dự án; chỉ chia sẻ các mẫu cần thiết, đã có sự đồng ý phù hợp.

## 7. Mẫu phản hồi ngắn để gửi lại

Người thực hiện điền và gửi cùng gói bàn giao:

```text
1. UP7000 / OS / OpenVINO / model / commit:
2. Camera mode đã chọn:
3. Device inference đã chọn (CPU/GPU/AUTO) và lý do:
4. Input size, confidence threshold, NMS threshold:
5. Capture FPS / detector FPS:
6. Latency P50 / P95 / P99 (ms):
7. Queue depth max / số frame bị drop:
8. RAM trung bình / tối đa; CPU load; nhiệt độ tối đa:
9. Kết quả live 60 phút (crash, camera reconnect, throttle nếu có):
10. Các bối cảnh đã test và lỗi detect quan trọng:
11. File/commit bàn giao:
12. Cấu hình đề xuất cho bước tracker:
```

## 8. Quy tắc chuyển sang bước tiếp theo

Chỉ sau khi có gói bàn giao trên, người phụ trách hệ thống sẽ đánh giá Gate P2 và quyết định cấu hình khóa cho Pha 3: chọn target, tracker, timeout và trạng thái `LOST`.

Output detector trong giai đoạn này không được nối sang Pixhawk hay bất kỳ kênh điều khiển bay nào.
