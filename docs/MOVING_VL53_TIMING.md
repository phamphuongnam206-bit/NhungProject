# Chuẩn bị đồng bộ VL53 với pose khi xe chuyển động

Firmware dùng `startContinuous()` không truyền chu kỳ, tức chế độ VL53L0X
continuous back-to-back. Theo [datasheet ST](https://www.st.com/resource/datasheet/vl53l0x.pdf),
phép đo kế tiếp bắt đầu ngay khi phép đo trước kết thúc. Thanh ghi data-ready
cho biết kết quả đã sẵn sàng; nó không ghi timestamp của giai đoạn đo.

Mốc này thêm hai thành phần phần mềm độc lập:

- `PlannerPoseWindow::interpolateAt(t)` nội suy x/y và góc theo cung ngắn nhất
  giữa hai mẫu pose liên tiếp. Nó từ chối timestamp ngoài history, khoảng trống
  mẫu quá 25 ms và dữ liệu không hữu hạn. Vòng qua mốc `millis()` được test.
- `PlannerRangeReadyTracker` lưu khoảng từ lần cuối đọc status chưa sẵn sàng
  tới lần đầu đọc status sẵn sàng của từng VL53. Giá trị
  `ready_bracket_ms[i] == UINT16_MAX` nghĩa là không quan sát được cạnh. Đây chỉ
  là khoảng của **thời điểm hoàn tất**, không phải thời điểm exposure.

Trong VS Code hoặc STM32CubeIDE, có thể watch `rangeReadyTracker.ready_bracket_ms[0..3]`
và `poseHistory.count` khi dùng bản chẩn đoán khóa motor. Firmware hiện vẫn chỉ
gửi `range_poses_ready` khi pose đứng yên đủ 100 ms như mốc trước. Nó không gọi
`interpolateAt()` bằng thời điểm đọc VL53, vì như vậy sẽ gắn pose sai cho xe
đang chạy.

Điều kiện trước khi cho phép mapping khi chuyển động: xác định được thời điểm
đo hoặc một khoảng đo có giới hạn trên phần cứng đang dùng; đo độ trễ status/I2C;
đảm bảo pose history phủ khoảng đó và sai số chiếu hit không vượt độ phân giải
ô bản đồ. Nếu không đạt, tiếp tục dừng an toàn hoặc lập bản đồ sau khi xe đứng
yên. Không tự đổi `HYBRID_PLANNER_INPUTS_VERIFIED` hoặc khóa motor.

Test host: `planner_pose_window_test.cpp` kiểm tra nội suy, yaw qua ±180 độ,
timestamp tràn và mất mẫu; `planner_range_ready_test.cpp` kiểm tra không có cạnh,
cạnh mới, reset và timestamp tràn. Build `hybrid_planner_check` PASS: Flash span
61692/64512 byte, RAM tĩnh 14784 byte, còn 4160 byte sau phần dự phòng linker.
Đây chưa phải kiểm thử thời gian thực hay phần cứng PASS.
