# Mốc kế tiếp: log encoder và cảm biến khi motor khóa

Nhánh này tiếp tục `one_run_safe`. Firmware giữ PB0/STBY LOW, không gọi
`control()` hoặc `applyMotors()`. Nó dùng lại pinout hiện tại: encoder trái
TIM3 PA6/PA7, encoder phải TIM2 PA0/PA1, MPU trên PB6/PB7, VL53L0X trên
PB10/PB11 với XSHUT PA2..PA5, UART PA9/PA10 ở 38400 baud.

Sau reset, UART in `ONE_RUN_LOCKED TELEMETRY ?=status`, rồi mỗi giây in
`TIM3_L`, `TIM2_R`, `MPU_WHO`, `VL_INIT` và `J8`..`J11`. Gửi `?` để in ngay.
`TIM3_L`/`TIM2_R` là tổng tick có dấu kể từ reset; không dùng giá trị
`COUNTS_PER_MM_*` ước lượng. `MPU_WHO` là byte thanh ghi 0x75, chưa chứng minh
gyro hay góc đúng. `Jx=-1` nghĩa là chưa có mẫu hợp lệ. Khởi tạo VL ghi cấu
hình cảm biến và chạy đo liên tục; nếu một trong bốn cảm biến thiếu hoặc lỗi,
`VL_INIT=FAIL` và phép thử cả dãy chưa PASS.

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e one_run_safe
```

Kiểm thử trên PCB chỉ sau khi nạp đúng env `one_run_safe`, tháo pin motor,
cấp 3,3 V cho logic và nối UART. PASS phần mềm: build thành công, STBY chỉ
được kéo LOW trong nhánh này. PASS phần cứng cần đo STBY gần 0 V và xác nhận:
bánh trái làm đổi `TIM3_L`, bánh phải làm đổi `TIM2_R`, WHO hợp lý, từng J8..J11
cho mm thay đổi theo khoảng cách. FAIL nếu STBY lên HIGH, hai bộ đếm không đổi,
hoặc cảm biến báo `INIT_FAIL`/`STALE` kéo dài. Chưa có kết quả PCB ở mốc này.
