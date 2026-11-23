# Mốc 1A — dừng thử motor khi mất xung encoder

Nhánh này chỉ sửa `src/pid_calibration.cpp`. Lệnh `MOTOR` và `SWEEP` ghi thời điểm xung cuối của từng bánh đang được cấp PWM. Nếu bánh đó không có xung trong `STALL_MS = 850 ms` đã có sẵn, firmware gọi `latchFault("STALL_L"/"STALL_R")`: PB0/STBY xuống LOW, PWM về 0, chế độ chuyển sang PAUSED, in `MOTOR_ABORT FAULT=...`. Lỗi được giữ cho đến khi reset; `STOP` vẫn dừng đầu ra. Bộ hẹn xung không khởi tạo lại ở mỗi bước `SWEEP` 800 ms, để SWEEP không bỏ lọt mất xung kéo dài qua ranh giới hai bước.

Thời gian lệnh MOTOR vẫn giới hạn 200–3000 ms. Lệnh ngắn hơn 850 ms kết thúc bởi giới hạn thời gian trước khi guard mất xung tác động. Guard kiểm tra **có xung**, chưa kiểm tra đúng chiều: dấu encoder theo chiều motor chưa được đo khi cấp VM. Một xung nhiễu có thể làm mới mốc thời gian; do đó chưa coi đây là bảo vệ độc lập trước mọi lỗi phần cứng. SysTick guard cũ cho trễ vòng điều khiển vẫn giữ nguyên.

## Kiểm tra phần mềm

- PlatformIO `run -e pid_calibration -t clean`: PASS.
- PlatformIO `run -e pid_calibration`: PASS (ARM firmware, không upload). Flash 56852/64512 byte (88,1%); RAM tĩnh 5284/20480 byte (25,8%). Warning `boolean` deprecated từ Pololu VL53L0X như baseline, không có lỗi build mới.
- Diff chỉ thay đổi đường MOTOR/SWEEP và chú thích ở đầu file. Pinout, PI, cảm biến và `ticks_per_rev` không đổi.
- Nạp `platformio run -e pid_calibration -t upload` qua ST-Link: **PASS** (`Programming Finished`, `Verified OK`, `SUCCESS`). OpenOCD vẫn in hai dòng `can't assert SRST`; đã đọc lại trạng thái sau nạp bằng SWD, nên không dựa riêng vào thông báo reset.
- Khi pin VM đã tháo và hai bánh đứng yên: GPIOB CRL cấu hình PB0 là output; GPIOB ODR bit 0 = 0 và IDR bit 0 = 0; TIM2/TIM3 có encoder mode 3, CNT = 0/0 và đứng yên sau 3 giây. RAM báo `allReady=1`, `fault=null`, `arming=0`, `paused=1`, `guardExpired=0`, `tickSeen=1`, `motorJob.active=0`. Đây là **PASS cho nạp và boot an toàn với VM tắt**, chưa phải PASS cơ chế stall.
- Kiểm tra runtime mất xung, thời gian thực tới STBY LOW và dòng UART `MOTOR_ABORT`: **NOT_TESTED_ON_HARDWARE**. Chưa cấp VM hoặc chạy motor. Chưa quan sát banner UART và chưa đo bằng đồng hồ tại chân STBY của TB6612 trong bản mới.

## Điều kiện kiểm thử phần cứng sau này

Giữ bánh kê khỏi mặt đất; xác minh nguồn VM, GND chung, TB6612 và nút/ngõ dừng. Nạp đúng môi trường `pid_calibration`, xác nhận boot PAUSED và PB0 LOW trước khi cấp VM. Dùng một lệnh MOTOR đơn bánh có thời gian đủ dài để quan sát, đo xung và STBY; nếu không có xung, kỳ vọng `MOTOR_ABORT FAULT=STALL_L/R`, STBY LOW và chỉ chạy lại sau reset. Không cố giữ trục để tạo stall. Kết quả phần cứng chỉ được ghi PASS sau khi có log/đo thực tế.
