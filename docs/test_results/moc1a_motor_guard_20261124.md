# Mốc 1A — dừng thử motor khi mất xung encoder

Nhánh này chỉ sửa `src/pid_calibration.cpp`. Lệnh `MOTOR` và `SWEEP` ghi thời điểm xung cuối của từng bánh đang được cấp PWM. Nếu bánh đó không có xung trong `STALL_MS = 850 ms` đã có sẵn, firmware gọi `latchFault("STALL_L"/"STALL_R")`: PB0/STBY xuống LOW, PWM về 0, chế độ chuyển sang PAUSED, in `MOTOR_ABORT FAULT=...`. Lỗi được giữ cho đến khi reset; `STOP` vẫn dừng đầu ra. Bộ hẹn xung không khởi tạo lại ở mỗi bước `SWEEP` 800 ms, để SWEEP không bỏ lọt mất xung kéo dài qua ranh giới hai bước.

Thời gian lệnh MOTOR vẫn giới hạn 200–3000 ms. Lệnh ngắn hơn 850 ms kết thúc bởi giới hạn thời gian trước khi guard mất xung tác động. Guard kiểm tra **có xung**, chưa kiểm tra đúng chiều: dấu encoder theo chiều motor chưa được đo khi cấp VM. Một xung nhiễu có thể làm mới mốc thời gian; do đó chưa coi đây là bảo vệ độc lập trước mọi lỗi phần cứng. SysTick guard cũ cho trễ vòng điều khiển vẫn giữ nguyên.

## Kiểm tra phần mềm

- PlatformIO `run -e pid_calibration -t clean`: PASS.
- PlatformIO `run -e pid_calibration`: PASS (ARM firmware, không upload). Flash 56852/64512 byte (88,1%); RAM tĩnh 5284/20480 byte (25,8%). Warning `boolean` deprecated từ Pololu VL53L0X như baseline, không có lỗi build mới.
- Diff chỉ thay đổi đường MOTOR/SWEEP và chú thích ở đầu file. Pinout, PI, cảm biến và `ticks_per_rev` không đổi.
- Nạp `platformio run -e pid_calibration -t upload` qua ST-Link: **PASS** (`Programming Finished`, `Verified OK`, `SUCCESS`). OpenOCD vẫn in hai dòng `can't assert SRST`; đã đọc lại trạng thái sau nạp bằng SWD, nên không dựa riêng vào thông báo reset.
- Khi pin VM đã tháo và hai bánh đứng yên: GPIOB CRL cấu hình PB0 là output; GPIOB ODR bit 0 = 0 và IDR bit 0 = 0; TIM2/TIM3 có encoder mode 3, CNT = 0/0 và đứng yên sau 3 giây. RAM báo `allReady=1`, `fault=null`, `arming=0`, `paused=1`, `guardExpired=0`, `tickSeen=1`, `motorJob.active=0`. Đây là **PASS cho nạp và boot an toàn với VM tắt**, chưa phải PASS cơ chế stall.
- USB–UART CH340 được nhận ở COM8 sau khi cắm lại cùng ST-Link. Gửi `HELP\n` ở 115200 baud, nhận đủ 692 byte bắt đầu bằng `PI_CAL_V5 UART115200. Boot locked...`: **PASS đường UART hai chiều**. SWD cùng lúc báo `paused=1`, `fault=null`, PB0 ODR/IDR bit 0 = 0. COM3 trước đó không trả lời vì ST-Link đã bị rút; COM8 là cổng hiện tại, không phải số cổng cố định.
- Người dùng đo bằng đồng hồ tại chân STBY/SBTY của TB6612 so với GND, khi pin VM đã tháo: **gần 0 V**. Đây là phép đo vật lý do người dùng báo, chưa đo lại khi cấp pin VM.
- Sau khi cắm pin, người dùng đo tại TB6612 so với GND: **STBY ≈ 0 V, VCC logic ≈ 4,27 V, VM ≈ 8,1 V**. SWD đọc `paused=1`, `fault=null`, PB0 ODR/IDR bit 0 = 0; gửi `STOP\n` qua COM8 nhận `PAUSED`. Chưa có lệnh MOTOR. Datasheet Toshiba ghi dải hoạt động VCC 2,7–5,5 V, VM 2,5–13,5 V và VIH(min)=0,7×VCC (ở 4,27 V là 2,989 V); mức HIGH thực tế tại chân driver chưa đo, điện áp định mức motor N20 chưa xác nhận. Nguồn logic 4,27 V cũng chưa được giải thích so với net `VCC5V` trên sơ đồ. Nguồn datasheet: https://toshiba.semicon-storage.com/info/TB6612FNG_datasheet_en_20141001.pdf?did=10660&prodName=TB6612FNG
- Kiểm tra riêng tình huống mất xung liên tục, thời gian thực tới STBY LOW và dòng UART `MOTOR_ABORT`: **NOT_TESTED_ON_HARDWARE**; ba lượt chạy bình thường bên dưới không tạo tình huống này.

## Thử MOTOR đơn bánh khi kê xe (người dùng cho phép, VM khoảng 8,1 V)

Nguồn motor định mức N20 chưa nhận dạng. Người dùng xác nhận hai bánh kê khỏi mặt bàn, dây ổn định và không có nóng/mùi lạ. Trước mỗi lượt: `STOP`, `TRACE 1` trả `PAUSED`, `TRACE_ON`; sau mỗi lượt gửi `STOP`. Các số dưới đây là log UART thực tế, chưa phải đo tốc độ bánh độc lập.

| Lệnh | Kết quả UART | Nhận xét |
| --- | --- | --- |
| `MOTOR R 80 1200` | `MOTOR_DONE`, `COUNTS_R=366`, `COUNTS_L=-1`, `FAULT=NONE`, `PAUSED`; PWM_R đọc lại 80 | RAW_R khoảng 650–1274 counts/s đầu lượt, về 0–18 gần cuối dù PWM vẫn 80. Người dùng thấy bánh phải vẫn quay suốt lượt. |
| `MOTOR R 100 1200` | `MOTOR_DONE`, `COUNTS_R=247`, `COUNTS_L=-5`, `FAULT=NONE`, `PAUSED`; PWM_R đọc lại 100 | RAW_R khoảng 982–794 đầu lượt, rồi phần lớn gần 0–168. Người dùng báo bánh vẫn quay, không nóng/mùi lạ. Tăng PWM lại cho số đếm thấp hơn, chưa giải thích. |
| `MOTOR L 80 1200` | `MOTOR_DONE`, `COUNTS_L=-2852`, `COUNTS_R=0`, `FAULT=NONE`, `PAUSED`; PWM_L đọc lại 80 | RAW_L giữ khoảng 2333–2595 counts/s sau khởi động. Người dùng xác nhận chỉ J13 quay đều, J12 đứng yên. |

Sau các lượt, SWD báo `paused=1`, `fault=null`, PB0 ODR/IDR bit 0 = 0. **J13: PASS bước đầu về cấp PWM, nhận xung, đúng bánh và tự dừng. J12: FAIL về tính ổn định số xung khi motor đang quay**; chưa xác định do encoder, dây, nguồn, driver hay cơ khí. `STALL_MS` chưa được kích hoạt vì J12 vẫn có xung rải rác; không coi guard mất xung đã PASS. Chưa thử đảo chiều, STOP giữa lúc motor chạy, điều khiển PI hoặc tải trên mặt đất. Dừng thử motor thêm cho đến khi kiểm tra riêng J12.

## Điều kiện kiểm thử phần cứng sau này

Giữ bánh kê khỏi mặt đất; xác minh nguồn VM, GND chung, TB6612 và nút/ngõ dừng. Nạp đúng môi trường `pid_calibration`, xác nhận boot PAUSED và PB0 LOW trước khi cấp VM. Dùng một lệnh MOTOR đơn bánh có thời gian đủ dài để quan sát, đo xung và STBY; nếu không có xung, kỳ vọng `MOTOR_ABORT FAULT=STALL_L/R`, STBY LOW và chỉ chạy lại sau reset. Không cố giữ trục để tạo stall. Kết quả phần cứng chỉ được ghi PASS sau khi có log/đo thực tế.
