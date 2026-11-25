# Mốc 0B — nền firmware một lượt và kế hoạch hiệu chuẩn

Ngày kiểm kê: 24/11/2026. Đây là hồ sơ phần mềm, **không phải chứng nhận phần cứng**. Không nạp hoặc chạy motor trong mốc này.

## Lựa chọn nền firmware

| Nguồn | Có sẵn | Trở ngại cho một lượt |
| --- | --- | --- |
| `hybrid_2laps.cpp` | TIM2/TIM3, TIM4, PI, MPU, 4 VL53, kiểm tra cung, phanh, guard | `ROUTE[]` cố định và chu trình hai lượt/Bluetooth. |
| `xe_map_auto.cpp` | Bộ lập kế hoạch theo bản đồ, cảm biến và bảo vệ | Luồng dạy đường Bluetooth, EEPROM lưu route và điều kiện khởi động gắn với hai lượt. Cần tách thuật toán sau khi đo SRAM và rà an toàn. |
| `xe_auto_test.cpp` | Một lượt trên bản đồ có sẵn, encoder, PI, MPU, 4 VL53, kiểm tra cung, dừng an toàn | Chưa có A* toàn cục hoặc định vị vị trí đầu; hằng số chuyển động chỉ ước lượng. Nhánh chạy có thể tự arm sau 3 giây. |
| `pid_calibration.cpp` | Lệnh MOTOR/SPEED có thời hạn, trace, phát hiện stall/sai chiều, STBY và guard | Dành riêng cho thử trên bàn, RUN bị khóa; không làm firmware điều hướng. |

Chọn `xe_auto_test.cpp` làm **nền một lượt ở trạng thái khóa motor**. Môi trường `map_one_run` chỉ biên dịch đúng nguồn này với `ONE_RUN_CALIBRATED=0`; nhánh này trả về khỏi `setup()`/`loop()` sau telemetry, luôn kéo PB0 LOW, không tới hàm điều khiển motor. `MAP_ONE_RUN` thêm lỗi biên dịch nếu ai bật `ONE_RUN_CALIBRATED=1`. Không đổi `default_envs=hybrid_2laps`. Bluetooth dạy đường, EEPROM route và waypoint điều khiển tay không có trong nguồn được chọn. Đây chưa phải firmware cho xe tự chạy: trước khi mở khóa phải thay hằng số đo thật, tích hợp thuật toán và xét lại an toàn.

`xe_auto_test.cpp` dùng Arduino STM32 core cùng CMSIS/thanh ghi TIM2/TIM3, nên chưa đáp ứng mục tiêu dài hạn C11 + CMSIS thuần. Mốc 0B không viết lại firmware đang hoạt động; phải đưa quyết định chuyển kiến trúc vào mốc tích hợp sau.

## Baseline build độc lập

Lệnh: `platformio run -e hybrid_2laps -e map_auto -e xe_auto_test -e pid_calibration -e one_run_safe -e map_one_run` (CLI PlatformIO tại `C:\Users\Admin\.platformio\penv\Scripts\platformio.exe`). Chỉ build, không `upload`. Kết quả trên máy ngày 24/11/2026:

| Environment | Build | RAM / 20.480 B | Flash / 64.512 B |
| --- | --- | ---: | ---: |
| `hybrid_2laps` | PASS | 2.628 B (12,8%) | 50.012 B (77,5%) |
| `map_auto` | PASS | 14.896 B (72,7%) | 61.488 B (95,3%) |
| `xe_auto_test` | PASS | 2.192 B (10,7%) | 45.936 B (71,2%) |
| `pid_calibration` | PASS | 5.276 B (25,8%) | 56.644 B (87,8%) |
| `one_run_safe` | PASS | 3.044 B (14,9%) | 38.740 B (60,1%) |
| `map_one_run` | PASS | 3.044 B (14,9%) | 38.740 B (60,1%) |

Trước đó một lần build bị lỗi quyền ghi cache PlatformIO (`WinError 5`), không phải lỗi code. Lần chạy có quyền ghi đã hoàn tất 6/6 PASS. Có cảnh báo `boolean` deprecated từ thư viện VL53L0X; không có lỗi biên dịch/liên kết. Build PASS chỉ xác nhận mã nguồn và dung lượng tĩnh, không xác nhận encoder, cảm biến, watchdog hay motor trên PCB.

Kiểm tra chốt biên dịch bằng `arm-none-eabi-g++ -std=gnu++11 -fsyntax-only -DXE_AUTO_NATIVE_TEST -DMAP_ONE_RUN=1 -DONE_RUN_CALIBRATED=<0|1> -Iinclude src/xe_auto_test.cpp`: giá trị `0` thoát mã 0; cố bật giá trị `1` thoát mã 1 tại `#error "map_one_run must remain motor-locked..."`. Đây là kiểm tra phần mềm, chưa phải phép đo STBY khi reset.

## Tài nguyên và an toàn cần kiểm trên PCB

| Chức năng | Chân/tài nguyên | Kiểm tra |
| --- | --- | --- |
| Encoder phải J12, trái J13 | PA0/PA1 → TIM2, PA6/PA7 → TIM3 | Không remap TIM2/3; xung cả hai kênh phải đổi cùng lượt, dấu cần đo. |
| Driver TB6612 | PB12/PB13 + PB8/TIM4_CH3 (phải); PB14/PB15 + PB9/TIM4_CH4 (trái); STBY PB0 | `setup()` ghi LOW trước khi cấu hình OUTPUT; nhánh khóa ghi BRR PB0 mỗi vòng. Kiểm điện áp STBY thật khi reset. |
| MPU6050 | PB6/PB7, I2C1 | Không trùng TIM4 PB8/PB9. WHO_AM_I, bias và tuổi mẫu phải được ghi log. |
| VL53L0X J8..J11 | PB10/PB11, I2C2; XSHUT PA2..PA5 | Bốn địa chỉ 0x30..0x33; timeout và mẫu cũ phải gây dừng ở nhánh chạy. |
| UART | PA9 TX, PA10 RX, 38400 ở `map_one_run` | Chỉ telemetry/lệnh Z,Y,? ở nhánh khóa; không dùng Bluetooth dạy đường. |

Trong nhánh chạy cũ, `HAL_SYSTICK_Callback` kéo PB0 xuống khi vòng điều khiển quá 100 ms hoặc mẫu cảm biến quá 125 ms; `applyMotors` chỉ arm nếu không lỗi, lệnh khác 0 và mẫu dưới 100 ms. Đây là **guard dựa trên SysTick, không phải xác nhận IWDG phần cứng**. Khi `map_one_run` khóa, guard motor không được kích vì không arm. Cần kiểm riêng reset, I2C treo, sensor mất mẫu, encoder đứng xung, nguồn yếu và timer wrap trước khi cho xe chạy. Nút dừng UART không thay thế ngắt pin/nguồn vật lý.

Trước khi `setup()` cấu hình PB0, mức STBY phụ thuộc phần cứng PCB; phần mềm không thể bảo đảm LOW trong pha reset. Phải xác nhận điện trở kéo xuống/điện áp thật ở pha cấp nguồn trước khi mở khóa. Sơ đồ chân trong bảng là đối chiếu mã nguồn với pinout PCB đã chốt, chưa phải phép đo điện từng net.

## Sổ cấu hình tập trung

`include/map_one_run_calibration.h` khai báo schema. Kích thước thân 130 × 100 mm đã được người dùng đo và xác nhận; các trường còn lại vẫn `NEEDS_MEASUREMENT`. Số 0 chỉ là giá trị giữ chỗ, **không phải chuẩn chạy**. Không dùng 45/100 counts/cm hay track 9/12 cm từ code cũ làm số đo. Firmware 128 KiB nay đối chiếu dữ liệu hiệu chuẩn với hằng số đã biên dịch và từ chối planner nếu chưa khớp; việc đối chiếu này chưa chứng minh motor an toàn để mở khóa.

| Nhóm | Trường | Trạng thái / cách đo |
| --- | --- | --- |
| Encoder | counts/cm trái, phải; dấu chiều tiến từng bánh | NEEDS_MEASUREMENT; quay theo mốc quãng đường có thước, ghi raw ticks tiến/lùi. |
| Hình học | đường kính từng bánh; track hiệu dụng; dài/rộng thân | Thân 130 × 100 mm: VERIFIED theo phép đo người dùng. Đường kính bánh và track hiệu dụng: NEEDS_MEASUREMENT; track từ quay có góc chuẩn. |
| MPU | dấu gyro Z; bias deg/s; drift deg/phút | NEEDS_MEASUREMENT; đứng yên đủ 500 mẫu, đo trôi tĩnh và hai chiều quay có mốc góc. |
| Motor | PWM khởi động tiến/lùi từng bánh; 5 điểm PWM–tốc độ mỗi bánh | NEEDS_MEASUREMENT; đo bằng bài MOTOR/SWEEP khi kê bánh. |
| Động học | gia tốc, giảm tốc, quãng phanh | NEEDS_MEASUREMENT; đo khi xe chạy trong vùng thử có chặn ngoài. |
| VL53 | x/y/yaw của J8, J9, J10, J11 so với tâm trục bánh | NEEDS_MEASUREMENT; đo bằng thước và mốc hướng, rồi đối chiếu tường/vật phẳng. |

Mỗi kết quả phải lưu số thô, đơn vị, ngày, cấu hình pin/nguồn, chiều quan sát, số lượt và sai số; chỉ chuyển từng trường sang `VERIFIED` sau khi phép thử lặp lại đạt tiêu chí. Gate `PlannerKinematicsReady` trên bản 128 KiB yêu cầu trạng thái đã xác nhận và so với hằng số điều khiển; giá trị giữ chỗ không thể cho phép planner chạy.

## Kịch bản thử dùng `pid_calibration`

Chỉ nạp firmware kiểm thử khi người vận hành chủ động. `RUN/FOLLOW` của firmware này bị khóa. Kiểm đúng PCB, nguồn logic, VM, dây và kê bánh trước mọi lệnh motor; sẵn `STOP` và cách ngắt VM vật lý. Lưu toàn bộ UART 115200 có timestamp, mã firmware/build, video khi cần. Khi gặp lỗi, gửi `STOP`, cắt VM, chỉ cấp lại sau khi kiểm tra.

| Bài | Thực hiện / biến và log | PASS | FAIL và dừng |
| --- | --- | --- | --- |
| 1. Encoder tiến/lùi | **VM tháo**. Đánh dấu bánh và mốc; quay từng bánh tiến rồi lùi số vòng xác định. Ghi TIM2/TIM3 raw count, chiều, số vòng, thời gian, C1/C2 nếu có trace. | Chỉ timer đúng bánh đổi; hai kênh có xung; đổi chiều thì dấu đảo; về mốc sai số trong dung sai đã định trước theo độ rơ. | Chỉ một kênh, sai bánh, không đảo dấu, trôi khi đứng yên: dừng, kiểm nguồn/cáp/pin. |
| 2. Motor có thời hạn | Kê bánh, gắn VM, `TRACE 1`, rồi `MOTOR L 80 500`, `MOTOR R 80 500` sau khi kiểm ngưỡng an toàn. Ghi PWM thực, raw ticks mỗi 100 ms, STBY, thời điểm dừng. | Đúng bánh/chiều; lệnh tự hết sau 500 ms; STOP kéo STBY LOW; không nóng/mùi lạ. | Quay sai bánh, không tự dừng, nguồn sụt, nóng: STOP và ngắt VM ngay. Không tăng PWM để che lỗi. |
| 3. Counts/cm | Đo chu vi bánh và quãng đường đi thật bằng thước/mốc; ghi số tick trước/sau, độ trượt, chiều, ít nhất nhiều lượt mỗi bánh. | Giá trị từng bánh lặp lại trong dung sai đã chọn theo dụng cụ; dấu khớp bài 1. | Không có quãng đường tin cậy/trượt: giữ NEEDS_MEASUREMENT, STOP/ngắt VM nếu đang chạy. |
| 4. Tốc độ/PI | Kê bánh. Dùng `SWEEP F`, `SWEEP R` để lập bảng PWM–CPS; sau đó `PI 0.010 0` (giá trị thử trong RAM, chưa là gain chuẩn), xác nhận `PI_SET`; `SPEED L 300 500`, tương tự R. Ghi target, ACT, RAW, MIN/MAX, CUT/ZERO, PWM và lỗi tích lũy. | Không fault; tốc độ ổn định theo dung sai đặt trước, không bão hòa kéo dài, dừng đúng hạn. | `PI_NOT_SET`, rung, overspeed, mất xung, timeout: STOP/ngắt VM. Chưa dùng dữ liệu kê bánh làm tốc độ chạy sàn. |
| 5. Phanh | Chỉ sau bài 1–4 và có vùng thử/chặn ngoài; đo tốc độ trước STOP, vị trí lúc STOP, vị trí dừng, thời gian, nguồn/độ bám. | Quãng dừng lặp lại dưới giới hạn theo bản đồ và khoảng trống thực tế. | Vượt vùng an toàn hoặc STOP không có tác dụng: cắt VM; giữ NEEDS_MEASUREMENT. |
| 6. Fault | Trên bàn thử, xác nhận log `STALL_*`, `ENC_DIR_*`, timeout/guard bằng tình huống có kiểm soát; không cố kẹt trục bằng tay. | Fault kéo STBY LOW, lệnh bị từ chối đến khi xử lý và reset phù hợp. | Motor tiếp tục khi fault, không log hoặc timeout không dừng: ngắt VM, cấm mở khóa. |

Lệnh `MOTOR`, `SPEED`, `SWEEP` chỉ chạy khi điều kiện `testAllowed()` và thời hạn của firmware cho phép; không suy từ build thành PASS thực tế. Bài 5–6 cần quy trình và ngưỡng cụ thể được thống nhất trước khi thử trên sàn.

### Bài phần cứng đầu tiên

Giữ pin VM tháo, cấp logic bằng nguồn ổn định. Dùng firmware `pid_calibration` **chỉ sau khi tự chọn nạp**; kiểm STBY ~0 V. Đánh dấu J12/J13, ghi mốc TIM2/TIM3, quay tay từng bánh tiến một vòng rồi lùi một vòng. Lưu log UART và video/mốc bánh. PASS chỉ khi từng timer tương ứng có xung, dấu đảo ở chiều ngược và đứng yên không tự đếm; thiếu bất kỳ điều kiện nào là FAIL/chưa xác nhận. Không dùng log cũ thay kết quả mới.

## Giao diện đề xuất cho mốc tích hợp sau

Đơn vị SI nội bộ và timestamp `uint32_t ms` dùng thống nhất. Chỉ thiết kế dữ liệu; chưa tích hợp A*, Pure Pursuit và PID.

| Kiểu | Trường tối thiểu |
| --- | --- |
| `RobotPose` | x/y m, yaw rad, timestamp, covariance/validity. |
| `WheelState` | tick raw/tích lũy, m/s, dấu, tuổi mẫu, fault từng bánh. |
| `SensorSnapshot` | bốn VL53: mm, status, age, valid; gyro Z, bias, age, valid. |
| `OccupancyGrid` | origin, resolution, width/height, cell occupancy, revision; giới hạn bộ nhớ tĩnh. |
| `PlannedPath` | danh sách waypoint có capacity cố định, count, grid revision, valid. |
| `WheelTargets` | tốc độ m/s trái/phải, giới hạn gia tốc, timestamp, hết hạn. |
| `SafetyStatus` | STBY permission, reason/fault latched, deadline điều khiển/cảm biến, reset policy. |

Luồng dự kiến: sensor/encoder → pose → grid/planner → path follower → wheel targets → PI → motor gate. Mỗi tầng phải được thử riêng; `SafetyStatus` có quyền phủ quyết ở đầu ra.

## Điều kiện chuyển Mốc 2/3

1. Build của `map_one_run` và baseline liên quan rõ PASS, Flash/RAM dưới giới hạn có dự phòng; pin/timer không xung đột trên PCB thật.
2. Bài encoder hai chiều, motor có thời hạn, PI, phanh, mất xung/sai chiều/stall và timeout có log mới đạt PASS; STBY LOW ở boot/reset/fault được đo thật.
3. Mọi thông số cần thiết cho điều khiển, pose, footprint, VL53 và phanh chuyển sang VERIFIED bằng phép đo lặp lại; không dùng giá trị ước lượng.
4. Xác định pose đầu, giới hạn bản đồ, vật cản và ngân sách SRAM; kiểm thử A* độc lập trên máy và target trước khi kết nối follower.
5. Có công tắc/cách ngắt VM và vùng thử an toàn, rồi mới bàn mở khóa motor. Chưa điều kiện nào trong mục này được mặc nhiên PASS từ build.
