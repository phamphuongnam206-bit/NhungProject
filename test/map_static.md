# Kiểm chứng map_static

Firmware chính ở src/map_static.cpp, hình học ở src/map_config.h. Các phần encoder, gyro, PI bánh, kiểm tra vùng quét và driver STM32 dùng chung với map_nam qua src/navigation. Bài kiểm thử map_nam cũ vẫn chạy thành công.

## Cách nạp và vận hành

Mở src/map_static.cpp trong VS Code trên Mac, lưu file rồi nhấn Command+Shift+B, chọn tác vụ nạp STM32. Có thể chạy trực tiếp:

```sh
~/.platformio/penv/bin/pio run -e map_static -t upload
```

Chờ LED PC13 nháy bốn lần và trạng thái CHO NHAN SW1. Đặt tâm xe ở (300,120) mm, hướng 90 độ. Với LOCALIZE_AT_START=false, nhấn SW1 để chờ 2 giây rồi chạy AUTO từ tọa độ đặt xe. Đặt cờ thành true để bật lại định vị START. Nhấn SW1 trong AUTO sẽ khóa dừng khẩn cấp, reset để bắt đầu lại. Khi toàn thân qua Y=0 trong cổng đích, xe dừng và trở lại chờ.

Environment map_static_competition bật COMPETITION_2_LAPS=1. Lần nhấn đầu vào điều khiển Bluetooth, gửi S để kết thúc lượt tay, đặt lại START rồi nhấn SW1 để AUTO. Lệnh hướng có timeout 667 ms. Cần xác nhận ứng dụng lặp lệnh khi giữ nút.

UART 38400 baud gửi tên trạng thái mỗi giây. Gửi ? để lấy chẩn đoán. SENSOR_STATUS dùng H cho hit, N cho no-return, I cho invalid, T cho stale, F cho lỗi giao tiếp. MAX_CONTROL_US và MAX_TICK_GAP_US phục vụ đo thời gian trên phần cứng.

## Waypoint cuối

| Node | X mm | Y mm |
|---|---:|---:|
| 0 | 300 | 120 |
| 1 | 395 | 140 |
| 2 | 425 | 300 |
| 3 | 425 | 680 |
| 4 | 560 | 895 |
| 5 | 970 | 845 |
| 6 | 1170 | 715 |
| 7 | 1430 | 715 |
| 8 | 1600 | 560 |
| 9 | 1600 | -100 |

START_Y đổi từ 75 thành 120 để cải thiện quan sát landmark. Đã thêm một waypoint ở đoạn đầu và chỉnh các góc còn lại theo kiểm tra vùng quét. Các tọa độ vật T1–T17 giữ theo cấu hình người dùng cung cấp. Cần đo lại tọa độ trên sân.

## Kết quả trên máy

Mô hình dùng động học vi sai lý tưởng, encoder lượng tử hóa, gyro lý tưởng và ray casting chính xác. Điều khiển chạy bước 10 ms, cảm biến 40 ms. Khoảng hở kiểm tra cả phần thân và bù quãng dịch chuyển góc xe giữa hai mẫu.

| Trường hợp | Thời gian mô phỏng | Khoảng hở tối thiểu có bù bước |
|---|---:|---:|
| T1–T13 | 40,67 s | 12,7 mm |
| Thêm T14 | 41,77 s | 12,7 mm |
| Thêm T15 | 42,05 s | 12,7 mm |
| Thêm T16 | 42,38 s | 12,7 mm |
| Thêm T17 | 42,43 s | 12,7 mm |
| Thêm hộp tại (1600,185) | 43,41 s | 12,7 mm |

Các bài đạt kiểm tra toàn thân qua FINISH và chuyển động AUTO chỉ tiến hoặc xoay. Hộp trên tuyến kích hoạt đường vòng. Tốc độ đỉnh mô phỏng 150 mm/s, giảm theo góc hướng, khoảng hở, cảm biến và sai số pose. Đây là tốc độ đặt ban đầu, chưa đo trên xe. Một số mức tốc độ thấp hơn gây dừng bảo vệ trong mô phỏng, nên thay đổi tốc độ cần chạy lại toàn tuyến.

Lưới START gồm 175 tổ hợp, X/Y bước 25 mm trong ±50 mm, heading bước 5 độ trong ±15 độ. Có 30 tư thế va chạm/ngoài biên, 126 bị từ chối định vị và 19 được chấp nhận với sai số dưới 8 mm, 3 độ trong dữ liệu lý tưởng. Chưa xác nhận toàn bộ miền liên tục giữa các điểm. Điều kiện START yêu cầu bốn hit và sai số từng tia tối đa 3 mm nên có thể từ chối trên cảm biến thật có nhiễu. Cần đo bias và độ ổn định thực tế trước khi nới ngưỡng.

Bộ test cũ kiểm tra encoder tràn, sai dấu, stall, gyro, odometry và các tình huống lỗi. Bộ test mới kiểm tra hình học, ray-box, tín hiệu mất/quá hạn, SW1 dừng khẩn, hai cấu hình và sáu tuyến mô phỏng.

```sh
clang++ -std=c++17 -O1 -fsanitize=undefined test/map_nam_host.cpp -o /tmp/map_nam_test
/tmp/map_nam_test
clang++ -std=c++17 -O1 -fsanitize=undefined test/map_static_host.cpp -o /tmp/map_static_test
/tmp/map_static_test
for n in 0 1 2 3 4 5; do /tmp/map_static_test "$n"; done
```

## Tài nguyên và giới hạn thực tế

| Environment | RAM tĩnh | Flash |
|---|---:|---:|
| map_static | 10892 / 20480 byte | 58820 / 64512 byte |
| map_static_competition | 10896 / 20480 byte | 58840 / 64512 byte |
| map_nam | 15560 / 20480 byte | 57012 / 64512 byte |

Build thành công với LTO. Stack cực đại, độ trễ I2C, chu kỳ điều khiển và watchdog chưa được đo trên STM32. Cơ chế bảo vệ hiện có là SysTick ngắt STBY khi trễ control/range, chưa có independent hardware watchdog.

Hiệu chỉnh bắt buộc theo xe thực gồm xung/mm trái phải, khoảng cách tâm bánh, dấu và bias gyro, tọa độ/góc/bias bốn cảm biến, kích thước thân, ngưỡng PWM khởi động và tốc độ thấp ổn định, gia tốc phanh 320 mm/s² trong mô hình. Ước lượng sai số pose hiện là biên kinh nghiệm, chưa phải covariance thống kê theo X/Y độc lập.

NO_RETURN được tách khỏi lỗi I2C. No-target lặp ba lần chỉ cho quan sát hạn chế khi bản đồ cũng dự đoán vùng xa/trống, tốc độ giảm còn 45 mm/s. Điều này cần kiểm chứng với vật hấp thụ tia trên sân. INVALID, STALE và SENSOR_FAULT không cấp quyền đi vào hướng thiếu quan sát. Phần bỏ một cảm biến ngoài cần kiểm tra bằng thử rút dây thực tế.

Chưa kiểm thử trên xe thật, chưa nạp bản này trong lần triển khai này. Mô phỏng chưa bao gồm đầy đủ trượt bánh, nhiễu/latency cảm biến, động lực động cơ và mọi sai lệch START. Khoảng hở nhỏ nhất 12,7 mm yêu cầu hiệu chuẩn trước khi chạy sân.
