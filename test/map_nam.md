# Kiểm chứng firmware map_nam

## Vận hành hai lượt

| Thao tác | Phản hồi của xe |
|---|---|
| Bật nguồn khi xe đứng yên | PC13 nháy 4 lần trong lúc khởi tạo và đo bias gyro. Sau đó đèn tắt, hai động cơ dừng ở IDLE. |
| Nhấn và nhả SW1 ở START | PC13 nháy. Xe lấy 7 khung đo. Khi báo `CHAY BLUETOOTH`, người lái có thể bắt đầu lượt khảo sát. |
| Lái bằng HC-05 ở 38400 baud | `F/B` tiến/lùi, `L/R` xoay, `G/I` cua tiến, `H/J` cua lùi, `0/X` nhả, `U/D/M/Q` chọn tốc độ. Lệnh hướng hết hiệu lực sau 667 ms, ứng dụng cần lặp lệnh khi giữ phím. |
| Qua FINISH rồi gửi `S` | Xe dừng và báo `DA LUU BAN DO`. Giữ nguồn trong lúc nhấc xe đặt lại START. |
| Nhấn và nhả SW1 lần hai | PC13 sáng liên tục. Xe đo lại START, kiểm tra mức phân biệt giữa các vị trí và hướng khả dĩ rồi mới cấp lệnh chạy. AUTO tự thực hiện cả lượt. |
| Cả thân xe qua FINISH | Xe dừng, xóa trạng thái điều khiển hành trình và trở về IDLE, PC13 tắt. Lần nhấn sau bắt đầu bản đồ mới. |

SW1 nối PB1 xuống GND khi nhấn, dùng `INPUT_PULLUP` và chống dội 35 ms. PC13 sáng ở mức LOW. Bluetooth gửi một dòng trạng thái khi đổi trạng thái và mỗi giây. Gửi `?` để hỏi trạng thái ngay.

## Kết quả build cho STM32F103C8T6

Lệnh build là `pio run -e map_nam`. Mỗi hàng là kết quả liên kết tại thời điểm hoàn thành giai đoạn. P8 gồm cấu hình SW1 đã xác nhận và tối ưu lúc liên kết. RAM là dung lượng tĩnh do PlatformIO báo. Mức sử dụng stack cực đại cần đo trên board.

| Giai đoạn | Phần đã build | RAM (byte) | Flash (byte) |
|---|---|---:|---:|
| P1 | TIM2/TIM3, PWM, MPU6050, 4 VL53L0X, UART và khởi động | 2 192 | 35 152 |
| P2 | Odometry vi sai, kết hợp yaw và lịch sử pose | 2 420 | 40 920 |
| P3 | Lượt Bluetooth, node, cạnh, vùng đã quan sát và khóa map | 9 204 | 47 468 |
| P4 | Thu mẫu đứng yên, ray casting, tìm kiếm pose quanh START | 10 120 | 51 592 |
| P5 | Hiệu chỉnh khi chạy, vật cản mới và tái định vị | 10 172 | 53 964 |
| P6 | Bám tuyến, điều khiển bánh và kiểm tra FINISH | 10 296 | 56 964 |
| P7 | Tìm đường vòng và nhập lại tuyến | 14 444 | 59 516 |
| P8 | Firmware tích hợp, chẩn đoán và kiểm thử hồi quy | 16 072 | 58 632 |
| Giới hạn của cấu hình | Blue Pill | 20 480 | 64 512 |

Toolchain đã dùng gồm PlatformIO ST STM32 20.0.0, Arduino STM32 3.0.0 và Pololu VL53L0X 1.3.1. Thư viện Pololu phát cảnh báo về kiểu `boolean` cũ. Tệp `.pio/build/map_nam/firmware.bin` dài 58 920 byte, gồm thêm 268 byte bảng vector và 20 byte bảng khởi tạo, kết thúc. Tệp ELF dành thêm 1 536 byte cho mức heap/stack tối thiểu ngoài 16 072 byte dữ liệu tĩnh.

## Kiểm thử đã chạy trên máy

Từ thư mục gốc của repo, có thể chạy lại bằng một trình biên dịch C++17.

```sh
clang++ -std=c++17 -O1 -Wall -Wextra -Wpedantic -fsanitize=undefined -fno-sanitize-recover=undefined test/map_nam_host.cpp -o /tmp/map_nam_test
/tmp/map_nam_test
```

| Bài thử | Kết quả quan sát |
|---|---|
| Encoder, gyro và điều khiển bánh | Qua kiểm tra tràn bộ đếm theo hai chiều, phát hiện mất xung, bias khi đứng yên và hủy lấy bias khi có chuyển động. Tích phân đúng các góc 30°, 45°, 90°, 180° và đoạn thẳng 200 mm với dữ liệu tổng hợp. |
| Học đường và nhấc xe | Nút dội và giữ nút sinh một sự kiện. START có nhiều khung đo. Encoder và khung cảm biến gửi sau `S` không làm đổi tuyến và lớp bản đồ đã học. |
| Định vị ban đầu | Đã thử đúng START, lệch tọa độ, lệch góc và lệch cả hai. Một bài thử điều khiển xe khảo sát để tự tạo bản đồ, rồi dùng chính bản đồ đó để định vị lần đặt lại lệch 24 mm theo X, 18 mm theo Y và 7°. |
| Từ chối và hiệu chỉnh định vị | Từ chối khi chỉ thấy một mặt phẳng hoặc chỉ có hai tia hợp lệ. Khi đủ mốc, hiệu chỉnh giảm sai lệch và nằm trong giới hạn 3,5 mm và 0,7° mỗi lần. Bất đồng kéo dài giữa encoder và gyro đưa xe về tái định vị. |
| Điều hướng và FINISH | Trên hành lang giả lập đã khảo sát, xe hoàn thành tuyến trống và bốn lần đặt thêm một khối 100 × 100 mm. Mỗi trường hợp có vật cản đều kiểm tra va chạm thân xe, tránh, nhập lại tuyến và vượt FINISH trước khi về IDLE. |
| Đường kín và lỗi | Xe dừng khi mất dữ liệu, khi không đủ không gian đã biết để lùi hoặc xoay, và khi một vật cản bịt kín hành lang. |

Bốn vị trí trong bài thử là tọa độ giả lập. Tọa độ thực địa T14–T17 chưa có trong dữ liệu dự án. Firmware dùng cùng một bộ tìm đường cho cả bốn trường hợp. Các bài này chưa đo được độ trượt bánh, độ trôi gyro, quãng phanh hay độ phản xạ của vật cản trên xe thật.

## Thông số cần đối chiếu trên xe

- TIM3 đếm bánh trái và TIM2 đếm bánh phải. PWM dùng PB9 cho bánh trái, PB8 cho bánh phải. Bốn địa chỉ J8–J11 là 0x30–0x33. MPU6050 dùng PB6/PB7, bốn VL53L0X dùng PB10/PB11.
- Giá trị khởi đầu 4,5 xung/mm mỗi bánh và khoảng cách tâm bánh 90 mm lấy từ `hybrid_2laps.cpp`. Có thể đặt bốn tham số `MAP_NAM_CPR_L`, `MAP_NAM_CPR_R`, `MAP_NAM_DIAMETER_L_MM`, `MAP_NAM_DIAMETER_R_MM` cùng `MAP_NAM_TRACK_MM` trong build flags. CPR là số đếm quadrature x4 trên một vòng bánh. Kiểm tra chiều encoder và hệ số quãng đường bằng đoạn thẳng đã đo, rồi kiểm tra khoảng cách tâm bánh bằng góc quay đã đo độc lập.
- Mô hình thân xe dùng nửa chiều dài 68 mm và nửa chiều rộng 58 mm. Tọa độ tâm quang học dùng độ lệch dọc 45 mm, ngang ±25 mm theo bản hybrid. Góc tia J8–J11 lần lượt là +35,43°, +10°, −10°, −35,43°. Kiểm tra các kích thước này và bias khoảng cách theo từng cảm biến.
- Kích thước sân, ô cấm, hai cửa và FINISH kế thừa hình học của bản hybrid. Điều kiện FINISH dùng mép sau cùng của thân xe và cộng biên sai số pose. Cần so sánh vị trí dừng với vạch thật.
