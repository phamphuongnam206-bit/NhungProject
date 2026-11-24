# Mốc 2A — chuẩn bị đo count/cm hai bánh

`tools/calibrate_wheels.py` là công cụ chạy trên máy tính. Nó chỉ tính hệ số từ
số tick và quãng đường **đo trên xe thật**, không điều khiển motor và không
ghi tham số vào firmware. Hai encoder TIM2/TIM3 và các lệnh UART đã có trong
`pid_calibration.cpp` được giữ nguyên.

## Dữ liệu cần có khi tiếp tục đo

Đo ít nhất ba lượt đi thẳng trên mặt đất, mỗi lượt có khoảng cách thực từ điểm
đầu đến điểm cuối và chênh lệch tick trái/phải của đúng lượt đó. Mỗi lượt phải
đi cùng một chiều, không quay xe hay quay bánh bằng tay giữa hai mốc. Nếu xe
trượt, đi lệch hoặc số đo dao động lớn, ghi lại và đo lại trước khi dùng để
điều khiển. Việc chạy xe chỉ thực hiện khi đã kiểm tra an toàn phần cứng.

Tạo file CSV cục bộ với đúng bốn cột sau (các dòng minh họa là **số giả định,
không phải dữ liệu của xe này**):

```csv
run,distance_cm,ticks_left,ticks_right
example_1,20,-400,420
example_2,30,-600,630
example_3,50,-1000,1050
```

Chạy `python tools/calibrate_wheels.py <duong_dan_csv>`. Công cụ lấy tổng trị
tuyệt đối tick chia tổng quãng đường cho từng bánh; dấu tick được giữ để phát
hiện lượt đo đảo chiều. Kết quả hiển thị sai lệch lớn nhất giữa từng lượt và
hệ số tổng. Không dùng riêng `counts/s` khi bánh kê cao để suy ra `count/cm`.

**Trạng thái hiện tại:** `counts_per_cm_left` và `counts_per_cm_right` =
`NEEDS_MEASUREMENT`. Đường kính bánh, khoảng cách hai bánh hiệu dụng, gyro
sign/bias, PWM khởi động, bảng PWM–tốc độ có tải, gia tốc và quãng đường phanh
vẫn `NEEDS_MEASUREMENT`. Không thay các hằng ước lượng trong source hiện có.

**PASS phần mềm:** kiểm tra nhập CSV, phép tính có trọng số theo quãng đường,
đổi chiều và dữ liệu thiếu/sai. **PASS hiệu chuẩn trên xe:** chỉ sau khi có log
ít nhất ba lượt hợp lệ và chủ xe xác nhận độ lặp cùng quãng đường; công cụ
không tự kết luận PASS phần cứng.

Chạy unit test bằng `python -m unittest discover -s tests -p test_calibrate_wheels.py`.
