# Chẩn đoán thời gian khi cảm biến chạy chung

Env one_run_safe giữ STBY LOW. Log được chụp vào bộ đệm tĩnh 768 byte,
gửi tối đa 32 byte mỗi vòng khi Serial.availableForWrite() còn chỗ.
Lệnh UART được xử lý sau khi dòng log đang gửi đã hết để tránh xen dòng.

- LOOP_MAX_US: khoảng thời gian lớn nhất giữa hai lần vào loop, gồm thời gian đọc và xử lý log.
- GYRO_GAP_MAX_MS: khoảng cách lớn nhất giữa hai mẫu gyro nhận thành công.
- VL_MAX_US: thời gian lớn nhất của một lần pollSensor, không phải chu kỳ đo nội bộ VL53.
- LOG_SKIPPED: số yêu cầu log bị bỏ vì dòng trước chưa gửi xong.
- LOG_OVERFLOW: số byte không vừa bộ đệm; phải bằng 0.

Các cực đại tính từ boot, không xóa bởi Z hoặc Y. Số trong dòng log là
ảnh chụp lúc tạo dòng, không phải thời điểm máy tính nhận byte cuối.

Build one_run_safe và xe_auto_test đều PASS trước khi bàn giao. Chưa chạy
unit test hoặc đo trên PCB. RAM tĩnh bản chẩn đoán 3036/20480 byte,
Flash 38452/64512 byte; chưa đo đỉnh stack.

Phép thử: tháo pin motor, nạp đúng one_run_safe, UART 38400 baud, để xe yên
30 giây. Ghi log với cả MPU và bốn VL53 nối đủ. PASS bước thời gian nếu không
có lỗi cảm biến, LOG_OVERFLOW=0, gyro không mất mẫu trên 50 ms và YAW_VALID
không mất sau khi gửi Y. Đây chỉ là kiểm tra ngưỡng chẩn đoán, không chứng nhận
đủ thời gian cho điều khiển hoặc chứng minh đã thu đủ mọi mẫu 100 Hz.
FAIL nếu mất mẫu, stale kéo dài hoặc tràn log. Chưa có kết quả phần cứng.