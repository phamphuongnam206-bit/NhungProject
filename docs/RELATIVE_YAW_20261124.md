# Góc tương đối khi motor khóa

Trong `one_run_safe`, đợi `BIAS_N=500`, giữ xe yên rồi gửi `Y` qua UART
38400 baud. `YAW_ZERO` xác nhận đặt mốc; mẫu gyro mới kế tiếp chỉ đặt mốc
tốc độ, sau đó tích phân hình thang theo dt thực tế bằng mili giây.
`YAW_DEG` dùng dấu trục Z cảm biến, chưa phải hướng tuyệt đối trong map.

Lỗi I2C, mất mẫu trên 50 ms, dt bằng 0, chạm thang hoặc kết quả không hữu hạn
làm `YAW_VALID=0` và đóng băng góc. Dữ liệu phục hồi không tự phục hồi góc:
đặt xe tại một hướng đã biết rồi gửi `Y` để đặt lại mốc. Bias vẫn chỉ là ứng viên
chẩn đoán. Không có deadband để che độ trôi, không sử dụng góc cho motor.

Phép thử PCB: tháo pin motor, nạp đúng `one_run_safe`, giữ xe yên khi đo bias.
Gửi `Y`, đợi log `YAW_VALID=1`, xoay cả xe trên mặt bàn theo góc đo bằng thước
góc rồi ghi `YAW_DEG`. Thử hai chiều và ghi độ trôi khi đứng yên 30 giây.
PASS kết nối khi số thay đổi đúng thao tác và không báo lỗi; độ chính xác góc
và dấu theo xe còn NEEDS_MEASUREMENT. FAIL nếu góc không đổi khi quay, mất mẫu,
chạm thang hoặc sai góc rõ. Chưa có kết quả phần cứng cho mốc này.
