# Chẩn đoán cấu hình MPU trong one_run_safe

Lần đọc UART đầu tiên sau nạp báo `GYRO=CONFIG`, dù WHO_AM_I=0x68.
Firmware mới in `CFG_STAGE`, `CFG_REG`, `CFG_EXPECT`, `CFG_ACTUAL`.
Log PCB ghi `CFG_STAGE=MISMATCH CFG_REG=0x1B CFG_EXPECT=0x10
CFG_ACTUAL=0x03` khi đọc burst ba byte bắt đầu từ 0x19; byte 0x03
trùng với giá trị CONFIG ở 0x1A.

Trong nhánh chẩn đoán khóa motor, đọc lại từng thanh ghi cấu hình bằng
giao dịch một byte. Sau nạp và verify, MPU đã thu đủ `BIAS_N=500`, bias
tham khảo 43.05 raw, `GYRO_GAP_MAX_MS=12`; bốn VL báo OK và log không tràn.
Gửi `Y` khi xe đứng yên nhận `YAW_ZERO`, `YAW_VALID=1`; sau 15 giây log
hiển thị góc khoảng 0.03 độ. Đây là một lần quan sát, chưa kiểm tra sai số
khi xoay hoặc độ trôi lâu dài. STBY chưa được đo trong lượt này; pin motor
đã tháo theo xác nhận của người dùng.

Build `one_run_safe` và `xe_auto_test` đều PASS. Nhánh `xe_auto_test` đang
chạy tự động giữ nguyên. Không dùng bias hay góc chẩn đoán làm hằng hiệu
chuẩn cho điều khiển.
