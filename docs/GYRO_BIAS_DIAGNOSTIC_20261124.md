# Log gyro Z và bias tham khảo khi motor khóa

Env `one_run_safe` giữ PB0/STBY LOW, dùng bus MPU PB6/PB7 và UART1 PA9/PA10
ở 38400 baud. Nhánh chẩn đoán nhận WHO_AM_I `0x68` hoặc profile `0x70` đã
quan sát trong repo `xe_tranh_vat_can`; `0x70` không được gọi là MPU6050 chuẩn.
ID khác báo `GYRO=UNSUPPORTED_ID`, không ghi cấu hình và không tính bias.
Nhánh tự chạy `xe_auto_test` cũ vẫn giữ quy tắc ID của nó.

Sau ID hợp lệ, firmware ghi `PWR_MGMT_1(0x6B)=1` để wake, `SMPLRT_DIV(0x19)=9`,
`CONFIG(0x1A)=3`, `GYRO_CONFIG(0x1B)=0x10` (±1000°/s), bật data-ready qua
`INT_ENABLE(0x38)=1`, rồi đọc lại các thanh ghi cấu hình chính. Poll
`INT_STATUS(0x3A)` bit 0 trước khi đọc gyro Z từ `0x47..0x48`; không đếm lặp
một mẫu cũ. Chờ warm-up 2 giây, sau đó thu 500 mẫu mới. Tổng tối đa
`500 × 32767` nằm trong `int32_t`.

Log UART có `GYRO`, `GZ_RAW`, `BIAS_N`; khi đủ mẫu có thêm
`BIAS_CANDIDATE_RAW` và `GZ_DPS=(GZ_RAW-bias)/32.8`. Bias chỉ là **ứng viên**
để chẩn đoán, chưa ghi vào hằng điều khiển hay tính theta. Cửa sổ đo bị đặt
lại nếu bánh có tick trong 0,5 giây gần nhất, raw vượt ±328, biên độ cửa sổ
vượt 65 LSB, khoảng cách hai mẫu mới trên 50 ms hoặc lỗi I2C. Cửa sổ đạt
không chứng minh xe chắc chắn đứng yên: quay rất chậm vẫn có thể lọt.

PASS phần mềm: build `one_run_safe` và `xe_auto_test`, không thay pinout hoặc
lệnh motor. PASS phần cứng cần nạp đúng env với pin motor tháo, đo STBY gần
0 V, thấy `BIAS_N` lên 500 khi xe đứng yên, `GZ_RAW` đổi khi xoay MPU và
`GZ_DPS` gần 0 khi đặt lại yên. FAIL nếu `NO_ID`, `CONFIG`, `I2C`, `STALE`,
`SATURATED` hoặc `SAMPLE_GAP` kéo dài. Chưa thử mốc này trên PCB. Dấu gyro
theo hướng xe và độ trôi lâu dài vẫn `NEEDS_MEASUREMENT`.
