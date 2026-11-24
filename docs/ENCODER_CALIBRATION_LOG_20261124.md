# Mốc đo encoder bằng quay tay, motor vẫn khóa

Env `one_run_safe` in tick có dấu `TIM3_L` (bánh trái J13) và `TIM2_R`
(bánh phải J12) qua UART1 PA9/PA10 ở 38400 baud. Gửi `Z` khi hai bánh
đứng yên: firmware chụp hai thanh ghi `TIM3->CNT`/`TIM2->CNT`, đặt hai tổng
tick về 0 và in `ENC_ZERO`. Gửi `?` để in số ngay; firmware cũng in mỗi giây.
`GPIOB->BRR` tiếp tục kéo PB0/STBY LOW; lệnh này không cấp PWM cho motor.

## Cách lấy số đo thực

1. Tháo pin motor, chỉ cấp nguồn logic đúng cách và mở Serial Monitor 38400.
   Kiểm tra dòng `ONE_RUN_LOCKED` và STBY gần 0 V. Không dùng env mặc định.
2. Đánh dấu bánh và mốc cố định. Giữ hai bánh đứng yên, gửi `Z` và đợi
   `ENC_ZERO` cùng `TIM3_L=0 TIM2_R=0`.
3. Quay **chỉ một bánh** đúng số vòng đã đếm, theo một chiều, rồi dừng. Ghi
   tick, số vòng, chiều quay và quãng đường bánh lăn đo bằng thước. Lặp riêng
   cho bánh còn lại; gửi `Z` trước mỗi lượt. Đừng suy count/cm từ chu vi ước lượng.
4. Với quãng đường đo thực `d_cm > 0`, tính `count_per_cm = abs(tick) / d_cm`.
   Làm ít nhất ba lượt mỗi bánh để xem độ lặp lại. Chưa ghi kết quả vào
   `COUNTS_PER_MM_L/R` cho tới khi dữ liệu ổn định và chiều tiến được xác nhận.

PASS phần mềm: build hai env `one_run_safe`, `xe_auto_test`; lệnh `Z` đặt mốc
không làm STBY lên HIGH. PASS phần cứng: từng bánh chỉ làm bộ đếm tương ứng
đổi, đổi chiều làm dấu tick đảo, đứng yên không tự tăng và kết quả nhiều lượt
gần nhau. FAIL nếu đếm sai bánh, chỉ một kênh có xung, tick nhảy khi đứng yên,
hoặc STBY không LOW. Chưa có số đo PCB cho mốc này.
