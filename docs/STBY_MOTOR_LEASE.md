# Giới hạn thời gian STBY khi vòng điều khiển bị treo

IWDG hiện reset sau khoảng 4 giây ở LSI danh định 40 kHz. Theo [RM0008 của
ST](https://www.st.com/resource/en/reference_manual/rm0008-stm32f103xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf),
tần số LSI có sai số, nên 4 giây không phải giới hạn chính xác. Kiểm tra
`CONTROL_LOOP_TIMEOUT_MS=150` trong `loop()` cũng không chạy được nếu `loop()`
đang kẹt trong I2C.

Nhánh `hybrid_2laps` chưa khóa motor nay dùng hook `osSystickHandler()` đã có
trong STM32 Arduino core. Sau mỗi lệnh bánh hợp lệ, lease chạy tối đa 80 ms.
Nếu hết hạn, ngắt SysTick ghi trực tiếp `GPIOB->BRR = 1` để kéo PB0/STBY LOW,
ghi nhớ `motorLeaseExpired=1`, và không cho lệnh motor muộn bật STBY lại.
Vòng chính khi hồi phục chốt lỗi `MOTOR_LEASE_EXPIRED`. Phanh bằng STBY HIGH
chỉ được giữ tối đa 120 ms trong một đợt, kể cả khi `stopMotors(true)` được gọi
lặp lại. Trạng thái LOCKED/FINISHED còn kéo STBY LOW trong vòng chính.

Các thanh ghi liên quan: `GPIOB->BRR` kéo PB0 xuống không cần Arduino API trong
ISR; `IWDG->KR/PR/RLR` vẫn giữ watchdog reset dự phòng. TIM2/TIM3 encoder,
TIM4 PWM, I2C và UART không bị đổi. Bản `hybrid_planner_check` vẫn biên dịch
với `HYBRID_MOTOR_LOCK=1`, không có đường bật STBY.

Test host biên dịch chính hai hàm motor và hook SysTick với GPIO giả lập: lease
79/80 ms, phanh 119/120 ms, phanh lặp, lệnh đến muộn, vòng `millis()` và
hai cấu hình MOTOR_LOCK=0/1 đều PASS. Hai môi trường PlatformIO build PASS;
ELF `hybrid_2laps` có symbol mạnh `osSystickHandler` và `SysTick_Handler`.

Giới hạn: SysTick không chạy nếu ngắt bị vô hiệu hóa hoặc CPU/clock dừng; khi
đó IWDG là đường reset cuối cùng. Cần kiểm tra điện trở kéo xuống STBY trên
PCB lúc STM32 reset. Chưa đo thời gian cắt PB0, quãng đường phanh hoặc stack
trên xe thật. Trước khi cho bánh chạy, thử riêng trên PCB với bánh kê lên và
nguồn motor có người giám sát: tạo một lỗi I2C có kiểm soát, quan sát PB0 bằng
máy đo/logic analyzer, xác nhận PB0 xuống LOW và không tự lên HIGH. FAIL nếu
PB0 vẫn HIGH sau 80 ms, lên HIGH lại không có reset, hoặc xe tiếp tục quay.
Không tự upload/chạy motor ở mốc phần mềm này.
