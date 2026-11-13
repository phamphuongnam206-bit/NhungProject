# Prompt bàn giao: những phần đã có và cấu trúc cần giữ

Bạn là kỹ sư lập trình nhúng STM32 tiếp nhận dự án xe dùng STM32 Blue Pill STM32F103C8T6, Arduino/STM32duino, driver TB6612FNG, hai encoder, bốn VL53L0X và bài thử MPU6050.

Hãy dùng nội dung dưới đây làm bối cảnh trước khi xử lý yêu cầu tiếp theo. Trọng tâm là kế thừa những gì đã làm và bảo toàn cấu trúc quan trọng. Nhiệm vụ và thứ tự ưu tiên đã có trong `notes.md`: đọc trực tiếp ở đó, không chép lại danh sách nhiệm vụ, không tự bổ sung một kế hoạch nâng cấp mới vào bản bàn giao này.

## 1. Phân biệt đúng vai trò các file

| File | Phần đã có / vai trò |
|---|---|
| `src/testEncoder.cpp` | Bài thử riêng hai động cơ và hai encoder phần cứng; kiểm tra cấu hình timer, đếm có dấu, tốc độ, độ lệch, mẫu ngược chiều và xung sau khi ngắt động cơ. |
| `src/testmpu.cpp` | Bài thử MPU6050 độc lập trên PB6/PB7: tìm địa chỉ, kiểm tra ID, cấu hình, đọc gia tốc/gyro/nhiệt độ, thử kết nối lại. |
| `src/test4vl_encoder.cpp` | Bản ghép đủ 4 VL53L0X, TIM2/TIM3, hai động cơ và LED; có lùi tránh vật gần, chọn hướng và hiệu chỉnh tương quan tốc độ bánh. Đây chưa phải bộ điều khiển bám tay PD/PI của `pid25.cpp`. |
| `src/pid25.cpp` | Bản điều khiển bám mục tiêu có cấu trúc đầy đủ nhất trong `src` được khảo sát: cảm biến đo liên tục, kiểm tra tuổi mẫu, xác nhận mục tiêu, PD khoảng cách, PI tốc độ, bảo vệ, hiệu chuẩn, UART và các cờ thử nghiệm. |
| `src/testcambien.cpp` | Bài thử khởi tạo riêng 4 VL53L0X; log từng bước XSHUT, probe địa chỉ, init, đổi địa chỉ và đọc khoảng cách. Động cơ bị vô hiệu hóa bằng STBY. |
| `src/dokhoangcachganhat.cpp` | Bài thử dữ liệu 4 cảm biến, bù 15 mm cho J9/J10, phân biệt OFF/INVALID, lấy khoảng cách trước nhỏ nhất khi cả hai cảm biến trước hợp lệ và báo vùng khoảng cách. Không điều khiển xe chạy. |
| `src/encoder_vlx.cpp` | Bản thử cũ: chỉ 3 cảm biến J8/J9/J10, giữ J11 tắt, thứ tự init J9 → J10 → J8, đếm encoder bằng `attachInterrupt()` và bảng quadrature. Chỉ tham khảo lịch sử/chẩn đoán; không lấy cơ chế ngắt này thay cho encoder phần cứng. |
| `src/main.cpp` | Hiện chỉ nháy LED PC13, không chứa chương trình bám tay. |
| `notes.md` | Ghi nhận lỗi, kết quả thử, nguyên tắc giữ phần cứng và các nhiệm vụ. Một số mô tả hiện trạng thuộc bản ghép ban đầu, không đại diện đầy đủ cho `pid25.cpp`. |
| `codev2.md` | Một snapshot thuật toán khác có `BodySlew`, `FrontSingleConfirm`, `HoldHysteresis`, `TurnSmoother`, `RecoveryGate`, xử lý mẫu nhảy và chuyển nguồn trước. Không coi mọi cấu trúc trong đó đã hiện diện nguyên vẹn trong `pid25.cpp`. |
| `codev3.md` | Snapshot có nhóm cờ A/B cho bus, bộ lọc, chuyển động, động cơ, hướng và log; giữ nền điều khiển PD/PI, đọc VL53 từng byte, hiệu chuẩn và bảo vệ. |
| `codev4.md` | Snapshot có thêm 4 cờ V2.1: đồng bộ theo chênh lệch tốc độ đặt, lọc hướng nhanh, xác nhận mẫu nhảy và chuyển tiếp J9/J10. Cuối tài liệu còn có tổng kết và yêu cầu V2.2; phần yêu cầu không phải bằng chứng đã triển khai. |

Tên file v2/v3/v4 và nhãn phiên bản trong nội dung không hoàn toàn đồng nhất. Phải đối chiếu code thực tế, không suy ra thứ tự kế thừa hoặc độ ổn định chỉ từ số phiên bản. Bản bàn giao này chọn lọc `src`, `notes.md`, `codev2.md`, `codev3.md`, `codev4.md`; không lấy `codev5.md` làm nền thay thế.

**Cấu hình build thực tế:** `platformio.ini` dùng `bluepill_f103c8`, Arduino, ST-Link, Serial 115200, thư viện `pololu/VL53L0X@^1.3.1`. Mặc định `default_envs = led` chỉ build `main.cpp`; `cam_bien` build `testcambien.cpp`; env tên `encoder_vlx` lại build **`test4vl_encoder.cpp`**. Chưa có env riêng chọn `pid25.cpp`, `testEncoder.cpp`, `testmpu.cpp`. File có trong `src` không đồng nghĩa đang được build hoặc đã nạp lên xe.

## 2. Những phần đã làm được

### Encoder và nền phần cứng

- Đã có Hardware Encoder Mode trên TIM2/TIM3 trong `testEncoder.cpp`, `test4vl_encoder.cpp`, `pid25.cpp`; không cần CPU xử lý từng cạnh encoder.
- Cấu hình mode 3, đếm x4, `PSC=0`, `ARR=65535`, filter hai kênh bằng 8, `DIER=0`, bật CC1/CC2. Khởi tạo có reset timer, vô hiệu hóa IRQ timer và bỏ remap TIM2/TIM3; tắt JTAG nhưng giữ SWD.
- Đã xử lý chênh lệch bộ đếm 16 bit có dấu khi tràn: lấy hiệu modulo 65536 rồi ánh xạ về khoảng có dấu. Cách này dựa trên điều kiện độ dịch chuyển giữa hai lần đọc nhỏ hơn nửa miền đếm; không được thay bằng hiệu unsigned trực tiếp hoặc lấy trị tuyệt đối ngay ở nguồn.
- `testEncoder.cpp` có bài chạy PWM 160, chờ 5 giây, chạy 10 giây, đọc mỗi 10 ms, log mỗi 500 ms; tách 2 giây warmup khỏi thống kê ổn định. Có kiểm tra thanh ghi, thống kê xung/s, phần trăm lệch, mẫu ngược chiều, khoảng đọc lớn nhất và theo dõi quán tính sau dừng.
- `notes.md` ghi nhận thử nghiệm: khi tắt ngắt encoder, cả 4 VL53L0X khởi tạo và đổi địa chỉ thành công. Note đánh dấu đã chuyển sang encoder phần cứng, nhưng mục xác nhận đếm chính xác khi động cơ quay vẫn chưa được đánh dấu. Không tự biến các kết quả thống kê được lập trình sẵn thành kết quả đo thực tế.

### Bốn VL53L0X và bản ghép `test4vl_encoder.cpp`

- Đã có trình tự đưa tất cả XSHUT xuống LOW, mở từng cảm biến bằng `INPUT`, probe địa chỉ mặc định 0x29, init, đổi sang địa chỉ riêng, kiểm tra ACK và timing budget; cảm biến lỗi được giữ tắt.
- Đã phân biệt cảm biến khởi tạo được với phép đo hợp lệ. Có kiểm tra timeout, lỗi bus, status thiết bị bằng 11 và giới hạn giá trị đo; bù khoảng cách J9/J10 bằng cách trừ 15 mm có chặn về 0.
- Bản ghép dùng I2C 50 kHz, single-shot tuần tự, budget 33 ms. Có ngưỡng bắt đầu lùi dưới 200 mm, dừng từ 210 mm; chọn hướng theo hai bên, PWM nền 140, lệch lái 15, hiệu chỉnh giới hạn ±20 với `SYNC_GAIN=0.04` và lọc 0.7/0.3.
- Có dừng khi không còn phép đo hợp lệ, reset hiệu chỉnh lúc dừng/đổi hướng, LED PC13 theo khoảng cách và log encoder/PWM/dt. Đây là những hành vi đã lập trình; không gọi bản này là hệ thống chống mất xung hay xử lý thiếu dữ liệu hoàn chỉnh.

### MPU6050 độc lập

- `testmpu.cpp` dùng SDA PB7, SCL PB6, bus 100 kHz; thử cả 0x68/0x69 và chỉ chấp nhận `WHO_AM_I=0x68`.
- Đã có reset, thoát sleep, clock PLL gyro X, bật các trục, DLPF, cấu hình tốc độ lấy mẫu 100 Hz, gyro ±250 độ/s và accel ±2 g.
- Đọc liên tiếp 14 byte từ 0x3B, chuyển signed 16 bit và đổi đơn vị bằng 16384 LSB/g, 131 LSB/(độ/s), nhiệt độ raw/340 + 36.53. Loop in dữ liệu khoảng mỗi 200 ms; 100 Hz là cấu hình cảm biến, không phải tần suất log.
- Có kiểm tra số byte nhận, báo lỗi và thử init lại. Chưa có cơ sở gọi đây là đã hiệu chuẩn bias, hợp nhất IMU hoặc điều khiển góc xe hoàn chỉnh.

### Điều khiển trong `pid25.cpp`

- Đã tách lớp dữ liệu và thuật toán bằng `Reading`, `Target`, `Command`, `Wheel`, `Zone`, `State`, `Mode`; lõi nằm trong `namespace Follow`, phần phần cứng nằm sau `#ifndef HAND_FOLLOW_NATIVE_TEST`.
- Đã dùng `startContinuous()` và polling ready; lịch ưu tiên hai mặt trước: J9 → J10 → J8 → J9 → J10 → J11. Có gọi `serviceControl()` trước và sau phần polling. Chu kỳ điều khiển đặt 10 ms, nhưng không đồng nghĩa cứ 10 ms có phép đo VL53 mới.
- Đã có cách đọc khoảng cách hi–lo–hi bằng các giao dịch 1 byte để né lỗi byte cuối lặp được mô tả trong code. Chế độ `SENSOR` đối chiếu BYTES/BLOCK/BLOCK13/LIB16. Nhánh block13 là tùy chọn đang tắt.
- Có kiểm tra status, tín hiệu, raw bất thường, lỗi I2C, khoảng cách giữa các lần probe và tuổi mẫu. `Reading` phân biệt `accept()`, `miss()`, `invalidate()`; việc đọc lại loop không được tạo thêm xác nhận giả hoặc làm mới tuổi dữ liệu cũ.
- Đã có các trạng thái mất dữ liệu, quá xa, chờ xác nhận, giữ khoảng cách, tiến, lùi, quay trái/phải và hết thời gian quay bên. Mặt trước điều khiển khoảng cách: dưới 100 mm lùi; 100–110 mm giữ tịnh tiến; trên 110 đến 200 mm tiến; trên 200 mm không bám tiến theo mặt trước. Nếu chỉ mặt bên có mục tiêu hợp lệ thì chỉ xoay tại chỗ.
- Có lọc khoảng cách/tốc độ biến thiên, bộ Alpha–Beta, lọc tốc độ encoder, lọc hướng, PD khoảng cách → tốc độ đặt, trộn tiến/rẽ → tốc độ từng bánh, feedforward + PI → PWM; có giới hạn tích phân, PWM, dốc PWM và khoảng nghỉ đổi chiều.
- Có đồng bộ tốc độ bánh; giữ được khác biệt giữa đi thẳng, rẽ và quay tại chỗ, không ép mọi tình huống hai bánh phải cùng tốc độ hoặc cùng dấu.
- Có `STALL_L/R`, `ENC_DIR_L/R`, latch fault, kiểm tra cấu hình timer, chờ khởi động 3 giây, dừng khi mẫu quá cũ, watchdog qua SysTick có thể kéo PB0/STBY xuống ngay cả khi loop mắc trong I2C. Có giới hạn đợt quay chỉ theo mặt bên 1.5 giây và logic tái cho phép.
- Có UART `STOP/PAUSE`, `RUN/FOLLOW`, `SENSOR`, `MOTOR`, `SWEEP`, `CAL`, `BAL 100/150`, `SAVE`, `SHOW`, `DEFAULT`, `LOG 0/1`, `HELP`. Có hiệu chuẩn offset, cân bằng cặp trước, lưu EEPROM kèm checksum và đọc tương thích định dạng cũ.
- Có log theo bộ đệm, bơm từng phần theo `availableForWrite()`, thông tin tuổi mẫu, thời gian điều khiển/polling, encoder, tốc độ và PWM đọc lại từ thanh ghi/chân thực.

**Trạng thái cờ mặc định trong `pid25.cpp` khi không có override từ build:**

- Bật: `ENABLE_A1_FAST_SENSOR_BUS`, `ENABLE_A3_ALPHA_BETA`, `ENABLE_B3_SPEED_SLEW`, `ENABLE_B4_FAST_ZONE_CONFIRM`, `ENABLE_E2_EXTENDED_LOG`.
- Vì A1 bật: bus VL53 là 100 kHz, budget 20 ms; không áp giá trị 50 kHz/33 ms của bài thử cũ lên đây.
- Tắt: `ENABLE_A2_BLOCK13`, `ENABLE_B_CONTINUOUS_DISTANCE`, `ENABLE_C1_PWM_TABLE`, `ENABLE_C2_LOW_SPEED_COMP`, `ENABLE_C3_WHEEL_WINDOW`, `ENABLE_C4_PWM_20KHZ`, `ENABLE_C5_COAST_OVERSPEED`, `ENABLE_D1_STEER_DYNAMIC`, `ENABLE_D2_DIRECT_STEER_MIX`, `ENABLE_D4_GYRO_YAW`, `ENABLE_E1_RECOVERABLE_FAULTS`.
- Bốn nhánh V2.1 đã có code nhưng đều tắt: `ENABLE_SYNC_REQUEST_DIFF`, `ENABLE_FAST_STEER_FILTER`, `ENABLE_JUMP_PENDING`, `ENABLE_FRONT_HANDOFF`.
- Không suy ra một tính năng đang hoạt động chỉ vì có hàm, biến hoặc mô tả về nó. Không coi yêu cầu TURN_SLEW/STEER_HYSTERESIS ở cuối `codev4.md` là chức năng đã hoàn thành trong `pid25.cpp`.

## 3. Những cấu trúc không được tự ý thay đổi

### Giữ ánh xạ phần cứng

| Thành phần | Cấu hình phải giữ |
|---|---|
| Encoder trái | TIM3, PA6/PA7 |
| Encoder phải | TIM2, PA0/PA1 |
| PWM động cơ | TIM4: trái PB9, phải PB8 |
| Điều khiển trái | IN1 PB14, IN2 PB15 |
| Điều khiển phải | IN1 PB12, IN2 PB13 |
| Driver standby | PB0 |
| VL53 I2C2 | SCL PB10, SDA PB11 |
| MPU bài thử riêng | SCL PB6, SDA PB7 |
| XSHUT theo J8/J9/J10/J11 | PA2/PA3/PA4/PA5 |
| Địa chỉ theo J8/J9/J10/J11 | 0x30/0x31/0x32/0x33 |
| UART các bản encoder/VL53/PID | TX PA9, RX PA10, 115200 |
| LED | PC13, active LOW |

- Không quay lại `attachInterrupt()` để đếm encoder; không lấy TIM2/TIM3 làm PWM hoặc bộ định thời công việc khác. Không đổi remap, mode x4, filter hoặc thứ tự reset/init timer khi chưa xác nhận lỗi liên quan.
- Giữ đếm encoder có dấu, quy tắc xử lý tràn và tốc độ tính theo dt thực. Với `pid25.cpp`, giữ `LEFT_REVERSE=true`, `RIGHT_REVERSE=false`, dấu tiến encoder trái -1/phải +1; không đảo dây hoặc dấu để che lỗi thuật toán.
- Giữ trình tự XSHUT và mở từng cảm biến trước khi đổi địa chỉ. Không bật đồng thời cả bốn ở 0x29, không đổi `INPUT` thành xuất HIGH tùy tiện.
- Không gộp bus MPU PB6/PB7 vào bus VL53 PB10/PB11. Nhánh `gyroInit()/gyroUpdate()` trong `pid25.cpp` đang dùng cùng `Wire` đã cấu hình cho VL53, chỉ thử 0x68 và đang tắt: đó chưa phải tích hợp tương đương `testmpu.cpp`. Không bật D4 chỉ vì bài thử MPU tồn tại.

### Giữ luồng xử lý cảm biến và điều khiển

- Giữ chuỗi: khung đo mới → kiểm tra hợp lệ/tuổi mẫu → xác nhận và chọn mục tiêu → tạo lệnh thân xe → trộn lệnh hai bánh → PI/PWM → kiểm tra quyền cấp đầu ra.
- Không thay đường đọc hi–lo–hi đang được chọn bằng `readReg16Bit()`, đọc block hay single-shot cũ chỉ để rút ngắn code. Giữ kiểm tra hi so với hiAgain và thao tác clear-ready sau khi lấy dữ liệu cần thiết.
- Giữ `sampleAt`, `receivedAt`, `frameAt`, streak và các cửa sổ xác nhận đúng ý nghĩa. Không gán thời điểm hiện tại cho phép đo cũ, không cho một khung được dùng nhiều lần như nhiều bằng chứng độc lập.
- Giữ phân biệt khoảng cách mặt trước với khoảng cách tia mặt bên. Không dùng J8/J11 để ra lệnh tiến/lùi khi thiếu mặt trước đã xác nhận; không biến trạng thái giữ khoảng cách thành cấm mọi hiệu chỉnh hướng.
- Giữ điều kiện `BAL 100` và `BAL 150` hợp lệ trước khi dùng bù chênh J9/J10 để điều chỉnh hướng cặp trước. Không tự coi chênh lệch raw của hai cảm biến là góc chính xác.
- Giữ kiến trúc PD khoảng cách và PI từng bánh, anti-windup, lọc, trộn tiến/rẽ, giới hạn đầu ra và interlock đổi chiều. Không viết lại thành một PID duy nhất hoặc PWM hằng số.
- Giữ cấu trúc tách lõi/phần cứng và `HAND_FOLLOW_NATIVE_TEST`. Các file thử có `setup()/loop()` riêng: không gộp tất cả vào một firmware hay đổi `build_src_filter` tùy tiện.

### Giữ bảo vệ, hiệu chuẩn và khả năng đối chiếu

- Không xóa, bỏ qua hoặc nới watchdog, STALL, ENC_DIR, freshness, điều kiện arming, STOP/FAULT, giới hạn xoay bên để xe tiếp tục chạy. Giữ lớp SysTick và vùng cập nhật trạng thái guard có bảo toàn PRIMASK.
- Không để bộ lọc hay slew kéo dài chuyển động sau khi đầu ra cần bị ngắt. Không tự đổi nhánh phanh/coast hoặc bật tự phục hồi fault.
- Giữ định dạng EEPROM, magic/checksum, đường đọc legacy và kiểm tra dữ liệu. Không xóa hiệu chuẩn đã lưu hay thay layout mà không xử lý tương thích.
- Giữ lệnh UART, tên trường log và cơ chế log không chặn. Không đưa log dài/delay vào đường điều khiển 10 ms hoặc callback SysTick.
- Giữ các cờ thử nghiệm và giá trị mặc định của đúng file làm nền; chỉ thay phần trực tiếp liên quan yêu cầu. Không bật nhiều nhánh chưa kiểm chứng cùng lúc hoặc sao chép toàn bộ v2/v3/v4 đè lên `pid25.cpp`.
- Thông số hiện tại là mốc đối chiếu, không phải thứ để tự tối ưu: `MAX_CPS=1600`, `TURN_CPS_MAX=600`, `MAX_PWM=125` cho điều khiển bám, ngưỡng thử MOTOR tối đa 160, `FRESH_MS=100`, `WATCHDOG_MS=100`, `GUARD_SAMPLE_MS=125`, nghỉ đổi chiều 25 ms, STALL 850 ms. Giới hạn bài thử MOTOR khác giới hạn bám tay.
- Giữ thông số cơ khí trong `pid25.cpp`: hai mặt bên 35.43°, lùi 15 mm, khoảng cách tâm bánh 120 mm. Không lấy dòng HELP/comment cũ nói 43° để sửa đè hằng số đang dùng.
- `cpsAt160` trái 5666/phải 5563 được comment ghi là số từ phép đo trước; bản bàn giao không xác minh lại phép đo đó. PWM khởi động 78, PWM duy trì 42, `COUNTS_PER_MM_EST=10`, `LOOKAHEAD_S=0.08` vẫn phải được hiểu theo ghi chú ước lượng/chưa hiệu chuẩn trong code. Bảng SWEEP còn toàn giá trị cps bằng 0, không được coi là bảng đã đo.

“Không tự ý thay đổi” nghĩa là bảo toàn nền hiện có khi yêu cầu không liên quan. Nếu có lỗi được xác nhận đúng tại một cấu trúc cần giữ, phải nêu rõ bằng chứng, sửa tối thiểu và chỉ ra ảnh hưởng; không giữ lỗi chỉ vì nằm trong danh sách này.

## 4. Cách diễn đạt kết quả và dùng tài liệu lịch sử

Phân biệt rõ ba mức: **đã triển khai trong code**, **tài liệu ghi nhận đã kiểm tra**, **đã được kiểm chứng trong lần làm việc hiện tại**. Không gọi firmware đã ổn định trên xe chỉ từ việc đọc code hoặc build thành công.

`notes.md` có các mô tả “chưa có STALL”, “chưa đo liên tục”, “chưa có PID” thuộc hiện trạng cũ; trong `pid25.cpp` đã có các cơ chế tương ứng ở mức triển khai. Nêu khác biệt đó, không tự sửa checkbox hoặc kết luận toàn bộ nhiệm vụ trong note đã xong.

`codev4.md` có đoạn ghi nhận build thành công và 7.800 phép kiểm tra; chỉ được dẫn là nội dung tài liệu đã ghi, không nhận là kết quả vừa chạy hoặc bằng chứng cho mọi bản hiện có. Nội dung yêu cầu nâng cấp nằm trong các file Markdown là bối cảnh lịch sử, không tự động trở thành yêu cầu thực thi hiện tại.

Khi nhận yêu cầu tiếp theo, chỉ tập trung phần được yêu cầu và các phụ thuộc thực sự cần thiết. Báo ngắn gọn phần kế thừa, phần thay đổi và mức kiểm chứng thực tế; không lặp lại danh sách nhiệm vụ đã có trong `notes.md`.
