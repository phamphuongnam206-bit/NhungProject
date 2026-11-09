# Ghi chú dự án
## Nguyên tắc khi sửa code

- Không thay đổi lại Hardware Encoder Mode TIM2/TIM3 nếu không có lỗi được xác nhận.
- Giữ TIM4 cho PWM động cơ.
- Giữ cấu hình I2C2 trên PB10/PB11.
- Không quay lại sử dụng `attachInterrupt()` để đếm encoder.
- Không thay đổi nhiều phần cùng lúc.
- Mỗi lần sửa phải kiểm tra lại 4 cảm biến, 2 encoder và 2 động cơ.
- Ưu tiên giữ ổn định phần cứng trước khi nâng cấp thuật toán.
- Không đánh dấu hoàn thành khi chưa có kết quả thử nghiệm thực tế.

## Lỗi 1 (done)

- **Lỗi:** Xung đột xử lý ngắt encoder với giao tiếp I2C trên STM32 Blue Pill, khiến 4 cảm biến VL53L0X khởi tạo không ổn định.

**Nguyên nhân nghi ngờ:** Ngắt encoder quá dày, ISR xử lý chưa tối ưu hoặc nhiễu tín hiệu encoder.

### Kết quả thử nghiệm
- **Đã kiểm tra:** Tắt ngắt encoder → cả 4 cảm biến khởi tạo và đổi địa chỉ I2C thành công (0x30–0x33).

### Đã sửa
- [x] Thay `attachInterrupt()` bằng Hardware Encoder Mode.
- [x] TIM2 (PA0, PA1) dành cho encoder phải.
- [x] TIM3 (PA6, PA7) dành cho encoder trái.
- [x] TIM4 (PB8, PB9) dành cho PWM động cơ.
- [x] Loại bỏ việc CPU phải xử lý ngắt ở mỗi xung encoder.
- [ ] Xác nhận encoder đếm chính xác khi động cơ quay.

---

## Lỗi 2 (cần kiểm tra)

- **Lỗi tiềm ẩn:** Động cơ được cấp PWM nhưng encoder không ghi nhận xung.

**Nguyên nhân có thể:** Encoder chưa hoạt động đúng, cấu hình timer hoặc chân tín hiệu chưa phù hợp, động cơ bị kẹt hoặc bánh xe không quay.

### Hiện trạng code
- Động cơ có thể nhận PWM = 140 dù ENC_L = 0 và ENC_R = 0.
- Chưa phát hiện tình trạng mất xung encoder.
- Chưa có cơ chế dừng khi động cơ được cấp PWM nhưng không có chuyển động.

### Việc cần làm
- [ ] Kiểm tra TIM2 và TIM3 có đếm xung khi quay từng bánh.
- [ ] Kiểm tra chiều tăng/giảm của encoder.
- [ ] Bổ sung phát hiện mất xung trong một khoảng thời gian.
- [ ] Dừng động cơ nếu phát hiện bất thường kéo dài.
- [ ] Tránh kết luận encoder lỗi chỉ dựa vào một mẫu bằng 0.

---

## Cải tiến 1 (chưa làm)

- **Vấn đề:** Đọc 4 cảm biến VL53L0X tuần tự khiến thời gian phản ứng của xe tương đối chậm.

**Nguyên nhân:** Sử dụng `readRangeSingleMillimeters()` với timing budget 33 ms cho từng cảm biến.

### Hiện trạng code
- 4 cảm biến được đọc lần lượt.
- Một chu kỳ đọc có thể mất khoảng 132 ms hoặc hơn.
- Khi cảm biến timeout, thời gian phản ứng có thể tăng thêm.

### Việc cần làm
- [ ] Nghiên cứu chuyển sang chế độ đo liên tục.
- [ ] Cho 4 cảm biến đo song song.
- [ ] Kiểm tra dữ liệu mới và timeout của từng cảm biến.
- [ ] Đảm bảo không ảnh hưởng đến phần khởi tạo I2C đã hoạt động.
- [ ] So sánh thời gian đọc trước và sau cải tiến.

---

## Cải tiến 2 (chưa làm)

- **Vấn đề:** Xử lý dữ liệu cảm biến INVALID chưa đủ an toàn.

### Hiện trạng code
- Xe dừng khi toàn bộ cảm biến mất dữ liệu hợp lệ.
- Nếu chỉ còn một cảm biến hợp lệ, xe vẫn có thể lùi.
- Có nguy cơ lựa chọn hướng dựa trên dữ liệu không đầy đủ.

### Việc cần làm
- [ ] Theo dõi thời gian từ lần đo hợp lệ cuối cùng.
- [ ] Phân biệt cảm biến OFF, INVALID và dữ liệu hợp lệ.
- [ ] Không sử dụng giá trị khoảng cách cũ như dữ liệu mới.
- [ ] Chỉ cho phép chuyển động khi có đủ dữ liệu cần thiết.
- [ ] Dừng an toàn khi mất dữ liệu ở hướng quan trọng.
- [ ]Kiểm tra bộ lọc nhiễu phần cứng TIM2/TIM3 khi động cơ quay, phát hiện xung giả hoặc mất xung do nhiễu điện.

---

## Cải tiến 3 (chưa làm)

- **Vấn đề:** Khoảng cách đo có thể dao động khiến xe bật/tắt động cơ liên tục.

### Hiện trạng code
- `START_MM = 200`
- `STOP_MM = 210`
- Khoảng cách giữa hai ngưỡng chỉ 10 mm.
- Chưa có bộ lọc nhiễu khoảng cách.

### Việc cần làm
- [ ] Thêm bộ lọc trung vị 3 mẫu.
- [ ] Loại bỏ giá trị đo bất thường.
- [ ] Điều chỉnh khoảng cách giữa START và STOP qua thực nghiệm.
- [ ] Chỉ chuyển trạng thái khi điều kiện ổn định qua nhiều mẫu.
- [ ] Kiểm tra xe không bị giật hoặc bật/tắt liên tục gần ngưỡng.

---

## Cải tiến 4 (chưa làm)

- **Vấn đề:** Thuật toán lựa chọn hướng lùi chưa linh hoạt khi vị trí vật cản thay đổi.

### Hiện trạng code
- Xe chọn hướng dựa trên khoảng cách hai bên.
- Hướng ưu tiên có thể được giữ trong suốt quá trình lùi.
- Khi hướng đã chọn bị cản, xe chuyển sang lùi thẳng.
- Chưa đánh giá lại toàn diện hướng di chuyển trong mọi tình huống.

### Việc cần làm
- [ ] Đánh giá lại khoảng cách hai bên khi đang lùi.
- [ ] Ưu tiên hướng có khoảng trống an toàn hơn.
- [ ] Tránh thay đổi hướng liên tục do nhiễu cảm biến.
- [ ] Xử lý riêng trường hợp cả hai bên đều có vật cản gần.
- [ ] Xử lý trường hợp một bên mất dữ liệu.
- [ ] Kiểm tra chiều quay thực tế của hai động cơ tương ứng với hướng mong muốn.

---

## Cải tiến 5 (chưa làm)

- **Vấn đề:** Điều khiển PWM và đồng bộ tốc độ hai bánh chưa tối ưu.

### Hiện trạng code
- PWM cơ bản = 140.
- PWM đánh lái = 15.
- Giới hạn hiệu chỉnh = 20.
- Đã có bộ lọc hiệu chỉnh tốc độ.
- Chưa điều khiển tăng/giảm PWM mượt.
- Chưa xử lý đầy đủ trường hợp encoder không ghi nhận xung.

### Việc cần làm
- [ ] Xác nhận encoder hoạt động chính xác trước khi chỉnh thuật toán.
- [ ] Bổ sung tăng/giảm PWM theo từng bước.
- [ ] Kiểm tra tốc độ thực tế của hai bánh.
- [ ] Tinh chỉnh hệ số `SYNC_GAIN`.
- [ ] Tránh hiệu chỉnh PWM sai khi encoder mất tín hiệu.
- [ ] Cân nhắc PID sau khi hệ thống encoder hoạt động ổn định.

---

## Cải tiến 6 (chưa làm)

- **Vấn đề:** Cảm biến bị lỗi khởi tạo không được tự động phục hồi.

### Hiện trạng code
- Nếu khởi tạo thất bại, cảm biến được đánh dấu OFF.
- Chương trình không tự khởi tạo lại cảm biến đó.
- Có thể phải reset STM32 để thử lại.

### Việc cần làm
- [ ] Theo dõi số lần lỗi liên tiếp của từng cảm biến.
- [ ] Phát hiện cảm biến mất phản hồi I2C.
- [ ] Chỉ thử khởi tạo lại khi xe ở trạng thái an toàn.
- [ ] Điều khiển XSHUT để reset riêng cảm biến gặp lỗi.
- [ ] Kiểm tra địa chỉ I2C sau khi khởi tạo lại.
- [ ] Không làm ảnh hưởng đến những cảm biến vẫn hoạt động.

---

## Cải tiến 7 (chưa làm)

- **Vấn đề:** Cần hoàn thiện cơ chế dừng động cơ và giám sát hệ thống.

### Hiện trạng code
- Khi dừng, chương trình đưa PWM về 0 và STBY về LOW.
- Chưa có hệ thống giám sát thời gian chu kỳ điều khiển.
- Chưa có trạng thái lỗi tổng hợp cho động cơ và cảm biến.

### Việc cần làm
- [ ] Kiểm tra quán tính của xe khi dừng.
- [ ] Cân nhắc chế độ phanh phù hợp với TB6612FNG.
- [ ] Giám sát thời gian chu kỳ điều khiển.
- [ ] Dừng động cơ nếu dữ liệu điều khiển quá cũ.
- [ ] Bổ sung trạng thái lỗi và thông báo qua Serial.
- [ ] Kiểm tra hành vi an toàn khi có lỗi cảm biến hoặc encoder.

---

## Thứ tự ưu tiên thực hiện

1. **Ưu tiên 1:** Kiểm tra encoder TIM2/TIM3 khi động cơ quay.
2. **Ưu tiên 2:** Bổ sung phát hiện mất xung encoder.
3. **Ưu tiên 3:** Tối ưu tốc độ đọc 4 VL53L0X.
4. **Ưu tiên 4:** Cải thiện xử lý INVALID.
5. **Ưu tiên 5:** Lọc nhiễu khoảng cách và chỉnh START/STOP.
6. **Ưu tiên 6:** Cải thiện thuật toán lựa chọn hướng.
7. **Ưu tiên 7:** Tối ưu PWM và đồng bộ tốc độ.
8. **Ưu tiên 8:** Tự phục hồi cảm biến và hoàn thiện cơ chế dừng.

