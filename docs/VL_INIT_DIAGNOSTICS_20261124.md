# Chẩn đoán khởi tạo bốn VL53L0X khi motor khóa

Env `one_run_safe` giữ PB0/STBY LOW và báo `VL_INIT=FAIL VL_FAIL=Jx:STAGE`
nếu khởi tạo dãy VL53L0X thất bại. `Jx` là cảm biến đầu tiên gặp lỗi theo thứ tự
J8..J11, tương ứng XSHUT PA2..PA5 và địa chỉ 7-bit 0x30..0x33. Bus chung
I2C2 dùng PB10/PB11; không đổi pinout PCB.

`BOOT` nghĩa là `sensor.init()` không thành công sau khi thả XSHUT.
`CONFIG` nghĩa là đổi địa chỉ hoặc cấu hình timing/signal thất bại.
`START` nghĩa là lệnh bắt đầu đo liên tục thất bại. `VL_INIT=OK` chỉ cho biết
khởi tạo đủ bốn cảm biến; số mm và độ chính xác vẫn phải kiểm tra riêng.

Nếu thiếu J9, ví dụ, có thể thấy `VL_INIT=FAIL VL_FAIL=J9:BOOT`. Các cảm biến
J10/J11 sau điểm lỗi chưa được thử trong lần boot đó, nên không kết luận chúng
PASS hoặc FAIL. Bốn giá trị `J8..J11=-1/INIT_FAIL` là trạng thái cả dãy, không
phải kết quả chẩn đoán độc lập từng cảm biến.

PASS phần mềm: build `one_run_safe` và `xe_auto_test`, không đổi trạng thái
PB0/STBY hoặc pinout. PASS phần cứng chỉ sau khi nạp đúng env, tháo pin motor,
đo STBY gần 0 V và thấy `VL_INIT=OK` cùng số mm hợp lý cho từng cảm biến.
FAIL nếu có `VL_FAIL`, timeout kéo dài hoặc STBY không LOW. Chưa thử trên PCB.
