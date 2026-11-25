# Kiểm tra liên kết planner với firmware hybrid — 25/11/2026

Đây là bước kiểm tra tài nguyên trước khi nối planner vào vòng điều khiển.
Chọn `hybrid_2laps.cpp` làm nền thử vì đã có TIM2/TIM3, PI hai bánh,
MPU, bốn VL53 và kiểm tra cung chuyển động. Không gộp các main lớn.
`map_one_run` hiện có vẫn giữ nguyên; `default_envs` không đổi.

## Phạm vi và giới hạn

- Môi trường mới `hybrid_planner_check` biên dịch nguồn hybrid hiện có với
  `HYBRID_MOTOR_LOCK=1`: cả PWM bằng 0, STBY PB0 LOW, kể cả khi yêu cầu phanh.
  Mức chân trước khi `setup()` chạy vẫn phụ thuộc phần cứng/reset.
- Core planner C11 lấy bằng Git dependency ghim commit
  `b84415d0a9852af61aaa08e3de1e380fe99b98d5` từ `haiddt0330/xe_tranh_vat_can`.
  Manifest chỉ chọn các module toán, không lấy main/startup/HAL/driver của repo đó.
  Firmware nền vẫn là STM32duino C++; đây không phải bản chuyển toàn bộ sang C11.
- Linker buộc giữ adapter, planner và vùng nhớ mission/config/input/output static.
  `setup/loop` **chưa gọi planner**; ROUTE cũ vẫn chỉ phục vụ phép kiểm tra nền.
  Bản này không phải firmware tự hành hoàn chỉnh và không chứng minh tuyến mới chạy được.
- Không thay pinout, PI, thông số hiệu chuẩn; không upload, mở COM hay chạy motor.

## Kết quả build và kiểm thử

Lệnh: `platformio run -e hybrid_2laps -e map_one_run` và
`platformio run -e hybrid_planner_check`.

| Môi trường | Kết quả | Flash theo PlatformIO | RAM tĩnh theo PlatformIO |
|---|---|---:|---:|
| hybrid_2laps (build lại sau thay đổi) | PASS | 50.012 B | 2.628 B |
| map_one_run (không đổi nguồn) | PASS | 38.740 B | 3.044 B |
| hybrid_planner_check (forward guard) | PASS | 62.636 B | 14.452 B |

Kiểm tra ELF đầy đủ tính cả vector, bảng khởi tạo, ảnh nạp `.data` và khoảng
căn hàng: **62.956/64.512 B Flash**, còn **1.556 B**. RAM gồm vùng linker
dự phòng là **15.992/20.480 B**, còn **4.488 B** ngoài vùng đó. Storage của
mission và các cấu trúc đi kèm là **11.824 B**. Đây chưa phải đo stack lúc chạy.
Không được chỉ dùng số phần trăm ở dòng PlatformIO để kết luận còn đủ dung lượng.

`tools/pio_planner_check.py` gắn kiểm tra vào bước link. FAIL nếu thiếu symbol
planner/mission, mất storage, quá Flash hoặc còn dưới 4 KiB RAM ngoài vùng
linker đã dành. Thử kiểm tra ELF hybrid cũ bị từ chối đúng vì thiếu planner.

`tools/test_hybrid_motor_lock.py` biên dịch trực tiếp hai hàm motor lấy từ nguồn
với GPIO/PWM giả lập. PASS cả bản thường và bản khóa; bản khóa không có lần
ghi STBY HIGH/PWM khác 0 khi drive, stop thường hoặc stop phanh. Lệnh đã chạy:

```powershell
python tools/test_hybrid_motor_lock.py --cxx C:/Users/Admin/AppData/Local/Temp/codex_ziglang/ziglang/zig.exe cc
```

Có thể dùng `--cxx g++` trên máy có GCC host. Các hệ số trong harness chỉ là
dữ liệu test, không được đưa vào cấu hình xe. Test không thay thế đo STBY/PWM thật.
Build còn warning `boolean` deprecated từ thư viện VL53L0X, không có lỗi compile/link.

Core C11: test mới tái hiện lỗi bỏ sót đoạn hiện tại khi lookahead đã qua góc cua.
Sau sửa, follower/controller/mission tests PASS; ARM build và forced link PASS
(text 29.536, data 96, bss 13.816 B ở repo core, không phải số firmware hybrid).

Bản core mới còn chặn yêu cầu quay/lùi chưa có kiểm tra vùng quan sát: controller
chỉ cho cung tiến có cả hai bánh dương, lỗi xóa lệnh và quyền chuyển động. Đã test
361 góc ban đầu, NaN/overflow và mission dừng/phục hồi; 5 nhóm test liên quan PASS.
Toàn bộ giá trị test là giả lập, không thay hệ số cơ khí trên xe.

## Debug và bước tiếp

Tiếp tục dùng VSCode/PlatformIO để build. Khi được phép kiểm thử phần cứng,
ELF đúng môi trường nằm tại `C:/PIOWork/NhungCode/build/hybrid_planner_check/firmware.elf`.
Trong CubeIDE có thể xem symbol `one_run_link_storage`, nhưng nó chưa được
cập nhật vì planner chưa chạy; breakpoint `OneRunAdapter_Poll` chưa được hit
là đúng với phạm vi này. Không dùng các số 0 đó làm kết quả định vị hay PASS.

Không sửa thanh ghi trong bước này. Các ngoại vi giữ nguyên: TIM2 PA0/PA1 phải,
TIM3 PA6/PA7 trái, TIM4 PB8/PB9 PWM, PB0 STBY, I2C1 PB6/PB7 MPU,
I2C2 PB10/PB11 VL53, XSHUT PA2–PA5, UART PA9/PA10.

Trước khi cho planner điều khiển: chuyển hệ trục yaw hybrid (0 tại +Y, dương
quay phải) sang core (0 tại +X, dương quay trái); ghép timestamp/pose từng VL53;
đưa localization và kiểm tra cung vào đường lệnh; xác nhận hiệu chuẩn và đo
thời gian thực thi/stack. Flash còn 1.556 B nên phải kiểm tra lại mỗi lần ghép.
PR #6 startup/safety đã được đọc; mô hình raycast của bản đó còn coi biên sân
là tường, không được dùng nguyên trạng cho sân chỉ có vạch dưới sàn.
Hybrid hiện chỉ đọc cờ reset IWDG/WWDG, chưa khởi tạo watchdog. Hai vấn đề này
chưa được sửa trong bước liên kết và không được đánh dấu đã hoàn thành.

PASS ở đây chỉ là build, kiểm thử host, lưu đủ symbol và đáp ứng ngân sách tĩnh.
Hiệu chuẩn còn `NEEDS_MEASUREMENT` theo cấu hình tập trung khi chưa có số đo
được xác nhận. Phần cứng và xe tự hành một lượt: **CHƯA KIỂM THỬ ở bản này**.
