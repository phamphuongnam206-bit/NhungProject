# Tối ưu Flash riêng bản planner khóa motor

Giữ `build_src_flags = -flto` chỉ cho các file ứng dụng và thêm cờ link tương
ứng qua script của hybrid_planner_check. Framework, driver thư viện và core
dependency giữ cách compile thường. Không sửa thuật toán, pinout, calibration,
PI, default_envs hoặc giới hạn Flash/SRAM.

LTO toàn bộ thử nghiệm giảm Flash xuống 56.988 B nhưng phát sinh Wodr về enum
PinName trong framework, nên **không giữ** cấu hình này. Bản cuối giới hạn LTO
ở ứng dụng không có Wodr/type-mismatch. Hai loại cảnh báo này được nâng thành
lỗi link để tránh vô tình chấp nhận ở build sau. Warning boolean deprecated
của VL53L0X vẫn là warning thư viện cũ.

ELF trước/sau (tính đầy đủ vector, bảng khởi tạo, ảnh .data và padding):

| | Trước | Sau |
|---|---:|---:|
| Flash span | 63.884 B | 63.252 B |
| Còn trong 64.512 B | 628 B | 1.260 B |
| RAM tĩnh | 14.432 B | 14.432 B |
| RAM gồm linker reserve | 15.968 B | 15.968 B |

Tiết kiệm 632 B; vẫn cần kiểm tra ngân sách khi nối snapshot/scheduler. Không
coi mức này đã đủ cho mọi chức năng còn lại. Không giảm stack reserve để fit.
Kiểm tra ELF mở rộng yêu cầu giữ cả snapshot và ba entry point runtime, ngoài
planner/mission/storage đã có. Các hàm core cần review vẫn giữ symbol bằng -u.

Build ARM PASS; host motor-lock (cả 0/1), runtime dùng core thật và 5 nhóm test
follower/wheel/safety/controller/mission PASS. Host test xác nhận logic nguồn,
không chứng minh mã tối ưu ARM đã chạy đúng trên xe. Chưa upload, chưa đo stack
hoặc benchmark thời gian, chưa xác nhận phần cứng PASS. LTO có thể inline hàm
nên breakpoint/Watch cục bộ kém thuận tiện hơn; firmware thường không đổi.
