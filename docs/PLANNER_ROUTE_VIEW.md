# Đọc waypoint planner không sao chép tuyến

`planner_route_view.h/.cpp` cung cấp `PlannerRoute_Sample`: nhận mission đang
FOLLOWING với planner OK và quãng đường dọc tuyến (cm); trả x/y (cm), đoạn tuyến
và tổng chiều dài. Điểm ô được đổi sang tâm ô 5 cm. Quá cuối tuyến trả điểm cuối,
không tự thêm điểm vượt vạch và không chứng nhận đã về đích.

Hàm đọc trực tiếp waypoints, không cấp phát hoặc lưu cache. Caller chạy ở main
context và không sửa mission trong lúc gọi. Kiểm tra toàn bộ tuyến, kể cả đoạn
sau điểm lấy mẫu: count ngoài 2..128, ô sai, điểm liên tiếp trùng nhau, khoảng
cách âm/NaN hoặc trạng thái replan/fault đều trả false và output bằng 0.
Caller phải dừng khi false, không được dùng điểm 0 như waypoint hợp lệ.

Đây là cầu nối hình học chuẩn bị cho bộ chọn cung hybrid. **Chưa thay ROUTE[]
và chưa gọi hàm từ loop**: cần ghép progress/pose, reset tiến độ khi đổi tuyến,
kiểm tra cung và cơ chế vượt vạch trước bước đó. Không đổi chân, PI hay hệ số xe.

Kiểm thử host PASS: đoạn thẳng, qua góc, cuối tuyến, trạng thái dừng, tuyến lỗi,
NaN/null và thay tuyến sau replan. Biên dịch tests/planner_route_view_test.cpp
cùng src/planner_route_view.cpp với C++11, include/ và Core/Inc của core đã ghim.

`platformio run -e hybrid_planner_check`: PASS; hàm được giữ thật trong ELF và
kiểm tra symbol tự động. Flash image span 63.428/64.512 B (còn 1.084 B), RAM
tĩnh 14.452 B; gồm linker reserve 15.992/20.480 B. Chưa đo stack thực.
Các số này mới hơn bảng build của bước liên kết trước.

Debug: xem giá trị trả về cùng PlannerRouteSample; false phải đi với output 0.
Breakpoint trong hàm sẽ chưa hit ở firmware hiện tại vì chưa gọi từ loop.
PASS ở đây là phần mềm/build, không phải chạy xe. Motor vẫn khóa; chưa upload.
