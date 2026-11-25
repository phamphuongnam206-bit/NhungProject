# Nối tiến độ planner vào bộ chọn cung (bản khóa motor)

`HYBRID_PLANNER_ROUTE=1` chỉ được biên dịch cùng `HYBRID_MOTOR_LOCK=1`.
Trong `choosePlan`, điểm nhìn trước nay lấy từ `PlannerRoute_Target`: chiếu
pose lên tuyến hiện tại, tính quãng đường tới hình chiếu rồi nội suy điểm phía
trước. Không giữ tiến độ tuyến cũ sau replan. Output có cross_track_cm và hai
segment riêng cho hình chiếu và lookahead. Không cấp phát thêm buffer tuyến.

Mission chưa FOLLOWING hoặc tuyến lỗi trả false, output 0 và Plan dừng.
Đã tách nhánh tính giảm tốc gần cuối tuyến, log tham chiếu, khởi tạo ROUTE và
ngoại lệ qua biên START/FINISH khỏi tuyến cố định. Bản planner hiện không cho
ngoại lệ qua biên và không tự xác nhận vượt vạch đích. Firmware hybrid thường
giữ logic tuyến cũ qua nhánh compile riêng.

**Chưa có scheduler khởi tạo mission và chạy A* từ pose/cấu hình đã xác nhận.**
Storage mặc định chưa có tuyến nên nhánh mới sẽ dừng. Đây là nối consumer vào
planner, chưa phải toàn bộ vòng tự hành. Chưa áp dụng ngưỡng lệch tuyến vì chưa
có cấu hình xác nhận; cross_track chỉ là dữ liệu chẩn đoán. Tuyến tự giao nhau
còn cần chính sách phân biệt tiến độ; hiện giữ đoạn gần nhất, ưu tiên đoạn trước
khi bằng khoảng cách. Không mở khóa motor bằng cách sửa cờ.

Host test PASS: pose lệch ngang, lookahead vượt góc trong khi xe chưa tới góc,
NaN và đổi tuyến tính lại tiến độ đúng. Build cả hybrid thường và bản planner
khóa motor PASS. Không upload, không đo phần cứng hay xác nhận PASS chạy xe.

Debug: xem plannerRouteTarget.progress_cm, cross_track_cm, current_segment,
lookahead.segment và one_run_link_storage.mission.state. Khi chưa có mission
hợp lệ, target bị xóa và choosePlan trả speed=0. Thanh ghi/pin/PI không đổi.
