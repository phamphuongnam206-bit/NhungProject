# Runtime planner khóa motor

Begin nhận map/config bên ngoài đã được caller xác nhận và chỉ khởi tạo khi
hai bánh đứng yên, đầu ra motor bị vô hiệu (stopped). Mỗi Begin vô hiệu tuyến
cũ trước; không dùng map trỏ vào storage của mission. Không tự lấy hệ số 45
counts/cm, track 9 cm hay pose MAP_START làm dữ liệu đã hiệu chuẩn.

Poll chuyển snapshot nhất quán đến OneRunAdapter; A* chỉ chạy ở trạng thái
planning/replan và stopped. Khi mất global_pose_verified hoặc snapshot null,
runtime xóa tuyến và cần Begin lại. Flag verified là hợp đồng của caller, không
phải phép tự đo hay chứng nhận định vị. Freshness và lỗi cấu hình được kiểm tra
tiếp bởi core; caller phải kiểm tra status/safety trước khi dùng tuyến.

**Chưa gọi Begin/Poll từ loop với cảm biến thật.** Đây là runtime đã link ARM và
test host; còn thiếu xây map sân, cấu hình đã xác nhận và snapshot encoder/MPU/
VL53 đồng bộ để nối vào scheduler. Hàm lấy mission chỉ phục vụ đọc hình học,
không cấp quyền chuyển động. STBY/PWM tiếp tục khóa và không upload.

Test thực dùng 12 module C core, không giả lập planner: Begin từ chối đầu vào
chưa xác nhận/chưa dừng; Poll chờ đứng yên, tìm tuyến thật, chuyển waypoint cho
route consumer; mất global pose xóa tuyến, cần Begin lại; pose quá hạn không
được lập tuyến. PASS bằng tools/test_planner_runtime.py --core <checkout-core>
--zig <zig.exe>. Snapshot/map/config trong test là giả lập, không phải số đo xe.

Build hybrid_planner_check PASS: Flash span 62.760/64.512 B; RAM tĩnh 14.432 B,
gồm linker reserve 15.968 B. Chưa đo stack thực. Debug theo dõi mission.state,
waypoint_count, output.preview.status/safety trong one_run_link_storage.
Không thay chân, thanh ghi hoặc PI trong bước này; không merge main.
