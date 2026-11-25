# Mốc phần mềm: chỉ lập bản đồ khi pose VL53 ổn định

VL53L0X chạy đo liên tục nhưng firmware chỉ biết thời điểm đọc thanh ghi kết
quả, không biết chính xác thời điểm chiếu và thu ánh sáng. Vì vậy không được
gắn pose lúc đọc vào một phép đo khi xe đang chuyển động.

`PlannerPoseWindow` lưu tối đa 16 pose từ vòng điều khiển. Một pose đọc được chỉ
đủ điều kiện khi có mẫu phủ ít nhất 100 ms trước lúc đọc, không có khoảng trống
giữa hai mẫu quá 25 ms, pose mới nhất không quá 25 ms, tọa độ từng trục không
lệch quá 0,5 cm và yaw không lệch quá 2 độ. Cảm biến cũng phải có lần đọc
thành công trước đó cách không quá 100 ms; lần đọc đầu sau khởi động/reset
không được dùng để lập bản đồ. Chỉ khi cả bốn range đạt điều kiện thì
`ranges_have_origins` mới bật. Nếu không, dữ liệu vẫn dùng cho an toàn khoảng
cách nhưng không được đưa vào `SensorMap_UpdateTimed`.

Đây là quy tắc bảo thủ cho phép đo lúc đứng yên, không phải timestamp exposure
thật. Vật cản ngẫu nhiên chưa thể được lập bản đồ đáng tin cậy khi xe chạy.
Muốn làm việc đó cần đồng bộ thời điểm đo VL53 với pose history và đánh giá
độ trễ I2C trên PCB. Tọa độ 17 vật cản cố định, pose xuất phát, kích thước vật
cản ngẫu nhiên và thông số xe vẫn phải được xác nhận trước khi mở khóa chạy.

Kiểm thử phần mềm:

```text
zig cc -std=c++11 -Wall -Wextra -Werror -Iinclude tests/planner_pose_window_test.cpp -o pose_window_test.exe
pose_window_test.exe
python tools/test_planner_runtime.py --core <core-checkout> --zig <zig.exe>
pio run -e hybrid_planner_check
```

PASS: cửa sổ pose ổn định được nhận; chuyển động, góc đổi, mất mẫu, mẫu cũ và
NaN bị từ chối; build planner qua giới hạn Flash/RAM. FAIL: một trường hợp sai
hoặc build vượt ngân sách. Watch trong debugger: `poseHistory.count`,
`frame[0..3].stableOrigin`, `source.ranges_have_origins` và
`snapshot.range_poses_ready`. Chưa có kiểm thử thời gian thực trên xe, chưa
upload và không có kết luận phần cứng PASS.
