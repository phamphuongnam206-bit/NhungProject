# Snapshot planner: hệ trục và tuổi dữ liệu

PlannerSnapshot_Build nhận snapshot nhất quán do caller lấy trong main context.
Hàm đổi yaw hybrid (0 ở +Y, dương quay phải) sang core (0 ở +X, dương quay trái).
Đây không phải phép tìm góc toàn cục: pose_verified chỉ được cấp sau khi định vị
và cấu hình đã xác nhận, không được lấy gyro zero hay MAP_START để tự đánh dấu.

Timestamp pose lấy mẫu cũ hơn giữa encoder/gyro; timestamp sensor lấy mẫu cũ
nhất trong bốn VL53. Không thay timestamp thật bằng thời gian poll. Giới hạn tuổi
và độ lệch thời gian phải do caller cấp. Quá hạn, thiếu kênh, giá trị không hữu
hạn, thời gian tương lai, dt ngoài (0,250 ms] hoặc frame lệch quá giới hạn đều
trả false và xóa output. Caller phải dừng/vô hiệu runtime khi false.

`ranges_ready` chỉ bật khi caller cung cấp origin pose riêng cho cả bốn phép đo.
Core dùng các pose này để chiếu từng hit vào runtime map; không dùng pose mới
nhất để chiếu lại range cũ. Nếu thiếu origin hoặc có giá trị không hữu hạn,
snapshot bị từ chối và runtime phải dừng.

Hybrid không biết chính xác thời điểm exposure của VL53. Chỉ khi pose được lấy
mẫu liên tục và ổn định trong 100 ms trước lúc đọc, cả bốn origin mới được
chuyển qua snapshot vào `SensorMap_UpdateTimed`. Khi xe chuyển động, mapping
động vẫn bị khóa; xem `STATIONARY_VL53_MAPPING.md`. Bản đồ sân, pose xuất phát
và calibration thật vẫn chưa được xác nhận.
Tốc độ đã quy đổi và các cờ valid là trách nhiệm của caller/driver.

Host test PASS: chuyển trục, mẫu cũ nhất, skew, mất xác nhận pose, NaN/null,
timestamp tràn uint32 và timestamp tương lai. Build STM32 cùng planner có giữ
symbol PlannerSnapshot_Build. Không cấp phát heap hay thêm buffer static.
Giữ khóa motor, pin/PI không đổi, chưa upload hoặc xác nhận phần cứng PASS.

Debug: safety.pose_sample_ms, sensor_sample_ms, front_min_mm, pose.heading_rad;
so tuổi bằng phép trừ uint32. false phải đi với pose_valid/sensor_valid=0.
Không ép ranges_ready=1 khi chưa có pose tương ứng từng phép đo.
