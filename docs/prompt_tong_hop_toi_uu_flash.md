# Prompt tiếp nhận và tối ưu Flash firmware xe tự hành

Hãy tiếp tục dự án trong workspace `D:\prj_Nhung\NhungProject_one_run`.

## Mục tiêu

Phân tích và tối ưu dung lượng firmware `hybrid_planner_check` cho STM32F103C8T6. Ưu tiên giảm Flash nhiều nhất có thể nhưng phải giữ nguyên:

- A*, mapping và dynamic replanning.
- Pure Pursuit/path follower.
- Hybrid local arc evaluation.
- Swept footprint và collision checking.
- Encoder, motor PI/control loop.
- MPU6050 và VL53L0X.
- Watchdog, timeout, safety guard và no-path handling.
- Motor lock trong firmware qualification.

Không upload firmware, không mở khóa motor và không thực hiện thao tác phần cứng.

## Những việc đã hoàn thành

1. Đã build và phân tích ELF/linker map bằng:
   - `arm-none-eabi-size`
   - `arm-none-eabi-nm -S --size-sort`
   - `arm-none-eabi-objdump`
   - linker map.

2. Đã xác định các vùng Flash lớn gồm:
   - STM32duino/HAL, Wire/I2C và VL53L0X.
   - `loop`, `choosePlan`, A*, mission runtime.
   - Path Follower, Path Smoothing và footprint validation.
   - libm, lượng giác và helper double.
   - Link probe/planner qualification.

3. Đã ghi nhận ảnh hưởng của LTO:
   - Nhiều hàm bị inline, clone hoặc hợp nhất.
   - Một số chi phí không thể phân bổ chính xác theo file nguồn.
   - Alias ABI như `__aeabi_dadd` và `__adddf3` chỉ được tính một lần.

4. Đã thêm chế độ debug gọn cho môi trường `hybrid_planner_check`:
   - Thêm `-DHYBRID_COMPACT_DEBUG=1` trong `platformio.ini`.
   - Rút gọn `printStatus()`, bỏ định dạng số thực và chuỗi telemetry dài.
   - Tắt Bluetooth telemetry trong bản compact để loại bỏ `serviceBluetooth()`, `Serial.begin()` và log debug không cần cho qualification.
   - Các log này vẫn còn trong cấu hình bình thường khi không bật macro.

5. Đã giữ lại các linker flag `-u` để firmware qualification vẫn kiểm tra đầy đủ các entry point planner:
   - `OneRunMission_Step`
   - `Planner_ReplanWithPolicy`
   - `AStar_FindPathWithPolicy`
   - `PathFollower_Compute`
   - `NavigationReplan_Evaluate`
   - `SensorMap_Update`
   - `PlannerSnapshot_Build`
   - `PlannerRuntime_Begin/Poll/Invalidate`
   - `OneRunAdapter_Poll`
   - `OneRunMission_Init`
   - `OneRunFirmwareLinkProbe`
   - `PlannerRoute_Sample`

## Kết quả đo trước và sau

Bản ban đầu:

- Flash image: 64,428 B.
- PlatformIO Flash: 64,108 B.
- RAM tĩnh: 14,508 B.
- Flash còn trống: 84 B.

Sau tối ưu hiện tại:

- Flash image: 60,436 B.
- PlatformIO Flash: 60,124 B.
- RAM tĩnh: 14,448 B.
- Flash còn trống trên giới hạn 64,512 B: 4,076 B.
- Đã giảm tổng cộng khoảng 3,992 B Flash.
- Build `hybrid_planner_check` thành công.

## Việc cần làm tiếp theo

- Kiểm tra diff và build sau mỗi thay đổi.
- Dùng ELF/map để đo chính xác Flash và RAM.
- Chỉ giữ thay đổi làm giảm dung lượng mà không làm mất thuật toán hoặc cơ chế safety.
- Có thể lập một cấu hình release riêng để giảm các symbol bị giữ bằng `-u`, nhưng phải báo rõ đây là firmware release và khác với bản qualification.
- Không tự ý xóa A*, mapping, replanning, Pure Pursuit, collision checking, watchdog hoặc motor lock.
- Báo cáo trước/sau bằng byte và phần trăm.

