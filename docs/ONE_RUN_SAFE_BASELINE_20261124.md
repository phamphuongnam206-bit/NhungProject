# Mốc baseline một lượt: build riêng, khóa motor

Env `one_run_safe` dùng lại `xe_auto_test.cpp` với `ONE_RUN_CALIBRATED=0`.
`setup()` cấu hình chân motor và kéo PB0/STBY LOW trước khi in trạng thái
`ONE_RUN_LOCKED NEEDS_MEASUREMENT`. `loop()` giữ motor vô hiệu hóa. Bản này
không khởi tạo cảm biến và không chạy điều khiển; đây là mốc an toàn để tích hợp
từng phần, không phải firmware tự hành.

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e one_run_safe
```

`xe_auto_test` cũ vẫn build và có thể tự chạy, vì vậy không nạp env đó trong
phép thử tĩnh. Không đổi `default_envs`; luôn chỉ định tên env. Không tự đổi
`ONE_RUN_CALIBRATED` sang 1: trước đó cần xác nhận count/mm hai bánh, track
hiệu dụng, dấu gyro/encoder, nguồn motor và cơ chế stop từ log thực tế. Việc
đổi cờ một mình cũng chưa đáp ứng A*, định vị START hay safety của Master Prompt.

PASS phần mềm khi cả `one_run_safe` và `xe_auto_test` build, mã khóa có nhánh
giữ STBY LOW. PASS PCB chỉ sau khi nạp đúng ELF của `one_run_safe` với pin motor
tháo, đo STBY gần 0 V và xác nhận hai bánh đứng yên. Chưa thực hiện phép thử PCB.
