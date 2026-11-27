#!/bin/zsh
set -eu
print 'Đang chờ cổng USB–Serial của STM32. Nhấn Ctrl+C để thoát.'
while true; do
  ports=(/dev/cu.(usbserial*|wchusbserial*|SLAB_USBtoUART*|usbmodem*)(N))
  (( ${#ports} > 0 )) && break
  sleep 1
done
port=$ports[1]
if (( ${#ports} > 1 )); then
  print 'Chọn cổng nối với STM32:'
  select port in "${ports[@]}"; do
    [[ -n $port ]] && break
  done
fi
print "Đọc $port ở 38400 baud."
exec "$HOME/.platformio/penv/bin/pio" device monitor --port "$port" --baud 38400
