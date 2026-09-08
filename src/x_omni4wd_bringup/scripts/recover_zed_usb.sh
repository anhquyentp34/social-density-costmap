#!/usr/bin/env bash
set -euo pipefail
echo "=== Recover ZED (hub reset + fix HID) ==="
pkill -f zed_sdk_bridge_node 2>/dev/null || true
pkill -f zed_wrapper_node 2>/dev/null || true
rm -f /tmp/.zed_enum_lock
modprobe usbhid 2>/dev/null || true

reset_dev() {
  local name="$1"
  if [[ -d "/sys/bus/usb/devices/$name" ]]; then
    echo "Reset hub/device $name"
    echo 0 > "/sys/bus/usb/devices/$name/authorized" || true
    sleep 2
    echo 1 > "/sys/bus/usb/devices/$name/authorized" || true
    sleep 3
  fi
}

# Reset Genesys hubs (nơi ZED từng gắn: 1-1.1 / 2-1.1)
reset_dev "2-1"
reset_dev "1-1"

echo "Chờ ZED enumerate..."
for i in $(seq 1 15); do
  if lsusb | grep -q '2b03:'; then
    echo "ZED xuất hiện trên lsusb:"
    lsusb | grep '2b03:'
    break
  fi
  sleep 1
done

# Gọi script gốc nếu có
FIX="$(dirname "$0")/fix_zed2i_usb.sh"
if [[ ! -f "$FIX" ]]; then
  FIX="/home/quyenanh/omni4wd_ws/src/x_omni4wd_bringup/scripts/fix_zed2i_usb.sh"
fi
if lsusb | grep -q '2b03:'; then
  bash "$FIX" || true
else
  echo "ERROR: Sau reset hub vẫn không thấy ZED (2b03). Cần rút/cắm lại cáp USB3."
  # Thử usbreset hubs
  usbreset 05e3:0626 2>/dev/null || true
  usbreset 05e3:0610 2>/dev/null || true
  sleep 5
  lsusb | grep '2b03:' || true
fi

if command -v ZED_Explorer >/dev/null 2>&1; then
  timeout 25 ZED_Explorer -a || true
fi
lsusb | grep -E '2b03|Stereo' || echo 'KẾT QUẢ: vẫn chưa có ZED trên USB'
