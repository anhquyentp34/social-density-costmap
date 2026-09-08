#!/usr/bin/env bash
# Khôi phục ZED 2i khi ZED SDK báo "CAMERA NOT DETECTED" dù lsusb vẫn thấy thiết bị.
# Nguyên nhân thường gặp: interface HID (2b03:f881) mất driver usbhid sau USB reset
# do vòng respawn zed_sdk_bridge_node / zed_wrapper_node.
#
# Chạy: sudo bash fix_zed2i_usb.sh
# Sau đó khởi động lại launch hoặc để node tự retry.

set -euo pipefail

if [[ "${EUID:-$(id -u)}" -ne 0 ]]; then
  echo "Cần quyền root. Chạy: sudo bash $0"
  exit 1
fi

echo "=== ZED 2i USB Recovery ==="

# Dừng process đang giữ lock SDK (nếu có)
pkill -f zed_sdk_bridge_node 2>/dev/null || true
pkill -f zed_wrapper_node 2>/dev/null || true
rm -f /tmp/.zed_enum_lock

modprobe usbhid 2>/dev/null || true

bind_hid_iface() {
  local iface="$1"
  if [[ -z "$iface" ]]; then
    return 1
  fi
  if [[ -L "/sys/bus/usb/devices/${iface}/driver" ]]; then
    local drv
    drv="$(basename "$(readlink "/sys/bus/usb/devices/${iface}/driver")")"
    if [[ "$drv" == "usbhid" ]]; then
      echo "  HID $iface: đã có driver usbhid"
      return 0
    fi
  fi
  echo "  Bind usbhid -> $iface"
  echo "$iface" > /sys/bus/usb/drivers/usbhid/bind 2>/dev/null && return 0
  return 1
}

reset_usb_device() {
  local devpath="$1"
  if [[ -z "$devpath" || ! -d "/sys/bus/usb/devices/${devpath}" ]]; then
    return 1
  fi
  echo "  Reset USB device ${devpath}..."
  echo 0 > "/sys/bus/usb/devices/${devpath}/authorized"
  sleep 2
  echo 1 > "/sys/bus/usb/devices/${devpath}/authorized"
  sleep 3
}

# Tìm HID (2b03:f881) và Video (2b03:f880)
HID_DEVPATH=""
VIDEO_DEVPATH=""
for dev in /sys/bus/usb/devices/*; do
  [[ -f "$dev/idVendor" ]] || continue
  vid="$(cat "$dev/idVendor" 2>/dev/null || true)"
  pid="$(cat "$dev/idProduct" 2>/dev/null || true)"
  name="$(basename "$dev")"
  if [[ "$vid" == "2b03" && "$pid" == "f881" ]]; then
    HID_DEVPATH="$name"
  elif [[ "$vid" == "2b03" && "$pid" == "f880" ]]; then
    VIDEO_DEVPATH="$name"
  fi
done

echo "Video (f880): ${VIDEO_DEVPATH:-KHÔNG THẤY}"
echo "HID   (f881): ${HID_DEVPATH:-KHÔNG THẤY}"

if [[ -z "$HID_DEVPATH" ]]; then
  echo "ERROR: Không tìm thấy ZED HID (2b03:f881). Kiểm tra cáp USB 3.0 và cắm trực tiếp vào cổng xanh."
  exit 1
fi

if ! bind_hid_iface "${HID_DEVPATH}:1.0"; then
  reset_usb_device "$HID_DEVPATH"
  bind_hid_iface "${HID_DEVPATH}:1.0" || true
fi

if [[ -n "$VIDEO_DEVPATH" ]]; then
  # Đảm bảo video interface cũng được authorize
  echo 1 > "/sys/bus/usb/devices/${VIDEO_DEVPATH}/authorized" 2>/dev/null || true
fi

sleep 2

# Kiểm tra hidraw ZED
ZED_HID_OK=false
for h in /sys/bus/hid/devices/*; do
  [[ -f "$h/uevent" ]] || continue
  if grep -qi '2B03/F881' "$h/uevent" 2>/dev/null; then
    ZED_HID_OK=true
    echo "OK: ZED HID đã bind ($(basename "$h"))"
    break
  fi
done

if [[ "$ZED_HID_OK" != "true" ]]; then
  echo "WARN: HID vẫn chưa bind. Thử rút/cắm lại cáp ZED vào cổng USB 3.0 trực tiếp."
fi

# Kiểm tra SDK
if command -v ZED_Explorer >/dev/null 2>&1; then
  echo ""
  echo "=== ZED_Explorer -a ==="
  timeout 20 ZED_Explorer -a || true
else
  echo "ZED_Explorer không có trong PATH — bỏ qua kiểm tra SDK."
fi

echo ""
echo "Hoàn tất. Nếu vẫn 'No ZED detected': rút/cắm cáp USB ZED, chạy lại script này."
