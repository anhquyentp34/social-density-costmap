#!/usr/bin/env bash
# Kiểm tra ZED 2i sẵn sàng trước khi mở SDK.
# Exit 0 = OK, 1 = chưa sẵn sàng.
#
# HID đôi khi có trên USB (2b03:f881) nhưng chưa xuất hiện trong
# /sys/bus/hid/devices — ZED SDK vẫn mở được qua video (f880).

check_video_usb() {
  lsusb -d 2b03:f880 >/dev/null 2>&1
}

check_hid_usb() {
  lsusb -d 2b03:f881 >/dev/null 2>&1
}

check_hid_sysfs() {
  for h in /sys/bus/hid/devices/*; do
    [[ -f "$h/uevent" ]] || continue
    if grep -qiE '2B03.*F881|ZED-2i HID' "$h/uevent" 2>/dev/null; then
      return 0
    fi
  done
  return 1
}

check_zed_sdk() {
  if ! command -v ZED_Explorer >/dev/null 2>&1; then
    return 1
  fi
  timeout 12 ZED_Explorer -a 2>&1 | grep -q "Camera Available"
}

VIDEO_OK=false
HID_USB_OK=false
HID_SYSFS_OK=false
SDK_OK=false

check_video_usb && VIDEO_OK=true
check_hid_usb && HID_USB_OK=true
check_hid_sysfs && HID_SYSFS_OK=true
check_zed_sdk && SDK_OK=true

# Đủ: video USB + (HID bind HOẶC HID trên USB) HOẶC SDK xác nhận camera
if [[ "$VIDEO_OK" == "true" ]] && { [[ "$HID_SYSFS_OK" == "true" ]] || [[ "$HID_USB_OK" == "true" ]]; }; then
  exit 0
fi

if [[ "$SDK_OK" == "true" ]]; then
  echo "[check_zed2i_ready] WARN: HID chưa bind sysfs nhưng ZED SDK thấy camera — tiếp tục." >&2
  exit 0
fi

echo "[check_zed2i_ready] ZED 2i chưa sẵn sàng:" >&2
if [[ "$VIDEO_OK" != "true" ]]; then
  echo "  - Thiếu video interface (2b03:f880). Kiểm tra cáp USB 3.0." >&2
fi
if [[ "$HID_USB_OK" != "true" ]]; then
  echo "  - Thiếu HID USB (2b03:f881)." >&2
elif [[ "$HID_SYSFS_OK" != "true" ]]; then
  echo "  - HID trên USB nhưng driver chưa bind. Thử:" >&2
  echo "    sudo bash $(rospack find x_omni4wd_bringup 2>/dev/null || echo src/x_omni4wd_bringup)/scripts/fix_zed2i_usb.sh" >&2
fi
if [[ "$SDK_OK" != "true" ]]; then
  echo "  - ZED_Explorer không thấy camera." >&2
fi
exit 1
