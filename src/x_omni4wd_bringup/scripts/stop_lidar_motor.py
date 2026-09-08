#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Tắt motor LiDAR (RPLidar) — ưu tiên ROS service, fallback DTR serial.

Dùng khi dừng thí nghiệm để LiDAR không còn quay sau khi kill node.
"""
from __future__ import print_function

import argparse
import os
import subprocess
import sys
import time


def _rosservice_call(svc, timeout=1.5):
    rosservice = "/opt/ros/noetic/bin/rosservice"
    if not os.path.isfile(rosservice):
        rosservice = "rosservice"
    try:
        r = subprocess.run(
            ["timeout", str(timeout), rosservice, "call", svc],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        return r.returncode == 0
    except Exception:
        return False


def stop_via_ros():
    for svc in ("/stop_motor", "/rplidarNode/stop_motor"):
        if _rosservice_call(svc):
            print("stop_lidar_motor: ROS %s OK" % svc)
            return True
    return False


def stop_via_serial(port, baud):
    try:
        import serial
    except ImportError:
        print("stop_lidar_motor: thiếu pyserial — bỏ qua fallback serial", file=sys.stderr)
        return False
    if not os.path.exists(port):
        print("stop_lidar_motor: không có %s" % port, file=sys.stderr)
        return False
    try:
        s = serial.Serial(port=port, baudrate=baud, timeout=0.3, write_timeout=0.3)
        # RPLidar A1/A2: motor điều khiển qua DTR — False = tắt
        s.setDTR(False)
        s.setRTS(False)
        time.sleep(0.25)
        # Lệnh STOP scan (0xA5 0x25) — an toàn nếu firmware nhận
        try:
            s.write(bytes([0xA5, 0x25]))
            s.flush()
            time.sleep(0.1)
        except Exception:
            pass
        s.close()
        print("stop_lidar_motor: serial DTR OFF trên %s" % port)
        return True
    except Exception as e:
        print("stop_lidar_motor: serial lỗi: %s" % e, file=sys.stderr)
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=os.environ.get("LIDAR_PORT", "/dev/robot_lidar"))
    ap.add_argument("--baud", type=int, default=int(os.environ.get("LIDAR_BAUD", "115200")))
    ap.add_argument("--serial-only", action="store_true")
    args = ap.parse_args()

    ok = False
    if not args.serial_only:
        ok = stop_via_ros()
    if not ok:
        ok = stop_via_serial(args.port, args.baud)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
