# zed_sdk_bridge

Cầu ZED SDK → ROS (không qua `zed_wrapper`): RGB/depth, body tracking (`/bodies`), VIO/IMU tùy chọn, ZED object detection tùy chọn (mặc định tắt — Track A/D dùng YOLO).

Trước đây tên package là `zed_body_tracking`.

## Yêu cầu

- ZED SDK (`/usr/local/zed`)
- ZED 2 / ZED 2i
- ROS Noetic + CUDA

## Build

```bash
cd ~/omni4wd_ws
catkin_make --only-pkg-with-deps zed_sdk_bridge
source devel/setup.bash
```

## Node chính

```bash
rosrun zed_sdk_bridge zed_sdk_bridge_node
# hoặc qua bringup:
roslaunch x_omni4wd_bringup zed2i_body_tracking_robot.launch
```

Topic thường dùng: `/bodies`, `/zed2i/zed_node/rgb/...`, depth, `/odom_zed` (khi bật VIO).
