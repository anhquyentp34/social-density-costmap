# fer_emotion

Gói ROS nhận diện biểu cảm khuôn mặt (FER) cho Track D — robot thật với ZED2i.

Pipeline:

```
/bodies  (ZED tracking ID = body.id)
/image_raw
  → emotion_detection_node
    → /person_emotions  name=person_{id}, emotion=Happy|Angry|Neutral
    → /image_emotion    overlay "ID:3 Angry"
  → skeleton_visualizer (/image_skeleton, RViz)  "ID:3 | Angry"
  → social_density_fusion_node_zed  gán emotion theo person_{id}
  → /social_state → EmotionLayer
```

Mỗi người được track bởi ZED ID. FER chỉ gán nhãn cảm xúc lên ID đó; fusion và costmap dùng cùng tên `person_<id>`.

## Yêu cầu

- ROS Noetic
- Python 3 + OpenCV (DNN)
- Package: `social_msgs`, `zed_sdk_bridge`, `cv_bridge`, `message_filters`
- Model ONNX (xem mục Tải model)

## Cấu trúc

```
fer_emotion/
├── launch/emotion_detection.launch   # Node ROS cho ZED
├── scripts/
│   ├── emotion_fer_backend.py        # FER+ ONNX, YuNet, logic Angry
│   ├── emotion_detection_node.py     # ROS node
│   ├── test_fer_webcam.py            # Test offline trên webcam
│   └── download_fer_model.sh         # Tải model
├── models/
│   ├── emotion-ferplus-8.onnx
│   └── face_detection_yunet_2023mar.onnx
└── config/emotion_fer_track_d.yaml   # Tham số mặc định Track D
```

## Tải model

Nếu chưa có file trong `models/`:

```bash
cd ~/omni4wd_ws/src/fer_emotion/scripts
./download_fer_model.sh
```

Hoặc sau khi build workspace:

```bash
rosrun fer_emotion download_fer_model.sh
```

## Test webcam (IdeaPad, không cần ROS)

Dùng để kiểm tra model và ngưỡng trước khi gắn ZED.

```bash
# Test nhanh 60 frame
python3 ~/omni4wd_ws/src/fer_emotion/scripts/test_fer_webcam.py \
  --device 0 --frames 60 --focus Angry

# Preview live (nhấn q để thoát)
~/omni4wd_ws/tracks/fer_emotion/run_test_webcam.sh \
  --show --focus Angry
```

Tham số hữu ích:

| Tham số | Mặc định | Ý nghĩa |
|---------|----------|---------|
| `--device` | 0 | `/dev/videoN` |
| `--focus` | Angry | `Angry` hoặc `balanced` |
| `--angry-min` | 0.12 | Ngưỡng nhóm Angry |
| `--group-margin` | 0.12 | Margin để đổi nhãn |
| `--frames` | 30 | Số frame (0 = chạy đến khi thoát) |
| `--show` | — | Hiện cửa sổ preview |

**Gợi ý đọc kết quả:** Neutral thường ổn định 80–97%. Happy khi cười rõ (H > 60%). Angry cần biểu cảm mạnh hoặc hạ `--angry-min` (vd. 0.08) khi debug.

**Webcam bị lock:** tắt process cũ trước khi chạy lại:

```bash
pkill -f test_fer_webcam
```

## Chạy với ZED (ROS)

Cần ZED đang publish `/image_raw` và `/bodies`:

```bash
source ~/omni4wd_ws/devel/setup.bash
roslaunch fer_emotion emotion_detection.launch
```

Override topic / ngưỡng:

```bash
roslaunch fer_emotion emotion_detection.launch \
  image_topic:=/zed2i/zed_node/rgb/image_rect_color \
  bodies_topic:=/bodies \
  angry_min_score:=0.10
```

Kiểm tra output:

```bash
rostopic echo /person_emotions
rostopic hz /person_emotions
```

## Tích hợp Track D (robot thật)

Script `run_real_track_d.sh` tự bật FER cho thí nghiệm **real_tn2** và **real_tn3**:

```bash
cd ~/omni4wd_ws/tracks/D-density-costmap
./run_real_track_d.sh real_tn2   # hoặc real_tn3
```

Launch điều hướng: `x_omni4wd_navigation/launch/D-density-costmap/x_omni4wd_navigation_real_track_d.launch`  
(arg `use_emotion_detection:=true` → include `fer_emotion/launch/emotion_detection.launch`)

## Tham số ROS (launch)

| Tham số | Mặc định | Ý nghĩa |
|---------|----------|---------|
| `focus_emotion` | Angry | Ưu tiên phát hiện giận (Track D) |
| `angry_min_score` | 0.12 | Điểm nhóm Angry tối thiểu |
| `angry_margin` | 0.04 | Angry phải vượt Happy ít nhất bấy nhiêu |
| `angry_neutral_slack` | 0.08 | Cho phép Angry gần bằng Neutral |
| `angry_raw_spike` | 0.04 | Spike anger/contempt/disgust đơn lẻ |
| `smooth_window` | 8 | Cửa sổ làm mượt theo person_id |
| `min_body_confidence` | 40.0 | Lọc skeleton ZED yếu |
| `min_fer_confidence` | 0.35 | Không publish nếu FER quá thấp |
| `group_margin` | 0.12 | Margin argmax nhóm emotion |

Chi tiết logic Angry: xem docstring trong `scripts/emotion_fer_backend.py`.

## Map emotion → social layer

| FER+ (raw) | Social label |
|------------|--------------|
| neutral, sadness | Neutral |
| happiness, surprise | Happy |
| anger, disgust, fear, contempt | Angry |

EmotionLayer dùng social label để đặt bán kính vùng an toàn (vd. Angry → 2.0 m).

## Xử lý sự cố

| Triệu chứng | Cách xử lý |
|-------------|------------|
| `FER+ ONNX not found` | Chạy `download_fer_model.sh` |
| `YuNet not found` | Cùng script trên |
| Webcam không mở | `ls /dev/video*`, `pkill -f test_fer_webcam` |
| `/person_emotions` rỗng | Kiểm tra `/bodies` + `/image_raw` sync, tăng `sync_slop` |
| Luôn Neutral trên ZED | Giảm `angry_min_score`, kiểm tra crop mặt từ keypoints |
| OpenCV warning DNN target | Bình thường trên OpenCV 4.x+, CPU vẫn chạy |

## Build

```bash
cd ~/omni4wd_ws
source /opt/ros/noetic/setup.bash
catkin_make --pkg fer_emotion
source devel/setup.bash
```

Nếu workspace báo duplicate `animated_marker_*`, vẫn có thể chạy script trực tiếp từ `src/fer_emotion/scripts/` (không cần build).
