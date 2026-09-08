# Track D — RViz configs (tách sim / real / POS)

| File | Dùng cho | Nội dung chính |
|------|----------|----------------|
| `track_d_sim.rviz` | Thí nghiệm **mô phỏng** Gazebo | Actor mesh, density, camera Gazebo |
| `track_d_real.rviz` | Robot thật **exp1–3** (live + FER) | People mesh `/social_state`, ZED, density; annotation ON |
| `track_d_real_pos.rviz` | Robot thật **exp4–6** (POS latch) | Cùng layout exp1–3; bật `emotion_zones`/`objects`; annotation OFF khi chạy |
| `track_d_annotation.rviz` | Annotation nhẹ `mode:=map` | Chỉ Map + InteractiveMarkers |
| `track_d_annotation_nav.rviz` | Annotation `mode:=nav` | Costmap Global/Local + density + marker kéo (annotation ON) |

Không dùng chung `x_omni4wd_simulation.rviz` / `x_omni4wd_navigation.rviz` cho Track D — các file gốc đó dành cho nav/bringup chung khác track.

`density_costmap.rviz` giữ lại như bản cũ (legacy); launch Track D mặc định trỏ `track_d_sim.rviz`.
