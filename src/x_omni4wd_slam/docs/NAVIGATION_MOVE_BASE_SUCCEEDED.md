# RF2O odometry và move_base SUCCEEDED

Package **x_omni4wd_slam** cung cấp **RF2O** (laser-based odometry), phát `/odom` và TF `odom` → `base_footprint`. Khi move_base dùng odometry từ RF2O (thay wheel encoder), pose có thể **nhiễu** hơn, dẫn tới move_base **ít khi** báo trạng thái SUCCEEDED (goal reached) nếu cấu hình mặc định.

## Khuyến nghị cấu hình (đã áp dụng trong diffbot_navigation / x_omni4wd_simulation)

- **TEB:** Nới `xy_goal_tolerance`, `yaw_goal_tolerance`; bật `free_goal_vel`; nới `trans_stopped_vel` / `theta_stopped_vel`.
- **move_base:** Giảm `planner_frequency` (vd. 2.5 Hz), tăng `controller_patience` và `oscillation_timeout`.

Chi tiết nguyên nhân và cách khắc phục: **diffbot_navigation/docs/MOVE_BASE_SUCCEEDED_X_OMNI4WD.md** và **diffbot_navigation/docs/TOM_TAT_BAI_HOC_X_OMNI4WD_GOAL.md** (trong workspace costmapex_unified_ws).
