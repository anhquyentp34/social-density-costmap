# Track D — D-density-costmap

**RQ-D:** Social density costmap — proxemic / social layers / density gate.

## Ngữ cảnh Cursor / IdeaPad (độc lập)

| File | Mục đích |
|------|----------|
| [`GHI_NHO_PHIEN_20260825_LATCH_EXP456_CUTOFF.md`](GHI_NHO_PHIEN_20260825_LATCH_EXP456_CUTOFF.md) | Ghi nhớ chốt: latch exp4–6, cutoff/TEB/keep_time, density gate |
| [`PROMPT_IDEAPAD_NGU_CANH_DOC_LAP.md`](PROMPT_IDEAPAD_NGU_CANH_DOC_LAP.md) | Prompt dán vào chat Cursor để nạp ngữ cảnh phiên mới |
| [`.cursor/rules/track-d-density-costmap.mdc`](../../.cursor/rules/track-d-density-costmap.mdc) | Rule tự gắn khi mở file Track D |

Commit neo gần nhất: `6f9c66b`.

## Robot thật — Exp.1 / 2 / 3 (live perception)

Odom: **`srf_zed_imu`** + **`scan_static_filter`** (`/scan` → `/scan_static` → SRF). Costmap vẫn `/scan`.  
Perception: Body Tracking + YOLO26 + group/interaction + FER (exp2/3).  
Launch / RViz **riêng real** — không dùng file mô phỏng (`sim_D_*`, `track_d_sim.rviz`).

| Exp | Lệnh | Launch riêng | Config |
|-----|------|--------------|--------|
| **1** | `./tracks/D-density-costmap/run_real_exp1_proxemic_only.sh` | `real_D_exp1_proxemic_only.launch` | `real_tn1` |
| **2** | `./tracks/D-density-costmap/run_real_exp2_social_layers.sh` | `real_D_exp2_social_layers.launch` | `real_tn2` + FER |
| **3** | `./tracks/D-density-costmap/run_real_exp3_density_gate.sh` | `real_D_exp3_density_gate.launch` | `real_tn3` + FER + density gate |

**RViz real:** `rviz/D-density-costmap/track_d_real.rviz`

## Robot thật — Exp.4 / 5 / 6 (POS — kịch bản + latch)

Odom: **`srf_zed_imu`** + **`scan_static_filter`** (`/scan` → `/scan_static` → SRF). Costmap vẫn `/scan`.  
Không FER. Costmap social từ `social_scenario_canonical_trackD_real.yaml`; **bật khi camera detect người, rồi giữ mãi**.

| Exp | Lệnh | Variant | Khác biệt |
|-----|------|---------|-----------|
| **4** | `./tracks/D-density-costmap/run_real_exp4_proxemic_only_pos.sh` | `real_tn4` | proxemic only |
| **5** | `./tracks/D-density-costmap/run_real_exp5_social_layers_pos.sh` | `real_tn5` | 4 lớp, không density gate |
| **6** | `./tracks/D-density-costmap/run_real_exp6_density_gate_pos.sh` | `real_tn6` | + density gate |

**RViz POS:** `rviz/D-density-costmap/track_d_real_pos.rviz` (layout giống exp1–3; zones/objects ON)

**Ghi waypoints (exp4–6 dùng chung):**
```bash
# Stack + AMCL đang chạy; teleop robot tới từng điểm:
./tracks/D-density-costmap/run_record_waypoints_trackd.sh
# → lưu: goal_sender/config/D-density-costmap/waypoints_trackD_real.yaml
# Lệnh: start | goal GOAL_1 | … | save | quit
```

```
| 3D / topics                         | ZED_view (/image_raw)          |
| SII | SGI (ngang, dưới ô 3D)         | ZED skeleton (/image_skeleton) |
```

**SII/SGI đo lường (chương trình chung):**  
`social_navigation_monitor/launch/track_d_sii_sgi_monitor.launch`  
— canonical → `/social_state_gt` → `/sii` `/sgi` → dashboard **horizontal** `/social_dashboard/image`.  
Cả exp1–3 và exp4–6 đều include file này (không phụ thuộc phiên bản riêng).  
Chạy riêng: `./tracks/D-density-costmap/run_sii_sgi_monitor.sh`

**Dựng / chỉnh kịch bản (RViz) — mặc định đủ như exp3 + marker kéo:**
```bash
export DISPLAY=:1
roslaunch social_density_fusion social_map_annotation_session.launch
# RViz: tool Interact → display "Map annotation edit" → kéo actor/object/zone
# Export: menu "Export canonical + MATLAB"
```

## Simulation Gazebo — khớp ICR2026 (nội dung tn1/tn2/tn3)

Cùng **world/map** ICARCV (`vcca2026_wall_v5`). Chỉ khác costmap + `density.button_state`.

| Exp | Lệnh | Nội dung | `button_state` |
|-----|------|----------|----------------|
| **1** | `./tracks/D-density-costmap/run_sim_exp1_proxemic_only.sh` | proxemic only | `false` |
| **2** | `./tracks/D-density-costmap/run_sim_exp2_social_layers.sh` | social layers (4 lớp) | `false` |
| **3** | `./tracks/D-density-costmap/run_sim_exp3_density_gate.sh` | social layers + density gate | `true` |

Launch: `sim_D_exp1_proxemic_only.launch` / `sim_D_exp2_social_layers.launch` / `sim_D_exp3_density_gate.launch`  
Config: `navigation/.../tn{1,2,3}/` + `social_config_exp{1,2,3}_*.yaml` (nội dung = ICR `costmapEx_15_tn*`)  
Engine: `sim_social_v5.launch`  
**RViz chung cả 3 exp:** `rviz/D-density-costmap/track_d_sim.rviz`  

```
| 3D / topics                         | Top_view      |
| SII | SGI (full ngang vùng trái)    | Robot camera  |
```

SII/SGI full chiều ngang **ô dưới-trái** (dưới 3D); cột phải Top_view trên / Robot camera dưới.

```bash
./tracks/D-density-costmap/run_sim_exp1_proxemic_only.sh
./tracks/D-density-costmap/run_sim_exp2_social_layers.sh
./tracks/D-density-costmap/run_sim_exp3_density_gate.sh
```

Ghi **N phút** (sim hoặc real):

```bash
./tracks/D-density-costmap/run_sim_exp2_social_layers.sh 10
./tracks/D-density-costmap/run_real_exp3_density_gate.sh 15
```

## Dev / FER test

```bash
./tracks/D-density-costmap/run_track_d.sh
./tracks/fer_emotion/run_test_webcam.sh --show --focus Angry
```

## Output (mỗi exp)

```
bags/D-density-costmap/<EXP>.bag
videos/D-density-costmap/<EXP>_screen.mp4
csv/D-density-costmap/<EXP>/data_config.m
runs/D-density-costmap/<EXP>.log
```
