# Track D — Density gate → suy giảm lớp social (Exp.5 vs Exp.6) + CSV MATLAB

**Máy:** IdeaPad · **Map:** `trackD_real_map_exp456_v3` · **POS runners:** `run_real_exp5_*` / `run_real_exp6_*`

---

## 1) Cơ chế trong code (không đoán)

### 1.1 `density.button_state` (social YAML)

| Exp | File | `button_state` | Hệ quả |
|-----|------|----------------|--------|
| 5 | `social_config_real_exp5_social_layers_pos.yaml` | **false** | Fusion **không** publish `/density_info`; `SocialState.density = 0` |
| 6 | `social_config_real_exp6_density_gate_pos.yaml` | **true** | Publish `/density_info` (ρ, people_count) + heartbeat; `SocialState.density = ρ` |

Nguồn: `social_density_fusion_node_zed_pos.py` (`config_density_publish_enabled`, `publish` `/density_info`, `state_msg.density`).

Layer đọc ρ từ **`/social_state.density`** (`social_layer.cpp` → `density_`).

### 1.2 `*_density_weight` — **không** phải “độ đậm lớp”

Đây là **hệ số suy giảm theo mật độ** (dynamic_reconfigure trên từng layer):

| Layer | Param | Công thức trong C++ |
|-------|-------|---------------------|
| Proxemic | `proxemic_density_weight` | `cost = base_cost / exp(w · ρ)` |
| Group / Object / Emotion | `human_group_density_weight` / `human_object_density_weight` / `emotion_density_weight` | `scale = exp(−w · ρ)` nhân bán kính/biên |

→ **w tăng hoặc ρ tăng** ⇒ cost/hình học social **nhỏ đi** (lớp “mờ / biến mất” trên costmap).  
→ **Không** tắt plugin (`enabled` vẫn true). Không phải weight→0 nghĩa là tắt lớp; trái lại **w→0** thì `exp(0)=1` ⇒ social **đầy đủ**.

### 1.3 `path_density_weight_controller`

File: `social_navigation_layers/src/path_density_weight_controller.py`

- Chỉ **HOẠT ĐỘNG** khi có heartbeat `/density_info` (timeout ~1 s) — tức Exp.6.
- Exp.5: không có `/density_info` → controller **degrade**, đẩy weight về base/min, **không relax**.
- Khi kẹt (NO PATH / TEB infeasible / …): **tăng** lần lượt  
  `emotion → human_object → human_group → proxemic` (mỗi lần +step).
- Khi tiến được: giảm dần về base (restore).
- Init mặc định: `reset_to_min_on_start` / `force_min_base_weight` → thường bắt đầu từ **min (0)** rồi mới tăng khi kẹt.

**Ý nghĩa “biến mất” trên paper (đúng code):**  
trên Exp.6, khi ρ > 0 và controller **nâng w**, hệ số  
`atten = exp(−w·ρ) → 0` ⇒ đốm social trên costmap **co lại / gần biến mất**.  
Exp.5: ρ hiệu lực = 0 (và controller không nới) ⇒ atten ≈ 1 ⇒ lớp social **giữ nguyên**.

### 1.4 Topic / param phản ánh theo thời gian

| Tín hiệu | Topic / param |
|----------|----------------|
| ρ, people_count | `/density_info` (`social_density_fusion/DensityInfo`) |
| ρ trong layer | `/social_state` field `density` |
| w_* hiện tại | rosparam / dynreconf  
  `/move_base/global_costmap/proxemic/proxemic_density_weight`  
  `.../human_group_layer/human_group_density_weight`  
  `.../human_object_layer/human_object_density_weight`  
  `.../emotion_layer/emotion_density_weight` |
| Timeline đóng gói | `/social_costmap/layer_weights` · `/social_costmap/layer_weights_json` |
| Costmap tổng | `/move_base/global_costmap/costmap` (ảnh phụ) |

---

## 2) Cách thể hiện trên paper / MATLAB

**Ưu tiên Fig timeline (A):**

- Trục thời gian chung với quỹ đạo / SII.
- Cột trái (hoặc subplot trên): `rho(t)`, `people_count(t)`.
- Subplot dưới: `w_proxemic, w_group, w_interaction, w_emotion`.
- Subplot atten: `atten_* = exp(-w*rho)` — **đây là tín hiệu “biến mất” rõ nhất** (1→0).

**So sánh Exp.5 vs Exp.6 (B):** cùng map + cùng scenario POS; Exp.5: w gần min/phẳng, ρ CSV trống hoặc 0; Exp.6: ρ biến thiên, w bậc thang khi kẹt, atten sụt.

**(C) Tuỳ chọn:** 2 snapshot RViz costmap ρ thấp vs ρ cao / sau relax — chỉ minh họa; số liệu chính vẫn CSV.

---

## 3) Pipeline ghi dữ liệu (đã tích hợp trên IdeaPad)

### Schema CSV

File: `<run_csv_dir>/layer_weights_timeline.csv`

```text
timestamp,rho,people_count,w_proxemic,w_group,w_interaction,w_emotion,atten_proxemic,atten_group,atten_interaction,atten_emotion
```

### Files

| Thành phần | Path |
|------------|------|
| Logger | `src/social_density_fusion/scripts/layer_weights_logger.py` |
| Launch logger | `src/social_density_fusion/launch/layer_weights_logger.launch` |
| Gắn POS | `x_omni4wd_navigation_real_track_d_pos.launch` (`use_layer_weights_logger`, `layer_weights_csv_path`) |
| Runner | `run_real_track_d_pos.sh` → `layer_weights_csv_path:=$PIPE_CSV_DIR/layer_weights_timeline.csv` |
| Bag thêm | `/social_costmap/layer_weights`, `/social_costmap/layer_weights_json` (+ `/density_info` sẵn có) |

Patch cũ: `research-pipeline-os/omni4wd_lab/patches/D-density-costmap-rescue/layer_weights_logger.py` — đã **port** vào package chính.

### Lệnh chạy thử Exp.6 ngắn

```bash
cd ~/omni4wd_ws && source devel/setup.bash
# Chạy vài phút rồi Enter dừng ghi (DURATION=0)
./tracks/D-density-costmap/run_real_exp6_density_gate_pos.sh
```

Hoặc chỉ smoke logger khi stack đã lên:

```bash
rosrun social_density_fusion layer_weights_logger.py \
  _csv_path:=/tmp/layer_weights_timeline.csv _rate:=10.0
rostopic echo -n3 /social_costmap/layer_weights
head /tmp/layer_weights_timeline.csv
```

### Kiểm tra output sau run

```bash
ls -la ~/omni4wd_ws/csv/D-density-costmap/D_real_exp6_*/layer_weights_timeline.csv
head -5 ~/omni4wd_ws/csv/D-density-costmap/D_real_exp6_*/layer_weights_timeline.csv
# Exp.6: cột rho không trống; atten giảm khi w và rho cùng lớn
# Exp.5: rho thường trống/NaN (không có /density_info) hoặc không đổi atten
```

---

## 4) Checklist xác minh Exp.5 vs Exp.6

- [ ] Exp.5: `rostopic hz /density_info` → **không** có / timeout.
- [ ] Exp.6: `/density_info` có `density`, `people_count`.
- [ ] Exp.6: khi robot kẹt / NO PATH → `w_*` tăng (bậc thang) trên CSV.
- [ ] `atten_*` Exp.6 có đoạn → gần 0; Exp.5 gần 1 hoặc không có chuỗi ρ.
- [ ] Cùng map `trackD_real_map_exp456_v3`, cùng waypoints/scenario POS.
- [ ] File `layer_weights_timeline.csv` nằm cạnh CSV extract của run.

---

## 5) PROMPT ngắn — dán sang Cursor Windows (MATLAB)

```
Bạn trên Windows (research-pipeline-os). Vẽ MATLAB so sánh density gate Track D POS Exp.5 vs Exp.6.

Dữ liệu (kéo từ IdeaPad):
  omni4wd_lab/csv/D-density-costmap/<D_real_exp5_...>/layer_weights_timeline.csv
  omni4wd_lab/csv/D-density-costmap/<D_real_exp6_...>/layer_weights_timeline.csv

Cột:
  timestamp, rho, people_count,
  w_proxemic, w_group, w_interaction, w_emotion,
  atten_proxemic, atten_group, atten_interaction, atten_emotion
  (atten = exp(-w*rho); 1=social đầy đủ, →0 = lớp social “biến mất” trên costmap)

Figure đề xuất (một cửa sổ, 3 subplot, time tương đối t−t0):
  (1) rho(t) [và people_count nếu cần]
  (2) bốn w_*(t)
  (3) bốn atten_*(t)
Hai đường/hai cột: Exp.5 vs Exp.6. Caption: Exp.5 button_state=false (không gate);
Exp.6 button_state=true (gate + controller nới density_weight khi kẹt).

Xuất: papers/D-density-costmap/fig/exp-5-vs-6/figure_density_layer_weights.png|pdf
Không bịa số; nếu thiếu CSV thì báo.
```

---

## Lưu ý diễn đạt paper

Tránh viết “weight lớp → 0 thì lớp tắt”.  
Viết: **hệ số suy giảm `*_density_weight` tăng (và/hoặc ρ tăng) làm biên social cost → 0 (`atten→0`)**.

---

## Prompt Windows — tô màu vùng còn lại / để trống vành co

Xem: `PROMPT_WINDOWS_MATLAB_DENSITY_SHRINK_FILL.md`
