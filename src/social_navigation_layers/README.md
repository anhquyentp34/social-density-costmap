# social_navigation_layers

## Path Density Weight Controller

Node `path_density_weight_controller.py` dieu chinh online cac `*_density_weight` cua social layers thong qua dynamic reconfigure de ho tro robot thoat ket va giam dao dong trong dieu huong.

Layer duoc dieu khien:

- `emotion_density_weight`
- `human_object_density_weight`
- `human_group_density_weight`
- `proxemic_density_weight`

Moi cap nhat deu day dong bo cho global + local costmap.

## Logic tong quan

### Relax (tang weight)

Controller tang weight khi gap cac trigger:

- `NO PATH`
- `TEB footprint infeasible`
- `TEB possible oscillation`
- `NO PROGRESS`
- `ACTIVE GOAL TIMEOUT` (tuy chon)

Thu tu relax hien tai (thong nhat cho moi trigger, bao gom TEB):

1. `emotion`
2. `human_object`
3. `human_group`
4. `proxemic`

Voi trigger TEB (`teb_footprint_infeasible`/`teb_oscillation`), moi lan relax co the tang toi da 2 layer lien tiep theo thu tu tren neu con room.

### Restore (giam ve base)

Controller giam weight theo `progress_restore` khi robot da di du quang duong tren `/odom`.

Guard de giam nhap nhay:

- `restore_min_interval_sec`
- `restore_hold_after_relax_sec`

## Data gating theo `/density_info`

Controller dung heartbeat tu topic `/density_info`:

- Co du lieu moi trong timeout -> controller hoat dong.
- Mat du lieu qua timeout -> controller "im":
  - dung tat ca relax/restore,
  - reset trigger counters,
  - dua cac weight ve `base_weight`.

Dieu nay duoc quan ly boi:

- `~density_info_topic` (default: `/density_info`)
- `~require_density_info_topic` (default: `true`)
- `~density_info_topic_timeout_sec` (default: `1.0`)
- `~degrade_when_density_unavailable` (default: `true`)

## Build

```bash
cd /home/quyenanhpt/VCCA2026
catkin_make --pkg social_navigation_layers
source devel/setup.bash
```

## Chay nhanh

```bash
rosrun social_navigation_layers path_density_weight_controller.py
```

## Tham so quan trong

### Co ban

- `~plan_topic` (default: `/move_base/GlobalPlanner/plan`)
- `~control_rate_hz` (default: `2.0`)
- `~client_timeout_sec` (default: `5.0`)
- `~reset_to_min_on_start` (default: `true`)
- `~force_min_base_weight` (default: `true`)

### Trigger/guard

- `~no_path_trigger_count` (default: `1`)
- `~no_path_grace_after_goal_sec` (default: `1.5`)
- `~restore_min_interval_sec` (default: `2.0`)
- `~restore_hold_after_relax_sec` (default: `2.5`)

### Trigger stuck

- `~use_teb_footprint_infeasible` (default: `true`)
- `~teb_footprint_infeasible_topic` (default: `/move_base/TebLocalPlannerROS/footprint_trajectory_infeasible`)
- `~teb_infeasible_trigger_count` (default: `2`)
- `~use_teb_oscillation_relax` (default: `true`)
- `~teb_oscillation_log_topic` (default: `/rosout_agg`)
- `~teb_oscillation_trigger_count` (default: `1`)
- `~teb_oscillation_relax_min_interval_sec` (default: `2.0`)
- `~use_no_progress_relax` (default: `true`)
- `~odom_topic` (default: `/odom`)
- `~no_progress_distance_eps` (default: `0.08`)
- `~no_progress_duration_sec` (default: `3.0`)
- `~no_progress_relax_min_interval_sec` (default: `1.5`)
- `~use_active_goal_timeout_relax` (default: `true`)
- `~active_goal_relax_delay_sec` (default: `12.0`)
- `~active_goal_relax_min_interval_sec` (default: `8.0`)

### Layer defaults

- Emotion: `step=2.0`, `min=0.0`, `max=25.0`
- HumanObject: `step=4.0`, `min=0.0`, `max=10.0`
- HumanGroup: `step=2.0`, `min=0.0`, `max=50.0`
- Proxemic: `step=1.0`, `min=0.0`, `max=50.0`

### Key va namespace dynamic reconfigure

Keys:

- `~emotion_weight_key` / `~emotion_weight_key_local`
- `~human_object_weight_key` / `~human_object_weight_key_local`
- `~human_group_weight_key` / `~human_group_weight_key_local`
- `~proxemic_weight_key` / `~proxemic_weight_key_local`

Server namespaces:

- `~emotion_server` / `~emotion_server_local`
- `~human_object_server` / `~human_object_server_local`
- `~human_group_server` / `~human_group_server_local`
- `~proxemic_server` / `~proxemic_server_local`

## Vi du launch

```xml
<node pkg="social_navigation_layers"
      type="path_density_weight_controller.py"
      name="path_density_weight_controller"
      output="screen">
  <param name="plan_topic" value="/move_base/GlobalPlanner/plan"/>
  <param name="density_info_topic" value="/density_info"/>
  <param name="require_density_info_topic" value="true"/>
  <param name="density_info_topic_timeout_sec" value="1.0"/>
  <param name="degrade_when_density_unavailable" value="true"/>

  <param name="use_teb_footprint_infeasible" value="true"/>
  <param name="use_teb_oscillation_relax" value="true"/>
  <param name="use_no_progress_relax" value="true"/>
</node>
```

## Luu y van hanh

- Node doc `base_weight` tu dynamic reconfigure luc ket noi server.
- Neu bat `force_min_base_weight` hoac `reset_to_min_on_start`, base/co hinh ban dau co the bi dua ve min theo chu y nghia tham so.
- Neu namespace server sai, node khong the ket noi va se canh bao.
- Neu `/density_info` khong co du lieu moi va che do gating duoc bat, controller se disable dung thiet ke.
