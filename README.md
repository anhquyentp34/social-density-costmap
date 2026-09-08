# Social Density Costmap

ROS packages for **density-aware social costmap navigation**: proxemic / group / object / emotion layers with a **sequential density-weight relaxation & restoration** controller.

This repository is a research release accompanying our paper on social-density costmap navigation.

## Features

- Social costmap layers (proxemic, human group, human–object, emotion)
- Density modulation: `scale = exp(-w · ρ)`
- Online controller that **relaxes** social weights when the planner is stuck and **restores** them after the robot makes progress
- Scenario latch mode for position-based social configuration (no FER)
- Live perception mode with optional FER emotions

## Requirements

- Ubuntu 20.04 + ROS Noetic
- `navigation`, `teb_local_planner`, `costmap_2d`, `dynamic_reconfigure`
- Optional: ZED SDK / body tracking for real-robot experiments

## Workspace layout

```
src/
  social_msgs/
  social_navigation_layers/   # layers + path_density_weight_controller
  social_density_fusion/      # fusion, density publish, scenario latch
  x_omni4wd_navigation/       # experiment configs / launch / rviz / map
  goal_sender/
  fer_emotion/                # optional (live emotion experiments)
tracks/social_density_costmap/
docs/
```

## Build

```bash
mkdir -p ~/catkin_ws/src
# copy or clone this repo into ~/catkin_ws/src (or clone at workspace root)
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Density gate (core algorithm)

When `density.button_state` is enabled and `/density_info` is fresh:

1. **Relax** (one layer, one step per trigger):  
   `emotion → human_object → human_group → proxemic`
2. **Restore** after travel ≥ 2.0 m (with hold timers after relax): decrease one step toward base (`w = 0` after controller init)

Higher `*_density_weight` increases attenuation `exp(-w·ρ)` and softens social cost so the robot can pass dense regions when stuck; restoration hardens constraints again after progress.

See `docs/` and `social_navigation_layers/src/path_density_weight_controller.py`.

## Experiments (scripts)

```bash
cd tracks/social_density_costmap
./run_real_exp3_density_gate.sh          # live + density gate
./run_real_exp6_density_gate_pos.sh      # scenario latch + density gate
./run_real_exp5_social_layers_pos.sh     # social layers, gate OFF
./run_real_exp4_proxemic_only_pos.sh     # proxemic only
```

Full robot bring-up (drivers, ZED, odometry) is **not** fully bundled here; use these packages inside a complete Noetic robot workspace.

## Citation

If you use this code, please cite our paper (update bibtex upon publication):

```bibtex
@inproceedings{YourKey2026,
  title={...},
  author={...},
  booktitle={...},
  year={2026}
}
```

## License

MIT (see `LICENSE`). Third-party ROS packages retain their own licenses.
