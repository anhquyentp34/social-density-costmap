# x_omni4wd_navigation (robot that)

Goi navigation cho robot x_omni4wd voi map `willo.yaml`, su dung `map_server + AMCL + move_base`.

## Launch khuyen nghi cho robot that

```bash
roslaunch x_omni4wd_navigation x_omni4wd_navigation.launch
```

Tuy chon hay dung:

- Doi map:
  ```bash
  roslaunch x_omni4wd_navigation x_omni4wd_navigation.launch map_file:=/absolute/path/to/map.yaml
  ```
- Tat bringup neu robot da chay san:
  ```bash
  roslaunch x_omni4wd_navigation x_omni4wd_navigation.launch use_bringup:=false
  ```
- Bat loc laser:
  ```bash
  roslaunch x_omni4wd_navigation x_omni4wd_navigation.launch use_laser_filter:=true
  ```
- Dung DWA thay TEB:
  ```bash
  roslaunch x_omni4wd_navigation x_omni4wd_navigation.launch algorithm_navigation:=dwa
  ```

## Diem da cung co

- Dung mot launch duy nhat `x_omni4wd_navigation.launch` cho robot that.
- Dung bo tham so da tune cho omni:
  - `config/costmap_common_params_x_omni4wd.yaml`
  - `config/costmap_global_params_x_omni4wd.yaml`
  - `config/costmap_local_params_x_omni4wd.yaml`
  - `config/teb_local_planner_params_omni.yaml`
- Loai bo xung dot TF map->odom khi bat AMCL.
- Bo sung cau hinh `laser_filter_params.yaml` va sua launch filter sang dung package.

## Kiem tra nhanh sau khi launch

- `rostopic echo /amcl_pose`
- `rostopic echo /move_base/status`
- `rostopic hz /scan`
- Trong RViz: dat `2D Pose Estimate` truoc khi gui `2D Nav Goal`.
