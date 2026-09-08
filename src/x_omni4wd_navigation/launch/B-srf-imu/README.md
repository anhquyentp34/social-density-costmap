# Launch Track B — B-srf-imu

- `x_omni4wd_navigation_real_track_b.launch` — robot thật, mọi `odom_mode`
- Odom SRF mặc định: **`scan_static_filter`** (`/scan` → `/scan_static` → SRF; map-only, không body)

17 script exp: `tracks/B-srf-imu/run_real_exp01_*.sh` … `run_real_exp17_*.sh`

```bash
roslaunch x_omni4wd_navigation x_omni4wd_navigation_real_track_b.launch odom_mode:=srf_zed_imu
```

Hoặc: `./tracks/B-srf-imu/run_real_exp15_srf_zed_imu.sh`
