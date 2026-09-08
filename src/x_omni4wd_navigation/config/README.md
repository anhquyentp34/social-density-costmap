# config — snapshot L1 vs overlay L2

| Folder | Lớp | Ghi |
|--------|-----|-----|
| `ICR2026/` | L1 đóng băng | Không |
| `ICARCV2026/` | L1 đóng băng | Không |
| `C-prediction-costmap/` | L2 Track C | Được khi ACTIVE = C |
| `D-density-costmap/` | L2 Track D (mật độ → costmap) | Được khi ACTIVE = D |
| `A-yolo-costmap/` | L2 Track A | Được khi ACTIVE = A |
| `B-srf-imu/` | L2 Track B | Được khi ACTIVE = B |
| `*.yaml` ở root | Legacy dùng chung | Không sửa in-place; copy sang folder track |

Symlink `demo_*.yaml` ở root trỏ vào `C-prediction-costmap/` để launch/docs cũ không gãy.
