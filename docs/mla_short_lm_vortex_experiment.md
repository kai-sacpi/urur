# mla_short 连续 LM 轨迹：首批真机回放清单

## 目的

这组轨迹用于在 190 mm TCP 的 `mla_short` 环境中采集连续运动数据，比较不同高度下的 LM 定位误差，并用一条带高度变化的轨迹检查空间纵深方向的表现。轨迹保持固定竖直姿态，不包含点阵、姿态扫描或 EKF。

## 几何与时间参数

- 传感器板坐标系原点为板中心；报告中的高度是磁铁尖端到板面的正距离。
- 所有源文件使用 `plane1`。该环境的 `+Z` 指向板内，所以物理高度 `h` 在 CSV 中编码为 `tip_z_m=-h`。
- 固定姿态四元数为 `(qx,qy,qz,qw)=(0,0,0,1)`，即竖直工具姿态。
- 平面螺旋：圆心 `(0,0)` cm、外半径 10 cm，向中心收拢 1.5 圈。
- 上升螺旋：半径 10 cm，物理高度从 7 cm 平滑上升到 15 cm，绕行 2 圈。
- 每条轨迹 60 s；前 3 s 静止、后 3 s 静止；中间使用五次最小跃度参数化。
- 源轨迹采样间隔 0.1 s，共 601 个源点。封存回放轨迹由环境的样条插值生成，不手工缩放、裁剪或重定时。

平面螺旋从 `(-10,0,z)` cm 出发，连续收拢到 `(0,0,z)` cm；上升螺旋从 `(-10,0,7)` cm 出发，在保持半径的同时上升到 `(-10,0,15)` cm。源轨迹的最大笛卡尔速度约为 2.1 cm/s（平面螺旋）和 4.4 cm/s（上升螺旋），适合先做低速真机采集。

## 首批六条路线

| 顺序 | 路线 | 物理含义 | 封存 recording |
|---:|---|---|---|
| 1 | `mla_lm_vortex_center_z07_r10` | `z=7 cm` 平面螺旋 | `mla_lm_vortex_center_z07_r10__20260913_013250` |
| 2 | `mla_lm_vortex_center_z09_r10` | `z=9 cm` 平面螺旋 | `mla_lm_vortex_center_z09_r10__20260913_013245` |
| 3 | `mla_lm_vortex_center_z11_r10` | `z=11 cm` 平面螺旋 | `mla_lm_vortex_center_z11_r10__20260913_013605` |
| 4 | `mla_lm_vortex_center_z13_r10` | `z=13 cm` 平面螺旋 | `mla_lm_vortex_center_z13_r10__20260913_013615` |
| 5 | `mla_lm_vortex_center_z15_r10` | `z=15 cm` 平面螺旋 | `mla_lm_vortex_center_z15_r10__20260913_013914` |
| 6 | `mla_lm_helix_center_z07_to_z15_r10` | `z=7 -> 15 cm` 上升螺旋 | `mla_lm_helix_center_z07_to_z15_r10__20260913_014326` |

六个 recording 均在 `mla_short` 中完成 mock execution、封存和 `inspect=0`。前五条的封存轨迹各为 6001 个 0.01 s 点；上升螺旋为 6225 个点（由同一环境的样条插值产生），源点数和源时间轴均为 601/60 s。

批量检查原始记录见 `vortex_recordings_summary.csv`；六条主路线的 authority 与 recording manifest 哈希均已复核一致。

## 文件位置

源轨迹与索引：

- `/home/huaxin/Codes/ur-waypoint/environments/mla_short/trajectories/`
- `/home/huaxin/Codes/ur-waypoint/environments/mla_short/trajectories/mla_short_lm_vortex_main_index.csv`

封存 recording：

- `/home/huaxin/Codes/ur-waypoint/environments/mla_short/recordings/`

生成器与本地权威副本：

- `/Users/huaxin/Codes/ICASSP2027/analysis/continuous_trajectory_reachability_20260912/generate_mla_short_vortex.py`
- `/Users/huaxin/Codes/ICASSP2027/analysis/continuous_trajectory_reachability_20260912/authority_vortex_v2/`

## 真机回放

已安装脚本：

```bash
cd /home/huaxin/Codes/ur-waypoint
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash
bash scripts/replay_mla_short_lm_vortex_main.sh \
  --port /dev/serial/by-id/<NUCLEO-device-id>
```

脚本会先校验六条 authority、recording manifest 和 SHA-256，再按顺序逐条执行 `tipdraw play --auto-prepare --record-nucleo`。中途需要续跑时，例如从第 4 条开始：

```bash
bash scripts/replay_mla_short_lm_vortex_main.sh \
  --port /dev/serial/by-id/<NUCLEO-device-id> \
  --start-sequence 4
```

若只想先试一条路线，可直接把相应 recording ID 代入：

```bash
ros2 run hx_waypoint_tools tipdraw --workspace /home/huaxin/Codes/ur-waypoint play \
  --env mla_short --recording mla_lm_vortex_center_z09_r10__20260913_013245 \
  --auto-prepare --record-nucleo \
  --nucleo-port /dev/serial/by-id/<NUCLEO-device-id> --nucleo-baudrate 115200
```

回放结果默认写入 `/home/huaxin/Codes/ur-waypoint/runs/mla_short_lm_vortex_real_<UTC时间>/`。本次只完成了仿真封存和检查，没有启动真实 UR5。

## 可选扩展

另外已通过检查的扩展路线（不纳入首批六条的主索引）包括：

- `mla_lm_vortex_center_z09_r15`
- `mla_lm_vortex_center_z13_r15`
- `mla_lm_vortex_safe_z13_r10`

`mla_lm_vortex_safe_z09_r10` 的 compile-only 通过，但一次并发封存受到模拟 telemetry 时间抖动门禁影响，故不放入主集合。
