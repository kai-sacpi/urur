# ICASSP 动态轨迹：四种图案 × 五个高度（2026-09-08）

## 范围

本次正式集合为 20 条连续轨迹：

- `circle`
- `rounded square`（文件/ID：`rounded_square`）
- `figure eight`（文件/ID：`figure8`）
- `depth loop`（工程中沿用 `space`；本次完整集合使用 `depth_loop_safe10mm`）

每种图案的 nominal height 为 `z_A = 7, 9, 11, 13, 15 cm`，工作帧为
`plane1`，映射为 `tip_z_m = -z_A`，姿态固定为四元数 `[0, 0, 0, 1]`。

所有 authority CSV 都是 0.1 s 采样、60 s 总时长、末尾显式 3 s 静止
hold，并使用 quintic minimum-jerk 参数化。`time_s` 和 waypoint 是硬约束；
执行端没有缩放、重定时、裁剪、跳点或替换。

## 深度环的上游重设计

原先居中的 40 mm `space` 深度环保持原文件不变，作为历史证据。严格 5 mm
场景/自碰撞净空检查显示其 7、9、15 cm 版本不可用，因此没有让 runtime
修改它们。

本次完整集合采用新的上游 authority：

`icassp_dynamic_depth_loop_safe10mm_z07.csv` … `z15.csv`

它保留 50 mm XY 半径和非平面环路，但将深度振幅改为 10 mm，并只向远离
传感器的方向运动，即每个高度的实际 `z_A` 距离范围为 `[nominal,
nominal + 1 cm]`。这是为当前 `mla` 碰撞环境重新设计的输入，不是运行时
补偿。

## 仿真验证

- 机器：Ubuntu24，RTX 3090
- 环境：`environments/mla`
- cuRobo revision：`a35a708ecfbb26eb9ab2d7ef22c65919c4fae4a9`
- controller：`joint_trajectory_controller`（mock hardware）
- 每条 recording：601 个 source waypoint，6001 个 joint 样本，60 s
- `uniform_time_scale = 1`
- controller result：`succeeded`
- recording status：`validated`

20/20 条均已通过独立 mock playback、hash sealing 和 `inspect`。完整映射、
authority SHA-256、recording ID 与相对路径见：

- `environments/mla/trajectories/icassp_dynamic_four_shapes_complete_manifest.json`
- `environments/mla/trajectories/icassp_dynamic_four_shapes_complete_index.csv`

Recording 根目录（Ubuntu24）：

`/home/huaxin/Codes/ur-waypoint/environments/mla/recordings/`

每条 recording 的目录名由 index 中的 `recording_id` 给出。旧的居中 40 mm
失败 authority 和失败日志仍保留在：

`/home/huaxin/Codes/ur-waypoint/runs/icassp_dynamic_four_shapes_sim_20260908T114028Z/logs/`

## 后续回放

真机连接和 MoveIt 已启动后，在 Ubuntu24 执行：

```bash
cd /home/huaxin/Codes/ur-waypoint
bash scripts/replay_icassp_dynamic_four_shapes_complete.sh
```

脚本按 index 的固定顺序播放 20 条 recording。`mla/recording.yaml` 的
`home_then_recording_start` 策略会在每条 recording 前执行碰撞检查的起始
准备；sealed recording 本身仍原样发送。

## 源码

- 原四形生成器：`scripts/generate_icassp_dynamic_four_shape_trajectories.py`
- 深度环安全版生成器：`scripts/generate_icassp_dynamic_depth_loop_safe_trajectories.py`
- 完整集合索引生成器：`scripts/assemble_icassp_dynamic_four_shapes_complete.py`
- 仿真批处理入口（含 `four_shapes_complete` profile）：
  `scripts/build_icassp_dynamic_recordings.sh`
- 真机完整回放入口：`scripts/replay_icassp_dynamic_four_shapes_complete.sh`

