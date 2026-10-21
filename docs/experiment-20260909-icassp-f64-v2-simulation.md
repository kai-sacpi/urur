# ICASSP F64 v2 仿真与回放验收（2026-09-09）

## 结果

在 `mla` 环境、UR5 模型和 333.65 mm 杆具配置下，13 条 F64 minimum-jerk 权威轨迹均已完成：

1. GPU compile：13/13 通过；
2. mock recording 封存与 `inspect`：13/13 通过；
3. 全新、独立的 mock runtime playback：13/13 退出码为 0。

执行端没有对权威输入做缩放、重定时、裁剪、跳点或替换 waypoint。三条原先的竖直姿态路线在上游重新设计为近竖直可行姿态；位置、`time_s`、点数和周期保持不变。这是新的 authority CSV，不是 runtime 补偿。

## 轨迹集合

| 顺序 | 路线 | 周期 | source 点数 | recording 点数 |
|---:|---|---:|---:|---:|
| 0 | `line_x_vertical_z09` | 12 s | 601 | 1201 |
| 1 | `line_x_tilt90_z09` | 12 s | 601 | 1201 |
| 2 | `line_y_vertical_z09` | 12 s | 601 | 1201 |
| 3 | `line_y_tilt90_z09` | 12 s | 601 | 1201 |
| 4 | `line_x_vertical_z13` | 12 s | 601 | 1201 |
| 5 | `line_x_tilt90_z13` | 12 s | 601 | 1201 |
| 6 | `line_y_vertical_z13` | 12 s | 601 | 1201 |
| 7 | `line_y_tilt90_z13` | 12 s | 601 | 1201 |
| 8 | `line_x_vertical_z15` | 12 s | 601 | 1201 |
| 9 | `line_x_tilt90_z15` | 12 s | 601 | 1201 |
| 10 | `line_y_vertical_z15` | 12 s | 601 | 1201 |
| 11 | `line_y_tilt90_z15` | 12 s | 601 | 1201 |
| 12 | `eight_tilt90_z13` | 16 s | 801 | 1601 |

所有路线的 `uniform_time_scale=1`，且 recording manifest 中 `simulation_completed=true`、`controller_result=succeeded`。

## 权威输入与源代码

- 生成器：`scripts/generate_icassp_f64_13_feasible_v2_trajectories.py`
- 权威 CSV：`environments/mla/trajectories/icassp_f64_mj_feasible_v2_*.csv`
- 权威清单：`environments/mla/trajectories/icassp_f64_13_feasible_v2_authority_manifest.json`
- 杆具/环境快照：`environments/mla/`

## recording 与审计产物（ubuntu24）

- 13 个 sealed recording：
  `/home/huaxin/Codes/ur-waypoint/environments/mla/recordings/icassp_f64_mj_feasible_v2_*`
- canonical recording manifest：
  `/home/huaxin/Codes/ur-waypoint/environments/mla/trajectories/icassp_f64_mj_feasible_v2_13_validated_recordings_manifest.json`
- 播放索引（按上述顺序）：
  `/home/huaxin/Codes/ur-waypoint/environments/mla/trajectories/icassp_f64_mj_feasible_v2_13_validated_recordings_index.csv`
- fail-closed 审计：
  `/home/huaxin/Codes/ur-waypoint/runs/icassp_f64_mj_feasible_v2_13_recordings_audit_final.csv`
- 首轮构建汇总：
  `/home/huaxin/Codes/ur-waypoint/runs/icassp_f64_feasible_v2_recordings_20260909T084311Z/summary.csv`
- 缺口重试汇总（仅 `line_y_vertical_z13`）：
  `/home/huaxin/Codes/ur-waypoint/runs/icassp_f64_feasible_v2_recordings_20260909T085601Z/summary.csv`

首轮唯一失败是 mock controller 的速度跟踪门限，非 IK/碰撞/动力学 compile 失败；在隔离 ROS domain 230 的干净 runtime 重试后通过，未改阈值或轨迹。

## 独立 playback 证据

脚本：`scripts/verify_icassp_f64_mock_playback.sh`

运行使用 ROS domain 232，启动一个全新的 mock runtime，然后逐条执行 `play --auto-prepare`。证据目录：

`/home/huaxin/Codes/ur-waypoint/runs/icassp_f64_mj_feasible_v2_independent_mock_playback_20260909T092055Z/`

- `summary.csv`：13 行，`play_status=0` 全部通过；
- `logs/<trajectory>.play.log`：每条实际回放的 telemetry 路径；
- `telemetry/<trajectory>.execution_telemetry.csv`：已复制归档的实际回放采样，所有文件的 `speed_scaling` 均为 1；
- `run_manifest.yaml`、`simulation.launch.log`：运行域和启动证据。

## 打包验收包

- ubuntu24：
  `/home/huaxin/Codes/ur-waypoint/artifacts/icassp_f64_mj_feasible_v2_13_recordings_with_playback_evidence_20260909T173800Z.tar.gz`
- 本机副本：
  `artifacts/icassp_f64_mj_feasible_v2_13_recordings_with_playback_evidence_20260909T173800Z.tar.gz`
- SHA-256：
  `f786feb982742d82ab21713d3d3514499f768948c7cc2adb17799aa9749b3e72`

## 真机回放入口

在 ubuntu24 启动 UR driver 后，进入仓库并执行：

```bash
cd /home/huaxin/Codes/ur-waypoint
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash scripts/replay_icassp_f64_13.sh
```

脚本默认选择 `icassp_f64_mj_feasible_v2` 索引；也可显式指定：

```bash
bash scripts/replay_icassp_f64_13.sh \
  --id-prefix icassp_f64_mj_feasible_v2
```

上述 mock 证据只证明 sealed recording 的链路完整和独立仿真回放通过，不等同于真机安全放行；真机仍需由操作者确认驱动、急停、工作区和杆具安装状态。
