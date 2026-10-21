# MLA 54 点可达性与回放仿真实验

## 结论

在操作者确认与现实一致的 `mla` 环境中，指定的 54 个 TCP 位置全部可达：

- `54/54 REACHABLE`；
- 54 个点均采用首选竖直姿态，`tilt=0 deg`；
- 没有移动目标位置，也没有使用邻近点替代；
- 没有触发 `5/10/15/20/25 deg` 倾斜候选；
- 54 个权威 CSV 均完成 GPU compile、mock controller 仿真执行、recording seal 和 inspect；
- 正式集合包含 54 个唯一、`status: validated` 的 recording。
- 最低层角点、中层中心点和最高层角点另做了 3/3 次独立 mock playback，全部成功。

本次只使用隔离的 mock ROS 域 `ROS_DOMAIN_ID=219`，没有调用真机 playback，也没有采集 NUCLEO 数据。

## 坐标与权威输入

传感器板坐标系 `A` 就是当前 `mla` 环境中的 `plane1`：原点位于阵列几何中心。已验证的 `plane1 +Z` 指向板内，因此实验高度使用以下精确映射：

```text
tip_x_plane1 =  x_A
tip_y_plane1 =  y_A
tip_z_plane1 = -z_A
```

目标集合为：

```text
x_A, y_A = -0.05, 0.00, +0.05 m
z_A      =  0.07, 0.08, 0.09, 0.10, 0.11, 0.12 m
```

每个权威 CSV 使用同一格式：

```text
time_s,frame_id,tip_x_m,tip_y_m,tip_z_m,tip_qx,tip_qy,tip_qz,tip_qw,phase
```

每个文件只有 `time_s=[0,1,3]` 三行，三行 Pose 完全相同，因而 TCP 在目标点严格静止 3 s。选定的竖直姿态为 `plane1` 中的四元数 `xyzw=[0,0,0,1]`。这些 CSV 一旦生成就是权威输入；执行链禁止缩放、重定时、裁剪、跳点或自动改写位置。

## 搜索与路径检查

每个位置按以下策略处理：

1. 先检查竖直姿态；
2. 若失败，再按 `5,10,15,20,25 deg` 搜索，每个非零倾角以 `30 deg` 步长枚举方位角；
3. 候选必须通过 IK、规范关节分支、碰撞、关节限位和奇异性指标检查；
4. 同时检查环境 home 到目标，以及上一可达目标到当前目标的完整 c-space 路径；
5. 只有全部姿态失败时才允许写入 `UNREACHABLE`，且不会生成替代轨迹。

网格按逐层蛇形顺序排列，以减少相邻目标间的关节变化。到目标的 prepare 路径不属于 3 s 权威 recording，也不会改变 recording 内的时间戳。

## 可达性结果

| `z_A` (m) | 可达 | 选定倾角 | 端点碰撞净空 (mm) | 关节限位最小余量 (rad) |
|---:|---:|---:|---:|---:|
| 0.07 | 9/9 | 0 deg | 9.998837 | 1.855240 |
| 0.08 | 9/9 | 0 deg | 19.996509 | 1.893058 |
| 0.09 | 9/9 | 0 deg | 22.835638–23.465708 | 1.932096 |
| 0.10 | 9/9 | 0 deg | 22.835638–23.465708 | 1.972477 |
| 0.11 | 9/9 | 0 deg | 22.835638–23.465708 | 2.014345 |
| 0.12 | 9/9 | 0 deg | 22.835638–23.465708 | 2.057875 |

完整 recording 逐采样碰撞复核的全局净空范围为 `9.904956–23.465708 mm`。54 份 validation report 的其他汇总结果为：

- controller result 全部为 `succeeded`；
- 每条 recording 为 301 个关节点，`dt=0.01 s`，总时长严格为 `3 s`；
- `uniform_time_scale=1`；静止段速度、加速度和 jerk 比率均为 0；
- 最大 source 位置误差约 `0.000659 mm`；
- 最大 source 姿态误差约 `0.000019 deg`；
- 最大 mock 关节位置跟踪误差约 `1.20e-7 rad`。

构建期间两个 recording 首次 inspect 被数值审计拒绝，原因是四元数序列化后 `acos(dot≈1)` 的浮点边界误差。修复仅稳定了报告一致性判断，真实安全阈值仍保持 FK `0.1 deg`、source `0.5 deg`，没有放宽碰撞或轨迹门限。最终正式索引将这两个同源、已验证产物标为 `RECOVERED`：

- `mla_grid_z090_xp050_yp050__20260904_234517`
- `mla_grid_z120_xz000_ym050__20260904_234558`

因此，不应按目录通配符选择远端历史产物；正式集合只以 `selected_recordings.csv` 的 54 行为准。

## 文件位置

Mac 项目：

```text
/Users/huaxin/Codes/ur-waypoint/environments/mla/trajectories/mla_grid_*.csv
/Users/huaxin/Codes/ur-waypoint/environments/mla/recordings/<recording_id>/
/Users/huaxin/Codes/ur-waypoint/artifacts/mla_grid_54_20260904T150316Z/
```

Ubuntu24：

```text
/home/huaxin/Codes/ur-waypoint/environments/mla/trajectories/mla_grid_*.csv
/home/huaxin/Codes/ur-waypoint/environments/mla/recordings/<recording_id>/
/data/ur-waypoint/mla_grid_54/20260904T150316Z/
```

关键索引和证据：

```text
search/reachability.csv
search/reachability_attempts.csv
search/capture_windows.csv
search/manifest.yaml
build/selected_recordings.csv
build/recording_build_index.csv
build/recording_build_manifest.yaml
playback_validation/manifest.yaml
playback_validation/runs/*/simulation/execution_telemetry.csv
```

便携归档：

```text
artifacts/mla_grid_54_20260904T150316Z/mla_grid_54_selected_20260904T150316Z.tar.gz
artifacts/mla_grid_54_20260904T150316Z/mla_grid_54_selected_20260904T150316Z.tar.gz.sha256
```

归档只包含正式选中的 54 个 recording、54 个权威 CSV、`mla` 环境快照、生成代码及全部证据；归档 SHA-256 为：

```text
c1d67dd6e0674e3699d5ba56413eca190e9228ed1beee14b570e419f69676654
```

> 若归档随后因文档补充而重新构建，应以同目录 `.sha256` 文件中的值为准。

## 在 Ubuntu24 查看一条仿真回放

终端 A：

```bash
cd /home/huaxin/Codes/ur-waypoint
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
export PYTHONNOUSERSITE=1
source install/setup.bash
export ROS_DOMAIN_ID=77
ros2 launch hx_waypoint_bringup simulation.launch.py \
  ur_type:=ur5 environment_id:=mla workspace:="$PWD" launch_rviz:=true
```

终端 B：

```bash
cd /home/huaxin/Codes/ur-waypoint
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
export PYTHONNOUSERSITE=1
source install/setup.bash
export ROS_DOMAIN_ID=77
ros2 run hx_waypoint_tools tipdraw --workspace "$PWD" play \
  --env mla \
  --recording mla_grid_z070_xm050_ym050__20260904_231257 \
  --auto-prepare
```

`mla` 的起点策略是 `home_then_recording_start`：runtime 先走无碰撞 prepare，再提交已经 sealed 的 3 s 静止 recording。播放阶段不重新求解、不修改坐标和时间。

## 实现与验证入口

```text
src/hx_waypoint_tools/hx_waypoint_tools/mla_grid.py
scripts/search_mla_grid_reachability.py
scripts/build_mla_grid_recordings.py
tests/test_mla_grid.py
tests/test_recording_store.py
```

本地和 Ubuntu24 的相关测试均为 `66 passed`。本机全量测试因本机没有安装 `torch` 无法收集 GPU 相关模块；Ubuntu24 已完成对应 GPU 与 ROS 链路验证。
