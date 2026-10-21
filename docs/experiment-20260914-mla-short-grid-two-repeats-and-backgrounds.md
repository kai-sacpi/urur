---
exp_id: MLA-UR5-260914-001
title: mla_short 245-point vertical grid, two real-robot repeats, and two empty-field repeats
date: 2026-09-14
timezone: Asia/Shanghai
status: completed_with_quality_flags
environment_id: mla_short
experiment_type: static_grid_repeat
robot: UR5
tool: rod_190mm_d5mm
sensor: NUCLEO four-channel magnetic array
point_repeat_count: 2
background_repeat_count: 2
data_host: ubuntu24
tags:
  - mla_short
  - static-grid
  - magnetic-field
  - real-robot
  - background-field
---

# `mla_short` 245 点竖直网格双重复与双空场报告

## 结论与正式数据选择

本报告归档两次新的 245 点真机逐点采集，以及机械臂移开、NUCLEO 重启后连续采集的两次 30 s 空场。两次点实验均完成 `245/245` 点，`UNREACHABLE=0`；机器人实际 FK 和磁场数据完整落盘。两次空场分别写入 2990 和 2992 个四通道完整组。

正式纳入本报告的目录只有下列四个：

```text
# 点实验 repeat 1
/data/ur-waypoint/mla_short_vertical_grid_live/20260913T152006Z/

# 点实验 repeat 2
/data/ur-waypoint/mla_short_vertical_grid_live/20260913T154606Z/

# 空场 repeat 1
/home/huaxin/Codes/ur-waypoint/runs/empty_field_30s_after_vortex_restart_1__20260913T170755.923338Z__79011bfa/

# 空场 repeat 2
/home/huaxin/Codes/ur-waypoint/runs/empty_field_30s_after_vortex_restart_2__20260913T170847.249669Z__80de3af0/
```

目录名中的时间戳由远端程序生成，以其 manifest 中的 UTC 时间为准；实验日在本报告中按 Asia/Shanghai 记为 2026-09-14。

## 实验合同

- 环境：`environment_id=mla_short`。
- 工作坐标系：`plane1`，原点为传感器阵列几何中心。
- TCP：`rod_190mm_d5mm`，杆长 190 mm、直径 5 mm。
- 姿态：严格竖直，`quaternion_xyzw=(0,0,0,1)`。
- 横向位置：`x,y in {-15,-10,-5,0,5,10,15} cm`。
- 高度：`z in {7,9,11,13,15} cm`；manifest 记录 `plane1_z_sign=negative`，即实际目标使用 `plane1` 局部负 Z 一侧。
- 总位置数：`7 x 7 x 5 = 245`。
- 每点停稳采集：3 s；机器人采样目标频率 100 Hz。
- 运动方法：MoveIt 碰撞检查、IK、规划并执行到每个目标点。
- 未使用 recording artifact；没有轨迹缩放、重定时、裁剪或跳点。
- NUCLEO：4 个通道，115200 baud；机器人和传感器表使用 Ubuntu 主机同步时基进行映射。

## 两次点实验结果

| 指标 | Repeat 1 | Repeat 2 |
|---|---:|---:|
| 数据目录 | `/data/ur-waypoint/mla_short_vertical_grid_live/20260913T152006Z/` | `/data/ur-waypoint/mla_short_vertical_grid_live/20260913T154606Z/` |
| manifest 状态 | `complete_with_quality_flags` | `complete_with_quality_flags` |
| NUCLEO 同步模式 | `performed_once` | `reused_existing` |
| 成功点 / 计划点 | 245 / 245 | 245 / 245 |
| 不可达点 | 0 | 0 |
| 映射到 245 个采集窗口的传感器帧 | 73,556 | 73,512 |
| 每点传感器帧范围 | 297--303 | 296--303 |
| 最大机器人—传感器映射偏差 | 8.652 ms | 8.811 ms |
| 窗口内 incomplete groups | 52 | 109 |
| 窗口内 channel sequence drops | 52 | 118 |
| 窗口内 reordered packets | 0 | 100 |
| 窗口内 malformed packets | 0 | 0 |

按映射帧数估计，channel-drop 标记约占 Repeat 1 的 `0.071%`、Repeat 2 的 `0.161%`。Repeat 2 另有约 `0.136%` 的 reordered 标记。两次均保留原始数据和逐点窗口质量表；这些标记没有导致点缺失，但分析时不得隐去。

每个点实验目录中的主要文件为：

```text
schedule.csv                 # 权威采集顺序和目标位姿
point_results.csv            # 每点执行结果
capture_windows.csv          # 每个 3 s 采集窗口
robot_actual_fk.csv          # 机器人实际关节/FK 时间序列
nucleo_raw_telemetry.csv     # 原始传感器时间序列
nucleo_mapped.csv            # 映射到机器人时基的传感器表
sensor_window_quality.csv    # 逐点解析与同步质量
manifest.json                # 环境、运动、同步和统计元数据
sha256sums.txt               # 完整性校验
```

## 两次背景场

两次背景场均在机械臂已移开后采集，没有启动 UR 控制，也没有发送机器人运动命令。NUCLEO 在 Background 1 前重启，因此 Background 1 执行了一次新 SYNC；Background 2 复用同一数据流，没有重复握手。

| 指标 | Background 1 | Background 2 |
|---|---:|---:|
| 数据目录 | `/home/huaxin/Codes/ur-waypoint/runs/empty_field_30s_after_vortex_restart_1__20260913T170755.923338Z__79011bfa/` | `/home/huaxin/Codes/ur-waypoint/runs/empty_field_30s_after_vortex_restart_2__20260913T170847.249669Z__80de3af0/` |
| 请求时长 | 30.0 s | 30.0 s |
| capture-clock 时长 | 30.025 s | 30.108 s |
| 完整四通道帧 | 2990 | 2992 |
| SYNC 模式 | `performed` | `reused_existing_stream` |
| incomplete groups | 12 | 10 |
| timeout groups | 0 | 1 |
| channel sequence drops | 12 | 25 |
| reordered packets | 0 | 32 |
| malformed packets | 0 | 0 |
| 原始 CSV SHA-256 | `aad599ec06882d3543fada1c9016ecfd86b1f2e1272011b27a2c0acd6e651c02` | `b46d3c2a54a08886108e1e3687129f50cdc567c8970948252428c77e463bd2ca` |

背景场目录包含：

```text
nucleo_raw_telemetry.csv
nucleo_capture.yaml
sha256sums.txt
```

两组空场的 manifest 均为 `complete_with_quality_flags`。它们拥有约 3000 个完整四通道组、没有 malformed packet，可作为本次点实验的两次独立背景估计；建议分别计算 64 个三轴节点/通道轴的均值、标准差和稳健中位数，并在派生结果中同时报告两个背景重复之间的差异。

## 配对和分析规则

推荐分析输入：

```text
点实验：robot_actual_fk.csv + nucleo_mapped.csv + sensor_window_quality.csv
背景场：nucleo_raw_telemetry.csv + nucleo_capture.yaml
```

注意：两组背景是在点实验结束后又一次重启 NUCLEO 后采集的，因此它们与点实验不属于同一 SYNC epoch。背景扣除只应按传感器通道/轴使用背景统计量，不能把背景文件的 `sync_time_ns` 与点实验机器人表直接做跨 run 时间连接。

所有背景扣除、异常值处理、滤波和统计汇总都应写入新的派生目录。不要改写以上原始 CSV、manifest 或 `sha256sums.txt`。

## 不纳入范围

- 2026-09-12 的旧 245 点网格 `/data/ur-waypoint/mla_short_vertical_grid_live/20260912T144235Z/` 不属于本次双重复。
- 更早的空场、标定和连续轨迹数据不属于本报告的四个正式输入。
- 本报告不包含连续 vortex 轨迹；连续轨迹及同一对背景场见独立报告 `experiment-20260914-mla-short-vortex-two-valid-repeats-and-backgrounds.md`。
