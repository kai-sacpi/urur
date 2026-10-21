---
exp_id: MLA-UR5-260914-002
title: mla_short LM vortex, two quality-accepted real-robot repeats, and two empty-field repeats
date: 2026-09-14
timezone: Asia/Shanghai
status: completed_with_quality_flags
environment_id: mla_short
experiment_type: continuous_vortex_repeat
robot: UR5
tool: rod_190mm_d5mm
sensor: NUCLEO four-channel magnetic array
trajectory_repeat_count: 2
background_repeat_count: 2
data_host: ubuntu24
tags:
  - mla_short
  - vortex
  - helix
  - magnetic-field
  - real-robot
  - background-field
---

# `mla_short` LM Vortex 两组质量正常真机实验与双空场报告

## 结论与正式数据选择

本报告只纳入两次传感器质量处于当前容忍范围内的完整 vortex 真机批次。每个批次均包含 5 条固定高度平面内向心 vortex 和 1 条变高度 helix，共 6 条；两批合计 12 次机器人回放全部成功，`play_exit_code=0`。正式窗口内每条仅有 3--13 个 incomplete/channel-drop 标记，`reordered_packets=0`、`malformed_packets=0`。

正式批次根目录：

```text
# Trajectory repeat 1：NUCLEO 重启后重新 SYNC
/home/huaxin/Codes/ur-waypoint/runs/mla_short_lm_vortex_real_20260914T003200Z/

# Trajectory repeat 2：复用同一 NUCLEO SYNC
/home/huaxin/Codes/ur-waypoint/runs/mla_short_lm_vortex_real_20260913T164909Z/
```

对应汇总文件：

```text
/home/huaxin/Codes/ur-waypoint/runs/mla_short_lm_vortex_real_20260914T003200Z/summary.csv
/home/huaxin/Codes/ur-waypoint/runs/mla_short_lm_vortex_real_20260913T164909Z/summary.csv
```

批次根目录名中的时间戳存在不同命名方式，不能仅按目录字符串排序判断先后；本报告中的 Repeat 1/2 顺序依据实际执行顺序和各 `nucleo_capture.yaml` 的 `generated_at_utc`。

## 实验合同

- 环境：`environment_id=mla_short`。
- 工作坐标系：`plane1`，原点为传感器阵列几何中心。
- TCP：`rod_190mm_d5mm`，杆长 190 mm、直径 5 mm。
- 姿态：固定竖直姿态，`quaternion_xyzw=(0,0,0,1)`。
- 主索引：`/home/huaxin/Codes/ur-waypoint/environments/mla_short/trajectories/mla_short_lm_vortex_main_index.csv`。
- 轨迹：`z=7/9/11/13/15 cm` 的 5 条半径 10 cm 平面 vortex，以及 `z=7 -> 15 cm`、半径 10 cm 的 vertical helix。
- 每条 authority 时长 60 s、601 个源 waypoint；执行端直接播放已验证 recording。
- 所有 authority waypoint 和 `time_s` 均保持原样；没有缩放、重定时、裁剪或跳点。
- 每条分别保存机器人实际 FK 和与其映射的 NUCLEO 四通道磁场。

## 两次正式轨迹结果

### Repeat 1 真实产物

```text
/home/huaxin/Codes/ur-waypoint/runs/mla_lm_vortex_center_z07_r10__20260913_013250__20260914_002954/real/
/home/huaxin/Codes/ur-waypoint/runs/mla_lm_vortex_center_z09_r10__20260913_013245__20260914_003127/real/
/home/huaxin/Codes/ur-waypoint/runs/mla_lm_vortex_center_z11_r10__20260913_013605__20260914_003301/real/
/home/huaxin/Codes/ur-waypoint/runs/mla_lm_vortex_center_z13_r10__20260913_013615__20260914_003435/real/
/home/huaxin/Codes/ur-waypoint/runs/mla_lm_vortex_center_z15_r10__20260913_013914__20260914_003609/real/
/home/huaxin/Codes/ur-waypoint/runs/mla_lm_helix_center_z07_to_z15_r10__20260913_014326__20260914_003742/real/
```

### Repeat 2 真实产物

```text
/home/huaxin/Codes/ur-waypoint/runs/mla_lm_vortex_center_z07_r10__20260913_013250__20260914_004931/real/
/home/huaxin/Codes/ur-waypoint/runs/mla_lm_vortex_center_z09_r10__20260913_013245__20260914_005121/real/
/home/huaxin/Codes/ur-waypoint/runs/mla_lm_vortex_center_z11_r10__20260913_013605__20260914_005253/real/
/home/huaxin/Codes/ur-waypoint/runs/mla_lm_vortex_center_z13_r10__20260913_013615__20260914_005427/real/
/home/huaxin/Codes/ur-waypoint/runs/mla_lm_vortex_center_z15_r10__20260913_013914__20260914_005602/real/
/home/huaxin/Codes/ur-waypoint/runs/mla_lm_helix_center_z07_to_z15_r10__20260913_014326__20260914_005737/real/
```

每个 `real/` 目录包含：

```text
execution_telemetry.csv   # 机器人实际关节与 FK 时间序列
nucleo_telemetry.csv      # 映射到机器人同步时基的传感器表
nucleo_capture.yaml       # 原始捕获、质量、映射和哈希 manifest
```

## 逐轨迹质量统计

| 轨迹 | Repeat 1 mapped frames | Repeat 1 incomplete/drop | Repeat 1 max offset | Repeat 2 mapped frames | Repeat 2 incomplete/drop | Repeat 2 max offset |
|---|---:|---:|---:|---:|---:|---:|
| `vortex_z07_r10` | 5997 | 8 / 8 | 4.986 ms | 5999 | 5 / 5 | 4.224 ms |
| `vortex_z09_r10` | 5987 | 12 / 12 | 5.505 ms | 6002 | 3 / 3 | 4.120 ms |
| `vortex_z11_r10` | 5995 | 9 / 9 | 7.569 ms | 5991 | 8 / 8 | 4.134 ms |
| `vortex_z13_r10` | 5989 | 10 / 10 | 4.106 ms | 5998 | 7 / 7 | 4.205 ms |
| `vortex_z15_r10` | 5999 | 5 / 5 | 4.666 ms | 5994 | 11 / 11 | 4.149 ms |
| `helix_z07_to_z15_r10` | 5990 | 9 / 9 | 4.453 ms | 5992 | 13 / 13 | 4.204 ms |

两次 repeat 的每条轨迹均满足：

- 机器人回放成功，`play_exit_code=0`；
- 约 6000 个磁场帧映射到 60 s 权威运动窗口；
- incomplete/channel-drop 为 3--13 组，约为窗口帧数的 `0.05%--0.22%`；
- `reordered_packets=0`；
- `malformed_packets=0`；
- 最大机器人—传感器最近邻映射偏差为 4.106--7.569 ms，低于 50 ms 上限。

各 `nucleo_capture.yaml` 如实保留 `status: incomplete` 和轻微丢包说明；这是显式接受的协议质量标记，不代表机器人回放失败，不应手工改写 manifest。

## 两次背景场

完成两组正式轨迹后，NUCLEO 被再次重启，机械臂从传感器附近移开，然后连续采集两组 30 s 空场。以下两组背景也被点实验报告共同引用：

| 指标 | Background 1 | Background 2 |
|---|---:|---:|
| 数据目录 | `/home/huaxin/Codes/ur-waypoint/runs/empty_field_30s_after_vortex_restart_1__20260913T170755.923338Z__79011bfa/` | `/home/huaxin/Codes/ur-waypoint/runs/empty_field_30s_after_vortex_restart_2__20260913T170847.249669Z__80de3af0/` |
| 完整四通道帧 | 2990 | 2992 |
| capture-clock 时长 | 30.025 s | 30.108 s |
| SYNC 模式 | `performed` | `reused_existing_stream` |
| incomplete groups | 12 | 10 |
| timeout groups | 0 | 1 |
| channel sequence drops | 12 | 25 |
| reordered packets | 0 | 32 |
| malformed packets | 0 | 0 |
| 原始 CSV SHA-256 | `aad599ec06882d3543fada1c9016ecfd86b1f2e1272011b27a2c0acd6e651c02` | `b46d3c2a54a08886108e1e3687129f50cdc567c8970948252428c77e463bd2ca` |

每个背景目录包含 `nucleo_raw_telemetry.csv`、`nucleo_capture.yaml` 和 `sha256sums.txt`。两组背景拥有约 3000 个完整组且无 malformed packet，可用于估计背景均值、方差和两次背景之间的漂移。

## 配对和分析规则

推荐每条轨迹使用：

```text
execution_telemetry.csv + nucleo_telemetry.csv + nucleo_capture.yaml
```

机器人表与轨迹磁场表可通过各自 run 内的 `sync_time_ns` 关联。背景场是在轨迹完成后再次重启 NUCLEO 才采集，因此背景与轨迹不属于同一 SYNC epoch；背景只能按传感器通道/轴形成统计基线，不能用绝对 `sync_time_ns` 跨 run 拼接。

推荐先分别用 Background 1 和 Background 2 做两套背景扣除，再把二者差异作为背景漂移/系统不确定性。所有扣除、滤波、mask replay 和统计结果必须输出到新的派生目录，保留原始 CSV 和 manifest 不变。

## 明确排除的批次

以下批次不属于“两组质量正常的轨迹实验”：

```text
# 机器人均执行成功，但 NUCLEO 丢包和重排逐条累积，最高约 724 组；不用于正式定量分析
/home/huaxin/Codes/ur-waypoint/runs/mla_short_lm_vortex_real_20260913T161325Z/

# NUCLEO preflight 中止的部分尝试；不是完整 repeat
/home/huaxin/Codes/ur-waypoint/runs/mla_short_lm_vortex_real_20260914T002700Z/
```

2026-09-13 更早的原始 vortex 实验也不在本次“两组新重复”的正式集合内；如需合并，应单独做跨会话背景和标定一致性审计。
