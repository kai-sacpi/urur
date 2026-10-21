# `mla_short` LM Vortex 真机实验简报

日期：2026-09-13  
环境：`mla_short`（UR5，`rod_190mm_d5mm`，190 mm / 5 mm TCP）  
工作坐标系：`plane1`（传感器板中心）  
姿态：固定竖直姿态，四元数 `(0, 0, 0, 1)`

## 实验内容

本次按主索引
`/home/huaxin/Codes/ur-waypoint/environments/mla_short/trajectories/mla_short_lm_vortex_main_index.csv`
执行 6 条连续轨迹：5 条固定高度平面内向心螺旋（z=7/9/11/13/15 cm）和 1 条 z=7→15 cm 上升螺旋。每条 authority 时长 60 s、601 个源点；真机回放同时记录机器人实际 FK 和 NUCLEO 四通道磁场。

首次运行时清理了残留的 simulation MoveIt/runtime 进程，并使用 `ROS_DOMAIN_ID=77` 连接真机。NUCLEO 在第 1 条前完成一次 SYNC，后续轨迹复用同一同步。两次首点静止门禁重试没有开始正式轨迹，未纳入结果；下表仅列成功的正式产物。

## 真机产物

每个目录均包含：

```text
execution_telemetry.csv   # 机器人实际关节/FK 时间序列
nucleo_telemetry.csv       # 与机器人 sync_time_ns 对齐的传感器表
nucleo_capture.yaml        # SYNC、解析质量和时间映射 manifest
```

| 序号 | 轨迹 | 真机产物目录 | playback-window 质量（incomplete / channel drops） |
|---:|---|---|---:|
| 1 | `mla_lm_vortex_center_z07_r10` | `/home/huaxin/Codes/ur-waypoint/runs/mla_lm_vortex_center_z07_r10__20260913_013250__20260913_153521/real/` | 7 / 7 |
| 2 | `mla_lm_vortex_center_z09_r10` | `/home/huaxin/Codes/ur-waypoint/runs/mla_lm_vortex_center_z09_r10__20260913_013245__20260913_153713/real/` | 3 / 3 |
| 3 | `mla_lm_vortex_center_z11_r10` | `/home/huaxin/Codes/ur-waypoint/runs/mla_lm_vortex_center_z11_r10__20260913_013605__20260913_153848/real/` | 7 / 7 |
| 4 | `mla_lm_vortex_center_z13_r10` | `/home/huaxin/Codes/ur-waypoint/runs/mla_lm_vortex_center_z13_r10__20260913_013615__20260913_154151/real/` | 1 / 1 |
| 5 | `mla_lm_vortex_center_z15_r10` | `/home/huaxin/Codes/ur-waypoint/runs/mla_lm_vortex_center_z15_r10__20260913_013914__20260913_154428/real/` | 4 / 4 |
| 6 | `mla_lm_helix_center_z07_to_z15_r10` | `/home/huaxin/Codes/ur-waypoint/runs/mla_lm_helix_center_z07_to_z15_r10__20260913_014326__20260913_154601/real/` | 6 / 6 |

所有 6 条机器人 playback 均返回 `0`；正式窗口内 `malformed_packets=0`，传感器帧映射量约 5993--6005，最大机器人—传感器时间偏差约 4.15--6.32 ms。原始传感器表和机器人表的 manifest 哈希均已核对。

批次摘要（由于中途静止门禁重试，分为三个批次目录）：

```text
/home/huaxin/Codes/ur-waypoint/runs/mla_short_lm_vortex_real_20260913T073459Z/summary.csv
/home/huaxin/Codes/ur-waypoint/runs/mla_short_lm_vortex_real_20260913T074131Z/summary.csv
/home/huaxin/Codes/ur-waypoint/runs/mla_short_lm_vortex_real_20260913T074406Z/summary.csv
```

## 对应背景场

六条轨迹完成后，机械臂移开，在不启动 UR 控制、不发送运动命令的条件下采集了 30 s 空场。目录：

```text
/home/huaxin/Codes/ur-waypoint/runs/empty_field_30s_after_mla_short_lm_vortex_real_20260913__20260913T075613.255742Z__55385c9b/
```

文件：

```text
nucleo_raw_telemetry.csv
nucleo_capture.yaml
sha256sums.txt
```

背景场结果：

- capture clock 时长约 `30.107 s`；
- `2998` 个四通道帧；
- `robot_motion_commanded=false`；
- 质量标记：4 incomplete groups、1 timeout group、18 channel sequence drops；
- `malformed_packets=0`；
- manifest 状态：`complete_with_quality_flags`；
- SHA-256：

```text
4e2e72fa46126545e51f91ab0e71840bfd20ab136de2f1baac3b4ee569a8a806
```

`sha256sum -c sha256sums.txt` 已通过。

## 使用建议

上述空场目录是本批次 6 条轨迹的对应背景参考。后续背景扣除或统计分析应生成新的派生目录，保留各 `real/` 目录、空场原始 CSV 和 manifest 不变。正式实验结束后控制器已恢复为 `scaled_joint_trajectory_controller`，临时 real runtime/cuRobo 已停止。

