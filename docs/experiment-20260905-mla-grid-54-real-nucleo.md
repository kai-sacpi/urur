# MLA 54 点真机与 NUCLEO 同步采集

## 结论

2026-09-05 在已确认与现实一致的 `mla` 环境中，54 个静态网格目标已依次完成一次真机执行和 NUCLEO 同步采集：

- 机器人执行：`54/54 SUCCEEDED`；
- 传感器完全无损：32 条；
- 传感器有界丢包、按既定门限接受：22 条；
- 被拒绝的采集：0 条；
- 第一条执行一次 NUCLEO SYNC，后续 53 条全部复用，未重复握手；
- 实验完成后，机械臂通过 MoveIt 碰撞检查返回环境 home。

本次执行没有缩放、重定时、裁剪、跳点或替换目标位置。每个正式 recording 的权威静止段为 3 s，54 条合计 162 s；包含回 home、到目标和规划开销的整批时间为 886.755 s（约 14 分 47 秒）。

## 执行方式

正式集合来自：

```text
/data/ur-waypoint/mla_grid_54/20260904T150316Z/build/selected_recordings.csv
```

执行顺序严格采用该索引的 1–54 顺序。每个目标执行：

1. 使用 MoveIt OMPL/RRTConnect 规划并执行当前状态到环境 home 的碰撞检查路径；
2. 使用 MoveIt 规划并执行 home 到 sealed recording 首关节状态的碰撞检查路径；
3. runtime 验证当前位置和静止状态；
4. 原样提交 3 s sealed recording；
5. 同时记录机器人实际状态和 NUCLEO 原始四通道数据；
6. 将两张表按共同的 monotonic `sync_time_ns` 做最近邻映射。

运行脚本：

```text
scripts/run_mla_grid_54_real_nucleo.sh
```

审计脚本：

```text
scripts/audit_mla_grid_54_real.py
```

## 机器人结果

- 54 条 controller playback 全部成功；
- 每条机器人实际 telemetry 为 375–381 行；
- `speed_scaling=1.0`；
- 全局最大 TCP 位置跟踪误差为 `0.174757 mm`；
- 54 条轨迹的实际 FK、共同时间戳和控制器状态均已保留；
- 最后一条完成后额外执行了碰撞检查的 return-home，证据为 `final_return_home.log`。

## 传感器结果

每个映射后的传感器帧均包含 64 个三轴节点，即 192 个磁场通道，并与机器人表共享：

```text
sync_time_ns
trajectory_time_s
robot_ros_time_ns
robot_row_index
```

质量汇总：

| 指标 | 全部 54 条合计 | 单条最大值 |
|---|---:|---:|
| incomplete groups | 25 | 2 |
| channel sequence drops | 44 | 3 |
| duplicate packets | 26 | 3 |
| reordered packets | 6 | 2 |

- 每条得到 296–306 个映射传感器帧；
- 最大机器人/传感器时间映射偏差为 `4.215221 ms`，低于 50 ms 门限；
- 所有 capture 的 `capture_error` 均为空；
- 22 条 `incomplete` 都属于操作者此前明确接受的少量丢包范围；
- 没有 malformed packet、unexpected channel、source sequence drop 或映射超限导致的拒绝项。

NUCLEO 当前已经完成过握手。如果不复位板子，后续采集必须使用 `--nucleo-reuse-sync`；只有板子复位后，下一次采集才重新执行普通 SYNC。

## 结果位置

Ubuntu24 的完整、持久化结果：

```text
/data/ur-waypoint/mla_grid_54_real/20260904T171229Z/
```

关键文件：

```text
audit_summary.yaml
final_artifact_index.csv
execution_schedule.csv
sensor_quality.csv
batch.log
final_return_home.log
SHA256SUMS
artifacts/<run_id>/real/execution_telemetry.csv
artifacts/<run_id>/real/nucleo_telemetry.csv
artifacts/<run_id>/real/nucleo_capture.yaml
```

其中 `final_artifact_index.csv` 是正式入口：每行对应一个目标，包含 recording ID、机器人和传感器状态、丢包计数、时间映射偏差、持久化文件路径及 SHA-256。

完整归档：

```text
/data/ur-waypoint/mla_grid_54_real/20260904T171229Z.tar.gz
/data/ur-waypoint/mla_grid_54_real/20260904T171229Z.tar.gz.sha256
```

归档 SHA-256：

```text
8fc4b4c32509ec140ec5cf49ba0f18b585597c7f34d4a328bb5da1cce03631e6
```

Mac 同步副本：

```text
/Users/huaxin/Codes/ur-waypoint/artifacts/mla_grid_54_real_20260904T171229Z/
```

Mac 上保存了完整压缩归档、最终索引、审计摘要、schedule、质量表和日志；压缩归档已通过 SHA-256 校验。
