# `mla_short` 逐点采集、TCP 绕轴标定与空场记录

日期：2026-09-12  
设备：UR5 + 190 mm / 5 mm 杆 + NUCLEO 四通道磁传感器板  
远端主机：`ubuntu24`  
仓库：`/home/huaxin/Codes/ur-waypoint`

## 1. 实验目的与环境

本组实验在 `environment_id=mla_short` 中完成，目的是获得同一环境合同下的：

1. 静态空间网格的机器人实际 FK 与磁场数据；
2. TCP 绕杆轴旋转标定数据，用于检查杆尖/磁铁安装偏心与固定坐标偏差；
3. 机械臂移开后的 30 s 空场背景参考。

环境约定如下：

- `plane1` 原点为传感器阵列几何中心，横向单位为米；
- `mla_short` 使用 `rod_190mm_d5mm`，杆长 190 mm、直径 5 mm；
- `plane1` 的局部 `-Z` 为实际传感器板上方方向；静态网格的高度 7--15 cm 在 manifest 中以 `plane1_z_sign=negative` 记录；
- 网格采集使用固定竖直姿态四元数 `(0, 0, 0, 1)`；
- NUCLEO 数据使用 Ubuntu 主机单调时钟，并通过 `sync_time_ns` 与机器人表关联。

本次使用的 NUCLEO 设备路径为：

```text
/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_3879336C3333-if00
```

## 2. 数据总览

| 数据 | 用途 | 推荐目录 |
|---|---|---|
| 245 点逐点采集 | 静态空间网格、实际 FK 与磁场 | `/data/ur-waypoint/mla_short_vertical_grid_live/20260912T144235Z/` |
| TCP 绕轴标定 | 固定杆尖位置下的 `0°→180°→0°` 旋转 | `/home/huaxin/Codes/ur-waypoint/runs/mla_short_yaw_calibration_center_z070_yaw_180deg__20260912_232631__20260912_234844/real/` |
| 30 s 空场 | 背景场估计与后续背景扣除 | `/home/huaxin/Codes/ur-waypoint/runs/empty_field_30s_mla_short_after_yaw_recalibration_20260912__20260912T155353.651426Z__b42567e5/` |

这些目录均为追加生成的独立产物，没有覆盖此前实验。

## 3. 245 点静态网格

### 采集设计

- `x,y ∈ {-15,-10,-5,0,5,10,15} cm`；
- `z ∈ {7,9,11,13,15} cm`；
- 共 `7×7×5=245` 个位置；
- 每点停稳采集 3 s；
- 竖直姿态；
- 每个目标由 MoveIt 做碰撞检查、IK、规划和执行；
- 未使用 recording artifact，也没有运行轨迹缩放、重定时、裁剪或跳点。

### 结果

- `245/245` 点 `CAPTURED`；
- `UNREACHABLE=0`；
- 每点 NUCLEO 帧数 296--303；
- 最大传感器—机器人时间映射偏差约 8.739 ms；
- 结束时机器人停在 `plane1` 中心、15 cm 高度。

主要文件：

```text
point_results.csv
robot_actual_fk.csv
nucleo_raw_telemetry.csv
nucleo_mapped.csv
sensor_window_quality.csv
manifest.json
sha256sums.txt
```

总解析统计中有少量协议质量标记（85 incomplete groups、91 channel sequence drops、6 source sequence drops），但没有 malformed packet；每个点的窗口帧数和映射质量均满足采集程序的最低要求。原始表和映射表必须一起保留。

## 4. TCP 绕杆轴标定

### 权威输入与仿真 recording

权威轨迹 CSV：

```text
/home/huaxin/Codes/ur-waypoint/environments/mla_short/trajectories/mla_short_yaw_calibration_center_z070_yaw_180deg.csv
```

190 mm TCP 对应的 recording：

```text
/home/huaxin/Codes/ur-waypoint/environments/mla_short/recordings/mla_short_yaw_calibration_center_z070_yaw_180deg__20260912_232631/
```

标定动作包含：

- 6 s 小幅 warm-up wave，使 tracker/采集链路进入 tracking；
- 2 s 严格静止；
- TCP 绕杆轴 `0°→180°`，18 s；
- 反向 `180°→0°`，18 s；
- 末尾 2 s 静止；
- 总时长 46 s；杆尖固定在 `plane1=(0,0,-70 mm)`。

### 真机结果

NUCLEO 重启后重新 SYNC，推荐使用以下真机目录：

```text
/home/huaxin/Codes/ur-waypoint/runs/mla_short_yaw_calibration_center_z070_yaw_180deg__20260912_232631__20260912_234844/real/
```

文件：

```text
execution_telemetry.csv
nucleo_telemetry.csv
nucleo_capture.yaml
```

结果摘要：

- 机器人回放成功，TCP XYZ 在旋转段的漂移约 0.1 mm；
- speed scaling 全程为 1.0；
- 原始 NUCLEO 帧 4724，映射到机器人窗口 4596；
- 最大机器人—传感器时间偏差约 4.15 ms；
- playback 窗口内 8 个 incomplete groups、10 个 channel sequence drops；
- malformed packet 为 0；
- 原始 staging 文件仍保留在：
  `/home/huaxin/Codes/ur-waypoint/runs/nucleo_capture__20260912T154843.215240Z__bdacc179.staging.csv`。

该 run 的 `nucleo_capture.yaml` 会如实标记为 `incomplete`，这是协议质量标记，不代表机器人动作失败。本次数据在设定的轻微丢包容忍范围内，可用于旋转偏心分析。

一次 NUCLEO 重启前的重试目录曾出现 714 个 incomplete groups，目录为：

```text
/home/huaxin/Codes/ur-waypoint/runs/mla_short_yaw_calibration_center_z070_yaw_180deg__20260912_232631__20260912_234608/real/
```

该目录仅作故障证据，不作为正式标定输入。

## 5. 30 s 空场背景

机械臂已移开，使用 `capture_empty_field.py` 采集；没有启动 UR 控制，也没有发送机器人运动命令。NUCLEO 使用本次标定后仍有效的同步状态，未重复发送 SYNC。

目录：

```text
/home/huaxin/Codes/ur-waypoint/runs/empty_field_30s_mla_short_after_yaw_recalibration_20260912__20260912T155353.651426Z__b42567e5/
```

文件：

```text
nucleo_raw_telemetry.csv
nucleo_capture.yaml
sha256sums.txt
```

采集结果：

- 请求时长 30.0 s，实际 capture clock 约 30.107 s；
- 2995 个四通道完整组；
- 8 个 incomplete groups、1 个 timeout group、22 个 channel sequence drops；
- malformed packet 为 0；
- `robot_motion_commanded=false`；
- manifest 状态为 `complete_with_quality_flags`；
- 原始 CSV SHA-256：

```text
334e4e478108637e8a49da9c685afbc6a587362e5994032bb9743cc78920d90f
```

`sha256sum -c sha256sums.txt` 已通过。后续做背景扣除时，应读取此目录的原始表并按传感器通道/轴计算背景均值；不要改写原始 CSV 或 manifest。

## 6. 推荐使用方式与可追溯性

推荐将第 3 节的静态网格和第 4 节的标定 run 与第 5 节空场配对使用：

```text
静态网格：robot_actual_fk.csv + nucleo_mapped.csv
TCP 标定：execution_telemetry.csv + nucleo_telemetry.csv
背景参考：empty-field/nucleo_raw_telemetry.csv
```

机器人表和同步传感器表中的 `sync_time_ns` 来自同一台 Ubuntu24 主机；空场没有机器人运动，因此只保存原始传感器时间序列。任何背景扣除、滤波或拟合都应生成新的派生目录，并保留上述三组原始证据不变。

