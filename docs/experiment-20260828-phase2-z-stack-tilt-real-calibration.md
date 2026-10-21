# 2026-08-28 Phase-2 Z-stack + tilt 真机标定实验交付说明

## 1. 当前结论

本次实验已经完成两条真实 UR5 轨迹及同步 NUCLEO 原始磁场采集：

```text
center：阵列中心
x50：   plane1 X=+50 mm
```

两条真机轨迹均成功执行，机械臂全过程 `speed_scaling=1.0`，并保存了实际
JointState/FK telemetry 和机器人 authoritative playback window 内的完整 64 节点
三轴原始磁场，即每个完整 NUCLEO group 包含 192 个磁场通道。

当前已经得到的是：

1. 经过 GPU、碰撞和动力学验证的权威轨迹；
2. validated recording；
3. 真机实际 FK/关节数据；
4. 与机器人共同主机单调时钟对齐的完整磁场数据；
5. 数据完整性和同步质量报告。

当前**尚未生成最终数值 `b,o` 标定结果**。原因是本次落盘数据是原始 NUCLEO
磁场与机器人 FK；还需要使用冻结的磁定位 tracker 将原始磁场离线转换为
`p_tracker`，随后才能拟合：

```text
p_tracker - p_rod_tip = b + R_plane1_from_rod_tip * o
```

因此，不应把“真机采集完成”写成“`b,o` 数值拟合已完成”。

## 2. 环境与坐标合同

实验环境 ID：

```text
new_board_4cm_rod33365_up50_icassp_phase2
```

坐标和碰撞值：

```text
当前传感器面：             plane1 z=-0.007 m
板上方 60 mm 实体 keepout：z=-0.067 m
额外 5 mm 净空边界：       z=-0.072 m
有效杆尖最低距离：          65 mm
传感器板 XY：               [-0.20,+0.20] m × [-0.20,+0.20] m
杆长/半径：                 0.33365 m / 0.0025 m
```

新环境只改变传感器碰撞体和对应的传感器面/分析边界；墙、顶部挡板、地面、
基座支撑、`plane1` TF、UR5 和杆具配置均保持不变。

Mac 环境目录：

```text
/Users/huaxin/Codes/ur-waypoint/
  environments/new_board_4cm_rod33365_up50_icassp_phase2/
```

Ubuntu24 环境目录：

```text
/home/huaxin/Codes/ur-waypoint/
  environments/new_board_4cm_rod33365_up50_icassp_phase2/
```

关键环境文件：

```text
environment.yaml   当前板面、up50 平移、板尺寸和 workspace metadata
zones.yaml         MoveIt/cuRobo 传感器 keepout、墙、地面及分析边界
setup.yaml         plane1 Feature、333.65 mm 杆和 rod_tip
planner.yaml       固定时间、碰撞净空和禁止重定时配置
recording.yaml     recording 和 prepare 合同
controllers.yaml   仿真/真机控制器合同
```

## 3. 标定程序路径

### 3.1 轨迹生成和数学拟合核心

Mac：

```text
/Users/huaxin/Codes/ur-waypoint/
  src/hx_waypoint_tools/hx_waypoint_tools/z_stack_calibration.py
```

Ubuntu24：

```text
/home/huaxin/Codes/ur-waypoint/
  src/hx_waypoint_tools/hx_waypoint_tools/z_stack_calibration.py
```

该模块提供：

```text
build_z_stack_tilt_experiments()
  生成 center 和 X+50 两条不可变的 Z-stack + tilt authority

generate_z_stack_tilt_experiments()
  写出轨迹和采集窗口；不同内容存在时拒绝覆盖

rigid_offset_design_matrix()
  构造 d=b+Ro 的设计矩阵并验证可辨识秩

fit_rigid_offsets()
  使用实际 FK 旋转矩阵和 tracker-rod_tip 差值拟合 b、o；
  非 proper rotation 或 rank<6 时直接报错
```

### 3.2 生成命令入口

Mac：

```text
/Users/huaxin/Codes/ur-waypoint/
  scripts/generate_phase2_z_stack_tilt_calibration.py
```

Ubuntu24：

```text
/home/huaxin/Codes/ur-waypoint/
  scripts/generate_phase2_z_stack_tilt_calibration.py
```

生成命令：

```bash
cd /home/huaxin/Codes/ur-waypoint
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash

python scripts/generate_phase2_z_stack_tilt_calibration.py
```

### 3.3 相关碰撞验证实现

```text
src/hx_waypoint_curobo/hx_waypoint_curobo/environment_builder.py
  fail-closed 校验 up50 板面、keepout、墙/地面及环境 ID

src/hx_waypoint_curobo/hx_waypoint_curobo/contact_policy.py
  校验数学 rod_tip、末端球、其余杆球和机器人球的平面净空

src/hx_waypoint_tools/hx_waypoint_tools/planning_scene_publisher.py
  向 MoveIt/RViz 发布与 cuRobo 相同的碰撞体和非碰撞标记
```

### 3.4 回归测试

```text
tests/test_z_stack_calibration.py
tests/test_up50_phase2_environment.py
tests/test_contact_policy.py
```

Ubuntu24 当前与本实验相关的最终回归为：

```text
141 passed
```

## 4. 实验设计

每个 XY 位置运行一次：

```text
68 → 76 → 84 → 92 → 84 → 76 → 68 mm
```

对应 `plane1` 杆尖 Z：

```text
-75, -83, -91, -99, -91, -83, -75 mm
```

在最深的 92 mm 处执行：

```text
normal
→ X +25° → normal
→ X -25° → normal
→ Y +25° → normal
→ Y -25° → normal
```

时间合同：

```text
Z 平移：       1.5 s minimum-jerk
倾斜/回正：    4.5 s minimum-jerk
每个 visit：   1 s settle + 3 s 固定采集窗
每条总时长：   105 s
每条采集窗：   15 个
每条源点：     931 个
```

倾斜姿态集合的 `d=b+Ro` 设计矩阵 rank 为 6，nominal condition number
小于 10，可以独立识别三维 `b` 和 `o`。拟合必须使用实际 FK 的 `R_i`，不能用
命令 quaternion 代替。

## 5. 权威轨迹及采集窗口

当前唯一可执行版本为 `v3`：

```text
environments/new_board_4cm_rod33365_up50_icassp_phase2/trajectories/
  z_stack_tilt_calibration_center_up50_phase2_v3.csv
  z_stack_tilt_calibration_x50_up50_phase2_v3.csv
  z_stack_tilt_calibration_capture_windows_v3.csv
```

SHA-256：

```text
center v3
890d31d6a748445015a66d7d7a42adc589dafa14d8d7682b236867cf7ec90bcf

x50 v3
ed62863bdcd69c8c87c78d42933799a75e324e886b1e2bc7ca1f5339aa7a6729

capture windows v3
8e8f292aa6406a40fa5ecca5100b95baa422f09dd42127f90a7e44fac92f6959
```

同目录的无版本文件和 `_v2` 文件是明确保留的失败候选：

```text
v1：30°/1.5 s，动力学失败
v2：30°/4.5 s，最小碰撞净空 4.837958 mm < 5 mm
```

它们禁止 GPU build、禁止 playback、禁止用于数据分析。

## 6. Validated recording 路径

Ubuntu24：

```text
/home/huaxin/Codes/ur-waypoint/
  environments/new_board_4cm_rod33365_up50_icassp_phase2/recordings/
    z_stack_tilt_calibration_center_up50_phase2_v3__20260828_201329/
    z_stack_tilt_calibration_x50_up50_phase2_v3__20260828_201834/
```

两个 recording 都满足：

```text
status=validated
duration_s=105
point_count=10501
source_waypoint_count=931
matched_source_waypoint_count=931
max_source_time_error_s=0
uniform_time_scale=1
minimum_collision_distance_m=0.005781898275017738
scene/self/bound collision cost=0
```

Recording 索引：

```text
Mac:
/Users/huaxin/Codes/ur-waypoint/
  docs/evidence/phase2-z-stack-tilt-20260828/recording_index.csv

Ubuntu24:
/home/huaxin/Codes/ur-waypoint/
  docs/evidence/phase2-z-stack-tilt-20260828/recording_index.csv
```

## 7. 本次真机数据路径

### 7.1 Center

Run ID：

```text
z_stack_tilt_calibration_center_up50_phase2_v3__20260828_201329__20260828_205450
```

Ubuntu24 原始目录：

```text
/home/huaxin/Codes/ur-waypoint/runs/
  z_stack_tilt_calibration_center_up50_phase2_v3__20260828_201329__20260828_205450/real/
```

文件：

```text
execution_telemetry.csv   实际机器人 JointState/FK、trajectory_time、速度和 scaling
nucleo_telemetry.csv      与机器人行映射后的完整 192 通道磁场
nucleo_capture.yaml       原始采集、解析器、覆盖率、同步和质量 manifest
```

### 7.2 X+50 mm

Run ID：

```text
z_stack_tilt_calibration_x50_up50_phase2_v3__20260828_201834__20260828_205814
```

Ubuntu24 原始目录：

```text
/home/huaxin/Codes/ur-waypoint/runs/
  z_stack_tilt_calibration_x50_up50_phase2_v3__20260828_201834__20260828_205814/real/
```

文件名与 center 相同。

### 7.3 `/data` 耐久归档

两条完整真机数据、命令日志和哈希表已复制到：

```text
/data/ur-waypoint/icassp2027_phase2/evidence/
  real_z_stack_tilt_20260828/
```

目录内：

```text
z_stack_tilt_calibration_center_...__20260828_205450/
z_stack_tilt_calibration_x50_...__20260828_205814/
z_stack_center_v3_real_nucleo.log
z_stack_x50_v3_real_nucleo.log
SHA256SUMS
```

该归档约 85 MB，位于 `/data` 的 1.6 TB 可用空间中。

## 8. 数据字段与关联方式

### `execution_telemetry.csv`

关键字段：

```text
sync_time_ns
trajectory_time_s / trajectory_time_ns
state
desired_<joint>_pos
<joint>_pos / <joint>_vel
tip_x / tip_y / tip_z
tip_vx / tip_vy / tip_vz
actual_tip_speed_mps
tip_wx / tip_wy / tip_wz
actual_tip_angular_speed_radps
position_error_m
speed_scaling
```

### `nucleo_telemetry.csv`

关联键：

```text
sync_time_ns
trajectory_time_s
robot_row_index
sensor_rx_sync_time_ns
sync_offset_ns
```

磁场列：

```text
sensor_00_x_ut ... sensor_63_z_ut
```

共 64 个节点、192 个磁场标量。`robot_row_index` 指向
`execution_telemetry.csv` 中对应的最近实际机器人样本；映射使用同一 Ubuntu24
host monotonic clock 的最近邻，同距时选择更早机器人样本，不进行时间插值。

## 9. 真机采集质量结果

| 位置 | Robot rows | Robot 窗口内完整 192 通道组 | Incomplete | Channel sequence drops | 最大映射残差 | Speed scaling |
|---|---:|---:|---:|---:|---:|---:|
| center | 13,132 | 10,505 | 0 | 0 | 6.279178 ms | 1.0–1.0 |
| X+50 | 13,126 | 10,500 | 6 | 31 | 5.406735 ms | 1.0–1.0 |

Center 为完全无丢组的四通道采集。X+50 的 31 个 sequence drops 相当于约
`7.92/3000` 组，6 个 incomplete groups 相当于约 `1.53/3000` 组；原始异常计数
被完整保留，没有插值或隐藏。两个 manifest 均为 `status: complete`，无 malformed
packet。

真机运行索引及精确 SHA-256：

```text
Mac:
/Users/huaxin/Codes/ur-waypoint/
  docs/evidence/phase2-z-stack-tilt-20260828/real_run_index.csv

Ubuntu24:
/home/huaxin/Codes/ur-waypoint/
  docs/evidence/phase2-z-stack-tilt-20260828/real_run_index.csv
```

## 10. 当前结果与后续数值结果路径

当前已经存在的结果：

```text
采集/QC/哈希结果：
/data/ur-waypoint/icassp2027_phase2/evidence/real_z_stack_tilt_20260828/

轨迹与 mock validation 结果：
/data/ur-waypoint/icassp2027_phase2/evidence/
```

最终 tracker/FK 映射与 `b,o` 数值拟合建议固定输出到：

```text
/data/ur-waypoint/icassp2027_phase2/analysis/z_stack_tilt_20260828/
  mapped_tracker_fk.csv
  window_qc.csv
  rigid_offset_fit_center.yaml
  rigid_offset_fit_x50.yaml
  rigid_offset_fit_joint.yaml
  residual_vs_distance.csv
  calibration_report.md
  SHA256SUMS
```

这些最终拟合文件当前尚不存在；只有在 tracker 使用冻结模型将原始 NUCLEO
转换为 `p_tracker` 后才能生成。

## 11. 离线拟合流程

1. 从 `z_stack_tilt_calibration_capture_windows_v3.csv` 读取 30 个权威 3 s 窗口；
2. 只选择窗口内完整的 192 通道 NUCLEO groups；
3. 使用冻结 tracker 输出每组 `p_tracker`；
4. 通过 `robot_row_index` 取得同组实际机器人关节/FK；
5. 将实际 FK 转换到 `plane1`，得到 `p_rod_tip` 和
   `R_plane1_from_rod_tip`；
6. 计算 `d_i=p_tracker_i-p_rod_tip_i`；
7. center、X+50 分别调用 `fit_rigid_offsets()`；
8. 再做共享 `b,o` 的 joint fit；
9. 用 Z-stack 往返窗口检查残差是否随距离、方向或时间漂移；
10. 保存数值、残差、输入哈希和完整失败信息。

如果实际姿态矩阵 rank 小于 6、不是 proper rotation、窗口未停稳、tracker 输出缺失
或残差显示明显距离依赖，应直接报错，不得通过重定时、裁剪窗口、补点或自动改轨迹
修复。

## 12. 实际执行命令记录

Center 是本次 NUCLEO boot 的唯一 SYNC：

```bash
cd /home/huaxin/Codes/ur-waypoint
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source .venv/bin/activate
export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=0

ros2 run hx_waypoint_tools tipdraw \
  --workspace /home/huaxin/Codes/ur-waypoint \
  play \
  --env new_board_4cm_rod33365_up50_icassp_phase2 \
  --recording z_stack_tilt_calibration_center_up50_phase2_v3__20260828_201329 \
  --record-nucleo \
  --nucleo-port /dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_3879336C3333-if00 \
  --nucleo-baudrate 115200 \
  --nucleo-max-incomplete-groups 40 \
  --nucleo-max-channel-sequence-drops 40
```

X+50 复用同一 SYNC，没有再次握手：

```bash
ros2 run hx_waypoint_tools tipdraw \
  --workspace /home/huaxin/Codes/ur-waypoint \
  play \
  --env new_board_4cm_rod33365_up50_icassp_phase2 \
  --recording z_stack_tilt_calibration_x50_up50_phase2_v3__20260828_201834 \
  --record-nucleo \
  --nucleo-reuse-sync \
  --nucleo-port /dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_3879336C3333-if00 \
  --nucleo-baudrate 115200 \
  --nucleo-max-incomplete-groups 40 \
  --nucleo-max-channel-sequence-drops 40
```

NUCLEO 每次重新上电后只允许一次正常 SYNC；同一 boot 内的后续轨迹必须
`--nucleo-reuse-sync`。

## 13. 相关文档索引

```text
docs/phase2-z-stack-tilt-calibration.md
  轨迹设计、数学可辨识性、生成方法和离线拟合协议

docs/evidence/phase2-z-stack-tilt-20260828/README.md
  v1/v2 失败证据、v3 mock/真机验证摘要

docs/evidence/phase2-z-stack-tilt-20260828/recording_index.csv
  validated recording、轨迹和独立 mock playback 哈希

docs/evidence/phase2-z-stack-tilt-20260828/real_run_index.csv
  真机 run ID、NUCLEO/FK 质量计数和 SHA-256
```
