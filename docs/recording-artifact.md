# Recording artifact 与安全门禁

Recording 是仿真完整成功后才发布的不可变执行包。`runs/` 中的编译结果、未完成的 staging 目录和普通 CSV 都不具备真机播放资格。

## 构建状态机

```text
VALIDATE_SOURCE
  → CUROBO_COMPILE
  → VALIDATE_COMPILED_TRAJECTORY
  → CREATE_STAGING
  → WAITING_START_STATE
  ├─ [已在起点] ─────────────────────────┐
  └─ [仅位置不匹配且策略允许] → PREPARE_PLANNING      │
                                      → PREPARING           │  # 独立 goal，recorder 未启动
                                      → PREPARE_SETTLING ───┘  # 复核 source t=0
  → CANONICAL_GOAL_ACCEPTED   # 从 source t=0 开始的 draw/hold goal
  → EXECUTING_AND_RECORDING
  → VALIDATE_SIMULATION
  → WRITE_MANIFEST_AND_SHA256
  → ATOMIC_RENAME
  → status: validated
```

cuRobo 离线编译和可选的起点 `prepare` 都在记录器启动之前完成。`prepare` 是 recording 之外的独立 cuRobo/controller goal，不拼接到 `joint_trajectory.csv`，也不产生 recording telemetry。只有 canonical goal 被控制器接受后 recorder 才启动。程序启动、文件读取、GPU warmup、cuRobo 解算、prepare、Action server 等待和 RViz 启动时间都不能进入轨迹时间轴。

`BuildRecording` 不接受任意外部 joint CSV。输入必须位于工作区受控的 `runs/<trajectory>__compiled__*` 直属目录；同目录 `compile_report.yaml` 必须记录 source SHA-256、完整环境配置/mesh 哈希映射、cuRobo commit、源/输出点数与时长、权威时间轴一致性、动力学比例、整轨 joint-winding policy/offset，以及逐采样 tip FK、drawing-plane contact、最小碰撞距离、scene/self/bound feasibility cost。runtime 会把报告中的环境哈希集合与当前环境做严格等集合比较：缺项、额外项或任一值不一致都拒绝。因此在编译后修改 `planner.yaml`、TCP、标定、碰撞模型或 mesh，旧 run 都不能继续构建 recording。`checked_sample_count` 必须覆盖所有关节轨迹点；三类 feasibility cost 必须不大于 `1e-7`，tip 误差、墙面接触和碰撞距离必须满足当前环境阈值。

源 waypoint 和通过“十进制 → ROS 整数纳秒 → binary64 → 同一整数纳秒”无损往返检查的 `time_s` 是权威输入。CSV/YAML 中的 `*_s` 为十进制秒展示，但 source/joint 时间、持续时间和逐点匹配都以 ROS 纳秒判定。canonical `joint_trajectory.csv` 必须只含 `draw/hold`，第一点精确为 source `t=0`，最后一点精确为源末时刻，且每个 source waypoint 的纳秒时戳和 phase 都必须在输出中逐点存在。

编译与 recording 构建只能验证，不得缩放、重定时、裁剪、改点或跳点；点数、纳秒时间戳、phase 或源 Pose 一致性检查失败时必须终止，不得修补 artifact 后继续。上游负责提供不过快、加速度与 jerk 合理且动力学可行的权威时标轨迹；任一 IK、碰撞、关节界限或动力学比例超限时直接报错。

## 目录契约

```text
<recording_id>/
├── manifest.yaml
├── source_tip_trajectory.csv
├── joint_trajectory.csv
├── compile_report.yaml
├── joint_frames.csv
├── sim_controller_state.csv
├── sim_actual_joint.csv
├── sim_actual_tip.csv
├── validation_report.yaml
├── environment_snapshot/
│   ├── environment.yaml
│   ├── setup.yaml
│   ├── zones.yaml
│   ├── planner.yaml
│   ├── recording.yaml
│   ├── controllers.yaml
│   └── robot/
│       ├── robot_model.yaml
│       ├── calibrated_kinematics.yaml
│       ├── curobo_robot.yaml
│       └── collision_spheres.yaml
└── sha256sums.txt
```

必需文件缺失或为空时，构建必须失败；不能创建空 CSV 占位后标记成功。环境中存在 meshes 时一并进入 snapshot 和 hash。

## 文件用途

| 文件 | 用途 | 真机控制输入 |
|---|---|---|
| `manifest.yaml` | schema、状态、机器人、点数、持续时间、验证指标和来源 hash | 校验用 |
| `source_tip_trajectory.csv` | 原始杆尖 Pose 来源追踪 | 否 |
| `joint_trajectory.csv` | cuRobo 输出的 `q/qd/qdd/time/phase` | **是** |
| `compile_report.yaml` | cuRobo 版本、source/hash、ROS 纳秒时间一致性、动力学、tip FK 和碰撞验证证据 | 校验用 |
| `joint_frames.csv` | 同一机器人模型 FK 得到的各关节 frame | 否 |
| `sim_controller_state.csv` | 仿真控制器 desired/actual/error | 否 |
| `sim_actual_joint.csv` | mock hardware 的实际关节状态 | 否 |
| `sim_actual_tip.csv` | 仿真杆尖 Pose/速度 | 否 |
| `validation_report.yaml` | 仿真完成、跟踪误差、碰撞距离和限制比例等结论 | 校验用 |
| `environment_snapshot/` | 编译和仿真时使用的完整环境配置 | 一致性校验 |
| `sha256sums.txt` | artifact 内容完整性 | 校验用 |

`joint_trajectory.csv` 的位置、速度和加速度必须来自同一条优化轨迹/同一导数体系。ros2_control 对同时具有 `q/qd/qdd` 的点使用五次 spline，避免只记录位置导致 waypoint 处速度不连续。

`source_tip_trajectory.csv` 由编译 run 逐字节复制进 recording；`joint_trajectory.csv` 也是 runtime 实际解析并一次性下发的 canonical payload。两者都只允许 `draw/hold`。任何起点 `prepare` 都是另一个短寿命 goal，不会改写、拼接或裁剪 canonical payload。

仿真成功返回后还必须对 `sim_actual_joint.csv` 和 `sim_controller_state.csv` 做数值门禁，不能只记录布尔状态。当前环境要求：

```yaml
recording:
  sample_dt_s: 0.010
  validation:
    max_missing_samples: 0
    max_time_jitter_s: 0.002
    max_quaternion_norm_error: 0.001
    require_action_success: true
    max_completion_overrun_s: 0.05
    max_controller_reference_position_error_rad: 0.02
    max_controller_reference_velocity_error_rad_s: 0.10
    max_joint_position_tracking_error_rad: 0.02
    max_joint_velocity_tracking_error_rad_s: 0.10
```

验证还会逐个检查 `joint_frames.csv` 中每个 frame 和 `sim_actual_tip.csv`：ROS/轨迹时间必须严格递增并覆盖完整命令，不能缺样或超出抖动阈值，所有数值必须有限，四元数范数误差不得超过配置。录制结束时间不得超过 canonical 时长加 `max_completion_overrun_s`，并在每个 canonical 控制点附近把 controller reference 与原始完整 goal 的 `q/qd` 对齐比较。`sim_controller_state.csv` 的 speed scaling 合同值必须全程为 `1.0`（容差 `1e-9`）。这里的 `1.0` 是精确锁定标准未缩放 JTC 后的控制器合同值，不是直接抄录 `JointTrajectoryControllerState.speed_scaling_factor`；当前未配置 scaling interface 的 Jazzy JTC 会让该新增消息字段保持默认 `0`，而这个默认值不是有效执行比例。

验证报告保存 actual/controller/frame/tip 各路的样本数、首末轨迹时间、覆盖时长、最大采样间隔、最大时间抖动、估算缺失样本数、四元数误差、speed-scaling 合同误差、controller-reference/canonical 误差，以及最大关节位置/速度跟踪误差。任何指标越过配置阈值，staging recording 都不能原子发布为 `status: validated`。编译报告中的权威时间轴一致性结论和速度、加速度、jerk 比例也会写入 validation 与 manifest；任一动力学比例超限都使构建失败。

## `inspect` 门禁

`ros2 run hx_waypoint_tools tipdraw inspect` 至少验证：

- `schema_version` 可识别；
- `status` 精确为 `validated`；
- 必需文件齐全；
- `sha256sums.txt` 与实际文件一致；
- manifest 中的 joint names 可用且顺序明确；
- `point_count` 与 `joint_trajectory.csv` 一致；
- `duration_s` 与最后一个轨迹时间一致；
- `compile_report.yaml` 存在、非空且在 checksum 封存范围内；
- `uniform_time_scale` 精确为 `1.0`，且每个 source waypoint 的 ROS 纳秒时戳/phase 在 canonical 轨迹中精确存在。

任何失败都返回非零状态，artifact 不得进入真机流程。

## 真机播放门禁

runtime 在发送完整 `FollowJointTrajectory` goal 之前还要验证：

1. 请求中的 `environment_id`、`recording_id` 和路径是安全、受控的目录。
2. recording 的环境 ID、机器人型号和六关节名称/顺序与当前配置一致。
3. 当前 `environment.yaml`、权威 `setup.yaml`（原始示教器位姿与杆具）、zones、planner、recording、controllers、机器人模型、标定和碰撞球文件 hash 与 recording 一致。
4. `calibrated_kinematics.yaml` 明确为 `deployment_allowed: true`；缺字段、解析失败或 `false` 都拒绝。
5. 当前 JointState 新鲜，包含完整且不重复的关节名、position 和 velocity，所有值有限。
6. 当前机械臂已经静止，且最大关节起点误差严格小于配置阈值。
7. 实时 `base → plane1`、alias 和 `tool0 → rod_tip` TF 与 recording 中封存的 `setup.yaml` 一致；runtime 在加载后和正式 goal 发送边界都会复核。
8. 当前环境锁定的未缩放 Action server 存在；不存在时拒绝，不尝试备用 controller。
9. runtime 在正式 goal 发送前收到一条调用后新到达的 `speed_scaling=1.0`；缺少、非有限或偏离 `1e-9` 容差均拒绝。

真机模式还禁止用通用 `action_name` 覆盖 `real_action_name`。最终名称必须与当前环境
`controllers.yaml` 完全一致，并且只能是标准未缩放 JTC 或 passthrough 的受控端点；
`new_board_4cm_rod33365` 明确锁定 `/joint_trajectory_controller/follow_joint_trajectory`。
仿真 Action 和 controller state 仍必须精确来自标准 JTC；参数覆盖、命名空间替代，
以及 Action 五个底层传输端点或 state topic 的 ROS remap 均会使 runtime 启动失败。

当前示例环境的起点阈值为：

```yaml
start_state:
  max_joint_error_rad: 0.02
  settle_velocity_rad_s: 0.01
  mismatch_policy: curobo_prepare
```

canonical recording 播放本身永远不调用 cuRobo。不传 `--auto-prepare` 时，起点不匹配直接拒绝。传入 `--auto-prepare` 时，runtime 只允许处理“JointState 新鲜、机械臂已静止、仅位置不匹配”的情况：它请求一条独立 cuRobo prepare 轨迹，验证起终点、完整 `q/qd/qdd`、ROS 纳秒时间、持续时间和动力学约束，以单独 controller goal 执行；成功后还必须重新等待 source `t=0` 处的新鲜静止 JointState。整个 prepare 阶段 recorder 未启动，不能绕过任何 recording 或起点门禁。

批量真机工作流也可显式传 `--externally-prepared-start`。该模式与 `--auto-prepare` 互斥，调用方
必须先用碰撞检查的 MoveIt 依次到达环境 home 和 sealed recording 的首关节点。runtime 不信任
调用方报告的当前位置，而是重新读取实时 JointState，要求位置命中 source `t=0`、速度低于静止
阈值且状态新鲜；任一条件不满足即拒绝。门禁通过后直接发送密封 recording，不请求 cuRobo，
也不改变 canonical `q/qd/qdd/time_from_start`。

## 真机 telemetry

控制器接受完整轨迹后才开始记录 actual telemetry。目标字段包括：

```text
ros_time_s
sync_time_ns
ros_time_ns
trajectory_time_s
trajectory_time_ns
actual q / qd
actual tip pose / linear velocity / angular velocity
desired tip speed
tracking error
speed scaling
execution state
```

recording 中的时间和速度是权威标称值。`speed_scaling` 仅作为执行证据记录，不能用于推进时间：telemetry 的 `trajectory_time_s`/`trajectory_time_ns` 只由 controller feedback 更新。正式播放中任何 speed scaling 偏离 `1.0` 都会立即取消完整 controller goal 并报错。`ros_time_ns` 是 ROS clock 的整数纳秒，`ros_time_s` 是其十进制秒展示；`sync_time_ns` 是同一 Ubuntu 主机上的 `CLOCK_MONOTONIC` 整数纳秒，用于把独立进程采集的数据精确映射到某个实际机械臂采样。canonical 轨迹时间的解析、端点和逐点一致性判定仍使用 ROS 纳秒。prepare 期间不会生成该文件的任何记录行。

## 可选 NUCLEO 同步表

`tipdraw play --record-nucleo` 会在 controller goal 发送前打开串口、执行 NUCLEO
SYNC/ACK，并等待首个完整 L5 superframe。预检失败时机械臂不会开始运动。采集采用
hospital 分支 `b80f4ebee945c6ac07be7732621fa8cd1dc056cd` 的 wire protocol 独立实现，
协议来源固定在仓库根目录 `nucleo.lock`。

正式执行结束后，同一 run 目录包含：

```text
execution_telemetry.csv
nucleo_telemetry.csv
nucleo_capture.yaml
```

传感器每个完整 frame 保留串口 read 完成时的 `sensor_rx_sync_time_ns`、capture 进程记录的
ROS 时间、设备
`src_ts_ms`、NUCLEO/channel sequence 和 64×XYZ 的 µT 原始量。其 `sync_time_ns`
不是伪造的设备 epoch，而是按 host monotonic 接收时间映射到最近的真实机械臂 telemetry
行，并从机械臂表逐字复制，因此两表可直接按 `sync_time_ns` 关联；映射偏差另存为
`sync_offset_ns`。该过程不会插值、重采样或修改 `joint_trajectory.csv`、waypoint 或
`time_s`。

NUCLEO 是非安全关键旁路。运行中串口异常不得触发 controller cancel，因为那会裁剪权威
轨迹；机械臂继续完成整条轨迹，传感器 manifest 标记为 `incomplete` 并保留 staging
证据。NUCLEO 表属于每次 playback 的执行证据，不进入 canonical recording 的环境 hash
或 `sha256sums.txt`，因此启用或禁用传感器不会改变已验证 recording。

`complete` 还要求正式 playback 窗口内 parser 完整性计数无增量、连续传感器帧空洞不超过
0.5 s、最近机械臂采样的绝对映射偏差不超过 50 ms。任一门禁失败只降级传感器证据，不会
触发轨迹缩放、重定时、裁剪、跳点或 controller cancel。manifest 先以
`staging_raw_csv.removed=false` 原子提交，raw staging 删除并 fsync 目录后才更新为 `true`。

## 可选 Tracker yaw 标定表

`tipdraw yaw-calibrate` 消费一条已经验证的组合 recording；其 source 在同一个不可修改的
时间轴内包含 warmup Wave、中心静止、yaw 正反往返和末尾 hold。这样可以避免
`home_then_recording_start` 在两次 playback 之间插入回起始位动作。Tracker capture 在
controller goal 提交前订阅指定的 `geometry_msgs/msg/PoseStamped`，但允许 tracker 在
warmup 期间才产生第一条有效位置。正式完成后必须有足够样本覆盖 yaw 起点、换向点和返回点，
否则拟合失败并保留原始 staging 证据。

每个 Tracker 样本保留 host-monotonic 接收时间，并映射到最近的实际机器人 telemetry 行。
`tracker_telemetry.csv` 同时保存 tracker 位置、由 recording snapshot 内 URDF/杆具和实际关节
重新计算的 `plane1→rod_tip` FK、实际 yaw、两者差值和 `used_for_yaw_fit` 标志。
`yaw_calibration_report.yaml` 拟合：

```text
p_tracker(theta) - p_rodtip = b + Rz(theta) o
```

其中 XY 的固定偏差和旋转偏心可独立辨识；纯 yaw 对 Z 轴只能辨识 `b_z + o_z`。Tracker
位置必须以米给出，坐标轴必须预先与 `plane1` 对齐，消息 frame_id 必须精确匹配命令参数；
程序不执行隐式 TF、坐标轴猜测或运行时轨迹修改。Tracker 结果与 NUCLEO sidecar 一样属于
每次 playback 的派生执行证据，不改变 canonical recording 或其 checksum。

## 2026-07-20 `setup.yaml` 闭包后已验证样例

三条 minimum-jerk 权威输入完成了 strict GPU compile、mock 执行、artifact 封存、`inspect` 和全新独立 mock runtime 的 `play --auto-prepare`：

| recording | 时长 / 点数 | source | v / a / jerk ratio | 独立 playback |
|---|---:|---:|---|---|
| `horizontal_line_mj__20260720_031210` | 33 s / 3301 | 122/122 | `0.007107 / 0.000380 / 0.000096` | 4125 行，speed scaling 全 1 |
| `shallow_s_mj__20260720_033621` | 33 s / 3301 | 122/122 | `0.012932 / 0.002153 / 0.000141` | 4126 行，speed scaling 全 1 |
| `circle_mj_v2__20260720_040223` | 63 s / 6301 | 122/122 | `0.013233 / 0.006077 / 0.003639` | 7882 行，speed scaling 全 1 |

三条 `uniform_time_scale` 均为 1，source 时间误差为 0，源 CSV 与 artifact 内 source copy 的 SHA-256 完全一致；三份 snapshot 的 `setup.yaml` SHA-256 均为 `d904c67c...33cb804`。最后 3 s 均含 301 个严格静止 canonical 点，位置不变且速度、加速度精确为 0。完整 checksum、TF/场景门禁、失败重试和独立 playback 证据见 [ubuntu24 2026-07-20 验证包](evidence/ubuntu24-20260720/README.md)。

旧 recording 因没有绑定新的权威 `setup.yaml` hash 而被 `inspect` 拒绝，runtime 不会用新 frame/杆长补偿旧 artifact。复用长期 runtime 时的一次 cuRobo prepare CUDA 故障，以及 `circle_mj_v2` 首试超过固定完成门限 `0.847624 ms` 的失败也已保留；没有重定时、裁剪或改点。prepare 故障已通过只对独立 MotionPlanner 禁用 CUDA graph 修复，同一 ROS service 连续两次 GPU prepare 已通过。历史 `ellipse_mj` 两次固定 1 mm source-pose 门禁失败继续保存在 2026-07-18/19 证据包。

这些 artifact 的环境哈希绑定当前名义标定、受控 URDF 和现有 drawing-plane contact policy。安装目标机器人真实标定或修改机器人/TCP/杆体/场景/contact policy 后，必须重新 compile、mock recording、validate、inspect 和封存；旧 artifact 会因环境哈希不一致而被拒绝，不能直接转为真机 recording。当前 `deployment_allowed=false`，不得将 mock 证据解读为真机播放已放行。
