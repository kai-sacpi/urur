# UR5＋NUCLEO 实测数据交接说明（mla_short，336 条）

版本：2026-10-18（打包时补充成功记录索引）。本文时间均为 Asia/Shanghai（UTC+08:00），除非字段明确写作 UTC。

适用对象：读取磁场观测、机器人位置标签及背景场的后处理、定位、标定和训练程序。本文描述已完成采集的数据，不是启动机器人或重新采集的操作说明。

## 1. 先读这几条

- 本批是 **UR5 真机＋NUCLEO 传感器板**采集，环境为 `mla_short`，不是早期 `new_board_4cm_rod33365`。使用的名义工具是 **190 mm 长、5 mm 直径**的直杆。
- 推荐位置标签是 `actual_fk_reference.csv` 中的 `plane1_tip_*`：由实际关节反馈计算的**名义杆尖**位姿，不是计划 waypoint，也不是经过独立计量验证的磁铁中心真值。
- 位置单位 **m**，关节角 **rad**，磁场 **µT**；四元数通常为 **xyzw**。`plane1` 的 **−Z 为物理向上**，名义离板法向高度为 `h_m = -plane1_tip_z_m`。
- 磁场与机器人通过同一 Ubuntu 主机的 **monotonic 接收时间最近邻映射**关联；不是硬件同时触发采样，也不是两个表等频、等长或严格一对一。
- 原始观测**没有扣背景、没有施加统一空间偏置、没有补丢帧**。磁铁中心/工具偏心、阵列原点与 `plane1` 的精确对应、绝对传输延迟仍需后处理标定。
- 336/336 已沿既有逐窗业务验收与实际 FK 审计证据链完成；历史失败尝试仍保留。**不能用 `runs/*` 通配全部当有效数据，也不能只按 strict `status=complete` 筛选。**有效窗口的选择见第 7 节。

## 2. 数据位置、范围与完成状态

数据仅保存在 Ubuntu24。以下使用两个根目录缩写：

```text
P = /home/huaxin/Codes/ur-waypoint
W = /home/huaxin/Codes/ur-waypoint/workstation_magtrack
```

本文 `environments/...`、`collection/...`、`runs/...`、`reports/...` 等相对路径均相对 **W**，不是 P，也不是报告文件所在目录。

| 内容 | 路径/标识 |
|---|---|
| 原始 336 条计划与封存 recording 清单 | `reports/overnight_delivery_20261017_v1/collection_manifest.json`，数组名为 `recordings` |
| 本批冻结环境 | `environments/mla_short/` |
| 权威末端轨迹 CSV | `environments/mla_short/trajectories/` |
| 编译、仿真验证后的封存关节回放文件 | `environments/mla_short/recordings/<recording_id>/` |
| 实际执行和传感器采集 | `runs/`，须由验收报告定位具体成功窗口 |
| 最终完成报告 | `reports/raw_resume26_session_20261018_v1/real_session_evidence.json` |
| 最后采集 campaign | `runs/raw_collection_20261018_resume26/` |
| 最后 campaign 软件出处 | `runs/raw_collection_20261018_resume26/software_provenance.json` |
| 打包补充的336条成功记录索引 | `docs/dataset_index_20261018/accepted_trajectories.json` / `.csv` |
| 索引构建范围、验证及限制 | `docs/dataset_index_20261018/index_build_report.json` |
| 迁移到 workstation 的解压/路径解析说明 | `docs/PACKAGE_README_20261018.md` |

最后 3 条是原清单第 **334–336** 条；此前累计 333 条由最终报告的 `prior_cumulative_333_evidence` 向前追溯。最终报告记录累计 336/336、最终回 home 成功、传感器会话关闭，结束于 **2026-10-18 16:06:29**。

本批授权范围为 `operator_authorized_raw_collection`，**不是已完成磁测量全链物理标定的正式 calibrated release**。环境模板中的 `deployment_mode: simulation`、旧清单中的 `real_execution_authorized: false` 是封存仿真资产原有标签；此次 RAW 真机授权另有 installation-bound release、各 campaign provenance 和实际 `/real` 证据。不要据此把真机记录误判为 mock，也不要反过来把仿真 CSV 当真机数据。

## 3. 测量设备与执行方式

| 项目 | 本批定义 |
|---|---|
| 机器人 | Universal Robots UR5；连接 IP `192.168.1.99` |
| 主机/ROS | Ubuntu24，ROS 2 Jazzy |
| 安装标识 | `mla_short_rod190_user_20261017` |
| 工具标识 | `rod_190mm_d5mm` |
| 名义杆长 | `0.190 m = 190 mm` |
| 名义杆直径/半径 | `0.005 m / 0.0025 m` |
| 工具固定变换 | `tool0 → rod_tip`：平移 `[0, 0, 0.190] m`，四元数 xyzw `[0, 0, 0, 1]` |
| 传感器数据 | 4 路 HUB channel，每路 16 个三轴传感器，共 64 个、192 个磁场标量/完整组 |
| 机器人执行 action | **非缩放** `/joint_trajectory_controller/follow_joint_trajectory` |

这里的“190 mm TCP/杆尖”是本项目运动学模型中的 `rod_tip`，沿 `tool0` **局部 +Z**。UR 控制器当前 active TCP **未读取，也不用于本次关节执行及 actual-joint FK**；不能声称示教器当前 TCP 已验证等于 190 mm。不要叠加旧杆长 333.65 mm 或其他历史 TCP 偏移。

冻结 URDF 使用目标 UR5 的官方运动学校准提取结果：`official_ur_calibration_extraction`，提取时间 `2026-08-05T11:33:14+08:00`，checksum `calib_11694654724454936412`。这校准的是机器人运动学，不等于独立校准了杆长、磁铁中心、夹具偏心或阵列坐标。

上游生成权威笛卡尔 CSV，cuRobo 等规划工具进行编译/碰撞与动力学验证并形成封存关节轨迹；ROS runtime 将回放提交给 UR driver/JTC。MoveIt 与场景发布用于共享环境/规划场景；到 home、停车位、轨迹首点的转场另外做碰撞检查，不属于绘制数据窗口。NUCLEO 接收、解析、写盘在独立 reader 进程，主采集程序调度轨迹、背景和窗口映射。实际 FK 审计根据实际关节和冻结模型复算，不用计划点冒充实际位置。

所有 waypoint 及 `time_s` 是权威约束；执行端不得缩放、重定时、裁剪或跳点，动力学/碰撞不满足时拒绝。`speed_scaling` 为无量纲比例，`1.0` 表示 100%；本批不允许使用 scaled controller 改变权威时间。

各 campaign 运行期间软件有修复，**不要假设今天的单一代码版本代表全部历史采集**。应保存对应 `software_provenance.json`、release、环境快照及文件 hash。

## 4. 坐标系、板面与碰撞环境

### 4.1 `base`、`base_link`、`plane1`

采用列向量约定：`T_A_B` 将 B 系坐标转换到 A 系。UR `base` 与 ROS `base_link` **不是同一个朝向**：冻结 URDF 的 `base_link → base` 为零平移、绕 Z 旋转 π。

本批示教器 Installation Plane Feature 名为 **`new_board_mla`**，不是旧 `new_board_4cm`。输入是：

```text
base → new_board_mla Feature
translation_m: [-0.08533, -0.48683, 0.24284]
rotation_vector_rad: [1.16388, 2.91348, 0.00029]
```

后三个数是 UR **旋转向量 rotvec**，模长为旋转角（rad），不是 roll/pitch/yaw。转为四元数之后，再沿 Feature **局部 +Z 平移 53 mm** 定义工作系：

```text
T_base_plane1 = T_base_Feature @ Translation_Feature(0, 0, 0.053)

base → plane1
translation_m: [-0.08511768728206211, -0.4869042639951418, 0.1898404772835752]
quaternion_xyzw: [0.3709743428036358, 0.9286406917135244,
                  0.00009243440854130527, 0.002119936583472163]

base_link → plane1
translation_m: [0.08511768728206211, 0.4869042639951418, 0.1898404772835752]
quaternion_xyzw: [-0.9286406917135241, 0.3709743428036358,
                  0.0021199365834721627, -0.0000924344085413051]
```

以上数值由冻结 `setup.yaml` 计算；没有人为扶正旋转或参照墙调整法向。四元数整体取负表示同一旋转。

`plane1` 原点是本环境中**名义传感器板中心/板面**，模型中板面为 `z=0`。其 **−Z 物理向上、远离板子**，+Z 朝板内/地面；微小倾斜按标定保留。

```text
名义离板法向高度 h_m = -plane1_tip_z_m
例：plane1 z = -0.070 m → 名义高于板面 70 mm
例：plane1 z = -0.200 m → 名义高于板面 200 mm
```

“抬高 10 mm”在此系应为 `z -= 0.010 m`。不能拿 `base_link z` 直接代替离板高度。工作区研究设计覆盖约 70–200 mm，具体每条的范围和实际运动应读取该条 CSV/FK，不从文件名推算。

FK 转换：

```text
T_base_link_tip = FK_frozen_URDF(q_actual) @ T_tool0_rod_tip
T_plane1_tip = inverse(T_base_link_plane1) @ T_base_link_tip
```

### 4.2 阵列原生坐标不是自动等于 `plane1`

本环境将板面中心建模为 `plane1` 原点，**不等于已经独立验证 hospital 传感器原生阵列坐标 S 与 plane1 完全重合**。

研究输入方案描述名义 8×8、50 mm 间距、350×350 mm 阵列跨度，以及外观中心相对原生阵列原点约 10 mm 的 X 差异；这是方案信息，不能自动当成本次实测坐标标定。采集代码没有提供已验证的 `sensor_XX → PCB XYZ` 或芯片轴 → `plane1` 旋转表。

下游必须从实际 hospital 配置/几何标定确认索引和各轴方向；**不要把 64 个索引直接 reshape 成 8×8 并假定物理行列，不要未经确认加减 10 mm，也不要直接把芯片 Bx/By/Bz 当作 plane1 分量**。

### 4.3 碰撞建模（不是传感器物理厚度）

- 传感器板 footprint：`plane1 x,y ∈ [-0.20,+0.20] m`，400×400 mm。
- `sensor_board_keepout`：同 footprint，`plane1 z ∈ [-0.060,+0.19096485659512854] m`。−60 mm 是板上方保护面，**不是板面 z=0**；+Z 侧保守覆盖到基座所在桌面。保护体厚度约 251 mm **不是 PCB 厚度**。
- 分析边界 `sensor_keepout_clearance_boundary` 在局部 `z=-0.065 m`，仅作用于板的上述 x/y footprint，比保护面再留 5 mm。
- 地面/基座所在平面：`base_link z=0`，3×3 m、厚 0.10 m，由四片 slab 构成，基座中心留 0.24×0.24 m cutout。
- 两堵墙及顶部挡板：`x_positive_limit_wall`、`y_positive_limit_wall`、`top_negative_z_panel`，固定在 **base_link**，不随 `plane1` 重标定移动。精确尺寸和位姿读取冻结 `zones.yaml`，不要套用早期坐标系下的墙边界数值。
- 场景共 8 个 cuboid。`zones.yaml` 的 pose 顺序为 **`[x,y,z,qw,qx,qy,qz]`（wxyz）**，与 ROS/CSV 的 xyzw 不同。

### 4.4 Home 与背景停车位

统一关节顺序：

```text
shoulder_pan_joint, shoulder_lift_joint, elbow_joint,
wrist_1_joint, wrist_2_joint, wrist_3_joint
```

| 位姿 | 上述顺序的角度（deg，仅此表为度） |
|---|---|
| Home | `[-129.73, -78.05, -60.56, -129.09, 90.14, 90.00]` |
| `empty_field_park_IMG_5335` | `[-141.48, -84.74, -22.66, 19.73, 99.31, 42.18]` |

机器人 CSV 的关节角仍为 **rad**。每条轨迹按调度先 home 再首点；背景按 home → 停车位 → 静止测量 → home；最终回 home。停车位由用户确认，预期磁铁干扰低，**未独立证明其完全无磁铁影响**。

## 5. 文件与量的字典

### 5.1 计划轨迹与真实采集不要混淆

| 文件 | 含义/使用方式 |
|---|---|
| authority `<trajectory_id>.csv` / recording 中 `source_tip_trajectory.csv` | 权威计划杆尖轨迹，不是实际位置标签 |
| recording 中 `joint_trajectory.csv`、`joint_frames.csv` | 封存的计划关节回放与相关计划量，不是这次真机反馈 |
| recording 中 `sim_actual_*`、`sim_controller_state.csv` | 编译验收的 mock/simulation 证据，不是真机数据 |
| `runs/<actual_run>/real/execution_telemetry.csv` | 本次实际关节与 runtime FK/状态 |
| `action_<id>_actual_reference/actual_fk_reference.csv` | 由实际关节复算的名义杆尖，明确提供 base_link 和 plane1 两套位姿 |
| `runs/<actual_run>/real/nucleo_telemetry.csv` | 已解码磁场＋与该机器人表的最近邻时间映射 |
| 对应 `real/nucleo_capture.yaml` | 映射、覆盖、hash、strict finalizer 状态 |
| session/window `nucleo_raw_telemetry.csv` | 解码后的磁场原始观测，未扣背景；完整 session 还包含转场、等待和失败尝试 |
| `window.json` | 时间窗、boot/epoch、原始文件、mapping 路径、parser/clock 等窗口证据 |
| `action_<id>.operational_acceptance.json` | 业务验收，含已授权的有界丢组接收结果 |
| `actual_telemetry_audit.json` | 实际 FK 审计，`passed` 表示软件 FK/时间/行等检查通过 |

权威末端 CSV 表头：

```csv
time_s,frame_id,tip_x_m,tip_y_m,tip_z_m,tip_qx,tip_qy,tip_qz,tip_qw,phase
```

`time_s` 为从 0 起、严格递增且可表示到 ROS ns 的计划相对秒数；XYZ 是 `frame_id` 中的米；xyzw 是 `rod_tip` 在该系中的单位四元数；`phase` 为 `draw`/`hold`，hold 要保持前一位姿严格不动。**工具固定变换不等于所有轨迹固定姿态**；本批有平滑姿态变化，具体以各权威 CSV 为准。

### 5.2 机器人原生 `execution_telemetry.csv`

下表 `<joint>` 按第 4.4 节六个关节名展开。

| 字段 | 单位/语义 |
|---|---|
| `sync_time_ns` | int64 ns，Ubuntu `CLOCK_MONOTONIC`；JointState 回调开始的**主机接收时刻**，不是控制器原生采样时刻/header |
| `ros_time_s`, `ros_time_ns` | s / int64 ns，runtime ROS node clock；本次 `use_sim_time=false`，用于墙钟关联 |
| `trajectory_time_s`, `trajectory_time_ns` | s / ns，最近一次控制器 `actual.time_from_start` 的缓存值，可能多行重复，不宜作为高频同步主时钟 |
| `state` | runtime 状态字符串，绘制窗中为 `EXECUTING` |
| `<joint>_pos` | **rad**，实际 `/joint_states.position` |
| `<joint>_vel` | **rad/s**，实际 `/joint_states.velocity` |
| `desired_<joint>_pos` | rad，按缓存轨迹进度对提交关节轨迹作插值所得的期望关节角，不是实际反馈 |
| `tip_x`, `tip_y`, `tip_z` | **m、base_link 系**，实际关节 FK 的名义 rod_tip，**不是 plane1** |
| `tip_vx`, `tip_vy`, `tip_vz` | m/s、base_link 分量，由实际 q/qd 推导 |
| `actual_tip_speed_mps` | m/s，上述线速度模长 |
| `tip_wx`, `tip_wy`, `tip_wz` | rad/s、base_link 分量，由实际 q/qd 推导的角速度 |
| `actual_tip_angular_speed_radps` | rad/s，角速度模长 |
| `desired_tip_speed_mps` | m/s，由期望关节状态推导 |
| `position_error_m` | m，实际名义 tip 与按缓存进度计算的期望 tip 之差；不是外部物理计量误差 |
| `speed_scaling` | 无量纲，1.0 表示 100% |

### 5.3 推荐标签 `actual_fk_reference.csv`

保留原机器人表的行顺序、原时间、实际关节值，没有重采样。主要字段：

```text
sync_time_ns, ros_time_ns, trajectory_time_s, trajectory_time_ns, state
<joint>_pos, <joint>_vel                       （六关节，rad / rad/s）
base_link_tip_x_m, base_link_tip_y_m, base_link_tip_z_m
base_link_tip_qx, base_link_tip_qy, base_link_tip_qz, base_link_tip_qw
plane1_tip_x_m, plane1_tip_y_m, plane1_tip_z_m
plane1_tip_qx, plane1_tip_qy, plane1_tip_qz, plane1_tip_qw
```

位置是 m；四元数无量纲、xyzw，表示杆尖坐标系在对应参考系中的方向。板上定位任务优先取 `plane1_tip_*`。

`max_native_fk_error_m` 是独立软件复算与 runtime FK 的一致性差，不是杆尖现实误差；例如约 `5e-16 m` 只是浮点数值一致性。审计中 `fk_tolerance_m=0.001` 也**不表示物理精度为 1 mm**。现实误差还包括机器人/关节反馈、工具长度与偏心、杆形变、板坐标标定等；本次没有独立实测给出绝对精度上界。

### 5.4 磁场与 parser 字段

每组完整数据有 `sensor_00_x_ut` 到 `sensor_63_z_ut` 共 192 列。每列已解码为 **µT**：

```text
wire: big-endian signed int16
B_µT = signed_count × 0.15
```

CSV **已经乘过 0.15，不能再乘一次**。转为 T 时再乘 `1e-6`。浮点长小数是保存格式，不代表仪器物理精度。“raw”指未扣背景/未加空间偏置的解码观测，不是 UART 字节流；未另外保存原始整数 counts。

| 字段/概念 | 语义 |
|---|---|
| `channel_ids` | 完整组的 HUB 数据路集合；channel 不是 XYZ 轴 |
| `sensor_00`–`sensor_15` | channel 1 |
| `sensor_16`–`sensor_31` | channel 2 |
| `sensor_32`–`sensor_47` | channel 3 |
| `sensor_48`–`sensor_63` | channel 4 |
| `logical_seq` | parser 已输出完整组的连续编号，从 0 起；丢组后仍连续，不能单独检出丢组 |
| `src_seq` | 源循环序号，0–15、mod16，不是全局唯一帧 ID |
| `source_time_ms_u32` | 四路源时间 wrap-aware median，设备 uint32 毫秒，不是 UNIX 时间 |
| `channel_N_seq` | 第 N 路 uint32 包序号，N=1…4 |
| `channel_N_nucleo_time_ms_u32` | 第 N 路 NUCLEO 时钟，uint32 ms |
| `channel_N_source_time_ms_u32` | 第 N 路源时钟，uint32 ms |
| `parser_*` | 自本次 reader 启动的累积计数，使用窗内差值解释，不能把累计总数当作该窗损失 |

HUB packet 是一路 16×3 轴的数据；四路按同一源序号集齐才输出一行完整 group/frame。缺一路不会输出“部分 64 点”、不补零。parser 计数包括 `hub_packets`、`completed_groups`、`incomplete_groups`、`timeout_groups`、`channel_sequence_drops`、`source_sequence_drops`、`duplicate_or_reordered_groups`、`duplicate_packets`、`reordered_packets`、`bytes_discarded`、`malformed_packets`、`unexpected_channel_packets`、`pending_evictions`，实际列均有 `parser_` 前缀。

当前格式没有独立温度列或饱和状态位；传感器具体型号、量程、硬件有效位数、芯片每轴方向/内置校准状态及磁铁尺寸/材料/磁矩/极性未由这些文件充分验证，不应推断。原始磁场分量仍在各芯片原生轴中。

## 6. 时间戳与两表关联

### 6.1 时钟定义

| 字段 | 单位/来源 |
|---|---|
| robot `sync_time_ns` | 主机 monotonic 接收时间，int64 ns |
| sensor `sensor_rx_sync_time_ns` | 串口 read 后取得的主机 monotonic 接收时间，完整组使用对应接收时刻的最大值 |
| `parser_host_monotonic_ns` | parser 记录的主机 monotonic 接收时间，本例与上述 sensor 时间相同 |
| `sensor_rx_ros_time_ns` | 此采集入口使用主机 `time.time_ns()` 的墙钟时间；不是磁芯采样时刻 |
| mapped sensor `sync_time_ns` | **复制被选中 robot 行**的 monotonic 时间 |
| mapped sensor `robot_ros_time_ns` | 复制所选 robot 行的 `ros_time_ns` |
| `*_time_ms_u32` | 设备/源 uint32 毫秒，可能回绕和重启归零，不可直接当主机时间 |

`_ns` 全程用整数/int64 读取，避免经过 float 丢精度；`_s` 是秒，`_ms_u32` 是 uint32 毫秒。需要视频墙钟对齐时用相应 `ros_time_ns`/wall clock，转 Asia/Shanghai；不要把 monotonic ns 当 UNIX 时间。

### 6.2 映射规则

对每个磁场完整组，在**同一轨迹的原生 robot 表**内按接收时间选最近行，等距时选较早行：

```text
i = argmin_i |sensor_rx_sync_time_ns - robot[i].sync_time_ns|
robot_row_index = i                         # 0-based，不含 CSV 表头
mapped.sync_time_ns = robot[i].sync_time_ns
sync_offset_ns = sensor_rx_sync_time_ns - mapped.sync_time_ns
```

offset 正数表示 sensor 接收晚于所选 robot 接收。mapped CSV 的 `trajectory_time_s/ns` 也从这行 robot 复制。只映射 robot 首末接收时间范围内的 sensor 行；外围流仍保存在 session/window raw 文件。

没有插值或补帧。两表名义频率约 **robot 125 Hz、sensor 100 Hz**，不能按行号直接对齐原始两表，也不能假设全部 `sync_time_ns` 在 mapped sensor 中唯一。应使用 **`robot_row_index` → 对应 robot/FK 行**，并核对复制的时间一致。跨文件合并还必须保留 `trajectory_id`、`recording_id`、`action_id`、`session_id`、`epoch_id`、`host_boot_id`、`hardware_boot_id`。

不同 host boot 的 monotonic 不可相减；NUCLEO 重启即使 host 时间连续，也要按新 hardware boot/epoch 分组，不跨会话混合设备时间或漂移基线。每次硬件启动只作一次初始 SYNC；同会话复用持续流，不重复握手。

### 6.3 时间质量门槛不是绝对同步精度

- 50 ms：finalizer 的最近邻映射最大允许 offset（`max_mapping_offset_s=0.05`）。
- 10 ms：源 → host 接收相位和通道间相对相位的**持续变化** guard；启动基线冻结，按 1 s bin、连续 3 bin 判定。不是绝对时延上界。
- 500 ms：存活/新鲜度限值，不是同步精度。
- 当前策略 `host_receive_mapping_source_and_interchannel_guard_v1`；共同 NUCLEO → host clock-rate 变化作为诊断保留，不单独作为映射拒绝依据。

**绝对时钟同步与绝对传输延迟未标定。**最近邻 offset 小并不能排除共同 USB 缓冲、板端采样延迟、机器人网络/反馈延迟。若后处理需要物理采样时刻或估计时间滞后，应另做时间标定，并保留原时间列和派生校正列。

## 7. 哪些记录可以使用：验收与丢组

有效数据入口是各封存 `real_session_evidence.json` 的 `accepted_drawings`（背景为 `accepted_backgrounds`），以及对应窗口的业务验收/FK 审计；不是目录最新时间、文件名相似程度或仅看 controller succeeded。

对每条绘制记录，应核对该 action 的 `operational_acceptance.accepted=true`、实际 FK audit `passed=true`、controller 成功，并读取该窗 clock/epoch/mapping 结果。保留原序号 `original_trajectory_ordinal`（1-based）和 `original_trajectory_index`（0-based），避免续采 campaign 的局部 index 被当成全局 index。

本批损失策略为 `bounded_group_loss_10_per_3000_v1`。令完整组 C、交叉证明的缺失组 M、期望 E=C+M：

```text
M / E ≤ 10 / 3000 = 1/300
等价整数判断：M × 300 ≤ C + M
```

不是每条固定最多丢 10 组。短窗 9/2554 ≈ 10.57/3000 会被拒绝。缺失数须结合源 mod16、四路包序号与 parser 差值交叉验证；`incomplete_groups` 与 `channel_sequence_drops` 可能反映同一损失，**不能直接相加**。乱序、重复、坏包、时钟/epoch/映射故障不因少量丢组政策而豁免。

strict finalizer 原来拒绝任何丢组，所以合法的业务接收窗口仍可能保留 `window.json.status="incomplete"`、`quality.accepted=false`、`nucleo_capture.yaml.status="incomplete"`。这些原始 strict 状态没有重写；另一个 operational acceptance 接受了已记录的有界损失。下游应保存两种状态，不能只按 strict 筛掉合法窗，也不能把整个失败 session 全部放行。

例：原第 334 条完整 2855、缺失 5、期望 2860，归一化为 `5.244755/3000`，strict incomplete、operational accepted、实际 FK 审计通过。该例 robot 3575 行，最大相邻间隔 8.394547 ms；sensor 最近邻最大绝对偏移 4.092300 ms。这些是**该例**统计，不代表全部 336 条的统一最坏值。

部分长会话在较后阶段出过时序故障；不能把“其中某窗已接收”扩展为“整个会话 clock-clean”，也不能仅因会话后来失败就自动否定此前已独立验收的窗口。使用对应窗口证据和后续封存裁定。

打包时已从18个明确的既有验收/续采证据源整理 **336条成功记录总索引**，位于 `docs/dataset_index_20261018/accepted_trajectories.json` / `.csv`；JSON数组名为 `accepted_trajectories`。按原manifest顺序核对原始index 0–335、ordinal 1–336，无缺失/重复；逐条核对trajectory/recording、业务accepted、实际FK audit passed、数据入口存在及JSON元数据绑定。具体来源与检查见同目录 `index_build_report.json`，可复现脚本为 `build_dataset_index.py`。

这次整理**只核对既有小型元数据，不是重新读取/哈希全部原始CSV或重新验收物理质量**；索引保留原始证据链。全量处理从该索引进入，不从runs通配进入。末次 `raw_resume26` 报告仍只含最后3条，第9节代码仅读取其中第334条，不是遍历336条的循环。

下面是末次报告/窗口的机器字段示例，不是对所有历史版本强行套用的新 schema：

| 位置 | 本例通过值/用途 |
|---|---|
| report drawing `.operational_accepted`、`.actual_fk_audit_passed`、`.robot_controller_succeeded` | 均为 `true` |
| report drawing `.window_clock_error_at_finalization`、`.window_session_sensor_guard_error` | 均为 `null` |
| operational `.accepted`、`.status`、`.policy_id` | `true`、`accepted_with_recorded_group_loss`、`bounded_group_loss_10_per_3000_v1` |
| operational `.complete_groups`、`.missing_groups`、`.expected_groups` | 本例 C=2855、M=5、E=2860；M 来自已执行的策略证明，不从 parser 列直接相加 |
| audit `.passed` | `true` |
| window `.clock_drift.error`、`.session_sensor_guard_error` | 均为 `null` |
| window `.mapping.path`、`.raw_csv`、`.robot_telemetry` | mapped sensor、raw 窗口、原生 robot 的实际路径 |
| window `.session_id`、`.epoch_id`、`.hardware_boot_id`、`.host_boot_id` | 不能为空，保留分组和来源；与相应 session/epoch 证据核对 |

旧的 strict-complete 业务验收可能不重复写出 C/M/E，不能把字段缺失当成 0。历史 schema 不认识、证据路径缺失、同一原始序号出现多个候选而无明确后续裁定时，批处理程序应标为待复核，不默认通过。使用包内已整理的索引及其元数据来源，不要通过“取最近修改时间”解决重试去重。索引各路径相对原W，迁移后的解析见PACKAGE_README；原JSON的绝对路径保留，不能在workstation上照搬。

## 8. 30 s 背景场与尚未应用的标定

背景调度约每 600 s，在完整轨迹结束后执行，故不是严格整十分钟：home → 停车位 → 静止采集 30 s → home。每个 NUCLEO boot 完成 3 次背景后按策略停止等待硬件重启；全部轨迹提前完成时直接最终 home/关闭，不强补 3 组。部分会话开始前也测了初始背景。

背景使用该 session 的 `windows/background_XXXX/`、`action_<id>.background.json`、业务验收及 `action_<id>.stationary.csv(.json)` 定位。不要把转场/等待流混入背景。窗口 raw 可能额外含 1 行窗前 baseline 用于 parser 差值证据，故 raw 行数不一定等于 `window.json.frames`；使用窗口边界/元数据区分，不把 baseline 当采集帧。

末次 boot 的背景例：`action_386114e1397f4a2283136f665db93193`，2992 完整＋8 缺失/3000 期望；stationary 3751 行、最大名义杆尖 FK 漂移 0.104127 mm。它说明该窗通过既定静止检查，**不证明停车位磁场为零**。

`docs/dataset_index_20261018/accepted_backgrounds.json`另整理29组被上述18个来源明确列为accepted的背景。该索引主要覆盖resume10起，**不声称穷尽所有早期背景**，也不是自动指定每条绘制应扣哪组背景；覆盖说明见该文件`coverage`。原始早期背景及其证据仍保留在包内runs中。

后处理建议按时间/硬件 boot、板位置和接线配置匹配已接收背景；若平均、插值或扣除，应另存派生数据与算法参数，不覆盖原始观测。不要无条件把所有日期/环境背景混成一个均值，也不要重复扣背景。

本批统一空间偏置策略为 `deferred_shared_bias_from_low_height_data`，`applied=false`。用户计划由低高度数据估计共同偏置，再应用到其他数据。本文不预先决定偏置符号或数值。若定义 `d = p_tracker - p_nominal_rod_tip`，固定偏差 b 下对 tracker 的平移校正为 `p_tracker_corrected = p_tracker - b`；若要改机器人标签，须明确另立约定，不能两侧同时改。

若发现绕杆轴的偏心随姿态旋转，则需要工具坐标中的偏心向量及旋转模型，不能一律用同一个世界系平移替代。背景补偿、空间刚体配准、工具偏心、时间滞后是不同操作，应分别记录版本、拟合数据和适用范围。磁铁物理中心的独立准确性在本批未验定。

## 9. 下游读取示例（只读，不控制硬件）

下面示例针对最终报告 `accepted_drawings[0]`，即原第 334 条。依赖 Python/pandas，仅读取该例数据，不改原文件。实际部署可按项目数据规模分块读取；这里展示字段与关联规则。

```python
import json
from pathlib import Path
import numpy as np
import pandas as pd

W = Path('/home/huaxin/Codes/ur-waypoint/workstation_magtrack')
report = json.loads((W / 'reports/raw_resume26_session_20261018_v1/'
                     'real_session_evidence.json').read_text())
d = report['accepted_drawings'][0]  # 本报告只含 334–336，不是全部 336 条
assert d['operational_accepted'] and d['actual_fk_audit_passed']
assert d['robot_controller_succeeded']
assert d['window_clock_error_at_finalization'] is None
assert d['window_session_sensor_guard_error'] is None

op = json.loads((W / d['operational_acceptance']['path']).read_text())
audit = json.loads((W / d['actual_fk_audit']['path']).read_text())
window = json.loads((W / d['strict_sensor_window']['path']).read_text())
assert op['accepted'] and audit['passed']
assert window['clock_drift']['error'] is None
assert window['session_sensor_guard_error'] is None
# 不要求 strict window.status == 'complete'；见有界丢组政策。

def resolve(path):
    path = Path(path)
    return path if path.is_absolute() else W / path

def read_csv_exact_ns(path):
    # 所有 ns 列从 CSV 直接以整数读取，不先转成 float。
    columns = pd.read_csv(path, nrows=0).columns
    dtype = {c: 'int64' for c in columns
             if c.endswith('_ns') or c == 'robot_row_index'}
    return pd.read_csv(path, dtype=dtype)

robot = read_csv_exact_ns(W / d['robot_actual_telemetry']['path'])
fk = read_csv_exact_ns(W / d['actual_nominal_fk_reference']['path'])
sensor = read_csv_exact_ns(resolve(window['mapping']['path']))
assert len(robot) == len(fk)
assert np.array_equal(robot.sync_time_ns.to_numpy(), fk.sync_time_ns.to_numpy())
i = sensor.robot_row_index.to_numpy(dtype=np.int64)
assert np.all((i >= 0) & (i < len(robot)))
selected_robot_time = robot.sync_time_ns.to_numpy()[i]
assert np.array_equal(sensor.sync_time_ns.to_numpy(), selected_robot_time)
assert np.array_equal(sensor.sync_offset_ns.to_numpy(),
                      sensor.sensor_rx_sync_time_ns.to_numpy() - selected_robot_time)

# 每个 sensor 观测的实际关节推导名义杆尖标签；单位 m / xyzw。
pose_cols = ['plane1_tip_x_m', 'plane1_tip_y_m', 'plane1_tip_z_m',
             'plane1_tip_qx', 'plane1_tip_qy', 'plane1_tip_qz', 'plane1_tip_qw']
pose = fk.iloc[i][pose_cols].reset_index(drop=True)
B_cols = [f'sensor_{s:02d}_{axis}_ut' for s in range(64) for axis in 'xyz']
B_uT = sensor[B_cols].to_numpy()       # 不再乘 0.15；芯片原生轴
height_m = -pose.plane1_tip_z_m.to_numpy()
# 后续另存派生结果，同时携带 d/window 中的轨迹、action、boot、epoch 标识。
```

本例路径也可直接由最终报告中的 `robot_actual_telemetry.path`、`actual_nominal_fk_reference.path`、`strict_sensor_window.path` 查得。`window.mapping.path` 指向 mapped sensor；`window.raw_csv` 指向原始窗口观测。哈希使用对应报告/审计/manifest 的记录；本文编写没有重新扫描所有原始 CSV 来重做全数据验收。

## 10. 可追溯配置与后续交接要求

定位/批处理前尚需补齐的资产，当前交付状态如下；这些缺口不会由本说明自动补偿：

| 资产 | 当前状态/建议记录格式 |
|---|---|
| 336 条成功采集总索引与背景关联 | 成功轨迹索引已整理到 `docs/dataset_index_20261018/`；背景索引29组且覆盖有限，逐绘制背景扣除/关联策略仍由下游明确 |
| 传感器物理几何和轴向注册 | 待由实际 hospital 配置/标定确认；建议每个 `sensor_index` 记录 `position_S_m[3]`、`R_S_chip[3×3]`，以及 `T_plane1_S`、坐标/旋转约定与版本 |
| 共同偏置/工具偏心 | 下游由低高度等数据拟合；明确修正哪一侧、符号、参考系、工具旋转模型和适用范围 |
| 绝对时间延迟 | 未标定；若拟合，另存校正值/模型与原接收时间，不改原表 |

名义杆尖位置与芯片原生磁场读取已经有明确字段；若算法要求物理阵列位置、磁场向量同系表达或绝对磁铁中心真值，则上述外部标定是前置条件，而不是可忽略的零偏默认值。

核心配置/代码：

- `environments/mla_short/setup.yaml`：Feature、plane1 和 190 mm 工具。
- `environments/mla_short/environment.yaml`、`zones.yaml`：板面、保护柱、墙/地面。
- `environments/mla_short/robot/{robot_model.yaml,driver_kinematics.yaml,calibrated_kinematics.yaml,assets/ur5.urdf}`：home、机器人模型/运动学校准。
- 每个 recording 的 `environment_snapshot/`、`manifest.yaml`、`sha256sums.txt`：该条封存资产。
- `collection/config.yaml`：背景停车位和调度策略；真实授权另看 `collection/raw_authorizations/` 及对应 released manifest。
- 各 campaign `software_provenance.json`、session 的 `session_manifest.json`、`epoch.json`、`startup_readiness.json`、`drift.json`：软件、boot、时序及收尾证据。
- `collection/{actual_telemetry.py,sensor.py,quality.py,drift.py}`：实际 FK、窗口/同步、丢组和漂移策略。
- `P/src/hx_waypoint_nucleo/hx_waypoint_nucleo/{protocol.py,capture.py}`：wire 解码与时间映射。声明参考 `hxhou0207/magnet_localization_real` 的 `hospital` 分支，commit `b80f4ebee945c6ac07be7732621fa8cd1dc056cd` 的 `magloc/sources/hardware/nucleo/protocol.py`；不是已经完成阵列几何注册的证明。
- `inputs/real_trajectory_collection_plan_20261013.md`：研究方案，仅作设计背景，不替代测量/标定事实。

冻结环境关键 SHA256：

```text
setup.yaml
eb8f79e46fffc50a72069c4531642737e45f0889c9962a741c2d904087f7684d
environment.yaml
f74e1d9deefb4645fd76f9f5fb138066828a6aee5af5e48b4a9d8516a1de9e40
zones.yaml
3de7d095bde6693d13a39eda552945fb5ec9ff4fda05b81f7f962ce87c9a46a1
robot/assets/ur5.urdf
2ef10ba4989720ebcc9e49c64b0ecb49ac94f20ef20147ad1e841d058258c9e6
```

最终完成证据 SHA256：

```text
reports/raw_resume26_session_20261018_v1/real_session_evidence.json
cc41f67fd5e491a6935abb4edc2dbdc42cadde714f906206e33e11972caa10bc
reports/raw_resume26_session_20261018_v1/checkpoint_complete_original.json
f87069545fe1e948ae9ebfaee96326af0279b0618e03e417d4c6b195802831f6
```

交给另一个程序时，至少保留：成功 action/window 的路径与 hash、原计划 ID/序号、实际机器人/FK 表、mapped sensor 表、raw 窗口、业务验收与 strict 状态、session/boot/epoch、对应背景及环境快照。所有校正输出应为新文件，携带校正模型和版本；原始磁场、实际关节、原时间、权威 waypoint/time_s 不应被静默改写。
