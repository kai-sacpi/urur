# ur-waypoint 项目现状汇报

## 结论摘要

截至 2026-07-20，项目已在 `ubuntu24` 完成 `setup.yaml` 单一现场装配源迁移，并重新完成 ROS 2 Jazzy/CUDA 12.8 构建、严格 cuRobo GPU 编译、mock hardware 录制封存、artifact 检查和独立 mock 播放。当前链路是：

```text
权威时标杆尖 Pose CSV
  → GPU IK/碰撞/关节路径编译
  → 保持原时间轴生成 canonical q/qd/qdd
  → mock 整轨执行与数值验证
  → validated recording
  → 同一完整轨迹一次性下发
```

当前 drawing-plane/contact policy、标定/URDF provenance、仿真未缩放执行合同都已实现 fail-closed 门禁。真机仍未放行：示例环境的 `calibrated_kinematics.yaml` 仍为 `deployment_allowed: false`，runtime 必须拒绝真实硬件播放；目标 UR 身份、真实标定、实际 driver/控制器、TCP/payload、安全配置和硬件在环验收也尚未完成，均是投产阻断项。

## 当前怎么做

1. 上游提交带权威 `time_s` 的杆尖 Pose CSV，并负责保证轨迹不过快、加速度和 jerk 合理、整体可行。
2. 系统严格解析 CSV 和环境目录，将杆尖 Pose 转换为 `tool0` 目标，并把杆体、机器人、禁区和场景交给 cuRobo。
3. 在 GPU 工作前先做严格端点静止可行性必要条件检查。canonical 首末关节速度、加速度固定为零；如果权威 Pose/time 在速度、加速度、jerk 上限下连理论位移上界都不满足，直接报错，不进入 IK。
4. cuRobo 在 GPU 上逐 Pose 求 IK。每一步只把达到 5 mm 硬 clearance 的候选放入连续前沿；previous seed 去重并保留多样分支，同时固定留出随机探索 seed，避免 32 个旧解占满 seed 池。所有 source waypoint 及原始纳秒时间保持不变。
5. 连续分支选择器得到逐 knot IK 路径后，只允许按 hash-bound URDF 限位给每个 revolute joint 的整条路径统一加减整数个 `2π`，以选择更接近环境 home 且不降低最小限位裕量的等价表示。绝不逐点 wrap，所有内部关节差保持不变。随后先在原时间轴上直接构造并验证关节轨迹；如果这条硬 Pose seed 已满足动力学，就直接保留。只有 seed 的固定时间动力学失败时，才调用不改时间的 cuRobo 关节路径 refinement，并对 refinement 输出再次执行同一整轨 winding 规范化。最终对每个 canonical 样本复验 Pose、scene/self collision、drawing-plane contact 和关节界限；速度、加速度和 jerk 则对每个连续多项式段检查解析极值。当前碰撞/位置限位证据是 10 ms 输出样本门禁，不表述为连续时间扫掠碰撞证书。cuRobo 的 time-optimal finetune 被禁用，因为它会缩短内部时间步。
6. NumPy/SciPy 在固定关节位置和固定时间上建立同一套连续多项式，统一导出 `q/qd/qdd`；它们可以增加内部密集采样点，但不能移动、替换或删除 source waypoint，也不能修改时间轴。
7. canonical 轨迹的 phase 只包含 `draw/hold`，但并非只有 source 行。例如当前 `circle_01` 是 10 个 source waypoint，输出为 2252 个控制点；新增点只用于连续执行和验证，10 个源点仍在原纳秒时刻精确存在。
8. 若机械臂不在起点，runtime 先规划并执行 recording 之外的独立 cuRobo `prepare` goal。prepare 完成后必须重新确认 source `t=0` 处的新鲜、静止 JointState，prepare 不写入 canonical recording。
9. mock `JointTrajectoryController` 一次性执行完整 canonical goal。runtime 记录 controller desired/actual、JointState、TF 和杆尖数据，做数值门禁并以 SHA-256 原子封存。
10. 正式播放只读取已验证的 `joint_trajectory.csv`，通过精确配置的 UR passthrough action 一次性发送；不重新 IK、不拆点流式发送、不跳点。

## 权威 waypoint CSV 格式

文件位于 `environments/<environment_id>/trajectories/<trajectory_id>.csv`，必须恰好包含以下 10 列，名称和顺序均不能改变：

当前 `lab_wall_a/trajectories/` 下的文件由项目方提供给本系统做严格测试，均按测试资产管理；单个文件通过不等同于该轨迹已获得生产或真机批准。

```csv
time_s,frame_id,tip_x_m,tip_y_m,tip_z_m,tip_qx,tip_qy,tip_qz,tip_qw,phase
0.000000000,wall_draw_frame,0.000000000,0.050000000,0.000000000,0.000000000,1.000000000,0.000000000,0.000000000,draw
0.500000000,wall_draw_frame,0.035355339,0.035355339,0.000000000,0.000000000,1.000000000,0.000000000,0.000000000,draw
1.000000000,wall_draw_frame,0.035355339,0.035355339,0.000000000,0.000000000,1.000000000,0.000000000,0.000000000,hold
```

| 字段 | 含义和硬约束 |
|---|---|
| `time_s` | 十进制秒；先精确解析为 ROS Duration 整数纳秒，再要求当前 binary64 计算链能往返到同一纳秒。首行必须为 `0 ns`，后续严格递增；亚纳秒值或虽在 ROS 类型范围内但无法无损往返的超大时间直接拒绝。 |
| `frame_id` | 全文件一致的 ROS frame；必须能由当前环境 `setup.yaml` 的 `plane1` 或零偏置 alias 解析到基座 frame。 |
| `tip_x_m..tip_z_m` | 杆尖位置，单位米，不是法兰或 `tool0` 位置。 |
| `tip_qx..tip_qw` | 杆尖四元数，顺序固定为 `xyzw`；范数误差不得超过 `1e-3`，只校验、不自动归一化。 |
| `phase` | 只允许 `draw` 或 `hold`。第一行必须为 `draw`；`hold` 的位置和四元数分量必须在 `1e-9` 容差内重复上一行，不接受用 `q/-q` 代替逐分量重复。 |

文件至少包含两行。NaN、Inf、空字段、缺列、多列、未知 frame、非法 phase、非单调或非纳秒时间都会拒绝整份文件。详细契约见 [正式源轨迹 CSV](source-trajectory-csv.md)。

CSV 没有速度/加速度列，因此执行合同还隐含固定边界：canonical 首点和末点的关节速度、关节加速度均为零，`hold` 区间严格静止。上游的可行性保证必须包含从静止启动并在源末时刻停止；内部近似恒速但首尾仍在运动的曲线，不会由下游自动添加加减速段。

## 上下游和软件库职责

| 层/库 | 当前职责 | 明确不做什么 |
|---|---|---|
| 上游轨迹产生方 | 给出完整杆尖 Pose、顺序、phase 和权威 `time_s`；按机器人能力提供不过快、加速度/jerk 合理且可行的轨迹。接口只依赖 CSV，生产上游可使用任意库；仓库示例生成器使用 Python `math`/NumPy | 不把不可行轨迹交给下游后期待自动放慢或删点 |
| Python `csv`、`Decimal`、PyYAML | 严格解析 CSV；将十进制时间映射到 ROS 纳秒；从单一 `setup.yaml` 加载原始示教器 `p[]` 与杆具并派生 TCP/frame，加载禁区、机器人、控制器和录制配置 | 不容错补列，不改 Pose，不归一化四元数 |
| Python `tipdraw` / `RecordingInspector` | 通过 `ros2 run hx_waypoint_tools tipdraw ...` 编排 compile/build-recording/play，生成 compile report，并对 recording schema、checksum、点数、时长和 source 纳秒证据执行 `inspect` | 不绕过 C++ runtime 的播放门禁 |
| cuRobo V2 | FK/IK、达到硬 clearance 的候选生成、连续关节分支、必要时的固定时间关节路径 refinement、floor/rear_keepout 场景、自碰/关节界限、drawing-plane 接触检查，以及独立 prepare 规划 | 不拥有 source 时间轴；time-optimal finetune 禁用；不能修改任何 waypoint 或 `time_s` |
| PyTorch/CUDA | 承载 cuRobo 的 GPU tensor、运动学、碰撞、IK 和优化计算 | 不定义业务 schema 或失败策略 |
| NumPy | 坐标变换、数组运算、连续分支代价和数值一致性检查 | 不修改 source 时间和 Pose |
| SciPy `CubicSpline/BPoly/PPoly` | 在固定关节 knot 和固定时间上构造同一套 C2 五次分段多项式；由同一曲线生成 `q/qd/qdd`，并检查每个多项式段的速度、加速度、jerk 解析极值 | 不重定时，不缩放，不选择新的 source waypoint |
| MoveIt 2/RViz | 发布/显示与 cuRobo 环境同源的 PlanningScene、机器人和执行过程 | 不承担正式轨迹的逐点 IK 或重规划 |
| UR mock hardware + ros2_control JTC | 在仿真中通过精确锁定且禁止 remap 的 `/joint_trajectory_controller/follow_joint_trajectory` 一次性执行完整 `JointTrajectory`，并从精确锁定的 `/joint_trajectory_controller/controller_state` 提供 controller state | 不生成 canonical 轨迹，不允许换成 scaled/custom controller |
| C++ runtime：`rclcpp_action`、tf2、KDL、yaml-cpp | action 状态机、起点/标定/环境门禁、prepare 隔离、录制、杆尖 FK/速度、数值验证、artifact/哈希和 playback | 不在正式播放阶段重新求解轨迹 |
| Universal Robots ROS 2 Driver + `PassthroughTrajectoryController` | 提供真实硬件接口，并把完整 `q/qd/qdd/time_from_start` goal 转交 UR 控制器执行 | 不允许 controller fallback，也不允许改为逐点流式发送 |

环境、机器人、杆体、`floor/rear_keepout` 禁区和 drawing-plane contact policy 由同一环境目录生成 cuRobo 配置、MoveIt PlanningScene 和 recording snapshot。`setup.yaml` 是原始 `base → plane1` 示教器位姿、`tool_id` 和杆体几何的唯一可编辑源；launch 从中发布实时 TF，runtime 在 Build/Playback 前和正式发送边界复核 TF。`zones.yaml` 的物体显式携带 `frame_id`，MoveIt/cuRobo 使用同一转换实现。编译报告与 artifact 对全部输入文件做严格哈希集合校验，避免两套场景静默漂移。最终 contact 门禁要求杆尖最大穿透不超过 1 mm，所有非 tip 杆体/机器人球离墙至少 5 mm；球数、tip 映射或 metadata 不一致时 fail closed。

## Legacy 轨迹、示教坐标和杆长假设

`localizaiton_ros2` 仓库中的 20 条旧 XLSX 已逐行转换为当前权威 CSV schema。转换只做表示变换：显式映射到 ROS 整数纳秒、保留原始 XYZ/姿态/顺序和停留行，并记录源文件与输出 CSV 的 SHA-256；不缩放时间、不重采样、不删点。旧程序留下了 194 mm 和 217 mm 两种 TCP 默认值证据，但没有留下“每个 XLSX 对应哪一次平面示教、哪一根杆”的完整绑定，因此这些轨迹只能作为带显式假设的兼容性测试，不能伪装成已恢复的历史生产配置。

用户提供的 16 行 UR 示教器 `base -> plane1` 位姿中，第 8、14 行完全相同，共 15 个唯一候选。数值上应分开看待：

- 第 1～15 行的平面法向基本朝 `+Z`，完整姿态约为绕 Z 轴 `41°～49°`；其中绝大多数法向倾斜仅约 `0.3°～1.2°`。
- 第 4、5、8、14 行位于另一片远端区域，中心约为 `(0.435,-0.988,0.283) m`，与其余主工作区中心相距约 `0.73 m`，不能与主工作区坐标混为一次标定。
- 第 16 行虽也是近水平面，但 plane `+Z` 基本朝下；它与前 15 行的完整有向坐标系相差约 `177°～180°`，属于独立的 `flipped-normal` 约定，不能与 p10/p15 平均或插值。

示教器命令发布的是 `base -> plane1`，受控 URDF 的环境根则是 `base_link`，且 `base_link -> base = Rz(pi)`。因此受控转换固定为：

```text
T_base_link_plane1 = T_base_link_base * T_base_plane1
translation: (x,y,z) -> (-x,-y,z)
```

现有测试使用的关键候选如下；杆长是另外一项显式 profile 参数，不由位姿数值推导：

| pose | 原始 `base -> plane1`（xyz + rotvec） | 用途/状态 |
|---|---|---|
| p10 | `[-0.07584,-0.38257,0.28285,-0.00534,0.00519,0.82389]` | 近场、正法向候选；用于大包络 optical 的 120 mm 测试 profile |
| p15 | `[-0.00327,-0.50126,0.27942,-0.00576,-0.00076,0.7778]` | 近场、正法向候选；分别与 120 mm optical、217 mm desk profile 配对 |
| p16 | `[-0.03413,-0.38168,0.33594,-1.25152,2.88032,-0.01303]` | 近场、反法向候选；已建立独立 `p16 + 194 mm` profile 验证 quaternion snake，不直接替换或覆盖 p10/p15 证据 |

因此 profile 的权威身份必须是 `(trajectory_id, setup_id, plane_pose_id, tool_id, normal_convention)`。每个环境的 `setup.yaml` 原样保存选定的 `base → plane1` rotation-vector 位姿和杆长；受控 URDF 只负责派生 `base_link → base`。仓库仍用统一 pose registry 保存全部 16 行原始位姿、p8/p14 重复关系和法向约定；importer、测试与 compile matrix 共用同一份逐轨迹 profile 映射。不同轨迹可以选择不同的合理杆长；候选组合失败时，应在上游选择并封存另一份明确 setup 或重新生成轨迹，runtime 仍不得修改 waypoint、姿态或 `time_s`。

首轮 20-case GPU 矩阵采用 p15/p10 与 120/194/217 mm 的四个固定测试 profile，结果为 0 条严格通过：8 条在 GPU 前被首末静止必要条件拒绝，9 条完成 IK 后在原时间轴上违反固定时间加速度或 jerk，1 条在连续 IK 前沿明确不可达，另 2 条在逐点 IK 候选搜索中达到 15 分钟预算后中断。最后两条只能表述为“本次算法/资源预算未完成”，不能误报为几何不可达。该结果并不与旧 MoveIt 低级接口能找到部分空间路径矛盾：旧接口主要回答离散 Pose 是否存在 IK/空间路径，当前链路还要求同一连续分支、杆体/场景 clearance、每个权威纳秒时刻、首末静止以及整段速度/加速度/jerk 同时成立。

其中一个 timeout 暴露了执行端自身的性能缺陷，而非更严格合同必然带来的代价：旧实现对每个已通过 IK/clearance/时序可达门禁的候选，逐个执行一次 GPU FK、一次 6×6 SVD，并通过两个 `.item()` 强制 CPU/GPU 同步。长轨迹最多会触发约 50～79 万次单样本调用。现已改为按原候选顺序 flatten、每 2048 个候选分块批量 FK/SVD，再按原索引回填相同 condition-number 分数；candidate 排序、selector 权重及所有硬门禁均不变，缺失/畸形/非有限 Jacobian 直接 fail closed。RTX 3090 同一 locked cuRobo 的三次运行中位数微基准为：128 个候选 `77.139 ms → 0.959 ms`（`80.43×`），512 个候选 `305.715 ms → 1.058 ms`（`288.93×`），2048 个候选 `1.226834 s → 1.726 ms`（`710.95×`）；三档结果均与逐候选实现严格相等。该优化不改变任何 waypoint、Pose、phase 或 `time_s`。

## 严格失败策略

上游可行性保证是一项接口责任，不等于下游取消检查。下游仍检查端点静止必要条件、每个 Pose 的 IK、碰撞和关节界限，并按 `robot/robot_model.yaml` 的关节速度、加速度、jerk 上限检查连续多项式每一段的解析极值。报告中的 ratio 是“实际最大值 ÷ 配置上限”；任一 ratio 大于 1 即失败。动力学错误同时给出限制关节、原多项式段、精确峰值时刻、物理值和上限，避免把 ratio 误读成物理量。

canonical `draw/hold` 链路禁止轨迹缩放、重定时、裁剪、改点和跳点；不得用密集采样结果替换任何 source waypoint。任何 IK、碰撞、关节位置、速度、加速度、jerk、source 纳秒时间、hash、起点或录制质量问题，整条轨迹直接报错，不做自动修复。独立 `prepare` goal 有自己的规划时间轴，但它不属于、也不能改写 canonical recording。

真机控制器层也不例外：正式 recording 发送前必须收到调用后新到达的
`speed_scaling=1.0`，播放期间任何偏离都会立即取消完整 goal 并明确报错。
telemetry 的权威 `trajectory_time_s` 只由 controller feedback 更新，不用墙钟时间或
`wall_dt * speed_scaling` 推算。

recording 必须包含 `manifest.yaml`、源/关节轨迹、`compile_report.yaml`、仿真记录、`validation_report.yaml`、环境快照和 `sha256sums.txt`。只有 `status: validated` 且完整性、环境、机器人、时间与数值门禁全部通过的 artifact 可以播放。详见 [Recording artifact 与安全门禁](recording-artifact.md)。

## 已验证证据与适用范围

当前已在 `ubuntu24` 对三条上游 minimum-jerk 权威输入完成真实 GPU compile、受管 mock recording、`inspect`、全新独立 mock runtime 和 `play --auto-prepare`。三条均逐点保留全部 source waypoint/ROS 纳秒时刻，`uniform_time_scale=1.0`，独立 playback 的 `speed_scaling` 全程为 `1.0`：

| 轨迹 | validated recording | 权威时长 / canonical 点 | source | v / a / jerk ratio | 最小净空 | 独立 playback |
|---|---|---:|---:|---|---:|---|
| `horizontal_line_mj` | `horizontal_line_mj__20260720_031210` | 33 s / 3301 | 122/122 | `0.007107 / 0.000380 / 0.000096` | 11.273 mm | 4125 行 telemetry，scale min=max=1 |
| `shallow_s_mj` | `shallow_s_mj__20260720_033621` | 33 s / 3301 | 122/122 | `0.012932 / 0.002153 / 0.000141` | 11.272 mm | 4126 行 telemetry，scale min=max=1 |
| `circle_mj_v2` | `circle_mj_v2__20260720_040223` | 63 s / 6301 | 122/122 | `0.013233 / 0.006077 / 0.003639` | 11.272 mm | 7882 行 telemetry，scale min=max=1 |

三条最后的 3 s 均为 301 个严格静止 canonical 点，位置不变，速度和加速度精确为 0。源 CSV 与 recording 内 `source_tip_trajectory.csv` 的 SHA-256 完全相同，source 时间最大误差为 0；三份 recording snapshot 的 `setup.yaml` hash 完全一致。旧 recording 因缺少新 `setup.yaml` hash 被明确拒绝。完整 checksum、compile/validation report、TF/场景门禁和失败尝试证据保存在 `docs/evidence/ubuntu24-20260720/`。

最终代码回归结果为 5 个 ROS 包构建成功，两个 C++ GTest 可执行文件 `70 + 13 = 83` 个 case 全部通过，`colcon test-result` 汇总为 85 tests、0 failure；Python pytest `314/314`。legacy matrix 的既有 `--validate-only` 20/20 数据行证据继续保留。

候选失败同样保留为证据：复用已执行过一条轨迹的长期 runtime 时，下一条 `shallow_s_mj` 的 prepare 曾触发上游 cuRobo CUDA illegal-instruction，runtime 原样失败且没有封存 recording。最小复现确认问题来自 graph-captured MotionPlanner 的第二次变 horizon 调用；最终代码已只对独立 prepare 禁用 CUDA graph，同一 cuRobo ROS service 连续两次 prepare 分别输出 491 点和 211 点并通过。`circle_mj_v2` 首试最后 telemetry 为 `63.050847624 s`，比固定 50 ms 完成门限多 `0.847624 ms`，因此直接拒绝；全新 runtime 对同一 CSV 重跑成功，没有裁剪 telemetry 或放宽门限。历史 `ellipse_mj` 两次 1 mm source-pose 门禁失败继续保留在 2026-07-18/19 证据中。

2026-07-17 的 `circle_01` 五次冷启动 compile 和完整 mock 链仍作为历史基线保留；新证据扩展了覆盖范围，不代表任意轨迹或真机工况自动通过。

### 2026-08-03 NUCLEO 同步录制扩展

新增 `hx_waypoint_nucleo` 可选 sidecar 后，Ubuntu24 已完成 6 个 ROS package 构建、C++
85 tests 和 Python 400 tests。NUCLEO 在机械臂 goal 发送前执行串口 SYNC/ACK 和完整帧
预检；正式执行时独立保存所有 superframe，结束后按同一 Linux monotonic clock 映射到
`execution_telemetry.csv` 的真实机械臂采样，生成 `nucleo_telemetry.csv` 和带 SHA-256
的 `nucleo_capture.yaml`。传感器故障不触发 controller cancel，因此不会裁剪权威轨迹。

50 Hz pseudo-terminal 验收使用 `horizontal_line_mj__20260720_031210`，得到 4126 行机械臂
telemetry 和 1651 行执行窗口内传感器数据；所有传感器 `sync_time_ns` 均精确存在于机械臂
表，最大映射偏差 `4.100515 ms`，最大传感器帧间隔 `24.675714 ms`，正式窗口 parser
完整性增量均为 0；两张 CSV 的 SHA-256 与 manifest 一致，raw staging 已在两阶段 manifest
提交后删除。该结果只验证软件协议与同步链路，真实 NUCLEO 和真机联合验收仍待硬件接入。

项目方提供的测试目录当前按以下语义使用：

| 测试资产 | 当前严格结论 |
|---|---|
| `circle_01.csv` | 停到停正例；保留全部 10 个 source waypoint 和 20.925 s 权威时长 |
| `circle.csv`、`figure8.csv`、`lissajous_xy_z_sweep.csv`、`rounded_square.csv`、`spiral_in_out.csv`、`straight_line.csv`、`vertical_bob.csv`、`zigzag_s_curve.csv` | 端点静止负例；均为 1000 点/9.99 s，内部运动可以平滑，但首尾仍保持姿态运动，在不改时间且首末 `qd=qdd=0` 时违反保守 jerk 位移上界，因此应在 GPU 前明确拒绝 |

这些结论说明测试集同时覆盖正向编译和严格失败关闭，不应把“九条轨迹全部编译成功”设为优化目标。

## 真机放行状态

当前示例环境明确配置：

```yaml
deployment_allowed: false
```

因此 runtime 必须拒绝真机播放，不得手工改成 `true` 绕过。当前 validated recording 的哈希绑定了名义 `setup.yaml`、受控 URDF、场景和 contact policy，不能在将来只把 `deployment_allowed` 改成 `true` 后直接转真机。装入目标机器人真实标定或修改示教器 frame、杆具、机器人或场景后，必须重新 GPU compile、mock recording、validate、inspect 和封存。之后还需确认实际 UR driver 和精确 passthrough action，核对 TCP、杆体、payload 与保护配置，并使用上游生成的低速权威轨迹完成清场、急停/保护停和硬件在环跟踪误差验收。速度滑块必须保持 `speed_scaling=1.0`，不能靠滑块降速修正时间轴。

部署与命令细节见 [Ubuntu 24.04 部署](ubuntu24-deployment.md)。
