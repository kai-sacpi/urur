# 正式源轨迹 CSV

源 CSV 描述的是**杆尖**在指定坐标系中的时标 Pose，不是 `tool0` 的位置轨迹。编译器从环境的权威 `setup.yaml` 派生 `tool0 → rod_tip`，把杆尖目标转换为 `tool0` 目标，并把同一杆体碰撞模型一起交给 cuRobo。

## 文件位置与表头

```text
environments/<environment_id>/trajectories/<trajectory_id>.csv
```

当前仓库 `lab_wall_a/trajectories/` 中的 CSV 是验收测试轨迹，不是生产任务白名单。

UTF-8 和带 BOM 的 UTF-8 都可读取。表头必须与下面内容完全一致，包括顺序；不接受别名、缺列或附加列。每一条数据行也必须恰好包含这 10 列，行尾多余的空列同样会拒绝。

```csv
time_s,frame_id,tip_x_m,tip_y_m,tip_z_m,tip_qx,tip_qy,tip_qz,tip_qw,phase
```

## 字段契约

### `time_s`

- CSV 中以十进制秒书写，但权威表示是 ROS Duration 的整数纳秒；`time_s` 不是可任意取值的浮点时间。
- 必须有限、非负。
- 每个值必须先精确表示为 ROS 纳秒；存在小于 1 ns 的小数部分或超出 ROS Duration 范围时直接拒绝。
- 当前规划/插值数值链使用 binary64 秒，因此解析器还要求该时间能够往返到完全相同的整数纳秒。少数虽在 ROS Duration 类型范围内、但绝对值过大而无法无损往返的时间同样 fail closed，不会被舍入后继续。
- 第一条必须精确为 `0 ns`。
- 后续必须严格递增，不允许同一时间写两行。
- 时间轴是不可修改的权威输入；编译、录制和播放都使用原始相对轨迹的 ROS 纳秒时间，而不是 CSV 读取或 cuRobo 解算的墙钟时间。
- 任何阶段都不得缩放、偏移、重定时或重写任何 `time_s`。

上游必须在提交 CSV 前确保轨迹不过快、加速度与 jerk 合理，且 IK、碰撞、关节界限和动力学均可行。编译器只使用原始 ROS 纳秒时间验证速度、加速度和 jerk；任一限制不满足时直接报错并拒绝整条轨迹。不存在可启用的时间缩放、重定时、裁剪、改点、跳点或其他自动修复模式。

CSV 不包含速度或加速度列，因此边界导数由执行合同固定：canonical 轨迹在第一点和最后一点的关节速度、关节加速度均为零，`hold` 区间同样严格静止。上游所说的“轨迹可行”必须包含从静止进入首段、并在源末时刻回到静止的可行性；不能只检查中间段的有限差分。把一段仍以恒速运动的周期曲线直接截成 `t=0..end`，即使内部速度和加速度很小，也可能因为首末几毫秒内无法启动或刹停而被正确拒绝。系统不会自动添加加减速段或延长时间。

### `frame_id`

- 必须是非空、无空白的 ROS frame 名称。
- 同一文件中的所有行必须完全相同。
- 必须能由环境的 `setup.yaml` 解析到机器人基座 frame。
- 新现场数据使用 `plane1`；示例环境还为既有权威 CSV 保留与 `plane1` 重合的 `wall_draw_frame` alias。

### `tip_x_m,tip_y_m,tip_z_m`

- 单位只能是米。
- 三个值都必须有限。
- 表示杆尖，不表示法兰或 `tool0`。
- 不支持通过文件名或额外参数把数值解释为毫米。

### `tip_qx,tip_qy,tip_qz,tip_qw`

- 四元数顺序固定为 `x,y,z,w`。
- 四个值都必须有限。
- 四元数范数与 1 的差必须不大于 `1e-3`；只校验，不归一化或改写输入值。
- 不能省略姿态，也不能用 RPY/欧拉角替代。
- `hold` 行必须把 `tip_qx,tip_qy,tip_qz,tip_qw` 四个分量逐值重复上一行。即使 `q` 与 `-q` 表示同一空间姿态，本 schema 也不接受这种替换。

### `phase`

正式源文件只接受：

- `draw`：杆尖沿给定 Pose 轨迹运动。
- `hold`：显式停留。该行必须逐值重复上一行的位置和四元数分量，只增加 `time_s`；从上一行时间到该行时间之间，最终关节轨迹保持位置不变，速度和加速度严格为零。

canonical `joint_trajectory.csv` 同样只允许 `draw/hold`。起点就位由 recording 之外的独立 cuRobo `prepare` goal 完成，不会由编译器在 canonical 轨迹前后生成 `approach/engage/disengage/exit` 前缀或后缀，也不会进入 recording。

## 行级和整文件约束

- 至少两个 Pose。
- 第一行必须为 `draw`；`hold` 只能从第二行开始并重复前一 Pose。
- 每一行 waypoint 及其 ROS 纳秒时戳都是不可修改的硬约束，必须在编译输出中逐点精确保留对应时刻和 phase。
- canonical 输出的第一点必须是源 `t=0`，最后一点必须精确等于源文件末时刻；不得在前后添加任何 recording 阶段。
- canonical 第一、末点的关节速度和关节加速度固定为零；上游时间轴必须为这两个静止边界留出足够的动力学余量。
- 不允许改点、重排、裁剪、删除或跳过 waypoint；任何内部采样或插值都不得替换、移动或消除源 waypoint。
- IK 的 revolute joint 数值可能相差整数个 `2π`。编译器只允许对某一关节的**整条关节路径**施加同一个整数圈数偏移，且完整路径必须留在 hash-bound URDF 限位内、不得降低最小限位裕量；禁止逐 waypoint wrap 或 modulo 比较。该操作不改变任何源 Pose、phase、`time_s` 或内部关节差，偏移后必须重新通过 FK、接触、碰撞、限位和动力学验证，并把 offset 写入 compile report。
- 任一 IK、碰撞、优化或动力学验证失败都会拒绝整条轨迹，不尝试自动修复。
- 输入点的时间顺序、frame 和姿态在进入 cuRobo 前先做 fail-closed 校验。
- 任何 NaN、Inf、空字段或无法转换为浮点数的内容都会拒绝整份文件。

## 正确示例

```csv
time_s,frame_id,tip_x_m,tip_y_m,tip_z_m,tip_qx,tip_qy,tip_qz,tip_qw,phase
0.000000000,wall_draw_frame,0.000000000,0.050000000,0.000000000,0.000000000,1.000000000,0.000000000,0.000000000,draw
0.500000000,wall_draw_frame,0.035355339,0.035355339,0.000000000,0.000000000,1.000000000,0.000000000,0.000000000,draw
1.000000000,wall_draw_frame,0.035355339,0.035355339,0.000000000,0.000000000,1.000000000,0.000000000,0.000000000,hold
```

## 常见错误

```csv
# 错：旧版 time,x,y,z 格式，缺 frame、姿态和 phase
time,x,y,z
```

```csv
# 错：四元数写成 wxyz；本项目固定读取 xyzw
time_s,frame_id,tip_x_m,tip_y_m,tip_z_m,tip_qw,tip_qx,tip_qy,tip_qz,phase
```

```csv
# 错：hold 改变了位置；停留行必须复用上一 Pose
1.000000000,wall_draw_frame,0.040000000,0.035355339,0.000000000,0,1,0,0,hold
```

## 编译输出与源文件的区别

源文件是杆尖 Pose 约束；`joint_trajectory.csv` 是 cuRobo 编译后的六关节轨迹，包含：

```text
time_s
<joint>_pos × 6
<joint>_vel × 6
<joint>_acc × 6
phase
```

真机播放只消费 recording 中的 `joint_trajectory.csv`，绝不在播放时重新打开源 CSV 做 IK。

编译输出必须保留每个源 waypoint 的原始 ROS 纳秒时刻、phase 与 Pose 约束；中间可以有额外的密集插值点，但这些点不得替换、移动或消除任何 source waypoint。如果无法在这些不可改写的约束下生成可行关节轨迹，编译失败，不会缩放、重定时、裁剪、改点或跳点。
