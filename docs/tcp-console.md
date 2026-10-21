# 实时 TCP 查看与手动位姿控制

## GUI（推荐）

现场手动调试优先使用图形界面。在 Ubuntu 24 上只需运行一条命令：

```bash
bash /home/huaxin/Codes/ur-waypoint/scripts/start_tcp_gui.sh
```

窗口内按下面四步操作即可：

1. 选择环境（默认 `mla`）、机器人 IP，以及是否打开 RViz；
2. 点 `START MANUAL STACK`，等待顶部同时显示 `LAUNCH ONLINE` 和
   `ROS READY`；
3. 打开“启用运动”开关；
4. 按住方向按钮，或按住 `W/A/S/D/R/F`，松开时停止连续点动。

默认速度为 `10 mm/s`，也可选择 `5 / 20 mm/s`。在 `mla + plane1` 组合中，
`R = -Z`，即物理抬高、远离传感器板；`F = +Z`，即靠近板面。切换到其他
环境后，界面只显示准确的坐标轴正负，不会擅自把局部 Z 称为物理“上下”。
如果 Servo 尚未就绪，顶部状态和右下日志会直接说明拒绝原因。

长按 Jog 使用 MoveIt Servo：`pressed` 事件同步发送第一条非零速度命令，
随后由 GUI 以 `50 Hz` 持续刷新；`released` 事件先清除本次按压的 generation，
再立即发送零速度。因此操作语义是“按下即动、松开即停”，不再为每个小步重复
运行 IK 和 MoveGroup 规划。Servo 使用完整 MoveIt planning scene 做碰撞、关节
边界和奇异点监控，线速度上限硬限制为 `20 mm/s`，角速度恒为零，杆尖姿态
保持不变。窗口失焦、关闭运动授权、取消、停止控制栈、状态过期或 Servo 异常
都汇入同一个立即归零路径；输入超时 `100 ms` 还提供独立停止保护。

绝对坐标区域只需要选择 `plane1/base`，再分别输入杆尖 `X`、`Y`、`Z`（米）。
无需输入 `p[...]` 或姿态；程序会在真正执行该命令时读取实时 `rod_tip`，仅替换
所选坐标系中的 XYZ，并严格保持当时的实际姿态。只有点击
`MOVE TO XYZ · KEEP CURRENT ORIENTATION` 才会执行，按 Enter 不会意外下发。
“取消”会请求停止当前动作。

点 `STOP MANUAL STACK` 或关闭窗口时，界面会先取消未完成动作，再依次关闭
自己启动的 MoveIt、UR driver 和 RViz；在进程组尚未确认退出时不会误报
`OFFLINE`，也不会允许启动第二套控制栈。

GUI 只是易用入口，仍复用所选环境绑定的完整碰撞场景、控制器状态、TF、TCP/
杆模型和机器人状态校验；任何门禁失败都会拒绝运动，不会绕过安全检查。

## `tcp-console`（高级/无界面入口）

`tcp-console` 是面向现场调试的终端程序。它以项目中定义的杆尖
`rod_tip` 为 TCP，以 `5 Hz` 连续显示最新位姿和关节状态；在显式解锁后，也可将
`rod_tip` 移动到一个绝对位姿，或按固定的小步长沿坐标轴移动。

交互式终端使用 curses 实时界面，字母键、方向键和 PageUp/PageDown 都是
即时按键。每次只允许一个完整 MoveIt 动作；动作期间的运动键会被忽略，
动作结束时还会清空终端输入缓冲，因此不会把旧按键变成下一条命令。若标准
输入或输出不是 TTY（例如管道/日志脚本），程序自动退回需要按 Enter 的行模式。

它不是 waypoint 播放器，也不会修改、缩放、重定时、裁剪或跳过任何
权威轨迹。手动运动不会自动录制 NUCLEO 数据；需要同步采集时仍应使用
正式的 recording 流程。

## 环境是一个不可拆分的启动选择

启动时必须显式选择一个环境，例如 `mla`。这里的环境不是单独的坐标系
别名，而是一套不可混用的合同：同一个 `environment_id` 同时给出碰撞场景、
`base -> plane1`、`tool0 -> rod_tip` TCP 与杆尺寸、UR 型号和标定运动学，以及
规划/关节分支约束。
`real.launch.py` 不再提供默认环境，控制台的 `--env` 也是必填项。

终端 A 的 `environment_id:=...` 和终端 B 的 `--env ...` 必须完全相同。
控制台会按 `--env` 选择的合同，在每次运动前逐项核对实时 TF、完整 planning
scene、附着杆和控制器；任何一项不符合都会直接拒绝运动。

控制器需要区分用途：环境 `controllers.yaml` 中的 `real` 合同仍服务于权威
recording 回放；手动程序使用 UR MoveIt 默认的
`scaled_joint_trajectory_controller`。连续 Jog 由 MoveIt Servo 输出位置型短时域
命令；绝对 XYZ 则先暂停 Servo，再用 MoveGroup 完整规划和执行，完成后
安全恢复 Servo。两条路径由同一个运动锁严格互斥，且每次会确认控制器处于
`active`。正式 recording 与手动模式不会同时启动，也不能互相 fallback。

## 坐标和 TCP 语义

- `base`：UR 控制器/示教器的 Base 坐标系。终端中的主位姿
  `base -> rod_tip` 以示教器常见的 `p[x,y,z,rx,ry,rz]` 形式显示；前三项
  单位为米，后三项是弧度制旋转向量，不是欧拉角。
- `base_link`：MoveIt 使用的规划根坐标系。UR 模型中
  `base_link -> base` 含绕 Z 轴的 180 度固定旋转，因此不能把
  `base_link` 的 X/Y 数值直接当成示教器 Base 数值。
- `plane1`：当前环境保存的工作/传感器板坐标系。程序同时显示
  `plane1 -> rod_tip`，绝对目标也可显式写在 `plane1` 中。
- `tool0`：MoveIt 直接规划的机器人末端 link。
- `rod_tip`：本项目实际控制和显示的 TCP。MLA 环境中
  `tool0 -> rod_tip` 是沿 tool0 +Z 的 `0.33365 m` 固定杆长，杆直径为
  `5 mm`。程序会把杆尖目标换算为 `tool0` 目标，不能把输入误认为
  `tool0` 位姿。

只有当示教器当前 Active TCP 也设置为同一根 `333.65 mm` 杆的杆尖时，
终端显示的 `base -> rod_tip` 才与示教器 TCP 页面具有完全相同的物理含义。

MLA 的板面法向与日常语言容易混淆：在 `plane1` 中，物理上的“抬高、
远离传感器板”是 **-Z**，靠近板面是 **+Z**。操作时以界面显示的明确
轴符号为准，不要只凭“上/下”判断。

每份状态快照包含：

- `base -> rod_tip` 的 UR `p[...]` 和 `quaternion_xyzw`；
- `plane1 -> rod_tip` 的 UR `p[...]` 和 `quaternion_xyzw`；
- 六个实际关节角的弧度值、角度值以及最新 JointState 的接收年龄。

这里的 `rod_tip` 来自 ROS TF 与 MLA 中保存的固定杆几何；程序不会读取或
修改 UR 控制器里可变的 Active TCP 设置。

## Ubuntu 24 启动

现场应保持示教器在手边，并确保人员、线缆和环境中未建模物体不会进入
机械臂运动区域。下面的专用手动模式会启动 UR driver、MoveIt、环境 TF、
完整 planning scene 和 RViz，同时不启动 waypoint runtime 与 cuRobo prepare
server，避免手动目标和 recording runtime 竞争同一个控制器。手动模式明确
激活 MoveIt 默认使用的 `scaled_joint_trajectory_controller`；每次运动前程序
还会从 controller manager 确认它确实处于 `active` 状态。

终端 A：

```bash
cd /home/huaxin/Codes/ur-waypoint
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
export PYTHONNOUSERSITE=1
source install/setup.bash
export ROS_DOMAIN_ID=77

ros2 launch hx_waypoint_bringup real.launch.py \
  robot_ip:=192.168.1.99 \
  environment_id:=mla \
  workspace:=/home/huaxin/Codes/ur-waypoint \
  manual_control:=true \
  launch_rviz:=true
```

`ur_type` 与 driver 的 `kinematics_params_file` 不再接受独立选择；启动程序会
分别读取 `environments/mla/robot/robot_model.yaml` 和同一目录内的
`robot/driver_kinematics.yaml`。因此不会把另一个环境或另一台机器人的标定文件
混入当前碰撞场景、坐标系与 TCP。

终端 B 重新 source 同一环境：

```bash
cd /home/huaxin/Codes/ur-waypoint
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
export PYTHONNOUSERSITE=1
source install/setup.bash
export ROS_DOMAIN_ID=77

ros2 run hx_waypoint_tools tcp-console \
  --workspace /home/huaxin/Codes/ur-waypoint \
  --env mla
```

上面的命令默认只读，可以用于先核对当前 TCP、关节角和 TF。若只想
打印一份当前状态后退出：

```bash
ros2 run hx_waypoint_tools tcp-console \
  --workspace /home/huaxin/Codes/ur-waypoint \
  --env mla \
  --once
```

## 运动解锁与安全边界

默认只读。只有在启动时显式加入 `--enable-motion`，并在交互界面内再次
arm 后，程序才允许提交运动请求：

```bash
ros2 run hx_waypoint_tools tcp-console \
  --workspace /home/huaxin/Codes/ur-waypoint \
  --env mla \
  --enable-motion
```

下面的两阶段 MoveGroup 链路适用于终端离散 Jog 和 GUI 绝对 XYZ。GUI
长按 Jog 使用前述 MoveIt Servo 连续碰撞监控，不进入逐次 IK/规划链路。
两类运动始终互斥且不排队，均不使用 URScript；键盘移动保持当前杆尖姿态不变。

每条手动命令都严格采用下面的两阶段规划与执行链路：

1. 将用户给出的精确 `rod_tip` 目标按环境中保存的固定 TCP 几何精确换算为
   `tool0` 目标；
2. 调用 `/compute_ik`，以当前实际关节状态为 seed，启用碰撞检查，并携带当前
   附着杆；IK 解必须通过 canonical 关节分支检查，且每个关节都位于实际起点
   附近的 `±20°` 局部范围与环境关节约束的交集内；
3. 将该精确 IK 关节解交给 MoveGroup，但只做 `plan_only=true`。规划请求显式
   携带实际 `start_state`，以及完整 world scene、附着杆和未经放宽的 Allowed
   Collision Matrix（ACM）；
4. 严格验证 MoveGroup 返回的轨迹，包括关节集合、有限数值、时间单调性、首点、
   末点以及整条路径的 canonical/局部关节范围；
5. 在执行前重新读取并核对完整场景和实际关节起点。只有二者仍与本次规划一致，
   才调用 `ExecuteTrajectory`，原样提交并执行 MoveGroup 返回的同一条轨迹；
6. 执行完成后以实际关节 FK 重新计算 `rod_tip`，核对最初的精确杆尖目标。

IK 终点不会直接下发给控制器，MoveGroup 也不会在一个不可复核的
plan-and-execute 请求中直接启动机器人。上述任一步失败、数据过期或执行前状态
发生变化，都会拒绝本次命令且不进入后续执行；程序不会放宽、近似或自动修改目标，
也不会替换、重定时或改写已经验证通过的返回轨迹。

允许运动前，程序还需要获得新鲜且完整的关节状态和必要 TF，确认 MoveIt
planning scene 中逐项核对 MLA 所有碰撞盒的 frame、类型、尺寸和位姿，并核对
附着杆的 link、圆柱尺寸、中心位姿、方向和 touch links。若正式 waypoint runtime 的 playback
action 仍存在、预期物体/杆模型不完整、机器人未停稳或状态过期，运动会被
拒绝。程序也会拒绝放宽这些物体碰撞的 Allowed Collision Matrix，并把权威
碰撞盒和附着杆随本次 MoveGroup goal 一起提交。只读显示仍可继续。

默认小步为 `1 mm`，可用 `--jog-step-mm` 修改，但程序硬性拒绝超过
`5 mm` 的值。GUI 的“仅 XYZ、保持当前姿态”绝对移动相对当前位置默认最多
平移 `150 mm`，可用 `tcp-console` 参数 `--max-absolute-translation-mm`
（`tcp-gui` 也接受同名启动参数）收紧或调整；因此 `50–150 mm` 的绝对位置
目标会继续进入精确 IK 和完整碰撞
规划，而不是被小步门限提前拒绝，超过该独立上限仍会直接报错。终端完整位姿
命令仍默认最多平移 `50 mm`、旋转 `10 deg`，分别由
`--max-translation-mm` 和 `--max-rotation-deg` 控制。每个关节还默认只能在
实际起点附近 `20 deg` 内运动，并与 MLA 的
`shoulder_pan_joint ∈ [-π,0]` 安装侧约束取交集；可用
`--max-joint-excursion-deg` 收紧该局部范围。它们都是单次手动命令的门限，
不会改变 waypoint 文件或 recording。

软件规划和取消不能代替 UR 示教器急停，也不能感知未加入 planning scene
的动态障碍物。首次真机使用应先只读核对，再在清空场地、低速和有人持守
示教器的条件下，以最小步长逐轴试动。

## 交互操作

交互式 TTY 中，每次运动使用一次性 arm：

1. 按一次 Space；界面应显示 `ARMED FOR EXACTLY ONE MOVE`。
2. 按一个小步键，或按 `G` 后输入绝对位姿并按 Enter。
3. arm 无论在规划成功还是拒绝后都会自动清除；下一次运动必须重新输入
   Space。按 Esc 可主动解除；若动作正在运行，Esc 会请求取消。

小步命令如下。默认在 `plane1` 中解释，按 `T` 可在
`plane1` 与 `base` 之间切换；切换会自动解除已有 arm：

| 输入文本 | 位移 |
| --- | --- |
| `a` 或 `left` | 当前小步坐标系 -X |
| `d` 或 `right` | 当前小步坐标系 +X |
| `w` 或 `up` | 当前小步坐标系 +Y |
| `s` 或 `down` | 当前小步坐标系 -Y |
| `r` 或 PageUp | 抬高：`plane1 -Z`；若切到 `base` 则为 `base +Z` |
| `f` 或 PageDown | 降低：`plane1 +Z`；若切到 `base` 则为 `base -Z` |

界面会始终显示当前 jog frame、步长及 Z 方向提示。特别是使用默认
`plane1` 时，`R`/PageUp 对应 -Z，也就是物理上抬高、远离板面。

绝对位姿先按 `G`，随后输入的内容必须显式包含坐标系，不接受无 frame 的
六元组。目标始终描述 `rod_tip`：

```text
base p[x,y,z,rx,ry,rz]
plane1 p[x,y,z,rx,ry,rz]
```

XYZ 的单位是米，旋转向量单位是弧度。提交前应再次核对界面显示的目标
frame、杆尖目标和换算后的运动方向。示例：

```text
Space
G
plane1 p[0.000000, 0.000000, -0.070000, 0.000000, 3.141593, 0.000000]
```

其余按键：`C` 清除 UI 错误；Esc 解除 arm 或请求取消当前动作；`Q` 在确认
当前动作已结束后退出。Ctrl-C 也会请求取消并退出。实际停止仍以 UR 控制器
和示教器安全状态为准。

非 TTY 行模式保留兼容命令：`arm`、`a/d/w/s/r/f`、`left/right/up/down`、
`pageup/pagedown`、`g base p[...]`、`g plane1 p[...]`、`t`、`c`、`q`，每条
都需按 Enter；行模式不会在等待输入时连续刷新。
