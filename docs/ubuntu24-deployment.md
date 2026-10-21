# Ubuntu 24.04 部署

本文只描述可复现部署和放行条件。脚本不会安装或升级 NVIDIA driver，也不会自动修改目标 UR 的标定状态。

## 已验证远端基线与剩余阻断项

目标主机：

```text
SSH alias: ubuntu24
repository: /home/huaxin/Codes/ur-waypoint
OS target: Ubuntu 24.04
ROS: /opt/ros/jazzy
Python: 3.12
CUDA_HOME: /usr/local/cuda
```

当前已知版本：

```text
ubuntu24 NVIDIA driver: 570.172.08
CUDA 12.8 GA Linux minimum driver: 570.26
```

根据 NVIDIA 官方 CUDA 12.8 GA 发行表，Linux x86_64 对应驱动为 `>=570.26`，因此当前为 **DRIVER_VERSION_READY**。此结论只放行后续 GPU 验证，不替代运行时检查。执行 `build-recording` 前仍必须：

1. 执行 `nvidia-smi --query-gpu=driver_version --format=csv,noheader`，确认所有可见 GPU 都满足锁文件版本。
2. 确认 `CUDA_HOME=/usr/local/cuda` 且 `${CUDA_HOME}/bin/nvcc` 报告 CUDA 12.8；不使用系统 PATH 中另一个 `/usr/bin/nvcc`。
3. 在项目 Python 3.12 + ROS 2 Jazzy 环境中确认 `rclpy`、PyTorch CUDA 和锁定 cuRobo 可联合导入。
4. 启动 cuRobo server，调用 warmup 服务，只有返回 `success: true` 才继续。

NVIDIA 发行表：<https://docs.nvidia.com/cuda/archive/12.8.0/cuda-toolkit-release-notes/index.html#cuda-toolkit-and-corresponding-driver-versions>

2026-07-20 已在 `ubuntu24` 上对 `setup.yaml` 单一装配源迁移后的代码完成以下验证：

- ROS 2 Jazzy/CUDA 12.8 环境中的 `horizontal_line_mj`、`shallow_s_mj`、`circle_mj_v2` 三条权威输入均完成 strict GPU compile、mock recording、`inspect` 和全新独立 mock runtime 的 `play --auto-prepare`。
- 三条分别为 `33 s / 3301`、`33 s / 3301`、`63 s / 6301` canonical 点；均精确保留 122/122 source waypoint、source ROS 纳秒时间误差为 0、`uniform_time_scale=1.0`、源 SHA-256 不变。
- 独立 playback telemetry 分别为 4125、4126、7882 行，`speed_scaling` 全程精确为 `1.0`；最后 3 s 均有 301 个严格静止 canonical 点，位置不变且速度、加速度精确为 0。
- 三份 recording 都绑定相同 `setup.yaml` hash；实时 `base → plane1`、alias、`tool0 → rod_tip` TF 与 MoveIt/curobo 同源场景已在独立 `ROS_DOMAIN_ID=77` 下验证。旧 recording 因缺少新的 setup hash 被拒绝。
- 复用长期 runtime 时的一次 cuRobo prepare CUDA 故障和 `circle_mj_v2` 首试超过固定完成门限 `0.847624 ms` 的失败均原样保留；没有调阈值、裁剪 telemetry、缩放时间或修改轨迹。prepare 故障已通过禁用 MotionPlanner CUDA graph 修复，同一 ROS service 连续两次 GPU prepare 已通过。
- 20 条 legacy 轨迹的首轮矩阵结果为 8 条 endpoint-rest、9 条 fixed-time dynamics、1 条连续 IK frontier 失败和 2 条 15 分钟预算中断。timeout 只表示本次预算未完成，不表述为不可达。
- 最终代码回归为 5 个 ROS 包构建成功，C++ GTest `70 + 13 = 83` 个 case 全部通过（`colcon test-result`：85 tests、0 failure），Python pytest `314/314`；legacy matrix 既有 `--validate-only` 20/20 数据行证据继续保留。
- 最新完整报告、失败尝试和 checksum 证据见 [`docs/evidence/ubuntu24-20260720/`](evidence/ubuntu24-20260720/README.md)；2026-07-18/19 包作为迁移前历史证据保留。

2026-08-03 的 NUCLEO sidecar 扩展回归为 6 个 ROS package 构建成功、C++
`85 tests`、Python `400 tests`。使用 50 Hz pseudo-terminal NUCLEO 对已验证的
`horizontal_line_mj` 做完整 mock playback，机械臂/传感器表分别为 4126/1651 行，全部
传感器 `sync_time_ns` 可在机械臂表中精确命中，最大映射偏差 `4.100515 ms`，最大帧间隔
`24.675714 ms`，正式窗口 parser 完整性增量全为 0，CSV hash 与 manifest 一致且 staging
已删除。证据目录为
`runs/horizontal_line_mj__20260720_031210__20260803_155914/simulation/`。该验证证明软件同步
链路，不替代真实串口板和真机硬件在环验收。

真机仍未放行：当前环境为 `calibrated_kinematics.yaml: deployment_allowed: false`；目标 UR 身份、真实标定、driver/controller、TCP/payload、安全配置和硬件在环验收同样未关闭。drawing-plane/contact policy 已进入逐样本几何门禁，但上述远端结果仍只验证构建、测试、GPU 编译、mock recording 和 mock playback，不是真机绘制验证。

## 本地同步

在本地仓库根目录：

```bash
./scripts/sync_to_ubuntu24.sh
```

脚本特性：

- 默认目标为 `ubuntu24:/home/huaxin/Codes/ur-waypoint/`。
- 在连接和写入远端前请求确认；`--yes` 可用于受控的非交互执行。
- 不使用 `--delete`。
- 排除 `.git`、`.venv`、`build/install/log`、缓存、`runs` 和现有 recording，避免覆盖远端产物。
- `--dry-run` 只显示 rsync 变化，但仍会建立 SSH 连接。

示例：

```bash
./scripts/sync_to_ubuntu24.sh --dry-run
./scripts/sync_to_ubuntu24.sh --yes
./scripts/sync_to_ubuntu24.sh \
  --remote ubuntu24 \
  --dest /home/huaxin/Codes/ur-waypoint
```

## 初始化 Python/CUDA/cuRobo

```bash
ssh ubuntu24
cd /home/huaxin/Codes/ur-waypoint
export CUDA_HOME=/usr/local/cuda
./scripts/setup_ubuntu24.sh
```

`setup_ubuntu24.sh` 做以下工作：

1. 拒绝已 source 其他 ROS 发行版的 shell，然后只 source `/opt/ros/jazzy/setup.bash`。
2. 校验 Python 3.12 和 lock 中的 `cp312`。
3. 要求 `CUDA_HOME` 最终严格为 `/usr/local/cuda`，并检查 CUDA toolkit 为 12.8。
4. 创建 `.venv`：

   ```bash
   python3.12 -m venv --system-site-packages .venv
   ```

   `--system-site-packages` 用于让 venv 继续访问 Jazzy 安装的 `rclpy` 等系统包。

5. 从 `curobo.lock` 读取固定 repository、commit 和 Git `source_tree`。
6. 在下载前明确提示将访问 PyPI、PyTorch cu128 index 和锁定的 cuRobo Git repository；PyTorch wheel 精确固定为 `2.11.0+cu128`。
7. 在临时 Git 仓库中仅 fetch 固定 commit，detached checkout 后验证 `HEAD` 等于锁定 commit、`HEAD^{tree}` 等于 `369c492056ba8a735c00f11342062dfb22af2d90`，再原子移动到 `.deps/curobo-<commit>`；不再依赖 GitHub 自动生成 tarball 的不稳定字节级 SHA。
8. 源目录必须是 origin 匹配、commit/tree marker 匹配且 `git status --porcelain --untracked-files=all` 为空的独立 Git checkout；editable install 后再次检查洁净状态，任一条件不满足立即失败。
9. 通过 `rosdep` 安装 `package.xml` 声明的 ROS/系统依赖（`ament_python` 是 build type，显式从 rosdep key 中排除），再安装项目锁定的 Python 依赖和 `torch==2.11.0+cu128`；cuRobo 以 editable `.[cu12-torch]` 语义安装，随后只对 cuRobo 的必需依赖做版本闭包检查，避免 Ubuntu 系统 Python 中与本项目无关的可选包使全局 `pip check` 产生假失败。
10. GPU 门禁通过时，验证 cuRobo 确实从锁定 source 导入、`torch.__version__ == "2.11.0+cu128"`、`torch.version.cuda == "12.8"`、`torch.cuda.is_available()` 为真，并确认 `cuda-core` distribution 与 `cuda.core` 模块已安装。
11. 读取 NVIDIA driver 并给出 READY/BLOCKED 结论；`rosdep` 可调用系统包管理器安装项目依赖，但脚本绝不请求 NVIDIA driver/CUDA toolkit、调用 DKMS、加载内核模块或自动重启。

Jazzy 的生成式 `setup.bash` 可能在 `AMENT_TRACE_SETUP_FILES` 尚未定义时读取该变量。所有部署脚本都在 source ROS/overlay setup 前临时执行 `set +u`，source 返回后立即恢复 `set -u`。修改脚本后应运行：

```bash
./scripts/check_deployment_scripts.sh
```

该检查会验证 Bash 语法、可执行位、所有实际 `source` 的 nounset guard，并用一个会读取未定义 `AMENT_TRACE_SETUP_FILES` 的临时 setup 文件做回归。

非交互部署必须显式表明已审核网络步骤：

```bash
./scripts/setup_ubuntu24.sh --yes
```

## 生成 UR5 URDF

cuRobo 配置引用：

```text
environments/lab_wall_a/robot/assets/ur5.urdf
```

从 Jazzy 的 `ur_robot_driver`/`ur_description` xacro 生成：

```bash
./scripts/generate_ur_assets.sh --env lab_wall_a --ur-type ur5
```

脚本只接受环境内受控的 `environments/lab_wall_a/robot/driver_kinematics.yaml`，不提供任意 `--kinematics` 路径入口。当前该文件是名义 factory model，足以支持 mock 仿真和前期集成，但不构成真机标定证明。

拿到目标 UR 通过官方流程提取的 kinematics YAML 后，必须先由责任人审计并替换环境内的受控文件，再更新 `calibrated_kinematics.yaml` 中的 SHA-256，随后运行上述生成命令。脚本会重建 `robot/assets/ur5.urdf` 和 `asset_provenance.yaml`，把受控标定源 hash 与生成 URDF hash 绑定。只有来源、机器人身份和生成资产全部复核后才能把 `deployment_allowed` 设为 `true`；不能仅为让命令通过而修改它。

## 构建与测试

```bash
./scripts/build_workspace.sh --test
```

脚本不访问网络。它会：

- source `/opt/ros/jazzy/setup.bash`；
- 激活现有 `.venv`；
- 在 URDF 缺失时调用 `generate_ur_assets.sh`；
- 执行 `colcon build --symlink-install --base-paths src`；
- `--test` 时对 C++ runtime 执行 `colcon test`/GTest，再运行仓库级 Python pytest；这样不会把无包内测试的 ament Python 包误判为 pytest exit 5 失败。仓库 pytest 会禁用未使用的系统级插件自动加载，避免 ROS Jazzy 的旧 `launch_testing` hook 与较新 venv pytest 冲突。

最终已验证基线的结果为：5 个 package 构建成功、C++ GTest `70 + 13 = 83` 个 case 通过（`colcon test-result` 汇总 85 tests）、Python pytest `314` 项通过。

构建完成后，新终端加载顺序固定为：

```bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
export PYTHONNOUSERSITE=1
source install/setup.bash
```

源 CSV 的所有 waypoint 和通过整数纳秒及 binary64 无损往返检查的 `time_s` 是不可修改的权威输入。canonical recording 只含 `draw/hold`，必须从 source `t=0` 开始、在源末时刻结束，并逐点精确保留每个 source waypoint 的纳秒时戳与 phase。部署环境不提供轨迹缩放、重定时、裁剪、改点或跳点开关；上游必须在部署前提供不过快、加速度与 jerk 合理且动力学可行的权威时标轨迹。编译器只校验权威输入，任一 IK、碰撞、关节限制、速度、加速度或 jerk 问题都直接报错，不自动修复。

## GPU warmup 放行检查

只有 driver、CUDA、依赖和构建均通过后，才执行：

终端 A：

```bash
ros2 run hx_waypoint_curobo curobo_server \
  --ros-args \
  -p workspace:=/home/huaxin/Codes/ur-waypoint \
  -p environment_id:=lab_wall_a
```

终端 B：

```bash
ros2 service call /hx_waypoint/curobo/warmup std_srvs/srv/Trigger '{}'
```

只有响应 `success: true` 才能继续 `build-recording`。warmup 失败时保留日志并停止，不得通过关闭碰撞、切换 CPU 假实现或降低校验标准来伪造通过。

已验证的 strict GPU `circle_01` compile report 为：

```text
duration_s: 20.925
point_count: 2252
source_waypoint_count / matched_source_waypoint_count: 10 / 10
uniform_time_scale: 1.0
max_joint_velocity_ratio: 0.061837
max_joint_acceleration_ratio: 0.465459
max_joint_jerk_ratio: 0.737863
minimum_collision_distance_m: 0.011255483
joint_winding.selected_seed_offsets_rad: [0, -2pi, 0, -2pi, -2pi, -2pi]
```

该报告的 canonical 轨迹精确从 source `t=0` 到源末时刻，只含 `draw/hold`。比例超过 `1.0` 时必须失败，不得通过缩放或重定时使其通过。

## 真机放行检查

真机 `play` 前至少确认：

- [ ] `calibrated_kinematics.yaml: deployment_allowed: true`，且来源确为目标机器人。
- [ ] recording 为 `status: validated`，`ros2 run hx_waypoint_tools tipdraw inspect` 成功。
- [ ] 当前环境配置与 recording snapshot/hash 完全一致。
- [ ] 如果安装过新标定，或修改过机器人、TCP、杆体、`zones.yaml`/contact policy，已经重新 GPU compile、mock recording、validate、inspect 和封存；没有复用旧环境哈希下的 artifact。
- [ ] 真机 UR driver 提供 `/passthrough_trajectory_controller/follow_joint_trajectory`。
- [ ] 没有启用 controller fallback。
- [ ] `/speed_scaling_state_broadcaster/speed_scaling` 持续发布且为 `1.0`；正式播放中任何偏离都会取消轨迹。
- [ ] 当前关节状态新鲜、速度低于 settle 阈值，并靠近 recording 起点。
- [ ] 如需 `--auto-prepare`，确认环境策略为 `curobo_prepare`；prepare 必须作为 recording 之外的独立 goal 执行，其间 telemetry recorder 保持未启动，完成后再复核 source `t=0` 状态。
- [ ] 示教器处于允许远程控制的模式，急停/保护停机功能正常。
- [ ] 工作区清场；第一次执行使用上游生成的低速、低加速度权威轨迹并由人员现场监护。速度滑块仍须保持使 `speed_scaling=1.0`，不得用滑块降速替代合法时间轴。
- [ ] 如启用 `--record-nucleo`，优先使用 `/dev/serial/by-id/...` 稳定设备路径；当前用户必须具有该设备的读写权限（Ubuntu 常见为 `dialout` 组），并在机械臂动作前看到 SYNC/ACK 和首个完整 frame 预检成功。

只要其中任一项无法证明，就不执行真机播放。
