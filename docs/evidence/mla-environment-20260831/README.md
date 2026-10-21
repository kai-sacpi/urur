# MLA 环境几何验收（2026-08-31）

本证据对应环境 [`environments/mla`](../../../environments/mla/)，母环境为
`new_board_4cm_rod33365_up50_icassp_phase2`。

## 变换审计

采用旧 Feature 局部坐标中的唯一偏移：

```text
^Feature t_plane1 = [0, -0.025, 0] m
```

平移前后 `base_link -> plane1` 原点分别为：

```text
old = [0.065280000000000, 0.469730000000000, 0.182440000000000] m
new = [0.082436500073763, 0.487913866910669, 0.182401407331770] m
delta = [0.017156500073763, 0.018183866910669, -0.000038592668230] m
```

旋转矩阵逐元素最大差为 `0`。上述 `base_link` 增量不是直接的全局 Y
减 25 mm，而是旧 Feature 的倾斜局部 `-Y` 经过其姿态旋转后的结果。

## 碰撞体差分

| 对象 | 物理结果（相对母环境） |
|---|---|
| `sensor_board_keepout` | 随 `plane1` 同一 `delta` 移动；尺寸 `0.40 × 0.40 × 0.167 m` 不变 |
| 五块 `base_ground_*` | `base_link` 中位置、姿态、尺寸完全不变 |
| `x_positive_limit_wall` | resolved `base_link` 位姿不变；局部 Y 补偿为 `0.025 m` |
| `y_positive_limit_wall` | resolved `base_link` 位姿不变；局部 Y 补偿为 `0.730 m` |
| `top_negative_z_panel` | resolved `base_link` 位姿不变；局部 Y 补偿为 `-1.075 m` |

MoveIt 与 cuRobo 对九个碰撞盒的 resolved `base_link` 尺寸、位置和姿态逐项
一致。传感器分析平面与 keepout 仍在 `plane1`，因此也随新坐标系移动。

## 可重复验证

```bash
PYTHONPATH=src/hx_waypoint_curobo:src/hx_waypoint_tools \
  .venv/bin/pytest -q tests/test_mla_environment.py \
    tests/test_environment_and_tool.py \
    tests/test_up50_phase2_environment.py \
    tests/test_new_board_environment.py \
    tests/test_planning_scene.py
```

结果：`102 passed`。

完整 YAML 解析和 Python `compileall` 也通过。

## Ubuntu 24 ROS/C++ 验证

在 `ubuntu24` 上用当前源码重新构建了全部 6 个 ROS 包；随后 C++ runtime 的三个
测试目标全部通过：

```text
test_runtime_core          passed
test_simulation_contract   passed
test_setup_config          passed
```

`test_setup_config` 明确覆盖可选偏移的零值兼容、Feature 局部偏移经姿态旋转后的组合，
以及错误长度、非有限值和 `null` 的 fail-closed 拒绝。随后从主工作区启动 MLA mock
simulation，实测结果为：

```text
cuRobo server ready: environment=mla
MoveIt scene: 9 collision cuboids + attached rod
RViz markers: 3 non-collision calibration planes
base -> plane1 translation:
  [-0.082436500074, -0.487913866911, 0.182401407332] m
base -> plane1 quaternion xyzw:
  [0.369218985408, 0.929339773605, -0.000051001500, 0.002218875842]
```

仿真在观察窗口内正常保持运行，随后由验收脚本主动停止；日志中唯一的 MoveIt
`No 3D sensor plugin(s) defined for octomap updates` 是未配置 Octomap 深度传感器的既有
提示，与本环境的静态碰撞场无关。Ubuntu 24 的 RViz 命令见
[`docs/environment-mla.md`](../../environment-mla.md)。
