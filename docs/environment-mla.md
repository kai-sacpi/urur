# `mla` 环境

`mla` 采用 UR 示教器 Installation 中的 Plane Feature `new_board_mla` 作为
标定来源，并将 `plane1` 原点定义在已确认的传感器板中心。

## 工作坐标系

2026-09-04 通过 UR Dashboard 读取到：

```text
new_board_mla:
p[-0.08533, -0.48683, 0.24284, 1.16388, 2.91348, 0.00029]
```

环境采用：

```text
T_base->plane1 = T_base->new_board_mla * Trans(0, 0, +0.053 m)
feature_to_work_frame_translation_m = [0, 0, +0.053]
```

对应四元数 `xyzw`：

```text
[0.3709743428036357,
 0.9286406917135243,
 0.00009243440854130525,
 0.002119936583471941]
```

因此 `plane1 [0,0,0]` 就是传感器板中心，也是所有新 waypoint 使用的工作空间
原点。平移在 Feature 局部坐标中组合；没有坐标缩放、姿态纠偏或墙面派生变换。

## 传感器和保护区

传感器板中心的权威位置为：

```text
plane1:   [0, 0, 0] m
base_link:[0.085117687282, 0.486904263995, 0.189840477284] m
```

`plane1 +Z` 几乎指向物理下方，`plane1 -Z` 指向物理上方。碰撞区覆盖完整
`0.40 x 0.40 m` 板面：

```text
局部 -Z 面：-0.060 m = 从传感器中心向上 0.060 m
局部 +Z 面：+0.19096485659512854 m
```

`+Z` 上界由新标定姿态、40 cm 方形板的四个角和 `base_link z=0` 桌面共同
计算。因为平面约有轻微倾斜，不能只使用中心线距离 `0.242842 m`；采用
`0.19096485659512854 m` 可保证整个下表面都到达或轻微进入已建模桌面，避免
角落留下缝隙。最终传感器碰撞柱为：

```text
dims: [0.40, 0.40, 0.25096485659512857] m
center in plane1: [0, 0, 0.06548242829756427] m
```

分析面同时更新为：

- 实际传感器面：`plane1 z=0`；
- 保护区上表面：`plane1 z=-0.060 m`；
- 非杆尖最小间隙仍为 `5 mm`，分析门限位于 `plane1 z=-0.065 m`。

## 其他碰撞体

`base_link`、UR5 模型和杆具均未改变。除了传感器碰撞柱以外，以下碰撞物的
尺寸和物理 `base_link` 位姿与重标定前 MLA 完全一致：

- 四块地面区域；
- `x_positive_limit_wall`；
- `y_positive_limit_wall`；
- `top_negative_z_panel`。

三块固定边界现在直接用 `base_link` 位姿保存。以后再次改变 `plane1` 时，它们
不会跟着工作坐标系误移动。原来填在基座 cutout 内、与地面重合的
`base_ground_fixed_base_support` 已删除；真实 UR5 基座仍由机器人模型表示。

## 文件与运行

- [`setup.yaml`](../environments/mla/setup.yaml)：原始 `base -> new_board_mla`
  标定与 Feature 局部 `+53 mm` 的传感器中心偏移；
- [`environment.yaml`](../environments/mla/environment.yaml)：传感器中心、桌面和
  保护区权威元数据；
- [`zones.yaml`](../environments/mla/zones.yaml)：MoveIt/cuRobo 共用碰撞体；
- `trajectories/` 和 `recordings/` 仍只保留 `.gitkeep`。旧环境轨迹/回放的 TF 和
  environment hash 不同，禁止直接复制改名。

Ubuntu24 查看 RViz：

```bash
cd /home/huaxin/Codes/ur-waypoint
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
export PYTHONNOUSERSITE=1
source install/setup.bash
export ROS_DOMAIN_ID=77
ros2 launch hx_waypoint_bringup simulation.launch.py \
  ur_type:=ur5 \
  environment_id:=mla \
  workspace:="$PWD" \
  launch_rviz:=true
```

新轨迹仍必须逐点保留 waypoint 和 `time_s`，运行时禁止缩放、重定时、裁剪或
跳点。
