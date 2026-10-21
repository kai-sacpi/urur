# 分层与全高度连续轨迹候选库

用户最新定义：**先做覆盖不同高度层的轨迹，再另做贯穿全高度的轨迹；XY 可随机。**
本库采用固定随机种子 `2026101607`，与原 96 条 active v4 完全独立。
主交付是 Ubuntu24 的 `/home/huaxin/Codes/ur-waypoint/workstation_magtrack/continuous_patterns/`；Mac 同路径项目下是同步副本。

## 本批结构

版本：`cp_layers_and_full_h070_200_20261016_v1`，共 156 条。

| 分组 | 条数 | 高度与几何 |
| --- | ---: | --- |
| `stratified` | 56 | 70、80、…、200 mm 共 14 个固定高度层；每层两种不同随机 XY 几何，每种各有固定/变姿态对照 |
| `full_height` | 60 | 10 种 XY 几何，各用 V1/V2/V3 与固定/变姿态，连续贯穿 70–200 mm |
| `full_height_reverse` | 20 | 上述 10 种几何 V2 的两种姿态，完整时间反向对照 |
| `guarded_full_height` | 20 | 相同 XY、独立新 ID 的 75–200 mm 保守下界对照；明确缺失 70–75 mm |

每层两种 XY 分别随机中心、尺寸、平面旋转、曲线相位及家族参数；保存所有参数与 seed。
10 家族包括编织 Lissajous、三叶结投影、2:5 环面结投影、内旋轮线、径向编织螺旋、球面扫掠投影、复合 harmonograph、弯管编织、剪切圆角方形及 Celtic weave。
固定层是平面轨迹，不能声称非零挠率；非平面性、曲率、挠率及带高度差的 XY 投影交叉按实际 3D 几何统计。

同一几何跨速度/姿态/反向对照保持明确配对。完整家族作为数据切分组；本库均为已查看的离线设计开发候选，不是盲测集或真实 session。

## 坐标与物理边界

环境 `mla_short`，UR5；名义 190 mm 长、5 mm 直径杆沿 `tool0` 局部 +Z 到 `rod_tip`。
CSV 在 `plane1` 中，米、秒、四元数 xyzw；`h=-tip_z_m`。XY 全曲线解析包络均在 ±130 mm 内。
未应用验收后的 `R_S_B,t_S_B,r_F_C`，磁化轴、有效 TCP、安装 ID 与 `plane1↔S` 注册也未验收。
所有 pose 仅是计划名义杆尖/工具轴，不能作为正式磁铁中心真值或实际反馈。

60/70 mm 曾被旧标定预检拒绝；本库保留最新 70 mm 请求。现有保护面 z=-60 mm、分析边界 z=-65 mm、非尖端最小间隙 5 mm、暂定位置误差 3 mm 与朝向误差 2° 均保持。
`src/clearance.py` 对整根名义刚性杆、现有板保护面及外部 cuboid 做保守 CPU 子集预检；不含机器人手臂、自碰、IK、关节动力学和进出场。**子集预检通过不能代表机器人碰撞通过。**
70 mm 候选的拒绝与 75 mm 对照分开保存，不能用后者掩盖请求范围的缺口。

## 源轨迹与时间合同

CSV 表头：

```csv
time_s,frame_id,tip_x_m,tip_y_m,tip_z_m,tip_qx,tip_qy,tip_qz,tip_qw,phase
```

20 Hz 权威源网格，首末各 0.5 s 严格静止；第一行 `draw`，所有 `hold` 行七个 pose 字段与前一行逐字节相同。
源 CSV 一旦写入即不可变。生成器仅允许重写逐字节相同的内容，差异内容直接报错；修订需新版本、新 ID、新文件与哈希。
禁止 runtime 缩放、重定时、裁剪、跳点或自动降速；以后固定时间编译须保留所有源时刻与 pose。

V1/V2/V3 是本连续曲线的**完整矢量中心速度峰值** 10/20/50 mm/s；报告另列均值、P95 和实际设计峰值。
变姿态 W1/W2/W3 峰值 3/10/20 °/s，围绕多种横向轴改变工具 +Z 方向，独立于位置切线；不是纯轴向自转。固定姿态 W0 单独标记。

XY 使用解析曲线及七次 C3 进度；固定层高度恒定。贯穿组用每个高度箱内的七次 C3 升降相连，XY 在箱界继续运动，位置速度/加速度/jerk 连续。
完整全高度运动时间中 15–20 cm 占 60%，五个 1 cm 箱各占 12%；其余 40% 分配到 7–15 cm。75 mm 对照低区为 7.5–15 cm。
全记录包含首末 hold 的时间比例、固定层比例及各速度档均由报告重新统计，不省略 hold。

每条都检查完整向量中心加速度 ≤0.05 m/s²、jerk ≤0.25 m/s³，以及法兰角加速度 ≤30 °/s²、角 jerk ≤150 °/s³。
连续证书采用解析链式法则、全局下一阶导数界与网格半间隔 Lipschitz 余量，包含向心项；CSV 独立差分另验。
首末位置/姿态速度、加速度、jerk 为零。时间在 CSV 创建前设计，保留完整路线，不裁剪到 20 s。
高频小尺度几何若不能保留 50 mm/s 峰值，则在源生成前明确调整整条几何；初始参数、选定参数、尝试记录和原因均保存在元数据，无 runtime 改速。

## 文件与复现

```text
config/contract_v1.json                  本库数值合同与seed
src/patterns.py                         10家族与0–4阶解析导数/全局界
src/trajectory.py                       高度、时间、姿态及连续证书
src/clearance.py                        有粗测误差的整杆/障碍子集预检
generate.py                            不可变CSV、JSON与独立manifest
validation/audit.py                     独立CSV、配对、覆盖与几何审计
visualization/plot_batch_v1_1.py        CSV驱动的3D图、剖面与离线图集
delivery/verify_delivery.py             交付哈希、图集链接与旧96条源文件核验
design_notes/geometry_sources.md         三份既有生成代码的来源与公式说明
batches/<version>/csv/                  156条权威源
batches/<version>/metadata/             全部随机/几何/时间/方向参数
batches/<version>/manifest.json          独立ID/路径/哈希清单
batches/<version>/provenance/             源代码和输入合同/环境快照与哈希
batches/<version>/reports/               源审计、层覆盖、碰撞拒绝与复现证据
batches/<version>/plots/index.html       可筛选的本地图集，关联PNG/CSV/JSON
```

Mac 从项目根运行（纯 CPU）：

```bash
.venv/bin/python workstation_magtrack/continuous_patterns/generate.py --plan-only
.venv/bin/python workstation_magtrack/continuous_patterns/generate.py
.venv/bin/python workstation_magtrack/continuous_patterns/validation/audit.py \
  --root workstation_magtrack/continuous_patterns \
  --manifest batches/cp_layers_and_full_h070_200_20261016_v1/manifest.json
.venv/bin/python workstation_magtrack/continuous_patterns/visualization/plot_batch_v1_1.py \
  --batch workstation_magtrack/continuous_patterns/batches/cp_layers_and_full_h070_200_20261016_v1
```

Ubuntu24 的现有 Python 支持 NumPy/SciPy，可独立重跑源审计；不更改原环境依赖。
该机旧 Matplotlib ABI 不匹配，因此图在 Mac 生成后随权威源同步。权威 CSV 的哈希在 Ubuntu24 重新核验。
冻结生成源和原始输入均随批次保存；固定种子同环境复现应逐字节相同。跨 Python/NumPy/libm 的末位差异不能自动覆写权威文件。
依赖版本与复现结果见批次 provenance 和 reports。
绘图修订 1.1 修正为读取权威 CSV 的 `tip_qx/tip_qy/tip_qz/tip_qw` 列；修订源和哈希另存于
`provenance/visualization_revision_1_1/`，生成时冻结的原始绘图源继续保留。图集逐条核对源公式、CSV 与哈希。

## 本批结果与入口

- [可筛选图集](batches/cp_layers_and_full_h070_200_20261016_v1/plots/index.html)：156 条 3D 图与运动剖面，可按分组、家族、速度、姿态和状态筛选。
- [分层总览](batches/cp_layers_and_full_h070_200_20261016_v1/plots/fixed_height_layers_overview.png)；[全高度总览](batches/cp_layers_and_full_h070_200_20261016_v1/plots/full_height_overview.png)。
- [中文交付说明](batches/cp_layers_and_full_h070_200_20261016_v1/reports/delivery_summary.md)；[Ubuntu24 源审计](batches/cp_layers_and_full_h070_200_20261016_v1/reports/source_audit.md)。

Mac 与 Ubuntu24 源审计均为 156/156 通过；同生成环境从保存参数重建 156 份 CSV，逐字节一致。
现有杆体子集预检 72 条通过、84 条拒绝；全部包含 70 mm 的候选均保留拒绝状态。
全高度组运动时间在 150–200 mm 约占 60%，五个 1 cm 箱各约 12%；合并固定层后，
全部候选运动时间高区占 59.754%，五箱为 9.705%、12.164%、9.932%、12.739%、15.214%，并不均匀。
全库是约 11.08 小时的候选运动数据，不是安排连续执行的日程。

## 当前验证范围

只做纯离线 CPU 生成、源合同/数学/几何预检与可视化。
GPU compile、mock build、inspect、独立机器人 playback、IK/关节/全机器人碰撞/进出场均未运行。
无 ROS domain、真机、NUCLEO、背景采集；不提交或推送。本库没有进入旧 96 条 active 清单。
具体通过数、拒绝数、7 cm 缺口、曲率/挠率/交叉、速度与高度时间覆盖以批次源审计报告为准。
