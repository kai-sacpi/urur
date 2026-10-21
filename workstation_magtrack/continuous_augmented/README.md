# 开放轨迹的平滑随机增强

本目录实现用户 2026-10-16 的新要求：不采用花纹式闭合路线，不反复绕圈或重复走同一路线；在已有直线、S 弯、C 弯和管内开放路线的思路上，随机平滑改变空间形状、速度和姿态。

主交付目录是 Ubuntu24：`/home/huaxin/Codes/ur-waypoint/workstation_magtrack/continuous_augmented`。Mac 同目录仅用于生成、绘图和同步备份。旧 `continuous_patterns` 的 156 条候选及其源文件保留，旧批量回放任务已停止，不是本版本的执行清单。

## 当前增强库：240 条，两个独立批次

用户进一步要求增加数量，并指定新产物只存 Ubuntu24。新增 `ca_open_elastic_batch120_20261016_v3` 已直接在 Ubuntu24 生成 120 条并通过独立源检查：84 条固定高度、36 条全高度单程，63,177 个 waypoint，合计 3152.85 s，单条 21.2–34.85 s。与 v2 及 12 条预览的 ID、实际 CSV 哈希、种子和子随机流均无交集；空间、速度与姿态控制场也没有精确重复。这不是独立统计数据集或机器人验收声明。

两批增强库合计 **240 条**（168 条固定高度、72 条全高度单程），总权威时长 6306 s（1 小时 45 分 6 秒）。连同原标定/同步/联调/Pilot 96 个文件，正式源集合共 **336 个**；旧花纹库和 12 条预览不计入。新增批次的 CSV、元数据、源检查、回放及日志只在 Ubuntu24 保存，不回拷到 Mac。

新生成代码为 `generate_extension.py`。它复用不变的生成数学，显式读取 v2 中经过 SHA 校验的历史参考副本，记录其真实路径，不假装它们是 Ubuntu24 父仓库的当前版本；全部实际生成代码和名义环境另存快照。远端可使用原参数重新生成相同字节（不是重设计）：

```bash
cd /home/huaxin/Codes/ur-waypoint/workstation_magtrack
../.venv/bin/python continuous_augmented/generate_extension.py \
  --output continuous_augmented/batches/ca_open_elastic_batch120_20261016_v3 \
  --seed 20261026120
```

v3 的回放构建入口为 `continuous_augmented/replay_v3/run_campaign.py`，独立计划写入 `replay_v3/planning/`，运行状态为 `runs/ca_batch120_20261016_v3/`。已于 2026-10-16 21:47:33（北京时间）在 tmux `wm_ca120_v3` 排队，当前为 `waiting_for_prior_campaign`，尚未启动任何 v3 GPU/ROS 阶段；无需重复启动。它必须等待 v2 整批共享锁释放，再重新核验源/环境后开始；不能在两批之间并发占用 mock runtime。v3 源报告为其 `reports/source_audit.*`、`cross_bank_diversity.*`；回放完成状态仍须看独立的 `reports/replay_index.*`，不以源检查代替。

## 首个 120 条批量任务（v2）

用户确认 12 条预览后，新增独立批次 `ca_open_elastic_batch120_20261016_v2`，旧预览和旧花纹源均保留，不覆盖：

- 84 条固定高度：70–200 mm，每 10 mm 一层，每层 6 个开放路线族。
- 36 条全高度单程：6 个路线族各 6 个随机变体，上行、下行各 3。
- 全部为平滑多轴姿态；空间、速度、姿态与时长使用独立随机流。共 63,183 个 waypoint，合计 3153.15 s（52 分 33.15 秒），单条 21.1–32.4 s。

120/120 已通过 Mac 与 Ubuntu24 的独立源检查，同参数完整重跑逐字节一致。源检查不等于机器人验收。2026-10-16 21:25:58（北京时间）已在 Ubuntu24 启动隔离仿真批处理，tmux 为 `wm_ca120_v2`，runtime batch 为 `ca_batch120_20261016_v2`。先做 7 cm、全高度上行、下行三条短 canary 的完整四阶段，再处理剩余 117 条；先编译，再执行 mock，避免编译负载干扰记录。

查看当前状态（不重新启动任务）：

```bash
cd /home/huaxin/Codes/ur-waypoint/workstation_magtrack
tail -n 5 runs/ca_batch120_20261016_v2/campaign.log
less runs/ca_batch120_20261016_v2/status.csv
```

整批完成须同时满足 `runs/ca_batch120_20261016_v2/campaign_summary.json` 的 `state="complete"`、`workflow_passed=true`、`complete_four_stage_passed=120`、`ready_on_this_host=120`，且所有阶段退出码为 0。回放索引位于本批 `reports/replay_index.json` / `.csv`，逐条核验四阶段、封存校验、实际证据哈希与同控制器时间 witness；pending 或失败项不可当作可用回放。任务进行中不代表已完成 120 个回放。

运行中请勿重复执行以下恢复命令；仅在确认原任务已退出后使用，相同 batch 会校验哈希后恢复：

```bash
cd /home/huaxin/Codes/ur-waypoint/workstation_magtrack
CUDA_VISIBLE_DEVICES=0 ../.venv/bin/python -u continuous_augmented/replay/run_campaign.py \
  --workspace "$PWD" --batch ca_batch120_20261016_v2 --jobs 2 --compile-timeout 3600
```

## 已确认的首批形态预览

首批 `ca_open_elastic_preview_20261016_v1` 为 12 条：6 条固定高度（70、90、120、150、180、200 mm）和 6 条单程贯穿 70–200 mm 的轨迹。全高轨迹交替向上、向下，不通过反复升降或绕圈增加时长。

- 空间：低阶控制场的平滑弹性形变、局部伸缩、剪切和深度变化。单调纵向投影作为开放、不倒走、无闭环的检查依据。
- 速度：独立随机流生成正密度时间映射，再平滑启停；可以局部加速、减速，但不倒放，不凭空添加循环来凑峰值档位。
- 姿态：独立随机流生成平滑多轴旋转，不强制跟随路径切线。检验真正的 SO(3) 角速度、角加速度和角 jerk，而不是将欧拉角导数当作它们。
- 可复现：保留种子、控制点、时间映射、姿态参数、来源与代码/环境哈希。

这是保留的 12 条源轨迹预览，形态已获用户确认；本次批量使用上文新的 120 条清单，不自动把预览混入批次。CSV 或源检查通过均不等于 GPU compile、整机碰撞、mock recording 或独立 playback 已通过。

## 不变的坐标与约束

仍使用 `mla_short`、UR5、190 mm 长/5 mm 直径杆；CSV 坐标为 `plane1`，米、秒、四元数 `xyzw`。板上方为局部 −Z，绘图高度 `h=-tip_z_m`。

XY 在 ±130 mm 内；高度 70–200 mm。用户已明确允许 70 mm 作为仿真候选，解除的是旧的额外 3 mm/2° 粗测误差包络准入阻挡，不是降低原 5 mm 碰撞间隙或修改保护区。实际整机、自碰、连续 IK、关节动力学以及 home/首点准备仍须在回放构建时逐项验证。

中心加速度 ≤0.05 m/s²、jerk ≤0.25 m/s³；角加速度 ≤30 °/s²、角 jerk ≤150 °/s³。首末速度、加速度、jerk 为零，首末各 0.5 s 严格静止。源网格 20 Hz，hold 行与前一行的七个 pose 字段必须逐字节相同。

随机变速在上游设计**新文件**时完成，所得到的 `time_s` 随新文件一同冻结。执行端仍禁止缩放、重定时、裁剪、跳点或自动降速；不通过就报错，重设计必须使用新 ID、新文件和新哈希，不修改既有权威 CSV。

未连接真机、NUCLEO 或采集背景。本目录输出仅为名义 `rod_tip` 仿真候选，不是验收后的磁铁中心标签；`real_execution_authorized=false`。

## 文件

```text
core.py                         连续曲线、随机时间映射和 SO(3) 模型
generate.py                     不可变源 CSV、元数据及 manifest
audit.py                        独立重建及源合同检查
generate_batch.py               新 120 条批次及不可变来源快照
batch_audit.py                  独立全批配额、种子及来源核验
tests/test_augmented.py          数学、源合同和不可变性测试
plot_preview.py                 从权威 CSV 绘制 3D 及速度/姿态剖面
plot_batch.py                   新批代表路线、运动剖面和覆盖图
replay/                        新批独立 prepare、调度及真实回放索引导出
batches/<batch>/csv/             新权威源
batches/<batch>/metadata/        种子和全部参数
batches/<batch>/reports/         源检查证据，不冒充机器人验收
batches/<batch>/plots/           预览图及绘图来源哈希
```

Mac 在父项目目录生成和检查（仅 CPU，不启动 ROS）：

```bash
cd /Users/huaxin/Codes/ur-waypoint
.venv/bin/python workstation_magtrack/continuous_augmented/generate.py
.venv/bin/python workstation_magtrack/continuous_augmented/audit.py \
  --manifest workstation_magtrack/continuous_augmented/batches/ca_open_elastic_preview_20261016_v1/manifest.json
.venv/bin/python workstation_magtrack/continuous_augmented/plot_preview.py \
  --batch workstation_magtrack/continuous_augmented/batches/ca_open_elastic_preview_20261016_v1
```

相同参数重跑只接受逐字节相同的内容。改变随机种子或算法必须使用新的 `--output` 批次目录；不得覆盖旧权威源。绘图入口参数以 `plot_preview.py --help` 为准。不得将旧目录中的全部 CSV 自动当成本次执行清单。

## 首批源检查结果

12/12 源级检查通过，14 项独立测试通过；6,143 个 waypoint，合计 306.55 s，单条含 hold 为 22.75–30.35 s。两条固定姿态作为对照，其余十条为平滑多轴姿态变化。连续包络的全批最大中心加速度 0.01215 m/s²、jerk 0.01693 m/s³，最大角加速度 11.00 °/s²、角 jerk 36.58 °/s³，均低于原硬限值。

检查报告为 `batches/ca_open_elastic_preview_20261016_v1/reports/source_audit.json`。三张预览位于同批 `plots/`：`fixed_height_overview.png`、`one_way_full_height_overview.png`、`speed_orientation_profiles.png`。3D 图实线来自权威 CSV，灰虚线只表示形变前原型；工具轴箭头按真实四元数绘制。

上述 12 条预览本身未构建机器人回放。新的 120 条批量任务已启动，实际通过数量以新批状态和回放索引为准。旧循环批次不会在后台自动继续。
