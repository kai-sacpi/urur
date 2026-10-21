# 标定、坐标与真机开放门槛

本文件解释为什么本轮 `mla_short` 的仿真回放暂不等于正式磁铁中心执行计划。用户本轮只要求离线仿真及回放，因此不包含连接真机或传感器的命令。

## 1. 名义场景与研究坐标不是同一份已验收标定

既有环境配置保存在 `environments/mla_short/`，名义配置为 UR5、190 mm × 5 mm 杆、`tool0 → rod_tip`。`plane1` 的姿态来自示教器的 `base → new_board_mla` 旋转向量，平移另复合 Feature 局部 `[0,0,0.053] m`，不是在 base 的 Z 上直接相加。

```text
raw base→Feature pose:
[-0.08533, -0.48683, 0.24284, 1.16388, 2.91348, 0.00029]

T_base_plane1 = T_base_Feature · Translation_Feature(0,0,0.053)
```

后 3 个读数是 UR 旋转向量，不是欧拉角。本轮不额外纠正它的朝向、不拿墙反推板位姿，也不移动现有碰撞区。`plane1` 局部 −Z 指向板上方，`h=-z`。

研究协议的 S 是 hospital 配置中的阵列原点。8×8 传感器、50 mm pitch、350 mm 跨度与约 400 mm 外观板并不能证明 S 恰在外观中心；协议指出约 10 mm 的 X 偏差。本轮候选保持现有 `plane1`，把 `plane1↔S` 标为待注册，不静默加减该 10 mm。

## 2. 后续必须冻结的合同

正式计划至少绑定以下信息及原始标定来源：

| 信息 | 本轮状态 | 正式要求 |
| --- | --- | --- |
| 机器人、负载、杆与载具 | 名义 UR5 / rod190_d5 | 核实实际型号、装配刚性、尺寸及材料 |
| 安装 ID | 待现场核实 | 板、机器人、载具和磁铁装配的有效状态 |
| `R_S_B, t_S_B` | 未取得本次验收参数 | 从机器人基座 B 到 S 的固定旋转、平移 |
| `r_F_C` | 只有名义杆尖 | 法兰 F 到磁参考中心 C 的固定偏移 |
| 控制器生效 TCP | 未读取/验收 | 保存真实 `T_F_TCP`，可恢复到法兰 |
| 磁化轴 | 名义工具轴 | 若需要方向标签，验收其在法兰中的固定方向 |
| 时间映射 | 只生成同步动作 | 独立同步实验测偏移、漂移和不确定度 |
| 背景 | 本轮不采 | 后续按批次取得合法目标前背景并关联 ID |

若当前反馈是 TCP，必须先恢复统一参考：

```text
T_B_F(t) = T_B_TCP(t) · inverse(T_F_TCP)

p_S_C(t) = R_S_B [p_B_F(t) + R_B_F(t) r_F_C] + t_S_B
```

该式中的 `p_B_F/R_B_F` 必须来自真实反馈；命令 CSV 只作为计划。完整实体姿态或磁化方向的标定与位置参考分开保存。轴对称磁体绕自身磁化轴的 roll 不因记录机器人姿态就变得可观测。

旧的单轴 yaw 检查适合区分固定板偏差与横向安装偏心，但不能验收这里的 9 个几何未知量，尤其不能分开轴向工具偏移与板端平移；不把旧结果直接升级为本次完整联合标定。

## 3. 本轮低高度动作设计

9 个 XY 位点为 `X,Y ∈ {-0.10,0,+0.10} m` 的笛卡尔组合；每点 75/80/100 mm 三高度，每高度包含基准、局部 X±20°、局部 Y±20° 五姿态，共 135 pose，分成 27 个权威 CSV。每 pose 稳定 0.5 s，capture 1 s，姿态间平滑启停；首末和 hold 保持静止。

`planning/calibration_repeat_schedule.json` 预先列出两次完整执行及独立 execution ID，共 270 capture window；2/9 完整 XY 组留出，组内所有高度和姿态不拆分，重复保持同组。这里是执行计划，不表示已采到磁观测或已完成两次真实重复。

原建议 60 mm 与环境 65 mm 分析边界冲突；70 mm 计入暂定 3 mm 位置误差、2° 朝向误差及整杆球包络后也不满足 5 mm 间隙。因此上游生成新 75/80/100 mm 方案，拒绝记录保留于 `planning/calibration_report.json`。3 mm / 2° 是保守预检假设，不是实测误差或已经验收的标定精度。

这些动作提供未来联合求解所需的多位置、多高度、多轴激励，但是否满秩、LM 是否稳定、是否饱和、低处模型是否有共同偏差仍需用真实观测验收。计划几何的可辨识性不等于毫米级绝对精度。

## 4. 从仿真候选到正式采集的顺序

1. 核实安装状态、工具、材料、板敏感平面及完整障碍，划分有效安装 ID。
2. 按合法目标前背景协议准备未来低高度标定；保留原始磁流、设备/主机时间与真实法兰反馈。
3. 由磁跟踪侧联合求解固定 `R,t,r`，检查多初值、尺度归一化雅可比秩、留出、重复、低高度子集与高处外推敏感性。
4. 用 6 条独立 XYZ 正反向、双速度同步动作核验残余时延；不根据高处测试误差择优时移。
5. 冻结标定、同步、实际 TCP 和安装 ID；使用新版本对候选中心轨迹进行法兰补偿，再做完整链路规划与仿真。
6. 未来真机依次验收 V0/W0、V2/W0、V0/W1 三条联调，定中心旋转检查补偿后的中心漂移。
7. 联调和同步预算通过后逐档开放 V3/W2/W3，再开展 2 session × 30 pilot。V4 不在此次核心 60 条中。

板移动、工具/TCP 变化、重新装配、滑动或刚性变化都会使旧绑定失效，必须新增安装/标定版本；不能继续沿用旧元数据。失败与定位困难分别标记，保留原始失败记录，不按 tracker 误差删除困难数据。

## 5. 什么才算“回放完成”

离线源检查检查 CSV、时间、hold、运动学设计与配额；GPU compile 验证固定时间 IK/关节曲线；mock recording 验证录制链；inspect 验证封存产物；全新 mock runtime playback 验证独立可回放性。它们是不同层次，不互相替代。

每条通过条件包括源 hash/全部源时刻与 pose 保留、`uniform_time_scale=1`、关节动力学不过限、FK 误差和整杆/机器人碰撞间隙合格，并保留 100 Hz mock 执行 telemetry。阈值沿用冻结环境，不为通过率临时放宽。进出场与 home→首点同样要碰撞检查，但准备段不挪入权威目标时间轴。

整批 96 条完成不能只看编译数量或单一计数。`runs/authority_v4/campaign_summary.json` 必须同时为 `state="finished"`、`workflow_passed=true`、`trajectory_count=96`、`complete_passed=96`；compile/build/inspect/play 四阶段各 96 条 passed，且 build/inspect/play 与 `inventory`、`independent_source_audit` postcheck 返回码全部为 0。`reports/replay_index.json` 最终还须关联该主批、`ready_on_this_host=96`，逐条通过独立 playback witness 与本机封存校验。中间汇总和早期 canary 索引不是整批完成证明。

仿真 telemetry 是 mock 控制器反馈，**不是物理机器人实际 FK，也不是磁铁真实位置**。即使所有仿真阶段通过，仍只证明所保存名义模型内的离线执行链通过。

当前参考等级：

```text
simulation_only: true
reference_center: nominal_rod_tip_pending_accepted_magnet_calibration
label_reference_type: planned_nominal_pose_only_no_actual_reference_labels
independent_absolute_validation: unavailable
real_execution_authorized: false
```

未来采用低高度 LM 联合标定后，参考应称“磁辅助标定的机器人参考”，不是已独立计量确认的几何中心绝对真值；共同磁模型偏差仍可能传入标签。需有独立计量支持才能作几何中心绝对精度结论。
