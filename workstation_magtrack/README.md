# workstation_magtrack

`real_collection_v2_20261016` 研究计划的独立轨迹工作区，复用 `mla_short` 场景及父项目的 ROS Jazzy / cuRobo 安装。输入、生成器、权威 CSV、配置快照、仿真回放和证据都保存在本目录，不混入之前的采集批次。

主交付目录位于**远端 Ubuntu24**：`/home/huaxin/Codes/ur-waypoint/workstation_magtrack`，GPU compile、mock build、inspect 和独立 playback 均在该机执行；源轨迹、封存回放和验证证据以该目录为主。Mac 上的同名目录保留生成／校验代码、测试和早期历史材料，**没有新增 v2/v3 批次的产物备份**，不替代远端交付目录。

最初轨迹交付阶段完成离线生成、GPU 编译、mock recording、inspect 和独立 mock playback；以下 336 条验收均是名义仿真证据。之后另行授权的 RAW 真机采集、背景和可靠性状态见 [collection/README.md](collection/README.md)，不追改原配置或封存产物中的 `simulation_only` 标记。逐条仿真进度见 `runs/<batch>/status.json`；生成了 CSV 或仅编译通过不等于已经获得验证通过的回放。

新增连续轨迹按用户最新要求改为 [开放轨迹的平滑随机增强](continuous_augmented/README.md)：空间、速度和姿态均可平滑随机变化，排除花纹式闭环和多圈重复。12 条预览已确认，先后生成两个各 120 条的独立源批（v2、v3），均各 84 条固定高度（70–200 mm、14 层 × 6 族）和 36 条全高度单程，全部多轴平滑姿态。增强库共 240 条、总时长 1 小时 45 分 6 秒；连同下述原 96 个标定/同步/联调/Pilot 文件，共 **336 条，三批均已完成四阶段名义仿真与本体独立审计**。停车 v2 往返资产也已通过 GPU 与独立 mock 验收；这些封存仿真资产自身不授予真机权限。

v2 的结果在 `runs/ca_batch120_20261016_v2/`，源批为 `continuous_augmented/batches/ca_open_elastic_batch120_20261016_v2/`；该源批目录内的 `reports/replay_index.*` 是其 120 条回放清单。70 mm 已按用户要求作为名义仿真候选解冻，原 5 mm 硬碰撞门槛、保护区和其他硬约束不变；本批不沿用早期额外暂定误差包络的准入阻挡。

v3 源批为 `continuous_augmented/batches/ca_open_elastic_batch120_20261016_v3/`，直接在 Ubuntu24 生成、检查和保存，不回拷新产物到 Mac。源 ID、实际 CSV、空间/速度/姿态参数及随机种子均核对无重复；其独立回放计划/调度在 `continuous_augmented/replay_v3/`，结果为 `runs/ca_batch120_20261016_v3/`，回放清单在源批目录内的 `reports/replay_index.*`。该批于 **2026-10-17 05:35:45（北京时间）**完成，即 UTC 2026-10-16 21:35:45；不改变任何输入或验收门槛。

旧 `continuous_patterns` 的 156 条候选保留但批量回放已停止，不能视为新版 active；下述已完成的原 96 条回放及其原始设计/准入记录不受影响。

## 本批仿真验收结果

2026-10-16 20:16:12（北京时间），`authority_v4` 的 96 条均完成 GPU compile → mock build → inspect → 独立 mock playback，四阶段各 96 passed、零失败。`runs/authority_v4/campaign_summary.json` 为 `state=finished`、`workflow_passed=true`、`complete_passed=96`，三个执行阶段与两个后检查退出码全部为 0。

远端 `reports/replay_index.csv` / `.json` 为最终回放清单，`ready_on_this_host=96`；其中记录实际 recording ID、相对目录、源哈希、封存校验及同控制器时间的独立回放见证。原始目标时间未缩放，所有 waypoint 均保留。这是名义环境仿真验收，不开放真机执行或正式磁铁中心标签。

后续 v2、v3 各 120 条同样完成 compile → build → inspect → 独立 playback，本体合计 336 条独立审计通过，所有原始 waypoint 与时间保持不变。停车往返 v2 的 GPU／独立 mock 验收通过，bundle 为 `collection/prepared_parking/parking_20261017_v2`。周期空场调度及其暂停规则见 [collection/README.md](collection/README.md)；停车位磁干扰仍未实测，仿真通过不等于真实采集已开放。

包含 **336 条本体 + 双向停车资产**的最终交付已发布在远端 `reports/overnight_delivery_20261017_v1/`，出口审计为 `accepted=true`，日志为 `runs/delivery_finish_20261017_v3.log`。该目录的五个文件是：`delivery.json`、`delivery.csv`、`collection_manifest.json`、`README.md`、`sha256sums.txt`；优先从这里查完整交付，三个批次各自索引仍保留作为来源证据。

## 坐标、工具和使用边界

- 环境仍叫 `mla_short`，不是 `workstation_magtrack`；后者通过 `--workspace` 指定。
- UR5；名义杆长 **190 mm**、直径 **5 mm**，沿 `tool0` 的局部 `+Z` 延伸到 `rod_tip`。不是此前的 333.65 mm 杆。
- CSV 在 `plane1` 中，长度 m、时间 s；板上方是局部 **−Z**，`h=-tip_z_m`，抬高应减小 z。
- 当前 `plane1` 来自既有 `new_board_mla` Feature，再沿 Feature 局部 `+Z` 偏移 53 mm。它是现有场景的名义板中心，**尚不能直接当作 hospital 配置的阵列原点 S**；外观中心与该配置原点约有 10 mm X 偏差，需实测注册。
- 当前没有本次安装已验收的 `R_S_B`、`t_S_B`、`r_F_C`，也没有有效安装 ID 与控制器生效 TCP 的完整绑定。轨迹中的中心与方向均为名义 `rod_tip` / 工具轴候选，不是已标定磁铁中心、磁化轴或真实参考标签。

这些名义仿真产物自身不授予真机权限，也不是磁铁中心标定验收。取得低高度原始标定数据可按另行哈希绑定的 RAW 授权进行，偏置由下游派生；需要正式磁铁中心标签的计划仍须绑定验收标定和有效安装。不要把原 `simulation_only=true` 当成应手改的启动故障。原参考等级说明见 [标定与真机开放门槛](docs/calibration_and_release_gate.md)，当前 RAW 启动入口见 [collection/README.md](collection/README.md)。

## 本批内容

| 集合 | 独立权威文件 | 设计 |
| --- | ---: | --- |
| 低高度几何标定 | 27 | 9 个 XY × 3 高度，每文件 5 姿态，共 135 capture pose |
| 独立动态同步 | 6 | X/Y/Z × 5/10 mm/s 峰值，各含正反向 |
| 三条联调 | 3 | 静态 V0/W0、平移 V2/W0、中心固定转向 V0/W1 |
| Pilot | 60 | 2 session × 30；静态、平移、转弯、定中心转向、混合三维各 12 |
| 合计 | 96 | 仅 active 清单进入回放构建，不扫描全部旧 CSV |

标定高度采用 **75/80/100 mm**，姿态为基准及局部 X/Y 各 ±20°。每姿态先稳定 0.5 s，再留 1 s capture；两组完整执行计划共 270 capture window，重复使用同一权威源，不复制或改写时间。2/9 完整 XY 组作为留出，跨高度、姿态和重复保持分组。60/70 mm 因现有保护区及粗测误差包络不满足间隙，已在上游拒绝，不降低保护区来兼容。

Pilot 主范围 XY ±130 mm、高度 80–200 mm；静态每 session 在 150/160/170/180/190/200 mm 各一条。active v4 的计划有效总时长约 1391.09 s，单条 20–41 s；独立源检查中高区计划时间占约 59.35%，五个高区 1 cm 箱占全部时间约 11.98/11.48/11.47/11.26/13.16%。这些是源设计覆盖，不是物理执行统计；独立报告另检查 V×W、类型和 session 的解耦。

- V1/V2/V3：10/20/50 mm/s。平移和转弯为巡航速度，混合为峰值；均速、P95 和最大值分别记录。
- W1/W2/W3：3/10/20 °/s，为名义工具轴方向变化率峰值；不是只绕杆轴自转。磁化轴物理绑定仍待标定。
- 候选中心加速度 ≤0.05 m/s²、jerk ≤0.25 m/s³；角加速度 ≤30 °/s²、角 jerk ≤150 °/s³。关节与整杆约束另验。
- 目标时长约 20 s；为保留完整几何路线、达到标称档位并平滑启停，有些文件更长。具体时长和原因保存在每条元数据，不裁剪凑时长。

## 文件索引与权威输入

```text
inputs/                                  用户任务及完整采集协议
generation/                              纯离线轨迹和标定动作生成器
planning/authority_index_v4.json         本轮唯一 active 集合及来源哈希
planning/calibration_*.json               标定窗、重复执行及拟合/留出分组
planning/active_pilot_manifest_v4.json    当前联调/Pilot 候选及替换关系
planning/frozen_nominal_snapshot_v4.json  冻结配置及来源哈希；不是验收安装参数
environments/mla_short/                   名义环境、杆、障碍、限位配置快照
environments/mla_short/trajectories/       权威源 CSV；包含保留的旧版本
environments/mla_short/recordings/         runtime 封存的回放文件
runtime/                                 隔离仿真批处理；细节见其 README
validation/                              独立源合同/覆盖检查和计划图
reports/                                 独立报告、计划图及 runtime 证据汇合
runs/                                    编译、mock 实际执行、日志与状态证据
docs/                                    坐标和后续现场验收门槛
```

CSV 表头为：

```csv
time_s,frame_id,tip_x_m,tip_y_m,tip_z_m,tip_qx,tip_qy,tip_qz,tip_qw,phase
```

`frame_id=plane1`；姿态为 **xyzw**。研究侧公共四元数约定为 **wxyz**，交接必须显式换序，不能直接拷贝。`phase` 为 `draw/hold`；第一行用 `draw`，后续 `hold` 行必须与上一行 XYZ 和姿态严格相同。capture / settle 等细分目的、起止时间保存在 JSON，不通过修改 CSV 语义表达。

每行 waypoint、姿态及 `time_s` 自写入起就是权威硬约束。禁止 runtime 缩放、重定时、裁剪、跳点或自动降速；插值输出只允许保持全部权威时刻与 pose 的固定时间编译。动力学、碰撞或连续 IK 不通过就报错。上游重设计必须产生新的轨迹 ID、CSV 和哈希，并保留被拒旧版本及原因；不可将目录中全部 v1/v2/v3 一起执行。

当前 active 是标定/同步 v1 + 联调/Pilot v4，共 96 条；Pilot v1/v2/v3 仅留 provenance。早期版本在上游修订高区时间箱、高/低区 × V × W 解耦及严格序列化 hold 连续性；旧文件没有被 runtime 修改。旧的 `planning/authority_index.json` 指向已退休的 v3，不用于本轮执行。

## 生成与检查

在本次源生成机 Mac 上使用父项目已配置、包含 NumPy / SciPy / PyYAML 的 Python；生成器不会启动 ROS 或设备：

```bash
cd /Users/huaxin/Codes/ur-waypoint/workstation_magtrack
../.venv/bin/python -m generation.calibration --root "$PWD"
../.venv/bin/python -m generation.pilot --root "$PWD"
../.venv/bin/python -m generation.assemble --root "$PWD" \
  --active-manifest "$PWD/planning/active_pilot_manifest_v4.json" --revision _v4
```

同主机、同路径、同配置可重跑核对相同文件。写入函数仅接受与既有文件逐字节相同的内容，否则报 `refusing to overwrite differing authority`，不会覆盖；生成器当前代码中的版本决定候选 ID，变更算法时必须提升版本后生成新 ID。部分原始生成 manifest 保留 Mac 路径；**Ubuntu24 直接使用同步后的相对路径索引与 CSV，不在原目录重新生成它们**。换生成路径时须用新的空工作区，不能为迁移而覆盖原 manifest。assembler 的新索引也须使用 `--revision _vN`，保留旧索引和配置快照。独立源检查、报告与图见 `validation/`；生成器自报的诊断不是完整机器人验证。

独立检查源合同、135 pose 分组、运动限值和计划覆盖；Mac 或 Ubuntu24 都可在其工作区目录运行，已有 runtime 状态时一并关联：

```bash
../.venv/bin/python -m validation.audit --workspace "$PWD" \
  --authority-index "$PWD/planning/authority_index_v4.json"
```

有运行证据后可追加 `--runtime-index "$PWD/runs/authority_v4/status.json"`；不存在时不要传入。报告为 `reports/independent_source_audit.{json,md}`、`calibration_observability.json`、`planned_coverage_audit.json` 与 `runtime_evidence_join.json`。计划图以 `planned_` 开头，分别展示三维路径/名义轴、速度/轴变化率、加速度/jerk、速度×高度、类型×session；它们不冒充实际采集图。

Ubuntu24 当前父环境的系统 Matplotlib 与 NumPy ABI 不匹配；在该机重跑检查时加 `--no-plots`，只刷新 JSON/Markdown，不更改依赖。完整 PNG/PDF 已在 Mac 生成并同步。回放交付索引为 `reports/replay_index.csv` / `.json`，逐条列出 source、recording ID、四阶段状态和封存文件校验结果；索引可以包含 pending 条目，不表示整批完成。JSON 的 `status_input` 说明它来自哪个批次快照，最终整批索引应关联 `runs/authority_v4/status.json`；早期 canary 索引不能代表主批进度。

v4 的 96 条源文件已通过严格 schema/hash/时间/hold 检查和采样 Cartesian 运动限值检查，60 条 Pilot 的离散配额通过。源网格有限差分检查不替代连续解析限值、机器人关节/碰撞检查或独立回放；全链路状态仍只以 runtime 证据为准。

## Ubuntu24：仅仿真构建回放

```bash
cd /home/huaxin/Codes/ur-waypoint/workstation_magtrack
bash runtime/run_batch.sh --ids-file planning/authority_index_v4.json \
  --batch authority_v4 --phase all --jobs 2 --adopt-existing-compiled
```

入口依次进行 GPU compile → mock recording → inspect → **全新 mock runtime** 的独立 playback。同一个 batch 名可断点恢复，只有产物哈希仍匹配的通过阶段才跳过。退出码 2 表示至少一项失败或阻塞，不能据此跳过失败项或放宽阈值。详细分阶段命令见 [runtime/README.md](runtime/README.md)。

Build 使用 localhost-only domain 219，独立 playback 使用 220；不要在相同 domain 同时启动其他 runtime。不要在 domain 0 运行这些批处理。编译和 mock 顺序进行，避免 GPU/CPU 高负载干扰 100 Hz 执行记录。

结果查 `runs/authority_v4/status.csv` / `status.json`。其中的 `recording_id` 才用于 `tipdraw play`；`trajectory_id` 是源文件 ID，二者不要混用。每条须同时具有编译、封存、inspect 和独立 playback 证据才算完整链路通过。标定重复执行清单与离线回放构建是两件事：本轮生成 27 个标定回放并不代表实际重复采集已经发生。

## 后续只在 RViz 看已封存回放

以下两个终端仅启动 mock；不要同时启动连接真机的 UR driver。先在**当前这台主机**的 `reports/replay_index.csv` / `.json` 中选择 `ready_for_simulation_replay_on_this_host=true` 的条目，再复制其 `recording_id` 替换终端 B 的占位符。该标志要求四阶段通过、独立 playback witness 通过且本机封存文件校验通过；不要把 `trajectory_id` 或仅 compile 通过的条目用于预览。

终端 A：

```bash
cd /home/huaxin/Codes/ur-waypoint/workstation_magtrack
source /opt/ros/jazzy/setup.bash
source ../.venv/bin/activate
source ../install/setup.bash
export ROS_DOMAIN_ID=221 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT PYTHONNOUSERSITE=1
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/ops/fastdds_loopback_udp.xml"
unset ROS_LOCALHOST_ONLY FASTDDS_BUILTIN_TRANSPORTS
export PYTHONPATH="$PWD/../.deps/curobo-a35a708ecfbb26eb9ab2d7ef22c65919c4fae4a9:$PWD/../.venv/lib/python3.12/site-packages:${PYTHONPATH:-}"
ros2 launch hx_waypoint_bringup simulation.launch.py \
  ur_type:=ur5 environment_id:=mla_short workspace:="$PWD" launch_rviz:=true
```

终端 B：

```bash
cd /home/huaxin/Codes/ur-waypoint/workstation_magtrack
source /opt/ros/jazzy/setup.bash
source ../.venv/bin/activate
source ../install/setup.bash
export ROS_DOMAIN_ID=221 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT PYTHONNOUSERSITE=1
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/ops/fastdds_loopback_udp.xml"
unset ROS_LOCALHOST_ONLY FASTDDS_BUILTIN_TRANSPORTS
export PYTHONPATH="$PWD/../.deps/curobo-a35a708ecfbb26eb9ab2d7ef22c65919c4fae4a9:$PWD/../.venv/lib/python3.12/site-packages:${PYTHONPATH:-}"
ros2 run hx_waypoint_tools tipdraw --workspace "$PWD" play \
  --env mla_short --recording '<通过检查的recording_id>' --auto-prepare
```

`--auto-prepare` 先经碰撞检查到规定 home，再到回放首点；准备运动不属于权威目标 CSV，不改变目标时刻和 waypoint。domain 221 仅用于此手动预览，避免占用批处理 domain。

## 已授权 RAW 采集的统一启动入口

ROS 栈与 RAW 采集分别使用 Ubuntu24 的 `ops/start_mla_short.sh` 和 `ops/collect_reliable.sh`，两者共用 `ops/ros_environment.sh`，固定父项目源码、锁定 cuRobo／venv site-packages 和 FastDDS loopback UDP 配置。不再复制混用旧的裸 `ros2 launch` 或 `ROS_LOCALHOST_ONLY=1` / `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` 命令；仅 source ROS／install 或激活 venv 不能保证已安装 planner 的 `/usr/bin/python3` 能找到锁定依赖。

获准启动真实 ROS 栈、且旧栈已由操作员确认停止后，终端 A 使用：

```bash
cd /home/huaxin/Codes/ur-waypoint/workstation_magtrack
bash ops/start_mla_short.sh --robot-ip 192.168.1.99 --launch-rviz true
```

这不是预览命令；默认会连接真机。脚本先用已安装 planner 的 shebang 解释器核验锁定依赖导入，不做 GPU warmup；发现受检查的既有栈节点、锁被占或无法确认 ROS graph 时拒绝启动，不停止或替换旧栈。驱动连接后仍由操作员在 UR 示教器按 Run／External Control；不会打开 NUCLEO。只读检查须显式使用 `--dry-run` 或 `--env-report`，后者不等于 GPU／live planner 就绪验收。

终端 B 的 RAW 采集参数、健康门槛和失败后处理见 [collection/README.md](collection/README.md#raw-采集的统一启动与可靠性边界)。采集入口不会启动或重启 ROS，也不会自动重试 SYNC 或失败任务。`resume9` 在历史 boot `191358` 的背景已接受，但第 29 条在准备规划阶段因当次 ROS 栈遗漏依赖路径而停止，没有绘制。操作员于 2026-10-17 20:03:32 CST 再次实际重启 NUCLEO，并明确授权新 boot `200332` 的一次初始 SYNC；不再使用此前拟用的 `195357`，旧 boot 背景不能继承。`resume10` 于 20:06:09 CST 的启动快照仍为同一 manifest 的 28 条目标已接受、308 条待采；已授权的顺序为 home→30 s 背景→home→从第 29 条继续。这只是启动历史，不是新采集完成或 real passed 证明，也不授予自动 retry。完整证据和 boot 边界见该说明。

上一轮完整软件回归结果：Ubuntu24 为 764 passed、241 subtests；Mac 为 765 passed、241 subtests；C++ 为 97 passed，证据在 `runs/collection_reliability_fix_20261017_v1/`。后续共享环境与启动入口的 Ubuntu24 专项测试另有 73 passed（collect 44＋start 29），独立审阅通过。20:02:57 CST 启动的新 ROS 栈已通过实际安装环境的依赖导入预检，随后 live server warmup 为 `success=True`；这不等于新目标采集通过或新版真机长期可靠性验收。详细日志与部署确认见 [collection/README.md](collection/README.md#代码测试)。

## 原交付与后续现场研究的证据边界

原 336 条离线交付本身不包含实采磁场或机器人实际反馈；后续已授权的现场 RAW、背景及可靠性修复状态单独记录于 [collection/README.md](collection/README.md)，不能用仿真证据或 CPU 测试代替。正式磁铁中心标签仍须由**实际**机器人反馈与验收固定标定派生，不能使用计划 CSV 或高处 tracker 的逐帧估计替代，也不能因曾做过仿真而省略实际背景协议。

## 原 96 条批次的任务与来源证据

原 96 条使用过 Ubuntu24 的 `wm_compile_v4` 与 `wm_finish_v4` tmux session，现已完成并退出。后者入口是 `ops/finish_campaign.py`，只操作本目录下的仿真，不访问真机或串口。以下命令查看原批结果；新增两批请分别查看上文 `ca_batch120_20261016_v2` 和 `ca_batch120_20261016_v3`，不要重新启动已完成批次：

```bash
cd /home/huaxin/Codes/ur-waypoint/workstation_magtrack
tmux list-sessions
tail -n 5 runs/authority_v4/campaign.log
less runs/authority_v4/status.csv
```

最终汇总写入 `runs/authority_v4/campaign_summary.json`。整批完成必须**同时**满足：`state="finished"`、`workflow_passed=true`、`trajectory_count=96`、`complete_passed=96`；`phase_counts` 中 compile/build/inspect/play 各有 96 条 passed；`phase_exit_codes` 中 build/inspect/play 及最终 postcheck（`inventory`、`independent_source_audit`）返回码全部为 0。仅编译数量达到 96、仅出现 `complete_passed=96` 或尚未跑完 postcheck 的中间汇总都不算完成。

最终交付索引还须来自该主批，且 `ready_on_this_host=96`；单条应有四阶段 passed、`coherent_playback_witness="passed"`、`local_sealed_checksum_status="passed"`。失败或中断保留原日志，不放宽阈值、不自动改源轨迹。若需要恢复，使用前文相同 batch 命令；不要与已有 coordinator 同时运行。

`provenance/` 保存 Ubuntu24 实际导入的 Python 源副本、相关源码 dirty diff、运行库和 C++ executable 指纹。父工程存在未提交改动，因此 commit 单独不足以复现本次运行；详细限制见该目录 README。本目录未提交或推送到 Git 远端。
