# 周期空场与 NUCLEO 重启暂停

这是独立采集调度层，不修改已冻结的 `mla_short` 场景、原 home、源 CSV 或 sealed recording。名义工具仍为 **190 mm × 5 mm** 杆，所有 waypoint / `time_s` 不缩放、不重定时、不裁剪、不跳点。

## 已确认的停车位

`config.yaml` 保存用户对 `IMG_5335.JPG` 六关节读数的确认：

```text
shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3
[-141.48, -84.74, -22.66, 19.73, 99.31, 42.18] degree
```

它是空场停车位，不取代原 home；照片确认的六关节角度保持不变。磁铁在此处“基本无干扰”是待测的实验假设；碰撞通过不等于磁干扰已验收。

## 2026-10-17 部署状态

代码部署至 Ubuntu24：`/home/huaxin/Codes/ur-waypoint/workstation_magtrack/collection/`。单元测试使用假传输／临时文件；这些不是实际串口或机器人运行证据。请以本次测试输出和下列实际产物为准，不把单元测试计数当成回放完成数量。

Ubuntu24 产物（不回拷 Mac）：

- `runs/parking_cpu_precheck_20261017_v1/validation.json`：仅 CPU 几何初筛；world 最小间隙约 31.175 mm、self 约 29.154 mm，不能当完整 GPU 路径证书。
- `collection/collection_manifest_preview_20261017_v1.json`：原 96 条已验收名义回放的有序、哈希绑定清单，只用于调度预览。
- `runs/collection_schedule_preview_20261017_v1/`：原 96 条 manifest 的历史虚拟预览保留，不作为最终 336 条清单的预览证据。
- 原 96 条、增强 v2 的 120 条、增强 v3 的 120 条均已完成 compile／build／inspect／独立 playback，共 336 条本体独立审计通过。v3 完成于北京时间 2026-10-17 05:35:45，不再等待该批执行。
- `runs/parking_gpu_validation_20261017_v1/` 及其日志保留旧停车 GPU 失败证据：solver clock 与 exact grid 组合形成 1 ns 区间，未通过原门槛；没有放宽 gate 或将失败资产标成通过。
- 停车 v2 采用全新上游 `minimum_jerk_joint_bridge_v1`：每方向 20 s、100 Hz、2001 点，解析生成 q／qd／qdd／time，并在验证前冻结 CSV。它不修改 v1，也不是 runtime 重定时。`runs/parking_gpu_validation_20261017_v2/validation.json` 已为 `passed_both_directions`、`status=passed`、`runtime_ready=true`，最小间隙约 29.15 mm。
- 双向 bundle 已封存至 `collection/prepared_parking/parking_20261017_v2`；`runs/parking_mock_validation_20261017_v2/evidence.json` 的独立 mock 为 `status=passed`，完成于北京时间 **2026-10-17 05:53:57**（UTC 2026-10-16 21:53:57）。原始 CSV 逐字节回放，`runtime_replanning=false`、`source_timing_changed=false`。
- `runs/delivery_finish_20261017_v3.log` 确认最终出口 `accepted=true`，包含 336 条本体和双向停车资产的五文件交付已发布，路径见下文。
- `runs/collection_schedule_preview_336_20261017_v1/` 的 `checkpoint.json` 与 `schedule_preview_session_001.json`：最终 **336 条 manifest 的首会话虚拟调度**已走到重启暂停点；68 次 play、3 次停车、3 次各 30 s 虚拟空场、6 次回 home、1 次 close，`status=waiting_for_nucleo_restart`、`next_cursor=68`。末段为 home→park→background 30 s→home→close，机器人／传感器使用均为 false；不是已经实际运行全部 336 条，后续会话仍须显式恢复。

Mac 仅保留相关代码／测试和早期历史材料，没有新增 v2/v3 批次或停车 v2 的产物备份；本节产物以 Ubuntu24 为准。

上述交付验收阶段未执行真机运动、未打开 NUCLEO、未采新的真实空场。后续单条现场试采状态见下文，不追改原交付结论。

## 调度行为

1. 同一个硬件 boot 只保持一个持续读取的 NUCLEO 会话。新 boot 先被动读取，仅完全无字节且预检超时时允许一次初始 SYNC；已有字节但不成帧则报错。同 boot 复用必须带 `--reuse-nucleo-stream`，即使完全无数据也不再握手。
2. 从会话成功打开／上次空场结束开始计约 600 s。只在**完整轨迹结束后**检查期限；超期也不截断轨迹，并保存超期时间。
3. 新鲜实际关节反馈 → 全路径碰撞检查回 home → 全路径碰撞检查到停车位 → 到位稳定 → 30 s 原始空场 → 全路径碰撞检查回 home。
4. 第 3 次成功空场后，先回 home、关闭串口，再保存 `waiting_for_nucleo_restart` 并退出。不会自动继续、自动重握手或跨硬件会话复用时间映射。
5. 只有从正常 `waiting_for_nucleo_restart` 暂停恢复时，才在用户实际重启硬件后用 `--resume --confirm-nucleo-restart NEW_BOOT_ID`；从下一条未执行轨迹继续。新 ID 不得复用；失败 checkpoint 不走该恢复路径。只是输入参数不能证明硬件真的重启，操作员须实际完成重启。
6. **本调度器**整批正常结束后也回 home 并关闭串口。旧单条 `tipdraw play` 的结束行为没有改变。异常/取消停止分发，保留证据，不假定此时自动返程安全。

Ctrl+C 与 SIGTERM 走所持有 ROS goal 的取消＋终态确认，不依赖退出客户端来停机器人。SIGKILL／断电无法保证软件清理；取消未确认会明确报警，必须人工确认实际机器人状态，不得假定已经停止。

因此暂停不是精确 30 分钟：完整轨迹、准备运动和三次空场都会增加时间。最后一条结束不为了凑次数再补空场。

## 数据与时钟

生产采集默认运行在独立的 `spawn` 子进程中：该进程唯一持有串口、解析器、原始 CSV writer 和接收时钟；主进程负责运动调度、完整文件/动力学校验和事后映射，仅按需请求状态快照。快照 IPC 不承载逐帧数据，也不将主进程收到快照的时间替换为传感器接收时间。子进程不继承 ROS 执行器、不导入运动控制模块；启动或通信失败直接报错，禁止退回同进程采集线程。

父子进程共享同一个已加锁的文件描述符，只有确认 reader 停止且子进程退出后才释放串口所有权；无法确认退出就保留锁并停止调度。父进程控制管道断开后，采集进程停止 reader 再退出。`epoch.json` 和窗口/会话报告的 `capture_process` 保存主进程 PID、采集 PID 与启动方式；新的现场试采 `software_provenance.json` 绑定 `capture_process.py` SHA。单次 SYNC、稳定启动基线、10 ms 漂移预算和全部轨迹约束均不变。

2026-10-17 进程隔离修复的 Ubuntu24 证据：`runs/capture_process_fix_20261017_1411/` 保存修改前/部署后源码快照、SHA 和测试日志。`tests_run2.log` 为 **539 passed、127 subtests passed**；首次 `tests.log` 保留 ROS 自动加载的旧 pytest 插件不兼容错误，第二轮只禁用测试插件自动加载，不修改生产配置。`pty_load_probe/isolation_report.json` 为 `passed_software_isolation_regression_only`：真实协议的 Linux PTY 模拟器以 100 Hz 发送，生产默认采集路径在独立进程保存 1333 组；同时主进程完整重验原停车包两次（各约 2.63 s），零丢组、零成批到达、无漂移报警，最大组间隔 10.438412 ms（正常周期 10 ms）。未访问实际 NUCLEO、未发送机器人动作，也不是绝对同步精度或真实硬件验收。

每个新 run、sensor session 和窗口使用独立目录，禁止覆盖旧窗口。每个硬件会话保留完整原始流；轨迹另外切出实际机器人反馈时间范围内的原始传感器片段及映射表。回放不传 `--record-nucleo`，避免另一个串口 reader 抢占持续连接。

轨迹映射复用 `finalize_nucleo_capture`，其输入是独立临时副本，绝不让它删除持续会话原始流。传感器原始接收时间、设备时间和序号保持原样；映射表的 `sync_time_ns` 对应机器人实际 telemetry。背景独立保存 30 s raw、parser 计数差、质量判断，以及全窗口实际关节/FK 静止见证。传感器质量失败或静止见证失败均停止，不计为成功空场。

每个会话有新的 session/epoch ID，主机 boot 变化则拒绝沿用旧 monotonic 时钟。checkpoint 在分发物理动作**之前**落盘；进程中断时有未确认动作即转 `needs_attention`，不自动重播可能已经完成的物理动作。

```text
runs/<new_collection_run>/
├── checkpoint.json
├── action_*.play.log
├── action_*.stationary.csv
├── action_*.background.json
└── sensor_sessions/<session_id>/
    ├── epoch.json
    ├── nucleo_raw_telemetry.csv       # 关闭时封存的全会话 raw
    ├── session_manifest.json
    └── windows/<独立轨迹或背景名>/
        ├── nucleo_raw_telemetry.csv
        └── window.json
```

## 当前可以运行的纯调度预览

以下命令不打开 ROS、机器人或串口。先选一个已经四阶段通过的 inventory；未通过条目会报错，不静默跳过。`prepare` 只读取并校验源/封存文件，写新 manifest；输出存在即拒绝覆盖。

```bash
cd /home/huaxin/Codes/ur-waypoint/workstation_magtrack
../.venv/bin/python -m collection.run prepare \
  --index reports/replay_index.json \
  --manifest collection/collection_manifest_preview_v1.json

../.venv/bin/python -m collection.run preview \
  --manifest collection/collection_manifest_preview_v1.json \
  --output runs/collection_schedule_preview_v1 \
  --boot-id preview_boot_1
```

虚拟时钟预览使用源时长和**仅用于演示的**每段 10 s 准备时间；不是实际运动、GPU 碰撞、时间精度或磁场采集证据。预览停在第 3 次空场时同样不会自动恢复。需要预览下一段时另行明确输入：

```bash
../.venv/bin/python -m collection.run preview \
  --manifest collection/collection_manifest_preview_v1.json \
  --output runs/collection_schedule_preview_v1 \
  --resume --confirm-nucleo-restart preview_boot_2
```

新增 v2、v3 各 120 条的 inventory 均位于各源批的 `reports/replay_index.json`，本体已完成验收。最终有序合并 manifest 已在下述交付目录发布，并绑定已验收的停车 v2 资产；不要修改已有 manifest 或 checkpoint 来插入/移除轨迹。需要不同顺序时必须另建新 manifest。

## 停车路径验收与真实执行门槛

`validate_parking.py` 用当前完整 `mla_short` 环境验证 home→park 和 park→home；共享现有 campaign 锁排队，不与正在执行的 100 Hz mock 抢资源。CPU 抽样只能初筛，不能冒充完整 GPU 证书。v2 对上游已冻结的 minimum-jerk 关节候选做原门槛验证，不在 runtime 缩放、重定时、裁剪或跳点；GPU 或独立 mock 失败就保留失败并停止，应以远端新 run 内实际报告为准。

完整 GPU 验证通过后，`prepared_parking.py` 才能创建不可覆盖的双向 prepare bundle。它逐字节保留两条关节 CSV 及原 GPU producer 源码快照，另外封存当前 consumer 的版本；不把旧 GPU 证书冒充新版 consumer 的运行证据。这是独立准备运动资产，不是 Cartesian drawing recording，也不授权真机。

`mock_parking.py` 已在独立 localhost-only ROS domain 223 中通过两条原始 CSV 的播放验证，另外保存 controller 同时刻反馈、实际 FK 和完整场景快照证据，不调用 planner 或串口。这是 mock 执行证据，不是物理机器人或磁干扰验收。

最终交付清单中的 `parking_prepare_bundle` 绑定 bundle、GPU 报告和独立 mock 证据的路径与 SHA。采集适配器只能读取该绑定目录，没有命令行覆盖入口：

- home→停车位、停车位→home：从新鲜实际关节检查起点，精确回放预生成 q/qd/qdd/time；起点或哈希不符即报错，不隐式改轨迹或重新规划。
- 完整绘制轨迹末点→home：调度器显式标注 `background_home` 或 `final_home` 时，才单独做完整环境碰撞规划。这不是修改绘制轨迹；未知目的或停车返程不能使用这项回退。

全部 336 条绘制／标定 recording 和双向停车资产已完成，最终只读出口审计已通过；需要复核时运行：

```bash
cd /home/huaxin/Codes/ur-waypoint
bash workstation_magtrack/ops/overnight_delivery.sh --once
```

已发布目录为 `workstation_magtrack/reports/overnight_delivery_20261017_v1/`，包含 `delivery.json`、`delivery.csv`、`collection_manifest.json`、`README.md`、`sha256sums.txt` 五文件，绑定全部 336 条本体和双向停车资产。出口不覆盖不同内容的既有交付；缺失或未通过会报 `accepted:false` 并退出 2，不创建伪完成目录。`--wait` 只等待已确认存活的相关进程，不自动重启缺失任务。

原研究交付仍保留 `simulation_only`、`real_execution_authorized=false`，旧配置和录制绝不追改。2026-10-17 用户明确：统一偏置由下游程序用低高度数据估计，再应用其余数据。因此本轮开放的是另行授权的 **RAW 真机采集**，不是“磁铁中心已标定”的验收；不再把后处理标定当成取得原始标定数据的前置条件。

## RAW 采集授权与下游偏置

`release.py` 新建独立授权文件和带 `formal_release.scope=operator_authorized_raw_collection` 的清单；名称中的 `formal_release` 是历史接口字段，`formal_calibrated_release` 明确为 false。授权绑定当前操作员说明、安装 ID、原父清单、完整环境、190 mm × 5 mm 杆、原配置、全部 source/seal 和停车 GPU/mock 证据；不靠修改旧布尔值开放执行。

保存实际关节、从这些关节计算的 `plane1` 名义杆尖 FK、原始磁场及原始设备/主机时间，`bias_policy.mode=deferred_shared_bias_from_low_height_data`、`applied=false`。不在 runtime 加偏置、不重采样或填补磁场，不宣称 FK 是独立计量的磁铁中心真值。下游应另存偏置/标定版本与适用的安装 ID，生成派生数据，不覆盖 raw。若安装偏心随姿态旋转，一个固定平移是否足够仍由下游验证。

运动使用关节 FJT，参考由实际关节和冻结 URDF/tool0→rod_tip 计算，不用示教器 TCP 位姿反演。未读取 active TCP 时如实标为 `unknown_not_used_for_joint_fk`，不能伪造其已测量/标定。原始时间关联也不等于已标定 USB 绝对时延。

新授权可明确选择 `strict_v1` 或用户此前接受的 `bounded_group_loss_10_per_3000_v1`。后者仅在独立复核 raw 序号、计数、窗口覆盖和机器人映射后接受不超过 10/3000 的少量丢组；漂移、过期、epoch、硬采集错误及映射门槛均不放宽。旧 strict 报告保持原状态，另外保存 `action_*.operational_acceptance.json`；不把旧失败数据追改为通过。

## RAW 采集的统一启动与可靠性边界

ROS 栈入口为 `ops/start_mla_short.sh`，采集入口为 `ops/collect_reliable.sh`；两者共用 `ops/ros_environment.sh`。共享环境固定 ROS Jazzy、父项目 install、`.venv`、当前源码，以及 `curobo.lock` 绑定的 `.deps/curobo-<commit>` 和 Python ABI 对应的 venv site-packages；使用 `rmw_fastrtps_cpp`、domain 0、`SYSTEM_DEFAULT` discovery，以及 `ops/fastdds_loopback_udp.xml` 中显式的 loopback UDP transport，同时取消 `ROS_LOCALHOST_ONLY` 和 `FASTDDS_BUILTIN_TRANSPORTS`。不要再混用旧的 `ROS_LOCALHOST_ONLY=1` / `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` 命令。

旧的裸 `ros2 launch` 命令若只 source ROS／install，会漏掉 planner 所需的锁定依赖路径。本次实际故障中的已安装 `curobo_server` 使用 `/usr/bin/python3`，仅激活 `.venv` 不会改变它的 build-time shebang；采集客户端环境正确也不会修复已经运行的 server。不要通过自动重启掩盖该故障。

终端 A：仅在获准启动、且操作员确认旧栈已停止后执行以下**实际真机启动命令**：

```bash
cd /home/huaxin/Codes/ur-waypoint/workstation_magtrack
bash ops/start_mla_short.sh --robot-ip 192.168.1.99 --launch-rviz true
```

栈入口先使用已安装 `curobo_server` 的 shebang 解释器执行锁定 cuRobo／PyTorch 的 CPU 导入预检，核对来源、ABI、版本和所需 API；不做 CUDA availability、solver warmup 或机器人运动。真正启动前取得 `.mla_stack.lock`，锁被占、ROS graph 查询失败，或发现 `/controller_manager`、`/hx_waypoint_runtime`、`/hx_waypoint_curobo_server`、`/move_group` 时拒绝启动；它不停止或替换已有栈。驱动连接后，仍须操作员在 UR5 示教器按 Run／External Control，脚本不会代按，也不打开 NUCLEO。

不接触设备的入口检查：两个入口的 `--dry-run` 都不加载 setup，只打印命令；`start_mla_short.sh --env-report` 加载共享环境并执行上述 CPU 导入预检，但不启动 ROS 节点，也不检查现有栈／GPU／live planner；`collect_reliable.sh --env-report` 只核对并加载环境，其 `--dependency-preflight` 才执行相同导入预检。这些均不是实际采集或长时稳定性验收。

终端 B：下面是**同一 boot 已经初始化并仍在出流**时的参数模板，不是刚重启且尚未初始化的新 boot 命令。先替换 boot ID、新输出目录和同 boot 背景收据，默认只打印；没有既有背景时才省略 `--prior-background-provenance`：

```bash
cd /home/huaxin/Codes/ur-waypoint/workstation_magtrack
bash ops/collect_reliable.sh --dry-run -- \
  --manifest collection/raw_authorizations/raw_20261017_v1/collection_manifest.json \
  --output runs/raw_collection_NEW_RUN \
  --boot-id CURRENT_HARDWARE_BOOT_ID \
  --reuse-nucleo-stream \
  --prior-background-provenance '<同boot已接受背景的哈希绑定收据.json>' \
  --port /dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_3879336C3333-if00
```

真正派发采集须去掉 `--dry-run`，这会控制真机；不能与另一采集进程并发。入口只执行一次命令，环境缺失、健康检查或采集失败都直接停止，不切换系统 Python、不自动重启 ROS、不自动重握手或重播。

启动顺序为：快速核对 RAW 授权／原配置 → **只读 ROS readiness** → 完整 source/seal 清单校验 → 打开采集会话。readiness 会实际请求 runtime 参数，检查工作区身份、唯一 active 的环境绑定控制器及其 FJT action，并要求至少两份时钟推进、源时间和接收时间均不超过 0.5 s 的完整实际关节反馈；可收到的 UR program=false 也会直接拒绝。它不派发运动、不打开串口，不能只凭“节点／action 能发现”当通过。

终端会先显示 `startup_ros_preflight_started/passed`，再显示 `startup_inventory_verification_started`；失败证据写入新输出目录的 `startup_ros_readiness_*.json`。readiness 有观测预算，后续 runtime identity 服务等待／响应也分别有界，但这些是分阶段预算，**不是整套启动墙钟 ≤10 s 的承诺**；完整封存核验仍单独保留。失败时按报告处理既有栈，不由入口自动重启 ROS。

- 同一 boot 续用现有流必须保留 `--reuse-nucleo-stream`：只被动读取，即使无数据也不发送 SYNC。若同 boot 已有已接受背景，须使用 `--prior-background-provenance` 绑定原证据收据并计入次数；不能换输出目录逃避每 boot 最多 3 次的限制。
- 只有操作员**实际重启 NUCLEO**后，才使用新的 boot ID 并省略 reuse，允许本次新 boot 的一次初始握手；无数据不构成无限重试理由。
- 仅状态为 `waiting_for_nucleo_restart` 的正常暂停可按提示 `--resume --confirm-nucleo-restart NEW_BOOT_ID`。旧 `failed`、`incomplete` 或 `needs_attention` checkpoint 不直接 resume，也不改写为成功；确认物理状态后创建新任务及新输出，并保留旧证据。
- 每 boot 最多 3 次成功空场，随后回 home、关闭串口、等显式硬件重启；不自动跨 boot 延续时钟映射。600 s 仍只在完整轨迹边界检查，原 waypoint/time 与碰撞、动力学门槛均不改变。

`prepare-raw-acceptance` 只生成当前人类授权的哈希绑定元数据，`prepare-release` 生成新清单，`check-release` 只读校验文件；这些命令不打开硬件，也不证明安装实物或磁中心已标定。`collect` 启动前完整校验，动作前只重验当前授权/环境与本条 source/seal，避免每条再遍历全部 336 条。正常结束回 home，异常保留证据并停止，不盲目返程。

## 单条现场试采与漂移诊断

`prepare-trial` 从最终发布清单中选择恰好一条 recording，绑定父清单 SHA 和当前操作员确认；`commission` 只执行该单条试采。它与正式 `collect` 分离，不修改原配置、封存文件或正式 release 标记，也不把试采称为磁铁中心标定验收。正常完成后回 home、关闭串口；失败不自动重播或盲目返程。

试采额外保存 `commissioning_manifest.json`、`software_provenance.json`，以及从原生实际关节反馈逐行重算的 `actual_fk_reference.csv`。该坐标是 **actual-joint-derived nominal rod_tip**，不是已校准的磁铁真值；原始反馈和时间关联不重采样。

`drift.py` 同时观察每个通道的 NUCLEO/source 时钟及通道间相对时钟。默认先预热 1 s，以 1 s 稳健相位窗口统计；至少观察 5 s 后，source 对主机的相位或通道间相位持续超出 10 ms 预算即报错。NUCLEO 对主机的共同速率差单独记录，不直接等同于采集延迟：实际映射使用主机接收时间，而不是 NUCLEO 计时器。该预算不是绝对同步精度证明，USB 缓冲延迟也可能参与变化。

每次派发轨迹前检查数据新鲜度、时钟 epoch 与漂移；失败则阻止后续派发。已经在运行的权威轨迹不因漂移而被缩放、重定时或跳点；结束后保留失败证据，不自动重播。每个窗口和完整硬件会话保存 `drift.json`（含 SHA），发生越界另存 `drift_failure.json`。设备时钟回退必须新建硬件 epoch，禁止自动重握手或偷偷修正原始时钟/磁场值。

原约 600 s 空场及三次后重启暂停策略保持不变，用于限制长时累计漂移并留下背景变化证据；软件不能替代硬件重启，也不会自动把背景变化当作已校准补偿。

2026-10-17 12:41 现场试采：`runs/real_trial_20261017_1240/` 保存单条 20 s 静态轨迹的全会话 raw、窗口和漂移证据；原生机器人 telemetry 与映射表位于 `runs/commissioning_01_static_v0_w0_v4__20261016_181629__20261017_124119/real/`。机器人返回成功，实际反馈 2501 行，正式 20 s 传感器窗口 2000 组、零丢组。初版 guard 将约 400 ppm 的 NUCLEO/source 共同速率差与启动接收效应合并判为越界，因此本轮仍保留原 `incomplete/needs_attention`，没有自动返 home 或重播；后续策略修正不覆盖原报告或冒充新版真机验收。

后续现场此前已保留 28 条目标记录及旧 boot 的 1 份已接受背景。2026-10-17 19:41:28 CST，RAW 采集 `resume9` 通过统一入口启动；当时的历史快照为：只读 ROS readiness 在 2.07 s 内通过，随后完整清单核验中，尚未打开串口，新 boot `191358` 尚无新背景或目标记录。该快照仅描述当时阶段，不是当前运行状态。

`resume9` 后续结果保存在 Ubuntu24 的 `runs/raw_collection_20261017_resume9/`，现已停止于 `needs_attention`，不是正在采集：

- 初始 30 s 背景收到 2998/3000 组，缺 2 组，按已授权 bounded-loss 策略的 operational acceptance 为 `accepted=true`，30 s guard 通过；背景证据为 `action_96df678c52bc4efeb93b57a0e3793dcb.background.json`，19:46:05 CST 已回 home。这是 boot `191358` 的第 1 份已接受背景。
- 随后第 29 条在准备规划阶段因 locked cuRobo unavailable 失败，没有执行绘制；证据为 `action_89beb335dfe7496bbc795a42df7e8651.play.log`。已确认根因是当次重启 ROS 栈遗漏 `.deps` 和 venv site-packages 的 `PYTHONPATH`，已运行 server 的解释器为 `/usr/bin/python3`；不是本轮 NUCLEO 握手失败。
- `sensor_sessions/7bdc1056457747078fc4af89f021ad93/session_manifest.json` 为 `status=closed`、`capture_error=null`、`clock_drift.error=null`。本次未新增第 29 条目标记录，也未完成新版长时真机 soak；既有数据不追改为新版程序的验收证据。

共享环境 helper 与新栈入口是对此次启动缺陷的修复，软件验证和后续现场运行应另存证据；`needs_attention` 不直接 resume。历史 boot `191358` 的 1 次背景及收据保持原样，只能用于该 boot 的证据追溯，不能应用于已经再次重启后的会话。

操作员随后实际重启 NUCLEO，远端设备节点的 stat 时间为 **2026-10-17 19:53:57.147 +0800**，当时曾拟用 boot ID `195357`。此后操作员确认同日 **20:03:32 CST 又一次实际重启**，并明确授权该新 boot 的一次初始 SYNC；因此 `resume10` 使用新 boot ID **`200332`**，不再使用 `195357`。新 boot 不得携带 `191358` 或更早 boot 的背景收据，也不能继承历史握手或旧时钟映射；一次初始 SYNC 的授权不允许重复握手或失败重试。

操作员已明确授权再次启动 ROS，并执行 home→30 s 背景→home→从第 29 条继续；新运行保留 `resume9` 的失败 checkpoint 和原始报告，不直接 resume 旧失败状态。**2026-10-17 20:06:09 CST 的启动历史快照**：`resume10` 已启动，仍使用同一 manifest，目标记录为已接受 28 条、待采 308 条（不含背景）。此处不宣称新背景、新目标或新版 real passed；不授权自动重启、无限重试或重复 SYNC，后续结果只以独立证据为准。

## 代码测试

上一轮完整软件回归结果：Ubuntu24 为 764 passed、241 subtests；Mac 为 765 passed、241 subtests；C++ 为 97 passed，证据汇合于 `runs/collection_reliability_fix_20261017_v1/`。共享环境 helper 与 ROS 栈入口的后续 Ubuntu24 专项测试为 **73 passed＝collect 44＋start 29**，另经独立审阅通过；这是独立专项结果，不与旧全套测试相加，也不以旧测试数冒充修复后验证。此前 source-phase 与静止观察生命周期修复中“8／2”是绑定源码文件数量，不是测试项数；205 项是前阶段组合 CPU 回归记录，不冒充新版全套测试。

2026-10-17 20:02:57 CST 启动的新 ROS 栈在实际安装环境下通过锁定依赖导入预检；启动后 live server 的正常 warmup 返回 `success=True`。日志为 `runs/mla_stack_lockeddeps_20261017_v1.log`，部署确认元数据为 `runs/collection_dependency_fix_20261017_v1/resume10_newboot200332_dependency_deployment_acknowledgement_v1.json`。CPU 导入预检本身不做 warmup；后续 live warmup 通过也不是新轨迹、串口长期运行或新版真机可靠性通过证明。

```bash
cd /Users/huaxin/Codes/ur-waypoint
PYTHONPATH="$PWD/src/hx_waypoint_curobo:$PWD/src/hx_waypoint_nucleo" \
  .venv/bin/python -m pytest workstation_magtrack/collection -q
```

实现按 `daily-coding` 的最小隔离修改与单测清单；远端验证按 `run-experiment` 先检查资源、同步必要代码并保存日志。单元测试不连接设备；现场试采与仿真、离线复算证据分别标注。
