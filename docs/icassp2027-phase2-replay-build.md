# ICASSP 2027 Phase-2 回放构建

本链路为环境 `new_board_4cm_rod33365_up50_icassp_phase2` 生成静态
settled-revisit recording。每个 condition 只封存一份 3 秒 recording，真机实验时
重复回放 25 次，不生成 2350 份 recording。

## Authority 结构

每个 condition 固定为：

```csv
time_s,frame_id,tip_x_m,tip_y_m,tip_z_m,tip_qx,tip_qy,tip_qz,tip_qw,phase
0.000000000,plane1,...,draw
1.000000000,plane1,...,hold
3.000000000,plane1,...,hold
```

三行 Pose 完全相同。任何 waypoint、时间、姿态都不允许缩放、重定时、裁剪或
跳点。

论文条件中的 `p_A` 是 array frame 下的磁铁中心，运行时 authority 是
`plane1` 下的 `rod_tip` Pose。转换只能使用显式测量并冻结的 proper SE(3)：

```text
T_P_R = T_P_A @ T_A_M @ inv(T_R_M)
```

其中 `P=plane1`、`A=array`、`R=rod_tip`、`M=magnet`。注册模板
[`experiment_registration.template.yaml`](../environments/new_board_4cm_rod33365_up50_icassp_phase2/experiment_registration.template.yaml)
故意保留 null；程序不猜坐标、磁矩极性或 roll。

## 生成 authority

Nominal 94 条只能显式生成 exploratory 产物：

```bash
python scripts/generate_icassp_phase2_trajectories.py \
  phase2_nominal_main_conditions.csv \
  experiment_registration.yaml \
  phase2_authorities \
  --environment-directory environments/new_board_4cm_rod33365_up50_icassp_phase2 \
  --expected-condition-count 94 \
  --exploratory
```

正式模式不接受 `held_out_nominal_preview` 或
`DO_NOT_FREEZE_AS_FINAL_UNTIL_SIGMA_HW_RECOMPUTE`。正式 conditions 和
registration 必须同时标为 `HARDWARE_FROZEN`。

生成器强制：

- condition 数量精确匹配；
- `quat_A_M` 将 `+Z_M` 映射到 `moment_q_A`；
- 两个注册变换旋转行列式均为 `+1`；
- 每个目标 `z_rodtip_P <= -0.072 m`，即当前传感器面以上至少 65 mm；
- conditions、registration、environment、authority 全部 SHA-256 绑定；
- formal/exploratory 目录隔离，bundle 使用 staging 后原子发布；
- 已有异内容绝不覆盖。

## 批量 GPU compile、mock、inspect

```bash
python scripts/build_icassp_phase2_recordings.py \
  --workspace /data/ur-waypoint/icassp2027_phase2/workspace \
  --authority-index /path/to/exploratory/phase2_authority_index.csv \
  --exploratory \
  --output-root /data/ur-waypoint/icassp2027_phase2/build_evidence \
  --fail-fast
```

构建器只允许下面两个命令：

```text
tipdraw build-recording
tipdraw inspect
```

它使用独立、仅本机发现的 ROS domain 和 mock hardware；不包含 `play`、Dashboard
或 real controller 入口。94 条严格串行，共用一个 simulation runtime；失败记录保留，
相同 hash 的 validated recording 可在再次运行时直接 inspect 后跳过。

canonical `runs/` 和 `recordings/` 必须位于同一个完整 workspace，禁止只对其中一个
目录做软链接。Ubuntu24 的完整 workspace 位于：

```text
/data/ur-waypoint/icassp2027_phase2/workspace
```

## 当前必须补齐的物理注册

生成任何真实 authority 前需要记录：

1. array `+X` 在 `plane1` 中的实体方向；结合已知中心和 `+Z_A=-Z_P` 后形成完整右手系；
2. 磁铁中心相对 `rod_tip` 的 `[x,y,z]`，单位 m；
3. 正磁矩 `+Z_M` 相对杆轴的方向/极性；
4. 磁铁 roll datum，或明确安装严格共轴轴对称且 roll 无物理意义。

缺少上述任一项时，生成器在创建任何 CSV 或 recording 前直接失败。
