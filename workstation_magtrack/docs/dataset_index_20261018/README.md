# 336 条真机采集的派生入口索引

本目录是 2026-10-18 对既有验收元数据的确定性整理，不是重新采集、重新验收或物理标定。原始 `runs/` 包含历史失败尝试，不能按文件时间或 `glob` 把所有文件都作为有效训练数据。

## 文件与用法

- `accepted_trajectories.json`：336 条完整入口，按原始清单 `original_trajectory_index=0..335` 排列；`original_trajectory_ordinal=1..336`。读取顶层 `accepted_trajectories` 数组。
- `accepted_trajectories.csv`：相同内容的平面表格。纳秒时间戳须用 `int64`，不要先转换为浮点。
- `accepted_backgrounds.json`：29 组由指定来源明确列出的、已通过 operational 和 stationary 验收的背景。读取顶层 `accepted_backgrounds` 数组。此表不保证包含所有早期背景，尤其不穷举 `resume10` 以前的采集。
- `index_build_report.json`：18 个确定证据来源、各自的原始序号区间、校验范围和小型 JSON SHA256。
- `build_dataset_index.py`：可复现构建脚本，只读取 JSON 元数据并检查文件存在；不会读取原始 CSV 内容或访问 ROS、GPU、串口。

所有索引中的路径均相对于解包后的 `workstation_magtrack/` 根目录。例如，`robot_actual_telemetry_csv` 是实际 `/joint_states` 记录，`actual_fk_reference_csv` 是基于这些实际关节计算的名义 `rod_tip` FK；`planned_source_csv` 和 `planned_recording_directory` 是规划输入/回放文件，不能冒充实际位置。

传感器优先入口为 `raw_sensor_csv`；`mapped_sensor_csv` 是已按 host monotonic 接收时间映射到机器人时间的表，并非严格一一对应或硬件同步。各表单位、坐标系、190 mm 杆长、时间偏差符号、板阵列几何未标定等限制详见上一级 `DATA_HANDOFF_20261018.md`。

## 本次索引检查了什么

336 条逐项对应原始 `reports/overnight_delivery_20261017_v1/collection_manifest.json` 的轨迹 ID、recording ID 和序号；逐项确认既有 `operational_acceptance.accepted=true` 与 `actual_telemetry_audit.passed=true/status=passed`；确认原始磁场、实际机器人、派生 FK、时间映射和源轨迹入口文件存在；检查元数据相互绑定及来源给出的 JSON 哈希。没有以“最新文件”猜测成功 attempt。

CSV 的哈希取自已保存的采集证据，本次未重新扫描或重算原始 CSV。包传输另做归档哈希校验，不能把它混同为重新完成的物理精度/磁场质量审计。

strict `incomplete` 可以与已接受的有限丢组 operational 结果同时存在；此表保留两者，不修改原报告。某会话后续时钟异常也不自动推翻其先前按窗口通过的记录，但本表不宣称整个会话 clock-clean。FK audit 的数值一致性不是磁铁中心的独立绝对精度；统一空间偏置和背景扣除均未在原始数据中应用。

## 重建

在解包后的目录内执行，选择一个不存在的派生输出目录，避免覆盖：

```bash
cd /path/to/workstation_magtrack
python3 docs/dataset_index_20261018/build_dataset_index.py \
  --workspace "$PWD" \
  --output "$PWD/docs/dataset_index_rebuilt"
```

构建脚本固定选择的 18 个证据来源、哈希和区间在 `index_build_report.json` 中可查。文件定位使用相对路径；旧证据内部保存的 Ubuntu24 绝对路径由脚本映射到当前解包根目录，不修改旧证据本身。
