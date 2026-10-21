# mla_short 真机336条数据包

本包由 Ubuntu24 上已完成采集的 `workstation_magtrack` 工作区打包，供 workstation 上的离线处理使用。打包不会移动机器人、打开传感器串口或修改原始采集值。

## 阅读顺序

1. `docs/DATA_HANDOFF_20261018.md`：环境、190 mm×5 mm名义杆、坐标系、单位、磁场格式、时间映射、背景与标定边界。
2. `docs/dataset_index_20261018/index_build_report.json`：派生成功索引的构建证据与覆盖限制。
3. `docs/dataset_index_20261018/accepted_trajectories.json` / `.csv`：原清单336条对应的成功采集入口，不从runs目录名猜测成功尝试。

索引是根据既有验收JSON整理的派生目录，不是重新扫描全部CSV、重做质量检查或新增物理标定。原始JSON、CSV和时间戳不重写。本包的成功索引在最初交接说明写好后补齐；关于索引的最新状态以本文件与构建报告为准。

## 范围

包含本工作区的 `runs`、`reports`、`environments`、`collection`、`inputs`、`docs`、来源/生成/验证相关代码及元数据。`runs`保留完整采集会话、逐轨迹实际关节与FK、sensor映射表、同期30秒停车背景、验收链，也保留失败尝试和仿真证据。

**包里有文件不等于该文件通过业务验收。**用成功索引及其引用的 operational acceptance、actual FK audit、窗口/boot证据选择数据；不要把所有CSV合并成训练集。仿真CSV不是真机测量，原始session流还含转场/等待。背景有效性也须按对应验收选择。

排除工作区的 `.deps`包外依赖链接、`.mla_stack.lock`、Python缓存、ROS日志`latest`链接；未包含独立旧批次 `backgrounds/`（100条×120秒背景）。**本次336条同期30秒背景仍完整位于包内 `runs/raw_collection_.../sensor_sessions/.../windows/background_.../` 等路径。**

## 解压与路径迁移

归档顶层只有 `workstation_magtrack/`。可选择一个新的空目录作为DEST，执行：

```bash
mkdir -p "$DEST"
tar --zstd -xf /path/to/mla_short_real336_20261018_bundle_v1.tar.zst -C "$DEST"
```

解压后的工作区根目录是 `W_NEW="$DEST/workstation_magtrack"`。命令中的DEST和压缩包路径由使用者指定；不要直接覆盖现有采集工作区。

原始元数据中有Ubuntu24的绝对路径，故**不能把原文中的绝对路径直接在workstation上使用**：

```text
W_OLD = /home/huaxin/Codes/ur-waypoint/workstation_magtrack
```

处理时采用只读路径解析规则：

- 以`W_OLD/`开头的路径，替换该前缀为`W_NEW/`。
- 相对路径（例如`runs/...`、`reports/...`、`environments/...`）相对于`W_NEW`解析，不相对于JSON所在目录。
- 其他绝对路径不能无条件替换或默认为存在，应报告并按来源另行确认。
- 不重写包内原始元数据以“修复”路径；在程序内建立路径映射，派生结果另外保存。

例如：

```python
from pathlib import Path
W_OLD = Path('/home/huaxin/Codes/ur-waypoint/workstation_magtrack')
W_NEW = Path('/your/extracted/directory/workstation_magtrack')

def resolve_dataset_path(value):
    path = Path(value)
    if not path.is_absolute():
        return W_NEW / path
    try:
        return W_NEW / path.relative_to(W_OLD)
    except ValueError:
        raise ValueError(f'包外绝对路径，需另行确认: {value}')
```

交接说明第9节示例在原Ubuntu24上可直接使用；迁移到workstation时，将W改成W_NEW，并用这里的resolver处理`window.mapping.path`等原绝对路径。时间值、boot/epoch、单位和坐标变换都不因文件迁移而改变。

## 完整性与测量边界

归档旁的`.sha256`用于验证整个包的传输完整性。包内原manifest/audit的hash用于原始资产追溯；成功索引构建不代表重新哈希全部原始CSV或证明绝对物理精度。

当前标签为实际关节推导的**名义rod_tip**，未施加共同空间偏置、未扣背景；磁铁中心/阵列几何轴向和绝对时间延迟仍按交接说明处理。不会因打包自动完成这些校准。
