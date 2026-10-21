# 2026-10-19 源码发布与数据边界

本次发布以 Ubuntu24 实际使用的程序为准，仅同步程序、环境配置、依赖锁、测试与说明；不把本次实验数据或已生成的回放文件加入 Git。原有 Git 历史中的样例/验收资料保持不变。

## 包含内容

- `src/`、`scripts/`、`tests/`：ROS 2 Jazzy 工具、离线编译、权威 recording 播放、同步采集和相关修复。
- `environments/mla_short/` 和 `workstation_magtrack/environments/mla_short/`：各自完整的非数据环境配置、机器人模型与标定文件。冻结环境按字节保留，不改其指纹。
- `workstation_magtrack/`：轨迹/标定动作生成、平滑随机变换、仿真验证、回放准备、批次调度、停车空场采集、独立传感器采集进程、会话时钟/质量门槛，以及相应测试。
- 生成协议、静态 contract 配置、数据交接说明和数据索引生成器。说明中提及的数据路径不是 Git 中已附带的数据。

## 不包含内容

原始 UR/NUCLEO 表、背景场、生成的轨迹 CSV、recordings、runs、batches、planning 索引、报告产物、压缩包、缓存、编译环境，以及历史 `raw_authorizations`、commissioning、prepared_parking 和 provenance 运行快照均不在本次提交中。数据集目录下仅保留明确的源码和说明，不附带已生成的数据索引。

数据包已经通过独立数据交接流程提供；不要把历史授权复制成新环境的默认放行配置。

## 新克隆的使用限制

Git 克隆不等于已经部署运行环境。仍需按项目说明安装 ROS/UR/MoveIt 依赖、构建工作区，并配置锁定的 cuRobo 和 `.deps`。依赖目录、构建结果和 GPU 编译缓存不随源码发布。

纯单元测试使用 fake backend 或临时生成的输入；读取既有实验资产的集成/验收测试需要对应的外部数据包。`test_new_board_trajectories_match_the_authoritative_schema` 在缺少完整历史 CSV 集合时明确标记 skip，恢复完整集合后仍严格核对文件清单和每条 CSV schema，不把缺数据当作通过验收。

`generation/calibration.py`、`generation/pilot.py` 和连续轨迹纯生成入口可用于创建新的输入，再按生成 → 仿真规划/验证 → 封存 recording 的流程准备资产。既有实验的回放、批次恢复或数据索引命令依赖相应数据包，不能在空的新克隆中直接执行。

特别是 `continuous_augmented/generate_extension.py` 会校验历史 v2 manifest、provenance/source_inventory、参考源码和旧 CSV/meta；`replay/prepare.py`、assemble/audit 等入口也依赖已生成的 authority/planning 资产。缺少这些文件时应明确报错；需恢复匹配资产，不能删除哈希门槛、伪造索引或放宽验证来绕过。

## 不变的执行约束

所有 waypoint 和 `time_s` 仍为权威输入，runtime 不做缩放、重定时、裁剪或跳点；不满足约束直接报错。生成阶段设计新轨迹与执行阶段修改已封存轨迹是两回事。

`collection/config.yaml` 默认保持 `simulation_only: true`、`real_execution_authorized: false`。真机必须完成独立放行、环境/控制器核对及碰撞检查；发布源码不授予运动权限。NUCLEO 已建立会话复用不重复 SYNC；新硬件启动的首次 SYNC 需按操作流程授权。
