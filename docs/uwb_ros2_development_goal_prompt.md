# UWB ROS2 开发 goal prompt

日期：2026-10-26（Asia/Shanghai）。本文件是下一轮开发的启动提示词，本次保存文件不表示已经启动或完成驱动实现。

复制下面完整内容到以本仓库为工作目录的开发会话中执行：

---

请根据以下规范，实际完成 UWB ROS2 驱动的软件开发、测试、文档更新和本地 Git 提交，持续推进到软件完成条件满足，不停留在规划阶段。

工作目录：`/home/dev/code/ws_tunnel/src/nlink_parser_uwb`。

首先阅读适用的 `AGENTS.md`，然后完整阅读：

1. `docs/uwb_driver_design_notebook.md`：协议、数据边界、硬件条件、黄金测试向量。
2. `docs/uwb_ros2_development_requirements.md`：本次范围、ROS2 决策、D0–D7 任务与 S1–S9 验收。

优先级为用户明确要求、适用仓库指令、本需求书的 ROS2/本地差异说明、原 handbook。发现协议冲突时记录具体字段和证据，不擅自改 wire format。ROS2 是明确要求，不能退回 ROS1/catkin 作为最终交付。

## 目标

在 ROS 2 Jazzy/Ubuntu 24.04 上实现：

```text
N0 唯一串口所有者 nlink_parser
  → 原始 NLink 帧与带接收上下文的 stamped 帧
  → liuo_uwb_bridge
  → /liuo/uwb/observations（liuo_uwb_msgs/msg/UwbObservationBatch）
  → 后续 LIUO 接入

固定节点本地测距帧
  → MCU 平台无关冻结快照/分包/调度逻辑
  → UWB 自带数传的 LIUO Range Relay v1 envelope
  → N0 的 Frame0/6 内层发送者 block
```

本地动态流事件驱动、有效边报告目标 ≥20 Hz；固定端每 200 ms 覆盖全部有效固定边。软件模拟可以验证处理能力，不能证明射频实测性能。主机与 MCU 逻辑必须完成，板级工程和实机测试依赖真实设备，未执行项明确保留。

## 已知基线与必须解决的差异

- 本地快照 commit：`5680f766f9b11a8426795f0589b1ccbfe9432af8`。这是已修改的 ROS1 导入代码，不应声称与 handbook 上游 SHA 完全一致。
- 现有包名 `nlink_parser`；顶层及 `extern/serial` 都依赖 catkin。保留来源与许可证，处理两层构建迁移。
- 本地 Frame3 已带 Header；部分其他回调用主机 Unix 微秒覆盖设备 local_time。恢复设备毫秒语义，保留原消息字段声明并记录行为变化。
- 当前依赖为 vendored 文件，不是真正的 Git submodule；不要盲目运行更新命令或用最新 master 覆盖。
- 相邻 LIUO 的 `UwbRange` 要求有效测量时间和方差，不能直接替代新消息。交付接入合同，未经明确范围扩展不修改 LIUO 优化器、不虚构方差或同步状态。

## 执行顺序

1. 检查 Git 状态、现有实现、Jazzy 与数据盘启动器；保留用户改动。创建任务分支，记录基线与实际环境。根仓库重排为需求书的 `packages/` 多包结构，避免 colcon 嵌套包发现问题。
2. 实现无 ROS 协议核心，先通过附录 A 黄金向量与附录 B 参考实现的交叉测试。
3. 将 LinkTrack 构建/消息/节点迁移到 ament、rosidl、rclcpp，去除活动构建链中的 catkin。保留 LinkTrack 原始话题及帧支持，复用已有测试数据。无关 TOFSense/AOA/IoT 保留源码但不强行扩展迁移范围。
4. 在 unpack 前完成长度、checksum、节点与 block 边界验证；加入有限缓冲、解析失败门控、接收 context、stamped、串口重连和默认关闭的 legacy TX。
5. 实现统一消息、身份注册表、按发送者拆粘 envelope、即时 bridge 输出；默认 Frame3/0，同时对 Frame2/5/6 完成适配与测试。
6. 实现可验证的帧级时间映射、会话管理、包/源帧观测去重、回放策略、诊断和有界资源。
7. 实现 MCU 平台无关快照/5 Hz 完整轮次/过期丢弃逻辑、HAL 接口与 host 模拟测试。板型未确认时不编造引脚和固件工程。
8. 提供 ROS2 launch、合法参数 YAML/独立注册表、QoS 与录包配置、构建测试工具；通过 PTY 做完整串口→观测链路、故障与负载验收。
9. 更新 README、迁移说明、LIUO 接入边界和软件验收报告。按 D0–D7 小步提交 Git，最后检查提交范围与工作区状态。

可调整实现顺序以减少返工，但必须覆盖需求书全部必需合同。缺硬件时继续完成所有可做的软件工作；环境阻碍需给出实际错误和替代验证范围，不能把未编译的程序称为完成。

## 不可改变的合同

- 协议：`~U:` + 标准 Base64 + LF；little-endian；30 B 头、8 B 边、2 B CRC；最多 5 条/100 B；CRC-16/CCITT-FALSE；逐字节匹配黄金向量。
- 结构校验先于 unpack，失败不能发布陈旧全局结果；单串口/单解析线程；bridge 不打开串口。
- 完成某帧的 read 提供其 ROS/monotonic 接收时间，每帧仅一个 context；同批可同时间但序号不同。重连清半帧并更新端口会话。
- Frame0/6 内层 block ID 才是远端发送者；registry 校验 reporter；DR_MODE1 32 位 ID 不截断，LP 角色不混淆；按发送者独立缓存残片。
- PacketKey 与 ObservationKey 分别去重；新轮次重发旧源帧不新增观测；新源帧同距离仍接受；双向报告均保留。所有缓存、tombstone、序号窗口有界。
- 本地与远端即时发布，缺分包不等齐；不能用 5 Hz 聚合器拖慢动态流；不能复制旧数据补频率。
- 时间是帧级语义，非逐边硬件 TWR 时间。DR_MODE1 system_time 无效；未验证共享时钟不跨节点映射；未知不确定度为 NaN；无映射 source stamp 为零且标志明确；freshness 默认 false。
- MCU 每轮完整覆盖固定邻居，8 条边每轮两包；只冻结单一源帧；禁止转发别人报告、自动多跳、逐包 ACK 风暴、长期旧帧重发。
- `enable_legacy_tx=false`、`allow_rate_degrade=false`、硬件 capability 初值 false。模式未知可采集/模拟，不能自动模式切换或发送未知管理命令。
- 无长期 Anchor 地图、滤波后冒充原始距离、NLOS 猜测或虚构方差；不增加 Wi-Fi/Ethernet 数据通路。

## 验证与存储

遵循需求书 S1–S9 与 handbook T01–T18。主要集成接缝是原始串口字节/PTY 输入到公开 ROS2 输出；协议和调度采用无 ROS 测试。特别验证坏长度后恢复、sender 隔离、重放/换轮、部分分包、32 位回绕、时钟跳变、重启旧会话、资源耗尽与慢订阅者。

通过 `data-run --check` 后，所有构建/测试/模拟/录包/分析命令经 `data-run -- ...` 执行，输出目录显式位于 `/home/dev/ws_iap`；只扫描本仓库 `packages/`。`colcon` 的 build/install/log/test 目录均不得落到源码或系统盘。保留报告与其引用证据，不绕过存储限制。

软件负载至少包含 50 Hz 本地流与多发送者 5 Hz 完整轮次；分别报告逐边频率、队列/内存上限、处理延迟。不要用统一话题总频率证明每条边达标，也不要用源序号跳号推定广播丢包。

硬件 H0–H5 单独记录设备/固件/模式和原始证据。C1 未确认时不能声称固定边真实测距可用；C2/C5 未确认时不能声称无线透传已通过。缺硬件不是跳过软件故障测试的理由。

## 完成条件与最终报告

软件完成：需求书 D0–D7 的必需项完成，S1–S7/S9 通过，S8 有界状态验证通过；代码、配置、操作说明、验收报告均提交本地 Git。长时实测、板级和无线测试缺失时独立列为未执行，完整系统仍待硬件验收和 LIUO 接入。

最终报告用中文，给出：

1. 实现范围与关键行为变化，特别是时间字段与 ROS1→ROS2 兼容边界。
2. 可复制的构建、测试、启动、录包/回放命令及产物路径。
3. 各验收项的通过/失败/未执行状态，实际测试证据与限制。
4. Git 分支、commit SHA、剩余用户改动与待办；无远程授权不 push。

不要仅以“完成提交”“测试通过”概括；必须说明究竟是软件完成，还是软件加真实硬件完成。不能通过删除测试、放宽频率、隐藏 unknown 或伪造输入来达成完成条件。
