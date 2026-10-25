# UWB ROS2 驱动开发任务需求书

版本：v1.0；日期：2026-10-26（Asia/Shanghai）。

状态：本需求书作为冻结开发输入保留；ROS2软件实现与验收已完成，见 [软件验收报告](software_acceptance.md)。板级、真实无线、数小时实测及LIUO接入仍未执行。本文最初与 [开发 goal prompt](uwb_ros2_development_goal_prompt.md) 一起保存于开发前基线；下列合同未因验收而放宽。

## 1. 目标、依据与交付边界

将本仓库的 ROS 1 LinkTrack 驱动改造为 ROS 2 Jazzy 驱动，实现机器人 N0 的本地测距与固定节点 MCU 转发报告的并行接收，输出包含来源、时间、会话和质量标志的统一观测，供 LIUO 后续接入。

规范来源为 [UWB driver handbook](uwb_driver_design_notebook.md) v1.0，重点为 §4–6、§9–16 与附录 A–C。用户指定 ROS2，因此覆盖手册默认 ROS 1/catkin 路线；协议字节、频率定义、来源语义和硬件限制仍遵循手册。本文明确 ROS2 工程决策与本地代码差异，不将这些决策表述为厂商承诺。

交付分成三个独立状态：

| 交付层 | 本开发范围 | 完成证据 |
|---|---|---|
| 主机软件 | 必须完成 | ROS2 构建、协议测试、串口模拟集成、故障注入、文档和 Git 提交 |
| MCU 可移植逻辑 | 必须完成 | 无 ROS 的协议核心、冻结快照/分包/调度逻辑及主机模拟测试 |
| 板级与实机验收 | 有设备和型号后执行 | MCU HAL 构建/烧录、C1–C5、H0–H5 的真实记录；缺硬件时明确未执行 |

具体 MCU 引脚、DMA、RTOS 和烧录工程不预设。LIUO 优化器与现有消息消费端的改造另列后续集成任务；本轮驱动开发必须交付接入合同，不得宣称已经实现定位融合。

## 2. 已核对的本地基线

仓库路径：`/home/dev/code/ws_tunnel/src/nlink_parser_uwb`。本地导入基线：`5680f766f9b11a8426795f0589b1ccbfe9432af8`。

| 检查项 | 本地事实 | 对开发的要求 |
|---|---|---|
| ROS 版本 | `package.xml` 包名为 `nlink_parser`，依赖 `catkin/roscpp` | 这是 ROS1→ROS2 移植，不是给已有 ROS2 驱动添加功能 |
| 环境 | Ubuntu 24.04，已安装 `/opt/ros/jazzy`；相邻 LIUO 项目采用 Jazzy | 默认目标 Jazzy；其他发行版不列为本次验收平台 |
| 串口依赖 | `extern/serial/CMakeLists.txt` 也依赖 catkin | 必须解除这层构建依赖，不能只替换顶层 CMake |
| 时间字段 | 多个回调用主机 Unix 微秒覆盖 `local_time`；Frame3 已恢复设备时间 | 恢复设备毫秒时间，并显式记录兼容性变化 |
| Frame3 消息 | 本地已经有 `Header`，`local_time` 为 `uint64` | 不能照抄手册“所有原帧无 Header”的基线描述 |
| 解析器 | unpack 有共享状态；调用入口缺少完整结构检查与成功结果门控 | 先验证边界，再 unpack，成功后才发布；保持串行调用 |
| 测试 | 已有 NLink 十六进制帧和字段断言，测试依赖 ROS1 | 复用真实协议夹具，迁移行为测试，不能以新 codec 测试替代 |
| Git 来源 | 原目录无 `.git`；依赖目录为实文件，只有历史 `.gitmodules` | 已保存为本地快照；不得声称已验证与手册的上游 SHA 完全一致 |
| LIUO 输入 | `liuo_tunnel_msgs/msg/UwbRange.msg` 要求有效测量时间、方差 | 与手册 `UwbObservationBatch` 不等价，不能直接改名或静默填默认值 |

保留许可证和第三方来源说明。后续实现中明确使用 vendored 依赖还是恢复真实 submodule；默认继续 vendored，并清理会误导使用者的 submodule 配置、补来源记录。不得直接更新到浮动 master。

## 3. 工程组织与 ROS2 迁移

建议将此目录作为多包仓库，使用 `git mv` 重排，保留 Git 历史；根目录不再作为 ROS 包，避免 colcon 被外层包截断而漏发现子包。

```text
nlink_parser_uwb/
  packages/
    nlink_parser/          # 原消息、LinkTrack 驱动、串口与 NLink 解析
    liuo_uwb_msgs/         # UwbObservation、UwbObservationBatch
    liuo_uwb_protocol/     # 独立 C/C++ 协议核心，带 ament 导出适配
    liuo_uwb_bridge/       # 注册表、解码、时间、去重、诊断、launch
  firmware/anchor_relay/   # 平台无关调度与 HAL 接口；不虚构板级工程
  tools/                  # 构建、测试、模拟输入、统计与录包入口
  docs/
```

ROS2 工程必须满足：

1. 使用 `ament_cmake`、`rclcpp`、`rosidl_default_generators/runtime`；消息中的 ROS1 `time` 转为 `builtin_interfaces/Time`。C++ 主机代码采用 C++17；协议核心保持可独立构建。
2. 保留包名 `nlink_parser`、可执行程序 `linktrack`、LinkTrack 原话题名称及原消息字段名称/类型（仅进行 ROS2 必需的类型语法迁移）。ROS1 与 ROS2 不承诺 wire、MD5 或 bag 直接兼容。
3. 保留 LinkTrack 已支持的帧解析与原始发布能力；主验收覆盖本地 Frame2/3/5 和数传 Frame0/6。Frame3/0 为默认链路，Frame2、Frame5/6 必须有夹具回归，不能默认启用所有流造成重复计数。
4. TOFSense、AOA、IoT 与其可视化迁移不在功能范围；保留源码和许可，明确列出 ROS2 尚不支持的可执行程序。不能让这些 ROS1 target 导致默认 ROS2 构建失败。
5. 串口库按其实际平台源码建立普通 CMake target，不调用 vendored catkin 工程；避免把 vendor `package.xml` 当作独立 ROS 包自动发现。
6. 提供 ROS2 Python launch，显式声明串口、波特率、配置路径、命名空间和 `use_sim_time` 参数。配置采用合法 `ros__parameters` 结构；复杂注册表可作为独立 YAML 文件由路径参数加载，不能把 ROS1 `rosparam` 数组字典直接当 ROS2 参数使用。
7. 驱动与 bridge 独立进程，驱动是唯一串口所有者。一个进程只处理一个串口；解析器串行执行；应用解码不能阻塞串口读取。
8. 明确 QoS：原始/stamped/观测默认 `best_effort + volatile + keep_last(100)`，诊断 `reliable + volatile + keep_last(10)`；这是项目初值，需通过负载测试调整。有界深度、订阅兼容和 rosbag2 QoS override 都必须记录。可靠传输可显式配置，但不能掩盖积压。

构建只扫描本仓库 `packages/`，不扫描混有 ROS1 项目的整个 `ws_tunnel/src`。构建、测试、录包及分析产物必须经 `data-run -- ...` 写入 `/home/dev/ws_iap` 数据盘；遵循上级 `AGENTS.md`，不得绕过启动器或将这些产物写到源码目录/系统盘。

## 4. 功能需求与合同

### R01：串口、NLink 验证与接收上下文

- 波特率默认 921600，超时与重连间隔可配置；读取块、输入缓冲、帧长和发送队列均有硬上限。初值 `max_nlink_rx_frame_bytes=8192`、`rx_ring_bytes=16384`。
- unpack 前验证 checksum、长度、节点数量与 block 边界：Frame3 `22+7n`，Frame2 `120+13n`，Frame5 `25+10n`；Frame0/6 按各自格式逐 block 验证 `data_length` 与尾部位置，不能套用固定记录公式。
- 拒绝坏帧后能重新同步并解析后续好帧；不无限等待超长长度，不任意丢弃所有后续有效字节。处理分配失败和 unpack 失败，禁止发布上一次解析结果。
- 每次 read 返回后立即捕获 ROS 时间与 host monotonic 时间；完整帧使用完成它的 read 上下文。一个 read 内多帧可同时间，但每个物理帧有唯一 `rx_frame_seq`。
- 新增 `NLinkRxContext` 及 Frame0/2/3/5/6 stamped 包装，字段按手册 §9.2；只创建一次 context。原帧和包装载荷必须一致。
- 断线清除半帧与过期 TX，限速重连，更新 `port_session_id`；bridge 收到会话变化后清除对应残片和失效状态。重连不产生补零距离。
- 恢复厂家 `local_time` 的设备毫秒语义，保留本地消息的 `uint64` 声明以避免不必要的 schema 变化。修复主机时间覆盖属于有意行为变更，写入迁移说明。Frame3 原有 Header 使用同一个接收上下文。
- `enable_legacy_tx=false`；显式开启时才允许 `std_msgs/msg/String` 写串口，同时限制负载、速率、队列和部分写入行为。任何写操作都经唯一串口所有者。

### R02：LIUO Range Relay v1 协议核心

严格实现手册 §6 与附录 A–C：`~U:` + 标准带 padding 的 Base64 + 单字节 LF；little-endian；Header 30 B、Edge 8 B、CRC 2 B；每包 1–5 条边；wire 长度依次为 60/68/80/92/100 B。

CRC 为 CCITT-FALSE（poly `0x1021`、init `0xFFFF`、不反射、xorout 0）；`123456789` 的校验值为 `0x29B1`。必须与附录 A 的 72 B 二进制/100 B wire 黄金向量逐字节相同。

字段逐项编解码，禁止裸发结构体。严格拒绝非规范 Base64、CRLF、超长、截断、非法版本/保留 flags、零 boot、非法 shard、重复或未升序 peer、自环、非法逻辑 ID、非法距离/时间哨兵。最大 shard_count 为 51；有效距离为正且不超过源 int24 正数上限，并受项目配置的有效范围限制。

RSSI 按 `-code/2` 还原；未知 RSSI 清有效位，不能把占位 0 当成高信号质量。帧级时间、hold age 和有效位必须遵循附录 B。核心不依赖 ROS、串口、地图或 MCU HAL，并有明确错误码及有界内存策略。

### R03：身份注册表与按发送者解流

注册键是 `(system_id, operating_mode, raw_role, raw_id)`，映射到 `logical_id/is_fixed`；用 `network_instance` 隔离数据集。启动时校验重复和冲突映射。N0 逻辑 ID 为 0，固定节点为 1–254；未知节点拒绝并计数。

Frame0/6 顶层 ID 是接收者；内层 block 的 role/id 才是发送者。校验接收者属于配置的 N0，并要求内层发送者映射值等于 payload reporter。DR_MODE1 32 位 ID 不得截断，LP 不同 role 的同号节点不得合并。

为每个已注册发送者保存独立、有界 envelope 状态，支持跨 block/消息拆分与粘包，不能混拼多个发送者。残片超过 100 B、超时或会话失效后可恢复同步。使用完成 envelope 的 stamped 接收上下文；片段重组本身不产生新的测量时间。

### R04：事件驱动桥接与统一消息

新增 `liuo_uwb_msgs/msg/UwbObservation` 与 `UwbObservationBatch`，完整保留手册 §10.2–10.3 的字段与枚举，仅将 `time` 改为 `builtin_interfaces/Time`。主话题 `/liuo/uwb/observations`，诊断 `/liuo/uwb/diagnostics`。

每个本地有效新帧立即形成 `LOCAL_RANGE/ROBOT_ANCHOR` batch；每个远端有效分包去重后立即形成 `REMOTE_EDGE_REPORT/ANCHOR_ANCHOR` batch。只有诊断采用周期发布；不得等齐分包、等固定边到达或使用 5 Hz 定时器节流本地流。没有新观测时不补造数据。

保持端点、reporter、session、源帧序号、发送轮次、分包信息、原始设备时间、接收时间、有效位、RSSI 与原始距离。未知时间不确定度用 NaN；无有效时间映射时 `source_frame_stamp=0` 并设正确 `time_basis`；`pair_measurement_freshness_verified` 默认 false。batch Header 是组装时间，不是统一测量时间；标量距离 frame_id 可为空。

N0 的 `source_frame_seq` 在成功解析本地测距帧时分配，不按 bridge 回调次数生成；可从可靠 stamped 帧身份确定性派生，需定义 64→32 位回绕规则与本地会话域。远端源会话将 network/reporter/boot 映射到明确的接收数据集会话，不将随机 ID 声称为全球唯一。

### R05：时间、会话与去重

时间处理不得把新输出帧等同于新 TWR 交互。RX/超时/缓存/统计使用单调时间；ROS 时间用于接口与记录。支持设备毫秒回绕、设备重启、端口重连和 ROS 时间跳变，必要时使拟合失效并重新建模。

交付可测试的 N0 帧级时钟映射（漂移与偏移、初始化/有效/失效状态、误差说明）。接收侧单向延迟不可完全辨识，拟合不等于精确硬件同步；不确定度无证据时保持未知。`TIME_RECEIVE_ONLY` 可用于链路调试，不能作为高动态融合的同步验收。

远端仅在共享系统时钟已经验证、映射有效且属于同一时钟域时使用 N0 映射；否则保持原始时间，跨节点映射未知。DR_MODE1 system_time 始终不可用于映射。hold age 仅表示 MCU 保留时间，不是端到端时延。

去重键按手册 §12：

```text
PacketKey      = network_instance/reporter_id/boot_id/tx_cycle_seq/shard_index
ObservationKey = network_instance/reporter_id/boot_id/source_frame_seq/peer_id
```

同包重放丢弃；新轮次重发同源帧同 peer 丢弃；新源帧距离值不变仍接受；N1→N2 与 N2→N1 均保留。默认 10 s/10000 条有界缓存，限定乱序与模序号窗口，保留有界旧会话 tombstone。缓存窗口外的数据不能仅因条目已过期而自动成为新观测；会话/序号策略必须有针对性测试。

回放使用记录的 stamped 接收时间和原始设备时间，禁止用回放回调的 now 覆盖。重复播放/seek 开启新的回放数据集会话并重置瞬态状态；live 的 monotonic 值不可直接当作另一次开机的当前单调时钟，离线统计应使用记录时间差与显式 replay 策略。

### R06：固定节点可移植转发逻辑

MCU 从真实本地帧筛选固定邻居，不转发 N0 动态边，不再次转发收到的广播。每 200 ms 冻结最新一个源帧、按 peer 排序、每包最多 5 条，并在该轮发送全部分包。8 条边必须每轮发 5+3 两包，不能两轮轮流发一包。

source_frame_seq 随本地成功解析帧推进，tx_cycle_seq 随广播轮次推进；距离值不变不影响前者。分包共享冻结帧和轮次身份，各包单独计算 hold_age。无有效边不伪造心跳距离；源帧过期停止发送。

默认 hold 上限 250 ms、到期后额外排队上限 50 ms、最小 TX 间隔 25 ms；支持按 id/boot 的初相位及有界抖动。容量不足必须报告不可满足，不能静默删边或降频。TX 仅保留当前轮，部分写入计失败、接收端可重同步；无逐包 ACK、无重传风暴、无多跳。

交付平台无关逻辑、HAL 接口和 host 模拟器；板型未知不妨碍这些软件任务。模拟输入显式标记，不能生成伪“实测”报告。

### R07：诊断、配置与性能统计

能力标志默认 false，模式 unconfirmed，容量 0；允许原始采集与明确 simulator 模式，但不得因参数加载成功而把硬件标志改为已验证。默认 `allow_rate_degrade=false`、固定边转发收到的报告为 false。

诊断默认 1 Hz，短/长统计窗口 10/60 s，输出模式和能力状态、逐边有效报告率、轮次/缺分包、CRC/结构错误、未知 ID、源帧停更、串口字节率/利用率、队列峰值、处理延迟、重连、时钟映射、缓存淘汰与降级状态。UART 利用率 70% 为告警起点。

必须区分 source_frame_rate、edge_report_rate、wire_tx_rate、received_unique_report_rate、pair_measurement_rate；最后一项无独立证据时为 unknown。广播丢包使用 cycle/shard 及发送日志判断，不能把 50 Hz 源帧在 5 Hz 抽样中的跳号当丢包。仅主机不可测的 MCU TX 成功数/无线丢包率标记 unknown 或 estimate。

## 5. LIUO 接入边界

驱动交付手册的无损观测，不直接发布到现有 LIUO `UwbRange` 接口。后者要求有效测量时间和 `variance_m2`，而手册允许未知时间且不提供可信方差；两者不能无条件转换。

后续集成需要独立明确：哪些 time_basis 可进入动态融合、噪声标定与方差来源、逻辑 ID/type 映射、producer/session/sequence 分配、双向 AA 报告相关性、无效时间的拒绝策略。未完成这些工作前，仅完成观测发布与录包，不声称 LIUO 已消费。

长期 Anchor 图、静态融合、NLOS 判断、坐标估计、因子权重及地图版本全部属于 LIUO。驱动不提供虚构方差，不做滑动均值后冒充原始测量，不因距离稳定而停止报告。

## 6. 开发任务与依赖

| 任务 | 内容 | 前置 | 可验收产物 |
|---|---|---|---|
| D0 | 固定本地基线、记录上游差异、规划包布局/依赖与隔离构建 | 无 | 基线与迁移记录；本次文档已完成基线保存部分 |
| D1 | 独立协议核心与 host 参考实现对照 | D0 | 黄金向量、随机往返、全部非法字段拒绝 |
| D2 | ament/rosidl/rclcpp 迁移、serial 去 catkin、LinkTrack 原始话题 | D0 | Jazzy 指定包构建、现有协议夹具回归 |
| D3 | NLink 边界加固、接收 context、stamped、重连和 TX 开关 | D2 | 拆粘帧/坏帧/PTY 故障与时间语义测试 |
| D4 | 新消息、registry、按发送者解流、事件驱动 bridge | D1、D3 | 本地与远端两条链路可端到端发布 |
| D5 | 时钟映射、源会话、双层去重、回放策略与诊断 | D4 | 重启/回绕/乱序/ROS 时间跳变/旧包测试 |
| D6 | MCU 平台无关快照与调度、host 模拟器 | D1 | 8/18 邻居每轮覆盖、超时/过载/无回环测试 |
| D7 | launch/YAML/QoS/录包工具、负载统计、接入合同、迁移 README | D3–D6 | 可复现软件验收报告和逐任务完成状态 |
| H0–H5 | 实机能力、单节点/全节点广播、隧道覆盖、LIUO 实测 | 软件与实际设备 | 单独的硬件验收记录；设备缺失则未执行 |

按这些边界做小提交，依赖顺序可合理调整；每次提交写明范围和验证。不得为本功能顺带迁移相邻传感器仓库。

## 7. 验收矩阵与完成定义

软件验收复用手册 T01–T18，并补充 ROS2 与本地差异：

| 组 | 必须验证的外部行为 | 证据 |
|---|---|---|
| S1 构建 | 四个包可发现、生成接口、链接、安装，默认目标无 ROS1 依赖 | 隔离 colcon build/test/test-result 日志与环境版本 |
| S2 协议 | 附录 A 逐字节一致，2000 组确定种子往返；结构/CRC/flags/哨兵/距离边界拒绝 | 自动化测试报告 |
| S3 提帧 | T01–T05；各切点拆帧、粘帧、垃圾前缀、截断、错误 block 后接好帧 | 原始字节→stamped 输出断言；ASan/UBSan 检查 |
| S4 身份与去重 | T06–T13；多 sender 片段隔离、32 位 ID、同号异角色、缓存过期旧包 | 观测数量、身份、拒绝计数与恢复断言 |
| S5 时间/故障 | T14–T16；设备/端口/ROS 时钟变化、停更、回放、旧会话延迟到达 | 不冒充同步，不发布旧 unpack 数据，资源上限可检查 |
| S6 MCU 逻辑 | T17；5 Hz 完整轮次、8 条边=每秒10包；18 条边=每秒20包；无效/过期源不发 | fake clock 调度与模拟 TX 成功日志 |
| S7 集成负载 | PTY 同时注入 50 Hz 本地流及多发送者 5 Hz 轮次、慢订阅者、拔插模拟 | 逐边统计、bounded queue、立即发布/不等齐分包；注明为软件模拟 |
| S8 长时资源 | T18；加速状态过期/缓存容量边界与有条件的数小时运行 | 状态上限、内存趋势；未跑的长时实测单列 |
| S9 接口操作 | launch 参数实际生效、QoS 匹配、rosbag2 记录/回放、安装路径可用 | smoke 测试及可复制操作命令 |

主要集成测试接缝是“原始串口字节/PTY 输入 → 公开 ROS2 stamped/观测/诊断输出”；协议核心与 MCU 调度另用无 ROS 测试。不要只测试内部 getter 或让测试机械复制实现。

正式性能验收须先冻结合格边集合与统计窗口。默认使用稳态连续 60 s，并报告其中 10 s 子窗口；启动、重连、进出覆盖单列。N0 每条合格边有效报告率目标 ≥20 Hz；完整广播轮次目标 5 Hz，全部有效固定边每轮覆盖。固定边 RX 报告实际值；若要求严格 RX≥5 Hz，5 Hz TX 的丢包结果不能算通过。需要提高广播轮次时显式修改配置、重新验证动态链路不受损。

硬件证据要求手册 C1–C5 与 H0–H5；C1 不通过即不能宣称固定边在线测距完成。模拟器、串口带宽计算和 ROS 话题总频率均不能替代无线验收。

软件完成定义：D0–D7 必需内容完成；S1–S7/S9 通过，S8 的有界状态测试通过；剩余长时/硬件项目逐项标记。README、配置、示例、验收报告与代码提交齐全。不得把缺 ROS2 构建、缺关键 bridge 行为或缺故障测试降格为“仅待硬件”。

完整系统完成定义：在软件完成基础上，板级与硬件证据通过，并完成单独约定的 LIUO 接入。两种完成状态必须分别报告。

## 8. 非目标与待确认项

非目标：射频固件、TWR/MAC/空中 TDMA、自组网多跳、自动模式切换、自动刷机、Wi-Fi/Ethernet 数据通路、全网离线采图、按需请求替代持续广播、长期地图与 SLAM 优化，以及未授权的远程 Git 推送。

部署前需补齐设备型号/固件/模式、C1 固定边来源、实际帧类型、100 B 透传边界、时钟域证据、MCU 板型、合格链路集合及 RX≥5 Hz 是否为严格硬指标。这些缺失不阻止无硬件软件开发，不得用默认配置伪造答案。

## 9. Git 与文档交付记录

本仓库原无 Git 元数据，已在驱动目录内初始化独立仓库。`5680f766f9b11a8426795f0589b1ccbfe9432af8` 保存导入的 ROS1 源码、vendored 依赖及原 handbook；不是上游提交 SHA，也不是 ROS2 实现提交。随后提交本需求书和 goal prompt。未配置远程仓库，不执行 push。

开发阶段继续在此仓库内提交，仅纳入本任务文件；保留用户已有改动，禁止覆盖相邻项目。报告完成项、测试证据、未执行项及 commit SHA，不以干净 Git 状态代替功能验收。
