# LIUO UWB 驱动设计与修改 Handbook

> 实施状态补记：本文保留原始设计与协议约定；当前ROS2软件交付及真实测试证据见[软件验收报告](software_acceptance.md)。下文ROS1路线属于原始设计背景，ROS2迁移差异按[需求书](uwb_ros2_development_requirements.md)执行。板级/无线实测未完成。

## 基于 Nooploop `nlink_parser` 的方案 B：固定节点 + MCU + UWB 自带数传

**版本：** v1.0  
**编写日期：** 2026-10-25  
**文档类型：** 可实施设计规范与修改路线，不是已经部署、编译或完成射频实测的驱动。  
**主要对象：** 机器人侧 ROS 驱动开发者、固定节点 MCU 开发者、LIUO SLAM 接入开发者。  
**默认软件路线：** 保留现有 ROS 1 / catkin 驱动，最小修改；协议核心不依赖 ROS，便于后续移植。  
**关键指标：** N0 本地有效距离输出目标不低于 20 Hz；每个固定节点的邻接固定边完整广播轮次为 5 Hz，分包后不能把单条边刷新率偷偷降到 5 Hz 以下。

> **本次确定的分工：** UWB/MCU/驱动持续发送、接收和标记观测；Anchor–Anchor 图的长期保存、静态融合、版本管理和 SLAM 因子复用由 LIUO 负责。驱动不承担静态地图构建，不采用“先全网切换模式、离线采图、再切回定位”的流程，也不把按需请求作为正常数据通路。
>
> **硬件边界：** 本方案转发的前提是固定节点的串口已经能够输出“本固定节点到其他固定节点”的距离。MCU 和 ROS 驱动不能创造固件没有测量或没有开放的距离。文档会明确区分手册支持的工作模式与尚未确认的实机能力，而不会把已有讨论中的推测当成硬件事实。

---

## 阅读路径

| 你要做的事 | 优先阅读 |
|---|---|
| 判断现有硬件能否实施 | §1–§3，尤其 §2 的前置条件 |
| 在 `nlink_parser` 基础上修改机器人驱动 | §4、§9–§12、§15–§16 |
| 编写 STM32 / ESP32 程序 | §5–§8、§13、附录 A–C |
| 设计 ROS 与 LIUO 接口 | §10–§12、§14 |
| 验证 20 Hz / 5 Hz 是否真正达到 | §3、§8、§13、§16 |
| 检查协议字节与代码一致性 | §6、附录 A–C |

文中 `[S1]`–`[S9]` 指文末可定位的原始资料。标记为 **“设计规定”** 的参数、报文、话题和验收阈值，是本项目的建议规范，不是厂商已经提供的功能。代码块也按“原仓库已有”“拟新增接口”“伪代码”“已执行的协议参考代码”分别说明。

---

## 1. 需求冻结：这版实现什么、不实现什么

### 1.1 系统行为

机器人携带 UWB 节点 **N0**，通过 USB 或 UART 接车载电脑。每个固定节点 **Ni** 接一个 MCU。Ni 的模块完成现有固件支持的测距；MCU 读取本地输出，将其中固定节点间的距离以 5 Hz 的完整报告轮次回写给同一个 UWB 模块，使用现有数传能力发出。

N0 的同一串口会出现两类输入：

- 本地测距帧：N0 到当前有效固定节点的距离。
- 数传帧：其他固定节点 MCU 发送的邻接边报告。

机器人驱动把两类输入规范化为同一种 ROS 观测消息，交给 LIUO。两类观测可以异步到达，不能为了凑齐固定边而阻塞 N0 的动态测距。

### 1.2 明确排除

本版不修改 UWB 射频固件，不自己实现 TWR，不承诺控制空中时隙，不实现自动多跳 Mesh，不自动切换 LP/DR/DT 工作模式，不自动刷固件，不执行一键标定，不把厂家求解的坐标当作原始测距，不在驱动里长期保存或优化 Anchor 地图。

本版也不引入 Wi-Fi/Ethernet 作为数据通路。ESP32 即使带 Wi-Fi，其无线网络能力也不是本设计的依赖。

### 1.3 “Anchor”与厂家“ANCHOR”的术语区别

本文的 **固定节点/Anchor** 是物理用途，表示设备固定安装。它不必等于 LinkTrack LP 模式中的枚举角色 `ANCHOR`。

厂家资料中，LP 模式区分 TAG/ANCHOR/CONSOLE；DR 模式的测距对象为 NODE。不能把“物理上固定的 NODE”与“LP 模式的 ANCHOR”混为一谈，更不能据此推断不同模式能够互通。[S1：§3、§4；S2：§1.5]

---

## 2. 前置条件与能力边界

### 2.1 先明确一个不能由软件绕过的条件

用户已指出，现有空循环设备不能直接执行此前讨论的“全网切换、自动获取完整 Anchor 图”的流程。本设计不依赖该流程。

但无论怎样设计转发协议，都需要一条真实的数据来源：

```text
固定节点 Ni 的本地串口
    必须能输出 Ni—Nj 的距离
    其中 Ni 和 Nj 都是固定节点。
```

若实机实际工作在仅提供 TAG–ANCHOR 距离的配置，而且 Ni 端不能获得 Ni–Nj 距离，则：

```text
可以继续开发并测试消息、转发和 ROS 接口；
不能宣称 Anchor–Anchor 在线测距已经实现；
必须先由现有受支持配置、厂家提供的功能，或另一个真实测距来源解决数据来源。
```

**加 MCU、调高串口速率、改 ROS 消息，都不会自动获得缺失的 Anchor–Anchor 距离。**

### 2.2 文档支持与实机状态分开记录

| 能力 | 原始资料支持什么 | 本项目应如何处理 |
|---|---|---|
| 节点读取自身到邻居的距离 | DR 模式说明支持本节点到其他节点的距离输出 | 视为候选能力，不等于用户实机已确认 |
| N0 自动输出整个固定节点图 | 手册明确不能直接得到其他两个节点之间的距离 | 必须通过本方案 MCU 转发 |
| 测距期间传用户数据 | LP/DR 模式有自带数传；接收端输出相应数传帧 | 先验证本地设备的实际帧与长度限制 |
| 全网统一精确测量时间戳 | DR_MODE0 提供系统时间；时间定义为输出协议帧的时刻 | 不能当作每条边的精确 TWR 时刻 |
| DR_MODE1 的 `system_time` | 用户手册 §7.11 指出其为无效值 | 不允许用其做跨节点时间同步 |
| 任意节点数仍保持 ≥20 Hz | 不支持这样的泛化承诺 | 按模式容量和实测结果验收 |
| MCU 精确控制射频发送时刻 | 提供的是 UART/USB 数传接口，不是开放 MAC 调度接口 | 只能做应用层限速和错峰 |

依据：[S1：§4.2.1.4–4.2.1.5、§7.11、FAQ Q38；S2：§3.2.2]。

### 2.3 最小能力验证，不需要先买齐 MCU

只做三个模块的检查：N0 为机器人节点，N1、N2 固定。

| 验证项 | 输入/操作 | 应看到的结果 |
|---|---|---|
| C1：本地固定边 | 读取 N1 串口 | 存在 N1–N2 的有效距离记录 |
| C2：同时数传 | 向 N1 写入一条已知用户负载 | N0 在数传帧中收到同样负载，测距帧仍继续输出 |
| C3：输出格式 | 保存原始串口数据 | 确认实际是 Node_Frame2/3/5 与 Node_Frame0/6 中的哪组 |
| C4：动态更新 | N0 移动，固定节点保持不动 | 本地距离输出目标 ≥20 Hz，并检查每条合格链路 |
| C5：实际透传容量 | 按本协议发送 100 字节负载 | 内容完整；没有截断、长期积压或意外配置响应 |

没有 C1，就停止 **硬件功能验收**，不要伪造固定边；其余软件仍可使用明确标记的模拟输入开发。没有 C2，就不能把“双向串口通信正常”当成“无线转发正常”。

### 2.4 关于 DR_MODE0：参考能力，不是自动切换要求

用户上传的数据手册中，DR_MODE0 的容量与最高输出频率如下：[S2：p.26 表12]

| 配置容量，含 N0 | 最大可选输出频率 | 每节点标称数传速率 | 无缓存用户负载上限 |
|---:|---:|---:|---:|
| 5 | 200 Hz | 160,000 bit/s | 100 B |
| 10 | 100 Hz | 80,000 bit/s | 100 B |
| 20 | 50 Hz | 40,000 bit/s | 100 B |
| 50 | 10 Hz | 8,000 bit/s | 100 B |

这些数值仅用于**实机确实使用这一模式时**的预算。不代表所有型号、固件或长隧道拓扑都可直接达到。

特别注意：容量 50 的该配置最高为 10 Hz，不能靠 ROS 定时器或提高 UART 波特率变成满足 N0 ≥20 Hz 的方案。容量确定后降低 `UpdateRate` 主要改变对外输出频率，不能据此宣称底层测距占空比已经降低。[S2：§3.2.2]

---

## 3. 频率合同：必须分别统计五个量

### 3.1 设计规定

| 指标 | 本版默认/目标 | 含义 |
|---|---|---|
| `robot_output_rate_config_hz` | 支持时配置 50 Hz；25 Hz 可作为低余量档 | N0 的测距帧串口输出配置，不是 ROS 插值频率 |
| `robot_min_valid_edge_hz` | ≥20 Hz | 合格、持续可见的 Robot–Anchor 链路在接收端的有效报告率目标 |
| `anchor_local_output_hz` | 支持时配置 25 或 50 Hz | 固定节点 MCU 获取本地邻接测距帧的速率 |
| `anchor_broadcast_hz` | 5 Hz | 每个固定节点发送一轮完整邻接边快照的频率 |
| `static_edge_tx_hz` | 稳定邻接条件下每条边 5 Hz | 分包后仍在每个 200 ms 周期覆盖该节点的全部有效固定边 |

“合格链路”要在测试前定义，例如持续覆盖、未刻意遮挡、已上电、处于允许距离范围内。不能在测试后只留下表现好的链路来计算频率。

### 3.2 5 Hz 的分包解释

例如 N3 有 8 条固定邻接边，本协议每个包最多 5 条，则每轮发 2 个包：

```text
每 200 ms 一轮：
    包0：边1–5
    包1：边6–8

每节点发送包数：2 × 5 = 10 包/s
每条边发送频率：5 Hz
```

禁止使用以下逻辑却仍称“所有边 5 Hz”：

```text
每 200 ms 只发一个包，两个包轮流发。
实际每条边只有 2.5 Hz。
```

### 3.3 发送率、接收率、真实测距刷新率不是一回事

每秒发送 5 次，存在丢包时就不保证每秒收到 5 次。在简单独立丢包近似下，接收率约为 `5 × (1−p)`；这只是预算，不是链路模型保证。

因此正常配置以**完整广播轮次 5 Hz**开始，同时报告接收端实际每边频率。需要严格的“接收端也 ≥5 Hz”时，应在带宽允许且不影响动态测距的前提下提高发送轮次，例如 6.25 Hz 或更高，而不是重复发布旧消息。这里调整的是 MCU 应用广播周期，不是厂家 `UpdateRate` 枚举。

另外，公开协议没有每条边的独立硬件测距序号，不能仅凭新的输出帧证明每条边均完成了一次新的射频测距。日志应区分：

```text
source_frame_rate             本地输出帧频率
edge_report_rate              每条边出现在新输出帧中的频率
wire_tx_rate                  MCU 实际发出的应用包频率
received_unique_report_rate   N0 收到并去除传输重复后的频率
pair_measurement_rate         真实每对节点测距更新率，缺乏证据时标为 unknown
```

不要用 `rostopic hz` 的结果代替全部指标，也不要用“距离值有变化”作为新测量判据。

---

## 4. 上游代码基线与复用边界

### 4.1 固定基线，避免以滚动 master 描述修改

本次读取的 `nooploop-dev/nlink_parser` 基线为：

```text
77a3238a5b73e7545a032570606492e2b7642b78
```

该基线的 submodule 指向：

```text
src/utils/nlink_unpack        1b790459c62bd7f15c18d98d9c87d13471f6db47
src/utils/protocol_extracter  a19abb76df85fa3e94f245089b258a4f36a7c404
extern/serial                69e0372cf0d3796e84ce9a09aff1d74496f68720
```

`.gitmodules` 中 serial 的 URL 为 `wjwwood/serial`；不要只按 README 中的文字另装一个不一致的串口库。[S4、S5]

### 4.2 已有、可直接复用的功能

| 文件/目录 | 已核对的现有行为 | 修改原则 |
|---|---|---|
| `src/linktrack/main.cpp` | 打开串口、读取字节、`AddNewData()`、`ros::spinOnce()`；循环为 1000 Hz | 保留流程，增加接收时间上下文、异常恢复、计数器 |
| `src/linktrack/init.cpp` | 各 NLink 帧转为 ROS 消息；含测距与数传回调 | 不删除原话题；新增带接收上下文的并行输出 |
| `src/linktrack/protocols.cpp/.h` | 各协议类型与 unpack 绑定 | 原则上不改协议字节定义；增加长度检查的插入点 |
| `src/utils/nlink_unpack` | 纯 C 解析器 | 机器人与 MCU 共享其解析逻辑；注意全局状态和内存分配 |
| `src/utils/protocol_extracter` | C++ 字节流提帧 | 机器人继续使用；MCU 可实现受限的 C 状态机 |
| `src/utils/init_serial.cpp` | 默认 921600，设置 10 ms timeout，失败时退出 | 新增可配置重连，不把打开失败伪装成正常空数据 |
| `msg/LinktrackNodeframe2/3/5.msg` | 本地测距；包含帧级时间，无 ROS Header | 保持原消息不变，用新消息包装 |
| `msg/LinktrackNodeframe0/6.msg` | 接收数传，含各发送节点的 `data[]` | 从内层 `data[]` 解本项目应用协议 |
| `launch/linktrack.launch` | `linktrack` 可执行程序；节点名 `linktrack0`；端口/波特率写在节点参数内 | 新建项目 launch；不要假定原文件已有 `arg` |
| `CMakeLists.txt` | catkin、消息生成、nutils 库、linktrack target | 只增添消息与必要依赖，保留已有 target |

原仓库已订阅 `nlink_linktrack_data_transmission`，消息类型是 `std_msgs/String`；其回调直接执行 `serial_->write(msg->data)`。接收数传已发布 `nlink_linktrack_nodeframe0/6`。[S5–S7]

### 4.3 推荐项目布局

以下为**拟新增结构**，不是声称上游已经存在：

```text
workspace/src/
├── nlink_parser/                       # fork；保留包名，最小补丁
│   ├── src/linktrack/...
│   ├── msg/NLinkRxContext.msg          # 新增
│   ├── msg/LinktrackNodeframe3Stamped.msg
│   ├── msg/LinktrackNodeframe0Stamped.msg
│   └── ...                            # 按实际帧补 2/5/6 的包装
├── liuo_uwb_msgs/                      # LIUO 输入消息
│   ├── msg/UwbObservation.msg
│   └── msg/UwbObservationBatch.msg
├── liuo_uwb_bridge/                    # 不打开串口；订阅 nlink_parser
│   ├── src/range_bridge.cpp
│   ├── src/clock_mapper.cpp
│   ├── src/diagnostics.cpp
│   ├── config/uwb.yaml
│   └── launch/robot.launch
└── liuo_uwb_protocol/                  # 可独立编译的协议核心
    ├── include/...
    ├── src/codec.c
    ├── src/crc16.c
    └── tests/...

firmware/
└── anchor_relay/                       # MCU 工程，不是 catkin 包
    ├── platform/                       # STM32 或 ESP32 的 UART/timer 适配
    ├── nlink_rx/
    ├── relay_scheduler/
    └── protocol/                       # 与机器人端使用同一协议版本
```

**不要在 ESP32/STM32 上移植整个 ROS 驱动。** MCU 复用 NLink C 解析逻辑和独立协议核心；ROS 留在机器人电脑。

### 4.4 ROS 版本与构建策略

本设计的可直接复用路线是 ROS 1 / catkin，因为上游源码采用 `roscpp`、`ros::NodeHandle`、catkin 与 ROS 1 消息定义。[S4–S7]

若 LIUO 使用 ROS 2，保留协议和串口层设计，仅另做 `rclcpp` 适配；本手册的 `.msg` 中 `time` 要改为 `builtin_interfaces/Time`。不把 ROS 1 的 `.launch`、`catkin_make` 命令写成 ROS 2 可直接运行的命令。不要为做本功能先大规模迁移全部传感器软件。

---

## 5. 数据通路和本地 NLink 帧

### 5.1 固定节点

```text
Ni UWB TX ──> MCU UART RX
                 │
                 ├─ 提取 NLink 本地测距帧
                 ├─ 按节点注册表剔除 N0/其他移动节点
                 ├─ 保留最新一帧的有效固定邻接边
                 ├─ 每 200 ms 冻结一份短期发送快照
                 └─ 编码 LIUO 应用负载
                         │
Ni UWB RX <── MCU UART TX ┘
       │
       └─ 使用厂家已有数传发送给通信范围内接收节点
```

MCU 短期缓存仅服务于异步收发和分包，一般为一帧/一轮、最多几百毫秒；它不是长期 Anchor 地图。

### 5.2 机器人

```text
N0 UWB USB/UART
    │
    └─ nlink_parser（唯一串口所有者）
          ├─ Node_Frame2/3/5：N0 的直接测距
          └─ Node_Frame0/6：每个发送者的 data[]
                  │
             liuo_uwb_bridge
          ├─ 解析 LIUO 包、检查发送者身份
          ├─ 恢复源帧时间语义
          ├─ 去除同一报告/同一源帧的转发重复
          └─ /liuo/uwb/observations
                       │
                     LIUO
               长期缓存/融合/建图/优化
```

### 5.3 本地帧选择

| 实机能力 | 测距输入 | 数传输入 | 注意 |
|---|---|---|---|
| 已确认的 DR_MODE0 | Node_Frame3 优先，或 Frame2 | Node_Frame0 | Frame3 更紧凑 |
| 已确认的 DR_MODE1 | Node_Frame5 | Node_Frame6 | 原始 ID 是 32 位；不能用无效 `system_time` |
| LP 的机器人 TAG | Node_Frame2/3 | Node_Frame0 | 可读动态边；不能据此声称固定端也有固定边 |
| 无固定边本地输出 | 无 | 即使有数传也不能解决 | 硬件功能前提未满足 |

依据：[S1：§4、§7；S3：Protocol Rule]。

### 5.4 Node_Frame3 长度与字段检查

NLink V1.4 和该基线 C 解析器对应：

```text
Frame header = 0x55
Function     = 0x05
fixed header = 21 bytes
每条邻居记录 = 7 bytes
checksum     = 1 byte
总长度       = 22 + 7 × valid_node_count
```

邻居记录是：`role:uint8, id:uint8, dis:int24, fp_rssi:uint8, rx_rssi:uint8`。距离为毫米整数编码；解析后为米。RSSI 使用厂家编码 `code = −2 × RSSI`，还原为 `RSSI = −code / 2`。[S3；S8]

**来源差异提醒：** 用户手册 p.51–52 的 Node_Frame3 示例表中出现了额外 `reserved` 行；NLink V1.4 的 Node_Frame3 正式表及固定版本解析代码均为 7 字节邻居记录。开发时按实际帧长度、正式协议和固定源码验证，不要把该示例的额外保留行拼进解析器。

对 Node_Frame2 与 Node_Frame5 也要按各自格式校验，不能把 Frame3 的长度公式套过去。根据 NLink V1.4：Frame2 为 `120 + 13n`；Frame5 为 `25 + 10n`。数据中的节点数量与帧长度必须一致，校验通过才调用 unpack。[S3]

### 5.5 数传输出帧不等于数传输入帧

本方案发给 UWB 串口的是**用户应用负载**。Node_Frame0/6 是 UWB 向主机输出已收到数传数据时的外层协议，不能反过来把一个完整 Node_Frame0 当作发送指令。

同样，`NLink_LinkTrack_User_Frame1` 在手册中用于 DT_MODE0 主机侧输入，本设计不把它移植为 DR 透传发送格式。[S1：§4.2.1.5、§5.1；S3]

---

## 6. LIUO 固定边应用协议 v1

这一节是**本项目新增协议**，不修改 Nooploop 的空中协议或 NLink 协议。

### 6.1 为什么需要独立协议

必须知道“谁报告、测的是谁、来自哪一帧、是否重发、源帧多旧”。仅传 `id + distance` 无法正确处理掉电、重复发送、串口缓冲、分包和时间同步。

v1 采用：

```text
固定长度二进制头 + 固定长度边记录 + CRC16
    ↓ 标准 Base64，保留 padding，不折行
ASCII envelope = "~U:" + Base64(binary) + "\n"
```

设计理由：用户负载完全由可打印字符与换行组成，不包含公开配置协议中 `0x54 0x00`、`0x52 0x00` 等二进制前缀。它也便于示波器/串口日志人工检查，且兼容上游的 `std_msgs/String` 输入。

这只是减少与公开二进制配置格式重叠的设计，不等于厂商对所有固件输入解析行为的承诺。仍需通过 C5 验证；不能发送未知管理命令试探设备。接收端不能对 Base64 字符串当原始距离解码。

### 6.2 字节序、长度与上限

所有多字节二进制字段均为 **little-endian**。字段逐项编码，禁止直接发送未经检查的 C `struct` 内存。

```text
Header = 30 B
Edge   = 8 B
CRC    = 2 B
Binary length = 32 + 8n
Wire length   = 3 + 4 × ceil((32 + 8n)/3) + 1
1 ≤ n ≤ 5
```

| 本包边数 n | 二进制长度 | 实际写入 UWB 的用户负载长度 |
|---:|---:|---:|
| 1 | 40 B | 60 B |
| 2 | 48 B | 68 B |
| 3 | 56 B | 80 B |
| 4 | 64 B | 92 B |
| 5 | 72 B | 100 B |

最大包 100 B 对应 DR_MODE0 资料中的无缓存单次用户负载长度。本值不适用于所有模式；例如 LP 参数表中部分角色的无缓存长度是 20 B，必须重新计算缓存延迟和吞吐，不能照搬。[S2：§3.2.1–3.2.2]

### 6.3 包头：30 字节

| 偏移 | 字段 | 类型 | 含义 |
|---:|---|---|---|
| 0 | `magic` | 2 B | ASCII `LR`，即 `4C 52` |
| 2 | `version` | uint8 | 固定为 1 |
| 3 | `flags` | uint8 | bit0 系统时间可用；bit1 本地时间可用；bit2 MCU 保留年龄可用；其余为 0 |
| 4 | `reporter_id` | uint8 | 项目逻辑节点 ID；固定节点 1–254 |
| 5 | `edge_count` | uint8 | 本包边数 1–5 |
| 6 | `boot_id` | uint32 | 报告者数据会话；重启/检测到 UWB 时间重置时更新；0 保留 |
| 10 | `tx_cycle_seq` | uint32 | 5 Hz 广播轮次序号；同轮各分包相同 |
| 14 | `source_frame_seq` | uint32 | MCU 对已成功解析的本地测距帧编号；不是硬件 TWR 序号 |
| 18 | `source_local_ms` | uint32 | 该源测距帧的厂家 `local_time` |
| 22 | `source_system_ms` | uint32 | 该源测距帧的厂家 `system_time`；无效时填 0 并清 bit0 |
| 26 | `hold_age_ms` | uint16 | MCU 收齐源帧到本包准备写入 UART 的时间；未知为 65535 |
| 28 | `shard_index` | uint8 | 从 0 开始的分包号 |
| 29 | `shard_count` | uint8 | 本轮快照分包总数；所有分包均可独立使用 |

`hold_age_ms` 不包含源模块输出前的内部时延，也不包含之后的 UART/UWB 转发时延，更不等于完整测量年龄。

### 6.4 边记录：8 字节

| 相对偏移 | 字段 | 类型 | 含义 |
|---:|---|---|---|
| 0 | `peer_id` | uint8 | 另一个固定节点的项目逻辑 ID |
| 1 | `edge_flags` | uint8 | bit0 距离有效；bit1 RSSI 可用；其余为 0 |
| 2 | `distance_mm` | int32 | 以毫米为单位；由真实源帧转换，不能凭地图坐标计算后冒充测距 |
| 6 | `fp_code` | uint8 | 保留厂家 `−2 × fp_rssi` 编码 |
| 7 | `rx_code` | uint8 | 保留厂家 `−2 × rx_rssi` 编码 |

v1 只发送有效正距离，默认表示范围不超过源 `int24` 的正数上限。超出项目允许距离范围时丢弃并计数，不能饱和到最大值后继续使用。RSSI 不可用时清 bit1，并将两个 code 字段设为 0；接收端按标志决定是否有效，不能把无效 0 当成 0 dB。

本记录的两个端点是 `reporter_id` 与 `peer_id`。本版不允许一个节点继续转发别人报告的边，因此不需要 `origin_id` 和 `hop_count`。未来做多跳必须升级协议，不能悄悄改变这两个字段的语义。

### 6.5 CRC 与严格解包

CRC 采用 **CRC-16/CCITT-FALSE**，参数固定：

```text
width=16, poly=0x1021, init=0xFFFF,
refin=false, refout=false, xorout=0x0000
check("123456789") = 0x29B1
```

覆盖包头与所有记录，不含 CRC 本身；CRC 两字节以 little-endian 附在尾部。Nooploop 外层 NLink checksum 仍正常检查，应用 CRC 是第二层保护，不是身份认证。

解码顺序：检查 envelope 总长度 → 前后缀 → 严格 Base64 → magic/version → `32+8n` 长度关系 → CRC → flags/ID/shard → registry 身份匹配 → 观测去重 → 发布。

### 6.6 逻辑 ID 注册表

逻辑 N0、N1 不一定等于所有工作模式的原始 ID。保持配置表：

```text
(system_id, operating_mode, raw_role, raw_id) → logical_id + is_fixed
```

DR_MODE0 可以默认逻辑 ID 与原始 ID 相同。DR_MODE1 的原始 ID 是 32 位，要显式映射；不能截断低 8 位。LP 下 TAG 0 与 ANCHOR 0 也不是同一身份。

从 Node_Frame0/6 的 `nodes[k]` 获取实际发送者的原始 role/id，再用注册表映射。**外层消息顶层 id 是接收串口所连的 N0；内层 block 的 id 才是远端发送者。** 内层发送者映射后必须与应用包的 `reporter_id` 一致。[S3；S6]

### 6.7 源帧快照与分包

每轮冻结最新一帧测距输出，筛出固定邻居，按逻辑 ID 排序，连续分为最多 5 条一包。所有分包使用相同 `source_frame_seq`、源时间、`boot_id`、`tx_cycle_seq`。各包 `hold_age_ms` 按实际发送时刻单独计算。

不要把过去不同时间的邻居缓存拼成一帧，却只带一个统一源时间。若确需每边独立历史缓存，应定义后续协议版本，逐条增加源时间/序号；v1 不支持该行为。

某个分包丢失时，其他分包立即有效；机器人不得等待所有分包到齐再发布已到达的边。无有效固定邻居时不发送伪造的零距离边；诊断可在本地记录，独立心跳需另定消息类型。

---
## 7. 固定节点 MCU 程序设计

### 7.1 硬件连接与串口所有权

按设备实际型号确认供电范围。UART 为 3.3 V TTL 信号，TX/RX 交叉并共地；不能将 RS232/RS485 直接接入 TTL UART。数据手册明确提醒，不要同时连接其 USB 与 UART 数据接口，部分双 UART 接口在硬件上也是相连的。[S2：p.7–8]

```text
UWB TX  → MCU RX
UWB RX  ← MCU TX
UWB GND — MCU GND
供电按型号独立核实，不能将“TTL 是 3.3 V”理解成“模块供电必须是 3.3 V”。
```

正常运行时，由 MCU 独占固定节点的数据接口。维护时先停止 MCU 的串口收发，再连接 NAssistant；调试日志走另一个串口，不把 printf 输出到 UWB 的透传 UART。

### 7.2 MCU 最小任务划分

| 组件 | 输入 | 输出/职责 |
|---|---|---|
| `uart_rx` | UART 中断或 DMA 字节 | 环形缓冲，记录单调时钟接收时间 |
| `nlink_framer` | 字节流 | 按帧头/功能字/长度/校验识别完整帧 |
| `range_adapter` | 本地测距帧 | 标准化本节点邻居列表，替换最新帧快照 |
| `relay_scheduler` | 最新帧、单调时钟 | 每 200 ms 冻结一轮，安排分包发送 |
| `relay_codec` | 单个分包 | 100 B 以内的 `~U:` envelope |
| `uart_tx` | envelope | 有限队列、单次连续写出，不忙等到阻塞 RX |
| `health` | 计数、错误状态 | 本地统计、调试日志；不伪造测距 |

**设计规定：** STM32 可采用 DMA + 环形缓冲/空闲中断；ESP32 可采用其 UART 驱动事件队列。具体引脚、DMA 通道、RTOS 优先级和 UART 驱动 API 随选用板卡确定，本手册不虚构具体型号引脚。

### 7.3 复用 `nlink_unpack` 的正确方式

保留固定版本的 C 解析文件及对应头文件、工具文件、许可证。只编译实际使用的帧类型，例如 Frame3 与必要公共代码；MCU 不需要链接含 ROS/serial 的 `nutils` 库。

所读 Frame3 C 源码使用全局解析结果和静态状态，并为节点对象调用分配宏。实现时必须：[S8]

1. 串行调用解析器；不要从 ISR 与任务同时调用。
2. 在调用前校验长度、节点数和记录边界；不能仅依赖加和校验。
3. 成功后立即复制所需字段，不长期保存内部结果指针。
4. 对 MCU 采用受控分配或固定池；内存申请失败必须返回错误，不能继续解引用。
5. 不直接把该解析器当成多串口、多线程重入库。

### 7.4 只转发本地固定边

处理一帧时：确认帧输出者就是本节点 → 遍历邻居 → 查注册表 → 保留 `is_fixed=true` 的邻居 → 丢弃自环、未知 ID、无效距离 → 保存为最新快照。

固定节点**不转发 N0–Ni 动态边**，因为 N0 已直接获取自己的测距；也不把收到的 Node_Frame0/6 负载再次写回 UWB。后者会产生广播回环和指数级重复。

默认发送源帧的原始距离与 RSSI，不在驱动/MCU 中做滑动均值、NLOS 修正或“静态地图一致性”筛除。算法性处理交给 LIUO；源值与异常计数保留以便分析。

### 7.5 源帧与发送轮次的两个序号

- `source_frame_seq`：每收到并成功解析一个本地测距帧递增一次。
- `tx_cycle_seq`：每开始一个广播轮次递增一次。

若串口已停更，发送轮次仍可能继续，但不能给旧帧分配新的 `source_frame_seq`。本版优先在源帧超过 `max_source_hold_ms` 后停止发送测距负载，而不是无限广播旧观测。

新的输出帧里距离值恰好相同，不是丢弃理由。固定节点之间本来就可能长期输出相同的毫米整数值。

### 7.6 MCU 主循环伪代码

以下是逻辑合同，不是某一芯片可直接编译的 HAL 工程：

```text
on_uart_bytes(bytes, mono_now):
    feed_nlink_stream(bytes)
    while one_complete_frame_available:
        verify_length_checksum_and_record_bounds()
        if frame_is_local_range:
            detect_device_reset_or_clock_discontinuity()
            if reset_detected:
                rotate_boot_id()
                clear_latest_snapshot_and_tx_queue()
            source_frame_seq += 1
            latest = copy_valid_fixed_neighbors_from_this_frame()
            latest.source_frame_seq = source_frame_seq
            latest.source_local_ms = frame.local_time
            latest.source_system_ms = validated_system_time_or_zero
            latest.rx_mono_ms = mono_now
        else:
            count_or_discard_without_relaying()

on_new_broadcast_cycle(mono_now):
    tx_cycle_seq += 1
    discard_unsent_previous_cycle_packets()
    if latest is absent or too_old(latest):
        increment(source_stale_count)
        return
    snapshot = immutable_copy(latest)
    sort_by_logical_peer_id(snapshot)
    shards = split_into_groups_of_at_most_5(snapshot)
    schedule_all_shards_within_this_cycle()

on_shard_due(shard, mono_now):
    if source_has_expired or uart_has_no_bounded_capacity:
        count_drop_and_do_not_block_rx()
        return
    hold_age_ms = mono_now - snapshot.rx_mono_ms
    packet = encode_header_edges_crc_and_base64(shard)
    write_one_contiguous_envelope(packet)
```

### 7.7 资源和故障策略

队列保留**当前一轮**，积压时丢弃过时轮次，不追赶发送历史队列。每次实际写入必须记录成功字节数；部分写入不能记录成“成功发送一包”。部分写入时需要接收端 envelope 重同步，不得紧接着把残余字节假装成完整新包。

正常 range packet 不做逐包 ACK，不无限重试旧测距。下一轮的新帧是主要恢复机制。需要严格保证交付的管理类信息应与高频 range 数据分离，不在本版隐式加入可靠广播协议。

---

## 8. 调度、容量与带宽预算

### 8.1 两层调度不能混为一谈

**厂家固件层：** 实际测距、无线发送、内部缓冲和调度。  
**MCU 应用层：** 何时向串口提交用户数据、提交多少数据、是否已经过时。

本方案实现第二层，不声称实现无碰撞空中 TDMA。TWR 的测距时钟特性，也不等于无需处理信道负载。不能通过 MCU 定时器保证 UWB 一定在指定毫秒发射。

### 8.2 推荐的第一版发送策略

默认 `broadcast_period_ms=200`。每个节点用不同的初始相位；同节点的多个分包在该周期内均匀错开。

```text
phase_i = stable_hash(logical_id, boot_id) mod 200 ms
shard_offset_s ≈ s × 200 / shard_count
```

可增加有界抖动，例如 ±2 ms，用来避免应用层长期同相提交；这是待测试参数，不是碰撞消除保证。周期使用 MCU 单调时钟推进。落后时跳过过期发送，不用 `while overdue: send()` 形成补发风暴。

如果已经验证共享 `system_time` 稳定，可使用它作粗相位参考，并用 MCU 单调时钟插值到下一次提交时机。但不要求新增全网高精度同步系统，也不把每 20 ms 才输出一次的帧时间直接解释为亚毫秒精确授时。

### 8.3 同节点发送间隔

参考配置的初值：`min_tx_gap_ms=25`，每个包最多 100 B。它适用于后述小规模原型的预算，并非厂商通用要求。

本轮需要 `k` 个包时，必须同时满足：

```text
k × actual_wire_packet_bytes / 0.2 s < 可用每节点数传预算
可安排 k 个包且满足 min_tx_gap_ms 和源帧年龄限制
N0 接收所有可见发送者的数据后，串口与解析线程不积压
```

若无法满足，不能只发前几条边而声称全部边 5 Hz。应报告不满足的配置，并按 §8.7 的顺序处理。

### 8.4 负载公式

固定节点 i 有 `d_i` 个有效固定邻居，分包数：

```text
k_i = ceil(d_i / 5)
node_packet_rate_i = 5 × k_i packets/s
node_payload_rate_i = 5 × sum(WireLength(shard_size)) bytes/s
```

重复几何边由不同节点报告时，在通信预算中必须各算一次。

**注意三种开销：** 本协议的 Base64/envelope 开销；UWB 透明传输的内部无线开销；接收端 NLink 外层和 UART 开销。下面只对已知应用和串口部分做估算，不假设公开资料已经给出了真实空中占用时长。

### 8.5 三组预算示例

假设所有固定节点相互可见，每个固定节点排除 N0，只上报其他固定节点；N0 能收到所有广播；固定边广播为 5 Hz。

| 系统规模 | 每固定节点固定邻居数 | 每轮分包 | 每节点用户负载 | N0 收到的用户负载合计 |
|---|---:|---|---:|---:|
| N0 + 4 固定节点 | 3 | 1×3 条，80 B | 400 B/s | 1,600 B/s |
| N0 + 9 固定节点 | 8 | 5+3 条，180 B | 900 B/s | 8,100 B/s |
| N0 + 19 固定节点 | 18 | 5+5+5+3 条，380 B | 1,900 B/s | 36,100 B/s |

在资料所列 DR_MODE0 容量 20、40,000 bit/s 即 5,000 B/s 的**每节点**数传名义预算下，第三行单节点用户负载约为其 38%。这只能说明应用字节数没有直接超出该数值，**不能证明全网射频性能达标**。[S2：p.26–27]

### 8.6 机器人串口预算示例

按 921600 baud、8N1，理论每方向字节传输上限：

```text
921600 / 10 = 92,160 bytes/s
```

以 N0 + 19 固定节点、N0 的 Frame3 输出 50 Hz 为例：

```text
本地测距帧：  (22 + 7×19) × 50 = 7,750 B/s
远端应用负载：                  36,100 B/s
外层预算：19×4×5 个应用包/s，每个暂按 16 B = 6,080 B/s
合计估算：                      49,930 B/s
理论 RX 方向占用率：             54.2%
```

16 B 是“每个应用包独立装入一个单 block Node_Frame0”时的示例开销；实际合并/拆分方式不同，应以串口日志测得的总字节率替换。不要将这个模型当成厂家对 framing 的承诺。[S3：Node_Frame0]

**设计规定：** 初版对实测持续 RX 利用率设置 70% 告警线，并同时监测高峰队列长度和解析延迟。即使平均低于 70%，突发缓冲、USB 延迟或慢回调仍可能丢数据。需要提高波特率时，UWB 配置、USB-UART 芯片和主机均须支持并实测。[S2：§3.3]

### 8.7 不达标时的处理顺序

正常运行目标始终是 N0 ≥20 Hz 与固定边 5 Hz。排障顺序：

1. 去掉反向转发回环、调试文本、重复的无用消息和人为累计队列。
2. 检查 NLink Frame3/紧凑协议、UART 利用率、读线程及消息复制开销。
3. 调整应用发送相位、分包间隔和突发长度；只在有证据时提高 UART 波特率。
4. 减少尚未被需求批准的额外功能，不自动删掉要求采集的固定边。
5. 仍不达标时，记录实际受限指标；启用经批准的降级配置，例如 5 → 4 → 2.5 Hz。

`allow_rate_degrade` 默认 **false**。临时降频必须显式标为 DEGRADED；不是“5 Hz 方案验收通过”。不能默认退回先前讨论的 1 Hz，也不能为了维持固定边频率将 N0 动态边压到 20 Hz 以下。

---

## 9. `nlink_parser` 的最小修改方案

### 9.1 保持原消息与原话题

不修改已有 `LinktrackNodeframe*.msg` 字段，避免 ROS 1 消息 MD5 改变、旧订阅者和历史 bag 不兼容。

新增带接收上下文的并行话题：

```text
nlink_linktrack_nodeframe3_stamped
nlink_linktrack_nodeframe0_stamped
```

实际使用 Frame2/5/6 时再新增对应包装。保留原 `nlink_linktrack_nodeframe3` 等话题，便于和原驱动逐字段对照。

### 9.2 新增接收上下文消息

`nlink_parser/msg/NLinkRxContext.msg`（拟新增，ROS 1）：

```text
# header.stamp: 本机读取到这一批串口字节的 ROS 时间，不是测距时间
std_msgs/Header header
uint64 host_monotonic_ns
uint64 port_session_id
uint64 rx_frame_seq
uint8 function_mark
```

`nlink_parser/msg/LinktrackNodeframe3Stamped.msg`：

```text
NLinkRxContext context
LinktrackNodeframe3 frame
```

`nlink_parser/msg/LinktrackNodeframe0Stamped.msg`：

```text
NLinkRxContext context
LinktrackNodeframe0 frame
```

其他帧的包装保持同样结构。`port_session_id` 在进程重启或重连后改变；它不能替代远端 MCU 的 `boot_id`。

### 9.3 `main.cpp`：在串口入口取时间

原实现读取字符串后调用 `protocol_extraction.AddNewData(str_received)`。[S5]

修改原则：

```text
一次 read 返回字节 → 立即记录 ROS 时间与 steady/monotonic 时间
                  → 设置本轮 rx_context
                  → AddNewData(...)
                  → 每个成功解析的完整帧递增 rx_frame_seq
```

同一 read 批次中的多个完整帧允许共享接收时间，但必须有不同 `rx_frame_seq`。跨多个 read 才完整的帧，使用**完成该帧的 read** 上下文。记录该时间是“用户态批量接收时间”，不要称它为硬件 UART 到达时间。

串口入口伪代码：

```cpp
// 设计接口示例；RxStamp/SetRxBatchContext 为本项目待新增函数。
const auto available = serial.available();
if (available > 0) {
  serial.read(buffer, std::min(available, max_read_chunk));
  const RxStamp stamp = CaptureRosAndMonotonicTime();
  init.SetRxBatchContext(stamp, port_session_id);
  protocol_extraction.AddNewData(buffer);
}
```

RX 周期与过期判断使用单调/墙钟机制，不能依赖可暂停或跳变的仿真 ROS 时间。ROS 时间仅用于对接传感器和 bag。断线时清除未完成帧状态，限速重连，并发布诊断；不对缺失数据补零。

### 9.4 `init.cpp/.h`：在原回调旁新增 stamped 发布

保留原有 unpack → msg 字段复制 → publish。随后用**同一个已解出的消息**生成包装输出：

```cpp
// 设计接口示例；MakeRxContext 为本项目新增。
original_publisher.publish(g_msg_nodeframe3);

nlink_parser::LinktrackNodeframe3Stamped wrapped;
wrapped.context = MakeRxContext(0x05);
wrapped.frame = g_msg_nodeframe3;
stamped_publisher.publish(wrapped);
```

Frame0 包装同理，其 context 属于 N0 本地的接收过程，不是远端测量时间。

每个物理帧只创建一次 context；原消息、包装消息和日志使用同一 `rx_frame_seq`。不要在两个 publish 前各递增一次。`MakeRxContext()` 的调用顺序应在实际补丁中统一封装，避免示例代码被机械复制后出现双计数。

### 9.5 TX 兼容接口

机器人 N0 正常工作主要接收，本版不需要它请求每个固定节点才有数据。MCU 在固定端主动广播，因此 N0 的 TX 可以默认关闭。

保留上游 `std_msgs/String` 数传接口作为兼容项，但新增显式开关：

```text
enable_legacy_tx: false
```

开启后才允许该话题写串口，并检查长度、发送间隔和队列上限。不要一边让原回调直接写串口，一边让另一个 ROS 节点或线程写同一端口。需要原行为的测试场景显式打开开关并记录配置。

由于本协议 wire envelope 是 ASCII，兼容 `std_msgs/String` 不涉及任意二进制字符串的跨语言编码问题。未来增加二进制发送接口时，应采用清晰的 byte array API；不要用 `strlen()` 截断含零字节的负载。

### 9.6 不要直接改成多线程 spinner

上游 `init.cpp` 有全局消息对象、全局 `serial_` 指针；C unpack 也有共享状态。[S6、S8]

第一版保持每进程单串口、单一解析线程。多个 UWB 串口使用不同进程和命名空间。不能仅把 `ros::spinOnce()` 换成多线程 `AsyncSpinner` 就声称提升性能；先完成状态隔离，才可以并行化。

机器人侧 `liuo_uwb_bridge` 是独立进程，不直接调用同一个解析器实例，不打开 N0 串口。这使应用解码与原串口接收分开，减少耦合。

### 9.7 长度验证必须放在 unpack 之前

接收缓冲必须有硬上限；`Frame Length`、`valid_node_count` 和内部 `data_length` 不能被盲目信任。

以 Frame3 为例，先要求 `length == 22 + 7*n`，n 不超过配置与数组容量，才调用原 unpack。Frame0/6 则逐 block 验证“剩余字节 >= block header + data_length”，检查累加溢出和尾部 checksum 位置。

初版建议可配置 `max_nlink_rx_frame_bytes=8192`、输入环形缓冲不小于其两倍；这只是工程起点，不是厂家协议上限。记录实机最大帧长后调整，不能因默认值太小而静默截断有效数传帧。

遇到坏长度时重新搜索下一帧头，不能永久等待恶意/错误的超长长度。不要为了恢复同步而每次任意清空全部已收到的有效后续帧。

### 9.8 构建文件修改

`CMakeLists.txt`：把 `NLinkRxContext.msg` 与实际使用的 stamped 消息加入 `add_message_files()`；它们只依赖同包消息和已有 `std_msgs`。保留 `linktrack` target、原解析库和已有测试。[S7]

`liuo_uwb_msgs`：依赖 `message_generation`、`message_runtime`、`std_msgs`。  
`liuo_uwb_bridge`：依赖 `roscpp`、`nlink_parser`、`liuo_uwb_msgs`；采用标准诊断消息时再加入 `diagnostic_msgs`。  
`liuo_uwb_protocol`：独立 C/C++ 库，无 ROS、无串口依赖，可编译到 MCU。

不要建立 `nlink_parser → liuo_uwb_msgs → nlink_parser` 的循环依赖。

---

## 10. 机器人聚合与统一 ROS 接口

### 10.1 一个输入话题，不是一个“同时采样”假设

主话题建议命名：

```text
/liuo/uwb/observations
类型：liuo_uwb_msgs/UwbObservationBatch
```

本地新测距帧到达时立即发一个 batch；远端固定边包到达并解码后，也立即发同一种 batch。两类消息在同一话题中按到达事件发布，LIUO 根据类型与源时间处理。

因此，主话题总频率可能高于 20/50 Hz；必须单独统计 `trigger_type=LOCAL_RANGE` 的动态流。绝不能设置一个 5 Hz 聚合定时器把 N0 动态数据也降成 5 Hz。

如果后续确实需要混合 batch，可将“本地帧 + 自上次发布以来新收到的远端观测”放在一起，但不能阻塞本地帧、不能塞入已经发布的历史边。第一版优先事件驱动，逻辑更容易验证。

### 10.2 `UwbObservation.msg`（拟新增，ROS 1）

```text
uint8 ROBOT_ANCHOR=0
uint8 ANCHOR_ANCHOR=1
uint8 TIME_UNKNOWN=0
uint8 TIME_FRAME_LOCAL_MAPPED=1
uint8 TIME_FRAME_SYSTEM_MAPPED=2
uint8 TIME_RECEIVE_ONLY=3

uint8 kind
uint8 reporter_id
uint8 node_a
uint8 node_b

# 数据来源与观测身份
uint64 source_session_id
uint32 source_frame_seq
bool source_frame_seq_valid
uint32 tx_cycle_seq
uint8 shard_index
uint8 shard_count

# 原始设备时间与映射状态
uint32 source_local_ms
uint32 source_system_ms
bool source_local_time_valid
bool source_system_time_valid
uint8 time_basis
time source_frame_stamp
time received_stamp
uint16 source_hold_age_ms
bool source_hold_age_valid
float32 time_uncertainty_s

# 测距与质量：保持源值；未知质量不可填成高置信度
float32 distance_m
float32 fp_rssi
float32 rx_rssi
bool rssi_valid
bool pair_measurement_freshness_verified
```

说明：

`source_session_id` 对远端来自 `boot_id` 加接收端数据集/源身份域；对 N0 来自本地设备会话。实现时可在会话注册表映射为 64 位编号，不声称一个随机数必然全局唯一。

`source_frame_seq` 对 N0 是驱动分配的本地成功测距帧序号，对远端是 MCU 分配的序号。它们都不是厂家没有提供的每边 TWR 序号。首次没有可靠序号的遗留数据，清 `source_frame_seq_valid`。

`time_uncertainty_s` 未知时用 NaN，不以 0 表示“不知道”。`source_frame_stamp` 无有效映射时填 0，并由 `time_basis` 明确说明；`received_stamp` 始终是本机接收上下文时间。`source_hold_age_ms=65535` 表示未知，不能理解为确实 65.535 秒。

`pair_measurement_freshness_verified` 默认 false。只有独立证据确认该模式每个有效记录对应新的测距且该保证在本帧有效时，才置 true。不能因为收到新 ROS 消息就置 true。

本消息不强制提供 `variance`，因为原始 NLink 输出并没有逐边可信测距方差。LIUO 可使用标定噪声模型；不要由驱动虚构一个标准差。

### 10.3 `UwbObservationBatch.msg`（拟新增，ROS 1）

```text
uint8 LOCAL_RANGE=0
uint8 REMOTE_EDGE_REPORT=1

# stamp 为本 batch 组装/发布时刻；不是所有观测的统一采样时刻
std_msgs/Header header
uint64 driver_session_id
uint64 batch_seq
uint8 receiver_id
uint8 trigger_type
UwbObservation[] observations
```

距离是端点间标量，本身没有必须指定的世界坐标系。`header.frame_id` 可留空；机器人 UWB 杆臂、传感器安装坐标系由 LIUO 的外参配置管理，不在这里伪造 `map` 下坐标。

### 10.4 桥接节点核心流程

```text
on_local_frame_stamped(frame):
    validate_receiver_identity(N0)
    map_frame_time_or_mark_unknown()
    for each valid fixed neighbor:
        create ROBOT_ANCHOR observation
        attach source frame identity and receive context
    publish_immediately()

on_data_frame_stamped(frame):
    for each sender_block:
        sender = registry.lookup(block.role, block.id)
        feed_this_sender_stream(block.data)
        for each complete ~U: envelope:
            decode_and_check_length_crc_fields()
            require(packet.reporter_id == sender.logical_id)
            reject_non_fixed_reporters_and_non_fixed_peers()
            deduplicate_transport_and_source_frame_observations()
            publish_arrived_shard_immediately()
```

### 10.5 为什么按发送者分别处理 `data[]`

一个 Node_Frame0/6 可以包含多个发送者 block，各 block 含自己的 `data_length`。[S3；S6]

不能将整个 ROS 消息的所有 `data[]` 拼成一个无来源的字节流。若实机可能将透传内容拆分或合并，则为每个已注册发送者维护有界 envelope 解析状态，按 `~U:` 与换行恢复边界。最多保留一个未完整行；超出 100 B 仍无合法结尾就重新同步并计数。

“单次 `serial.write()`”不等于“接收端一定对应一个独立完整应用包”；必须用 C5 测试真实边界行为。若缺片导致 Base64 或 CRC 失败，丢弃该包，不从残片猜测距离。

### 10.6 诊断话题

可另发 `/liuo/uwb/diagnostics`，例如 1 Hz；这是健康统计，不是把测距或固定边降到 1 Hz。

诊断至少包含：模式配置、已确认/未确认能力、每边报告率、每发送者接收轮次、丢包/缺分包估计、源帧停更、CRC 错误、未知节点、UART 字节率、队列峰值、接收处理延迟、时钟映射状态、DEGRADED 状态。

---

## 11. 时间处理：保留原始语义，不制造硬件时间戳

### 11.1 三个时间域

| 时间域 | 例子 | 用途 |
|---|---|---|
| 模块时间 | `local_time` / 经验证的 `system_time`，单位 ms | 关联源帧、拟合到机器人时钟 |
| MCU 单调时间 | UART 收齐、计划发送、实际发送时刻 | 调度、`hold_age_ms`、检测停更 |
| 机器人时间 | ROS 时间 + host monotonic | LIUO 接入、bag、驱动延迟与超时 |

不能直接将一个 Anchor MCU 的启动时间当作 ROS 时间，也不能从 N0 的接收时间减去一个自定义常数后声称得到了精确 TWR 时刻。

### 11.2 厂家时间戳的已知边界

用户手册将 `local_time` 与 `system_time` 定义为**输出该协议帧时**的时间。LP 以 A0 为系统时间基准，DR_MODE 说明以最早上电 NODE 为基准；同节另明确 DR_MODE1 的系统时间无效。[S1：p.42–43]

因此：

- 没有每条边硬件时间戳时，所有记录只能继承源帧时间，标为帧级近似。
- `system_time` 可以辅助对齐，但不能仅凭字段存在就认为有效。
- 长隧道中网络分裂、重新加入、基准节点掉电后的时钟连续性需要实测；不能默认所有空间分区始终共享同一时间域。
- 不读取协议 `reserved` 字节并猜测它们是更精确的测量时间。

### 11.3 N0 的优先处理

N0 动态测距应直接从本地源帧映射到机器人时间，不经 MCU 转发路径。

参考模型：

```text
t_ros_frame ≈ alpha × unwrap(device_time_ms) × 1e-3 + beta
```

拟合输入为设备帧时间与串口接收上下文；接收侧存在单向延迟，因此不能只凭该拟合完全辨识绝对偏移。可使用稳健拟合/低延迟样本，并通过独立同步或动态标定估计偏移；结果仍需保留不确定度。

前期允许 `TIME_RECEIVE_ONLY` 作为明确标记的降级时间语义，用于链路调试；不能在高动态融合验收中把它当成“已同步”。不要仅取第一帧的 PC 时间偏移，然后永久假定不漂移、不重启。

### 11.4 远端固定边

远端携带原始源帧时间和 MCU 保留年龄。共享 UWB 系统时间已验证有效时，可沿用 N0 建立的系统时间映射；否则保留原始时钟字段和接收时刻，将跨节点映射标为未知。

固定边的两端静止，使得短时转发延迟通常不改变它约束的几何对象，但**不能因此伪造时间**。LIUO 可以根据固定性采用不同的时间处理，而驱动必须保留真实来源。

`hold_age_ms` 只覆盖 MCU 缓存时间，不能作为“端到端总时延”指标。报告“源帧年龄”也不能写成“精确测距年龄”。

### 11.5 回绕、重启和离线回放

32 位毫秒计数约 49.71 天回绕。实现时将正常回绕与重启区分：小的模运算正向增量可视为回绕；不合理跳变触发会话/时钟映射重建。`system_time` 与 `local_time` 应分别跟踪，不能用一个 unwrap 状态处理所有节点。

MCU 重启或识别出 UWB 重启后更新 `boot_id`、清理待发快照；机器人重连后更新 `port_session_id`、清除半帧。为旧会话保留短期 tombstone，避免延迟到达的旧会话包重新激活。

录 bag 时同时保存原始帧、stamped 帧、规范化观测与配置。回放时使用已记录的时间语义，不在桥接回调里用新的 `ros::Time::now()` 覆盖源时间。

---

## 12. 去重规则：通信去重不等于测距图融合

### 12.1 三类重复必须区别处理

| 重复类型 | 例子 | 驱动动作 | LIUO 动作 |
|---|---|---|---|
| 同一个传输包重复到达 | 同 reporter/boot/cycle/shard | 丢弃重复包，计数 | 不可产生新观测 |
| 同一源帧被下一轮再次广播 | tx_cycle_seq 不同，但 source_frame_seq 相同 | 按源帧+peer 去重，不伪造新测量 | 不重复累加信息 |
| 两端分别报告同一几何边 | N1 报 N1–N2，N2 报 N2–N1 | 两条原始报告都保留来源 | 在同一无向几何边下处理相关性和质量 |

### 12.2 推荐键值

```text
PacketKey = (network_instance, reporter_id, boot_id, tx_cycle_seq, shard_index)
ObservationKey = (network_instance, reporter_id, boot_id, source_frame_seq, peer_id)
GeometricEdgeKey = (min(node_a,node_b), max(node_a,node_b))
```

前两个键用于驱动传输去重；第三个键交由 LIUO 做地图/因子管理。

不要仅用 `(min_id,max_id)` 删除来自另一端的报告。机器人可能只收到较大 ID 节点的广播，若强制只留较小 ID 所有者，就会在局部覆盖情况下丢掉本来可获得的边。

### 12.3 双向报告不一定独立

公开资料没有为这类报告提供可验证的共同 TWR transaction ID。因此 `d_ij` 与 `d_ji` 可能源自同一交互或相关测量，不能默认是两个独立高斯约束。

驱动保留 reporter、源帧和方向。LIUO 选择一个来源、建立相关观测模型，或形成一个保守静态边估计；不能每收到一次广播就无限添加同一个高置信度因子。

### 12.4 驱动去重缓存不是长期地图

去重缓存建议按最近 10 s 的观测身份做有界窗口，限制最大条目数并清理过期项。它只记录“这个数据是否已交付”，不估计 Anchor 坐标、不保存长期距离均值、不判断边是否应长期用于 SLAM。

静态地图需要的历史、标定方差、NLOS 分类、边版本和设备移动检测都属于 LIUO。驱动不得因为距离长期没变化就停止 5 Hz 广播。

---
## 13. 性能监测、拥塞和安全降级

### 13.1 监测窗口与统计定义

使用单调时钟计算速率和队列等待时间。默认短窗口 10 s、长窗口 60 s；启动、重连、节点刚进入覆盖范围的过渡期单独标记。

| 指标 | 推荐统计方式 | 不允许的替代 |
|---|---|---|
| N0 本地帧率 | 唯一成功解析本地测距帧数/时间 | 统一话题的总发布率 |
| N0 每边报告率 | 新源帧中含该 peer 的有效记录数/时间 | 数组长度×话题频率的无条件估算 |
| Anchor 广播轮次率 | MCU 实际完成的 `tx_cycle_seq`，并记录缺分包 | 定时器 callback 被执行次数 |
| 固定边到达率 | 按 ObservationKey 去重后的新记录数/时间 | 重发同一源帧的次数 |
| 包交付率 | 对照发送日志中的 cycle/shard 与接收日志 | 源帧序号跳了 10 就认为丢了 9 个广播包 |
| 源帧年龄 | 有效时钟映射下的接收时刻−源帧时刻 | 把 `hold_age_ms` 当完整端到端延迟 |
| 动态测距影响 | 广播关闭/开启的配对测试 | 只看广播流量是否低于名义带宽 |

因为 50 Hz 源帧在 5 Hz 广播中被抽样，`source_frame_seq` 自然会跳号。广播丢包应根据 `tx_cycle_seq + shard_index` 分析，同时区分 MCU 主动丢弃的过时轮次。

### 13.2 设计阈值的建议起点

以下为待工程验证的项目阈值，不是厂家性能保证：

```yaml
robot_min_valid_edge_hz: 20.0
anchor_broadcast_hz: 5.0
max_source_hold_ms: 250
max_transport_queue_ms: 50
rx_utilization_warn: 0.70
statistics_window_s: 10.0
diagnostics_period_s: 1.0
allow_rate_degrade: false
```

`max_source_hold_ms` 限制 MCU 内部积压；`max_transport_queue_ms` 限制某个分包**到期后**仍等待写出的额外延迟，不包括它在 200 ms 周期内正常安排的分包偏移，也不涵盖无法观测的固件内部缓存。检测到内部延迟异常但无法精确拆分时，报告“未知内部延迟/总观测延迟异常”，不要虚构各阶段数值。

### 13.3 拥塞优先级

RX 接收与解析优先于固定端 TX 排队；机器人本地动态流不等待固定边。通信过载时先清过时 TX 包和诊断输出，不让它们堆积到数秒后再传。

不得以“换成更大的发送队列”作为实时性问题的唯一修复。更大队列可能改善不丢包的表象，却让到达的距离更旧。

### 13.4 节点消失不等于距离为零

邻居从当前测距帧中消失，表示本帧没有该条有效报告。不要在数据消息中补一条 `distance=0`。驱动诊断可报告该边最后一次收到的时间；是否保留它在长期 Anchor 图中的约束，由 LIUO 决定。[S1：§7.1]

### 13.5 为什么第一版不在通信层强制“边唯一所有者”

允许 N1 与 N2 分别报告 N1–N2，可以提高 N0 只覆盖一端时的可获取性。其代价已经计入预算。先在 LIUO 处理同一几何边的多个来源，暂不引入 leader 选举、几何边所有权转移或多跳。

若实测需要减少这部分流量，必须通过显式新需求重新设计，并验证局部可见条件下的信息损失；不能偷偷按 `reporter_id < peer_id` 丢掉一半边。

---

## 14. 与 LIUO 的边界合同

### 14.1 驱动交付什么

交付原始距离、两端 ID、报告者、源帧身份、原始时间、可用时间映射、接收时间、RSSI 与明确的有效性标记。传输 CRC、身份不一致、结构错误或完全重复的观测不会进入正常数据通路，但有诊断记录。

驱动只做结构有效性和配置允许范围检查，不假设 RSSI 某个阈值就能确定 LOS/NLOS，也不把固定边的短期离散度直接声明为准确度。

### 14.2 LIUO 负责什么

长期保存 Anchor–Anchor 距离图；决定双向报告如何融合；应用标定噪声和 NLOS 模型；估计未知 Anchor 坐标；处理固定节点移动/替换；管理静态因子的版本与信息权重；融合机器人动态测距、LiDAR 和 IMU。

以下属于数学建模示意，不是驱动功能：

```text
固定边： r_ij = ||a_i − a_j|| − d_ij
动态边： r_0i(t) = ||p(t) + R(t) l_UWB − a_i|| − d_0i(t)
```

其中驱动只提供 `d`、端点和观测上下文。Anchor 位置、杆臂、姿态、权重与优化策略由 LIUO 管理。

### 14.3 5 Hz 连续发送与长期缓存并不矛盾

通信层持续交付新报告，LIUO 决定是否用新数据更新静态估计。新广播不必对应“新增一个独立静态因子”，也不必每 200 ms 重建整个图。

相同源帧重发已被驱动去重；不同源帧的数值相同仍属于新的**报告**，但其统计独立性未被通信协议保证。LIUO 不能仅凭报文数量使一个静态约束的信息无限增大。

### 14.4 接入时的三条硬约束

1. 使用每条观测的时间语义，不以 batch.header.stamp 代替所有源时间。
2. 依赖观测类型分开处理动态边和固定边，不把远端固定边误挂到当前机器人姿态。
3. 地图长期缓存状态不回灌成新的传感器测量消息；回放也保持该约束。

---

## 15. 配置、启动与修改任务清单

### 15.1 配置示例

以下 YAML 是**拟新增桥接/MCU 配置规范**。它不是厂家 NAssistant 参数文件；`validated_capabilities` 必须由实际测试填写，不能因为加载了配置就当成硬件验证通过。

```yaml
network_instance: tunnel_test_01
robot_logical_id: 0

validated_capabilities:
  fixed_to_fixed_local_range: false     # C1 通过后改 true
  simultaneous_ranging_and_data: false # C2 通过后改 true
  hundred_byte_payload: false          # C5 通过后改 true
  shared_system_clock: false
  pair_freshness_guaranteed: false

source_protocol:
  range_frame: nodeframe3              # 必须和实机输出一致
  data_frame: nodeframe0
  operating_mode: unconfirmed          # 实测填入，不由驱动自动设置
  capacity: 0                         # 0=未确认；不能拿来做性能承诺

serial:
  baud_rate: 921600
  max_nlink_rx_frame_bytes: 8192
  rx_ring_bytes: 16384
  enable_legacy_tx: false

relay:
  broadcast_hz: 5.0
  max_edges_per_packet: 5
  max_wire_payload_bytes: 100
  max_source_hold_ms: 250
  min_tx_gap_ms: 25
  initial_phase_mode: id_boot_hash
  jitter_ms: 2
  forward_received_reports: false
  allow_rate_degrade: false

bridge:
  publish_mode: event_driven
  dedup_window_s: 10.0
  dedup_max_entries: 10000
  unknown_node_policy: reject_and_count
  partial_shard_policy: publish_arrived_observations

requirements:
  robot_min_valid_edge_hz: 20.0
  static_edge_tx_target_hz: 5.0
  static_edge_rx_target_hz: 5.0

# 示例仅表示 DR_MODE0 的原始 ID 与逻辑 ID 恰好相同。
# 实机为其他模式时重新填写 raw_role/raw_id，不得直接使用。
nodes:
  - {logical_id: 0, raw_role: 0, raw_id: 0, is_fixed: false}
  - {logical_id: 1, raw_role: 0, raw_id: 1, is_fixed: true}
  - {logical_id: 2, raw_role: 0, raw_id: 2, is_fixed: true}
  - {logical_id: 3, raw_role: 0, raw_id: 3, is_fixed: true}
```

`false/unconfirmed` 的作用是暴露未验证状态，不是让整个软件不可启动。允许仅采集原始数据或明确的 simulator 模式；正式固定边功能验收不得绕过这些状态。

### 15.2 固定上游基线

在新的工作空间内执行；不要覆盖现有 LIUO 工作区：

```bash
mkdir -p ~/liuo_uwb_ws/src
cd ~/liuo_uwb_ws/src
git clone --recursive https://github.com/nooploop-dev/nlink_parser.git
cd nlink_parser
git checkout -b liuo/uwb-relay-v1 77a3238a5b73e7545a032570606492e2b7642b78
git submodule update --init --recursive

git rev-parse HEAD
git submodule status --recursive
```

新建 `liuo_uwb_msgs`、`liuo_uwb_bridge` 与协议核心后，在已有可用 ROS 1 环境中编译：

```bash
cd ~/liuo_uwb_ws
catkin_make
source devel/setup.bash
```

这组命令没有安装 ROS、没有刷 UWB 固件，也没有修改模块工作模式。具体操作系统与 ROS 发行版沿用用户已验证的环境；不要把仓库基线固定理解为操作系统版本也被自动兼容。

### 15.3 项目 launch 示例

这是新增 `liuo_uwb_bridge/launch/robot.launch` 的结构示例。只有完成本手册新增节点、消息和参数后才能运行。

```xml
<launch>
  <arg name="port" default="/dev/ttyUSB0"/>
  <arg name="baud" default="921600"/>
  <arg name="config" default="$(find liuo_uwb_bridge)/config/uwb.yaml"/>

  <group ns="uwb/n0">
    <node pkg="nlink_parser" type="linktrack" name="linktrack" output="screen">
      <param name="port_name" value="$(arg port)"/>
      <param name="baud_rate" value="$(arg baud)"/>
      <param name="publish_stamped_frames" value="true"/>
      <param name="enable_legacy_tx" value="false"/>
    </node>
  </group>

  <node pkg="liuo_uwb_bridge" type="range_bridge" name="liuo_uwb_bridge" output="screen">
    <rosparam command="load" file="$(arg config)"/>
    <remap from="local_range_stamped"
           to="/uwb/n0/nlink_linktrack_nodeframe3_stamped"/>
    <remap from="remote_data_stamped"
           to="/uwb/n0/nlink_linktrack_nodeframe0_stamped"/>
    <remap from="observations" to="/liuo/uwb/observations"/>
    <remap from="diagnostics" to="/liuo/uwb/diagnostics"/>
  </node>
</launch>
```

原仓库的 `linktrack.launch` 并没有把端口写成可通过 `port_name:=...` 传入的 launch arg；项目版要显式定义 `<arg>`，不要只在终端增加一个不存在的参数就认为端口变了。[S5]

### 15.4 建议提交拆分

| 提交 | 范围 | 完成标准 |
|---|---|---|
| P0 | 固定基线、保存配置、原始抓包 | 不改功能；原驱动可以重现 |
| P1 | 接收上下文与 stamped wrapper | 原话题逐字段不变；源帧与接收序号可追踪 |
| P2 | 协议核心与 host 单元测试 | 和附录黄金向量一致；错误包被拒绝 |
| P3 | MCU 本地采集与 5 Hz 广播 | C1/C2/C5 通过；每条固定边确实按轮覆盖 |
| P4 | ROS bridge 与统一消息 | N0 动态输入不被 5 Hz 节流；远端数据来源正确 |
| P5 | 时钟、去重、重启和诊断 | 故障注入通过，未知时间不伪装成已同步 |
| P6 | 多节点性能与 LIUO 接入 | 按 §16 达成指标，或明确记录未达标项 |

### 15.5 逐文件修改检查表

| 路径 | 必做修改 | 禁止顺带改动 |
|---|---|---|
| `nlink_parser/src/linktrack/main.cpp` | 接收时间入口、会话、重连、缓冲与计数 | 不把串口数据人工重采样成 20 Hz |
| `nlink_parser/src/linktrack/init.h/.cpp` | context 接口、包装发布、TX 开关 | 不删除原消息字段；不无条件多线程化 |
| `nlink_parser/src/linktrack/protocols.*` / 校验入口 | 结构长度检查与错误计数 | 不自行解释 reserved 字节 |
| `nlink_parser/msg/*Stamped.msg` | 原帧包装 | 不破坏原 message MD5 |
| `nlink_parser/CMakeLists.txt` | 注册新消息与 target 依赖 | 不无关重构 TOFSense/AOA/IOT |
| `liuo_uwb_protocol/*` | header/CRC/Base64/边界检查 | 不依赖 ROS/动态地图 |
| `firmware/anchor_relay/*` | RX、源帧快照、分包、5 Hz 调度 | 不自动 Mesh，不逐包 ACK 风暴 |
| `liuo_uwb_bridge/*` | sender 解码、ID 映射、时间、去重、事件发布 | 不长期融合 Anchor 图 |
| `tests/*` | 原驱动回归、协议向量、故障与频率测试 | 不只测正常包 |

---

## 16. 测试与验收

### 16.1 软件测试矩阵

| 测试 | 输入/故障 | 必须看到的行为 |
|---|---|---|
| T01 原驱动回归 | 已有正常 NLink 帧 | 原话题字段、单位与上游一致 |
| T02 串口拆帧 | 一帧按任意字节切分 | 完整后只发布一次；不提前读取字段 |
| T03 串口粘帧 | 多帧一次到达 | 每帧有独立 rx_frame_seq，允许共享批次接收时间 |
| T04 坏长度 | 极大 frame_length、node_count 不匹配 | 有界拒绝、可恢复，不越界、不永远等待 |
| T05 外层校验 | NLink checksum 错误 | 不进入 unpack 正常路径 |
| T06 应用校验 | Base64、CRC、版本、count 错误 | 丢弃并计数，不产生距离 |
| T07 ID 不匹配 | outer sender=N2，payload reporter=N3 | 拒绝 |
| T08 单边重复 | 同一个 PacketKey 重放 | 不新增观测 |
| T09 旧源帧换轮次 | 不同 cycle，source_frame_seq 不变 | 不伪装成新观测 |
| T10 固定距离相同 | 新 source_frame_seq，distance 相同 | 接受新的报告 |
| T11 双向报告 | N1→N2 与 N2→N1 | 保留 reporter 信息，不能盲目视为独立约束 |
| T12 部分分包丢失 | 8 条边仅收到第一个 5 条包 | 已到 5 条立即交付，缺分包计数 |
| T13 序号跳变/回绕 | 模拟 uint32 边界 | 以会话与有限窗口判断，不误认新旧 |
| T14 设备重启 | local_time 回到低值 | 更新会话/时钟状态、清空半帧和待发旧包 |
| T15 时钟无效 | DR_MODE1 或未验证共享时钟 | 不使用 system_time；unknown 明确输出 |
| T16 过载 | 消费者变慢，TX/RX 队列积压 | 有界丢弃与告警，不拖延数秒再输出旧值 |
| T17 回环 | 固定端收到别人广播 | 不继续写回 UWB |
| T18 长期地图边界 | 连续运行数小时 | 驱动缓存有界；长期图只在 LIUO |

协议单元测试不能替代原 NLink 串口提帧测试；两层分别测试。

### 16.2 分阶段硬件测试

**阶段 H0：三节点能力验证。** 完成 §2.3 的 C1–C5，保存设备型号、固件版本、模式、Capacity、输出协议、UpdateRate、System ID/CH 和 UART 波特率。不以论文/别的产品的参数代替当前实机参数。

**阶段 H1：广播关闭的基线。** 先测 N0 本地帧率、各边报告率、距离序列、串口字节率、接收延迟与掉包。所有固定节点仍按正常工作配置测距，仅 MCU 不注入透传负载。

**阶段 H2：一个固定节点 5 Hz 广播。** 逐项确认源帧序号、source age、100 B 负载、RSSI、方向与 ID。检查 N0 动态流没有被固定边逻辑节流。

**阶段 H3：所有固定节点 5 Hz。** 比较同步提交、错峰提交、有界抖动的实际结果；不能预设某种调度一定最好。用发送日志与接收日志区分源帧缺失、MCU 未发、无线未达和 ROS 排队。

**阶段 H4：局部覆盖与长隧道。** 让机器人只覆盖一条固定边的一端，验证双端上报策略不会错误丢边；测试节点进入/离开、隐藏节点、网络分裂/重连以及系统时间连续性。

**阶段 H5：LIUO 实测。** 先记录 bag，检查观测身份和时间，再用于联合优化。不要边调协议边把定位误差改善归因于某一个算法因素。

### 16.3 验收表

| 项目 | 正常模式要求 | 不通过时如何表述 |
|---|---|---|
| 固定边数据来源 | 实测本地确有 Ni–Nj 距离 | “硬件前提未满足”，不是驱动 bug 已修复 |
| N0 合格边报告率 | 稳态统计目标 ≥20 Hz | 列明受影响的边与阶段，不用总话题率掩盖 |
| 广播覆盖 | 每轮包含全部当前有效固定边，轮次目标 5 Hz | 缺边/漏轮次直接计数 |
| 固定边接收率 | 分别报告实际值；需要严格 ≥5 Hz 时实测满足 | 5 Hz TX 但 4.8 Hz RX 不能声称 RX ≥5 Hz |
| 源帧新鲜性 | 序号/时间前进，无长期重发冒充新数据 | 无法证明真实 TWR 更新时标为 unknown |
| 协议可靠性 | 错包不产生观测；长度一致；可恢复 | 保留错误原始字节，不编造补值 |
| 动态链路影响 | 开广播后仍满足 ≥20 Hz；与基线比较 | 对明显恶化先排查，不用 UART 空闲证明无线无影响 |
| 资源 | MCU 与主机内存、队列有界 | 停止积压，丢弃过期包并告警 |
| 状态透明 | 不自动隐藏降频、无效时钟与未知能力 | 明确 DEGRADED / UNKNOWN |

若希望增加“广播开启后 N0 每边报告率下降不超过 5%”的回归门槛，可作为项目验收指标，但应预先定义统计方法和基线。本手册没有将该数值冒充厂家保证。

### 16.4 ROS 检查与录包示例

在新增程序实现后，可执行：

```bash
# 原始本地帧率：只反映该输出话题，不能证明每条边都达到相同频率。
rostopic hz /uwb/n0/nlink_linktrack_nodeframe3

# 统一话题包含两类事件，不能把它的总频率当成 N0 测距率。
rostopic echo -n 1 /liuo/uwb/observations
rostopic echo -n 1 /liuo/uwb/diagnostics

rosbag record -O liuo_uwb_test.bag \
  /uwb/n0/nlink_linktrack_nodeframe3 \
  /uwb/n0/nlink_linktrack_nodeframe0 \
  /uwb/n0/nlink_linktrack_nodeframe3_stamped \
  /uwb/n0/nlink_linktrack_nodeframe0_stamped \
  /liuo/uwb/observations \
  /liuo/uwb/diagnostics
```

此外保存配置 YAML、git commit/submodule SHA、固件参数截图和 MCU 发包日志。高价值实验还应保存一份原始串口字节流，以便区分 radio、串口和 ROS 层问题。

---

## 17. 给开发者的实施顺序

先把原驱动封存为可回归基线；用抓包确认固定端真实测距来源；实现独立协议并通过黄金向量；在 N1 上用 MCU 发送已知报告，N0 解码；再加入真实固定边、源时间和序号；最后开启全节点 5 Hz 与统一 ROS 输出。

第一轮不要同时做多跳、地图优化、跨 ROS 版本迁移、复杂所有者选举或自定义空中调度。每次增加功能都要能回答“这条距离从哪个真实源帧来、在哪一层被延迟/重复/丢弃”。

完成后的系统应具备以下确定行为：

```text
N0 直接测距：尽可能高频，合格边有效报告目标至少 20 Hz。
Ni 固定边报告：正常模式每 200 ms 覆盖一轮，不等机器人请求。
无线通路：只使用 UWB 模块已有数传，不增加 Wi-Fi/Ethernet。
数据边界：驱动做通信和时间语义；LIUO 做长期静态图与融合。
失败边界：测距源缺失、更新率不足、时钟未知时明确暴露，不补造。
```

---

## 18. 尚需实机确认、不得由手册推定的事项

这些不是继续编写软件的阻碍，而是正式部署验收必须有答案的项目。

| 项目 | 为什么重要 |
|---|---|
| 当前每台设备型号、固件、模式和真实输出协议 | 决定是否有固定边来源，以及数传格式 |
| 本地输出帧与每条 TWR 结果的更新关系 | 决定“≥20 Hz 测距”能否被证明，而不只是输出频率 |
| 各边的采样时刻相对帧时间的偏移/跨度 | 决定高速机器人融合时的时间误差 |
| 多个用户串口写入在无线端如何排队、合并、拆分 | 决定包长、间隔和丢包定位 |
| 每节点 DT Rate 是否在当前局部覆盖/长隧道拓扑仍可达到 | 决定全网负载是否可扩展 |
| 时间基准节点离开/重启、网络分裂再合并后的行为 | 决定跨节点 system_time 是否仍可用 |
| 同一边两端报告是否复用同一测距交互 | 决定 LIUO 是否可以按独立观测使用 |

应直接使用串口抓包、MCU 日志和厂家对当前固件的说明回答这些问题；不能把芯片层面的通用 TWR 能力当成商用封闭固件的既有接口。

---

## 19. 来源、版本与可追溯性

### S1. 用户提供的《LinkTrack 用户手册 V2.3》

文件：`LinkTrack_User_Manual_V2.3_zh.pdf`，78 页。  
SHA-256：`5eed4d84fde41eda93d27f7b03fb48dd5f9f282dd25907023ac82c229e1c1e2b`。

主要定位：p.26–29 DR 模式、节点局部距离与数传；p.39 距离有效性；p.42–43 时间字段；p.47–52 协议解析；p.73 FAQ Q38 的控制器读取后回写转发路径。

### S2. 用户提供的《LinkTrack 数据手册 V2.3》

文件：`LinkTrack_Datasheet_V2.3_zh.pdf`，59 页。  
SHA-256：`1ae16c35b899360cd63fb0e050496470d15c0d00fcc347403e6c475d99397e74`。

主要定位：p.7–8 UART 接口、电平与并用限制；p.17 DR 行为；p.24–27 模式容量、UpdateRate、DT Rate、长度与缓存；p.28 波特率；p.38–40 协议分类。

### S3. 用户提供的《NLink V1.4》

文件：`NLink_V1.4.pdf`，3 页。  
SHA-256：`f4588128a1de8a7e285a8edfaabe9d8567fda733a4a6c44fdd459235f41fbb05`。

主要定位：p.1–2 Node_Frame0/2/3/5/6 格式；p.3 Protocol Rule、Role Table。Frame3 的 7 B 记录结构与固定版本代码相符，用户手册示例中的额外 reserved 行差异已经在 §5.4 显式注明。

### S4. 上游仓库与 README

[nooploop-dev/nlink_parser 固定基线](https://github.com/nooploop-dev/nlink_parser/tree/77a3238a5b73e7545a032570606492e2b7642b78)  
[README.md](https://github.com/nooploop-dev/nlink_parser/blob/77a3238a5b73e7545a032570606492e2b7642b78/README.md)

依据：ROS 1 用法、现有测距/数传话题、开源解析层说明。固件闭源不等于这个主机解析驱动闭源；本项目修改的是可访问的主机代码和自有 MCU 应用。

### S5. 串口入口、参数和 submodule

[main.cpp](https://github.com/nooploop-dev/nlink_parser/blob/77a3238a5b73e7545a032570606492e2b7642b78/src/linktrack/main.cpp)  
[init_serial.cpp](https://github.com/nooploop-dev/nlink_parser/blob/77a3238a5b73e7545a032570606492e2b7642b78/src/utils/init_serial.cpp)  
[linktrack.launch](https://github.com/nooploop-dev/nlink_parser/blob/77a3238a5b73e7545a032570606492e2b7642b78/launch/linktrack.launch)  
[.gitmodules](https://github.com/nooploop-dev/nlink_parser/blob/77a3238a5b73e7545a032570606492e2b7642b78/.gitmodules)

### S6. 已有发布/订阅和消息定义

[init.cpp](https://github.com/nooploop-dev/nlink_parser/blob/77a3238a5b73e7545a032570606492e2b7642b78/src/linktrack/init.cpp)  
[LinktrackNodeframe3.msg](https://github.com/nooploop-dev/nlink_parser/blob/77a3238a5b73e7545a032570606492e2b7642b78/msg/LinktrackNodeframe3.msg)  
[LinktrackNodeframe0.msg](https://github.com/nooploop-dev/nlink_parser/blob/77a3238a5b73e7545a032570606492e2b7642b78/msg/LinktrackNodeframe0.msg)  
[LinktrackNode2.msg](https://github.com/nooploop-dev/nlink_parser/blob/77a3238a5b73e7545a032570606492e2b7642b78/msg/LinktrackNode2.msg)

### S7. 构建定义

[CMakeLists.txt](https://github.com/nooploop-dev/nlink_parser/blob/77a3238a5b73e7545a032570606492e2b7642b78/CMakeLists.txt)  
[package.xml](https://github.com/nooploop-dev/nlink_parser/blob/77a3238a5b73e7545a032570606492e2b7642b78/package.xml)

### S8. MCU 可复用的 C 解析器

[nlink_unpack 固定基线](https://github.com/nooploop-dev/nlink_unpack/tree/1b790459c62bd7f15c18d98d9c87d13471f6db47)  
[nlink_linktrack_nodeframe3.c](https://github.com/nooploop-dev/nlink_unpack/blob/1b790459c62bd7f15c18d98d9c87d13471f6db47/nlink_linktrack_nodeframe3.c)

### S9. 厂家公开下载入口

[Nooploop 资料下载](https://www.nooploop.com/download/)

本手册关于协议和能力的主要依据是用户上传的上述三个文件及固定源码，不以搜索摘要或其他产品页面替换实机能力。公开下载入口仅供查找原厂后续说明；若更新文档或固件，应重新执行前置验证与回归测试。

---

## 附录 A. 黄金测试向量

本附录是**自定义协议 v1 的软件测试向量**，不是实际 UWB 采集的数据。

参数：

```text
flags=7
reporter_id=1
boot_id=0x10203040
tx_cycle_seq=7
source_frame_seq=1234
source_local_ms=123456
source_system_ms=123400
hold_age_ms=12
shard_index=0
shard_count=1
peer_id=2..6
edge_flags=3
distance_mm = 10000 + peer_id×123
fp_code = 160 + peer_id
rx_code = 150 + peer_id
```

Binary hex，共 72 B，最后两字节 `46 67` 是 little-endian CRC：

```text
4c 52 01 07 01 05 40 30 20 10 07 00 00 00 d2 04 00 00
40 e2 01 00 08 e2 01 00 0c 00 00 01
02 03 06 28 00 00 a2 98
03 03 81 28 00 00 a3 99
04 03 fc 28 00 00 a4 9a
05 03 77 29 00 00 a5 9b
06 03 f2 29 00 00 a6 9c
46 67
```

实际 UART 用户负载，共 100 B；下面展示的 `\n` 表示末尾**一个换行字节**，不是两个字符：

```text
~U:TFIBBwEFQDAgEAcAAADSBAAAQOIBAAjiAQAMAAABAgMGKAAAopgDA4EoAACjmQQD/CgAAKSaBQN3KQAApZsGA/IpAACmnEZn\n
```

已在本次编写环境执行的检查：包头 30 B、记录 8 B；1–5 条边的 wire 长度 `[60,68,80,92,100]`；黄金向量往返；2,000 组随机合法报文往返；CRC 错误与异常 envelope 拒绝。

**没有执行：** ROS/catkin 编译、STM32/ESP32 编译、真实 UWB 测距或射频吞吐实验。因此上述软件测试不能作为 20 Hz/5 Hz 的硬件性能证明。

---

## 附录 B. 主机端编解码参考代码

将以下代码保存为 `protocol_reference.py`，使用 Python 3.9 或以上运行。它只实现完整单包编解码与测试，不包含 UART stream framer、ROS bridge、跨包去重、时钟估计或 MCU HAL。C/C++ 实现应使用附录 A 做跨语言对照。

```python
"""LIUO Range Relay v1: host-side codec reference, not a UWB/ROS driver."""
from __future__ import annotations
import base64
import binascii
import math
import struct
from dataclasses import dataclass
from typing import Tuple

# < : little endian, no native alignment. Header=30 B, record=8 B.
HEADER = struct.Struct('<2sBBBBIIIIIHBB')
EDGE = struct.Struct('<BBiBB')
MAX_EDGES = 5
MAX_WIRE = 100
PREFIX = b'~U:'
SUFFIX = b'\n'
SYS_TIME_VALID = 1
LOCAL_TIME_VALID = 2
HOLD_AGE_VALID = 4
RANGE_VALID = 1
RSSI_VALID = 2

@dataclass(frozen=True)
class Edge:
    peer_id: int
    flags: int
    distance_mm: int
    fp_code: int
    rx_code: int

@dataclass(frozen=True)
class Packet:
    flags: int
    reporter_id: int
    boot_id: int
    tx_cycle_seq: int
    source_frame_seq: int
    source_local_ms: int
    source_system_ms: int
    hold_age_ms: int
    shard_index: int
    shard_count: int
    edges: Tuple[Edge, ...]

def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc

def _uint(name: str, value: int, bits: int) -> None:
    if not isinstance(value, int) or isinstance(value, bool) or not 0 <= value < (1 << bits):
        raise ValueError(f'{name}: expected uint{bits}')

def validate(p: Packet) -> None:
    if p.flags & ~7:
        raise ValueError('reserved packet flag bits are nonzero')
    if not 1 <= p.reporter_id <= 254:
        raise ValueError('reporter must be a fixed logical node in 1..254')
    for name in ('boot_id', 'tx_cycle_seq', 'source_frame_seq', 'source_local_ms', 'source_system_ms'):
        _uint(name, getattr(p, name), 32)
    if p.boot_id == 0:
        raise ValueError('boot_id=0 is reserved')
    if not 1 <= p.shard_count <= 51 or not 0 <= p.shard_index < p.shard_count:
        raise ValueError('invalid shard index/count')
    if not 1 <= len(p.edges) <= MAX_EDGES:
        raise ValueError('v1 range packet needs 1..5 edges; no synthetic heartbeat edge')
    _uint('hold_age_ms', p.hold_age_ms, 16)
    if p.flags & HOLD_AGE_VALID:
        if p.hold_age_ms == 0xFFFF:
            raise ValueError('known hold age cannot use unknown sentinel')
    elif p.hold_age_ms != 0xFFFF:
        raise ValueError('unknown hold age must be 0xFFFF')
    if not (p.flags & SYS_TIME_VALID) and p.source_system_ms != 0:
        raise ValueError('invalid system time must be encoded as zero')
    if not (p.flags & LOCAL_TIME_VALID) and p.source_local_ms != 0:
        raise ValueError('invalid local time must be encoded as zero')
    ids = []
    for e in p.edges:
        if not 1 <= e.peer_id <= 254 or e.peer_id == p.reporter_id:
            raise ValueError('invalid fixed peer id')
        if e.flags & ~3 or not (e.flags & RANGE_VALID):
            raise ValueError('invalid edge flags')
        if not 1 <= e.distance_mm <= 0x7FFFFF:
            raise ValueError('positive range must fit source int24 millimetres')
        _uint('fp_code', e.fp_code, 8)
        _uint('rx_code', e.rx_code, 8)
        if not (e.flags & RSSI_VALID) and (e.fp_code or e.rx_code):
            raise ValueError('unavailable RSSI must encode as zero with flag cleared')
        ids.append(e.peer_id)
    if ids != sorted(set(ids)):
        raise ValueError('peer IDs must be strictly ascending within a shard')

def encode(p: Packet) -> bytes:
    validate(p)
    raw = HEADER.pack(b'LR', 1, p.flags, p.reporter_id, len(p.edges),
                      p.boot_id, p.tx_cycle_seq, p.source_frame_seq,
                      p.source_local_ms, p.source_system_ms, p.hold_age_ms,
                      p.shard_index, p.shard_count)
    raw += b''.join(EDGE.pack(e.peer_id, e.flags, e.distance_mm, e.fp_code, e.rx_code) for e in p.edges)
    raw += struct.pack('<H', crc16_ccitt_false(raw))
    wire = PREFIX + base64.b64encode(raw) + SUFFIX
    if len(wire) > MAX_WIRE:
        raise ValueError('wire payload exceeds 100 bytes')
    return wire

def decode(wire: bytes) -> Packet:
    if not isinstance(wire, bytes):
        raise TypeError('wire must be bytes')
    if len(wire) > MAX_WIRE or not wire.startswith(PREFIX) or not wire.endswith(SUFFIX):
        raise ValueError('bad envelope or length')
    encoded = wire[len(PREFIX):-len(SUFFIX)]
    try:
        raw = base64.b64decode(encoded, validate=True)
    except (binascii.Error, ValueError) as exc:
        raise ValueError('invalid Base64') from exc
    if base64.b64encode(raw) != encoded:
        raise ValueError('non-canonical Base64')
    if len(raw) < HEADER.size + EDGE.size + 2:
        raise ValueError('truncated payload')
    fields = HEADER.unpack_from(raw)
    magic, version, flags, source, count, boot, cycle, seq, local, system, age, shard, shards = fields
    if magic != b'LR' or version != 1 or count > MAX_EDGES:
        raise ValueError('unknown magic/version or count')
    if len(raw) != HEADER.size + count * EDGE.size + 2:
        raise ValueError('length/count mismatch')
    if struct.unpack_from('<H', raw, len(raw)-2)[0] != crc16_ccitt_false(raw[:-2]):
        raise ValueError('CRC mismatch')
    edges = tuple(Edge(*EDGE.unpack_from(raw, HEADER.size + k*EDGE.size)) for k in range(count))
    p = Packet(flags, source, boot, cycle, seq, local, system, age, shard, shards, edges)
    validate(p)
    return p

def rssi_code(db: float) -> int:
    if not math.isfinite(db) or not -127.5 <= db <= 0:
        raise ValueError('RSSI outside representable vendor range')
    return int(round(-2.0 * db))

if __name__ == '__main__':
    import random
    assert HEADER.size == 30 and EDGE.size == 8
    assert crc16_ccitt_false(b'123456789') == 0x29B1
    p = Packet(7, 1, 0x10203040, 7, 1234, 123456, 123400, 12, 0, 1,
               tuple(Edge(j, 3, 10000+j*123, 160+j, 150+j) for j in range(2,7)))
    wire = encode(p)
    assert len(wire) == 100 and decode(wire) == p
    print('GOLDEN_ASCII_REPR=', repr(wire))
    print('GOLDEN_BINARY_HEX=', base64.b64decode(wire[3:-1]).hex(' '))
    print('WIRE_LENGTHS=', [len(encode(Packet(7,1,1,1,1,1,1,0,0,1,p.edges[:n]))) for n in range(1,6)])
    rng = random.Random(20261025)
    for _ in range(2000):
        n = rng.randint(1,5)
        edges = tuple(Edge(j,3,rng.randint(1,500000),rng.randrange(256),rng.randrange(256))
                      for j in range(2,2+n))
        q = Packet(7,1,rng.randint(1,2**32-1),rng.randrange(2**32),rng.randrange(2**32),
                   rng.randrange(2**32),rng.randrange(2**32),rng.randrange(65535),0,1,edges)
        assert decode(encode(q)) == q
    raw = bytearray(base64.b64decode(wire[3:-1]))
    raw[35] ^= 1
    corrupted = PREFIX + base64.b64encode(raw) + SUFFIX
    try:
        decode(corrupted)
    except ValueError:
        pass
    else:
        raise AssertionError('CRC corruption not rejected')
    for bad in (wire[:-1], wire+b'X', b'~U:!!!!\n', wire[:20]+b'\n'):
        try:
            decode(bad)
        except ValueError:
            pass
        else:
            raise AssertionError('malformed envelope not rejected')
    print('PASS: golden vector, 2000 randomized round trips, CRC and malformed input checks')
```

---

## 附录 C. MCU 与 ROS 实现必须共享的约定

| 约定 | 必须一致的内容 |
|---|---|
| 类型与字节序 | `uint8/uint16/uint32/int32` 宽度固定，little-endian |
| Base64 | 标准 alphabet，保留 `=` padding，不插入换行；只在 envelope 末尾换行 |
| envelope | 精确为 `~U:` + Base64 + `0x0A`，不额外添加 `0x0D` |
| CRC | CCITT-FALSE 的全部参数与 little-endian 尾部存放 |
| 长度 | Header30、Edge8、CRC2；每包最多5条，wire最多100B |
| 身份 | 逻辑 ID 来自同一注册表；不同原始角色/ID不得错误合并 |
| 源帧 | 分包来自同一冻结源帧，不混合不同历史时刻 |
| 时间 | 保留帧级时间，未知字段有明确标志，不冒充每边采样时刻 |
| 数据新鲜性 | source_frame_seq 递增的是本地输出帧，不是假造 TWR transaction ID |
| 会话 | MCU/UWB 重启与时钟异常触发会话处理；不能复用旧序号域 |
| 去重 | PacketKey 与 ObservationKey 各司其职；GeometricEdgeKey 属于 LIUO |
| 通信范围 | 仅保证已到达节点的直接广播；不自动多跳 |
| 广播频率 | 5 Hz 指完整邻接快照轮次，不是只发送一个分包的定时器频率 |
| 降级 | 必须显式配置和告警，不把旧值重发成新观测维持表观频率 |

距离编码尽量保留厂家原始毫米整数。若 MCU 仅取得官方解析器的 float 米值，应采用经测试的四舍五入转换，而非简单截断；跨语言测试应包含边界距离。RSSI 无效性以 flags 表达，不占用一个可能合法的厂家编码作为未说明的哨兵。

序号比较使用限定窗口内的模运算或会话重置策略，不能将无符号回绕后的低值一概视为旧包。所有状态都有容量限制；异常来源、错误长度和重复数据不能让内存无限增长。

---

## 最终设计结论

本版采用 **“高频 N0 直接测距 + 固定节点 MCU 每 200 ms 广播一轮本地固定边 + `nlink_parser` 最小修改 + 统一带来源/时间的 ROS 观测输入”**。

持续广播与长期静态地图分属不同层：前者由 MCU/驱动负责，后者由 LIUO 负责。任何降低广播频率、减少边集合、改成按需获取或增加多跳的变化，都需要显式修改设计，而不能以实现细节的名义替换这版需求。