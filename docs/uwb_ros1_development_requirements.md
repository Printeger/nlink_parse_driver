# UWB ROS1 版本开发任务需求书

版本：v1.1。日期：2026-10-26。状态：待启动开发。

以当前 ROS2 版本为功能基线，在当前工作区的 ROS1 分支实现同等行为的驱动、桥接、消息和测试，完成后提交并推送 Git。保留 ROS2 分支引用；允许当前仓库工作目录随切换和开发更新为 ROS1。本文件只规定后续开发任务，不表示 ROS1 代码、分支或验收已经完成。

## 1 基线与分支边界

| 项目 | 固定值 |
|---|---|
| 当前工作区 | `/home/mint/ws_calib` |
| 当前源码目录 | `/home/mint/ws_calib/src` |
| 当前开发仓库 | `/home/mint/ws_calib/src/nlink_parse_driver` |
| 保留的 ROS2 分支 | `feat/uwb-ros2-relay-v1` |
| ROS2 基线提交 | `a13dc6dc26ff7fed11272c2ec9a9f29020805b9a` |
| 远端 origin | `git@github.com:Printeger/nlink_parse_driver.git` |
| ROS1 开发与推送分支 | `feat/uwb-ros1-relay-v1` |
| 本需求书 | `/home/mint/ws_calib/docs/uwb_ros1_development_requirements.md` |
| 配套 goal prompt | `/home/mint/ws_calib/docs/uwb_ros1_development_goal_prompt.md` |

直接在当前仓库从指定基线创建并切换 ROS1 分支，不另建 workspace、worktree 或克隆。当前已核实 HEAD 等于基线、所在分支为 ROS2 分支、工作树干净；实际启动时重新记录 branch/HEAD/status、remote 和分支历史。已有本地或远端 ROS1 分支时先检查是否可继续，保留其历史并正确设置跟踪关系。不得通过强制切换、reset、自动 stash 或覆盖目录处理用户已有修改。

本任务可以改变当前仓库工作目录内容，但不得移动 ROS2 分支引用或向其提交、推送。启动和结束记录 ROS2 分支 SHA，保留用户期间自行产生的更新。不得修改 src 下相邻工程；共享工作区已有其他包和构建结果，清理与构建限于本任务范围。

两份任务文档当前位于仓库之外；后续开发启动时复制到当前仓库 `docs/` 并纳入 ROS1 分支，后续修订同步两个位置。用户已授权本任务的小步提交和完成后推送 `origin/feat/uwb-ros1-relay-v1`，无需再次申请常规推送许可。只暂存任务相关文件，不 force push、不合并到 main 或 ROS2 分支。推送后核验远端分支 SHA 与本地 HEAD 一致；失败时保留提交并报告实际阻碍。

本次仅修订两份任务文档，不启动开发 goal、代码移植、分支切换或推送；以上授权与步骤用于后续实际开发。

## 2 目标环境与交付范围

当前主机已核实为 Ubuntu 20.04.6 LTS、ROS Noetic（`/opt/ros/noetic`）、ROS_VERSION=1、Python 3.8.10，catkin 和 roscore 可用。默认直接在此主机及 `/home/mint/ws_calib` 开发和验证，使用 C++17、CMake 3.16 及以上、catkin/roscpp/rospy。启动时复核依赖和工具链版本；已安装 ROS1 不代表目标四包已编译或通过验收。

使用独立 shell 加载 `/opt/ros/noetic/setup.bash`，避免 ROS2、Conda 和旧工作区缓存混入，不修改用户 shell 配置。记录实际编译器、CMake、Python 和依赖版本，以及必要的依赖获取步骤。

当前工作区已有 build/devel/logs 和其他包。先检查 catkin 配置，优先采用专用 profile 及工作区内 `build_uwb_ros1`、`devel_uwb_ros1`、`install_uwb_ros1`、`logs_uwb_ros1`，仅构建目标四包与依赖。干净构建和 install 验证均在本任务专用目录进行；禁止删除工作区整体 build/devel、执行全局 catkin clean 或改动相邻包以绕过无关构建错误。软件测试使用 PTY 和独立 master，证据及 bag 放 `/home/mint/ws_calib/evidence/uwb_ros1/`。

| 交付项 | 本轮要求 |
|---|---|
| ROS1 主机软件 | 四个包完成 catkin 构建、运行、安装、消息生成和测试 |
| 协议与状态核心 | 复用已验证算法及修复，保持行为和 wire bytes |
| MCU 平台无关逻辑 | 保持可独立构建及模拟测试，必要时只调整路径和构建连接 |
| 板级固件与无线验收 | 单独记录；缺少设备/板型不能冒充完成 |
| LIUO 优化器接入 | 提供 ROS1 消费接口合同和示例；不修改相邻 LIUO 工程 |
| ROS1 与 ROS2 跨域互通 | 不包含 ros1_bridge、ROS2 bag 转换或混合运行部署 |

## 3 必须继承的当前功能

以下是本次迁移的基线，不得因切回 ROS1 而退回旧驱动行为：

1. 单串口所有者 `nlink_parser/linktrack`，独立 `liuo_uwb_bridge/range_bridge`；串口断线重连、有限输入缓冲、完整结构检查、解析成功门控和接收 context。
2. Range Relay v1、按发送者拆粘包、身份注册、双层去重、时间映射、会话变化处理、诊断和回放重置。
3. 两类数据即时进入 `/liuo/uwb/observations`：机器人到固定节点的 `ROBOT_ANCHOR`；远端固定节点上报的 `ANCHOR_ANCHOR`。没有新观测时不发送虚构或重复数据。
4. 默认串口 `/dev/ttyACM0`、921600 baud、range_frame=3、data_frame=0、input_namespace=/uwb/n0。
5. 当前注册表中 `(system_id=0, operating_mode=unconfirmed, raw_role=0, raw_id=19)` 对应 `logical_id=0, is_fixed=false`。19 是硬件编号，N0 是机器人逻辑身份；不要要求用户重设硬件 ID。
6. 默认 `publish_debug_topics=false`，只创建选中的两路 stamped 输入及诊断；不能把其他原始话题改名成“隐藏话题”后继续发布。

当前 19 号到 12 号的串口距离曾验证可解析，修正注册表后临时 bridge 曾输出 N0→N12 观测。这只证明当时的本地链路，不证明全网固定边转发或硬件 5 Hz 验收。固定节点 1–18 的其他注册项仍是待核对示例；mode/capacity/capability 保持未确认初值。

`CMakeLists.ros1.txt`、`package.ros1.xml` 和 `src/linktrack/*` 是历史参考，不能直接作为恢复入口：它们未包含当前全部功能，且旧代码存在覆盖设备时间等行为。实际移植入口是当前 `src/ros2/driver.cpp`、`publish.cpp`、桥接代码及其测试。

## 4 ROS1 迁移要求

### R01 工程与构建

保留 `packages/` 多包结构及四个包名：`nlink_parser`、`liuo_uwb_msgs`、`liuo_uwb_protocol`、`liuo_uwb_bridge`。ROS1 分支的活动构建改用 catkin、message_generation/message_runtime、roscpp、std_msgs、diagnostic_msgs、std_srvs、yaml-cpp；活动 target 不依赖 ament/rclcpp/rclpy/rosidl/builtin_interfaces。

确保消息生成 target 的依赖、头文件导出、库链接顺序、安装目标均正确；干净的并行构建不能靠前次生成文件才成功。devel 和 install 两种使用方式都要测试。协议包继续支持 `UWB_STANDALONE=ON`；使用普通 CMake 构建核心和 MCU 测试时不得要求 ROS。

继续以普通库 target 编译 vendored serial，保留许可证和既有修复；核对 catkin 发现结果，必要时给 vendor 增加 `CATKIN_IGNORE`，不能把 `COLCON_IGNORE` 当成 catkin 的忽略规则。不得恢复 vendor 的旧 catkin 嵌套构建，也不得自动更新依赖到浮动 master。

ROS1 适配代码可放在 `src/ros1/`。现有 `src/ros2/framer.cpp` 实际是无 ROS 核心；可以保留路径或在新分支移到中性目录，但必须同步 MCU CMake、测试脚本和全部引用。不能通过重新启用旧 NProtocolExtracter 来绕过新版结构校验。

### R02 消息兼容与序号

保持 UwbObservation、UwbObservationBatch、NLinkRxContext、所有 LinkTrack 消息及 stamped 包装的字段名称、数值宽度、枚举、单位和业务含义。仅做 ROS1 必需的语法/API 适配：

| ROS2 | ROS1 |
|---|---|
| `.msg` 中 `builtin_interfaces/Time` | 内建 `time` |
| `std_msgs/Header` | 保留类型，但包含 ROS1 自有 `seq` 字段 |
| `pkg::msg::Type`、snake_case 生成头 | `pkg::Type`、`pkg/Type.h` |
| ROS2 消息共享指针 | ROS1 生成的 ConstPtr 等实际类型 |
| Python `stamp.sec/nanosec` | 使用 rospy.Time API/`secs/nsecs` |

ROS1 `Header.seq` 不是设备帧序号，也不是去重键；不得替换 `rx_frame_seq`、`source_frame_seq`、`batch_seq`。原始帧和包装的距离、ID、设备时间及 header 时间必须一致；对照测试可以排除 ROS1 发布层管理的 Header.seq，不能排除真正的协议序号。ROS1 Header 定义见[官方源码](https://raw.githubusercontent.com/ros/std_msgs/noetic-devel/msg/Header.msg)。

不承诺 ROS1 和 ROS2 的序列化、MD5、bag 或订阅二进制兼容。也不假定与旧 ROS1 导入版完全兼容：原始 `local_time` 仍保留设备毫秒语义，禁止恢复为主机 Unix 微秒。新增接口的 MD5/消息定义应记录；旧消费者需要重编译和检查时间语义。

### R03 节点与执行模型

ROS1 默认节点名保持 `/uwb/n0/linktrack` 和 `/liuo_uwb_bridge`。驱动唯一拥有串口，bridge 只订阅 stamped 消息，不打开串口。

保持解析、串口写入与有状态桥接回调的串行执行。可使用单线程 callback queue 和 ros::spin；不能直接换成并行 spinner 后共享 unpack 全局状态、Serial 或去重表。定时器和 publisher/subscriber/service 对象必须保持有效生命周期。

串口轮询、重连、诊断、缓存过期使用单调时间驱动，不能依赖 `/clock` 推进。Noetic 提供 `createSteadyTimer`，也可采用明确使用 steady_clock 的有界循环；普通 ROS 时间 Timer 不能用于必须在暂停时继续工作的轮询。[roscpp API 源码](https://raw.githubusercontent.com/ros/ros_comm/noetic-devel/clients/roscpp/include/ros/node_handle.h)

保留现有串口超时、读取块、帧长、ring buffer、TX 队列等边界。串口失败应在诊断中可定位；启动配置和限速错误日志应能说明实际 port、baud 及断线原因，避免只有“进程启动”却无法判断连接状态。

### R04 参数与 launch

新增 ROS1 XML 启动文件 `liuo_uwb_bridge/launch/robot.launch`，支持 port、baud、config、registry、namespace、use_sim_time。示例：

```bash
roslaunch liuo_uwb_bridge robot.launch port:=/dev/ttyACM0 baud:=921600
```

把 `/**: ros__parameters:` 包装转换为 ROS1 可加载的扁平 YAML，并分别加载到 driver、bridge 的私有参数空间。明确通过私有 NodeHandle 读取参数。launch 显式参数覆盖 YAML 同名值；用测试证明端口、波特率、namespace 和注册表路径实际生效。

`/use_sim_time` 按 ROS1 全局参数语义设置，不能只设置两个节点的私有 `~use_sim_time`。测试使用独立 master，不改变用户已有 ROS1 master 的全局时钟设置。参数默认启动读取，修改 YAML 后需重启；不暗示支持热加载。

注册表继续是独立 YAML 文件，由 `registry_path` 加载。保留 19→N0、12→固定 N12；拒绝重复原始身份、重复逻辑 ID、缺失 N0 和 fixed 标志冲突。缺文件、非法 range_frame/data_frame、错误配置应明确失败，不悄悄使用其他身份继续运行。

### R05 话题与 ROS1 传输

默认启动且 debug=false 时，只创建以下业务发布话题：

```text
/liuo/uwb/observations
/liuo/uwb/diagnostics
/uwb/n0/nlink_diagnostics
/uwb/n0/nlink_linktrack_nodeframe3_stamped
/uwb/n0/nlink_linktrack_nodeframe0_stamped
```

ROS1 的 `/rosout`、`/rosout_agg` 以及按需出现的 `/clock` 不算业务冗余话题。ROS1 没有必须照搬的 `/parameter_events`。

range_frame 支持 2/3/5，data_frame 支持 0/6；相应替换选中的 stamped 话题。只有 `publish_debug_topics=true` 才创建全部原始帧和 stamped 话题。接收到未选帧时仍验证和统计，不额外 advertise、不发布旧数据。关闭调试输出不能破坏远端 Frame0/6 接收。

ROS1 默认采用 TCPROS、有界队列、非 latch 发布；数据队列初值 100，诊断初值 10，可在证据支持下调整。订阅可开启 tcpNoDelay 降低小包等待，但必须经延迟与慢订阅测试。ROS1 不存在 ROS2 best_effort/reliable QoS 的一一映射，不能把 TCPROS 描述成 best_effort；本轮不要求 UDPROS。`reliable_qos` 和 rosbag2 QoS 文件退出 ROS1 活动配置，并在迁移文档说明替代语义，禁止保留“看似有效但无行为”的开关。

### R06 协议与来源

Range Relay v1 wire 完全保持：`~U:`＋标准带 padding 的 Base64＋LF；little-endian；30 B header、8 B edge、2 B CRC；每包 1–5 条边，最多 100 B；CRC-16/CCITT-FALSE。保留全部非法字段、长度、CRC、Base64 和结构校验及黄金向量。

Frame0/6 顶层 ID 是接收者，内层 block 的 role/id 才是发送者。按完整 registry key 校验接收者=N0，内层发送者须与 payload reporter 匹配。DR_MODE1 原始 ID 保留 32 位，LP 同号异角色不合并。每个发送者独立、有界地重组 envelope，重连/超时清理残片。

保持 PacketKey 和 ObservationKey 两层去重：重复包丢弃，新轮次重发相同源帧同 peer 不形成新观测；新源帧即使距离不变也接受；双向报告保留但不冒充统计独立测量。保留序号回绕、退休水位、TTL、容量边界和旧会话 tombstone 策略。

### R07 时间与频率

完成一帧的串口 read 提供其接收 ROS 时间和 monotonic 时间；同一次 read 多帧允许同时间但具有不同 rx_frame_seq。断线、设备时间回绕/重启、ROS 时间跳变时按当前算法更新会话和映射状态。

保留帧级 ClockMapper，未知时间不确定度为 NaN，未映射 source_frame_stamp 为零且 time_basis 明确。DR_MODE1 system_time 不用于跨节点映射；只有验证过共享系统时钟且映射有效才允许远端映射。freshness 默认 false，不把帧序号变化或 ROS 发布频率当成新 TWR 证据。

在 use_sim_time=true 且未收到 `/clock`、时钟暂停、跳回时，串口读取/诊断仍能执行。零 ROS 时间不得被当成已验证的有效测量时间；明确其无效标志及恢复行为。禁止用“等待时钟有效”的无限阻塞把实时串口线程卡住。

本地有效观测即时发布，目标每条合格机器人边 ≥20 Hz；固定端 MCU 每 200 ms 发送一轮完整邻接报告。5 Hz 是固定端报告轮次目标，observations 总频率由两类事件共同决定。不得用 5 Hz 聚合器节流本地流，不等齐远端分包，不复制旧距离补频率。

### R08 MCU 与独立核心

复用 `firmware/anchor_relay`：每轮冻结单一最新源帧，按 peer 排序，每包最多 5 条，每轮覆盖全部有效固定边；8 条边仍为每轮 5+3 两包。只报告自身本地固定边，不转发 N0 动态边或他人广播，不增加多跳、逐包 ACK 或重传风暴。

源帧超时、排队超时、部分写、容量不足、boot/session 管理及模拟标记保持原合同。ROS1 迁移不改变无线协议，不需要为了适配 ROS1 重写 MCU 算法。MCU standalone 与 host simulation 必须重新编译运行，防止路径重排破坏依赖。

### R09 录包与回放

提供 ROS1 `rosbag record/play` 脚本和操作说明，默认录制两路选中 stamped 输入、统一观测和两路诊断；不要默认订阅已关闭的全部原始话题。rosbag2 数据不能直接当 ROS1 bag 使用。

两种回放方式分别演示：直接播放 observations 给消费者；或只播放 stamped 输入给 `replay_mode=true` 的 bridge 重新生成观测。后者不得同时播放已录制 observations，且不启动串口驱动。

保留 `/liuo_uwb_bridge/reset_replay`（std_srvs/Trigger）。每次循环播放或 seek 前显式重置/重启；单流倒退自动检测保留，跨话题回调顺序不当成 seek。回放按记录 monotonic 差值处理，不拿它与另一次开机的当前时钟比较。验收至少包括两次回放、接收时间保留、新数据集会话与 ROS 模拟时间暂停/跳变。

### R10 LIUO 接口与范围

ROS1 主接口是 `/liuo/uwb/observations`，类型为 `liuo_uwb_msgs/UwbObservationBatch`，同时承载 ROBOT_ANCHOR 和 ANCHOR_ANCHOR。提供可运行的 rospy 或 roscpp 示例，逐条读取 kind、端点、来源会话、时间依据、distance_m 和 RSSI 有效标志；明确 ID 为逻辑 ID。

当前相邻 LIUO 的既有 UwbRange 接口涉及有效测量时间和 variance_m2。ROS1 新接口不是即插即用的同类型替换，不创建虚构方差、不使用 batch header 充当所有边统一测量时间。本轮不修改优化器、不实现 ROS1↔ROS2 跨域桥、不宣称已完成融合。

## 5 实施任务

| 任务 | 具体工作 | 可验收结果 |
|---|---|---|
| D0 | 当前工作区、精确基线、ROS1 分支、Noetic 环境 | 分支与环境记录完整，ROS2 引用和相邻工程不变 |
| D1 | 四包 catkin、接口生成、依赖和安装 | 干净并行构建，devel/install 均可发现包和导入消息 |
| D2 | 驱动 roscpp 适配及选择性 advertise | 串口到 selected stamped 正常，debug 恢复原始输出 |
| D3 | bridge、registry、时间、去重和服务适配 | 本地/远端两路输出、身份错误诊断及重置可测 |
| D4 | ROS1 launch、YAML、传输和诊断 | 默认 5 个业务话题及覆盖参数正确 |
| D5 | rospy/rostest/PTY、回放和性能验证 | 原功能回归、故障、负载、bag 测试证据 |
| D6 | standalone/MCU 回归与 ROS1 文档 | 无 ROS 测试通过，用户操作命令可复制 |
| D7 | 验收报告、提交和推送、ROS2 引用复核 | 软件状态逐项明确，origin 同名 ROS1 分支 SHA 与本地一致 |

## 6 验收矩阵

主要集成测试入口是“原始串口字节/PTY → ROS1 stamped/observations/diagnostics”，不能只测试内部方法。测试输入明确标为合成数据，并使用独立 ROS_MASTER_URI/测试 master，避免误连用户节点。测试只清理自己启动的进程。

| 编号 | 必测行为 | 通过标准 |
|---|---|---|
| S0 | 开发范围与分支 | 当前仓库 ROS1 分支含指定基线；ROS2 引用、相邻工程及用户原有修改受保护 |
| S1 | 构建与安装 | 当前 Noetic 环境中使用干净的本任务构建目录构建四包、生成 C++/Python 消息；devel/install smoke 均通过 |
| S2 | 协议和解析 | 黄金向量、跨语言往返、所有帧夹具、拆粘帧/坏长度/坏 checksum 恢复通过；保留基线字段断言 |
| S3 | 本地与远端 | Frame3/0、Frame2/0、Frame5/6；LP 角色及 DR_MODE1 32 位 ID；两种 kind 的端点/距离正确 |
| S4 | registry 回归 | raw19→logical0 后输出 N0→N12；错误映射计 unknown_receiver；重复身份和缺 N0 启动失败 |
| S5 | 话题可见性 | debug=false 仅选中 stamped 和诊断；收到未选帧也不新增；debug=true 恢复全部原始/stamped 且字段一致 |
| S6 | 时间/身份/资源故障 | 重连清残片、回绕、设备重启、旧会话拒绝、两层去重、乱序、发送者隔离、缓存耗尽均与基线合同一致 |
| S7 | ROS1 特有行为 | 私有参数与覆盖正确；header.seq 不干扰身份；无 clock/暂停/跳变不阻塞读取和诊断；异常可定位 |
| S8 | 性能负载 | 连续 ≥60 s，9 条本地边各 50 Hz 输入，3 个发送者各 8 条远端有向边按 5 Hz 完整轮次输入；记录发送/接收逐边计数、10 s 窗口、延迟、队列/内存与慢订阅影响 |
| S9 | 录包回放 | ROS1 bag 实录实放；两种回放模式，重播/seek 重置、时间保留和新会话正确，无双重发布 |
| S10 | MCU 与无 ROS 构建 | standalone codec、framer、状态核心和 relay simulation 通过；8/18 边轮次不降频 |
| S11 | 交付与使用 | launch、echo、hz、诊断、调试开关、Python 示例和回放命令可执行；文档状态与证据一致；任务提交已推送，远端 SHA 核验一致 |

S8 的无故障 PTY 测试必须核对每条预定义边的发送/接收数量，不接受只用总话题频率证明达标。本地合格边输出不得低于 20 Hz；远端合成数据逐边应完整覆盖已发轮次。明确启动/连接建立区间、统计时间边界和尾部排空，不能用统计窗口外补发伪造通过。慢消费者在独立进程运行，确认其不会拖垮快消费者和串口读取。

ROS1 和 ROS2 对照使用相同固定夹具及参考期望值；对时间戳、会话随机数等非确定字段做有理由的归一化，保留顺序关系/唯一性/有效位断言。缺少 ROS2 环境时允许使用已冻结夹具和独立协议参考，不得伪造“已运行双版本对比”。

既有测试 `pty_driver_test.py`、`pty_bridge_test.py`、`state_integration_test.py`、`legacy_tx_test.py`、`bag_smoke.py`、`topic_visibility_test.py` 的功能覆盖必须迁移到 ROS1。更新 rospy API、时间字段、发现机制和进程清理；不能只将脚本名称从 ros2 改为 rosrun。Python 脚本须能在 Noetic 的 Python 3.8 上运行。

硬件验收另列：真实串口、C1 固定边本地来源、C2 同时测距与数传、C5 100 B 透传、单节点/全节点广播、动态每边频率与长期运行。已经通过的 ROS2 软件模拟不能自动记为 ROS1 或无线实测通过。

## 7 文档与产物

新分支至少交付：本需求书、goal prompt、ROS1 README、ROS2→ROS1 迁移说明、ROS1 LIUO 接入合同、软件验收报告、构建/测试/录包回放入口。明确旧 ROS1 历史文件、ROS2 冻结文档和当前 ROS1 活动操作说明的区别。

构建产物放当前工作区的本任务专用 build_uwb_ros1/devel_uwb_ros1/install_uwb_ros1/logs_uwb_ros1；证据、bag、MCU 构建放 evidence/uwb_ros1 或工作区内本任务专用输出目录，不纳入 Git。不要照搬旧文档 `/home/dev/ws_iap`、`data-run` 等另一台机器的路径；先读取实际适用 AGENTS.md，当前路径和用户指定的原地开发要求优先。

每个验收项记录命令、环境版本、结果、日志/报告路径、测试来源（合成/实机）和所测代码 SHA。报告和证据引用必须可定位，不能引用未复制过来的旧机器绝对路径并声称本次已执行。

## 8 完成条件

软件完成要求 D0–D7 和 S0–S11 必需项通过，代码与文档已在当前仓库 ROS1 分支提交并推送 origin 同名分支，远端 SHA 与本地 HEAD 核验一致，工作区状态可解释。推送失败不能标记交付完成。无 ROS1 编译环境、未执行 ROS1 launch/bag 或仅通过协议单测时，只能报告部分完成并列出阻碍。

硬件、板级固件、数小时实测和 LIUO 融合单独列出通过/失败/未执行，不把软件完成写成完整系统完成。不通过删除测试、放宽频率、复制旧数据、隐藏 unknown 或自动认领硬件能力来完成验收。

## 9 依据与优先级

用户当前要求、实际适用 AGENTS.md、本需求书的 ROS1 决策、冻结基线代码及测试、原设计手册依次用于解决工程差异；协议语义出现冲突时列出字段和证据，不自行更改 wire format。

基线仓库中的主要依据：

- `docs/uwb_driver_design_notebook.md`：协议、频率定义、数据流和硬件边界。
- `docs/uwb_ros2_development_requirements.md`：ROS2 功能合同与验收覆盖，ROS2 专属构建部分由本需求书替代。
- `docs/liuo_integration_contract.md`、`packages/liuo_uwb_msgs/README.md`：来源/时间/会话与消费端边界。
- `docs/software_acceptance.md`、`docs/hardware_acceptance.md`：区分已有软件证据与硬件未执行事项。
- 基线提交中的 `publish_debug_topics`、registry、launch 及 topic_visibility_test：当前运行默认值和新增回归。

外部资料仅用于确认平台/API 差异；协议与业务验收以冻结项目合同为准。ROS1 Header/时间语义与 ROS2 的区别还可参照[官方设计说明相关章节](https://design.ros2.org/articles/changes.html#messages-services)，该文较早，不把其中其他历史状态当作当前 ROS2 状态。
