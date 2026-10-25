# ROS2 实施记录

## D0：2026-10-26（Asia/Shanghai）

- 起点 HEAD：47da44a；导入快照：5680f766f9b11a8426795f0589b1ccbfe9432af8。开始时工作区干净，无 remote。
- 分支：feat/uwb-ros2-relay-v1。只修改本仓库；LIUO 消费端不在范围。
- 实际环境：Ubuntu 24.04 LTS，/opt/ros/jazzy；data-run --check 成功，Landlock ABI 7，系统盘写入被拒绝。
- 输出根：/home/dev/ws_iap/uwb_ros2；所有构建、测试、模拟、分析经 data-run；仅 packages/ 为 colcon 发现根。
- git mv 将原 ROS 包移至 packages/nlink_parser，仓库根不再有 package.xml。四包结构将逐步实现。
- extern/serial、src/utils/nlink_unpack、protocol_extracter 是 vendored 实文件；删除误导性的 .gitmodules，保留全部来源代码及 LICENSE。不更新依赖。
- 来源 URL：github.com/wjwwood/serial、github.com/nooploop-dev/nlink_unpack、github.com/nooploop-dev/protocol_extracter。手册列出的 upstream SHA 未与本地逐字节核验，不能宣称相同。
- ROS2 迁移以需求书覆盖手册 ROS1 构建路线；Frame3 保留现有 Header/uint64 local_time。其他帧恢复设备毫秒语义，不再写入主机 Unix 微秒。
- 协议差异：手册 §5.4 已记录 Frame3 示例 reserved 行冲突；采用正式格式及 vendored 代码的 22+7n，不改 wire format。
- 验证接缝遵循用户明确授权：独立协议/MCU 公共接口，以及原始字节/PTY → 公开 ROS2 输出；不为测试内部 getter 扩展接缝。

## 状态

D0 布局准备完成；D1–D7 尚未验收。硬件 C1–C5、H0–H5、板级烧录及 LIUO 接入均未执行。

## D1：协议核心

固定容量（Packet 5 条、binary 72 B、wire 100 B）C++17 codec，无 ROS/HAL/串口依赖，显式错误码，失败不覆盖输出 Packet。附录 B 原样提取为独立 Python 参考实现。

证据：/home/dev/ws_iap/uwb_ros2/evidence/d1-red.log（实现前链接失败）、d1-build.log、d1-test.log；独立 CMake Debug + ASan/UBSan 两项 CTest 通过，黄金向量、2000 组固定种子跨语言对照与非法字段测试。

首次 sanitizer 运行因 data-run 注入线程命名 shim 的 LD_PRELOAD 顺序失败；保留 shim 与全部存储保护，设置 ASAN_OPTIONS=verify_asan_link_order=0 后通过。该参数只允许现有 preload 排序，不关闭地址/未定义行为检查。

## D2/D3：ROS2 串口驱动

Jazzy ament/rosidl/rclcpp 构建并安装成功（d2-build-retry.log）。原 LinkTrack Anchor0/Tag0/Node0–6 原话题保留，Frame0/2/3/5/6 增加 stamped。原 ROS1 及非 LinkTrack 源码保留，但不在活动 target。serial 以普通 Linux CMake target 链接，vendor COLCON_IGNORE 防止独立发现。

- 恢复所有原帧 local_time 设备毫秒；Frame3 Header 与 context 相同。Node1 vendored unpack 遗漏 role 赋值，已按原始头字段补上，不改 wire。
- 验证长度/checksum/count/block（含 224 B vendor 单 block 容量）后才 unpack，按返回值门控发布；malloc 失败原宏已有 return 0，保留。
- 有界 framer、100 ms 半帧超时恢复、单串口单 executor、完成 read 时间、不同帧独立序号；重连清 RX/TX 并换 session。
- legacy TX 默认禁用，开启时上限 100 B/包、8 包、25 ms 间隔、50 ms 过期，部分写计失败。
- d3-framer.log：ASan/UBSan，原始九组夹具 1785 切点，坏长度/checksum/block 恢复，1 MB 垃圾输入资源上限。
- d3-pty.json 与 /home/dev/ws_iap/uwb_ros2/pty-driver/driver.log：真实 PTY → ROS2 原始/stamped，全部 LinkTrack 帧、设备时间、关键距离/数传长度、Frame3 所有切点与故障恢复通过。此处为软件模拟，不是射频证据。
- 尚需 D7 扩展端到端故障/负载/拔插与全字段回归；当前不宣称 S1–S9 全部完成。

## D4/D5：统一消息、身份、即时 bridge、时间及会话

四包 Jazzy 构建通过：d5-build.log。新消息逐字段取自手册，仅 ROS2 Time 语法迁移。独立 registry 校验 raw/system/mode/role/uint32 ID 与逻辑身份，接收者 N0 与内层 reporter 双重检查。按 sender 独立 envelope（100 B、250 ms），本地/每个分包即时发布。

有界双层去重：每 sender 最多32序号记录、127回溯窗口、最大前进1000000；包/观测各最多10000身份键，10 s 退休。退休 floor 防止过期重放复活；每 sender 最多64旧 boot，端口64旧会话，耗尽时拒绝新会话并计数，需显式新数据集。双向来源保留。

帧级时钟128样本线性漂移/偏移拟合，检测毫秒回绕、重启和 ROS 跳变。单向绝对延迟不可辨识，uncertainty 始终 NaN；远端只有明确共享时钟域才能映射，DR_MODE1 禁用 system_time。回放使用记录 context，倒退或 reset_replay 开启新 dataset；无效 source stamp=0、freshness=false。

证据：d5-state.log（缓存/回绕/时钟），d5-pty.log 与 pty-bridge-v4/faults 日志（24条预期观测、串口输入故障恢复/去重/部分分包/重连）。详细负载、资源、录放和其他帧适配由 D7 完成。
