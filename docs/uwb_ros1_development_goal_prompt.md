# UWB ROS1 开发启动 Goal Prompt

日期：2026-10-26。此文件用于下一轮实际开发；当前仅更新任务文档，不启动 goal、不创建分支、不执行代码移植或 Git 推送。下述提交和推送授权用于后续实际开发。

将下方“启动提示词”至文末的内容完整复制到有权访问这些路径的开发会话中。配套文件为同目录的 `uwb_ros1_development_requirements.md`；两份文档应一起交给执行者。

## 启动提示词

请创建并启动一个开发 goal：以指定的当前 ROS2 提交为基线，在当前工作区的 ROS1 分支完成 UWB 驱动的软件移植、测试、文档、Git 提交与远端推送，持续推进到配套需求书的软件完成条件满足。若环境提供 create_goal 工具，使用本任务作为 objective；本任务未指定 token budget。不要停留在规划或仅改 CMake 后宣称完成。

本次允许在当前仓库切换到 ROS1 开发分支并修改文件，完成后提交并推送 origin 上的同名分支；保留 ROS2 分支引用，不向 ROS2 分支提交或推送。现在开始实际执行下述任务。

### 固定输入

```text
当前工作区：/home/mint/ws_calib
当前源码目录：/home/mint/ws_calib/src
当前仓库：/home/mint/ws_calib/src/nlink_parse_driver
保留的 ROS2 分支：feat/uwb-ros2-relay-v1
基线 SHA：a13dc6dc26ff7fed11272c2ec9a9f29020805b9a
ROS1 开发分支：feat/uwb-ros1-relay-v1
远端：origin（git@github.com:Printeger/nlink_parse_driver.git）
需求书：/home/mint/ws_calib/docs/uwb_ros1_development_requirements.md
本提示词：/home/mint/ws_calib/docs/uwb_ros1_development_goal_prompt.md
默认目标：ROS Noetic / Ubuntu 20.04 / Python 3.8 / C++17 / catkin
```

首先完整阅读配套需求书、当前工作区、文档目录和仓库适用的 AGENTS.md。随后阅读基线中的设计手册、ROS2 需求书、LIUO 接入合同、消息 README 和软件/硬件验收报告。ROS1 平台与 API 差异按本需求书执行；wire 协议、业务身份、时间和频率语义继续遵守基线合同。

### 在当前工作区建立 ROS1 开发分支

先检查当前仓库 branch/HEAD/status、remote 和已有分支，记录启动状态并验证基线提交。当前核实 HEAD 为上述基线、分支为 feat/uwb-ros2-relay-v1、工作树干净；执行时仍须重新检查。若 ROS1 分支不存在且工作树允许安全切换，在当前仓库执行：

```bash
git -C /home/mint/ws_calib/src/nlink_parse_driver switch \
  -c feat/uwb-ros1-relay-v1 a13dc6dc26ff7fed11272c2ec9a9f29020805b9a
```

若本地或远端已有同名分支，先检查历史，继续已有任务分支或建立对应跟踪分支，不覆盖历史。遇到未提交的用户修改，保留并判断能否安全继续；不得自动 stash、reset、强制切换或删除文件。无需新建 workspace、worktree 或克隆仓库；当前仓库文件切换为 ROS1 是本次预期行为，ROS2 分支引用保持不变。

将当前两份任务文档复制到 `/home/mint/ws_calib/src/nlink_parse_driver/docs/` 并随实现提交。后续修改任务文档时同步外部原件与仓库副本，避免执行依据分叉。仅提交本任务文件，不改动 src 下相邻工程，不将构建产物、bag 或大体积原始日志提交 Git。

### 使用当前 Noetic 环境

已核实当前主机为 Ubuntu 20.04.6、`/opt/ros/noetic`、ROS_VERSION=1、Python 3.8.10，并具备 catkin 与 roscore。直接使用当前主机和工作区，不以 ROS2 Jazzy 主机或另建容器为前提。启动时复核环境和依赖版本，独立 shell 加载 `/opt/ros/noetic/setup.bash`，避免 ROS2/Conda 混入；不修改用户 shell 配置。

所有构建、测试、安装、日志和录包均留在 `/home/mint/ws_calib`。该工作区已有其他包及 build/devel/logs，先检查 catkin 配置与已有缓存；优先使用专用 catkin profile 和工作区内独立目录 `build_uwb_ros1`、`devel_uwb_ros1`、`install_uwb_ros1`、`logs_uwb_ros1`，仅构建目标四包及其依赖。干净构建验证使用本任务专用目录，禁止删除整个工作区的 build/devel 或全局 catkin clean。证据和 bag 放 `evidence/uwb_ros1/`。记录实际构建参数和 source 路径，不假定旧 devel 可用于本任务。

测试使用专用 ROS master 和 ROS_MASTER_URI。软件测试通过 PTY 输入，不争用实体串口；实体设备测试先确认该端口无其他所有者。只停止本任务创建的进程，不用全局 pkill 杀用户节点。

### 本轮必须完成的系统

```text
N0 串口 → nlink_parser/linktrack（ROS1，唯一串口所有者）
              ├─ 选中测距帧的 stamped 输出
              └─ 选中数传帧的 stamped 输出
                         ↓
                  liuo_uwb_bridge/range_bridge
                         ↓
                  /liuo/uwb/observations
                  liuo_uwb_msgs/UwbObservationBatch
```

本地直接测距输出 ROBOT_ANCHOR，远端固定节点 MCU 上报输出 ANCHOR_ANCHOR。新观测到达即发布，远端缺包不能拖慢本地流。固定节点每 200 ms 广播一轮完整邻接边，但 observations 不能被统一节流为 5 Hz。

默认配置保持当前用户已确认的行为：

- port=/dev/ttyACM0，baud=921600，range_frame=3，data_frame=0，input_namespace=/uwb/n0。
- raw_role=0、raw_id=19 对应 logical_id=0、is_fixed=false；12 号为固定逻辑 N12。不要重新把硬件 0 号当机器人，也不要保留 19 的冲突固定条目。
- publish_debug_topics=false；默认只创建两路 stamped 输入、observations 和两路 diagnostics。系统 rosout/rosout_agg 另外存在。
- debug=true 才恢复全部原始/stamped 话题。保持 Frame2/3/5 测距和 Frame0/6 数传选择能力。
- mode=unconfirmed、capacity=0、硬件 capability=false、enable_legacy_tx=false、allow_rate_degrade=false；串口有数据不等于固定端无线转发通过。

### 实施顺序

1. 完成 D0：核对当前工作区、Noetic 环境与基线，在当前仓库建立 ROS1 分支，保存文档并建立验收表。
2. 完成 D1：四包 catkin、消息生成、库导出、安装与依赖；保持 protocol/MCU standalone。先建立可编译的 ROS1 骨架和消息接口。
3. 完成 D2：基于当前 src/ros2/driver.cpp 和 publish.cpp 移植 roscpp 驱动。复用新版 framer、结构检查、串口重连、context 和 debug 开关。
4. 完成 D3：移植 bridge 的 ROS API，复用 registry/state/codec 算法，落实双层去重、时钟映射、reset_replay 和诊断。
5. 完成 D4：提供 robot.launch、平面 YAML、私有参数、ROS1 全局 use_sim_time 和 TCPROS/队列配置，验证所有 launch 覆盖参数。
6. 完成 D5：迁移全部相关 Python/集成测试至 rospy/ROS1 工具，通过 PTY 验证真实 ROS1 发布链路、故障、60 秒负载、慢订阅者和 ROS1 bag 回放。
7. 完成 D6：重新运行无 ROS 核心、MCU 模拟和协议交叉测试；更新 ROS1 README、迁移说明、使用示例和 LIUO 接入合同。
8. 完成 D7：按需求书 S0–S11 逐项核对证据，按逻辑小步提交到 ROS1 分支并推送 origin 同名分支；复核 ROS2 分支引用和相邻工程未被本任务修改。

允许调整顺序以减少返工，但不能跳过必需验收。环境阻碍时先推进可完成的软件核心和测试，再准确报告阻碍；没有执行 ROS1 编译和运行不能标为软件完成。

### 不得遗漏的 ROS1 差异

- 活动构建使用 catkin/message_generation/message_runtime，保持 C++17；生成接口和库导出必须支持干净并行构建与 install 使用。不要只恢复旧 CMakeLists.ros1.txt/package.ros1.xml；历史 src/linktrack 路径不是当前功能基线。
- `.msg` 中 builtin_interfaces/Time 转为 time；字段/枚举/单位不变。ROS1 Header.seq 不作协议序号；跨版本对照仅对这一传输差异做明确处理。device local_time 始终是设备毫秒。
- ROS1 NodeHandle 私有参数与 `/use_sim_time` 全局参数必须分清；配置文件不能仍保留 ROS2 的 `/**/ros__parameters` 包装。
- 串口读写、unpack 和有状态回调串行执行。用单调时钟定时器或循环维持轮询/诊断，不能让无 `/clock`、模拟时间暂停或倒退卡住串口。
- TCPROS 的有界 queue/non-latch 代替 ROS2 QoS 配置；不要声称 ROS1 实现了同等 best_effort，也不要遗留无效 reliable_qos 开关。
- vendor 发现规则核对 CATKIN_IGNORE；不要依赖 COLCON_IGNORE。移动 framer 时更新 firmware/anchor_relay 和测试脚本的路径引用。
- ROS1 时间类型的 C++/Python 字段和消息指针类型按实际生成代码处理；Python 测试必须兼容 3.8。
- rosbag2 改为 ROS1 rosbag。回放 stamped 时只启动 replay bridge，不能同时播放旧 observations 或打开串口。

### 不可放宽的业务合同

- wire 固定为 `~U:` + 标准 Base64 + LF，little-endian，30/8/2 字节结构，每包最多 5 条/100 B，CRC-16/CCITT-FALSE，匹配原黄金向量。
- 校验先于 unpack，失败不发布陈旧结果；每个物理帧唯一 rx_frame_seq，完整帧时间来自完成它的 read；重连清半帧并更新会话。
- Frame0/6 的内层 block 才标识发送者；检查 registry/reporter，保留 32 位 raw ID 和 role，发送者残片不能混拼。
- 两层去重、回绕、退休水位、缓存/tombstone 上限保留；新源帧相同距离仍接受，旧源帧新轮次仍拒绝重复。
- 时间映射是帧级估计；未知不确定度 NaN，无映射 stamp 为零且有效位明确，freshness=false；不将 DR_MODE1 system_time 用于跨节点同步。
- 本地有效边目标 ≥20 Hz，远端按 5 Hz 完整轮次报告。不能等待分包齐全、复制旧数据、删除不合格边或用话题总频率掩盖逐边失败。
- MCU 只转发自身本地固定边，一轮冻结单一源帧并发完全部分包；8 边每轮 5+3。不得增加自动多跳、ACK 风暴或未知设备配置命令。
- 统一观测不是既有 LIUO UwbRange 的直接替代；不填虚构 variance，不修改相邻 LIUO 优化器，不把软件模拟称为实机无线验收。

### 验收与完成报告

执行需求书 S0–S11，保留已执行命令、日志、报告、环境和代码 SHA。重点确保：

1. 默认话题数量正确；未选帧到达也不新增话题；debug 能恢复全部原始输出，原字段回归保持。
2. raw19→N0 修复有回归测试，错误映射能诊断；本地和远端两类观测均可从 PTY 走通。
3. Frame3/0、Frame2/0、Frame5/6、LP 同号异角色、DR_MODE1 32 位 ID 通过。
4. 坏帧恢复、拆粘包、未知 ID、发送者隔离、分包缺失、重复、乱序、重启、回绕、时间跳变、缓存耗尽和慢订阅均有对应验证。
5. 至少 60 秒 50 Hz 本地流＋多发送者 5 Hz 完整轮次，冻结边集合并分别报告逐边统计和 10 秒子窗口；不以总话题 hz 代替。
6. rosbag 实录实放，两次回放/reset、seek、/clock 暂停和跳变、接收时间保留与新会话通过；无重复观测来源。
7. core/MCU standalone 仍可无 ROS 构建，ROS2 分支引用及相邻工程未被本任务修改。

最终用中文报告：当前仓库路径、ROS1 分支、提交 SHA 和远端推送核验结果；实现范围与 ROS1 差异；可复制的构建/启动/诊断/录包回放命令；全部验收项状态及证据路径；ROS2 分支引用和相邻工程保护结果；剩余硬件、板级或 LIUO 集成工作。

用户已授权完成修改后推送 Git，无需再次询问常规提交与推送权限。执行 `git -C /home/mint/ws_calib/src/nlink_parse_driver push -u origin feat/uwb-ros1-relay-v1`，核验远端同名分支 SHA 与本地 HEAD 一致；不 force push、不合并到 main 或 ROS2 分支。认证、网络或远端冲突导致推送失败时保留本地提交并明确报告，不能宣称交付完成。标为 goal complete 前必须满足软件完成定义；预算或时间不足不能代替验收。硬件未执行可以独立列出，但不能把未完成的 ROS1 软件必需项包装成“仅待硬件”。
