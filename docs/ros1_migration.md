# 从固定 ROS2 基线迁移到 ROS1

基线 `a13dc6dc26ff7fed11272c2ec9a9f29020805b9a` 保留在 `feat/uwb-ros2-relay-v1`，当前工作目录切换到 `feat/uwb-ros1-relay-v1`。没有运行 ROS2 双版本环境；使用同一组冻结原始字节夹具、164项参考字段和独立协议 Python 参考验证。不能宣称 ROS1/ROS2 MD5、序列化或 bag 兼容。

四包从 ament/rosidl 改为 catkin/message_generation/message_runtime；C++17、协议库导出及安装保持。原 ROS2 驱动适配到 `packages/nlink_parser/src/ros1/`，无 ROS framer 仍保留原 `src/ros2/framer.cpp` 路径，因此 MCU 与测试引用不变。旧 ROS1 `src/linktrack` 不用于当前 target。

消息只把 `builtin_interfaces/Time` 改为内建 `time`；C++ 头为 `pkg/Type.h`、类型为 `pkg::Type` 和 ConstPtr；Python 使用 `secs/nsecs` 或 `to_nsec()`。ROS1 Header.seq 是传输层字段，不参与任何业务去重；原始与包装字段对照只归一化这一差异。local_time 保持设备毫秒，绝不恢复旧驱动的主机 Unix 微秒覆盖。

NodeHandle 私有参数，平面 YAML。launch 显式参数覆盖 YAML，`/use_sim_time` 全局生效。两个进程分别单线程 ros::spin，有状态逻辑串行。SteadyTimer 不依赖 /clock；零或暂停/倒退的 ROS 时间不能产生已验证源时间映射。时间不确定度 NaN、freshness=false，DR_MODE1 system_time 不用于同步。

数据 TCPROS queue=100、diagnostic queue=10、非 latch；bridge 使用 tcpNoDelay。有界队列会在慢消费者时丢弃，不等同于 ROS2 best_effort。`reliable_qos` 已退出活动配置，ROS2 launch 和 rosbag QoS 文件移至冻结 `docs/ros2_reference/`，不安装。

`rosbag record/play` 替换 rosbag2。等待订阅连接避免短 bag 丢开头帧；直接 observations 与 stamped→bridge 两种回放互斥。重复/seek 前 Trigger reset 或重启 bridge，数据集会话变更。live reset 被拒绝。

wire、CRC、逻辑身份、双层去重、退休水位和 tombstone 上限不变。固定节点 MCU 只转发自身本地固定边，5 Hz 完整轮次、8边5+3，不增加多跳。真实无线与板级、数小时实机、LIUO 噪声与时间策略仍未验收。

构建、运行和测试详见仓库 [README](../README.md)，本轮证据详见 [验收](ros1_software_acceptance.md)。
