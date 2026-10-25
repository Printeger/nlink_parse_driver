# ROS1 → ROS2 迁移边界

目标 Ubuntu 24.04 / ROS 2 Jazzy，主机 C++17。仓库根不再是包；只扫描 `packages/`，发现 nlink_parser、liuo_uwb_protocol、liuo_uwb_msgs、liuo_uwb_bridge。根目录原 CMake/package 通过 git mv 保留历史，ROS1 构建定义另保存在 nlink_parser/CMakeLists.ros1.txt、package.ros1.xml。

包名 nlink_parser、可执行程序 linktrack、LinkTrack Anchorframe0/Tagframe0/Nodeframe0–6 消息与原话题名保留。原消息字段名称/类型保留（Frame3 原已有 Header），ROS2 必需的 Header/Time 类型语法迁移除外。新 stamped 只包装原帧，不修改数传 wire。ROS1 MD5、ROS1 bag、catkin launch、订阅者二进制均不直接兼容 ROS2；需重新生成接口及迁移消费者。

**有意时间行为变化：** 导入版本曾将多种 local_time 写为主机 Unix 微秒；现在全部恢复厂家设备毫秒值，保留原 uint64 声明。Frame3 Header 是完成该帧的 read 的 ROS 时间，与 context.header 相同；不是 TWR 测量时间。NLinkRxContext 同时保留 monotonic、端口 session、唯一接收帧序号、function mark。坏长度阻塞后的恢复也使用完成好帧的原 read 时间，而非超时恢复时刻。

单进程单串口、单 executor 串行 unpack。结构验证（包括逐 block 边界和 vendor 224 B 数传数组容量）先于 unpack，失败不发布旧结果。最大 NLink 帧8192 B，RX容量16384 B；半帧自首字节100 ms后可恢复搜索。每字节批次标记与有限 read context 一起保留，断线清空。原始/stamped/观测 QoS 默认 best_effort/volatile/keep_last100，诊断 reliable/volatile/keep_last10；`reliable_qos=true` 必须两端一致，并重新检查积压。

Linux serial 以普通 CMake target 编译实际平台源码，不进入其 catkin 工程；保留 vendor 的 LICENSE、原 package.xml 并加 COLCON_IGNORE。nlink_unpack/protocol_extracter 是实文件，未更新远端或 submodule。导入快照5680f766f9b11a8426795f0589b1ccbfe9432af8并非手册上游SHA；来源URL及差异见实施记录。

vendored 两处修正有独立回归：Node1 漏复制 role；int24 解析对有符号 int 左移24位产生 UBSan 错误，改为无符号组装后显式符号扩展，不改变字段/字节/单位。Frame3 正式7 B邻居结构按原协议/源码保留，不采用手册示例中的额外reserved行。

TOFSense、TOFSenseM、AOA、IoT、RViz converter 原源码和旧launch保留但不编译成 ROS2 可执行程序。它们的源码存在不表示功能已迁移。不要执行原 README 的 catkin/submodule 更新命令。

legacy TX 默认关闭；开启后std_msgs/String限定100 B、队列8、25 ms间隔、50 ms期限。短写计失败不重试，端口断开清队列。该接口仍不应发送未知配置命令；bridge 永不打开串口或发送管理命令。
