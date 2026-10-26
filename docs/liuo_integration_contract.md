# LIUO 后续接入合同

当前交付 `/liuo/uwb/observations`（liuo_uwb_msgs/UwbObservationBatch），不是现有 `liuo_tunnel_msgs/UwbRange`，未修改优化器。UwbRange要求有效测量时间与variance_m2，本驱动不具备默认可信方差，不能无条件转换。

每个本地新帧/远端已到分包立即发布；不等分包齐全、不复制旧距离补频率。Batch Header为组装时间；received_stamp来自完成输入read的记录；source_frame_stamp只有有效映射才非零。距离单位米，保持源毫米编码，RSSI按-code/2，未知RSSI为NaN且rssi_valid=false。freshness始终false，不能由源帧序号证明独立TWR交互。

时间映射是128样本帧级线性回归，设备毫秒先按限定增量展开；至少8样本/100ms后检查漂移[0.995,1.005]与残差<50ms。设备跳变>10s、主机单调异常、ROS/monotonic增量差>100ms使拟合重建。此工程门限不是同步精度证据，单向串口延迟不可辨识，time_uncertainty_s为NaN。远端只在shared_system_clock=true、clock_domain非空、模式不是unconfirmed/DR_MODE1、N0系统时间映射有效时映射；DR_MODE1永远不使用system_time。映射仅限最近10秒设备时间邻域。

身份域是network_instance配置及batch.driver_session_id数据集；source_session_id为该数据集内的计数编号，不能脱离driver_session_id单独当全局唯一ID。N0源序号为成功测距帧的rx_frame_seq低32位，包含其他帧造成的跳号不是丢包；高32位变化时换本地source_session。远端由reporter/boot映射source_session。端口重连换本地会话、清碎片，但保留远端去重以拒绝重连后重放。

PacketKey按reporter/boot/cycle/shard，ObservationKey按reporter/boot/source_frame/peer。限定乱序窗口127、最大正向推进1000000；10秒/容量退休留下序号floor，不能因缓存清理而复活旧观测。每源两个窗口各32帧记录，全局包/观测各10000键；旧boot每源64、旧port64，耗尽拒绝并诊断，需显式开启新数据集。源必须以重启/设备时钟重置更新boot，不能复用序号域。

回放使用replay_mode=true，按记录monotonic差值处理，禁止把记录的monotonic与另一开机当前时间比较。每次重复/seek前调用`/liuo_uwb_bridge/reset_replay`（std_srvs/Trigger），或重启bridge。单输入流记录时间倒退也会自动开启新数据集；跨话题回调顺序不是seek。不得同时播放旧observations并启动bridge生成同一批新observations。

LIUO集成前必须决定：可接受time_basis及同步证据、噪声标定/方差来源、端点逻辑ID映射、producer/session/sequence域、双向AA报告相关性、无效时间拒绝策略。双向报告可能来自同一TWR交互，不是独立高斯约束。长期Anchor图、滤波、坐标、地图版本、NLOS与因子权重均归LIUO；驱动不生成这些结论。

ROS1 消费示例：`rosrun liuo_uwb_bridge observation_reader.py`。时间对象用 secs/nsecs 或 to_nsec()，Header.seq 为 ROS1 传输字段而非协议身份。传输为 TCPROS、有界队列100、非latch；慢消费者应自行处理丢弃，不能把接收数量当成独立测量数量。ROS1 操作入口与本次证据见 [README](../README.md) 和 [ROS1验收](ros1_software_acceptance.md)。
