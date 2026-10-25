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
builtin_interfaces/Time source_frame_stamp
builtin_interfaces/Time received_stamp
uint16 source_hold_age_ms
bool source_hold_age_valid
float32 time_uncertainty_s

# 测距与质量：保持源值；未知质量不可填成高置信度
float32 distance_m
float32 fp_rssi
float32 rx_rssi
bool rssi_valid
bool pair_measurement_freshness_verified
