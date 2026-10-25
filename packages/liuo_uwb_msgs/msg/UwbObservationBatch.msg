uint8 LOCAL_RANGE=0
uint8 REMOTE_EDGE_REPORT=1

# stamp 为本 batch 组装/发布时刻；不是所有观测的统一采样时刻
std_msgs/Header header
uint64 driver_session_id
uint64 batch_seq
uint8 receiver_id
uint8 trigger_type
UwbObservation[] observations
