# UWB 观测消息：功能、格式与使用方法

`liuo_uwb_msgs` 是 ROS 2 Jazzy 的消息接口包，将机器人本地测距和固定节点上报的测距边统一为可订阅、录包和回放的数据。它只定义消息，实际发布者是 `liuo_uwb_bridge/range_bridge`。

| 项目 | 内容 |
|---|---|
| 默认公开话题 | `/liuo/uwb/observations` |
| 话题类型 | `liuo_uwb_msgs/msg/UwbObservationBatch` |
| 单条观测类型 | `liuo_uwb_msgs/msg/UwbObservation` |
| 默认 QoS | Best effort、Volatile、Keep last，深度 100 |
| 距离单位 | 米；保留源测距值，不在驱动内滤波 |
| 目标平台 | Ubuntu 24.04 / ROS 2 Jazzy |

## 1. 功能与数据流

```text
N0 串口 → nlink_parser/linktrack（唯一串口所有者）
              ├─ 原始 LinkTrack 消息
              └─ 带接收 context 的 Stamped 消息
                         ↓
                liuo_uwb_bridge/range_bridge
                         ↓
                /liuo/uwb/observations
                         ↓
                显示、记录、分析、后续 LIUO 接入
```

桥接节点接收两类输入：

- **本地测距**：默认 Frame3，也支持配置为 Frame2 或 Frame5。输出 N0 到固定节点的 `ROBOT_ANCHOR` 观测。
- **远端报告**：固定节点 MCU 将自己的单帧测距快照封装为 Range Relay v1，经 UWB 数传送达 N0。桥接节点从 Frame0（或 Frame6）的**内层发送者 block**提取报告，输出 `ANCHOR_ANCHOR` 观测。桥接节点不打开串口。

本地新帧到达即发布；远端每个有效分包到达即发布其中的新观测，不等待整轮收齐。例如一个固定节点有 8 条边，一轮分为 5 条和 3 条两个包，通常对应两个 batch，而不是一个包含 8 条边的 batch。若后一包没有到达，前 5 条仍可使用。

同一个源帧在下一轮再次发送，不会变成新观测；新源帧即使距离数值相同，仍会被接受。A→B 与 B→A 分别保留，但不能据此认为它们是两次独立测量。

## 2. 消息格式

以下字段与实际 `.msg` 文件对应。ROS 消息通过 ROS2 的序列化机制传输；它不是串口上的 `~U:` + Base64 + LF 文本。30 字节头、8 字节边、CRC、每包最多 5 条/100 字节的限制属于 Range Relay v1 无线数传 envelope，不是所有 ROS batch 的长度限制。

### UwbObservationBatch：一次输出事件

定义：[UwbObservationBatch.msg](msg/UwbObservationBatch.msg)。

```text
uint8 LOCAL_RANGE=0
uint8 REMOTE_EDGE_REPORT=1

std_msgs/Header header
uint64 driver_session_id
uint64 batch_seq
uint8 receiver_id
uint8 trigger_type
UwbObservation[] observations
```

| 字段 | 语义与使用方式 |
|---|---|
| `header` | 标准 ROS Header，包含下述 stamp 与 frame_id |
| `header.stamp` | bridge 组装/发布这个 batch 的时间，**不是统一测量时间** |
| `header.frame_id` | 当前实现留空；消息不提供坐标或坐标系变换 |
| `driver_session_id` | bridge 数据集会话标识；bridge 重启或显式重置回放数据集后变化，不等于串口端口会话 ID |
| `batch_seq` | 当前数据集中的 batch 序号，从 1 递增；不能用于估算无线丢包率 |
| `receiver_id` | 接收端逻辑 ID，当前为 N0，即 `0` |
| `trigger_type` | `LOCAL_RANGE=0`：本地测距；`REMOTE_EDGE_REPORT=1`：远端报告 |
| `observations` | 本次接受的新观测列表；消费者应逐条处理，不能假设包含全部邻居 |

### UwbObservation：一条有来源的测距边

定义：[UwbObservation.msg](msg/UwbObservation.msg)。

| 字段 | ROS 类型 | 语义 |
|---|---|---|
| `kind` | `uint8` | `ROBOT_ANCHOR=0` 或 `ANCHOR_ANCHOR=1` |
| `reporter_id` | `uint8` | 报告者逻辑 ID；本地为 0，远端为实际固定节点 |
| `node_a` | `uint8` | 边的报告端；本地为 0，远端为 reporter |
| `node_b` | `uint8` | 边的另一端逻辑 ID |
| `source_session_id` | `uint64` | 数据集内的源会话编号；不能脱离 `driver_session_id` 作为全局唯一标识 |
| `source_frame_seq` | `uint32` | 源帧序号；本地来自驱动接收序号低 32 位，远端来自 MCU 报告 |
| `source_frame_seq_valid` | `bool` | 源帧序号是否有效；消费者应检查 |
| `tx_cycle_seq` | `uint32` | 远端发送轮次；本地为 0，不适用 |
| `shard_index` | `uint8` | 当前分包索引，从 0 开始；本地为 0 |
| `shard_count` | `uint8` | 该轮分包总数；本地为 1，不表示所有分包已到达 |
| `source_local_ms` | `uint32` | 源设备 local time，单位毫秒，具有 32 位回绕语义 |
| `source_system_ms` | `uint32` | 源设备 system time，单位毫秒；不是 Unix 时间 |
| `source_local_time_valid` | `bool` | 原始 local time 是否有效 |
| `source_system_time_valid` | `bool` | 原始 system time 是否有效；DR_MODE1 始终为 false，不等于跨节点已同步 |
| `time_basis` | `uint8` | 下表所列的时间依据 |
| `source_frame_stamp` | `builtin_interfaces/Time` | 映射到 ROS 时间域的源帧时间；无有效映射时为 `{sec: 0, nanosec: 0}` |
| `received_stamp` | `builtin_interfaces/Time` | 完成接收输入帧的那次串口 read 的 ROS 时间；远端为 N0 收到报告的时间 |
| `source_hold_age_ms` | `uint16` | 远端源帧在发送端持有至封包的年龄，毫秒；不是完整端到端延迟 |
| `source_hold_age_valid` | `bool` | hold age 是否可用；本地为 false，数值初始化为 65535，不能当有效年龄 |
| `time_uncertainty_s` | `float32` | 时间不确定度，单位秒；当前未知，输出 NaN，不能解释为 0 |
| `distance_m` | `float32` | 原始距离，单位米；无效或超出配置范围的距离不会作为有效观测发布 |
| `fp_rssi` | `float32` | 首径 RSSI，dBm |
| `rx_rssi` | `float32` | 接收 RSSI，dBm |
| `rssi_valid` | `bool` | RSSI 是否有效；未知时为 false，两项 RSSI 为 NaN |
| `pair_measurement_freshness_verified` | `bool` | 是否有证据确认这条边是新的逐对测距；当前实现始终 false |

这些 `uint8` 节点 ID 是**注册表逻辑 ID**，不是把设备 ID 截断成 8 位。注册表按 `system_id / operating_mode / raw_role / raw_id` 匹配，保留 DR_MODE1 的完整 32 位设备 ID，并区分 LP 模式下同号不同角色。多网络消费者还需要保存配置中的 `network_instance`；该值不在消息字段中。

### 时间依据与三个时间戳

| `time_basis` 常量 | 值 | 当前含义 |
|---|---|---|
| `TIME_UNKNOWN` | 0 | 源帧没有有效 ROS 时间映射；`received_stamp` 仍可用于接收分析 |
| `TIME_FRAME_LOCAL_MAPPED` | 1 | 本地源帧 local time 已映射到 ROS 时间域 |
| `TIME_FRAME_SYSTEM_MAPPED` | 2 | 远端 system time 在显式确认共享时钟等条件下完成映射 |
| `TIME_RECEIVE_ONLY` | 3 | 接口预留的仅接收时间语义；当前 bridge 不主动输出此值 |

例如，一条远端边先被测距设备写入源帧，再由 MCU 分包发送，最后到达 N0。`source_frame_stamp` 对应源帧，`received_stamp` 对应 N0 接收，`batch.header.stamp` 对应 bridge 输出。三者不能互换；即使有源帧映射，也不是逐条边的硬件 TWR 时间戳。

启动后的映射预热、设备重启或时钟跳变期间，源时间可能保持未知。默认 `shared_system_clock=false`，不将不同设备的 local time 强行对齐。不要为了接入算法而用接收时间填补未知源时间。

### 一条例子的读法

假设某个 batch 的 `trigger_type=1`，其中一条观测为：

```yaml
kind: 1
reporter_id: 2
node_a: 2
node_b: 7
source_frame_seq: 123
source_frame_seq_valid: true
tx_cycle_seq: 9
shard_index: 0
shard_count: 2
time_basis: 0
source_frame_stamp: {sec: 0, nanosec: 0}
distance_m: 4.25
pair_measurement_freshness_verified: false
```

这是**示意字段片段，不是实测记录或可直接发布的完整消息**：固定节点 N2 报告到 N7 的距离为 4.25 m，来自源帧 123、发送轮次 9 的第一个分包。它不说明第二个包已收到，也不说明该距离具有已同步的测量时间或已验证的逐对新鲜度。

## 3. 构建、启动与查看

本机的 `data-run` 是数据盘执行启动器，用于将运行产物限制在 `/home/dev/ws_iap`；不是 ROS 命令。构建步骤和安装依赖详见[仓库 README](../../README.md)。

```bash
cd /home/dev/code/ws_tunnel/src/nlink_parser_uwb
data-run --check
data-run -- tools/build_test.sh
```

先按真实设备修改[注册表](../liuo_uwb_bridge/config/registry.yaml)和[参数文件](../liuo_uwb_bridge/config/uwb.yaml)。示例身份和 `unconfirmed` 模式不代表已检测到真实设备。再启动驱动与 bridge：

```bash
data-run -- bash -c 'source /opt/ros/jazzy/setup.bash; source /home/dev/ws_iap/uwb_ros2/install/setup.bash; ros2 launch liuo_uwb_bridge robot.launch.py port:=/dev/ttyUSB0 baud:=921600 namespace:=/uwb/n0 use_sim_time:=false'
```

在另一个终端进入受保护的 ROS 环境，后续命令都在这个 shell 内运行：

```bash
data-run -- bash
source /opt/ros/jazzy/setup.bash
source /home/dev/ws_iap/uwb_ros2/install/setup.bash

ros2 interface show liuo_uwb_msgs/msg/UwbObservationBatch
ros2 interface show liuo_uwb_msgs/msg/UwbObservation
ros2 topic info /liuo/uwb/observations --verbose
ros2 topic echo /liuo/uwb/observations \
  liuo_uwb_msgs/msg/UwbObservationBatch --qos-reliability best_effort
```

没有数据时，检查串口、设备输出帧类型、registry 身份及 `/liuo/uwb/diagnostics`。默认测距/数传帧为 3/0，可配置为 2/0 或 5/6 等受支持组合。仅构建消息包或启动消费者不会产生观测。

## 4. Python 订阅示例

在上面的受保护 ROS shell 中直接运行下面的代码。它逐条打印距离和时间依据，不把未知时间转换为测量时间；这里使用 best-effort QoS 匹配默认发布者。可靠订阅者不能匹配 best-effort 发布者。

```bash
python3 - <<'PY'
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from liuo_uwb_msgs.msg import UwbObservation, UwbObservationBatch

class Reader(Node):
    def __init__(self):
        super().__init__('uwb_observation_reader')
        qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = self.create_subscription(
            UwbObservationBatch, '/liuo/uwb/observations', self.on_batch, qos)

    def on_batch(self, batch):
        for obs in batch.observations:
            mapped = obs.time_basis in (
                UwbObservation.TIME_FRAME_LOCAL_MAPPED,
                UwbObservation.TIME_FRAME_SYSTEM_MAPPED,
            )
            source_time = (
                f'{obs.source_frame_stamp.sec}.{obs.source_frame_stamp.nanosec:09d}'
                if mapped else 'unknown'
            )
            self.get_logger().info(
                f'dataset={batch.driver_session_id} batch={batch.batch_seq} '
                f'source_session={obs.source_session_id} '
                f'edge={obs.node_a}->{obs.node_b} reporter={obs.reporter_id} '
                f'distance={obs.distance_m:.3f}m source_time={source_time} '
                f'time_basis={obs.time_basis} '
                f'freshness_verified={obs.pair_measurement_freshness_verified}')

rclpy.init()
node = Reader()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
PY
```

项目化使用时，在消费者的 `package.xml` 中加入 `<depend>liuo_uwb_msgs</depend>`；Python 节点还需要 `rclpy`。C++ 节点包含 `liuo_uwb_msgs/msg/uwb_observation_batch.hpp`，在 CMake 中 `find_package(liuo_uwb_msgs REQUIRED)`，并将 `liuo_uwb_msgs` 加入目标依赖。

消费者若要跨重启持久化观测身份，应同时保存网络配置、`driver_session_id`、`source_session_id`、reporter、有效源帧序号及端点。不要只用距离值或 `tx_cycle_seq` 去重，也不要把序号跳号直接解释成无线丢包。

## 5. 录包、回放与接入边界

使用仓库工具同时记录原始帧、stamped 帧、观测和诊断：

```bash
cd /home/dev/code/ws_tunnel/src/nlink_parser_uwb
data-run -- tools/record.sh /home/dev/ws_iap/uwb_ros2/my_observation_bag
```

回放有两种用途：直接播放已录制的 `/liuo/uwb/observations` 给消费者，或者播放 stamped 帧给 `replay_mode=true` 的 bridge 重新生成观测。不要同时播放旧观测又让 bridge 生成同一批观测。完整命令、QoS 文件及重复/seek 前的 `reset_replay` 操作见[主 README 的录包与回放](../../README.md#录包与回放)。

此消息不能直接替代相邻 LIUO 的 `UwbRange`：后者需要有效测量时间及 `variance_m2`，而本消息没有虚构方差。接入优化器前需明确时间依据接受策略、噪声标定和双向边相关性，详见 [LIUO 接入合同](../../docs/liuo_integration_contract.md)。

## 6. 带来的效果与已验证范围

| 效果 | 具体变化 |
|---|---|
| 下游接口统一 | 消费者通过一个话题处理本地机器人边和远端固定边，不必自行解码多种 LinkTrack 帧或 Base64 envelope |
| 来源可追踪 | 每条边携带报告者、源会话、源帧、发送轮次及分包信息，可区分重发与新源帧 |
| 时间含义明确 | 分开源帧时间、接收时间和发布时刻，未知状态显式保留，避免伪同步 |
| 动态流即时输出 | 本地流不等待固定端 5 Hz 轮次，远端缺包不阻塞已收到的边 |
| 便于复现故障 | stamped 接收上下文、会话和回放策略支持从原始输入重新生成观测 |
| 保留原始数据用途 | 距离保持源值，RSSI 及有效性明确；不添加 Anchor 地图、NLOS 猜测或滤波后的伪原始距离 |

已有软件验收中，60 秒 PTY 负载包含九条本地边**各 50 Hz**和三个发送者的 24 条远端有向边**各 5 Hz**，并覆盖慢订阅者。read 完成到 bridge 回调的峰值延迟约 0.583 ms；它不是射频延迟、真实测量年龄或完整消费者延迟。详细证据见[软件验收报告](../../docs/software_acceptance.md)。

这些结果验证软件处理能力，不证明真实 UWB 测距频率、无线透传成功率或定位精度提高。板级固件、真实无线、长时间实机运行及 LIUO 融合仍待[硬件验收](../../docs/hardware_acceptance.md)。ROS1 原消息字段保留，但现交付为 ROS2；设备 `local_time` 已恢复毫秒语义，迁移消费者时须按[迁移说明](../../docs/ros2_migration.md)调整。
