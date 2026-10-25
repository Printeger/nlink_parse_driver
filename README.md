# UWB ROS2 驱动与 Range Relay v1

目标平台 Ubuntu 24.04 / ROS 2 Jazzy。单串口所有者 `nlink_parser/linktrack` 发布原始 LinkTrack 帧及接收 context，独立 `liuo_uwb_bridge/range_bridge` 即时输出 `/liuo/uwb/observations`。平台无关 MCU 逻辑每200ms冻结单一源帧并发完整分包轮次。

软件验收状态与证据见 [验收报告](docs/software_acceptance.md)。**没有实机无线、板级烧录或 LIUO 融合验收**；模拟≥20Hz不证明真实TWR频率。默认硬件能力false、模式unconfirmed、capacity=0、legacy TX与降级关闭。

规范：[handbook](docs/uwb_driver_design_notebook.md)、[ROS2需求](docs/uwb_ros2_development_requirements.md)。接口兼容性见 [ROS2迁移说明](docs/ros2_migration.md)，时间/会话与消费者要求见 [LIUO接入合同](docs/liuo_integration_contract.md)，MCU见 [HAL与模拟说明](firmware/anchor_relay/README.md)。原ROS1说明保留在docs/README.ros1.*，不能按其中catkin/submodule命令构建当前版本。

## 构建与测试

依赖已安装的 Jazzy、colcon、CMake、C/C++编译器、yaml-cpp、Python3/rclpy/rosbag2。只扫描本仓库packages，不扫描相邻ROS1项目。全部产物在数据盘：

```bash
cd /home/dev/code/ws_tunnel/src/nlink_parser_uwb
data-run --check
data-run -- tools/build_test.sh
data-run -- tools/test_nlink_core.sh
data-run -- cmake -S firmware/anchor_relay -B /home/dev/ws_iap/uwb_ros2/mcu-build -DCMAKE_BUILD_TYPE=Debug
data-run -- cmake --build /home/dev/ws_iap/uwb_ros2/mcu-build -j2
data-run -- ctest --test-dir /home/dev/ws_iap/uwb_ros2/mcu-build --output-on-failure
```

完整PTY软件验收（新的输出目录；不能覆盖活动记录）：

```bash
data-run -- bash -c 'source /opt/ros/jazzy/setup.bash; source /home/dev/ws_iap/uwb_ros2/install/setup.bash; python3 tools/pty_driver_test.py --output /home/dev/ws_iap/uwb_ros2/recheck-driver'
data-run -- bash -c 'source /opt/ros/jazzy/setup.bash; source /home/dev/ws_iap/uwb_ros2/install/setup.bash; python3 tools/pty_bridge_test.py --output /home/dev/ws_iap/uwb_ros2/recheck-integration --load-seconds 60'
data-run -- bash -c 'source /opt/ros/jazzy/setup.bash; source /home/dev/ws_iap/uwb_ros2/install/setup.bash; python3 tools/state_integration_test.py --output /home/dev/ws_iap/uwb_ros2/recheck-state'
data-run -- bash -c 'source /opt/ros/jazzy/setup.bash; source /home/dev/ws_iap/uwb_ros2/install/setup.bash; python3 tools/bag_smoke.py --output /home/dev/ws_iap/uwb_ros2/recheck-bag'
```

测试使用同一ROS域，依次执行，测试期间停止其他同名驱动。工具显式生成模拟串口输入，不标为实测。

## 启动与配置

先按真实设备身份编辑独立registry.yaml；示例只是逻辑N0–N19与raw role=0/id相同的未确认配置，不会自动检测模式或修改设备。键为system_id/operating_mode/raw_role/raw_id，DR_MODE1保留32位ID，LP同号异角色必须分开。配置文件为合法ROS2 `ros__parameters`；能力标志只有独立实测证据支持后才能手工开启。

```bash
data-run -- bash -c 'source /opt/ros/jazzy/setup.bash; source /home/dev/ws_iap/uwb_ros2/install/setup.bash; ros2 launch liuo_uwb_bridge robot.launch.py port:=/dev/ttyUSB0 baud:=921600 namespace:=/uwb/n0 use_sim_time:=false'
```

launch还提供`config:=/绝对路径/uwb.yaml`与`registry:=/绝对路径/registry.yaml`。选择range_frame=2/3/5及data_frame=0/6，默认3/0，不能同时开所有测距流制造重复。设备local_time现为毫秒，不是导入版本写入的Unix微秒。

## 录包与回放

```bash
data-run -- tools/record.sh /home/dev/ws_iap/uwb_ros2/my_new_bag
# 单独启动回放bridge；registry必须与录包数据集一致，不启动串口驱动。
data-run -- bash -c 'source /opt/ros/jazzy/setup.bash; source /home/dev/ws_iap/uwb_ros2/install/setup.bash; ros2 run liuo_uwb_bridge range_bridge --ros-args -p registry_path:=/home/dev/code/ws_tunnel/src/nlink_parser_uwb/packages/liuo_uwb_bridge/config/registry.yaml -p replay_mode:=true'
data-run -- bash -c 'source /opt/ros/jazzy/setup.bash; source /home/dev/ws_iap/uwb_ros2/install/setup.bash; ros2 bag play /home/dev/ws_iap/uwb_ros2/my_new_bag --topics /uwb/n0/nlink_linktrack_nodeframe3_stamped /uwb/n0/nlink_linktrack_nodeframe0_stamped --qos-profile-overrides-path /home/dev/code/ws_tunnel/src/nlink_parser_uwb/packages/liuo_uwb_bridge/config/bag_qos.yaml'
# 每次重复/seek前开启新的回放数据集：
data-run -- bash -c 'source /opt/ros/jazzy/setup.bash; ros2 service call /liuo_uwb_bridge/reset_replay std_srvs/srv/Trigger "{}"'
```

原始/stamped/观测默认best_effort/volatile/keep_last100，诊断reliable/volatile/keep_last10。可靠订阅者不能匹配best_effort发布者；录包override已提供。记录时同时保存配置、registry、Git SHA、设备/固件/模式以及真实MCU发送日志。主机不能独立测出无线丢包率或真实逐对TWR频率，相应诊断为unknown。

不push，不修改相邻LIUO优化器。软件缓存不构建长期Anchor图，不提供虚构方差、NLOS判定、自动多跳或Wi-Fi/Ethernet通路。
