# UWB ROS1 驱动与固定边转发

当前分支 `feat/uwb-ros1-relay-v1` 面向 Ubuntu 20.04 / ROS Noetic / Python 3.8 / C++17。从 ROS2 提交 `a13dc6dc26ff7fed11272c2ec9a9f29020805b9a` 移植；软件验证、已知限制及证据见 [ROS1 验收报告](docs/ros1_software_acceptance.md)。真实无线、MCU 板级和 LIUO 融合未验收。

`nlink_parser/linktrack` 唯一拥有串口，使用新版 framer、校验、接收 context 和重连逻辑；独立 `liuo_uwb_bridge/range_bridge` 订阅 stamped 输入。每个新本地帧立即发布 ROBOT_ANCHOR，每个远端有效分包立即发布 ANCHOR_ANCHOR。固定端 5 Hz 指完整轮次，不把机器人本地流节流为 5 Hz。

## 构建与测试

当前工作区 `/home/mint/ws_calib` 已有其他工程。本任务使用专用 catkin profile，命令仅构建四包及其依赖，不清理旧 build/devel。

```bash
cd /home/mint/ws_calib/src/nlink_parse_driver
bash --noprofile --norc tools/build_test.sh
source tools/ros1_env.sh
# 另建专用 master，全部 PTY 合成输入；默认包含60秒负载。
tools/run_ros1_acceptance.sh
# 无 ROS 的解析/分配失败测试（ASan/UBSan）
tools/test_nlink_core.sh
# 无 ROS 的 MCU、协议黄金向量和2000组跨语言检查
cmake -S firmware/anchor_relay -B /home/mint/ws_calib/evidence/uwb_ros1/mcu-user -DCMAKE_BUILD_TYPE=Release
cmake --build /home/mint/ws_calib/evidence/uwb_ros1/mcu-user -j4
(cd /home/mint/ws_calib/evidence/uwb_ros1/mcu-user && ctest --output-on-failure)
```

安装验证采用第二个隔离 profile，不依赖旧 devel：

```bash
source /opt/ros/noetic/setup.bash
catkin config --workspace /home/mint/ws_calib --profile uwb_ros1_install \
  --extend /opt/ros/noetic --build-space build_uwb_ros1_install \
  --devel-space devel_uwb_ros1_install --install-space install_uwb_ros1 \
  --log-space logs_uwb_ros1_install --install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DPYTHON_EXECUTABLE=/usr/bin/python3
catkin build --workspace /home/mint/ws_calib --profile uwb_ros1_install \
  nlink_parser liuo_uwb_msgs liuo_uwb_protocol liuo_uwb_bridge -j4 -p2 --no-status
# 在新的干净 shell 中加载 install，而非旧工作区 devel：
source /opt/ros/noetic/setup.bash
source /home/mint/ws_calib/install_uwb_ros1/setup.bash
rosmsg show liuo_uwb_msgs/UwbObservationBatch
python3 -c 'from liuo_uwb_msgs.msg import UwbObservationBatch; print(UwbObservationBatch._md5sum)'
```

依赖为 catkin、roscpp、rospy、std_msgs、diagnostic_msgs、std_srvs、message_generation/message_runtime、rosbag、roslaunch、yaml-cpp，测试还使用 Python yaml、rosgraph_msgs。串口库 vendored，普通 CMake target 编译，禁止自动更新依赖。`extern/serial/CATKIN_IGNORE` 防止嵌套包发现。

## 启动与诊断

```bash
source /home/mint/ws_calib/src/nlink_parse_driver/tools/ros1_env.sh
roslaunch liuo_uwb_bridge robot.launch port:=/dev/ttyACM0 baud:=921600
# 其他终端加载同一环境后：
rostopic echo -n 1 /liuo/uwb/observations
rostopic echo -n 1 /liuo/uwb/diagnostics
rostopic echo -n 1 /uwb/n0/nlink_diagnostics
rostopic hz /uwb/n0/nlink_linktrack_nodeframe3_stamped
rosrun liuo_uwb_bridge observation_reader.py
```

`rostopic hz` 仅检查帧话题，不代替逐边频率验收。主话题类型为 `liuo_uwb_msgs/UwbObservationBatch`，同时含两种 kind，字段见 [消息说明](packages/liuo_uwb_msgs/README.md) 和 [LIUO 合同](docs/liuo_integration_contract.md)。

默认仅有五个业务发布话题：

```text
/uwb/n0/nlink_linktrack_nodeframe3_stamped
/uwb/n0/nlink_linktrack_nodeframe0_stamped
/uwb/n0/nlink_diagnostics
/liuo/uwb/observations
/liuo/uwb/diagnostics
```

`/rosout`、`/rosout_agg`、按需 `/clock` 是系统话题。range_frame 支持 2/3/5，data_frame 支持 0/6。配置 [uwb.yaml](packages/liuo_uwb_bridge/config/uwb.yaml) 为平面 YAML，分别加载到两个节点私有空间；修改后重启。要恢复全部9种原始与5种 stamped 输出，把配置副本中的 `publish_debug_topics` 设为 true 并用 `config:=绝对路径` 启动。`enable_legacy_tx` 默认为 false；显式开启也保留100 B、队列和速率限制。

launch 支持 `port`、`baud`、`config`、`registry`、`namespace`、`use_sim_time`。显式 launch 参数在 YAML 后设置，优先级更高。`use_sim_time` 设置全局 `/use_sim_time`；串口、重连及诊断使用单调定时器，暂停仿真时不阻塞。

默认 registry 中 `(system_id=0, operating_mode=unconfirmed, raw_role=0, raw_id=19)` 映射到逻辑机器人 N0，raw12 是固定 N12；19 不再有冲突固定条目。其他固定条目是待核对示例。未知身份有诊断计数，不自动认领。mode=unconfirmed、capacity=0、硬件能力=false，不因串口有数据而声明无线转发通过。

## ROS1 录包与回放

```bash
# 默认五个业务话题；只录选中 stamped，不订阅关闭的 raw。
tools/record.sh /home/mint/ws_calib/evidence/uwb_ros1/session.bag
# 非默认选帧：record.sh OUTPUT.bag /uwb/n0 5 6
rosbag info /home/mint/ws_calib/evidence/uwb_ros1/session.bag
```

直接给消费者播放已有 observations 时，停止驱动与 bridge：

```bash
rosbag play /home/mint/ws_calib/evidence/uwb_ros1/session.bag \
  --wait-for-subscribers --delay 1 --topics /liuo/uwb/observations
```

从 stamped 重新生成观测时，只启动 replay bridge，不打开串口、不播放旧 observations：

```bash
roslaunch liuo_uwb_bridge replay.launch
# 另一终端，启动消费者后：
rosbag play /home/mint/ws_calib/evidence/uwb_ros1/session.bag \
  --clock --wait-for-subscribers --delay 1 --topics \
  /uwb/n0/nlink_linktrack_nodeframe3_stamped /uwb/n0/nlink_linktrack_nodeframe0_stamped
# 每次重播或seek之前显式重置；不要用无重置的 --loop。
rosservice call /liuo_uwb_bridge/reset_replay '{}'
rosbag play /home/mint/ws_calib/evidence/uwb_ros1/session.bag \
  --clock --start 1 --wait-for-subscribers --delay 1 --topics \
  /uwb/n0/nlink_linktrack_nodeframe3_stamped /uwb/n0/nlink_linktrack_nodeframe0_stamped
```

回放参数与录制的 registry/帧类型必须一致。空格暂停/继续；模拟时间倒退会使映射失效。记录的 received_stamp 不被回放当前时间覆盖，单调时间按记录差值使用。ROS2 bag 无法直接作为 ROS1 bag 读取。TCPROS 有界队列、非 latch，不承诺 ROS2 best_effort 等价语义。

## 文档边界

[需求书](docs/uwb_ros1_development_requirements.md)、[迁移说明](docs/ros1_migration.md)、[实施日志](docs/ros1_implementation_log.md) 为本分支依据。`docs/software_acceptance.md`、`ros2_*`、`docs/ros2_reference/` 是冻结 ROS2 资料，里面另一台机器路径和测试不算本次证据。`CMakeLists.ros1.txt`、`package.ros1.xml`、旧 `src/linktrack` 和旧 `docs/README.ros1.*` 仅历史参考；当前 ROS1 入口是 `src/ros1`，其他历史 AOA/TOF/IoT 节点不在活动构建内。
