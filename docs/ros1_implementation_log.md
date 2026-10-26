# ROS1 移植实施记录

2026-10-26，工作区 `/home/mint/ws_calib`，仓库 `src/nlink_parse_driver`。

D0：启动 HEAD/ROS2 分支均为 `a13dc6dc26ff7fed11272c2ec9a9f29020805b9a`，工作树干净。远端 origin 为 `git@github.com:Printeger/nlink_parse_driver.git`，远端无 ROS1 同名分支。原地创建 `feat/uwb-ros1-relay-v1`。未发现适用 AGENTS.md。任务两份外部文档原样复制，文档顶部“仅修订文档”的历史说明由本轮明确开发授权取代。启动状态与相邻文件 SHA256 在 `evidence/uwb_ros1/startup.json`。

主机 Ubuntu20.04.6、Noetic、Python3.8.10、GCC9.4、CMake3.16.3。专用 catkin profiles `uwb_ros1` 和 `uwb_ros1_install`，只构建四包及 catkin prebuild，跳过相邻38包；默认旧 profile/构建目录未清理。测试 master 为 `http://127.0.0.1:11381`，只用 PTY。

D1–D4：四包原生 catkin、roscpp 适配，保持 framer 路径与 standalone 引用。私有 NodeHandle 参数，串行 spin，SteadyTimer，TCPROS 100/诊断10非latch。独立干净 install profile 已构建。协议库 catkin 导出最初置于 target 之后导致 devel 库路径错误，修正为 target 前初始化 catkin；失败日志保留。

D5：保留全部原始164字段断言，ROS1 Header.seq 单独归一化。PTY 故障、模式组合和话题选择测试通过。旧固定15ms检查在主机负载下偶发抢先断言，改为有界等待目标计数，不更改观测结果要求。bag 首次未等待连接丢开头帧，改为 `--wait-for-subscribers --delay 1`，两次完整回放与 seek 验证通过。

验收状态以 `ros1_software_acceptance.md` 的最终逐项证据为准；本日志不将未执行项视为通过。所有输入为合成数据，不是无线验收。

最终补充：暂停ROS时间使映射失效的回归已通过；测试参数残留用专用master内清理本任务节点私有参数隔离。最终双路bag覆盖40条原观测、两次重播与seek/reset、/clock暂停恢复。`final-suite-r2`全部通过，slow消费者实际301批，34,200条逐边身份完整。独立空install构建及安装消费者/launch/录包通过。ASan/UBSan关闭LeakSanitizer退出扫描后通过，不宣称泄漏检查通过。相邻3017文件哈希和ROS2引用一致。最终证据及交付核验见验收报告。

交付审核补充：原生NodeHandle参数类型不匹配会回退默认值，现改为明确抛错退出。新增各节点/发布器专项参数类型回归，正常话题和最终安装CLI仍通过。最终提交及远端SHA由delivery.json核验。
