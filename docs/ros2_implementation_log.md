# ROS2 实施记录

## D0：2026-10-26（Asia/Shanghai）

- 起点 HEAD：47da44a；导入快照：5680f766f9b11a8426795f0589b1ccbfe9432af8。开始时工作区干净，无 remote。
- 分支：feat/uwb-ros2-relay-v1。只修改本仓库；LIUO 消费端不在范围。
- 实际环境：Ubuntu 24.04 LTS，/opt/ros/jazzy；data-run --check 成功，Landlock ABI 7，系统盘写入被拒绝。
- 输出根：/home/dev/ws_iap/uwb_ros2；所有构建、测试、模拟、分析经 data-run；仅 packages/ 为 colcon 发现根。
- git mv 将原 ROS 包移至 packages/nlink_parser，仓库根不再有 package.xml。四包结构将逐步实现。
- extern/serial、src/utils/nlink_unpack、protocol_extracter 是 vendored 实文件；删除误导性的 .gitmodules，保留全部来源代码及 LICENSE。不更新依赖。
- 来源 URL：github.com/wjwwood/serial、github.com/nooploop-dev/nlink_unpack、github.com/nooploop-dev/protocol_extracter。手册列出的 upstream SHA 未与本地逐字节核验，不能宣称相同。
- ROS2 迁移以需求书覆盖手册 ROS1 构建路线；Frame3 保留现有 Header/uint64 local_time。其他帧恢复设备毫秒语义，不再写入主机 Unix 微秒。
- 协议差异：手册 §5.4 已记录 Frame3 示例 reserved 行冲突；采用正式格式及 vendored 代码的 22+7n，不改 wire format。
- 验证接缝遵循用户明确授权：独立协议/MCU 公共接口，以及原始字节/PTY → 公开 ROS2 输出；不为测试内部 getter 扩展接缝。

## 状态

D0 布局准备完成；D1–D7 尚未验收。硬件 C1–C5、H0–H5、板级烧录及 LIUO 接入均未执行。
