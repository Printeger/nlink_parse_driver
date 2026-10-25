# 固定节点平台无关 relay

`Receiver` 接收本地 NLink 字节，使用主机相同结构验证器，适配 Frame2/3/5，按完整 role/uint32 ID 注册回调筛固定邻居；Frame0/6 只计数，不转发。`Relay` 保留最新帧和当前轮冻结帧，无长期地图。

`Hal` 由板级适配提供单调毫秒和非阻塞、有界 UART write，返回实际写入字节数。每轮 200 ms、每包≤5边、间隔≥25 ms、hold≤250 ms、到期排队≤50 ms。容量不足整轮报告 overload，不静默删边；短写不重传。启动 phase 默认 id/boot hash，抖动≤2 ms。没有可验证数传预算时，生产实例拒绝启动；host 模拟显式 `simulator=true`。

板型未确认，未提供引脚、DMA、RTOS、烧录工程。调用者负责从持久计数器/足够熵源提供非零 boot_id，避免设备重启复用旧会话；检测 UWB 时间重置时当前实例递增 boot。32 位会话号不是全球唯一标识，部署仍需持久会话管理及旧会话拒绝。所有 RX 与 tick 在同一任务串行调用，不在 ISR 直接运行。

```bash
data-run -- cmake -S firmware/anchor_relay -B /home/dev/ws_iap/uwb_ros2/mcu-build -DCMAKE_BUILD_TYPE=Debug
data-run -- cmake --build /home/dev/ws_iap/uwb_ros2/mcu-build -j2
data-run -- ctest --test-dir /home/dev/ws_iap/uwb_ros2/mcu-build --output-on-failure
```

这些是软件模拟，不证明 C1 固定边来源、C2 同时无线数传或 C5 100 B 透传能力。
