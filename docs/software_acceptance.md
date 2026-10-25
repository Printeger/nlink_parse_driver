# UWB ROS2 软件验收报告

日期：2026-10-26（Asia/Shanghai）。结论：**软件完成；软件加真实硬件的完整系统未完成**。D0–D7必需软件项已实现，S1–S7/S9及S8有界状态验证通过。数小时实测、MCU板级、无线C1–C5/H0–H5、LIUO消费端接入未执行，详见[硬件清单](hardware_acceptance.md)。输入均明确为黄金向量、原仓库协议夹具或软件合成PTY流，没有射频实测数据。

测试代码提交：`ef3db9fac5e31836cb3319d0708c0db34ec4524f`；分支 `feat/uwb-ros2-relay-v1`。本报告及操作文档在后续仅文档提交交付。导入快照 `5680f766f9b11a8426795f0589b1ccbfe9432af8`，开发起点 `47da44a`；不声称导入快照等同手册上游SHA。开始时工作区干净，无remote；未改相邻仓库，未push。

## 环境与可复制入口

Ubuntu24.04、Jazzy；rclcpp28.1.16、rosbag2 0.26.9、colcon-core0.20.1、yaml-cpp0.8.0。证据：[environment.txt](/home/dev/ws_iap/uwb_ros2/evidence/environment.txt)、[storage-check.json](/home/dev/ws_iap/uwb_ros2/evidence/storage-check.json)。data-run验证Landlock ABI7，系统盘写入受限。

所有命令在仓库根执行，经`data-run -- ...`；build/install/log/test-results分别在`/home/dev/ws_iap/uwb_ros2/{build,install,log,test-results}`。只扫描packages/，发现恰好四个ament包：[final-packages.txt](/home/dev/ws_iap/uwb_ros2/evidence/final-packages.txt)。

[README](../README.md)给出完整可复制构建、测试、launch、录包、回放命令。主要入口：`data-run -- tools/build_test.sh`、`data-run -- tools/test_nlink_core.sh`、`tools/pty_driver_test.py`、`tools/pty_bridge_test.py --load-seconds 60`、`tools/state_integration_test.py`、`tools/bag_smoke.py`。Python ROS工具须先在受保护bash中source Jazzy与数据盘install；每次使用新的数据盘输出目录。

## 验收矩阵

| 项目 | 状态 | 实际验证与最终证据 |
|---|---|---|
| D0/S1布局 | 通过 | git mv多包根、四包发现；vendored来源/许可保留；无活动catkin target |
| D1/S2协议 | 通过 | 黄金72B/100B逐字节、CRC检查值、1–5边长度、2000固定种子Python→C++往返、非法字段/CRC/Base64/哨兵/排序/距离拒绝；[final-mcu-test.log](/home/dev/ws_iap/uwb_ros2/evidence/final-mcu-test.log)、[协议测试](../packages/liuo_uwb_protocol/tests/crosscheck.py) |
| D2/S1构建 | 通过 | 四包生成接口、编译、链接、安装；colcon test/test-result为3项0错误0失败（协议2、状态1）；其他PTY/MCU/sanitizer为独立测试，不冒充colcon计数；[final-build-test.log](/home/dev/ws_iap/uwb_ros2/evidence/final-build-test.log) |
| D3/S3提帧 | 通过 | 原九组LinkTrack夹具、1785拆点、粘帧、垃圾、坏长度/block/checksum、1MB垃圾上限、短头超时/旧前缀后新半帧、延迟恢复read上下文、分配失败恢复；实际vendor unpack经ASan/UBSan；[final-nlink-sanitizer.log](/home/dev/ws_iap/uwb_ros2/evidence/final-nlink-sanitizer.log) |
| D3/S3公开消息 | 通过 | PTY→原始/stamped；164个原字段断言、设备毫秒、原帧/包装一致、接收序号、Frame3各切点与坏帧恢复；[final-driver.log](/home/dev/ws_iap/uwb_ros2/evidence/final-driver.log) |
| D3 TX | 通过 | 默认禁写；显式启用可写；101B拒绝；[final-tx.log](/home/dev/ws_iap/uwb_ros2/evidence/final-tx.log) |
| D4/S4身份与去重 | 通过 | Frame0/6内层sender、reporter不符拒绝、独立碎片、32位ID、LP TAG0/ANCHOR0、同包/换轮旧源拒绝、同值新帧接受、双向、缺第二分包首包5边立即发、boot旧会话、uint32回绕；[final-integration/report.json](/home/dev/ws_iap/uwb_ros2/final-integration/report.json) |
| D5/S5时间/故障 | 通过 | 设备回绕/重启、ROS跳变、远端独立到达时共享映射失效、跨话题乱序、停更10秒频率归零、PTY重连清两层半帧、未知时间/freshness明确；[final-state.log](/home/dev/ws_iap/uwb_ros2/evidence/final-state.log)、[final-integration.log](/home/dev/ws_iap/uwb_ros2/evidence/final-integration.log) |
| D6/S6 MCU | 通过 | fake clock：8边每秒10包、18边20包，全部5Hz完整轮次；同轮源帧冻结、默认相位/有界抖动、过期/排队迟到/过载/短写/无回环；真实原夹具验证Frame2/3/5适配，255重复peer保护；ASan/UBSan；[final-mcu-test.log](/home/dev/ws_iap/uwb_ros2/evidence/final-mcu-test.log)、[模拟测试](../firmware/anchor_relay/tests/simulation.cpp) |
| D7/S7负载 | 通过 | 连续60s、50Hz本地+3发送者5Hz完整轮次、200ms慢订阅者；逐边及10s子窗口均记录；见下节 |
| S8有界状态 | 通过 | 100000序号容量测试、TTL退休后旧包拒绝、16键故障配置、64旧boot耗尽后拒绝、1MB垃圾、255重复peer；[final-state/report.json](/home/dev/ws_iap/uwb_ros2/final-state/report.json)、上述sanitizer报告；60s RSS采样 |
| S8数小时实测 | 未执行 | 不能用加速状态测试或60秒RSS替代数小时/真实设备验证 |
| S9操作 | 通过 | 安装launch实际baud=460800、QoS兼容、真实rosbag2录得20个stamped；两次回放接收时间逐一相同、dataset不同；[final-bag.log](/home/dev/ws_iap/uwb_ros2/evidence/final-bag.log)，保留[实际bag](/home/dev/ws_iap/uwb_ros2/final-bag/recorded) |

T01–T05由S3覆盖；T06–T13由S2/S4覆盖；T14–T16由S5/S7覆盖；T17由MCU无回环测试覆盖；T18有界状态部分通过，数小时部分明确未执行。

## 60秒软件负载结果

测试前冻结集合：本地N0→N1…N9，共9边；发送者N1/N2/N3分别报告N1…N9中除自身外8边，共24条有向固定边。每个发送者每轮两包5+3，双向报告不合并。发送日志累计3000本地帧、1800远端包；每发送者300完整轮次。没有用总话题频率或源序号跳号推定逐边频率/无线丢包。

最终结果原表：[final-integration/report.json](/home/dev/ws_iap/uwb_ros2/final-integration/report.json)；摘要：[final-load-summary.json](/home/dev/ws_iap/uwb_ros2/evidence/final-load-summary.json)。

| 指标 | 实际值 |
|---|---|
| 9条本地边逐条报告率 | 每条50.000Hz；6个10s子窗口均50.000Hz，超过20Hz目标 |
| 24条远端有向边逐条唯一报告率 | 每条5.000Hz；6个10s子窗口均5.000Hz |
| 完整轮次 | N1/N2/N3各300/300，缺分包估计0 |
| 驱动已解析帧/接收字节 | 4800帧 / 445800B；60s平均7430B/s |
| RX环形缓冲实际峰值/溢出 | 116B / 0；unpack错误0、坏字节拒绝0（负载阶段） |
| 接收read完成→bridge回调峰值 | 0.582812ms；这是主机处理入口延迟，不含无线、MCU保留、TWR或LIUO，非端到端测量年龄 |
| bridge最终包/观测缓存键数 | 192 / 768；各配置上限10000 |
| 驱动进程树RSS | 首58996KiB，末/峰59292KiB |
| bridge进程树RSS | 首58516KiB，末/峰58928KiB |
| 慢订阅者 | 独立订阅回调sleep200ms、depth2；上述快订阅链路仍达标 |

RSS包含ros2 CLI父进程与节点子进程，不能当成纯C++堆大小。短时初始化增长不等于长期泄漏结论；数小时仍未执行。诊断速率使用有界秒桶，尾部诊断窗口可能包含停流/不完整秒，正式频率采用预先冻结的60秒及10秒计数区间。

## 资源与失败边界

| 状态 | 软件上限/失效策略 |
|---|---|
| 串口read/帧/RX | read4096B（可配≤8192），frame8192，RX16384；每字节时间标记同容量；read context按仍在缓冲的read退休，至多容量+1个 |
| vendor结果 | 节点数组固定；数传每block224B，超限拒绝；分配失败返回0，不发布旧全局结果 |
| TX | 8包×100B，25ms最小间隔，50ms过期；断线清理，短写失败不重试 |
| 应用解流 | 已注册固定sender≤254，每sender100B残片，250ms期限；未知sender不建状态 |
| 去重 | 每sender包/观测各32序号记录、127回溯窗口；10s退休floor；每次包处理后全局各≤10000键，处理内最多暂增1包/5观测键，仍有固定硬界 |
| 会话 | 每sender64旧boot、64旧port；耗尽fail-closed并计数，不任意淘汰后接纳旧会话 |
| 轮次/统计 | 每sender最多32轮，1s后统计缺片；最多1024边×60秒桶，淘汰诊断边状态会计数，不删除实际观测 |
| 时钟 | 本地/系统各128样本；无映射stamp=0、不确定度NaN，freshness=false |
| MCU | 最新+冻结两份254边快照，当前一轮分包索引，无历史TX队列；容量/hold/gap不满足整轮报告失败 |

数据未知不会被补为零距离、高RSSI、零时间误差或虚构方差。时钟模型、序号窗口/重启策略、ROS1兼容行为见[迁移](ros2_migration.md)与[接入合同](liuo_integration_contract.md)。

## Standards审查

初审/复审合计5项正确性发现已关闭：MCU255记录越界、跨话题水位、零窗口容量、短头过期、新半帧被旧expiry误删。另有1项判断性维护建议：bridge ROS适配类职责集中，未作为硬违规或验收阻塞。最终只读复核ef3db9f未发现修复范围内新的确定缺陷。

## Spec审查

6项已关闭：MCU越界、跨话题丢帧/误重放重置、恢复帧read时间、停更统计、expiry误删新半帧、远端独立到达时ROS跳变映射失效。对应公共PTY/stamped或无ROS回归均保留。审查不能替代测试；最终通过依据上列实际执行记录。

两轴初始最严重问题均涉及MCU越界/观测丢失，现已修复。最终证据SHA-256及重复模拟bag退休记录见[manifest.json](/home/dev/ws_iap/uwb_ros2/evidence/manifest.json)；保留最终引用的bag，早期重复bag已删除并在原目录availability.json标记。失败构建、初始不通过测试及原因保留在evidence与[实施日志](ros2_implementation_log.md)，没有删除失败用例、放宽频率或隐藏unknown达成验收。

## Git与剩余事项

提交链：caa96a5（D0布局）、1310be7（D1协议）、505e49a（D2/D3驱动）、ace9066（D4/D5桥接）、540edc7（D6 MCU）、ef3db9f（D7审查修复与验收工具），之后仅文档提交。所有实现、配置、操作说明和本报告纳入本地Git；最终HEAD以交付回复列出的SHA为准。

剩余事项只有本报告明确未执行的长时/板级/无线/LIUO接入工作及非阻塞维护建议；不将缺硬件当作软件故障测试的豁免。C1未确认不能宣称真实固定边测距可用，C2/C5未确认不能宣称无线透传已通过。
