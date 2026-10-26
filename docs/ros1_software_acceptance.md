# ROS1 软件验收报告

日期：2026-10-26（Asia/Shanghai）。当前仓库 `/home/mint/ws_calib/src/nlink_parse_driver`，分支 `feat/uwb-ros1-relay-v1`。**必需软件验证通过，真实无线/板级/LIUO 融合未完成。** 最终提交及远端同名分支 SHA 核验保存于 [delivery.json](/home/mint/ws_calib/evidence/uwb_ros1/delivery.json)，与本报告一起用于判断 Git 交付完成；不能以软件测试替代推送核验。

固定基线 `a13dc6dc26ff7fed11272c2ec9a9f29020805b9a`。主要运行时代码提交 `bb7a550c3287dd1911788ae44abdc573678c17a6`；参数类型严格拒绝补丁为 `2e4937b`，通过 `strict-params.log`、`strict-topics.log`、`strict-install-build.log` 与 `strict-install-cli.log`。最终测试提交 `6fd1efd`（之后补充参数类型回归），最终文档提交不改变运行行为。完整源文件哈希、执行时版本差异说明见 [tested-source.json](/home/mint/ws_calib/evidence/uwb_ros1/tested-source.json)。没有 ROS2 双版本运行环境；对照使用冻结字节夹具和独立协议参考，不宣称运行过双版本对比。

## 环境与证据

Ubuntu20.04.6、ROS Noetic、Python3.8.10、GCC9.4、CMake3.16.3；实际依赖版本见 [environment.json](/home/mint/ws_calib/evidence/uwb_ros1/environment.json)。专用 profile `uwb_ros1` 生成 `build_uwb_ros1/devel_uwb_ros1`；install 最终从空目录在 `uwb_ros1_clean` profile 的 `build_uwb_ros1_clean/devel_uwb_ros1_clean/install_uwb_ros1/logs_uwb_ros1_clean` 构建。之前的 install 验证产物保留在 evidence，不用于最终发现。

以下证据根为 `/home/mint/ws_calib/evidence/uwb_ros1/`，全部本次执行，未引用旧机器报告作为通过依据。完整命令入口在 [README](../README.md)、[build_test.sh](../tools/build_test.sh)、[run_ros1_acceptance.sh](../tools/run_ros1_acceptance.sh)。集成测试 master 为独立 URI，完整套件自行选择空闲端口并清理自己的 master，其他定向测试为 `http://127.0.0.1:11381`。只使用 PTY 合成数据，无实体串口访问；构建产物、日志与 bag 不入 Git。

## S0–S11 核对

| 项目 | 状态 | 本轮证据与覆盖 |
|---|---|---|
| S0 范围/分支 | 通过 | `startup.json`、`protection.json`：精确基线、干净启动、原地新分支；相邻3017文件无增加/修改/删除；ROS2引用不变 |
| S1 构建/安装 | 通过 | `final-build.log`、`clean-install-final.log`：四包干净并行生成C++/Python消息；`install-consumer.log` 独立C++消费协议库与生成头、Python导入；`install-runtime.log` 安装launch实际运行；38个相邻包跳过 |
| S2 协议/解析 | 通过 | `core-final.log`：9类夹具、1785拆点、坏长度/block/checksum、1MB垃圾及分配失败恢复；`mcu-sanitizer-final.log`：黄金、2000组跨语言往返、非法结构；`final-suite-r2/pty_driver_test.log`：164原字段断言与公开raw/stamped输出 |
| S3 两类观测 | 通过 | `final-suite-r2/pty_bridge_test/report.json`：Frame3/0、Frame2/0、Frame5/6、LP同号异角色、DR_MODE1完整32位ID、端点/距离/两种kind |
| S4 registry | 通过 | `final-suite-r2/topic_visibility_test.log`：raw19→N0、N0→N12与N12远端报告；`ros1_contract_test/report.json`：错误映射unknown_receiver、重复raw/逻辑、缺N0、fixed冲突及非法帧选择拒绝 |
| S5 话题可见性 | 通过 | `topic_visibility_test.log`：全部5业务话题精确集合，未选夹具输入不新增，3种选择与覆盖namespace；`pty_driver_test.log` debug恢复全部9种raw/5种stamped，字段一致（仅排除ROS1 Header.seq） |
| S6 故障/资源 | 通过 | `pty_bridge_test/report.json` faults、`state_integration_test/report.json`：发送者隔离、部分分包、重复/旧源、boot/端口会话、PTY重连残片、回绕/重启、16键缓存、64 tombstone耗尽；`state-standalone.log`：10s退休水位、100000序号容量、乱序与时钟 |
| S7 ROS1 差异 | 通过 | `topic_visibility_test.log`、`ros1_contract_test/report.json`：私有参数/namespace/baud覆盖、零时间/暂停/前后跳仍读串口与发诊断；`state_integration_test/report.json`：Header.seq固定为7不影响协议去重，跨话题乱序及映射失效 |
| S8 60秒负载 | 通过 | `final-suite-r2/pty_bridge_test/report.json` load、`load-summary.json`：每条边精确对照发送身份，6个10秒窗口、独立慢订阅者、队列/内存/延迟；详细结果如下 |
| S9 rosbag | 通过 | `final-suite-r2/bag_smoke/report.json`、`install-bag-both/report.json`：实录20测距+20数传stamped、40原observations；直接播放、两次重播、seek/reset、/clock暂停恢复、诊断不阻塞、接收时间保留/新数据集、唯一观测发布者 |
| S10 standalone/MCU | 通过 | `core-final.log`、`state-standalone.log`、`mcu-release.log`、`mcu-sanitizer-final.log`：无ROS编译framer/state/codec/relay；8边每秒10包和18边20包，完整5Hz轮次，冻结源/过期/迟到/过载/短写/无回环 |
| S11 文档/命令/Git | 通过（最终SHA见交付记录） | `cli-smoke/report.json`：已安装reader、echo/hz、33种消息定义/MD5；README、迁移、合同、任务副本已交付；最终push与SHA核验在 `delivery.json`、`push.log` |

D0–D7 对应上述矩阵：D0→S0，D1→S1，D2→S2/S5，D3→S3/S4/S6，D4→S7，D5→S8/S9，D6→S10及文档，D7→S11和最终保护核验。当前两份任务文档与 `/home/mint/ws_calib/docs/` 外部原件逐字节一致。

## 负载结果

测试前固定9条本地边N0→N1…N9；3个远端发送者N1/N2/N3各报告其余8条固定有向边，合计24边。每轮两包5+3，双向报告分别计数。启动连接完成后才开始60秒，尾部只排空、不补发；按发送时间分配10秒子窗口，并逐一匹配观测来源身份。

| 指标 | 实测软件结果 |
|---|---|
| 发送 | 3000本地帧；1800远端分包；每发送者300完整轮次 |
| 本地逐边 | 每条3000/3000，50.000Hz；六个10s窗口各50.000Hz |
| 远端逐边 | 每条300/300，5.000Hz；六个10s窗口各5.000Hz |
| 总观测身份 | 34200/34200，零重复，无未预定义边 |
| 慢消费者 | 独立进程，每回调sleep200ms，queue2，实际收到301批；快消费者与串口仍完整 |
| 串口 | 445800B、4800成功帧、RX峰116B，overflow=0、unpack错误=0 |
| read完成→bridge回调峰值 | 33.442267ms；主机处理入口延迟，不是无线/真实TWR年龄或消费者端到端延迟 |
| 驱动进程树RSS | 首19916KiB，末/峰19928KiB |
| bridge进程树RSS | 首20580KiB，末/峰20608KiB |

不能以此证明真实无线5Hz、硬件TWR新鲜性或数小时无泄漏。源设备时间与ROS映射只是帧级估计，未知不确定度NaN、freshness=false；DR_MODE1不使用system_time同步。

## 失败记录与修正

- 首轮catkin协议库导出时序错误，修正find_package/catkin_package在创建target之前；失败在 `build-first.log`，修正与干净构建证据保留。
- 固定15ms测试等待造成抢先断言，改为有界等待期望计数，未改变结果要求；`state-test.log` 与后续通过报告均保留。
- 首次bag未等待订阅连接丢开头两条，工具统一加 `--wait-for-subscribers --delay 1`，后续双路/安装回放全部通过。
- 同一测试master残留私有namespace，测试Harness在创建节点前清理本任务节点参数；`final-suite`失败保留，`final-suite-r2`整套通过。此前 `load-final` 没收到慢订阅消息不能作为通过证据；最终套件已实际记录301批。
- MCU默认LeakSanitizer退出检查在当前环境卡住；显式 `ASAN_OPTIONS=detect_leaks=0:verify_asan_link_order=0 UBSAN_OPTIONS=halt_on_error=1` 后ASan/UBSan完整通过。没有宣称LeakSanitizer通过。正常Release独立测试也通过。

最终审核增加错误参数类型拒绝：`range_frame`、`baud_rate`、bridge去重容量使用非预期类型时明确启动失败，不回退默认值。专项测试与正常选帧/话题回归均通过，最终devel/install已重编译；数据处理、去重与wire算法未改动，不重复运行无关负载。

## 未执行的系统项目

真实C1固定端本地距离、C2同时测距/数传、C5的100B无线透传、型号/模式/容量核实、单节点/全节点广播、隧道/长期动态逐边频率、MCU HAL与板级烧录、数小时实机、LIUO方差/时间/双向相关性策略及优化器融合均未执行。默认硬件capability=false保持不变，未修改相邻LIUO或其他工程。详细硬件清单沿用 [hardware_acceptance.md](hardware_acceptance.md)，不将ROS2或本次PTY模拟称为无线验收。
