# 核实的接口和合同

2026-10-26，本次只读核查，未连接机器人。

## 源码版本与接入位置

| 来源 | 实际版本/观察 |
|---|---|
| `/data/ros2_ws/com.bxi.sonic-main` | HEAD `9623da11171f661e6e5dc51949a2ce8a8bdf7090`，工作树未修改 |
| `/data/ros2_ws/bxi_rl_controller_ros2_example-main` | HEAD `f1023778cbbbf586030e8ae77a43f485f5937e8e`，无该新 framework 结构 |
| `/data/ros2_ws/bxi_rl_controller_ros2_example-test01-underlay` | 无 Git 元数据，用 `patches/expected_sources.json` 的逐文件 SHA256 限定补丁版本 |

下列路径相对于 underlay 的 `src/bxi_example_py_elf3/`：

- `bxi_example_py_elf3/policies/normal.py`：`NormalMotionPolicyMjlab.step` 真正执行 backend；`raw_action * 模型metadata.action_scale + 模型metadata.default_joint_pos` 得到目标。`publish_output()`主要在初始化/reset调用，**不能用它当每次推理观察点**。
- `bxi_example_py_elf3/framework/runtime/controller.py`：`update` 处理状态机和过渡，是控制周期/切换标记的公共位置。
- `bxi_example_py_elf3/framework/platform/runtime.py`：`_run_control_cycle` 调用 framework 后交给平台发布。
- `bxi_example_py_elf3/bxi_example_demo.py`：`publish_motor_frame` 内先 `_motor_override.apply`，再组装 `ActuatorCmds` 并发布。最终指令必须取这里之后的 ROS 消息。补丁在发布成功后发出仅含源时间戳/模型序号的 `command_link`，不会再发控制指令。
- `mods/com.bxi.basic_actions/normal_run_state.py` 调用上述 Normal policy；公共框架未暴露所有模型原始输出的统一位置，首版用一个小模型观察点和公共周期/发布观察点，未声称一处接入覆盖所有模型。

SONIC 源码 `policy.py.infer`：`clip(raw_action,-20,20)`，再按该实例的 scale/default 得到 `candidate`，随后 `_blend_source_target` 得到 policy target。旧 STK1 中 `decoded_target` 是本工具的 `q_model`，不是混合后的 `policy_target`。原始/限幅 action 和 scale/default 在原记录保留，导入会核对重算结果。未把 SONIC 常量复制给 Normal。

## 关节和坐标

`src/elf3_eval/body29.json` 是独立可读合同。身体29名称与旧合同、underlay 的 `ELF3_POLICY_JOINTS` 一致。接收数组可包含额外头部，可乱序；按名称映射，缺失/非有限值成为显式缺项；名称重复或数组长度不一致拒绝该事件。

本版本采用框架与 ROS 已使用的同名 actuator 关节坐标：rad、同方向、同零位，不额外乘符号或加零位。这里只证明软件数据链使用同一坐标，**未重新现场校验编码器零位、关节方向或机械限位**。新适配器若坐标不同，必须明确转换后再发送物理角，不可只改标签。

Normal 的 scale/default 来自每个实际 ONNX 的 metadata；已核对的 `model_normal.onnx` SHA256 为 `2bb6f8f23f783d9daacbc6786823125a9d0872488dedb9da3f5d5b59e61ff3b4`。该源码的动作转换没有 action clip 或关节角限位；框架之后如何处理由最终 q_command 实测。SONIC 的20是归一化 action 限幅，不是机械关节限位。软件不虚构缺少来源的物理限位。

## 事件格式 v1

一行一个 JSON 对象；新记录可 gzip。规范角度事件：

```json
{"schema":1,"kind":"model","t":100.01,"received_monotonic":100.011,"unit":"rad","names":["waist_y_joint"],"q":[0.1],"model_id":"example","model_hash":"artifact sha256","session":"process-session","seq":42,"phase":"active","position_target":true,"inference_start":100.0,"inference_end":100.01,"adapter":"reviewed-adapter","stage":"post-scale-pre-framework"}
```

此短例只有一关节，报告会明确28关节缺失；完整模型应发送29名称和对应角度。模型事件必须有 model_id/session/seq/phase/position_target；hash应提供，不可用随机值冒充模型指纹。记录器真实接收时会覆盖 `received_monotonic`，保留生产者的 `t`。所有生产者必须与记录器同主机/同 boot 的 monotonic 域；跨主机不能直接混用。

- `model`：真实成功推理完成后的物理目标。`phase` 为 active/hold/blend/unknown；`position_target=false` 表示不适用于角度模型跟踪。纯力矩/速度模型若没有角目标，不能填假的 q；可只采 command/actual，并在适配说明声明不适用。
- `command`：最终位置目标。保留 kp/kd/torque、源 ROS `source_stamp_ns`、`ros_now_ns`、源时钟名称和接收单调/墙钟时间。可带 `model_session/model_id/model_hash/model_seq` 明确关联推理。
- `actual`：真实反馈的 name/position。可带可信的 `enabled`。目前核实的 ROS 反馈接口不含硬件使能位，报告会写 `hardware_enable_not_observed`；KP>0仅表明位置控制意图，不冒充电机已使能的证明。显式 disabled 或命令 KP<=0 不评分。
- `mode`：状态、阶段及原始状态 JSON。状态/阶段变化分段，不使用 SONIC 专属状态白名单。
- `command_link`：源消息stamp精确关联当前周期的模型身份和序号。模型事件丢失时，序号关联失败，禁止自动退回猜测模型。该周期没有模型输出则 `model_seq=null`，不会沿用上一周期的模型。
- `tap_sent/tap_dropped/tap_errors`：生产者累计计数；记录器自己的 queue_lost/writer_errors 独立保留。

ROS适配原始记录保留 `/hardware/actuator_states` 的 `name/position/velocity/effort` 和 `/hardware/actuators_cmds` 的 `actuators_name/pos/vel/kp/kd/torque`。JSON字段使用旧工具的统一 `names/position` 名称，但没有改值。头部额外元素在原记录中保留。

## 时间和评分

ROS源时刻映射到主机单调轴：`t = received_monotonic - (ros_now_ns - stamp_ns)/1e9`。两次时钟读取不可能完全同时，本方法不等于硬件同步；来源时间必须正数、年龄非负且不超过 import 的 `--source-age-max`（默认1秒，仅是时间可信度检查）。接收单调倒退、ROS与单调偏移跳变（默认20ms，可设）或boot变化切为新时钟段。原始时间始终保留。

每个反馈点只找**不晚于反馈的最终指令**；每个指令优先用完整session/model/hash/sequence关联推理，否则用**不晚于指令的模型目标**并明确标注 `causal_time_only`。因此 q_model 是该控制指令对应的模型目标，不是未下发的新推理。显式序号丢失/过期不允许时间回退。目标按采样保持，不做线性插值、不使用未来目标。

模型身份/hash/session/阶段变化、状态变化和异常事件建立隔离边界。已知序号跳帧的缺口禁止无限保持旧目标；在前一模型周期后停止其评分。源时间乱序/重复保留、标记无效并设边界。纯时间戳推断无法证明每个物理包是否丢失；报告同时列实测频率、最大间隔、超过3倍中位周期的缺口数。

默认只评 active 模型阶段；指令独立存在且phase未知时允许计算指令误差并记录unknown，不推断模型结果。`--include-phases active,hold,blend` 可另做探索报告，阶段仍独立。

覆盖分两种：有效角度样本比例；各反馈点至下一点的时间支持（跨超过反馈gap的缺口给0，最后一点给0）。部位“完整覆盖时间”要求该部位所有关节都有效。分母是该段首末反馈时刻跨度，不代表整个记录器启动窗口；模型频率和反馈频率分别计算。

滞后估计只在当前段内部、连续有效窗口使用因果采样保持搜索0到250ms（默认步长5ms）。至少100样本、至少2秒跨度、至少5°变化才估计；边界命中和不足原因明确报告。候选滞后用相同样本集合，补偿RMSE单列，不改原时间轴主误差。部位滞后为可估计关节滞后的中位数，列出参与数量；不是网络延迟或某条命令的电机响应时间。


## 可选 SONIC 重建参考（0.2.0）

SONIC STK1 导入额外保留同次推理的 `motion_token`、`sonic_source`、`sonic_policy_status`、`source_blend_active` 及参考帧/序号/接收时间；导入元数据 `sonic_trace_fields_version=1` 标记此能力。记录协议和记录回调未改变。旧SQLite仍可分析原有三组误差；参考模式发现缺token时明确要求从原始JSONL重新导入，不自动寻找机器人或重跑控制模型。

`--sonic-reference` 只启用离线旁路。参考 `q_ref` 由同一个已关联模型事件的token解码，直接复用最终指令→模型的身份/因果关联、时钟分段、缺口隔离及反馈有效掩码。参考年龄 `reference_age_s` 是反馈时刻减对应推理完成时刻，包含模型到指令和指令到反馈两段时间；不是电机响应延迟。

参考需 `sonic-stk1` 适配器、manifest允许的控制模型SHA、64维合法post-FSQ token、active/live/live_reference且无source blend。`--include-phases` 不取消这个参考专属条件。该规则只影响参考比较，不重新引入通用核心的SONIC状态白名单，也不阻止其他模型的指令/执行评分。

每段 `sonic_reference.inference_status_counts` 按独立推理计数；`association_status_counts` 按反馈配对计数。只有后者满足原有反馈/使能证据/阶段等条件才计入 `valid_body29_frames`。报告同时保留这两种计数，缺项不填0误差。manifest或模型资产损坏报输入错误；不兼容策略/缺token/解码输出失败显示参考不可用并保留执行结果。
