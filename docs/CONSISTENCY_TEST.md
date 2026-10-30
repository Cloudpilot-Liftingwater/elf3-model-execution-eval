# ELF3 执行跟随一致性测试

主指标是 `q_actual - q_model`：29 个身体关节的编码器角度与真实模型解码后物理目标角之差。
同时看 `q_actual - q_command` 和 `q_command - q_model`，分别定位最终指令跟随与框架处理的影响。
所有误差先按未平移的时间轴计算；滞后估计单列，不用平移后的曲线替代主结果。

这是给定策略、增益、动作及负载下的执行跟随测试。闭环策略在不同机器人上可能输出不同目标，
策略的 PD 目标也可能超出实际关节行程；不能只凭一次误差超限断言机械故障，
也不能从编码器一致推定外部绝对姿态精度。

## 当前工程初筛配置

`elf3-dynamic-screening-v1` 是本项目提出的可修改初筛值，不是 ISO 或 BXI 的验收标准。
全部 29 个关节逐项检查，不能靠身体平均值掩盖单个关节异常：

| 指标 | 门槛 |
| --- | --- |
| MAE | ≤ 3° |
| RMSE | ≤ 5° |
| P95 绝对误差 | ≤ 10° |
| 绝对偏差 | ≤ 3° |
| 最大绝对误差 | ≤ 20° |

只评 `active`，排除混合、保持、切换和无模型角度目标的段。
单段至少 2 秒、50 次有效独立模型事件，每关节有效时间覆盖率至少 95%。
缺失、乱序、时钟跳变、采集丢包和未知使能状态继续在数据完整性中报告，不补零。
模型事件按会话、模型、指纹、序号去重计数，1000 Hz 反馈不能算成 1000 Hz 推理。
有足够数据的超限段报告 `screening_fail`，其他短段/不足段仍单独列出；
无足够数据则为 `insufficient_data`，没有目标则为 `not_assessed`。
已知记录丢失等完整性问题会阻止正面的整体初筛结论。

流程是否完整和精度是否满足配置分开：提前回 normal 为 `cancelled`，整次执行为 `incomplete`；
即使剩余样本误差小也不能把它说成完整通过。精度初筛是离线判断，不触发停机或零力矩。

```bash
elf3-eval record --seconds 0 --robot-id ELF3-001 --output /path/to/new-run
elf3-eval import /path/to/new-run --output /path/to/input.sqlite
elf3-eval analyze /path/to/input.sqlite --output /path/to/new-report --acceptance-profile elf3-dynamic-screening-v1
```

不传 `--acceptance-profile` 时，保持原有只输出指标的行为。
该参数也接受 JSON 文件，字段与 `elf3_eval.acceptance.PROFILE` 相同。
`joint_overrides` 可覆盖单关节，例如 `{"l_knee_y_joint":{"rmse_deg":7.0}}`。
不同模型需要不同门槛时用 `--model` 选择对应模型，传入其校准后的 JSON 配置；
报告保存实际使用的配置和每项超限值。仿真入口可用 `ELF3_EVAL_PROFILE=/path/to/profile.json` 指定配置。

## 多机对照条件

为每台机器人填写 `--robot-id`；不从记录工作站名推断机器人编号。
同一动作建议每台至少重复 3 次，先收集已确认健康机器的基准分布，再修订每模型、每关节的门槛。
对照模型及动作文件指纹、解码版本、KP/KD、控制频率、限幅/滤波配置、固件、负载、地面、
电量和初始温度。用健康机器的重复波动判断其他机器是否偏离，而不是把上述初筛值当成统一产品规格。
相同模型不代表闭环中的每一帧 q_model 相同，因此三组角度必须一起保留。

## 温度与故障记录

记录器保留按原始关节名排列的 `motor_temperature`、`driver_temperature`，包含消息中的头部关节。
报告给出每关节温度范围、有效数、非法数组计数；旧记录没有温度时为 `unobserved`，仿真为 `not_assessed_simulation`。
新模型状态复用原 B 测试的电机型号配置：达到 `torque_limit_c - 15°C` 停止，
当前配置为 75°C 或 85°C，温度反馈缺失、异常或超过 0.5 秒未更新也触发原故障退出。
驱动器温度记录到报告，软件不猜测缺少厂家规格的驱动器保护阈值。

控制侧 `ERROR` 表示故障、`WARN` 表示人工中止或中断、`INFO` 表示进度/完成/不适用。
事件保存状态、会话、时间、级别、分类、故障码、原因及可用的实测值和门槛。
ROS 文本日志和 eval 的 `test_events`/`diagnostics` 均保留。
底层驱动已有保护继续生效；当前 ActuatorStates 没有硬件故障码或使能标志，不编造这些状态。

## 参考资料与适用范围

- [ROS 2 Humble joint_trajectory_controller 参数](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/parameters.html)：运动容差和终点容差按关节配置；不提供适用于 ELF3 舞蹈的通用数值。
- [ISO 9283:1998](https://www.iso.org/standard/22244.html)：工业机器人性能测试方法；本项目不是 ISO 9283 符合性认证。
- [NISTIR 8090](https://www.nist.gov/publications/measuring-and-representing-performance-manufacturing-assembly-robots)：按任务选择性能度量。
- [SONIC 原论文](https://arxiv.org/html/2511.07820v1)：参考动作模仿还使用身体位置及物理指标；不能把那些指标的门槛直接移作电机目标角跟随门槛。

以上资料支持测量方法和配置思路；本页的 3°/5°/10° 等数值是待健康真机基准校准的工程选择。
