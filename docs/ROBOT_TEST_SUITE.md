# ELF3统一手柄测试

评测器负责旁路记录、误差分析和报告；配套BXI控制器负责机器人测试动作及状态切换。
两个库分别安装，使用同一ROS环境和相同的观察socket。

- 评测器：[Cloudpilot-Liftingwater/elf3-model-execution-eval](https://github.com/Cloudpilot-Liftingwater/elf3-model-execution-eval)，版本0.4.0。
- 控制器：[Cloudpilot-Liftingwater/bxi_rl_controller_ros2_example，elf3-execution-suite分支](https://github.com/Cloudpilot-Liftingwater/bxi_rl_controller_ros2_example/tree/elf3-execution-suite)。
- 原控制器基线：`bxirobotics/bxi_controller_ros2` 的 `robot_test`，提交 `e1d412f8efbe932ae82481385341ea5d52ac88e7`。
- [控制器真机部署和验收说明](https://github.com/Cloudpilot-Liftingwater/bxi_rl_controller_ros2_example/blob/elf3-execution-suite/src/bxi_example_py_elf3/mods/com.bxi.execution_suite/HARDWARE.zh-CN.md)。

## 两个平台相同的按键

先启动控制器、硬件或仿真、手柄输入及记录器；控制器完成初始化并处于PD后操作：

| 步骤 | 手柄操作 | 结果 |
| --- | --- | --- |
| 1 | LB＋B | 进入悬挂待测，回中准备完成后等READY |
| 2 | 单按B后松开 | 自动关节行程→300秒振动→持续跑步 |
| 3 | RB＋Y | 跑步进入playback后，受控回中、进入normal；仿真自动释放虚拟挂绳，真机人工放绳 |
| 4 | LB＋A | CXK load2完整执行后回normal等待 |
| 5 | LB＋X | SONIC06完整执行后回normal等待 |
| 6 | LB＋Y | SONIC11完整执行后回normal等待 |

模型的资源准备、入场、执行和退场时，RB＋X可主动中止并回normal。
View/Back单键在正常测试流程优先请求零力矩，无长按等待，也不要求先松开其他键。
初始化完成前只同步按键，需就绪后重新按下；原IMU锁定保护屏蔽遥控事件。
手柄、ROS或控制线程失效时，软件按键不提供独立停机通道，也不替代硬件急停。

手柄统一加载 `remote_controller/config/execution_suite_gamepad.yaml`，并禁用系统启停指令。
当前映射核对过PowerA手柄：LB=js6、RB=js7、A=js0、B=js1、X=js3、Y=js4、View/Back=js10。
同一机器人只保留一份正确配置的输入进程和一份控制器。

## 控制器启动

在配套控制工作区构建 `bxi_example_py_elf3` 和 `remote_controller`，source其安装环境，
确保控制器Python能导入本评测器。模型资产由现场单独提供：

```bash
export BXI_CXK_ROOT=/path/to/com.bxi.cxk
export BXI_SONIC_ROOT=/path/to/com.bxi.sonic
export ELF3_EVAL_ENABLE=1
export ELF3_EVAL_SOCKET=elf3_model_eval_v1
```

真机统一入口复用原硬件启动程序，默认同时启动测试手柄输入：

```bash
ros2 launch bxi_example_py_elf3 example_launch_execution_suite_hw.launch.py
```

它传入manual模式和被动观察开关，沿用现场机器人配置、硬件复位、IMU选择和进程锁。
`enable_imu:=false`适用于现场已经运行独立IMU的方式；BMS沿用现场安排。
它包含原硬件入口，不能与另一份原硬件/控制器同时启动。

仿真沿用官方BXI MuJoCo和相同控制器，在本机隔离的ROS域中启动：

```bash
export ROS_DOMAIN_ID=73
export ROS_LOCALHOST_ONLY=1
ros2 launch bxi_example_py_elf3 example_launch_execution_suite.launch.py \
  execution_eval_socket:=elf3_model_eval_v1
```

在另一终端source相同环境、设置同一ROS域，再启动手柄：

```bash
ros2 run remote_controller remote_controller --driver joystick --disable-system-actions \
  --config "$(ros2 pkg prefix remote_controller)/share/remote_controller/config/execution_suite_gamepad.yaml"
```

仿真和真机只选对应的一套入口；不要将仿真隔离域设置带到现场真机启动环境。

## 记录和评判

模型观察和记录器需在同一主机，且 `ELF3_EVAL_SOCKET` 与 `--address`一致。
先运行记录器，原始目录每次新建，持续记录到人工停录：

```bash
# 真机；使用现场相同ROS域。
elf3-eval record --seconds 0 --robot-id ELF3-001 --address elf3_model_eval_v1 --output /path/to/new-run

# 仿真；使用同一隔离ROS域，并选择simulation话题。
elf3-eval record --seconds 0 --robot-id mujoco-local --address elf3_model_eval_v1 \
  --state-topic /simulation/actuator_states --command-topic /simulation/actuators_cmds \
  --mode-topic /simulation/state_machine_info --output /path/to/new-sim-run
```

记录器出现 `events.jsonl.gz` 并保持运行后开始测试；测试完成后停止记录器，确认
`capture_result.json`，然后离线分析：

```bash
elf3-eval import /path/to/new-run --output /path/to/new-run/input.sqlite
elf3-eval analyze /path/to/new-run/input.sqlite --output /path/to/new-run/report \
  --acceptance-profile elf3-dynamic-screening-v1
```

原B测试只有指令目标；CXK/SONIC同时记录模型目标、最终指令和编码器反馈。
完整执行、人工中止、故障、数据完整性和执行精度分别报告。
阈值、温度和多机对照条件见[一致性测试说明](CONSISTENCY_TEST.md)。

## 验证范围

评测器105项离线测试、配套控制器338项测试、4项C++测试和2项静态检查通过。
完整手柄路由测试在内存反馈和虚拟时钟上保留300秒振动；真实SONIC解码及官方ROS
MuJoCo入/退场另有仿真验证。软件尚未完成真机部署验收，功能流程完成不等于精度合格。
