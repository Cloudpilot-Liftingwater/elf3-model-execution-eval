# robot_test 舞蹈评测接入

**本页说明旧的独立舞蹈接口。当前CXK＋SONIC六步测试使用
[统一测试流程](ROBOT_TEST_SUITE.md)，其中LB＋X是SONIC06，不沿用下述旧舞蹈键位。**

针对 `bxirobotics/bxi_controller_ros2` 的 `robot_test`，基于 `e1d412f8efbe932ae82481385341ea5d52ac88e7`。
配套补丁新增 `com.bxi.dance_test`，normal 状态 LB+X 进入，完整播放一遍后平滑回 normal。
现有 A/B/X/Y 悬挂测试不变。没有自动部署到机器人或修改上游分支。

评测库 0.3.0 新增 `host.register_policy`、`host.policy_output`、`host.test_event`。
舞蹈 wrapper 调用现有策略的 step，直接记录返回的 `output.joints.position`，不重新解释归一化 action。
新 Mod 可以选择旧 MJLab、Gravity MJLab、Gravity IsaacLab V1/V2/V3 解码器。
自动识别仅限已知 154/157 维合同；历史/残差语义必须显式配置，不能仅凭输出维度猜测。

注册时计算模型和动作 SHA256（控制回调无文件读取），记录 backend、解码规则、scale/default
和实际动作帧号。首版固定用 ONNX Runtime，不宣称其他推理后端已适配。
框架每周期清除关联，并在 transition 前后标记混合；最终 ROS publish 成功后关联消息时间戳和推理序号。
当前周期没有模型时不沿用上一模型。预热、混合、保持不进入默认 active 模型误差评分。

## 人工使用流程

先在部署窗口核对同一提交和配套资产，完成仿真，再安装审核后的控制补丁和 eval wheel。
在已有 ROS 环境中使用可见的 eval 解释器；不需要 conda，也不引入训练环境。

```bash
# 终端一，同机器人主机：先录制，原始目录必须为空。
elf3-eval record --seconds 120 --output /path/to/new-dance-run

# 终端二：环境变量传给将启动的控制进程。
export ELF3_EVAL_ENABLE=1
export ELF3_EVAL_SOCKET=elf3_model_eval_v1
export BXI_DANCE_MOTION=/path/to/verified/dance.npz
export BXI_DANCE_MODEL=/path/to/verified/dance.onnx
export BXI_DANCE_VARIANT=auto
export BXI_DANCE_START_FRAME=0
export BXI_DANCE_FIXED_POS=false
# 用经过核实的原有入口启动；机器人进入 normal 后手动 LB+X 开始。
# 本分支硬件 launch 可显式指定 execution_eval:=1，将开关传入控制节点。
# 动作末尾自动退出；LB+X 可取消，原有安全操作保持有效。
```

完整 sim2sim 套件现在默认使用 `record --seconds 0`，跟随仿真退出收尾；显式正数仍表示录制时限。
BXI 仿真复位只发送一次 `step1`，在 normal 中首次发送 `step2/release=true`，不在启动时消耗 step2。

本任务 MuJoCo 入口 `work/mujoco_dance.sh` 于 2026-10-28 默认切换到用户指定的
`com.bxi.cxk/plugin.py::_load2`：`cxkv5.npz` + `cxkv5_15k.onnx`，V3、起始帧 0、
`BXI_DANCE_FIXED_POS=true`。完整动作 109.08 秒，自动记录默认 300 秒。
键盘入口 `keyboard` 使用 B 确认 pd_brake、N 进入 normal 并等待状态/连续有效 PD 指令确认后释放初始悬挂、
D 开始/取消舞蹈，Q 退出键盘；仿真不启动手柄节点。仿真 launch 的 reset 阶段保持悬挂。
这是独立测试副本中的设置，没有更改稳定库或安装到真机。

记录端是旁路订阅者，没有新增 actuator 发布者；模型进程和记录器必须同机以共享单调时钟和 Unix socket。
另一个终端的新 export 不会影响已运行的遥控服务；需把环境设置给真正的控制进程，
或在已安排的启动中使用 `example_demo_hw.launch.py execution_eval:=1`。不要启动第二套控制器。
记录结束后可在工作站执行：

```bash
elf3-eval import /path/to/new-dance-run --output /path/to/new-dance.sqlite
elf3-eval analyze /path/to/new-dance.sqlite --output /path/to/new-dance-report
```

查看 `report.html` 的 `actual_minus_model`：模型目标与编码器反馈误差；
同时检查 `actual_minus_command` 与 `command_minus_model`，将框架处理和执行跟随区分开。
原始误差不移动时间轴，滞后估计单列。没有模型观察时报告“模型输出未观测”，不填 0。
`report.json.test_events` 为完整记录的状态标记，包含 started/playback_complete/cancelled/interrupted/failed；
即使指定分析时间窗，仍保留完整生命周期供核对。

## 完成不等于精度通过

`playback_complete` 只说明最后动作帧已使用，随后请求回 normal；不表示回位误差达标。
现场核查实际回到 normal，且记录器队列/回调/写盘计数正常，并检查 29 维模型和反馈覆盖。
精度是否通过需要用户给出动作、关节、误差和覆盖率的验收条件；本软件不虚构统一门槛。

本轮新增端到端测试生成小型 ONNX 与动作，实际使用 ONNX Runtime、现有 DanceMotionPolicyMjlab、
新的舞蹈状态、原最终发布函数（仅替换 ROS 传输）和 eval 导入/分析。
模型—反馈人为差值 0.05 rad、最终指令偏移 0.02 rad，期望三组 MAE 为 0.05/0.03/0.02 rad。
包含 31 维带头部、身体关节乱序、完成/中止/故障、观察异常隔离和末帧保持。
生成资产仅用于验证。仓库真实 dance 资产、ROS/Unix socket 运行和真机未在当前受限环境验证。
