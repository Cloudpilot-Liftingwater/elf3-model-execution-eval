# 现场人工部署与验收

本轮没有连接机器人，没有安装补丁，没有停服务、切状态、使能、CAN操作或修改现有稳定库。下面是准备好的人工流程；现场窗口和启动方式由机器人操作者安排。

## 先确认实际环境

在机器人本机已有终端只读确认当前使用的工作区、Python、ROS domain和进程启动路径。历史 `/home/bxi/bxi_vla_ws/...` 和 `/home/bxi/bxi_ws/...` 不能互换。主机IP会变；此项目不内置IP、不执行SSH。

把独立项目放入新目录，按 README 创建 venv。`elf3-eval check --ros` 只验证导入，不初始化控制节点。核对三个硬件话题的消息类型；收到身体29+头部2的31维合法，不要删头部以通过检查。

只做指令—执行采集不需要任何宿主补丁。旧 SONIC 若已启用 STK1 观察点，可用 `--legacy-sonic`，不再改 SONIC 的 policy 或 tracking_tap。

## Normal 观察点：先在副本检查

补丁只适用于 `expected_sources.json` 中三个完全匹配的源码文件，不适用于旧 main 副本。先在工作区副本执行：

```bash
# 此变量由操作者设置为实际 package 根，含 bxi_example_py_elf3/ 子目录
ELF3_PACKAGE_ROOT=/path/to/reviewed-copy/src/bxi_example_py_elf3
python tools/check_patch_sources.py "$ELF3_PACKAGE_ROOT"
```

此命令只有哈希检查和 `patch --dry-run --fuzz=0`。任何不匹配均停止，不猜路径、不自动寻找另一套现场库。

已安排现场窗口、已备份这三个文件、操作者确认使用该副本后，人工执行：

```bash
patch --batch --fuzz=0 -p1 -d "$ELF3_PACKAGE_ROOT" -i "$PWD/patches/underlay-normal-observer.patch"
```

补丁只增加：Normal推理物理角观察；公共控制周期身份清空/过渡标记；最终ROS发布后的时间戳/序号关联。每个观察调用都有异常隔离。模型算法、动作缩放、控制返回值、消息发布内容均保持原逻辑。

依照原有现场构建/启动流程使这份副本生效；本项目不提供自动停止或重启机器人命令。启动控制程序前，在**该控制进程实际使用的Python环境**安装独立包，并设置：

```bash
export ELF3_EVAL_ENABLE=1
export ELF3_EVAL_SOCKET=elf3_model_eval_v1
```

导入时读取开关；在已运行进程外设置环境变量不会改进程。未设置开关时不发送观察事件。Normal模型指纹在policy构造阶段计算，控制周期不读模型文件。传入多artifact的ModelSpec时，hash是artifact清单摘要，标记为 `hash_scope=artifact_manifest`，不能冒充已确定的单个物理后端文件hash。

先启动记录器，再按操作者原流程运行动作：

```bash
elf3-eval record --seconds 60 --output /path/to/new-normal-run
```

机器人是否能活动、姿态/支撑与测试动作由现场现有流程决定。记录器没有要求零力矩或使能，不替代控制系统。

## 现场验收条件

1. 新记录存在command/state与真实模型事件，模型ID/hash对应实际加载的模型，名称含全部29身体关节，单位/方向/零位核对一致。
2. 检查 command_link 精确匹配ROS源stamp，模型session/hash/sequence关联成功；不能将仅时间推断当成已证实序号关联。
3. 以操作者允许的短时静态和小幅动作记录；检查三条曲线及反馈来源。人工动作过程不由本工具发命令。
4. 检查各源实测频率、队列和tap丢样、源时间年龄、时钟段、状态/保持/混合标记。未取得硬件使能位时保留unknown，不宣称硬件使能已被验证。
5. 停记录器或让接收队列饱和的观察失败测试，只能在操作者安排的允许窗口做；确认控制返回与原行为一致。先比较观察关闭/开启的控制时序，再决定是否长期启用。
6. 复制原始记录、metadata、capture_result与实际宿主文件hash到工作站离线分析；核查模型结果和指令结果没有混称。
7. 单独填写该模型的现场结论：通过/失败/未测、模型hash、框架hash、时间、操作者、完整性依据、采用的角度容差。容差必须由用户按任务确定，不使用本工具虚构的通用达标线。

只有完成这些实际测量才标记“现场通过”。旧SONIC记录回放不构成新记录器验收，Normal真实ONNX离线推理不构成Normal真机验收。

## 撤销

记录器 Ctrl+C 只停止自身。操作者可在原启动流程里去掉 ELF3_EVAL_ENABLE，或从已备份的三个文件恢复观察补丁；不要对正在运行的程序自动改文件或重启。若需要人工反向补丁，先在对应副本运行 `patch --dry-run --reverse --fuzz=0` 并检查差异；不对不同版本强行反向应用。
