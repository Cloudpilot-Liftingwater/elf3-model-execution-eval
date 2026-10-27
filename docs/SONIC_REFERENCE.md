# SONIC 重建参考与执行比较

0.2.0 增加可选的离线 `g1_kin` 旁路。复用旧评测工具的角度拆解和 FSQ token 检查，使用当前独立库的配对、分段、覆盖率和误差统计。现场记录回调及机器人控制程序不因这一功能改变。

## 四种角度

| 曲线 | 来源 | 含义 |
|---|---|---|
| `q_ref` | 同次推理的 post-FSQ token → g1_kin | 模型重建的当前参考姿态 |
| `q_model` | g1_dyn action → 实际clip/scale/default | 模型生成的物理控制目标 |
| `q_command` | 最终ROS位置指令 | 框架完成处理后发布的目标 |
| `q_actual` | actuator编码器反馈 | 上报的关节执行角度 |

参考模式增加 `actual_minus_reference`、`model_minus_reference`、`command_minus_reference` 三组结果。原有三组结果保持原定义。每组仍包含每关节/部位的MAE、RMSE、P95、最大绝对误差、偏差、样本/时间覆盖和独立的轨迹滞后结果。

`q_ref` 是学习模型重建，不是外部测量真值；控制目标与参考不必逐点相等。参考曲线不向机器人下发，不取代控制目标，也不用于计算“硬件精度达标”。

## trace 和状态具体指什么

- **SONIC trace**：每次真实推理后，通过既有STK1观察点记录的内部快照，含模型指纹、会话/序号、推理时间、原始action、物理目标、token及来源信息。它提供模型目标和参考重建所需输入。
- **SONIC策略状态**：trace中的 `source`、`policy_status`、`source_blend_active`。例如 `live_reference` 表示使用实时参考，`stale_hold` 表示参考陈旧后的保持；混合标志表示切换来源时的过渡。
- **框架状态**：`/hardware/state_machine_info` 的当前状态/过渡信息，用于划分运行段。通用评测不要求某个SONIC专属状态名称。
- **电机反馈**：`/hardware/actuator_states` 的关节位置/速度等。虽然话题含states，它不是上面所说的运行状态机。

只有指令和电机反馈、没有trace时，软件仍可计算指令—反馈误差；无法恢复模型目标或token，不会生成假的参考角。

## 准备与运行

在项目自己的uv/venv安装可选依赖：

```bash
uv pip install --python .venv/bin/python -e '.[sonic-reference]'
```

参考资产目录需要两个相互匹配的文件：

```text
reference-bundle/
  kinematic_manifest.json
  g1_kin_step28800.onnx
```

可从旧工具 `tools/tracking_eval/evaluation/` 原样取这两个文件。它们可放在独立目录，不需要旧SONIC Python包、训练环境或整套仓库。本版本只实现并验证了此manifest规定的10帧布局，不能只改文件名或SHA就宣称兼容其他训练模型。

```bash
# 已有原始记录，使用新版重新导入。0.1的SQLite未保留token。
elf3-eval import /path/to/recording --output /path/to/new-sonic.sqlite
elf3-eval analyze /path/to/new-sonic.sqlite --output /path/to/new-report \
  --sonic-reference /path/to/reference-bundle

# 现场已具备SONIC观察点时，沿用原来的被动记录入口
elf3-eval record --legacy-sonic --seconds 60 --output /path/to/new-recording
```

`record` 不运行 g1_kin。以上现场命令由操作者安排窗口执行，新记录器仍未完成现场验收。

## 转换和时间约定

解码器输出 `(N,10,64)`。前58列先按训练布局展平为580项，再拆成前290项角度、后290项速度，各自恢复成 `(N,10,29)`；剩余6列是相对锚点朝向。角度已经是绝对关节坐标的rad，不再缩放或加默认姿态。按manifest名称映射到body29，取第0帧作为当前 `q_ref`；其他9帧只存档，不参与当前误差评分。

每个反馈先关联不晚于它的有效指令，再关联该指令对应的模型推理。参考来自这同一次推理，按采样保持，不独立寻找“更近”的token。模型/会话/阶段切换、时钟异常、序号缺口继续沿用通用边界；不跨边界插值。年龄阈值仍使用本次CLI配置或各时钟段的周期估计，不恢复旧工具固定50ms门槛。

使用 `--start/--end` 时，反馈误差只统计所选时间窗；推理次数及存档参考窗口按包含这些反馈的完整运行段计数，保留配对上下文，可能包含窗外推理。推理次数不能当成该时间窗内的反馈帧数。

参考只评匹配模型的 `active + live + live_reference` 且无source blend阶段；即便使用 `--include-phases active,hold,blend`，保持/混合也不计入参考评分。这样不会将过渡时的混合目标误当成同一个token直接产生的控制结果。

## 不可用原因与输出

| 原因 | 处理 |
|---|---|
| `missing_motion_token_reimport_required` | 从原始记录重导入；若原记录本来没有trace，则无法重建 |
| `incompatible_policy_hash` | 选择与实际控制模型匹配且已验证的参考资产 |
| `invalid_fsq_token` | 保留缺项，不将非法token修正或填零后解码 |
| `hold_or_blend_reference_excluded` / `not_live_reference` | 单独统计这些阶段，不纳入参考误差 |
| `not_sonic_trace` | 其他模型继续运行通用评测，不套用SONIC参考 |
| `model_not_associated` | 反馈没有通过现有指令—模型配对，不回退猜token |
| `decoder_failed` / `nonfinite_decoder_output` | 显式记录解码问题，保留已有执行比较 |

指定资产不存在、SHA不匹配或manifest格式不支持时，命令报输入错误；需要实际解码但缺ONNX Runtime时提示安装可选依赖。报告不是只给PARTIAL：每段列推理级原因、反馈关联原因、实际评分覆盖。反馈时间无效或使能证据不足等原因仍在通用完整性报告中。

- HTML新增绿色 `q_ref`，可以切换六种误差和不同运行段。
- `report.json` / `metrics.csv` 增加三组参考比较；`sonic_reference` 记录资产指纹、配对约定和不可用原因。
- `aligned.npz` 新增 `sN_q_ref`、`sN_valid_reference`、`sN_reference_age_s`、`sN_reference_reason`。角度为rad；年龄为反馈时刻减推理完成时刻。
- `sonic_reference.npz` 保存推理时间 `t`、模型身份/会话/序号/时钟段、`reason`、完整 `reference_window_rad`、速度窗口与朝向。不可用项为NaN；读取使用 `allow_pickle=False`。

0.2.0已通过旧15秒及120秒真机数据回放、旧参考输出逐值对照和合成异常测试，详情见 [验证记录](VALIDATION.md)。这是离线验证，不是新增现场验收。
