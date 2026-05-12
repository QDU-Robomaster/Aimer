# Aimer

`Aimer` 是 tracker 后面的瞄点与弹道模块。它不订阅图像和 IMU，只在收到
`tracker/target` 后选择要打的装甲板，预测目标运动，解算最终 `yaw/pitch`，
并把每帧决策、开火理由和 shot 事件落盘。

## 数据流

- 输入 `tracker/target`：`ArmorTracker` 发布的 `ArmorTrackerTarget`。
- 输入 `referee/bullet_speed`：裁判系统或上游估计的当前弹速，异常或过低时回退到默认弹速。
- 输入 `gimbal/rotation`：云台当前姿态，只用于自动开火判定。
- 输出 `tracker/target_eulr`：云台角度 topic，名称保持现有拼写。
- 输出 `tracker/gimbal_plan`：Planner-compatible TinyMPC 云台计划，包含目标角、计划角、角速度和角加速度。
- 输出 `tracker/fire_notify`：开火通知，由 `tracker/send.is_fire` 派生。
- 输出 `tracker/send`：瞄点、角度和开火标志。
- 输出 `aimer/metrics`：调试统计，包括是否有效、迭代次数、弹速、飞行时间和本模块处理耗时。
- 输出 `aimer/trajectory`：调试用模型弹道，包含当前图像时间戳、命中点和固定数量的世界系弹道采样点。
- 输出 `aimer/decision`：每帧决策快照，包含策略分档、选面原因、开火门槛和命中时刻。
- 输出 `aimer/shot_event`：每次实际发射命令的事件快照，便于 Webots / 内录离线复盘。

坐标约定：`ArmorTrackerTarget` 使用 `x` 向前、`y` 向上、`z` 向左；
Aimer 的 yaw、半径展开、水平距离和弹道解算都以 `x-z` 为水平面，
高度使用 `y`。

## 策略

- 每个 `tracker/target` 回调都会发布一组输出；目标丢失或弹道不可解时输出默认空命令。
- Aimer 按 SP `Planner` 主路径语义工作：先按 yaw 速度选择固定延迟，再预测到延迟后目标，随后选择水平距离最近的装甲板。
- 第一次弹道飞行时间用于继续预测目标，最终瞄点是命中时刻的最近装甲板。
- `gimbal_plan` 使用 yaw/pitch 双积分 TinyMPC，默认 `HORIZON=100`、`dt=0.01`、`HALF_HORIZON=50`、`max_yaw_acc=100`、`max_pitch_acc=100`，Q/R 与 SP Planner 默认配置一致。
- `aimer/trajectory` 画的是当前模型下的预测弹道，不代表 Webots 或实机里已经发射出的真实弹丸轨迹。
- `is_fire` 需要命令稳定、云台对齐且目标可打；没有 `gimbal/rotation` 时不会自动开火。

## 边界

- Aimer 不负责目标跟踪，也不修改同步链路。
- Aimer 不依赖旧版跳变标志、额外序号或历史 delay 兼容字段，当前输入以
  `ArmorTrackerTarget` 字段和显式 `*_extra_predict_s` 延迟配置为准。
- 当前 `shot_event` 是发射命令事件，不是 Webots 物理真实碰撞；真实命中判定仍由离线真值脚本完成。
- `latency_ms` 是本模块回调处理耗时，不是传感器采集时间。
