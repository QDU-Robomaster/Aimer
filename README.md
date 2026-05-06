# Aimer

`Aimer` 是 tracker 后面的瞄点与弹道模块。它不订阅图像和 IMU，只在收到
`tracker/target` 后选择要打的装甲板，预测目标运动，解算最终 `yaw/pitch`。

## 数据流

- 输入 `tracker/target`：`ArmorTracker` 发布的 `ArmorTrackerTarget`。
- 输入 `referee/bullet_speed`：裁判系统或上游估计的当前弹速，异常或过低时回退到默认弹速。
- 输入 `gimbal/rotation`：云台当前姿态，只用于自动开火判定。
- 输出 `tracker/target_eulr`：兼容旧下游的云台角度 topic，名称保持历史拼写。
- 输出 `tracker/gimbal_plan`：SP-style TinyMPC 云台计划，包含目标角、计划角、角速度和角加速度。
- 输出 `tracker/fire_notify`：兼容旧下游的开火通知，由 `tracker/send.is_fire` 派生。
- 输出 `tracker/send`：兼容旧下游的瞄点、角度和开火标志。
- 输出 `aimer/metrics`：调试统计，包括是否有效、迭代次数、弹速、飞行时间和本模块处理耗时。
- 输出 `aimer/trajectory`：调试用模型弹道，包含当前图像时间戳、命中点和固定数量的世界系弹道采样点。

## 策略

- 每个 `tracker/target` 回调都会发布一组输出；目标丢失或弹道不可解时输出默认空命令。
- 低速普通目标直接选择距离最近的装甲板，避免无意义切面。
- 高速旋转目标和前哨站先按配置延迟预测目标，再按进入角、离开角优先选择可击打装甲板；当前没有可击打窗口时继续瞄准锁定面或最近面，但不开火。
- `target.position` 表示整车中心；多装甲目标按 `position + radius * [cos(yaw_i), sin(yaw_i)]` 还原装甲板坐标，与 `ArmorTracker` 发布语义保持一致。
- 弹道飞行时间最多迭代 10 次，收敛阈值为 `0.001 s`。
- `gimbal_plan` 使用 yaw/pitch 双积分 TinyMPC，默认 `HORIZON=100`、`dt=0.01`、`max_yaw_acc=50`、`max_pitch_acc=100`。
- Webots 默认 `mpc_max_iter=7`，用于避免同进程仿真链路被 planner 计算拖乱；需要接近 SP 默认 10 次迭代时应先把 planner 从同步关键路径解耦。
- `aimer/trajectory` 画的是当前模型下的预测弹道，不代表 Webots 或实机里已经发射出的真实弹丸轨迹。
- `is_fire` 需要命令 yaw 稳定且云台 yaw 已对齐；没有 `gimbal/rotation` 时不会自动开火。

## 边界

- Aimer 不负责目标跟踪，也不修改同步链路。
- Aimer 不依赖旧版跳变标志或额外序号，当前输入以 `ArmorTrackerTarget` 字段为准。
- `latency_ms` 是本模块回调处理耗时，不是传感器采集时间。
