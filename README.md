# Aimer

`Aimer` 负责从 `tracker/target` 中选择可击打装甲板，并使用迭代弹道求解输出最终 `yaw/pitch`。

## Runtime Role

- 输入: `tracker/target`、`referee/bullet_speed`
- 输出: `tracker/send`、`tracker/target_eulr`、`aimer/metrics`

## Algorithm Notes

- 使用 `target.jumped` 决定是否直接锁定当前装甲板
- 按 `yaw_rate_threshold` 切换低速目标和高速旋转目标分支
- 最多 10 次飞行时间迭代，收敛阈值 `0.001 s`
- `is_fire` 兼容输出使用 `sp_vision/shooter.cpp` 风格容差判定，不再走旧 `ShouldFire`
