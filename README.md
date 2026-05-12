# Aimer

`Aimer` 是 tracker 后面的瞄点与弹道模块。它不订阅图像和 IMU，只在收到
`tracker/target` 后选择要打的装甲板，预测目标运动，解算最终 `yaw/pitch`，
并发布云台计划、开火通知和下级兼容命令。

## 数据流

- 输入 `tracker/target`：`ArmorTracker` 发布的 `ArmorTrackerTarget`。
- 输入 `referee/bullet_speed`：裁判系统或上游估计的当前弹速，异常或过低时回退到默认弹速。
- 输入 `gimbal/rotation`：云台当前姿态，只用于自动开火判定。
- 输出 `tracker/gimbal_plan`：TinyMPC 云台计划，包含目标角、计划角、角速度和角加速度。
- 输出 `tracker/fire_notify`：开火通知，由 `tracker/send.is_fire` 派生。
- 输出 `tracker/send`：瞄点、角度和开火标志。

坐标约定：`ArmorTrackerTarget` 使用 `x` 向前、`y` 向上、`z` 向左；
Aimer 的 yaw、半径展开、水平距离和弹道解算都以 `x-z` 为水平面，
高度使用 `y`。

## 代码结构

- `Aimer.hpp` 保留模块 manifest、公有消息结构、配置项、`Aimer` 类声明和 topic 成员。
- `AimerMath.hpp` 放角度归一化、坐标水平面约定、动态开火阈值和弹道解算。
- `AimerTargetModel.hpp` 放 tracker target 预测、装甲板展开和瞄点选择。
- `AimerPlanner.hpp` 放 TinyMPC 参考轨迹、求解器初始化和 `gimbal_plan` 生成。
- `AimerImpl.hpp` 放 topic 回调、命令发布和主回调流程；模块自身实现已改为头文件内联，TinyMPC 自身 `.cpp` 仍由 CMake 编译。

## 策略

- 每个 `tracker/target` 回调都会发布一组输出；目标丢失或弹道不可解时输出默认空命令。
- Aimer 先按 yaw 速度选择固定延迟，再预测到延迟后目标，随后选择水平距离最近的装甲板。
- 第一次弹道飞行时间用于继续预测目标，最终瞄点是命中时刻的最近装甲板。
- `gimbal_plan` 使用 yaw/pitch 双积分 TinyMPC，默认 `HORIZON=100`、`dt=0.01`、`HALF_HORIZON=50`、`max_yaw_acc=100`、`max_pitch_acc=100`。
- `is_fire` 需要命令稳定、云台对齐且目标可打；没有 `gimbal/rotation` 时不会自动开火。

## 边界

- Aimer 不负责目标跟踪，也不修改同步链路。
- Aimer 不依赖旧版跳变标志、额外序号或历史 delay 兼容字段，当前输入以
  `ArmorTrackerTarget` 字段和显式 `*_extra_predict_s` 延迟配置为准。
