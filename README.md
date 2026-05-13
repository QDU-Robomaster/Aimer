# Aimer

`Aimer` 是 tracker 后面的瞄点与弹道模块。它不订阅图像和 IMU，只在收到
`tracker/target` 后选择要打的装甲板，预测目标运动，解算最终 `yaw/pitch`，
并发布 DevC 云台目标和发射许可。

## 数据流

- 输入 `tracker/target`：`ArmorTracker` 发布的 `ArmorTrackerTarget`。
- 输入 `host/robot_game_ref`：完整裁判摘要，载荷为 Aimer 本地定义的 31 字节 `AimerRefereeSummary`，用于更新反馈弹速并记录热量上限和冷却值；异常或过低时回退到默认弹速。
- 输入 `gimbal/rotation`：云台当前姿态，只用于自动开火判定。
- 输出 `host/target_euler`：DevC `HostData` 接收的云台目标，载荷为角度、角速度和角加速度前馈。
- 输出 `host/fire_notify`：DevC `LauncherCMD` 接收的发射许可，值与最终云台计划开火门控保持一致。

坐标约定：`ArmorTrackerTarget` 使用 `x` 向前、`y` 向上、`z` 向左；
Aimer 的 yaw、半径展开、水平距离和弹道解算都以 `x-z` 为水平面，
高度使用 `y`。

## 代码结构

- `Aimer.hpp` 保留模块 manifest、公有消息结构、配置项、`AimerCore` 运行核心和外层 `Aimer<Info>` 模块。
- `AimerMath.hpp` 放角度归一化、坐标水平面约定、动态开火阈值和弹道解算。
- `AimerTargetModel.hpp` 放 tracker target 预测、装甲板展开和瞄点选择。
- `AimerPlanner.hpp` 放 TinyMPC 参考轨迹、求解器初始化和 host 云台目标生成。
- `AimerImpl.hpp` 放 topic 回调、命令发布和主回调流程；模块自身实现已改为头文件内联，TinyMPC 自身 `.cpp` 仍由 CMake 编译。
- `AimerPreview.hpp` 是 `Aimer<Info>` 内部持有的预览实现，订阅原始帧和 Aimer 输出，绘制 tracker 装甲模型与最终瞄点。

## 策略

- 每个 `tracker/target` 回调都会发布一组输出；目标丢失或弹道不可解时输出默认空命令。
- Aimer 先按 yaw 速度选择固定延迟，再预测到延迟后目标，随后选择水平距离最近的装甲板。
- 第一次弹道飞行时间用于继续预测目标，最终瞄点是命中时刻的最近装甲板。
- 云台目标使用 yaw/pitch 双积分 TinyMPC，默认 `HORIZON=100`、`dt=0.01`、`HALF_HORIZON=50`、`max_yaw_acc=100`、`max_pitch_acc=100`。
- `is_fire` 需要命令稳定、云台对齐、目标可打，且 TinyMPC 计划误差满足开火阈值；没有 `gimbal/rotation` 时不会自动开火。
- 运行期 info 日志只记录统计事件：`host/robot_game_ref` 反馈弹速变化、开火状态翻转以及热量/冷却配置变化。

## 预览

`Aimer` 内置 preview 是可选功能，不参与瞄准决策。它从构造参数 `sync`
传入的 `CameraFrameSync<Info>` 取原始帧，用同一时间戳的 `tracker/target`、
`host/target_euler` 和 `host/fire_notify` 绘制：

- tracker 整车几何展开后的装甲面轮廓。
- 白色目标中心十字。
- 红色 host 云台目标投影；开火状态下额外画红圈。

预览不会订阅 detector 结果，也不会输出额外调试 topic。

在 BSP 里只实例化 `Aimer`，不要单独实例化 `AimerPreview`。开启方式是给
`Aimer` 配置相机模板参数、`sync` 和 `cfg.preview`：

```yaml
- id: aimer
  name: Aimer
  template_args:
    Info: {constexpr: AutoAimRunConfig::HikCameraInfo}
  constructor_args:
    cfg:
      preview:
        enabled: true
        output_mode: web
        web_stream_name: aimer_preview
    sync: '@camera_frame_sync'
```

## 边界

- Aimer 不负责目标跟踪，也不修改同步链路。
- Aimer 直接发布 DevC 接口的 `host/target_euler` 和 `host/fire_notify`；不需要额外 bridge 模块。
- Aimer 不依赖旧版跳变标志、额外序号或历史 delay 字段，当前输入以
  `ArmorTrackerTarget` 字段和显式 `*_extra_predict_s` 延迟配置为准。
- 运行日志不参与控制闭环；热量反馈缺失时不会伪造当前热量。
