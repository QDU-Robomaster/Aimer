# Aimer

`Aimer` 是 tracker 后面的瞄点与弹道模块。它收到 `tracker/target_frame`
后选择要打的装甲板，预测目标运动，解算最终机械俯仰 roll 和 yaw，并发布
DevC 云台目标和发射许可。控制逻辑只使用 tracker 目标状态；同源图像和 IMU
只供内置 preview 投影使用。

## 数据流

- 输入 `tracker/target_frame`：`ArmorTracker` 发布的同源目标帧，包含
  `ArmorTrackerTarget` 和 detector 源图像/IMU。
- 输入 `host/robot_game_ref`：完整裁判摘要，载荷为 Aimer 本地定义的 31 字节 `AimerRefereeSummary`，用于更新反馈弹速并记录热量上限和冷却值；异常或过低时回退到默认弹速。
- 输入 `host/gimbal_quat`：C 板回传的云台当前姿态，只用于自动开火判定。
- 输出 `host/target_euler`：DevC `HostData` 接收的云台目标，载荷为角度、角速度和角加速度前馈；机械俯仰轴使用 roll 字段。
- 输出 `host/fire_notify`：DevC `LauncherCMD` 接收的发射许可，值与最终云台计划开火门控保持一致。

坐标约定：`ArmorTrackerTarget` 使用右手系，`x` 向右、`y` 向前、`z` 向上。
Aimer 的 yaw 以前向为 0、左转为正；水平距离使用 `x-y` 平面，高度使用 `z`。

## 代码结构

- `Aimer.hpp` 保留模块 manifest、公有消息结构、配置项、`AimerCore` 运行核心和外层 `Aimer<Info>` 模块。
- `AimerMath.hpp` 放角度归一化、坐标水平面约定、动态开火阈值和弹道解算。
- `AimerTargetModel.hpp` 放 tracker target 预测、装甲板展开和瞄点选择。
- `AimerPlanner.hpp` 放 TinyMPC 参考轨迹、求解器初始化和 host 云台目标生成。
- `AimerImpl.hpp` 放 topic 回调、命令发布和主回调流程；模块自身实现已改为头文件内联，TinyMPC 自身 `.cpp` 仍由 CMake 编译。
- `AimerPreview.hpp` 是 `Aimer<Info>` 内部持有的预览实现，在 tracker 同源图像上绘制 tracker 装甲模型与最终瞄点。

## 策略

- 每个 `tracker/target_frame` 回调都会发布一组输出；目标丢失或弹道不可解时输出默认空命令。
- Aimer 先按 yaw 速度选择固定延迟，再预测到延迟后目标，生成直接弹道候选。
- 弹道解算只使用二次空气阻力模型：给定发射仰角后用 RK4 积分弹丸运动，再用一维括区求根求低弹道仰角；不可解时输出空命令，不回退到无阻力模型。
- TinyMPC 参考轨迹在 `HORIZON=100`、`dt=0.01`、`HALF_HORIZON=50`
  的预测窗口内逐采样重新选择几何最近装甲板，使云台在切板前具备提前减速能力。
- 云台目标使用 yaw/roll 双积分 TinyMPC，默认 `max_yaw_acc=50`、
  `max_roll_acc=100`、`q_yaw_pos=q_roll_pos=9000000`、
  `q_yaw_vel=q_roll_vel=0`、`r=1`。
- `host/fire_notify` 绑定单发弹丸的未来命中候选：在开火采样点按计划枪线遍历所有物理装甲面，选择角误差最小的命中面，再检查该面命中时刻是否可打。
- `is_fire` 需要命中面可打、计划枪线与命中候选一致、命令稳定、云台对齐；没有 `host/gimbal_quat` 时不会自动开火。
- 运行期 info 日志只记录统计事件：`host/robot_game_ref` 反馈弹速变化、开火状态翻转以及
  热量上限/冷却配置变化；当前热量缺失时日志明确写 `heat=unknown`。

## 预览

`Aimer` 内置 preview 是可选功能，不参与瞄准决策。它使用
`tracker/target_frame` 中的 detector 源图像和同帧 IMU 绘制：

- tracker 整车几何展开后的装甲面轮廓。
- 白色目标中心十字。
- 红色预测瞄点投影；开火状态下额外画红圈。

预览不会订阅原始图像、detector 结果或 host 输出，也不会输出额外调试 topic。

在 BSP 里只实例化 `Aimer`，不要单独实例化 `AimerPreview`。开启方式是给
`Aimer` 配置相机模板参数和 `cfg.preview`：

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
```

## 边界

- Aimer 不负责目标跟踪，也不修改同步链路。
- Aimer 直接发布 DevC 接口的 `host/target_euler` 和 `host/fire_notify`；不需要额外 bridge 模块。
- Aimer 不依赖旧版跳变标志、额外序号或历史 delay 字段，当前输入以
  `ArmorTrackerTarget` 字段和显式 `*_extra_predict_s` 延迟配置为准。
- 运行日志不参与控制闭环；热量反馈缺失时不会伪造当前热量。
