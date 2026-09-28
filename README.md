# Aimer

`Aimer` 是 tracker 后面的瞄点与弹道模块。它收到 `tracker` 域的 `target_frame` 后选择要打的
装甲板，预测目标运动，解算最终机械俯仰 roll 和 yaw，并发布 DevC 云台目标和发射许可。控制
逻辑使用 tracker 目标状态和同帧 IMU；同源图像只供内置 preview 投影使用。

`tinympc/` 下的 TinyMPC 源码为第三方代码（MIT），来源与修改说明见 [NOTICE](NOTICE)。

## 数据流

- 输入 `tracker/target_frame`：`ArmorTracker` 同步发布的 `const TrackedFrame*`，包含
  `SharedFrame` 图像所有权，以及按值携带的 IMU、`ArmorTrackerTarget` 和投影变换。同帧 IMU
  四元数同时作为当前云台姿态，用于自动开火的对齐判定。
- 裁判输入位于 `host` 域，话题名由 `cfg.referee_topic` 配置（默认 `robot_game_ref`，须非空），
  类型直接使用 `Referee::RobotGameRefereePack`（`AimerRefereeSummary` 是同一类型的别名），
  发布方须使用同一类型。回调读取热量上限和冷却值用于日志，弹速仍取配置的
  `default_bullet_speed`，不把摘要当作实测弹速。
- 输出 `host/target_euler`（`AimerHostGimbalTarget`）：DevC `HostData` 接收的云台目标，包含
  角度、角速度和角加速度前馈，单位 rad、rad/s、rad/s^2；机械俯仰轴使用 `rol*` 字段，`pit*`
  字段为兼容旧接口镜像同一值。
- 输出 `host/fire_notify`（`AimerHostFireNotify`）：DevC `LauncherCMD` 接收的发射许可，值与
  最终云台计划开火门控保持一致。

坐标约定：`ArmorTrackerTarget` 使用右手系，`x` 向右、`y` 向前、`z` 向上。Aimer 的 yaw 以
前向为 0、左转为正；水平距离使用 `x-y` 平面，高度使用 `z`。

## 代码结构

- `Aimer.hpp`：模块 manifest、公有消息结构、配置项 `AimerConfig`、`AimerCore` 运行核心和外层
  `Aimer<FrameLayoutV>` 模块。
- `AimerMath.hpp`：角度归一化、坐标水平面约定、动态开火阈值和弹道解算。
- `AimerTargetModel.hpp`：tracker target 预测、装甲板展开、瞄点和命中候选选择。
- `AimerPlanner.hpp`：TinyMPC 参考轨迹、求解器初始化和 host 云台目标生成。
- `AimerImpl.hpp`：topic 回调、命令发布和主回调流程。
- `AimerPreview.hpp` / `AimerPreviewGeometry.hpp`：`Aimer` 内部持有的预览实现。
- `GimbalPlan.hpp`：内部云台计划结构。

模块实现为头文件内联，TinyMPC 自身的 `.cpp` 由 CMake 编译进 `xr`。

## 策略

- 每个 `target_frame` 回调都会发布一组输出；目标丢失或弹道不可解时输出全零云台目标且不开火。
- 预测延迟为 `image_to_now_s + vision_to_command_delay_s + command_transport_delay_s +
  gimbal_response_delay_s`，再按 `|v_yaw|` 是否超过 `yaw_rate_threshold` 加上
  `high_speed_extra_predict_s` 或 `low_speed_extra_predict_s`。Aimer 预测到延迟后的目标，
  选择瞄点，再按弹丸飞行时间二次预测并重新选择瞄点。
- 瞄准命令和开火候选的弹道使用二次空气阻力模型（`a = -k·|v|·v`）：给定发射仰角后用 RK4 积分
  弹丸运动，再在 `[ballistic_min_elevation_deg, ballistic_max_elevation_deg]` 内用二分括区
  求根求仰角；不可解时输出空命令，不回退到无阻力模型。TinyMPC 参考轨迹的逐采样命令使用
  无阻力解析弹道近似。
- TinyMPC 参考轨迹在 `PLAN_HORIZON = 75`、`dt = 0.01 s`、中点 `PLAN_HALF_HORIZON = 37` 的
  预测窗口内逐采样重新选择几何最近装甲板，使云台在切板前具备提前减速能力；输出取窗口中点
  的状态。
- 云台目标使用 yaw/roll 双积分 TinyMPC，默认 `max_yaw_acc = 50`、`max_roll_acc = 100`、
  `q_yaw_pos = q_roll_pos = 9000000`、`q_yaw_vel = q_roll_vel = 0`、`r = 1`。MPC 关闭、
  求解失败或输出偏离参考超过 `mpc_fire_thresh` 时，回退为不带前馈的直接 yaw/roll 命令。
- `host/fire_notify` 绑定单发弹丸的未来命中候选：在开火采样点（中点后 `fire_delay_s`，为 0
  时取 2 个采样）按计划枪线遍历所有物理装甲面，选择角误差最小的命中面，再检查该面命中时刻
  是否可打。
- 开火需要 `auto_fire` 打开、命中面可打、计划枪线与命中候选一致、本帧命令与上一帧命令之差
  小于阈值的 2 倍，且同帧 IMU 给出的云台姿态与命令的偏差小于动态阈值。动态阈值由目标距离、
  装甲尺寸和视角计算，并限制在 `min_fire_threshold` 到 `max_fire_threshold` 之间。
- 运行期 info 日志（`enable_runtime_log`）只记录统计事件：弹速变化、开火状态翻转、热量上限/
  冷却变化，以及每 30 次 MPC 规划一次的耗时；当前热量缺失时日志明确写 `heat=unknown`。

`OnMonitor()` 打印 target_frame 回调的累计耗时统计（次数、平均、最小、最大，单位微秒）。

## 预览

`Aimer` 内置 preview 是可选功能（`cfg.preview.enabled`），不参与瞄准决策。它使用
`target_frame` 中的源图像和同帧 IMU 绘制：

- tracker 整车几何展开后的装甲面轮廓和中心（前向面绿色、背向面紫色，当前绑定面加粗）以及
  相邻面连线。
- 预测选中装甲面的轮廓和瞄点标记：不开火时为橙色，开火时为红色并加圆圈。
- 顶部状态行：跟踪状态、目标编号、绑定面、瞄准面和开火状态。

预览不会订阅原始图像、detector 结果或 host 输出，也不会输出调试 topic。

预览重投影使用原生相机内参和畸变系数投影到原生像素，再根据当前帧 geometry 映射到图像坐标，
因此 wide 和 centered ROI 共用同一套标定。每帧 ROI、下采样和翻转关系只从
`target_frame.image.Get()->geometry` 读取。无效标定或需要预先去畸变的模型不生成预览投影点；
此处理不改变弹道和开火策略。在 BSP 里只实例化 `Aimer`，不要单独实例化 `AimerPreview`。

## 职责

- Aimer 不负责目标跟踪，也不修改同步链路。
- Aimer 直接发布 DevC 接口的 `host/target_euler` 和 `host/fire_notify`。
- Aimer 的输入以 `ArmorTrackerTarget` 字段和显式预测延迟配置为准。
- 相机标定只影响内置预览投影，不进入控制、弹道或 MPC 计算。
- 运行日志不参与控制闭环；热量反馈缺失时不会伪造当前热量。

## 依赖

- `QDU-Robomaster/ArmorTracker`：`TrackedFrame` / `ArmorTrackerTarget` 类型和 `target_frame`
  输入。
- `QDU-Robomaster/CameraBase`：标定、帧布局和 geometry 类型。
- `QDU-Robomaster/VisionPreview`：预览输出。
- `xrobot-org/DurationStatistics`：回调耗时统计。
- `QDU-Robomaster/Referee`：`Referee::RobotGameRefereePack` 裁判摘要类型。
- 外部：OpenCV 4（`core`、`calib3d`）、Eigen；仓库内置 TinyMPC（`tinympc/`）。

## 构造接口

```cpp
template <CameraTypes::FrameLayout FrameLayoutV>
class Aimer : public AimerCore;

Aimer(
    Config cfg = DefaultConfig(),
    CameraCalibration calibration = DefaultCalibration());
```

模板参数：

- `FrameLayoutV`：帧布局，必须与上游相机和 ArmorTracker 相同。

依赖：无构造依赖参数；`target_frame` 和裁判输入通过 topic 订阅。

配置：

- `cfg`（`AimerConfig`，`DefaultConfig()` 即全部默认值）：

| 字段 | 默认值 | 说明 |
| --- | --- | --- |
| `yaw_offset` | `-1.0` | 施加到 yaw 命令的固定偏置，deg。 |
| `roll_offset` | `-1.4` | 施加到机械 roll 轴命令的固定偏置，deg。 |
| `yaw_rate_threshold` | `2.0` | 高/低速预测补偿的 yaw 角速度阈值，rad/s。 |
| `default_bullet_speed` | `21.0` | 弹速，m/s。 |
| `min_valid_bullet_speed` | `14.0` | 低于该值时改用 `default_bullet_speed`，m/s。 |
| `ballistic_drag_k` | `0.02` | 二次阻力系数 `k`。 |
| `ballistic_integration_dt_s` | `0.001` | RK4 积分步长，s。 |
| `ballistic_max_iterations` | `16` | 仰角求根最大迭代次数（限制在 4–64）。 |
| `ballistic_min_elevation_deg` / `ballistic_max_elevation_deg` | `-20.0` / `35.0` | 仰角搜索范围，deg。 |
| `auto_fire` | `true` | 是否启用自动开火门控。 |
| `image_to_now_s`、`vision_to_command_delay_s`、`command_transport_delay_s`、`gimbal_response_delay_s` | `0.0` | 预测延迟各分量，s。 |
| `fire_delay_s` | `0.0` | 开火命令到出膛的延迟，决定开火采样点，s。 |
| `low_speed_extra_predict_s` / `high_speed_extra_predict_s` | `0.015` / `0.03` | 低速/高速目标的额外预测，s。 |
| `min_fire_threshold` / `max_fire_threshold` | `0.003` / `0.05` | 动态开火角度阈值范围，rad。 |
| `enable_mpc_plan` | `true` | 是否启用 TinyMPC 云台计划。 |
| `mpc_fire_thresh` | `0.05` | 允许使用 MPC 输出和开火的最大计划偏离，rad。 |
| `max_yaw_acc`、`q_yaw_pos`、`q_yaw_vel`、`r_yaw_acc` | `50.0`、`9000000.0`、`0.0`、`1.0` | yaw 轴 MPC 约束与代价。 |
| `max_roll_acc`、`q_roll_pos`、`q_roll_vel`、`r_roll_acc` | `100.0`、`9000000.0`、`0.0`、`1.0` | roll 轴 MPC 约束与代价。 |
| `preview` | 关闭 | `VisionPreview::RuntimeParam`，字段见 VisionPreview。 |
| `enable_runtime_log` | `true` | 是否输出运行期统计日志。 |
| `bullet_speed_log_delta` / `heat_log_delta` | `0.05` / `1.0` | 弹速、热量日志的变化阈值。 |
| `convert_raw_gimbal_quat_to_body` | `false` | 仅作用于内部 `ahrs_quaternion` 回调；该输入当前未注册，不影响运行。 |
| `referee_topic` | `"robot_game_ref"` | `host` 域裁判话题名。 |

- `calibration`：原生相机标定，只用于预览投影，构造时检查其合理性。默认
  `DefaultCalibration()` 为 1280x720、`fx = fy = 800`、主点 `(640, 360)`、零畸变；启用预览时
  应传入与相机一致的标定。

## 使用

```sh
xrobot module add QDU-Robomaster/Aimer
xrobot setup
xrobot instance add QDU-Robomaster/Aimer
```

`xrobot instance add` 在 `User/xrobot.yaml` 中写入一个实例，默认值按源码写出；本模块没有需要
填写的依赖。帧布局用 constexpr 定义，必须与相机输出一致：

```yaml
constexpr_includes:
  - CameraBase.hpp
constexprs:
  FrameLayout:
    type: CameraTypes::FrameLayout
    value: '{.width = 640, .height = 480, .step = 1920, .encoding = CameraTypes::Encoding::BGR8}'
modules:
  - module: QDU-Robomaster/Aimer
    id: aimer_0
    template_args:
      - ProjectConstexpr::FrameLayout
    args:
      - cfg: Aimer<ProjectConstexpr::FrameLayout>::DefaultConfig()
      - calibration: Aimer<ProjectConstexpr::FrameLayout>::DefaultCalibration()
```

本模块不使用 BSP 对象，不需要 `XR_REGISTER`。它通过 `tracker/target_frame` 接收同一
`template_args` 的 ArmorTracker 实例输出，通常列在 ArmorTracker 之后。

`cfg` 也可以写成 YAML map（字段名同上表，字符串写成 C++ 字符串字面量），`calibration` 可以
引用 `CameraTypes::CameraCalibration` 类型的 constexpr。例如打开 Web 预览并设置弹速：

```yaml
cfg:
  yaw_offset: 0.0
  roll_offset: 0.0
  yaw_rate_threshold: 2.0
  default_bullet_speed: 23.0
  min_valid_bullet_speed: 14.0
  ballistic_drag_k: 0.02
  ballistic_integration_dt_s: 0.001
  ballistic_max_iterations: 16
  ballistic_min_elevation_deg: -20.0
  ballistic_max_elevation_deg: 35.0
  auto_fire: true
  image_to_now_s: 0.0
  vision_to_command_delay_s: 0.0
  command_transport_delay_s: 0.0
  gimbal_response_delay_s: 0.0
  fire_delay_s: 0.0
  low_speed_extra_predict_s: 0.015
  high_speed_extra_predict_s: 0.03
  min_fire_threshold: 0.003
  max_fire_threshold: 0.05
  enable_mpc_plan: true
  mpc_fire_thresh: 0.05
  max_yaw_acc: 50.0
  q_yaw_pos: 9000000.0
  q_yaw_vel: 0.0
  r_yaw_acc: 1.0
  max_roll_acc: 100.0
  q_roll_pos: 9000000.0
  q_roll_vel: 0.0
  r_roll_acc: 1.0
  preview:
    enabled: true
    preview_window_name: '"aimer_preview"'
    preview_scale: 0.5
    preview_wait_key_ms: 1
    queue_capacity: 1
    output_mode: '"web"'
    web_bind_address: '"0.0.0.0"'
    web_port: 8080
    web_stream_name: '"aimer_preview"'
    max_fps: 30.0
  enable_runtime_log: true
  bullet_speed_log_delta: 0.05
  heat_log_delta: 1.0
  convert_raw_gimbal_quat_to_body: false
  referee_topic: '"robot_game_ref"'
```

填好后再次运行 `xrobot setup`，生成 `User/xrobot_main.hpp`。

`xrobot module show .`（在本仓库中）或 `xrobot module show Modules/QDU-Robomaster/Aimer`
（在 BSP 中）打印当前的构造函数。

## 测试

在打开 `BUILD_TESTING` 的 BSP 构建中，本模块加入 `aimer_preview_geometry_test`、
`aimer_stage_frame_access_test`，以及用默认和自定义裁判话题名运行的
`aimer_referee_default_topic_test` / `aimer_referee_sentry_topic_test`，用 `ctest` 运行。
