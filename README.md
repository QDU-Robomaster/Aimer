# Aimer

弹道瞄准模块：选择装甲板、预测目标运动并解算云台目标与发射许可 / Ballistic aiming Module that selects the armor plate, predicts the target motion and solves the gimbal target and fire permission

## 1. 模块作用 / Purpose

Aimer 订阅 `tracker` 域的 `target_frame`（ArmorTracker 发布的 `const TrackedFrame*`，包含 `SharedFrame` 图像所有权，以及按值携带的同帧 IMU、`ArmorTrackerTarget` 和投影变换）。每收到一帧，Aimer 选择要打的装甲板，预测目标运动，解算机械俯仰轴 roll 与 yaw，并向 `host` 域发布云台目标 `target_euler` 和发射许可 `fire_notify`。同帧 IMU 四元数同时作为当前云台姿态，用于自动开火的对齐判定。目标丢失或弹道不可解时，输出全零云台目标，发射许可为 `false`。

预测延迟为 `image_to_now_s + vision_to_command_delay_s + command_transport_delay_s + gimbal_response_delay_s`，再按 `|v_yaw|` 是否超过 `yaw_rate_threshold` 加上 `high_speed_extra_predict_s` 或 `low_speed_extra_predict_s`；`lead_calibration` 打开时再加上在线标定的修正量（见下文）。Aimer 预测到延迟后的目标，选择瞄点，再按弹丸飞行时间二次预测并重新选择瞄点。

弹道使用二次空气阻力模型（`a = -k·|v|·v`）：给定发射仰角后用 RK4 积分弹丸运动，再在 `[ballistic_min_elevation_deg, ballistic_max_elevation_deg]` 内用二分括区求根求仰角；不可解时输出空命令。TinyMPC 参考轨迹的中心采样使用同一有阻力弹道求飞行时间和仰角，其余采样使用无阻力解析弹道，仰角加上中心采样两种解之差。

云台目标使用 yaw 与 roll 两轴的双积分 TinyMPC 生成。参考轨迹覆盖 `PLAN_HORIZON = 75` 个采样（`dt = 0.01 s`），逐采样重新选择几何最近的装甲板，使云台在切板前开始减速；输出取窗口中点 `PLAN_HALF_HORIZON = 37` 的状态，包含角度、角速度和角加速度前馈。MPC 关闭、求解失败或输出偏离参考超过 `mpc_fire_thresh` 时，输出直接的 yaw 与 roll 命令，速度与加速度前馈为 0。

发射许可绑定单发弹丸的未来命中候选：在开火采样点（中点后 `fire_delay_s`）按计划枪线遍历所有物理装甲面，选择角误差最小的命中面，再检查该面在命中时刻是否可打。`fire_notify` 为 `true` 的条件为：

- `auto_fire` 打开，命中面可打，计划枪线与命中候选一致。
- 上一帧命令按其角速度推进到本帧后，与本帧命令之差小于动态阈值的 2 倍。
- 同帧 IMU 给出的云台姿态与命令的偏差小于动态阈值。动态阈值由目标距离、装甲尺寸和视角计算，限制在 `min_fire_threshold` 到 `max_fire_threshold` 之间。

`heat_aware_fire` 打开且已收到裁判系统的热量上限时，以上条件满足后还要通过按热量分配的开火判定。Aimer 在本地推算枪管热量：每次计入的开火加 `heat_fire_shot_heat`，相邻两发至少相隔 `heat_fire_min_interval_s`，热量按裁判系统给出的冷却值连续下降。单发命中概率按出膛时刻估计。可命中的半宽为命中面在命中时刻的投影半宽（装甲宽度的一半乘以视角余弦，再减去散布）。横向偏差为目标距离乘以 `|heat_fire_gimbal_error_gain · e + heat_fire_rate_bias_s · ω|`，其中 `e` 为请求时刻实测云台 yaw 减命令 yaw，`ω` 为命令 yaw 角速度：请求时刻的云台误差到出膛时留下一部分，命令转得越快，出膛时的偏差越大。横向标准差由 `heat_fire_sigma_m`、随预测时域增长的装甲相位误差和目标距离乘以 `heat_fire_rate_spread_s · ω` 三项合成。`heat_fire_gimbal_error_gain` 取 1、两个角速度项取 0、`heat_fire_sigma_m` 取 0.02 时，即按请求时刻云台误差估计。开火所需的命中概率从 `heat_fire_p_low`（热量为 0）线性升到 `heat_fire_p_high`（热量达到上限）；热量低于上限一半，且距上一发和开始跟踪当前目标都超过 `heat_fire_relax_s` 时，所需概率在随后的 `heat_fire_relax_s` 内降到 `heat_fire_p_floor`。出弹数受热量限制时，这一判定把出弹集中到命令转得慢、云台到出膛时仍对准的时刻。默认系数按 Webots 目标车世界的射击记录拟合（指向超前量已标定），实车按实车射击记录重新拟合。

`lead_calibration` 打开时，Aimer 在线标定指向超前量。每帧记下与图像同步的 IMU 给出的云台 yaw 和当前生效命令的 yaw 角速度。子弹若在该帧图像时刻出膛，一个飞行时间后到达；到达时刻之后的第一帧直接观测到装甲板，把该帧目标状态回推几毫秒即得到达时刻的装甲板方位。云台 yaw 与该方位之差对命令 yaw 角速度回归，斜率就是指向超前的时间，为正表示云台沿目标运动方向超前。前两批每批 250 帧、之后每批 400 帧，每批按回归结果修正预测延迟，修正量绝对值不超过 `lead_calibration_max_adjust_s`；命令角速度变化太小的批次不用。`lead_calibration_batches` 批后，修正量固定为最后三批的平均，日志给出该值，可加到 `gimbal_response_delay_s` 上长期使用。固定后继续监视，连续 `lead_calibration_monitor_batches` 批超前时间同号且绝对值超过 `lead_calibration_monitor_threshold_s` 时重新标定。标定在跟踪目标期间逐帧进行，使用每帧已有的数据。标定的对象是指向的时间误差；弹速误差、相机与 IMU 之间的时间偏移同时作用于命令和观测，需另行标定。

`host` 域的裁判 Topic（名称由 `referee_topic` 配置）提供热量上限和冷却值，用于日志和按热量分配的开火判定。

`enable_runtime_log` 打开时，运行期 info 日志记录弹速变化、开火状态翻转、热量上限与冷却变化，以及每 30 次 MPC 规划一次的耗时；当前热量缺失时日志写 `heat=unknown`。`lead_calibration` 打开时还记录每批的超前时间、修正量和固定后的修正量。`OnMonitor()` 输出 target_frame 回调的累计耗时统计（次数、平均、最小、最大，单位 μs）。

`cfg.preview.enabled` 为 `true` 时，Aimer 内置预览，使用 `target_frame` 中的源图像和同帧 IMU 绘制：

- tracker 整车几何展开后的装甲面轮廓和中心（前向面绿色、背向面紫色，当前绑定面加粗）以及相邻面连线。
- 预测选中装甲面的轮廓和瞄点标记：不开火时为橙色，开火时为红色并加圆圈。
- 顶部状态行：跟踪状态、目标编号、绑定面、瞄准面和开火状态。

预览使用 `calibration` 的内参和畸变系数投影到原生像素，再根据当前帧 geometry 映射到图像坐标，因此 wide 与 centered ROI 共用同一套标定。每帧的 ROI、下采样和翻转关系读取自 `target_frame.image.Get()->geometry`。标定无效或需要先去畸变的模型不生成预览投影点。预览不进入瞄准、弹道和开火的计算。

Aimer subscribes to `target_frame` in the `tracker` domain (a `const TrackedFrame*` published by ArmorTracker, holding the `SharedFrame` image ownership and carrying the same-frame IMU, the `ArmorTrackerTarget` and the projection transform by value). For each frame, Aimer selects the armor plate to aim at, predicts the target motion, solves the mechanical pitch-axis roll and the yaw, and publishes the gimbal target `target_euler` and the fire permission `fire_notify` in the `host` domain. The same-frame IMU quaternion also serves as the current gimbal attitude for the alignment check of the automatic fire. When the target is lost or the ballistic solution does not exist, the gimbal target is all zero and the fire permission is `false`.

The prediction delay is `image_to_now_s + vision_to_command_delay_s + command_transport_delay_s + gimbal_response_delay_s`, plus `high_speed_extra_predict_s` or `low_speed_extra_predict_s` depending on whether `|v_yaw|` exceeds `yaw_rate_threshold`; with `lead_calibration` on, the correction of the online calibration is added as well (see below). Aimer predicts the target to the delayed time, selects the aim point, predicts again by the projectile flight time and selects the aim point again.

The ballistics use a quadratic air-drag model (`a = -k·|v|·v`): for a given launch elevation the projectile motion is integrated with RK4, and the elevation is found by bracketed bisection within `[ballistic_min_elevation_deg, ballistic_max_elevation_deg]`; an empty command is output when no solution exists. The centre sample of the TinyMPC reference trajectory uses the same drag ballistics for the flight time and the elevation; the other samples use the drag-free analytic solution with the elevation offset between the two solutions at the centre added.

The gimbal target is generated by a double-integrator TinyMPC on the yaw and roll axes. The reference trajectory covers `PLAN_HORIZON = 75` samples (`dt = 0.01 s`) and reselects the geometrically nearest armor plate at every sample, so that the gimbal starts decelerating before a plate switch; the output is the state at the window midpoint `PLAN_HALF_HORIZON = 37`, including the angle, angular-velocity and angular-acceleration feedforward. When the MPC is disabled, fails to solve, or deviates from the reference by more than `mpc_fire_thresh`, the direct yaw and roll commands are output with zero velocity and acceleration feedforward.

The fire permission is bound to the future hit candidate of a single projectile: at the fire sampling point (`fire_delay_s` after the midpoint), all physical armor faces are traversed along the planned gun line, the hit face with the smallest angular error is selected, and then it is checked whether that face can be hit at the hit time. `fire_notify` is `true` when:

- `auto_fire` is on, the hit face can be hit, and the planned gun line matches the hit candidate.
- The previous frame's command, advanced to this frame with its rate, differs from this frame's command by less than twice the dynamic threshold.
- The gimbal attitude from the same-frame IMU deviates from the command by less than the dynamic threshold. The dynamic threshold is computed from the target distance, armor size and view angle, and limited to `min_fire_threshold` to `max_fire_threshold`.

With `heat_aware_fire` on and the heat limit received from the referee, a shot that meets the conditions above also has to pass the heat-aware fire decision. Aimer estimates the barrel heat locally: every counted shot adds `heat_fire_shot_heat`, two counted shots are at least `heat_fire_min_interval_s` apart, and the heat falls continuously at the cooling value from the referee. The single-shot hit probability is estimated for the muzzle exit. The hittable half-width is the projected half-width of the hit face at the hit time (half the armor width times the cosine of the view angle, minus the spread). The lateral offset is the target distance times `|heat_fire_gimbal_error_gain · e + heat_fire_rate_bias_s · ω|`, where `e` is the measured gimbal yaw minus the command yaw at the request and `ω` is the command yaw rate: part of the request-time gimbal error remains at the exit, and the faster the command turns, the larger the offset at the exit. The lateral standard deviation combines `heat_fire_sigma_m`, the plate-phase error that grows with the prediction horizon, and the target distance times `heat_fire_rate_spread_s · ω`. With `heat_fire_gimbal_error_gain` at 1, both rate terms at 0 and `heat_fire_sigma_m` at 0.02, the estimate uses the request-time gimbal error. The hit probability required to fire rises linearly from `heat_fire_p_low` (zero heat) to `heat_fire_p_high` (heat at the limit); when the heat is below half the limit and both the last shot and the start of tracking the current target are more than `heat_fire_relax_s` ago, the required probability falls to `heat_fire_p_floor` over the next `heat_fire_relax_s`. When the number of shots is limited by heat, this decision concentrates the shots at moments when the command turns slowly and the gimbal is still on target at the exit. The default coefficients are fitted to shot records of the Webots target-vehicle world (with the pointing lead calibrated); on a robot they are refitted to the robot's shot records.

With `lead_calibration` on, Aimer calibrates the pointing lead online. For every frame it keeps the gimbal yaw from the IMU synced to the image and the yaw rate of the command in force. A projectile leaving at that image time arrives one flight time later; the first frame imaged after the arrival observes the plate directly, and propagating that frame's target state back by a few milliseconds gives the plate bearing at the arrival. The difference between the gimbal yaw and that bearing is regressed on the command yaw rate; the slope is the pointing lead time, positive when the gimbal is ahead along the target motion. The first two batches have 250 frames and later batches 400 frames; each batch corrects the prediction delay by its regression result, with the absolute correction limited to `lead_calibration_max_adjust_s`; batches with too little change in the command rate are not used. After `lead_calibration_batches` batches the correction is frozen at the mean of the last three, and the log reports the value, which can be added to `gimbal_response_delay_s` for permanent use. Once frozen, the lead is still monitored, and a recalibration starts when `lead_calibration_monitor_batches` consecutive batches have a lead of the same sign above `lead_calibration_monitor_threshold_s` in magnitude. The calibration runs on every frame while a target is tracked, using data available in each frame. It covers the timing of the pointing; a bullet-speed error or a time offset between camera and IMU acts on the command and the observation alike and is calibrated separately.

The referee Topic in the `host` domain (name set by `referee_topic`) provides the heat limit and cooling value used for logging and for the heat-aware fire decision.

With `enable_runtime_log` on, the runtime info log records bullet-speed changes, fire-state flips, heat-limit and cooling changes, and the duration of every 30th MPC planning; the log shows `heat=unknown` while the current heat is unavailable. With `lead_calibration` on, it also records the lead and correction of every batch and the frozen correction. `OnMonitor()` outputs the accumulated duration statistics of the target_frame callback (count, average, minimum, maximum, in μs).

With `cfg.preview.enabled` set to `true`, Aimer provides a built-in preview drawn from the source image in `target_frame` and the same-frame IMU:

- The armor-face outlines and centers of the tracker's whole-vehicle geometry (front faces green, back faces purple, the currently bound face in bold) and the lines between adjacent faces.
- The outline of the predicted selected armor face and the aim-point marker: orange when not firing, red with a circle when firing.
- A status line at the top: tracking state, target id, bound face, aimed face and fire state.

The preview projects with the intrinsics and distortion coefficients of `calibration` to native pixels and then maps them to image coordinates according to the geometry of the current frame, so wide and centered ROIs share one calibration. The ROI, downsampling and flip of each frame are read from `target_frame.image.Get()->geometry`. An invalid calibration or a model that requires undistortion first produces no preview projection points. The preview is independent of the aiming, ballistic and fire computations.

## 2. 坐标约定 / Coordinate Convention

`ArmorTrackerTarget` 使用右手系，`x` 向右、`y` 向前、`z` 向上。Aimer 的 yaw 以前向为 0、左转为正；水平距离在 `x-y` 平面内计算，高度取 `z`。

`ArmorTrackerTarget` uses a right-handed frame with `x` to the right, `y` forward and `z` up. The Aimer yaw is 0 forward and positive to the left; the horizontal distance is computed in the `x-y` plane and the height is `z`.

## 3. 构造接口 / Constructor

```cpp
template <CameraTypes::FrameLayout FrameLayoutV>
class Aimer : public AimerCore;

Aimer(Config cfg = DefaultConfig(),
      CameraCalibration calibration = DefaultCalibration());  // 节选 / excerpt
```

模板参数：

- `FrameLayoutV`：帧布局，与上游相机和 ArmorTracker 的帧布局相同。

模块通过 Topic 接收输入，构造参数均为配置参数：

- `cfg`（`AimerConfig`，`DefaultConfig()` 为全部默认值）：

| 字段 | 默认值 | 说明 |
| --- | --- | --- |
| `yaw_offset` | `-1.0` | 施加到 yaw 命令的固定偏置，deg。 |
| `roll_offset` | `-1.4` | 施加到机械 roll 轴命令的固定偏置，deg。 |
| `yaw_rate_threshold` | `2.0` | 高/低速预测补偿的 yaw 角速度阈值，rad/s。 |
| `default_bullet_speed` | `21.0` | 弹速，m/s。 |
| `min_valid_bullet_speed` | `14.0` | 有效弹速的下限，m/s。 |
| `ballistic_drag_k` | `0.02` | 二次阻力系数 `k`。 |
| `ballistic_integration_dt_s` | `0.001` | RK4 积分步长，s（限制在 0.0001 到 0.02）。 |
| `ballistic_max_iterations` | `16` | 仰角求根最大迭代次数（限制在 4 到 64）。 |
| `ballistic_min_elevation_deg` / `ballistic_max_elevation_deg` | `-20.0` / `35.0` | 仰角搜索范围，deg。 |
| `auto_fire` | `true` | 是否启用自动开火门控。 |
| `image_to_now_s`、`vision_to_command_delay_s`、`command_transport_delay_s`、`gimbal_response_delay_s` | `0.0` | 预测延迟各分量，s。 |
| `fire_delay_s` | `0.02` | 开火命令到出膛的延迟，决定开火采样点，s；按发射机构实测值设置。 |
| `low_speed_extra_predict_s` / `high_speed_extra_predict_s` | `0.015` / `0.03` | 低速/高速目标的额外预测，s。 |
| `min_fire_threshold` / `max_fire_threshold` | `0.003` / `0.05` | 动态开火角度阈值范围，rad。 |
| `enable_mpc_plan` | `true` | 是否启用 TinyMPC 云台计划。 |
| `mpc_fire_thresh` | `0.05` | 允许使用 MPC 输出和开火的最大计划偏离，rad。 |
| `max_yaw_acc`、`q_yaw_pos`、`q_yaw_vel`、`r_yaw_acc` | `50.0`、`9000000.0`、`0.0`、`1.0` | yaw 轴 MPC 加速度约束（rad/s^2）与代价。 |
| `max_roll_acc`、`q_roll_pos`、`q_roll_vel`、`r_roll_acc` | `100.0`、`9000000.0`、`0.0`、`1.0` | roll 轴 MPC 加速度约束（rad/s^2）与代价。 |
| `preview` | 关闭，`preview_window_name` 为 `"aimer_preview"`，`preview_scale` 为 `0.5`，`web_stream_name` 为 `"aimer_preview"` | `VisionPreview::RuntimeParam`，其余字段取 VisionPreview 的默认值，字段见 VisionPreview。 |
| `enable_runtime_log` | `true` | 是否输出运行期统计日志。 |
| `bullet_speed_log_delta` / `heat_log_delta` | `0.05` / `1.0` | 弹速、热量日志的变化阈值。 |
| `convert_raw_gimbal_quat_to_body` | `false` | 原始云台四元数到 body 轴的转换开关。 |
| `referee_topic` | `"robot_game_ref"` | `host` 域裁判 Topic 名称，须非空。 |
| `heat_aware_fire` | `false` | 是否启用按热量分配的开火（见第 1 节）。 |
| `heat_fire_p_low` / `heat_fire_p_high` | `0.55` / `0.85` | 热量为 0 与达到上限时开火所需的命中概率。 |
| `heat_fire_p_floor` / `heat_fire_relax_s` | `0.3` / `0.5` | 门槛放宽的下限，以及开始放宽前的不开火时间，s。 |
| `heat_fire_sigma_m` / `heat_fire_phase_sigma_rad_s` | `0.01` / `0.7` | 命中概率模型的基础横向标准差（m）和相位误差增长率（rad/s）。 |
| `heat_fire_horizon_extra_s` | `0.05` | 加在弹丸飞行时间上的预测时域，覆盖开火命令到出膛的延迟，s。 |
| `heat_fire_shot_heat` / `heat_fire_min_interval_s` | `10.0` / `0.05` | 单发热量，发射机构相邻两发的最小间隔（s）。 |
| `heat_fire_gimbal_error_gain` | `0.65` | 出膛时保留的请求时刻云台误差比例。 |
| `heat_fire_rate_bias_s` / `heat_fire_rate_spread_s` | `0.01` / `0.01` | 横向偏差与横向标准差中乘以命令 yaw 角速度的时间，s。 |
| `lead_calibration` | `false` | 是否在线标定指向超前量（见第 1 节）。 |
| `lead_calibration_batches` | `5` | 修正量固定前的修正批数。 |
| `lead_calibration_max_adjust_s` | `0.05` | 修正量绝对值上限，s。 |
| `lead_calibration_monitor_threshold_s` / `lead_calibration_monitor_batches` | `0.003` / `3` | 固定后触发重新标定的超前时间（s）与同号连续批数。 |

- `calibration`：原生相机标定，用于预览投影，构造时检查其合理性。默认 `DefaultCalibration()` 为 1280x720、`fx = fy = 800`、主点 `(640, 360)`、零畸变；启用预览时传入与相机一致的标定。

Template parameter:

- `FrameLayoutV`: frame layout, identical to that of the upstream camera and ArmorTracker.

The Module receives its inputs through Topics; all constructor parameters are configuration:

- `cfg` (`AimerConfig`; `DefaultConfig()` holds all defaults):

| Field | Default | Meaning |
| --- | --- | --- |
| `yaw_offset` | `-1.0` | Fixed bias applied to the yaw command, in deg. |
| `roll_offset` | `-1.4` | Fixed bias applied to the mechanical roll-axis command, in deg. |
| `yaw_rate_threshold` | `2.0` | Yaw angular-velocity threshold for the high/low-speed prediction compensation, in rad/s. |
| `default_bullet_speed` | `21.0` | Bullet speed, in m/s. |
| `min_valid_bullet_speed` | `14.0` | Lower bound of a valid bullet speed, in m/s. |
| `ballistic_drag_k` | `0.02` | Quadratic drag coefficient `k`. |
| `ballistic_integration_dt_s` | `0.001` | RK4 integration step, in s (limited to 0.0001 to 0.02). |
| `ballistic_max_iterations` | `16` | Maximum iterations of the elevation root search (limited to 4 to 64). |
| `ballistic_min_elevation_deg` / `ballistic_max_elevation_deg` | `-20.0` / `35.0` | Elevation search range, in deg. |
| `auto_fire` | `true` | Enables the automatic fire gating. |
| `image_to_now_s`, `vision_to_command_delay_s`, `command_transport_delay_s`, `gimbal_response_delay_s` | `0.0` | Components of the prediction delay, in s. |
| `fire_delay_s` | `0.02` | Delay from the fire command to the projectile leaving the barrel, determines the fire sampling point, in s; set to the measured launcher delay. |
| `low_speed_extra_predict_s` / `high_speed_extra_predict_s` | `0.015` / `0.03` | Extra prediction for low-speed / high-speed targets, in s. |
| `min_fire_threshold` / `max_fire_threshold` | `0.003` / `0.05` | Range of the dynamic fire angle threshold, in rad. |
| `enable_mpc_plan` | `true` | Enables the TinyMPC gimbal plan. |
| `mpc_fire_thresh` | `0.05` | Maximum plan deviation for using the MPC output and firing, in rad. |
| `max_yaw_acc`, `q_yaw_pos`, `q_yaw_vel`, `r_yaw_acc` | `50.0`, `9000000.0`, `0.0`, `1.0` | Yaw-axis MPC acceleration constraint (rad/s^2) and costs. |
| `max_roll_acc`, `q_roll_pos`, `q_roll_vel`, `r_roll_acc` | `100.0`, `9000000.0`, `0.0`, `1.0` | Roll-axis MPC acceleration constraint (rad/s^2) and costs. |
| `preview` | disabled, `preview_window_name` is `"aimer_preview"`, `preview_scale` is `0.5` and `web_stream_name` is `"aimer_preview"` | `VisionPreview::RuntimeParam`; the other fields take the defaults of VisionPreview, see VisionPreview for the fields. |
| `enable_runtime_log` | `true` | Enables the runtime statistics log. |
| `bullet_speed_log_delta` / `heat_log_delta` | `0.05` / `1.0` | Change thresholds of the bullet-speed and heat logs. |
| `convert_raw_gimbal_quat_to_body` | `false` | Switch for converting the raw gimbal quaternion to the body axes. |
| `referee_topic` | `"robot_game_ref"` | Name of the referee Topic in the `host` domain; non-empty. |
| `heat_aware_fire` | `false` | Enables the heat-aware fire decision (see Section 1). |
| `heat_fire_p_low` / `heat_fire_p_high` | `0.55` / `0.85` | Hit probability required to fire at zero heat and at the heat limit. |
| `heat_fire_p_floor` / `heat_fire_relax_s` | `0.3` / `0.5` | Lower bound of the relaxed threshold, and the time without a shot before the relaxation starts, in s. |
| `heat_fire_sigma_m` / `heat_fire_phase_sigma_rad_s` | `0.01` / `0.7` | Base lateral standard deviation (m) and phase-error growth rate (rad/s) of the hit-probability model. |
| `heat_fire_horizon_extra_s` | `0.05` | Horizon added to the projectile flight time, covering the delay from the fire command to the muzzle, in s. |
| `heat_fire_shot_heat` / `heat_fire_min_interval_s` | `10.0` / `0.05` | Heat per shot, and the minimum interval between two shots of the launcher (s). |
| `heat_fire_gimbal_error_gain` | `0.65` | Share of the request-time gimbal error that remains at the muzzle exit. |
| `heat_fire_rate_bias_s` / `heat_fire_rate_spread_s` | `0.01` / `0.01` | Times multiplied by the command yaw rate in the lateral offset and the lateral standard deviation, in s. |
| `lead_calibration` | `false` | Enables the online calibration of the pointing lead (see Section 1). |
| `lead_calibration_batches` | `5` | Number of correcting batches before the correction is frozen. |
| `lead_calibration_max_adjust_s` | `0.05` | Limit of the absolute correction, in s. |
| `lead_calibration_monitor_threshold_s` / `lead_calibration_monitor_batches` | `0.003` / `3` | Lead time (s) and number of consecutive same-sign batches that trigger a recalibration once frozen. |

- `calibration`: native camera calibration used for the preview projection, checked for plausibility at construction. The default `DefaultCalibration()` is 1280x720 with `fx = fy = 800`, principal point `(640, 360)` and zero distortion; with the preview enabled, a calibration matching the camera is passed.

## 4. Topic

| Topic | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `tracker/target_frame` | 订阅 | `const TrackedFrame<FrameLayoutV>*` | ArmorTracker 发布的目标帧，含同源图像与同帧 IMU |
| `host/<referee_topic>`（默认 `robot_game_ref`） | 订阅 | `RefereeTypes::RobotGameRefereePack` | 裁判数据，热量上限与冷却值用于日志和按热量分配的开火 |
| `host/target_euler` | 发布 | `AimerHostGimbalTarget` | 云台目标：角度、角速度、角加速度前馈，单位 rad、rad/s、rad/s^2；机械俯仰轴使用 `rol*` 字段，`pit*` 字段取相同的值 |
| `host/fire_notify` | 发布 | `AimerHostFireNotify` | 发射许可，值与最终云台计划的开火门控一致 |

| Topic | Direction | Type | Meaning |
| --- | --- | --- | --- |
| `tracker/target_frame` | Subscribe | `const TrackedFrame<FrameLayoutV>*` | Target frame published by ArmorTracker, with the source image and the same-frame IMU |
| `host/<referee_topic>` (default `robot_game_ref`) | Subscribe | `RefereeTypes::RobotGameRefereePack` | Referee data; the heat limit and cooling value are used for logging and the heat-aware firing |
| `host/target_euler` | Publish | `AimerHostGimbalTarget` | Gimbal target: angle, angular-velocity and angular-acceleration feedforward, in rad, rad/s, rad/s^2; the mechanical pitch axis uses the `rol*` fields and the `pit*` fields hold the same values |
| `host/fire_notify` | Publish | `AimerHostFireNotify` | Fire permission, equal to the fire gating of the final gimbal plan |

## 5. 配置示例 / Configuration Example

`xrobot instance add QDU-Robomaster/Aimer --template-arg <FrameLayout>` 写入的实例，`template_args` 为帧布局，`calibration` 引用相机标定，`cfg` 为 `DefaultConfig()` 的全部默认值，也可写成以字段名为键的映射（字段见第 3 节）。帧布局与标定以 constexpr 定义，与相机输出一致：

An instance written by `xrobot instance add QDU-Robomaster/Aimer --template-arg <FrameLayout>`, with `template_args` holding the frame layout, `calibration` referring to the camera calibration and `cfg` holding all defaults of `DefaultConfig()`; `cfg` can also be written as a mapping keyed by the field names (fields in section 3). The frame layout and the calibration are defined as constexprs matching the camera output:

```yaml
constexpr_namespace: AutoAimRunConfig
constexpr_includes:
  - CameraBase.hpp
constexprs:
  FrameLayout:
    type: CameraTypes::FrameLayout
    value: '{.width = 720, .height = 540, .step = 2160, .encoding = CameraTypes::Encoding::BGR8}'
  MainCameraCalibration:
    type: CameraTypes::CameraCalibration
    value: '{.native_width = 1440, .native_height = 1080, .camera_matrix = {2348.0610281828863, 0.0, 753.7199990513768, 0.0, 2341.430205137848, 544.3093638576938, 0.0, 0.0, 1.0}, .distortion_model = CameraTypes::DistortionModel::PLUMB_BOB, .distortion_coefficients = {-0.09324913488777978, 0.3089185338125273, 0.0011528970103605383, -0.0010514494107999794, 0.0}, .rectification_matrix = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}, .projection_matrix = {2334.0852301109603, 0.0, 753.2190261545588, 0.0, 0.0, 2331.8191930180155, 544.7774518626655, 0.0, 0.0, 0.0, 1.0, 0.0}}'
modules:
  - module: QDU-Robomaster/Aimer
    id: aimer
    template_args:
      - AutoAimRunConfig::FrameLayout
    args:
      - cfg: Aimer<AutoAimRunConfig::FrameLayout>::DefaultConfig()
      - calibration: AutoAimRunConfig::MainCameraCalibration
```

`template_args` 与同一相机链路上 ArmorTracker 实例的 `template_args` 相同，Aimer 实例列在 ArmorTracker 实例之后。

The `template_args` equal those of the ArmorTracker instance on the same camera chain, and the Aimer instance is listed after the ArmorTracker instance.

## 6. 依赖与硬件 / Dependencies and Hardware

依赖：

- `QDU-Robomaster/ArmorTracker`：`TrackedFrame` / `ArmorTrackerTarget` 类型与 `target_frame` 输入。
- `QDU-Robomaster/CameraBase`：标定、帧布局与 geometry 类型。
- `QDU-Robomaster/VisionPreview`：预览输出。
- `QDU-Robomaster/Referee`：`RefereeTypes::RobotGameRefereePack` 裁判数据类型。
- `xrobot-org/DurationStatistics`：回调耗时统计。
- LibXR、OpenCV 4（`core`、`calib3d`）、Eigen。
- `tinympc/` 下的 TinyMPC 源码为第三方代码（MIT），来源与修改说明见 [NOTICE](NOTICE)，其 `.cpp` 由 CMake 编译进 `xr`。

硬件：云台由 DevC 控制，接收 `host` 域的 `target_euler` 与 `fire_notify`。

Dependencies:

- `QDU-Robomaster/ArmorTracker`: the `TrackedFrame` / `ArmorTrackerTarget` types and the `target_frame` input.
- `QDU-Robomaster/CameraBase`: calibration, frame layout and geometry types.
- `QDU-Robomaster/VisionPreview`: preview output.
- `QDU-Robomaster/Referee`: the `RefereeTypes::RobotGameRefereePack` referee data type.
- `xrobot-org/DurationStatistics`: callback duration statistics.
- LibXR, OpenCV 4 (`core`, `calib3d`) and Eigen.
- The TinyMPC sources under `tinympc/` are third-party code (MIT); see [NOTICE](NOTICE) for the origin and modifications. Its `.cpp` files are compiled into `xr` by CMake.

Hardware: the gimbal is controlled by DevC, which receives `target_euler` and `fire_notify` of the `host` domain.
