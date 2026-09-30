# METR4202 Team 13：初始化、算法调整与启动说明

本文只说明 `metr4202_2026_team13` 自研仓库。外部的 TurtleBot3、Nav2、SLAM Toolbox、Gazebo 与 DynamixelSDK 只作为依赖，不纳入自研算法结构。

适用环境：Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11。下文命令默认在 WSL 的 Ubuntu 终端内执行。

## 1. 自研包结构

| ROS 2 包 | 作用 | 主要文件 |
| --- | --- | --- |
| `metr4202_interfaces` | 定义任务、ArUco 结果以及探索/覆盖/恢复服务接口 | `msg/`、`srv/` |
| `metr4202_navigation` | 探索、覆盖、任务状态机与恢复节点 | `exploration.py`、`coverage.py`、`coverage_geometry.py`、`mission_manager.py`、`recovery.py` |
| `metr4202_aruco` | ArUco 检测、PnP 定位与多帧融合 | `aruco_detection.py`、`aruco_localization.py`、`localization_core.py` |
| `metr4202_bringup` | 仿真/真机总启动，以及 SLAM Toolbox 参数 | `simulation.launch.py`、`real_robot.launch.py`、`slam_toolbox.yaml` |

集中参数文件：

- `metr4202_navigation/config/navigation.yaml`：Exploration 与 Coverage。
- `metr4202_aruco/config/aruco.yaml`：ArUco 检测与定位/融合。
- `metr4202_bringup/config/slam_toolbox.yaml`：在线 SLAM。

## 2. 整体算法架构

### 2.1 数据与控制流

```text
LiDAR /scan ──> SLAM Toolbox ──> /map ─┬─> Exploration ─┐
                                       └─> Coverage ────┤
                                                        ├─> Mission Manager
Camera /camera/image_raw ─> ArUco Detection              │      │
                              │ /aruco/detections        │      ├─> Nav2 NavigateToPose
/camera/camera_info ──────────┴─> ArUco Localization     │      └─> Nav2 Spin
TF(odom <- camera, image stamp) ────────┘                │
                         │ 保存相对观测                  │
                         ├─探索完成：final map <- odom───┤
                         └─重投影、过滤、Kalman 融合     │
                                  │ /aruco/marker_map    │
                                  └──────────────────────┘（不参与 Coverage 选点）
```

所有运动都由 `mission_manager` 统一提交给 Nav2。Exploration 与 Coverage 只生成 `MotionTask`，不直接控制底盘。

### 2.2 Mission Manager 状态机

状态顺序：

1. `WAITING_FOR_SERVICES`：等待 Exploration、Coverage、Recovery、Nav2 action server，并确认 Nav2 lifecycle 已 active。
2. `EXPLORATION`：逐次请求最近的安全 frontier 目标，交给 Nav2 执行。
3. `ARUCO_FINALIZE`：探索结束后调用 `/aruco/finalize_exploration`，固定最终 `map <- odom`，重投影并融合探索阶段的全部相对观测，不清空结果。
4. `COVERAGE_INIT`：冻结当时的最终 OccupancyGrid，构建 Coverage 几何模型。
5. `COVERAGE`：每个候选点先 `NAVIGATE`，成功后原地 `SPIN 2π`；再更新覆盖状态并选择下一点。
6. `DONE`：没有未覆盖目标，或遇到不可恢复错误。

Nav2 任务失败时会询问 Recovery。当前 `recovery.py` 是占位实现，只返回 `SKIP`；没有清图、后退、重规划或替代目标。

### 2.3 Exploration V1

输入是 `/map` 与 TF 中的机器人位置。每次服务请求都会：

1. 将地图分成 free、occupied、unknown。
2. 找出“free 且八邻域接触 unknown”的 frontier cells。
3. 对 frontier 做八连通聚类，过滤过小簇。
4. 从 frontier 向可通行区域 BFS，寻找满足障碍净距的候选点。
5. 过滤离机器人过近、过远，以及历史上无效目标附近的候选点。
6. 选择欧氏距离最近的候选点，并令目标朝向该点的行进方向。
7. Nav2 完成后等待若干次地图更新；若已知栅格增长不超过阈值，则永久记录为无效区域。

当前声明但未参与计算的兼容参数：`information_gain_weight`、`distance_weight`、`bootstrap_spin_yaw`。修改它们不会改变 V1 行为。

### 2.4 Coverage V1

Coverage 在 INIT 时冻结最终地图，此后不会用新地图重建模型：

1. 按 `occupied_threshold` 分类栅格；occupied 与 unknown 都阻挡视线。
2. 以 `max(robot_footprint_radius, camera_radial_offset) + obstacle_clearance` 膨胀障碍，求机器人所在的四连通安全区域。
3. 用 OpenCV 提取障碍轮廓并简化折线，在面向自由空间的一侧采样带法向的边界目标。
4. 在可达安全栅格上采样候选视点；若粗采样遗漏目标，则逐级减半间距，最低到 `minimum_candidate_spacing` 或地图分辨率。
5. 对每个“候选视点—边界目标”检查相机距离、表面入射角和 supercover 栅格视线，建立不可变可见性集合。
6. 将没有任何可见候选点的目标标为 `INITIAL_UNCOVERABLE`。
7. 每轮以

   `score = 当前可覆盖目标数 / (机器人到候选点距离 + fixed_visit_cost)`

   选择最高分候选点。
8. 到达候选点后原地旋转一周；NAVIGATE 与 SPIN 都成功才把对应目标标为 covered。候选点失败后，若目标不再有其他可用候选点，则标为 `RUNTIME_UNCOVERABLE`。

`/coverage/status_map` 的覆盖层数值：covered=20、uncovered=50、initial-uncoverable=70、runtime-uncoverable=85、occupied=100。

### 2.5 ArUco

检测节点把相机图像转灰度，使用配置字典检测四角，过滤像素周长过小的 marker，发布 ID 与四个角点。定位节点：

1. 从 `/camera/camera_info` 获取相机矩阵 `K` 与畸变向量 `D`。
2. 用 marker 实际边长和 `SOLVEPNP_IPPE_SQUARE` 求解正深度姿态，选重投影误差最小解。
3. 过滤深度和重投影误差不合格观测。
4. 用图像时间戳查询 `odom <- camera optical frame` TF。对每次合格检测保存 marker ID、相机系三维位置、时间戳、该时刻的 `odom <- camera` 位姿和测量方差；探索阶段不把当时的 map 坐标作为永久结果。
5. 探索完成时查询最新的最终 `map <- odom`，清空的只是临时滤波状态，不清空历史观测；按时间顺序将全部观测通过 `camera -> odom -> final map` 重新投影。
6. 重投影后先按 marker ID 使用坐标中位数、MAD 和测量方差下限剔除严重批量离群点，再按原始时间顺序执行频率门限、观察位姿新颖性门限、Mahalanobis 门限和每 ID 独立二维 Kalman 融合。
7. Coverage 阶段保留探索阶段形成的滤波状态。新观测继续保存，并使用同一个固定 `map <- odom` 基准投影后进入同一 Kalman 滤波器。
8. 达到 `confirmation_count` 后标为 confirmed，并发布 transient-local `/aruco/marker_map`。

`/aruco/reset_map` 仍作为人工维护接口保留；它会同时清空历史相对观测、最终变换和滤波结果。正常任务状态机不再调用它。

## 3. 必须人工确认或标定的参数

### 3.1 Coverage：最终机器人与相机几何

编辑 `metr4202_navigation/config/navigation.yaml` 的 `coverage.ros__parameters`。

| 参数 | 当前值 | 后续如何确定 | 影响 |
| --- | ---: | --- | --- |
| `robot_footprint_radius` | 0.22 m | 量机器人中心到最外侧实体的最大水平距离，含保险杠/支架 | 决定安全候选区域 |
| `obstacle_clearance` | 0.08 m | 按定位误差、地图误差和安全裕量实测 | 加到安全半径上 |
| `camera_radial_offset` | 0.10 m | 量 `base_footprint` 旋转中心到相机光心的水平距离 | 把机器人中心距离换算成相机距离 |
| `visual_distance_min` | 0.20 m | 实测能稳定成像/检测的最近表面距离 | 过滤过近目标 |
| `visual_distance_max` | 2.00 m | 在最终分辨率、光照、marker/目标尺度下测试可靠最远距离 | 过滤过远目标 |
| `viewing_angle_max_degrees` | 60° | 实测相对目标表面法向的最大可靠斜视角 | 过滤过斜视线 |

注意：

- `viewing_angle_max_degrees` 是目标表面入射角，不是相机水平/垂直 FOV。
- 当前模型假设机器人到点后旋转 360°，因此没有单独建模瞬时水平 FOV；也没有建模相机高度、俯仰角、目标高度和垂直 FOV。若任务需要严格三维可见性，这些不是改 YAML 就能解决，需要扩展 `coverage_geometry.py`。
- 安全半径实际为 `max(robot_footprint_radius, camera_radial_offset) + obstacle_clearance`。

### 3.2 ArUco：相机标定与物理量

编辑 `metr4202_aruco/config/aruco.yaml`，但不要把 `fx/fy/cx/cy` 或畸变系数手抄进该文件。当前代码只从 `/camera/camera_info` 读取标定：

- `K = [fx, 0, cx; 0, fy, cy; 0, 0, 1]`
- `D = [k1, k2, p1, p2, ...]`
- 图像宽高和 optical frame 必须与 `/camera/image_raw` 一致。

必须人工确认：

| 参数/数据 | 当前值 | 要求 |
| --- | ---: | --- |
| 相机 `CameraInfo.K/D` | 由相机驱动发布 | 用最终分辨率、镜头和对焦状态标定；确认不是全零默认值 |
| `odom_frame` | `odom` | 必须与底盘连续里程计 frame 一致 |
| 相机 optical-frame TF | 由 URDF/TF 发布 | 平移和旋转必须与最终安装位姿一致，且每个图像时间戳能查到 `odom <- camera` |
| 最终 `map <- odom` TF | 由 SLAM 发布 | Exploration 完成时必须可查询；该次结果会固定为 Coverage 阶段的 ArUco 坐标基准 |
| `marker_length_m` | 0.10 m | 测量黑色 marker 正方形有效边长，不含外部白边 |
| `dictionary` | `DICT_4X4_50` | 必须与打印/仿真 marker 字典一致 |
| `min_marker_perimeter_px` | 40 px | 用实际成像尺寸权衡远距离漏检和噪声误检 |
| `min_depth_m` / `max_depth_m` | 0.15 / 4.0 m | 按可靠 PnP 距离实验确定 |
| `max_reprojection_error_px` | 3.0 px | 用标定后的重投影误差分布确定 |
| `measurement_stddev_m` | 0.08 m | 对已知位置 marker 重复测量，按地图平面误差标准差估计 |

完成相机标定后再调以下检测、批处理和融合参数：

| 参数 | 当前值 | 含义与调整方向 |
| --- | ---: | --- |
| `max_detection_rate_hz` | 0.0 | 0 表示处理每帧；CPU 不足时限制检测频率 |
| `batch_outlier_sigma` | 3.5 | 最终统一重算时的 MAD 倍数；减小会更严格 |
| `batch_outlier_noise_floor_stddevs` | 3.0 | 批量过滤阈值的测量噪声下限倍数，避免正常噪声被 MAD 误删 |
| `confirmation_count` | 3 | marker 成为 confirmed 所需的已接受观测数 |
| `process_noise_m2_per_s` | 0.0001 | 静态 marker 模型随时间增加的不确定度；过小会过度信任旧估计 |
| `innovation_gate_squared` | 9.21 | 二维 Mahalanobis 平方门限；减小会更严格 |
| `min_fusion_interval_s` | 0.20 s | 同 ID 两次融合的最短时间间隔 |
| `min_observer_translation_m` | 0.10 m | confirmed 后，观察位置至少变化多少才接受相关观测 |
| `min_observer_yaw_degrees` | 10° | 或观察 yaw 至少变化多少才接受相关观测 |
| `tf_timeout_s` | 0.10 s | 等待时间戳 TF 的最长时间 |
| `publish_debug_image` | false | 调检测时设 true，发布 `/aruco/debug_image` |

话题名、`odom_frame` 和 `map_frame` 也在 `aruco.yaml` 中可改，但它们属于系统接口匹配项，不是性能调参。先保证单帧 PnP、`odom <- camera` 和最终 `map <- odom` 正确，再调批处理与 Kalman 门限。

检查标定消息：

```bash
ros2 topic echo /camera/camera_info --once
ros2 topic hz /camera/image_raw
ros2 run tf2_ros tf2_echo odom camera_rgb_optical_frame
ros2 run tf2_ros tf2_echo map odom
```

第一条 TF 命令中的相机 frame 应替换为 `CameraInfo.header.frame_id`。若更换分辨率、裁剪、binning、镜头或对焦方式，应重新生成/加载对应 CameraInfo。

### 3.3 其余算法参数

Exploration：

| 参数 | 默认值 | 调整依据 |
| --- | ---: | --- |
| `occupied_threshold` | 50 | 与 SLAM/Nav2 对占用概率的定义保持一致 |
| `frontier_minimum_size` | 5 cells | 增大可抑制小噪声 frontier |
| `obstacle_clearance` | 0.20 m | 至少覆盖机器人半径与定位误差；当前它直接作用于 occupied，不含 unknown |
| `goal_separation` | 0.20 m | 无效目标周围的永久排除半径 |
| `minimum_goal_distance` / `maximum_goal_distance` | 0.50 / 12.0 m | 限制单次目标距离 |
| `candidate_search_radius` | 0.50 m | 从 frontier 向自由空间寻找安全点的最大半径 |
| `progress_map_updates` | 5 | 到点后等待多少次地图更新再判断效果 |
| `progress_known_cell_threshold` | 5 cells | 已知格增长不超过该值即认为无效 |

Coverage 采样与评分：

- `target_spacing=0`：使用地图分辨率作为边界目标间距。
- `candidate_spacing=0.20`：第一层候选视点间距。
- `minimum_candidate_spacing=0`：细化最低到地图分辨率。
- `contour_simplification=0`：自动用 `1.25 × map resolution`。
- `fixed_visit_cost=0.50`：越大越偏好一次覆盖更多目标，越小越偏好近点。

SLAM 参数在 `metr4202_bringup/config/slam_toolbox.yaml`。重点按雷达型号和环境调整 `min_laser_range`、`max_laser_range`、`resolution`、更新频率、运动阈值和闭环门限。

## 4. Launch 可修改配置清单

可用以下命令查看安装后真实参数：

```bash
ros2 launch metr4202_bringup simulation.launch.py --show-args
ros2 launch metr4202_bringup real_robot.launch.py --show-args
ros2 launch metr4202_navigation team13_system.launch.py --show-args
```

### 4.1 仿真总启动 `simulation.launch.py`

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `ros_domain_id` | 环境变量 `ROS_DOMAIN_ID`，否则 0 | DDS 隔离域；同一系统所有终端/机器人必须一致，通常使用 0–232 |
| `world` | TurtleBot3 `turtlebot3_world.world` | Gazebo Classic 世界文件的绝对路径 |
| `robot_model` | `TURTLEBOT3_MODEL`，否则 `waffle_pi` | `burger` / `waffle` / `waffle_pi` |
| `x_pose`、`y_pose` | -2.0、-0.5 | 机器人出生点 |
| `nav2_params_file` | TurtleBot3 对应型号 YAML | Nav2 参数 |
| `slam_params_file` | 自研 `slam_toolbox.yaml` | SLAM 参数 |
| `navigation_params_file` | 自研 `navigation.yaml` | Exploration/Coverage 参数 |
| `aruco_params_file` | 自研 `aruco.yaml` | ArUco 参数 |
| `autostart` | true | Nav2 lifecycle 自动激活 |
| `use_respawn` | false | Nav2 节点异常退出后重启 |
| `log_level` | info | Nav2 日志等级 |
| `start_gazebo_gui`、`start_rviz`、`start_team13` | true | 分别开关 GUI、RViz、自研任务节点 |
| `gazebo_gui_delay` | 3.0 s | gzclient 延迟 |
| `navigation_start_delay` | 6.0 s | SLAM/Nav2 延迟 |
| `rviz_start_delay` | 7.0 s | RViz 延迟 |
| `team13_start_delay` | 10.0 s | 自研节点延迟 |
| `gazebo_gui_software_rendering`、`rviz_software_rendering` | false | WSLg/OpenGL 异常时启用 |
| `controller_frequency`、`planner_frequency` | 8.0、5.0 Hz | 覆写 Nav2 控制与期望规划频率 |

更换仿真地图示例：

```bash
ros2 launch metr4202_bringup simulation.launch.py \
  world:=/absolute/path/to/team13_arena.world \
  x_pose:=0.0 y_pose:=0.0
```

`world` 是 Gazebo 场景，不是 Nav2 的静态 `map.yaml`。本系统默认在线 SLAM，会从新场景重新生成 `/map`。自定义 world 引用的模型还必须能通过 `GAZEBO_MODEL_PATH` 找到。

### 4.2 真机总启动 `real_robot.launch.py`

真机支持与仿真相同的 `ros_domain_id`、三类参数文件、Nav2 频率、RViz/任务开关；此外：

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `robot_model` | `waffle_pi` | 真机型号 |
| `lds_model` | `LDS-02` | `LDS-01` / `LDS-02` / `LDS-03` |
| `usb_port` | `/dev/ttyACM0` | OpenCR 串口 |
| `start_robot_bringup` | true | 驱动若已在机器人计算机启动，则设 false |
| `use_respawn` | true | Nav2 进程恢复 |
| `rviz_start_delay` | 5.0 s | RViz 延迟 |

### 4.3 自研节点启动 `team13_system.launch.py`

- `use_sim_time`：仿真 true，真机 false。
- `navigation_params_file`：Exploration/Coverage YAML。
- `aruco_params_file`：ArUco YAML。

如果只调算法节点，可在已有 SLAM/Nav2/相机系统上单独启动此文件。

## 5. 首次初始化

### 5.1 Windows 先启动 WSL

```powershell
wsl -d Ubuntu-22.04
```

后续命令只在 Ubuntu 终端执行。

### 5.2 安装依赖并构建

以当前工作区为例：

```bash
cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash

sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install --packages-up-to \
  metr4202_interfaces metr4202_aruco metr4202_navigation metr4202_bringup
source install/setup.bash
```

迁移到其他工作区时，复制整个 `metr4202_2026_team13` 到新 workspace 的 `src/`。不要复制旧 workspace 的 `build/`、`install/`、`log/`。

### 5.3 设置 Domain ID 与机器人型号

```bash
export ROS_DOMAIN_ID=13
export TURTLEBOT3_MODEL=waffle_pi
```

同一套系统的机器人计算机、WSL、遥控/调试终端必须使用相同 `ROS_DOMAIN_ID`。也可在总启动命令传 `ros_domain_id:=13`；其他独立终端仍需 export 相同值。

## 6. 算法调整流程

推荐顺序：

1. 先标定相机并验证 `/camera/image_raw`、`/camera/camera_info`、图像时间戳对应的 `odom <- optical frame`，以及 SLAM 的 `map <- odom`。
2. 在 `aruco.yaml` 写入实际 marker 边长和字典，单独验证检测与单帧 PnP。
3. 测量机器人外形与相机光心水平偏置，更新 `navigation.yaml` 的 Coverage 几何参数。
4. 在实际照明与速度下测视觉距离/角度范围，再更新可见性参数。
5. 调 Exploration，确认能完整建图且不会反复访问无收益 frontier。
6. 调 Coverage 的采样间距与 `fixed_visit_cost`，观察 `/coverage/status_map`。
7. 最后调 ArUco 多帧融合参数和 Nav2/SLAM。

YAML 修改后无需改 Python，但必须重新构建或显式传源码 YAML 的绝对路径。默认 launch 读取的是 `install/` 中已安装的文件：

```bash
cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  metr4202_navigation metr4202_aruco metr4202_bringup
source install/setup.bash
```

临时试参示例：

```bash
ros2 launch metr4202_bringup simulation.launch.py \
  navigation_params_file:=/absolute/path/to/navigation_test.yaml \
  aruco_params_file:=/absolute/path/to/aruco_test.yaml
```

## 7. 启动

### 7.1 仿真分阶段验证

先只验证 Gazebo、SLAM、Nav2 与 RViz：

```bash
cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=13
export TURTLEBOT3_MODEL=waffle_pi

ros2 launch metr4202_bringup simulation.launch.py \
  ros_domain_id:=13 start_team13:=false
```

确认 `/scan`、`/map`、TF 与 Nav2 正常后，按 `Ctrl+C`，启动完整系统：

```bash
ros2 launch metr4202_bringup simulation.launch.py ros_domain_id:=13
```

无界面运行：

```bash
ros2 launch metr4202_bringup simulation.launch.py \
  ros_domain_id:=13 start_gazebo_gui:=false start_rviz:=false
```

WSLg 图形异常：

```bash
ros2 launch metr4202_bringup simulation.launch.py \
  ros_domain_id:=13 \
  gazebo_gui_software_rendering:=true rviz_software_rendering:=true
```

### 7.2 真机

先确认硬件、相机和 TF：

```bash
ls -l /dev/ttyACM0
ros2 topic echo /camera/camera_info --once
```

启动：

```bash
cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=13
export TURTLEBOT3_MODEL=waffle_pi
export LDS_MODEL=LDS-02

ros2 launch metr4202_bringup real_robot.launch.py \
  ros_domain_id:=13 usb_port:=/dev/ttyACM0
```

若底盘/雷达/相机驱动已在机器人计算机运行：

```bash
ros2 launch metr4202_bringup real_robot.launch.py \
  ros_domain_id:=13 start_robot_bringup:=false
```

### 7.3 只启动自研节点

外部 SLAM、Nav2、TF 和相机已就绪时：

```bash
ros2 launch metr4202_navigation team13_system.launch.py \
  use_sim_time:=false \
  navigation_params_file:=/absolute/path/to/navigation.yaml \
  aruco_params_file:=/absolute/path/to/aruco.yaml
```

单独测试 ArUco 两阶段流程时，可在积累观测后手动触发一次统一重算：

```bash
ros2 service call /aruco/finalize_exploration std_srvs/srv/Trigger "{}"
ros2 service call /aruco/get_marker_map metr4202_interfaces/srv/GetArucoMap "{}"
```

正常完整任务由 Mission Manager 自动调用 finalize，不要在 Exploration 尚未结束时人工触发。

## 8. 启动后检查

另开 Ubuntu 终端，加载相同环境与 Domain ID：

```bash
cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=13

ros2 node list
ros2 topic hz /scan
ros2 topic hz /map
ros2 topic hz /camera/image_raw
ros2 topic echo /camera/camera_info --once
ros2 topic echo /aruco/detections --once
ros2 topic echo /aruco/marker_map --once
ros2 topic echo /coverage/status_map --once
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
```

服务与 action：

```bash
ros2 service list | grep -E 'exploration|get_next_task|recovery|aruco'
ros2 action list | grep -E 'navigate_to_pose|spin'
```

预期：

- `/controller_server` 与 `/planner_server` 为 active。
- `/scan`、`/map`、`/camera/image_raw` 持续有数据。
- 完整启动后存在 `exploration`、`coverage`、`recovery`、`mission_manager`、`aruco_detection`、`aruco_localization`。
- ArUco 图像与 CameraInfo 的宽高、frame_id 匹配。
- Exploration 完成后日志出现 `Reprojected ... saved observations`，随后 Coverage initialized；marker 记录不会被清空。

## 9. 测试与静态检查

```bash
cd ~/turtlebot3_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon test --packages-select \
  metr4202_interfaces metr4202_aruco metr4202_navigation metr4202_bringup
colcon test-result --verbose
```

仅检查 Python 与 launch 语法：

```bash
python3 -m compileall \
  src/metr4202_2026_team13/metr4202_aruco \
  src/metr4202_2026_team13/metr4202_navigation \
  src/metr4202_2026_team13/metr4202_bringup
```

## 10. 常见问题与当前限制

- 没有 `/camera/camera_info`：ArUco detections 会被丢弃，先修相机驱动/标定加载。
- Image 与 CameraInfo 宽高或 frame 不一致：定位节点会拒绝整帧。
- 查不到图像时间戳对应 `odom <- camera`：检查仿真时钟、`use_sim_time`、连续 odom、静态相机 TF 和 TF 缓存。
- Finalize 失败：确认 Exploration 完成时可查询最新的 `map <- odom`。
- Coverage 初始化失败：确认最终地图非空、机器人在地图内，且安全膨胀后仍有连通自由空间。
- Coverage 显示大量 initial-uncoverable：检查相机距离/入射角是否过严、候选采样是否过稀、unknown 是否挡住视线。
- Exploration 提前结束：检查 frontier 最小簇、净距、候选搜索半径和最大目标距离。
- Nav2 失败后目标被跳过：这是当前 Recovery 占位行为，不是完整恢复策略。
- ArUco marker 当前不会改变 Exploration 或 Coverage 目标；Coverage 只会继承并继续融合探索阶段的 marker 估计。
- Coverage 使用 INIT 时的冻结地图；覆盖阶段的新障碍不会重建可见性图，动态安全仍依赖 Nav2 costmap。
- Domain ID 修改后，所有独立终端都必须同步，否则会表现为节点/话题“消失”。

正常停止时，在总 launch 终端按一次 `Ctrl+C` 并等待所有子进程退出。
