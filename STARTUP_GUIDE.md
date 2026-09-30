# METR4202 Team 13 启动说明

本文档适用于 ROS 2 Humble 与 Ubuntu 22.04。源码包不依赖固定用户名或工作区路径，可复制到任意 ROS 2 workspace 的 `src` 目录。所有 ROS 命令均在 Ubuntu 终端内执行。

## 1. 已验证环境

| 组件 | 当前版本 |
| --- | --- |
| Windows Subsystem for Linux | WSL 2.7.11.0，WSL2 内核 6.18.33.2 |
| Ubuntu | 22.04.5 LTS |
| ROS 2 | Humble |
| Gazebo | Gazebo Classic 11.10.2 |
| Nav2 | 1.1.20 |
| SLAM Toolbox | 2.6.10 |
| RViz2 | 11.2.27 |
| Python / NumPy / OpenCV | 3.10.12 / 1.21.5 / 4.5.4 |
| 图形环境 | WSLg，Mesa 23.2.1，D3D12 AMD Radeon 610M |
| TurtleBot3 源码 | `humble` 分支，约 2.3.7～2.3.8 |

启动文件已按上述 ROS 2 Humble、Gazebo Classic 11、OpenCV 4.5 及 WSLg 环境适配。

## 2. 迁移到新的 workspace

以下以 `~/team13_ws` 为例；可替换为任意 workspace 路径：

```bash
mkdir -p ~/team13_ws/src
cp -a /path/to/metr4202_2026_team13 ~/team13_ws/src/
cd ~/team13_ws
source /opt/ros/humble/setup.bash

sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source install/setup.bash
```

需要迁移的是整个 `metr4202_2026_team13` 目录，其中包含四个相互依赖的 ROS 包：`metr4202_interfaces`、`metr4202_aruco`、`metr4202_navigation` 和 `metr4202_bringup`。不要复制原 workspace 的 `build`、`install` 或 `log` 目录。

## 3. 每次启动前

先从 Windows 启动 Ubuntu：

```powershell
wsl -d Ubuntu-22.04
```

然后只在 Ubuntu 终端中执行：

```bash
cd ~/team13_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
```

`waffle_pi` 带相机，完整 ArUco 功能应使用此型号。

## 4. 首次或源码修改后构建

```bash
cd ~/team13_ws
source /opt/ros/humble/setup.bash
rosdep check --from-paths src --ignore-src
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select \
  metr4202_interfaces metr4202_aruco metr4202_navigation metr4202_bringup
colcon test-result --verbose
```

若 `rosdep check` 仅提示系统已安装依赖，继续构建即可；不要混用其他 ROS 发行版的环境。

## 5. 推荐的可视化启动

先进行基础界面验证，不启动自动任务：

```bash
ros2 launch metr4202_bringup simulation.launch.py start_team13:=false
```

应看到 Gazebo 世界、TurtleBot3 Waffle Pi 和 RViz。确认界面可用后，按 `Ctrl+C` 停止，再启动完整系统：

```bash
ros2 launch metr4202_bringup simulation.launch.py
```

完整启动包含：

- Gazebo 服务端和延迟启动的图形客户端
- TurtleBot3 状态发布、模型生成及相机/雷达插件
- SLAM Toolbox 在线建图
- Nav2 导航栈
- RViz2
- 探索、覆盖、恢复和 ArUco 检测/定位节点

启动文件默认等待 3 秒再打开 Gazebo，等待 6 秒再启动 SLAM/Nav2，等待 7 秒再打开 RViz，等待 10 秒再启动 Team 13 任务节点。这样可确保机器人、传感器和 TF 已经就绪。

## 6. 常用启动参数

```bash
# 仅后台运行，无 Gazebo/RViz 界面
ros2 launch metr4202_bringup simulation.launch.py \
  start_gazebo_gui:=false start_rviz:=false

# WSLg 图形异常时强制软件渲染
ros2 launch metr4202_bringup simulation.launch.py \
  gazebo_gui_software_rendering:=true rviz_software_rendering:=true

# 慢机器延长界面等待时间
ros2 launch metr4202_bringup simulation.launch.py \
  gazebo_gui_delay:=8.0 navigation_start_delay:=10.0 \
  rviz_start_delay:=12.0 team13_start_delay:=15.0

# 单独验证仿真、建图和导航，不启动 Team 13 自动任务
ros2 launch metr4202_bringup simulation.launch.py start_team13:=false
```

Nav2 默认使用适合当前 WSL2 调度环境的 `controller_frequency:=8.0` 和 `planner_frequency:=5.0`。如果主机负载较高，可降至 5 和 2：

```bash
ros2 launch metr4202_bringup simulation.launch.py \
  controller_frequency:=5.0 planner_frequency:=2.0
```

## 7. 启动后的功能检查

另开一个 Ubuntu 终端，并重新加载环境：

```bash
cd ~/team13_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

检查关键节点和话题：

```bash
ros2 node list
ros2 topic list | grep -E '^/(map|scan|odom|camera/image_raw|aruco/detections)$'
ros2 topic hz /scan
ros2 topic hz /map
ros2 topic hz /camera/image_raw
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
```

预期 `/controller_server`、`/planner_server` 为 `active`；`/scan`、`/map`、`/camera/image_raw` 持续有数据。运行完整任务时还应看到探索、覆盖、恢复和 ArUco 相关节点。

## 8. 界面故障排查

先检查 WSLg 和 OpenGL：

```bash
printf 'DISPLAY=%s WAYLAND_DISPLAY=%s XDG_RUNTIME_DIR=%s\n' \
  "$DISPLAY" "$WAYLAND_DISPLAY" "$XDG_RUNTIME_DIR"
glxinfo -B
```

至少 `DISPLAY` 或 `WAYLAND_DISPLAY` 应有值。若两者都为空，请退出当前终端，从 Windows 重新执行 `wsl -d Ubuntu-22.04`。启动文件会在这种情况下直接给出明确错误，不再静默失败。

如果 Gazebo 黑屏、材质加载失败或窗口退出：

```bash
ros2 launch metr4202_bringup simulation.launch.py \
  gazebo_gui_software_rendering:=true
```

如果 RViz 黑屏或出现持续的 OpenGL 着色器错误：

```bash
ros2 launch metr4202_bringup simulation.launch.py \
  rviz_software_rendering:=true
```

RViz 在收到第一帧地图时可能输出一次 `active samplers with a different type`。这是 RViz 地图插件的已知单次日志；如果后续仍出现 `Trying to create a map` 且地图、雷达和 TF 正常更新，它不代表启动失败。软件渲染也可能保留该单次日志。

当前安装的 RViz2 是 11.2.27，而软件源已提供 Humble 11.2.29。升级需要用户输入 Ubuntu 的 sudo 密码：

```bash
sudo apt-get install --only-upgrade \
  ros-humble-rviz2 ros-humble-rviz-common \
  ros-humble-rviz-default-plugins ros-humble-rviz-rendering
```

查看最近一次 ROS 日志：

```bash
ls -td ~/.ros/log/* | head -n 1
grep -RniE 'error|failed|exception|segmentation' "$(ls -td ~/.ros/log/* | head -n 1)"
```

## 9. 真机启动

连接 TurtleBot3 后先确认串口存在：

```bash
ls -l /dev/ttyACM0
```

然后执行：

```bash
export TURTLEBOT3_MODEL=waffle_pi
export LDS_MODEL=LDS-02
ros2 launch metr4202_bringup real_robot.launch.py usb_port:=/dev/ttyACM0
```

如果硬件驱动已在机器人计算机上运行，本机只启动导航和应用：

```bash
ros2 launch metr4202_bringup real_robot.launch.py \
  start_robot_bringup:=false
```

真机启动同样支持 `start_rviz`、`rviz_start_delay`、`rviz_software_rendering`、`controller_frequency` 和 `planner_frequency` 参数。

## 10. 正常停止

在启动终端按一次 `Ctrl+C`，等待所有进程依次退出。若界面关闭但终端中的 `gzserver`、Nav2 或任务节点仍运行，说明只是 GUI 被关闭；回到启动终端按 `Ctrl+C` 才会停止整套系统。

在当前 WSLg、Mesa 23.2.1 与 RViz2 11.2.27 组合下，RViz 正常运行后在整套系统收到 `Ctrl+C` 时偶尔会以 `exit code -11` 结束。这是退出阶段的图形上下文清理问题；只要它发生在用户主动停止之后，而运行期间地图持续更新，就不表示启动或导航失败。
