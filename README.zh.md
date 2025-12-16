GO2 ROS2 项目说明（中文）

本文档记录了本仓库中与 Unitree GO2 四足机器人相关的 ROS2 包、构建与运行步骤、依赖关系、启动配置和开发说明的技术细节。目标读者为熟悉 ROS2 的开发者或系统集成工程师。

## 项目概述

本仓库包含若干 ROS2 包，用于 Unitree GO2 机器人的描述、驱动、导航与可视化。主要模块：

- `go2_driver`：GO2 机器人的驱动组件，使用 `rclcpp`、`rclcpp_lifecycle` 与组件化接口暴露机器人控制与传感器数据。
- `go2_description`：机器人 URDF/xacro 文件、Meshes、以及用于仿真（Gazebo）的配置文件。
- `go2_navigation`：基于 Nav2 的导航配置、行为树与成本地图设置，提供 mapping 与导航 launch 文件。
- `go2_rviz`：RViz 预置配置文件与启动脚本。
- 其他与集成相关的包：`go2_bringup`（系统级启动）、`go2_robot`（元包/组织配置）、`unitree_api` 与 `unitree_go`（与 Unitree 官方 SDK/消息交互）、`go2_interfaces`（自定义 ROS2 接口消息/服务）、`livox_ros_driver2`（Livox 激光雷达驱动）等。

## 硬件与软件要求

- 推荐板载计算机：NVIDIA Jetson（仓库 README 中提到 Jetson Nano），用于在机器人上运行 ROS2 节点。
- 激光雷达：LIVOX MID-360（仓库中包含 `livox_ros_driver2`）。
- 推荐 ROS2 版本：仓库原始 README 提到 `ros2 foxy`（注意：Foxy 对应 ROS2 发行版，若在其他发行版上使用，需检查兼容性）。
- 编译工具：`colcon`/`ament_cmake`、CMake >= 3.5/3.8（各包 CMake 最低要求不同）。

## 代码结构（高层）

- 顶层包与作用：
  - `go2_driver/` — robot driver implementation (C++), depends on `rclcpp`, `rclcpp_lifecycle`, `rclcpp_components`, `unitree_api`, `unitree_go`, `go2_interfaces`。
  - `go2_description/` — xacro/URDF, meshes, gazebo configs, depends on `xacro`。
  - `go2_navigation/` — nav2 configs, launch files and behavior trees.
  - `go2_rviz/` — RViz configuration.
  - `livox_ros_driver2/` — Livox LiDAR driver sources and launch files.
  - `go2_bringup/` — system bringup launch and integration.

## 关键依赖（从 package.xml / CMakeLists.txt 抽取）

- build tool: `ament_cmake`。
- core ROS2 libraries: `rclcpp`, `rclcpp_lifecycle`, `rclcpp_components`。
- messages: `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `tf2_ros`。
- project packages: `unitree_go`, `unitree_api`, `go2_interfaces`。
- third-party drivers: `livox_ros_driver2`（仓库内含 driver）。

注意：某些包在 `package.xml` 中标注了 `test_depend`（如 `ament_lint_auto`、`ament_lint_common`），用于代码质量检查。

## 构建说明

- 在包含本仓库的 ROS2 工作空间的 `src/` 目录下执行：

```bash
cd <your_ros2_ws>
colcon build
source install/setup.bash
```

- 在 Windows 上开发时请注意：本仓库主要面向 Linux（嵌入式 Jetson）环境，Windows 只能用于代码编写与静态检查，不能直接运行机器人/仿真组件。

## 运行与启动

- 示例：启动导航 mapping（根据顶层 README）

```bash
ros2 launch go2_navigation mapping.launch.py rviz:=True
```

- 常用启动文件位置：
  - `go2_bringup/launch/go2.launch.py` — 系统级启动。
  - `go2_driver/launch/go2_driver.launch.py` — 驱动组件启动。
  - `go2_navigation/launch/mapping.launch.py`、`navigation.launch.py` — 导航与建图。
  - `go2_rviz/launch/rviz.launch.py` — 启动 RViz 与预置视图。

## 驱动节点实现细节（`go2_driver`）

- 语言：C++（`C++14`），构建系统：`ament_cmake`。
- 组件化：`go2_driver` 编译为一个可作为 `rclcpp_components` 的共享库，并通过 `rclcpp_components_register_nodes` 注册节点 `go2_driver::Go2Driver`。
- 依赖：`unitree_api` 与 `unitree_go` 提供与 Unitree 硬件/SDK 的接口（需配套原厂 SDK 或本仓库给出的实现）。

## 机器人描述（`go2_description`）

- 使用 `xacro` 组织 URDF 模块（`xacro/` 下有 `robot.xacro`、`leg.xacro` 等），并提供 `go2_description.urdf.xacro` 作为组合入口。
- 提供 Gazebo 插件与 `gazebo.urdf.xacro` 用于仿真场景。

## 导航配置（`go2_navigation`）

- 使用 Nav2（请在目标 ROS2 发行版中确认 Nav2 版本兼容性）。
- `config/` 包含 `amcl_config.yaml`、`nav2_config.yaml`、`costmap_config.yaml`、`behavior_tree.xml` 等，控制定位与路径规划行为。

## Livox 雷达集成

- 仓库包含 `livox_ros_driver2`，带有驱动实现与 launch 文件，适配 Livox MID 系列雷达（例如 MID-360）。请参考包内 README 和 `launch` 目录获取初始化参数与 TF 配置。

## 开发与贡献

- 代码风格与静态检查：部分包启用了 `ament_lint_auto` / `ament_lint_common`，建议在提交 PR 前运行这些检查。
- 提交流程： Fork → feature 分支 → PR（包含变更说明、测试说明与运行截图/日志）

## 已知注意事项与建议

- ROS2 发行版兼容性：仓库原始 README 提到 `ros2 foxy`，如果使用 Humble/其他发行版需要逐包确认 API 与 Nav2 兼容性并修改 `package.xml` 与依赖版本。
- Jetson 平台：在 Jetson Nano 上部署请注意交叉编译、CUDA/硬件驱动与 JetPack 版本、以及实时性/性能调优。
- unitree 官方 SDK：若需要直接控制硬件，请确保 `unitree_api` / `unitree_go` 与所用固件版本匹配。

## 许可证

- 各包内 `package.xml`、`LICENSE` 文件中可能包含具体许可条款，例如 `go2_driver` 与 `go2_description` 标注为 Apache-2.0。合并或发布前请逐包核查 LICENSE 文件。

## 参考与文件位置

- 顶层 README: [README.md](README.md)
- `go2_driver` 包定义: [go2_driver/package.xml](go2_driver/package.xml)
- `go2_navigation` 包定义: [go2_navigation/package.xml](go2_navigation/package.xml)
- `go2_description` 包定义: [go2_description/package.xml](go2_description/package.xml)

---

如果你希望我把 README.zh.md 调整为更详细的安装指南（例如为 Jetson Nano 编写特定步骤、交叉编译说明或 Nav2 版本兼容性表），告诉我需要的深度，我会继续完善。

## 命令行示例与常用运行组合

- 完整构建与启动（在 Linux + ROS2 工作空间）:

```bash
cd <your_ros2_ws>
colcon build --symlink-install
source install/setup.bash
ros2 launch go2_bringup go2.launch.py rviz:=True lidar:=False realsense:=False
```

- 仅运行导航（使用已保存地图）:

```bash
ros2 launch go2_navigation navigation.launch.py rviz:=True map_file:=/path/to/map.yaml
```

- 启动建图（SLAM）并打开 RViz:

```bash
ros2 launch go2_navigation mapping.launch.py rviz:=True slam_enable:=True
```

## Launch 参数清单（默认值与说明）

下面列出仓库内主要 `launch` 文件所声明的参数、默认值与简要说明，方便定制运行。

- `go2_bringup/launch/go2.launch.py`:
  - `lidar` (default: `False`): 是否启动外部 lidar 驱动（例如 Hesai），可选 `True/False`。
  - `realsense` (default: `False`): 是否启动 RealSense 摄像头驱动，可选 `True/False`。
  - `rviz` (default: `False`): 是否启动 RViz 视图。

- `go2_description/launch/robot.launch.py`:
  - `description_package` (default: `go2_description`): 包含 robot URDF/xacro 的包名。
  - `description_file` (default: `go2_description.urdf`): URDF/XACRO 文件名称（相对于包的 `urdf/` 目录）。
  - `prefix` (default: ``): 发布到 TF 的 frame 前缀（用于命名空间或多机器人场景）。
  - `use_sim_time` (default: `True`): 是否使用仿真时间（Gazebo）。

- `go2_navigation/launch/mapping.launch.py`:
  - `rviz` (default: `False`): 是否启动 RViz。
  - `nav2_params_file` (default: `go2_navigation/config/costmap_config.yaml`): Nav2 参数文件的完整路径（可传入自定义 `yaml`）。
  - `slam_enable` (default: `True`): 是否启用 SLAM（mapping）模块；如果为 `False`，则通常运行定位/导航流程。

- `go2_navigation/launch/navigation.launch.py`:
  - `rviz` (default: `False`): 是否启动 RViz。
  - `map_file` (default: `/home/unitree/go2_ws1/map/map.yaml`): 用于定位的地图文件路径（yaml）。
  - `nav2_params_file` (default: `go2_navigation/config/costmap_config.yaml`): Nav2 参数文件路径。
  - `slam_enable` (default: `True`): 是否启用 SLAM。注意：navigation.launch.py 通常用于定位/导航，应将其设为 `False` 并传入 `map_file`。

### 其他运行时参数（在 launch 中以 `Node(parameters=[...])` 形式设置）

- `pointcloud_to_laserscan` 节点（在 `mapping.launch.py` / `navigation.launch.py` 中）参数概述：
  - `target_frame`: `livox_frame`（点云转换到的目标 frame）。
  - `transform_tolerance`: `0.01`。
  - `min_height`: `0.10`~`0.15`（根据 launch 文件有所不同，建议为 `0.10-0.15m`）。
  - `max_height`: `1.0`~`1.35`（取决于车辆高度与感兴趣区域）。
  - `angle_min` / `angle_max`: `-3.14` / `3.14`（rad）。
  - `angle_increment`: `0.0087`（rad，大约 0.5 度）。
  - `range_min` / `range_max`: `0.1` / `100.0`（m）。
  - `use_intensities`: `False`。
  - `concurrency_level`: `1`~`2`。
  - `use_sim_time`: 通常由 `use_sim_time` launch 参数控制。

## 说明与建议

- 当你需要为真实机器人运行（非仿真）时，将 `use_sim_time` 设为 `False`，并确保 `nav2` 的 `params_file` 中 `use_sim_time` 也被正确覆盖。
- 若同时接入 Livox、Hesai、RealSense 等多个传感器，先单独验证各驱动节点的 TF/Topic 输出，再在 `go2_bringup` 中组合启动以确保重映射（remap）规则正确。

