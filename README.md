

### 总体结构

- `firmware/`：ESP32 侧 microROS 固件骨架（C++）
- `ros2_ws/`：ROS 2 Humble 工作空间示例
  - `maze_nav_bringup/`：整体启动、参数与代价地图配置
  - `maze_nav_fsm/`：有限状态机与阶段切换
  - `maze_nav_planner/`：基于 A* 的全局路径规划
  - `maze_nav_vision/`：蓝色胶带软障碍检测（OpenCV）

本仓库仅包含与论文关键思想对应的核心参考代码，硬件驱动及部分底层细节需结合实际设备与官方 SDK 补全。

### 依赖环境（建议）

- Ubuntu 22.04 + ROS 2 Humble
- micro-ROS + ESP-IDF（或 Arduino-ESP32）开发环境
- Python 3.10+
- OpenCV (`opencv-python`)、`numpy`

### 使用方式（概要）

1. 在宿主机上创建并构建 `ros2_ws`：
   - 将 `ros2_ws` 复制到实际 ROS 2 工作空间下
   - 执行 `colcon build` 并 `source install/setup.bash`
2. 在 ESP32 上编译并烧录 `firmware` 中的 microROS 固件，使其通过 UDP 与 ROS 2 侧 agent 通信。
3. 启动 `maze_nav_bringup` 提供的启动文件，加载 FSM、A* 规划器、视觉节点以及导航相关参数。



