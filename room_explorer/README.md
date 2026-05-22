# Autonomous Mapping and Navigation of a Robomaster EP in CoppeliaSim using ROS 2

This ROS 2 package, `room_explorer`, implements a complete mapping and autonomous navigation pipeline for a Robomaster EP mobile robot equipped with holonomic Mecanum wheels inside the CoppeliaSim simulation environment. 

This repository was developed by **Carlo Fiammenghi** and **Manuel Tagliaferri** for the Robotics Course (USI, A.Y. 2025/26) under the supervision of **Professor Alessandro Giusti**.

---

## 🛠️ Prerequisites & Installation

### 1. Requirements
* **ROS 2** (Humble Hawksbill or compatible desktop version).
* **CoppeliaSim** (tested on version 4.X, using the required room scenes).
* **Nav2 Suite** & **SLAM Toolbox**:
  ```bash
  sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox
  ```

### 2. Workspace Compilation
Navigate to the root directory of your ROS 2 workspace and compile the package:
```bash
# Clean build and compile
colcon build --symlink-install --packages-select room_explorer

# Source the workspace setup
# (Use setup.bash if you are running Bash instead of Zsh)
source install/setup.zsh
```

---

## 🚀 Step-by-Step Testing Guide for TAs

Before running any ROS 2 launcher, **open CoppeliaSim** and load the room environment scene:
* `old_scene.ttt` (for mapping evaluation)
* `room_scene.ttt` (for navigation and exploration evaluation)
Make sure to click the **Start Simulation** button in CoppeliaSim.

---

### Step 1: 🗺️ 2D SLAM Mapping Demonstration
This launcher starts the coordinate bridges, the LiDAR conversion bridge, the RViz visualizer, and the SLAM toolbox mapping node.

To launch the mapping node, run:
```bash
ros2 launch room_explorer ep_lidar_mapping.launch.py sync:=true
```

#### Synchronous vs. Asynchronous SLAM Toggles:
* **Synchronous Mode (Recommended):** Set `sync:=true` (default). Processes 100% of scans. If the computer experiences load, the simulation time will scale down, ensuring perfectly orthogonal $90^\circ$ walls even during fast teleoperation.
* **Asynchronous Mode:** Set `sync:=false`. Drops intermediate scans to keep real-time constraints. Fast rotations might cause scan-matching drops and duplicate walls.

#### Manual Teleoperation:
To drive the robot manually and explore the room to build the map, run keyboard teleop on another terminal:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/rm0/cmd_vel
```
*Use keys `u, i, o, j, k, l, m` to drive. Drive slowly during rotations to ensure optimal scan matching.*

#### Saving the Map:
Once the map is complete in RViz, save it to the configuration directory:
```bash
ros2 run nav2_map_server map_saver_cli -f src/room_explorer/config/mappa_casa
```

---

### Step 2: 🎯 Autonomous Navigation (Nav2 Stack)
This launcher loads a pre-saved static map (`mappa_casa.yaml`), initializes the Costmaps (Global & Local), and starts the AMCL particle-filter localization and Nav2 planning stack.

To launch navigation, run:
```bash
ros2 launch room_explorer ep_nav2.launch.py
```

#### 1. AMCL Initialization (RViz):
1. In the open RViz window, click the **"2D Pose Estimate"** tool at the top bar.
2. Click at the bottom of the map (inside the long vertical corridor/room where the Robomaster EP is located in CoppeliaSim).
3. **Drag the arrow** in the direction the robot is facing (pointing upward) and release.
4. You will see the cyan LiDAR points perfectly align with the black walls, and the green particle cloud concentrate around the robot.

#### 2. Goal Navigation:
1. Click the **"Nav2 Goal"** tool at the top bar in RViz.
2. Click anywhere in the mapped rooms (e.g., the top-right bedroom or the main kitchen room).
3. The global planner will compute the shortest path using **Dijkstra's algorithm** (shown as a thin line).
4. The local DWB controller will immediately send velocity commands to the Mecanum wheels. The robot will translate sideways and turn autonomously to follow the path, avoiding obstacles without collisions.

---

### Step 3: 🔄 Reactive Explorer (Wander Mode)
If you want to test the robot's ability to autonomously explore unknown rooms reactively **without using a map or a localization stack**, you can test our custom reactive wall-follower node.

1. Start CoppeliaSim and the baseline robot sensors launcher:
   ```bash
   ros2 launch room_explorer ep_lidar_mapping.launch.py sync:=false
   ```
2. Run our reactive ToF exploration node:
   ```bash
   ros2 run room_explorer room_explorer_node
   ```

#### How it works:
* The node queries distance readings from the robot's Time-of-Flight (ToF) proximity sensors at a loop frequency of $10\text{ Hz}$.
* If the frontal path is clear, it drives forward.
* If an obstacle is detected, it compares the lateral clearances ($d_{\text{left}}$ vs. $d_{\text{right}}$) and steers the chassis towards the side with more free space. It can escape tight dead ends reactively.

---

## 📂 Package Directory Structure
* `launch/`: Contains the launch scripts (`ep_lidar_mapping.launch.py`, `ep_nav2.launch.py`, etc.).
* `room_explorer/`: Python source code containing:
  * `room_explorer_node.py`: Reactive ToF wander explorer.
  * `lidar_bridge.py`: Translates raw CoppeliaSim laser signals to standard ROS LaserScans.
  * `coppeliasim_odom.py`: Publishes ground-truth TF odometry frames with correct coordinate offsets.
* `config/`: Contains RViz layouts, SLAM toolbox configurations, and pre-saved room maps (`mappa_casa.yaml`, `mappa_casa.pgm`).
* `scenes/`: Contains CoppeliaSim `.ttt` simulator files.
