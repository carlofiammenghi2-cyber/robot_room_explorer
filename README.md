## Repository Structure

The workspace is organized into the following ROS 2 packages:

1. **`room_explorer` (Main Project):**
   * A dedicated package for SLAM mapping, Nav2 autonomous navigation, reactive distance-based ToF exploration, and simulator bridges (`coppeliasim_odom` & `lidar_bridge`) for the Mecanum-wheeled Robomaster EP robot simulated in CoppeliaSim.
   *  **For detailed, step-by-step instructions on launching Mapping, Nav2 Navigation, and Reactive Exploration, please see the [room_explorer README](room_explorer/README.md).**
   
2. **`robomaster_driver` (Robot SDK Driver):**
   * The low-level robot driver encapsulating the DJI Robomaster SDK, real-robot connection utilities (`client`, `ftp`, `discover`), camera H264 decoders, battery indicator, and physical launch scripts. Fully decoupled from high-level navigation.

3. **`hw1_usi_turtle`:**
   * Homework 1 assignment implementing simple turtle simulation control.

4. **`robomaster_description` & `robomaster_msgs`:**
   * URDF description models, meshes, custom interfaces, and message types for the DJI Robomaster EP robot.

---

## Quick Build & Run Instructions

To compile the entire workspace, open your terminal at the root of this repository and run:

```bash
# 1. Compile all packages in the workspace
colcon build --symlink-install

# 2. Source the setup script
source install/setup.zsh
