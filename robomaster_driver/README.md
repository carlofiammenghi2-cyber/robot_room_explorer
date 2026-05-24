# DJI RoboMaster EP ROS 2 Driver Package

This ROS 2 package, `robomaster_driver`, provides the core hardware interface, driver nodes, and launch utilities to communicate with the physical **DJI RoboMaster EP** mobile robot using the official DJI RoboMaster Python SDK.

To maintain clean modularity and follow ROS 2 architecture principles, the low-level hardware communication is completely decoupled from high-level navigation stacks (which reside in `room_explorer`).

---

## 📂 Package Structure

* `launch/`: Contains the launch files to start the driver and connection tools on the real robot:
  * `main.launch`: The primary launcher to start the Robomaster driver node and bring up all hardware modules.
  * `ep.launch`: Shortcut launcher to start the driver configured for the RoboMaster EP model.
  * `s1.launch`: Shortcut launcher to start the driver configured for the RoboMaster S1 model.
  * `camera.launch`: Launches the camera interface node.
  * `decoder.launch`: Starts the PyAV H264 hardware-accelerated video decoding node.
  * `teleop.launch` / `teleop_linux.launch`: Starts the joystick-based teleoperation node.
* `config/`: Holds hardware calibration and controller profiles:
  * `joy_config_ep.yaml` / `joy_config_s1.yaml`: Button mappings for joystick teleoperation.
  * `servos.yaml`: Configuration parameters for robotic arm servos.
  * `rm_camera_calibration_*.yaml`: Factory camera lens calibration files (360p, 540p, 720p).
* `robomaster_driver/`: Python modules implementing the RoboMaster ROS client and services:
  * `client.py` & `robomaster_driver.py`: Encapsulates the main `RoboMasterROS` node.
  * `ftp.py` & `action.py`: Handles media transfers and robot SDK actions.
  * `discover.py`: Utility to discover active RoboMaster robots on the local WiFi network.
  * `modules/`: DJI SDK hardware modules (arm, chassis, gripper, gimbal, speaker, camera, led, tof).

---

## 🚀 How to Run (Real Robot Connection)

Ensure your computer is connected to the same WiFi network as the RoboMaster EP (Station Mode) or connected directly to the robot's built-in access point (AP Mode).

### 1. Discover Robots on the Network
To scan the network and obtain the robot's IP and Serial Number:
```bash
ros2 run robomaster_driver discover
```

### 2. Launch the Robot Driver
To bring up the driver node and initialize all hardware modules (arm, chassis, camera, speaker, ToF):
```bash
ros2 launch robomaster_driver ep.launch
```

### 3. Manual Joystick Teleoperation
If you have a joystick/gamepad connected and want to drive the real robot:
```bash
ros2 launch robomaster_driver teleop.launch
```
