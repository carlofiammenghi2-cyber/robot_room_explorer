# USI Angry Turtle — ROS 2 Homework

A ROS 2 controller that makes **turtle1** autonomously write **"USI"** on the
Turtlesim canvas and react to intruders.

## Features

| # | Feature | Description |
|---|---------|-------------|
| 1 | **Write "USI"** | turtle1 follows waypoints to draw the letters U, S and I |
| 2 | **Chase intruder** | If turtle2 enters within `k_chase` metres, turtle1 becomes *angry* and pursues it |
| 3 | **Kill & respawn** | When turtle1 gets within `k_kill` metres it calls `/kill`, then `/spawn` to recreate turtle2 at a random position. States: `WRITING → ANGRY → RETURNING` |
| 4 | **Predictive interception** | turtle1 aims `m` metres ahead of turtle2 (`m = gain × v_target × distance`) instead of directly at it |

## Prerequisites

* ROS 2 (tested with Humble / Iron / Jazzy)
* `turtlesim` package

## How to Run

### 1. Build the package

```bash
cd ~/Desktop/2nd_semester/robotics/ros/my_ros2_project
colcon build --packages-select hw1_usi_turtle
source install/setup.bash
```

### 2. Start Turtlesim

```bash
ros2 run turtlesim turtlesim_node
```

### 3. Launch the controller

```bash
ros2 run hw1_usi_turtle usi_turtle
```

turtle1 will start drawing "USI" and turtle2 will be **spawned automatically**.

### 4. Test the chase (optional — teleoperate turtle2)

In a new terminal:

```bash
ros2 run turtlesim turtle_teleop_key --ros-args -r /turtle1/cmd_vel:=/turtle2/cmd_vel
```

Drive turtle2 close to turtle1 to trigger the angry behaviour.

## ROS Parameters

All parameters can be set at launch or changed at runtime:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `k_chase` | 3.5 | Distance (m) that triggers the chase |
| `k_kill` | 0.5 | Distance (m) at which turtle2 is killed |
| `chase_speed` | 4.0 | Max linear speed while chasing |
| `write_speed` | 2.0 | Max linear speed while writing |
| `intercept_gain` | 0.8 | Gain for predictive lookahead (`m = gain × v × d`) |

Example — launch with custom thresholds:

```bash
ros2 run hw1_usi_turtle usi_turtle --ros-args \
    -p k_chase:=4.0 \
    -p k_kill:=0.3 \
    -p chase_speed:=5.0
```

## State Machine

```
WRITING ──(turtle2 < k_chase)──▶ ANGRY
ANGRY   ──(turtle2 < k_kill)───▶ RETURNING  (kill + respawn)
RETURNING ──(reached saved wp)──▶ WRITING
```

## Author

Carlo Fiammenghi
