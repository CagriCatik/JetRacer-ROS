# jetracer_ros

The primary ROS 1 Melodic package for the Waveshare JetRacer AI Kit.

## Package Purpose
This package provides hardware bringup, teleoperation, and educational examples for the JetRacer platform.

## Dual Hardware Warning
This package contains two distinct driver paths depending on your hardware:
1. **Serial MCU (C++)**: Subscribes to `/cmd_vel` and expects an MCU on `/dev/ttyACM0`. Launched via `jetracer.launch`.
2. **I2C Direct (Python)**: Subscribes to `/throttle` and `/steering` and expects standard Waveshare PCA9685 hardware via the external `jetracer` library. Launched via `rosrun jetracer racecar.py`.

## Discovered Nodes (Implemented in `nodes/`)
- `jetracer` (C++): Primary `/cmd_vel` to `/dev/ttyACM0` serial bridge. Outputs `/odom` and `/imu`.
- `racecar_node.py` (Python): Direct I2C driver subscribing to `/throttle` and `/steering` Float32s.
- `teleop_node.py` / `teleop_gamepad_node.py`: Converts gamepad/keyboard inputs.
- `lane_following_node.py`: OpenCV-based BEV lane detection. Publishes `/cmd_vel_lane`.
- `joint_state_node.py`: Publisher for URDF joint states.
- `odom_ekf_node.py`, `multipoint_nav_node.py`: Odometry and multi-point navigation scripts.
- *Voice/Audio nodes*: `face_detect_node.py`, `color_tracking_node.py`, `line_follow_node.py`.

## Utility Scripts (Implemented in `scripts/`)
- `camera.py`: **NOT a ROS Node**. Runs a standalone Flask server on port 5000.
- *Voice/Audio scripts*: `aiui.py`, `tts_en.py`.

## Missing / Planned Nodes (Proposed Architecture)
To align with a standard ROS autonomous driving stack, the following nodes are proposed (TODO) but **not currently implemented**:
- `cmd_vel_to_jetracer_control_node.py`
- `semantic_behavior_node.py`
- `collision_safety_node.py`
- `object_detection_node.py`
- `twist_mux`

## Topics (Implemented)
| Topic | Type | Direction | Description |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Subscribed | Velocity command for C++ `jetracer` node |
| `/throttle` | `std_msgs/Float32` | Subscribed | Raw throttle command to `racecar.py` |
| `/steering` | `std_msgs/Float32` | Subscribed | Raw steering command to `racecar.py` |
| `/odom_raw` | `nav_msgs/Odometry` | Published | Odometry data from C++ `jetracer` node |
| `/imu` | `sensor_msgs/Imu` | Published | IMU data from C++ `jetracer` node |

## Dependencies
- Base ROS: `rospy`, `roscpp`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `tf`.
- **Hidden External Python Dependencies**: `pygame` (for teleop_gamepad), `getkey` (for teleop), `flask` (for camera), and NVIDIA's `jetracer`.

## Run Examples
```bash
# If using Serial MCU:
roslaunch jetracer jetracer.launch

# If using direct I2C hardware:
roscore
rosrun jetracer racecar.py
```
