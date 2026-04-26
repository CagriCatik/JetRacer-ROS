# Safety Layer (Proposed)

> [!NOTE]
> Collision safety features are **partially implemented** (a skeleton node has been added to `scripts/collision_safety_node.py`). Full hardware validation is pending.

## Safety Requirements
For a real mobile robot, safety is paramount. The `collision_safety_node` (and motor driver) should implement:
1. **Command Timeout**: The motor node must stop the robot if no velocity command is received within 0.5 seconds.
2. **Stop on Shutdown**: Ensure the robot halts when the ROS node terminates.
3. **Throttle Limits**: Maximum forward and reverse throttle limits must be enforced via `config/calibration.yaml`.
4. **Emergency Stop**: A subscriber to `/safety/emergency_stop` (Bool) that immediately halts the vehicle.

## Twist Mux Arbitration
The safety node outputs to `/cmd_vel_safety`, which should be configured in `twist_mux.yaml` with the **highest priority** (Priority 1) over teleop, navigation, and lane following.
