# Calibration

> [!NOTE]
> Configuration templates have been provided in `jetracer_ros/config/calibration.yaml`. 

## Calibration Parameters
For accurate teleop and autonomous driving, hardware variations require calibration. Do not hardcode these values in the Python scripts.

* **Steering Trim (`steering.trim`)**: Corrects the mechanical offset of the steering servo so that a command of 0.0 results in straight driving.
* **Throttle Deadband (`throttle.deadband`)**: The minimum PWM signal required to overcome motor friction.
* **Max Limits (`throttle.max_forward`, `throttle.max_reverse`)**: Caps the maximum physical speed to ensure safety during testing.

## How to Calibrate
1. Place the JetRacer on a block so the wheels are off the ground.
2. Publish manual Float32 commands to the `/steering` topic until the wheels point perfectly straight. Record this value as `steering.trim`.
3. Publish gradually increasing `/throttle` values until the wheels just begin to spin smoothly. Record this as `throttle.deadband`.
