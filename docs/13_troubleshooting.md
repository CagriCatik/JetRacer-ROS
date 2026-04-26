# Troubleshooting

## Hardware Issues

### Nano Reboots Unexpectedly
- **Cause**: Power draw spikes, especially when motors engage or the GPU initializes.
- **Solution**: Ensure your battery pack is fully charged. Verify that the power adapter provides at least 5V/4A.

### Camera Not Detected
- **Cause**: Ribbon cable loose or backwards.
- **Solution**: Check the CSI ribbon cable connection on both the Jetson Nano and the camera module. The blue side of the cable should face the Ethernet port on the Nano.

## Software Issues

### `pygame.error: video system not initialized`
- **Cause**: Attempting to run `teleop_gamepad.py` over an SSH session without X11 forwarding. `pygame` requires a graphical display environment to capture inputs.
- **Solution**: Run the script directly on the Jetson Nano with a monitor attached, or switch to the standard `joy_node` (`sensor_msgs/Joy`) for headless teleoperation.

### `ImportError: No module named rospy`
- **Cause**: Using a Python 3 shebang (`#!/usr/bin/env python3`) on Ubuntu 18.04 without installing the ROS/Python3 compatibility bridge.
- **Solution**: Run `sudo apt install python3-rospkg python3-yaml`.

### `jetracer.launch` Fails Immediately
- **Cause**: The `jetracer` C++ node requires an MCU to be connected to the serial port `/dev/ttyACM0`.
- **Solution**: Check if your expansion board utilizes the serial protocol. If it is standard Waveshare (I2C only), do not use `jetracer.launch`. Use `rosrun jetracer racecar.py` instead. If you do have a serial MCU, ensure your user is in the `dialout` group (`sudo usermod -a -G dialout $USER`) and `/dev/ttyACM0` exists.

If you encounter issues that aren't listed here, please check the [Cytron Technical Forum](https://forum.cytron.io/).
