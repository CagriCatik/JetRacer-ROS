# Gamepad & Xbox Controller Setup

This guide explains how to use an Xbox controller (or any standard USB/Bluetooth gamepad) to teleoperate the JetRacer.

## 1. Connecting the Controller

### USB Connection
Simply connect your controller to one of the USB ports on the Jetson Nano.

### Bluetooth Connection
1. Put your controller in pairing mode.
2. On the Jetson Nano, open the terminal and run:
   ```bash
   sudo bluetoothctl
   # Inside bluetoothctl:
   agent on
   default-agent
   scan on
   # Find your controller's MAC address (e.g., XX:XX:XX:XX:XX:XX)
   pair XX:XX:XX:XX:XX:XX
   trust XX:XX:XX:XX:XX:XX
   connect XX:XX:XX:XX:XX:XX
   exit
   ```

## 2. Choosing a Teleop Method

The repository provides two different nodes for gamepad control. Choose the one that matches your hardware driver.

### Method A: Pygame Method (For I2C Direct Driver)
Use this if you are using the standard Waveshare setup with `racecar_node.py` (which subscribes to `/throttle` and `/steering`).

*   **Node**: `teleop_gamepad_node.py`
*   **Dependency**: `pip3 install pygame`
*   **How to run**:
    ```bash
    # Terminal 1: Start the driver
    rosrun jetracer_ros racecar_node.py
    
    # Terminal 2: Start the teleop
    rosrun jetracer_ros teleop_gamepad_node.py
    ```
*   **Default Controls**:
    *   **Throttle**: Left Thumbstick (Vertical)
    *   **Steering**: Right Thumbstick (Horizontal)

### Method B: ROS Joy Method (For Serial MCU Driver)
Use this if you are using the serial MCU setup with `jetracer` (which subscribes to `/cmd_vel`).

*   **Launch File**: `joy.launch`
*   **Dependency**: `sudo apt install ros-melodic-joy`
*   **How to run**:
    ```bash
    roslaunch jetracer_ros joy.launch
    ```
*   **Controls**:
    *   **Deadman Switch**: You MUST hold down **Button 6** (usually the "Back" or "Select" button) to enable movement.
    *   **Throttle**: Right Thumbstick (Vertical)
    *   **Steering**: Left Thumbstick (Horizontal)

## 3. Troubleshooting

### Permission Denied
If the node cannot access the controller, ensure your user is in the `input` and `dialout` groups:
```bash
sudo usermod -a -G input $USER
sudo usermod -a -G dialout $USER
# Log out and log back in for changes to take effect.
```

### Checking Device Index
If you have multiple gamepads, you may need to change the device index in the scripts:
- In `teleop_gamepad_node.py`: `pygame.joystick.Joystick(0)`
- In `joy.launch`: `<param name="dev" type="string" value="/dev/input/js0" />`
