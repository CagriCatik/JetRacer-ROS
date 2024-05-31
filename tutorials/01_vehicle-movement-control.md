
# Vehicle Movement Control

## Step 1: Add Serial User Group

1. **Check Driver Board Connection**
   Enter the command `ls /dev` on the Jetson Nano to check whether the driver board is connected to Jetson Nano normally and whether ttyACM0 and ttyACM1 devices are found.
   - `ttyACM0` is used to communicate with the microcontroller.
   - `ttyACM1` is used to communicate with the radar.

   ```bash
   ls /dev
   ```

2. **Add the Serial Port User Group**
   Enter the following commands to add the serial port user group. If you are using a configured system with already added permissions, you can skip this step.

   ```bash
   ls -l /dev/ttyACM*            # View the serial port user group as dialout
   id -Gn                        # View the user groups that the current user belongs to, the first one is the current user
   sudo adduser jetbot dialout   # Add the current user to the user group dialout where the serial port is located
   ```

3. **Restart Jetson Nano**
   Enter the command `sudo reboot` to restart. The password is `jetson`.

   ```bash
   sudo reboot   # Restart Jetson Nano
   ```

   **Note**: The addition of the user to the dialout group takes effect only after restarting. Without serial port permissions, the robot chassis node will report an error after booting up.

## Step 2: Start the Robot Chassis Control Node

1. **Start the Robot Master Node**
   Connect to the robot via SSH, open the terminal, and enter the following command to start the robot master node.

   ```bash
   roscore   # Start the robot master node
   ```

2. **Start the Robot Chassis Node**
   Enter the following command to start the robot chassis node. The command will also automatically start the master node if it’s not already running.

   ```bash
   roslaunch jetracer jetracer.launch   # Start the robot chassis node
   ```

3. **Verify Multi-Machine Communication**
   Open the Ubuntu virtual machine terminal and enter the following command to verify that the multi-machine communication connection is normal.

   ```bash
   rostopic list
   ```

   If the expected topics appear, multi-device communication is successful. If not, recheck the IP and hostname configurations and ensure they are connected to the same WiFi.

   - `/cmd_vel`: Robot movement control topic.
   - `/imu`: Robot IMU topic.
   - `/motor/*`: Actual encoded speed and set speed of the left and right motors.
   - `/odom`: Robot odometer.
   - `/odom_combined`: Robot fusion odometer, combining encoded odometer with IMU data.

## Step 3: Start Topic Publishing Node Control

1. **Start the Topic Publishing Node**
   Keep the robot chassis node running and place the car on the ground. Open a terminal in the Ubuntu virtual machine and enter the following command.

   ```bash
   rosrun rqt_publisher rqt_publisher   # Start topic publishing node
   ```

2. **Publish a Topic**
   In the pop-up window, select the topic `/cmd_vel` and click "+" to create a new topic. Configure the following:
   - `linear->x`: Linear speed of the robot (-1.2m/s to 1.2m/s)
   - `angular->z`: Steering angle of the robot’s front tires (-0.6 radians to 0.6 radians)

   Change `linear.x` to 0.5, right-click, and select "Publish selected once" to make the robot move forward 0.5 meters. Setting `linear.x` to 0 and `angular.z` to 0.6 will make the robot turn left.

   **Note**: Ensure to select the `Twist` type for `/cmd_vel`.

3. **Close the Topic Publishing Node**
   Press `Ctrl+C` to close the topic publishing node.

4. **View Topic Information**
   Re-open a terminal on the robot side and enter the following command to view the topic information of the car movement.

   ```bash
   rostopic echo /cmd_vel   # Display /cmd_vel topic
   ```

   **Special Note**: `angular->z` is the steering angle of the robot's tires, not the angular velocity.

## Step 4: Control the Robot Movement with Keyboard

1. **Open Keyboard Control Topic Node**
   Press `Ctrl+Alt+t` on the Ubuntu virtual machine to open a new terminal and enter the following command.

   ```bash
   roslaunch jetracer keyboard.launch
   ```

2. **Control the Robot**
   Use the arrow keys to control the robot’s movement. The movement data can be viewed in the topics received by the robot terminal.

3. **Interrupt Node Running**
   Press `Ctrl+C` to stop the node.

   **Note**: The controls are only valid when the robot master node and the robot chassis node are activated. If the robot does not respond, check the node operation and multi-machine communication.

## Step 5: Connect the Virtual Machine to the Gamepad Control

1. **Check Gamepad Connection**
   Connect the USB receiver of the gamepad to the computer. A box will pop up, select "Connect to Virtual Machine -> Ubuntu JetRacer."

2. **Test Gamepad Connection**
   Run the following command in the virtual machine to check if the gamepad is connected.

   ```bash
   ls /dev/input/
   ```

   `js0` represents the game joystick handle. Run the following command to test the game handle.

   ```bash
   jstest /dev/input/js0
   ```

   Press different buttons on the remote control handle to verify the corresponding button values change.

3. **Start Gamepad Control Node**
   Enter the following command in the virtual machine to start the gamepad control node.

   ```bash
   roslaunch jetracer joy.launch
   ```

4. **Control the Robot with Gamepad**
   Turn on the power of the handle, press the HOME button, and a red light will turn on. Press and hold the L1 button while controlling the left joystick for steering and the right joystick for forward and backward movement. Release the L1 key to stop the robot.

   **Note**: If the remote control fails, open the program file to adjust the values.

   ```bash
   sudo nano ~/catkin_ws/src/jetracer_ros/scripts/teleop_joy.py   # Open the remote control program file
   ```
