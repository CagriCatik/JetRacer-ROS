# Robot Odometer Calibration

## Introduction

The robot can output odometer information to track its traveled distance and turning angles. The default program may have inaccuracies, which can be corrected by calibrating the odometer. This guide will help you achieve higher accuracy, although initial operations can proceed without calibration if precision is not crucial.

## Step 1: Calibrate the Linear Velocity

### 1. Enable the Chassis Node

1. Connect to the robot via SSH and open the terminal.
2. Enter the following command to enable the chassis node:

   ```sh
   roslaunch jetracer jetracer.launch
   ```

   **Note:** Ensure the chassis node is not enabled multiple times, as it may cause errors.

### 2. Enable Keyboard Control

1. Press `Ctrl + Alt + T` to open a new terminal.
2. Enter the following command to enable keyboard control of the topic nodes:

   ```sh
   roslaunch jetracer keyboard.launch
   ```

### 3. Control the Robot

1. Press the `I` key on the keyboard to move the robot forward.
2. Observe if the robot moves in a straight line. If it does not, proceed to the next step to calibrate.

### 4. Use rqt_reconfigure for Calibration

1. Open a new terminal and run:

   ```sh
   rosrun rqt_reconfigure rqt_reconfigure
   ```

2. Adjust the `servo_bias` parameter until the robot moves straight.
3. Save the calibration:

   ```sh
   vi ~/catkin_ws/src/jetracer_ros/cfg/jetracer.cfg
   ```

   Modify the `servo_bias` line with the new value.

## Step 2: Linear Velocity Calibration

### 1. Close Previous Chassis Node

1. Ensure the previously started chassis node is closed while keeping the robot master node running.

### 2. Start Linear Velocity Calibration

1. Open a terminal in Jetson Nano and run:

   ```sh
   roslaunch jetracer calibrate_linear.launch
   ```

   Do not close this terminal to keep the calibration option visible.

### 3. Open Dynamic Parameter Configuration

1. In the virtual machine, run:

   ```sh
   rosrun rqt_reconfigure rqt_reconfigure
   ```

2. Set the following parameters:
   - `test_distance`: Default is 1m.
   - `speed`: Robot's linear speed.
   - `tolerance`: Error margin to reach the goal.
   - `odom_linear_scale_correction`: Adjust this based on test results.

### 4. Perform the Test

1. Place the robot on the ground and mark its starting position.
2. Check `start_test` to initiate calibration.
3. Measure the actual distance traveled and calculate the correction ratio.
4. Adjust `odom_linear_scale_correction` by multiplying it with the ratio and repeat until the distance is accurate.

### 5. Save Calibration Parameters

1. Open the file to save the new parameters:

   ```sh
   vi ~/catkin_ws/src/jetracer_ros/launch/jetracer.launch
   ```

2. Modify the `linear_correction` value.

## Step 3: Angular Velocity Calibration

### 1. Initial Setup

1. Unplug the motor wire to keep the motor in a free state.
2. Modify the parameters to set servo control to `y = x`:

   ```sh
   vi ~/catkin_ws/src/jetracer_ros/cfg/jetracer.cfg
   ```

   Set `servo_bias` to 0.

   ```sh
   vi ~/catkin_ws/src/jetracer_ros/launch/jetracer.launch
   ```

   Set `coefficient_a`, `coefficient_b`, `coefficient_d` to 0, and `coefficient_c` to 1.

### 2. Enable the Chassis Node

1. Connect via SSH and run:

   ```sh
   roslaunch jetracer jetracer.launch
   ```

### 3. Enable Topic Publishing

1. Run the following command:

   ```sh
   rosrun rqt_publisher rqt_publisher
   ```

2. Add `/cmd_vel` topic information.

### 4. Measure Turning Diameter

1. Set `/cmd_vel.angular.z` to `-1` and gradually increase to `1`, recording the turning diameter at each value.

### 5. Fit the Polynomial

1. Update the program with actual measurements:

   ```sh
   vi ~/catkin_ws/src/jetracer_ros/scripts/servo_calibratin.py
   ```

2. Run the calibration script:

   ```sh
   python ~/catkin_ws/src/jetracer_ros/scripts/servo_calibratin.py
   ```

### 6. Save Fitting Parameters

1. Open the launch file and save the new parameters:

   ```sh
   vi ~/catkin_ws/src/jetracer_ros/launch/jetracer.launch
   ```

2. Convert scientific notation to floating-point numbers if necessary.

## Conclusion

After successful calibration, the robot should move in a straight line and handle turns accurately. Adjust the `servo_bias` or `d` parameter if further fine-tuning is required.
