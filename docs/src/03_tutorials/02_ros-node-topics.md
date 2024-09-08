# Viewing Node Topics with ROS

This comprehensive tutorial will guide you through the process of viewing node topics with ROS using the JetRacer ROS AI Kit. The steps include checking node topics, displaying data waveforms, adjusting PID parameters, viewing robot odometer information, understanding coordinate relationships, and viewing IMU information.

## Prerequisites

1. Ensure you have completed the previous tutorials on assembling the JetRacer, installing the Jetson Nano image, installing the Ubuntu virtual image, configuring multi-machine communication, and controlling robot movement.
2. Ensure your JetRacer and computer are connected to the same WiFi network.

## Step 1: Check Node Topics

1. **Start the Robot Chassis Node:**
   - Connect to the JetRacer via SSH to open the terminal.
   - Enter the following command to start the robot chassis node:

     ```bash
     roslaunch jetracer jetracer.launch
     ```

     **Note:** The robot chassis node cannot be restarted repeatedly. If the previous chassis node is not closed, it does not need to be restarted.

2. **View Chassis Node Topics:**
   - Open the terminal in the Ubuntu virtual machine.
   - Enter the following command to view the topics of the chassis node:

     ```bash
     rostopic list
     ```

     If the topics are displayed correctly, it means that multi-device communication is successful. If not, check the IP and hostname configurations and ensure both devices are on the same WiFi network.

## Step 2: Display Data Waveform via `rqt_plot`

1. **Open `rqt_plot`:**
   - In the virtual machine terminal, run the following command to open the data waveform display tool:

     ```bash
     rosrun rqt_plot rqt_plot
     ```

2. **Add Motor Topics:**
   - Enter the motor topic (e.g., `/motor/lset` or `/motor/rset`) in the `/Topic` field and click `+` to add it.
   - Control the robot using the keyboard or remote control handle and observe the changes in the motor curve. This can help diagnose motor operation issues.

3. **Adjust Display Settings:**
   - If the display speed is too fast, adjust the X-axis and Y-axis range settings by clicking the appropriate icon.

## Step 3: Adjust PID Parameters via `rqt_reconfigure`

1. **Start Dynamic Parameter Adjustment Interface:**
   - Run the following command in the virtual machine terminal:

     ```bash
     rosrun rqt_reconfigure rqt_reconfigure
     ```

2. **Adjust PID Parameters:**
   - Select `JetRacer` and adjust the PID parameters as needed. Start with `Ki`, then `Kp`, and finally `Kd` (or set `Kd` to 0 initially).
   - Adjustments will only affect the current session. To make permanent changes, save the parameters to the configuration file (`jetracer/cfg/jetracer.cfg`).

   **Note:** Non-professionals should avoid adjusting PID parameters to prevent abnormal robot behavior or damage.

## Step 4: View Robot Odometer Information via RVIZ

1. **Start RVIZ:**
   - In the virtual machine terminal, run the following command:

     ```bash
     rviz
     ```

2. **Add TF Components:**
   - Add TF components and set the Fixed Frame to `odom`. You should see the `odom`, `base_footprint`, and `base_imu_link` coordinates.
   - Move the robot forward and observe the coordinate changes.

   **Note:** If `base_footprint` rotates, restart the chassis master with the robot in a static state to recalibrate the IMU.

## Step 5: View Coordinate Relationship via TF Tree

1. **Run TF Tree Command:**
   - Re-open a terminal in the virtual machine and run the following command:

     ```bash
     rosrun rqt_tf_tree rqt_tf_tree
     ```

2. **Understand Coordinate Transformations:**
   - The `odom->base_footprint` transformation is issued by the `/robot_pose_ekf` node.
   - The `base_footprint->base_imu_link` transformation is issued by the `/base_footprint_to_imu` node.

## Step 6: View Robot IMU Information via RVIZ

1. **Start RVIZ:**
   - In the virtual machine terminal, run the following command:

     ```bash
     rviz
     ```

2. **Add IMU Component:**
   - Add the `rviz_plugin_tutorials/Imu` component and select the `/imu` topic.
   - Enable the box and axes displays, and set the acceleration vector scale to 0.2.

3. **Observe IMU Data:**
   - Rotate or tilt the robot to observe changes in the IMU data displayed in RVIZ.

   **Note:** Ensure the IMU is calibrated correctly by keeping the robot still and horizontal during startup. Restart the chassis node if necessary to recalibrate.

By following these steps, you can effectively view and analyze the node topics, data waveforms, PID parameters, odometer information, coordinate relationships, and IMU data for your JetRacer ROS AI Kit.
