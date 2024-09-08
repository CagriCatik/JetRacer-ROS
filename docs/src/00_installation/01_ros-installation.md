# JetRacer ROS AI Kit Advanced Tutorial V: Install ROS System on Jetson Nano & Environment Configuration

## Introduction

This guide provides comprehensive instructions for installing the ROS system and configuring the environment on a Jetson Nano. While using the pre-configured image provided is recommended, these steps will help you set up the system manually. Ensure a stable internet connection throughout the installation and configuration process. The tutorial uses Ubuntu 18.04 and ROS Melodic.

## Step 1: Jetson Nano Programming System

1. **Download Jetbot Image:**
   - Jetbot website: [Jetbot GitHub](https://github.com/NVIDIA-AI-IOT/jetracer)
   - 4G image version: `jetcard_nano-4gb-jp451.zip`

2. **Prepare SD Card:**
   - Use an SD card with at least 64GB capacity.
   - Insert the SD card into the Jetson Nano.
   - Connect the power to enable the device.

3. **Network Configuration:**
   - Connect the network cable and log in via SSH.
   - Scan available WiFi networks:

     ```bash
     sudo nmcli device wifi list
     ```

   - Connect to WiFi:

     ```bash
     sudo nmcli device wifi connect <ssid_name> password <password>
     ```

   - Query the IP of wlan0 on the WiFi interface:

     ```bash
     ifconfig
     ```

## Step 2: Configure ROS Software Repository

1. **Add ROS repository:**

   ```bash
   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   ```

2. **Add repository keys:**

   ```bash
   sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
   ```

## Step 3: Install ROS Melodic

1. **Update source list:**

   ```bash
   sudo apt-get update
   ```

2. **Install Desktop-Full version of ROS:**

   ```bash
   sudo apt install ros-melodic-desktop-full
   ```

## Step 4: Add Environment Variables and Install ROS Dependencies

1. **Add environment variables:**

   ```bash
   echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

2. **Install dependencies:**

   ```bash
   sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
   ```

## Step 5: Initialize `rosdep`

1. **Install `rosdep`:**

   ```bash
   sudo apt install python-rosdep
   ```

2. **Initialize and update `rosdep`:**

   ```bash
   sudo rosdep init
   rosdep update
   ```

## Step 6: Verify the ROS Environment

1. **Start ROS Master:**

   ```bash
   roscore
   ```

## Step 7: Set up Workspace

1. **Create and compile catkin workspace:**

   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws
   catkin_make
   sudo sh -c 'echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc'
   source ~/.bashrc
   ```

## Step 8: Download & Compile JetRacer ROS

1. **Clone JetRacer ROS repository and compile:**

   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/waveshare/jetracer_ros.git
   cd ~/catkin_ws
   catkin_make
   source ~/catkin_ws/devel/setup.bash
   ```

## Step 9: Install Dependency Libraries

1. **Install various ROS libraries:**

   ```bash
   sudo apt-get install ros-melodic-robot-pose-ekf ros-melodic-gmapping ros-melodic-hector-slam ros-melodic-slam-karto ros-melodic-cartographer-ros ros-melodic-navigation ros-melodic-teb-local-planner ros-melodic-audio-common
   ```

## Step 10: Install Camera Function Package

1. **Install GStreamer and related packages:**

   ```bash
   sudo apt-get install gstreamer1.0-tools libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev
   ```

2. **Clone camera repositories and compile:**

   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/peter-moran/jetson_csi_cam.git 
   git clone https://github.com/ros-drivers/gscam.git
   cd gscam
   sed -e "s/EXTRA_CMAKE_FLAGS = -DUSE_ROSBUILD:BOOL=1$/EXTRA_CMAKE_FLAGS = -DUSE_ROSBUILD:BOOL=1 -DGSTREAMER_VERSION_1_x=On/" -i Makefile
   cd ~/catkin_ws
   catkin_make
   source ~/catkin_ws/devel/setup.bash
   ```

3. **Adjust camera settings if the image is too red:**

   ```bash
   wget https://files.waveshare.com/upload/e/eb/Camera_overrides.tar.gz
   tar zxvf Camera_overrides.tar.gz 
   sudo cp camera_overrides.isp /var/nvidia/nvcam/settings/
   sudo chmod 664 /var/nvidia/nvcam/settings/camera_overrides.isp
   sudo chown root:root /var/nvidia/nvcam/settings/camera_overrides.isp
   ```

## Step 11: Install the Lidar Function Package

1. **Clone Lidar repository and compile:**

   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/Slamtec/rplidar_ros.git
   cd ~/catkin_ws
   catkin_make
   source ~/catkin_ws/devel/setup.bash
   ```

By now, the ROS environment of the Jetbot robot should be mostly completed. If you need an intelligent voice function, additional settings will be required.
