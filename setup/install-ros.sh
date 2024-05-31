#!/bin/bash

# Step 1: Set up Jetson Nano
echo "Setting up Jetson Nano..."

# Jetbot program library URL
JETBOT_REPO="https://github.com/NVIDIA-AI-IOT/jetracer"
JETCARD_IMAGE="jetcard_nano-4gb-jp451.zip"

# Checking if required tools are installed
command -v ssh >/dev/null 2>&1 || { echo >&2 "SSH is required but it's not installed. Aborting."; exit 1; }
command -v nmcli >/dev/null 2>&1 || { echo >&2 "nmcli is required but it's not installed. Aborting."; exit 1; }
command -v ifconfig >/dev/null 2>&1 || { echo >&2 "ifconfig is required but it's not installed. Aborting."; exit 1; }

echo "Please insert your SD card and power on your Jetson Nano. Connect the network cable and log in via SSH."

# Connect to WiFi
read -p "Enter your WiFi SSID: " ssid_name
read -sp "Enter your WiFi Password: " password
echo

sudo nmcli device wifi connect "$ssid_name" password "$password"
sudo ifconfig

# Step 2: Configure ROS Software Repository
echo "Configuring ROS software repository..."
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Step 3: Install ROS Melodic
echo "Installing ROS Melodic..."
sudo apt-get update
sudo apt install -y ros-melodic-desktop-full

# Step 4: Add Environment Variables and Install ROS Dependencies
echo "Adding environment variables and installing ROS dependencies..."
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential

# Step 5: Initialize rosdep
echo "Initializing rosdep..."
sudo apt install -y python-rosdep
sudo rosdep init
rosdep update

# Step 6: Verify the ROS Environment
echo "Verifying the ROS environment..."
roscore &

# Step 7: Set up Workspace
echo "Setting up catkin workspace..."
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
sudo sh -c 'echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc'
source ~/.bashrc

# Step 8: Download & Compile JetRacer ROS
echo "Downloading and compiling JetRacer ROS..."
cd ~/catkin_ws/src
git clone https://github.com/waveshare/jetracer_ros.git
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash

# Step 9: Install Dependency Libraries
echo "Installing dependency libraries..."
sudo apt-get install -y ros-melodic-robot-pose-ekf ros-melodic-gmapping ros-melodic-hector-slam ros-melodic-slam-karto ros-melodic-cartographer-ros ros-melodic-navigation ros-melodic-teb-local-planner ros-melodic-audio-common

# Step 10: Install Camera Function Package
echo "Installing camera function package..."
sudo apt-get install -y gstreamer1.0-tools libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev

cd ~/catkin_ws/src
git clone https://github.com/peter-moran/jetson_csi_cam.git 
git clone https://github.com/ros-drivers/gscam.git
cd gscam
sed -e "s/EXTRA_CMAKE_FLAGS = -DUSE_ROSBUILD:BOOL=1$/EXTRA_CMAKE_FLAGS = -DUSE_ROSBUILD:BOOL=1 -DGSTREAMER_VERSION_1_x=On/" -i Makefile
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash

# Optional: Improve camera image color if needed
echo "If the IMX219-160 camera image is too red, the color will be corrected..."
wget https://files.waveshare.com/upload/e/eb/Camera_overrides.tar.gz
tar zxvf Camera_overrides.tar.gz 
sudo cp camera_overrides.isp /var/nvidia/nvcam/settings/
sudo chmod 664 /var/nvidia/nvcam/settings/camera_overrides.isp
sudo chown root:root /var/nvidia/nvcam/settings/camera_overrides.isp

# Step 11: Install the Lidar Function Package
echo "Installing Lidar function package..."
cd ~/catkin_ws/src
git clone https://github.com/Slamtec/rplidar_ros.git
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash

echo "ROS environment setup for Jetbot robot is mostly complete. Additional settings are required for intelligent voice functions if needed."

echo "Setup finished."
