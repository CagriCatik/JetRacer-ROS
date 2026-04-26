# Software Setup

This guide walks you through setting up the software environment for the Waveshare JetRacer ROS AI Kit on the Jetson Nano.

## OS & ROS Version Verification

Ensure your environment matches the target platform:
```bash
lsb_release -a
# Expected: Ubuntu 18.04.x LTS (bionic)

rosversion -d
# Expected: melodic
```

## System Dependencies

Update your system and install necessary ROS Melodic tools:
```bash
sudo apt update
sudo apt install ros-melodic-desktop-full
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
```

## Python 3 & External Dependencies
While ROS Melodic relies on Python 2, the original repository scripts (`racecar.py`, `teleop_gamepad.py`, `camera.py`) use Python 3. You must install the ROS/Python 3 bridge packages and external pip dependencies to run them:

```bash
# Install Python 3 ROS bridges
sudo apt install python3-pip python3-yaml python3-rospkg

# Install hidden dependencies required by the scripts
pip3 install pygame getkey flask traitlets
```
> [!WARNING]
> Without installing `python3-rospkg`, attempting to run `rosrun jetracer racecar.py` will result in `ImportError: No module named rospy`.

Additionally, you must install the Waveshare/NVIDIA `jetracer` library as instructed by the manufacturer to support `racecar.py` I2C communication.

## Workspace Setup

Create a catkin workspace and build the `jetracer` ROS package:

```bash
# Create the workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Clone the repository
git clone <repo-url>

cd ~/catkin_ws

# Initialize and update rosdep
sudo rosdep init
rosdep update

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
catkin_make

# Source the workspace
source devel/setup.bash
```
