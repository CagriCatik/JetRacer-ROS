# ROS Melodic Setup

This guide walks you through setting up ROS 1 Melodic Morenia for the JetRacer.

## 1. Install ROS Melodic
Follow the official ROS Wiki instructions to install `ros-melodic-desktop-full` on Ubuntu 18.04:
[ROS Melodic Installation Instructions](http://wiki.ros.org/melodic/Installation/Ubuntu)

```bash
# Example basic installation commands
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-melodic-desktop-full
```

## 2. ROS Dependencies
Install standard ROS build and dependency resolution tools:
```bash
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
```

## 3. Catkin Workspace Setup
Create a catkin workspace. **The repository must be cloned into `src/`**, not the root of the workspace.

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone <this-repo-url>
```

## 4. Building the Workspace
Use `rosdep` to automatically install any missing dependencies defined in the `package.xml`:

```bash
cd ~/catkin_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

## 5. Verification
Confirm that ROS Melodic is active and your package is found:
```bash
echo $ROS_DISTRO
# Expected: melodic

rosversion -d
# Expected: melodic

rospack find jetracer_ros
# Expected: /home/username/catkin_ws/src/jetracer_ros
```
