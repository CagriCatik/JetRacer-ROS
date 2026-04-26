<div align="center">
  <h1>JetRacer ROS AI Kit</h1>
  <p>
    <img src="https://img.shields.io/badge/ROS-Melodic-blue" alt="ROS Melodic">
    <img src="https://img.shields.io/badge/Ubuntu-18.04-orange" alt="Ubuntu 18.04">
    <img src="https://img.shields.io/badge/Platform-Jetson%20Nano-green" alt="Jetson Nano">
    <img src="https://img.shields.io/badge/Python-2.7%20%2F%203.6-blue" alt="Python 2.7 / 3.6">
  </p>
  <p>This package provides ROS bringup and educational autonomy examples for the Waveshare JetRacer ROS AI Kit.</p>
</div>

<hr>

## Compatibility Matrix

| Component | Version |
|---|---|
| Jetson board | NVIDIA Jetson Nano Developer Kit B01, 4GB |
| JetPack | 4.4 / 4.4.x |
| Ubuntu | 18.04 LTS Bionic |
| ROS | ROS 1 Melodic |
| Python default | Python 2 for many ROS Melodic tools |

> [!WARNING]  
> This project strictly targets **ROS 1 Melodic** on **Ubuntu 18.04 Bionic**. Do not attempt to build this project on ROS Noetic (Ubuntu 20.04) or convert it to ROS 2 without explicitly migrating all node dependencies and Python 2 scripts.

## Repository Layout

```text
JetRacer-ROS/
├── README.md               # This file
├── LICENSE                 # Licensing information
├── .gitignore              # Git ignore rules
├── docs/                   # Setup guides, hardware info, and tutorials
└── jetracer_ros/           # The ROS workspace package
    ├── package.xml
    ├── CMakeLists.txt
    ├── launch/
    ├── config/
    ├── scripts/
    ├── src/
    └── ...
```

## Setup & Installation

Please clone this repository into a Catkin Workspace, not directly into `~/catkin_ws`.

### 1. Verification
Verify your Jetson environment before starting:
```bash
lsb_release -a
uname -m
echo $ROS_DISTRO
rosversion -d
```
*Expected: Ubuntu 18.04 Bionic, aarch64, melodic.*

### 2. Workspace Setup
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Clone this repository
git clone <repo-url>

cd ~/catkin_ws
```

### 3. Dependencies & Build
Install system and ROS dependencies (detailed in [05_software_setup.md](docs/05_software_setup.md)):
```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

### 4. Build Smoke Test
Verify the package is built correctly:
```bash
rospack find jetracer_ros
roslaunch jetracer_ros jetracer.launch
```

For hardware-free testing:
```bash
roscore
rosrun jetracer_ros teleop_node.py
```

## Documentation

For full setup guides, please see the `docs/` folder:
- [Getting Started](docs/01_getting_started.md)
- [Hardware Setup](docs/02_hardware_setup.md)
- [Software Setup](docs/05_software_setup.md)
- [Jetson OS Setup](docs/03_jetson_setup.md)
- [ROS Melodic Setup](docs/04_ros_melodic_setup.md)
- [Camera Setup](docs/07_camera_setup.md)
- [Calibration](docs/06_calibration.md)
- [Troubleshooting](docs/13_troubleshooting.md)
- [References](docs/14_references.md)

## Attributions

* **Waveshare**: [JetRacer ROS AI Kit Product Page](http://www.waveshare.com/JetRacer-ROS-AI-Kit.htm)
* **Cytron**: Based on the [Getting Started with ROS and JetRacer AI Kit](https://tutorial.cytron.io/?p=38120) tutorial.
