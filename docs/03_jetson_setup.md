# Jetson Nano & OS Setup

This project requires a specific OS baseline to function seamlessly with ROS Melodic.

## Target Hardware
* **Board**: NVIDIA Jetson Nano Developer Kit B01 (4GB)
* **OS Image**: JetPack 4.4 / 4.4.x (L4T 32.x)

## Verifying the OS Environment
Before installing any software, verify that your Jetson Nano is running the expected Ubuntu 18.04 Bionic distribution.

```bash
# Check Ubuntu Version
lsb_release -a
# Expected: Ubuntu 18.04.x LTS (bionic)

# Check Architecture
uname -m
# Expected: aarch64

# Check JetPack / CUDA installation
nvcc --version
```

## OS Dependencies
Ensure your base system is up to date and has the necessary build tools:
```bash
sudo apt update
sudo apt upgrade -y
sudo apt install build-essential git python-pip
```

> [!WARNING]
> Do NOT upgrade to Ubuntu 20.04 or install JetPack 5+ if you intend to use this repository as-is. ROS Melodic is strictly coupled to Ubuntu 18.04.
