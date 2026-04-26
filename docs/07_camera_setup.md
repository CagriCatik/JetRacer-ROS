# Camera Setup

This document describes the setup for the CSI camera included with the Waveshare JetRacer kit.

## Hardware Verification
Ensure the CSI ribbon cable is securely connected to the Jetson Nano (blue side facing the ethernet port). 

To test the camera outside of ROS:
```bash
# Check if the camera device is registered
ls /dev/video*

# Check V4L2 device details
v4l2-ctl --list-devices
```

## Fake ROS Camera Node (Operational Warning)
> [!WARNING]  
> The repository currently includes `scripts/camera.py`. **This is NOT a ROS node.**
> It is a standalone Flask web server that streams OpenCV frames. It does NOT publish to `/camera/image_raw` or use `rospy`.

### How to use `camera.py`
If you run `python3 scripts/camera.py`, you can view the camera stream by opening a web browser on another computer and navigating to:
`http://<jetson-ip-address>:5000`

### Integrating with Autonomous Driving
If you wish to use the camera for autonomous driving (e.g. Lane Following or Object Detection), you **must** replace `camera.py` with a standard ROS node such as `jetson_camera` or `gscam` that publishes `sensor_msgs/Image` to the `/camera/image_raw` topic.
