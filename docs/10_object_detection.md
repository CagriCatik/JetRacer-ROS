# Object Detection (Proposed)

> [!NOTE]
> This functionality is **not currently implemented**. It is documented here as part of the proposed autonomous driving architecture.

## YOLO Compatibility Rules
The base ROS package must build and run **without YOLO installed**.
Do not install `ultralytics`, `torch`, or Python 3.8 dependencies. ROS Melodic primarily targets Python 2, and the base package must remain compatible.

## Proposed Detectors
If object detection is added in the future, it must run as an optional, isolated process:
- **Recommended Compatible Option**: YOLOv4-tiny using Darknet.
- **Conservative Fallback**: YOLOv3-tiny using Darknet.
- **Future Optimization**: TensorRT-optimized YOLOv4-tiny.
- **Not Supported Default**: YOLO11 (requires modern Python/PyTorch stack incompatible with base Melodic).

## Proposed Architecture
- **Node**: `object_detection_node.py` (Optional / External)
- **Subscribes**: `/camera/image_raw`
- **Publishes**: 
  - `/perception/detections` (Custom or vision_msgs)
  - `/perception/debug_image`
  - `/perception/status`
- **Important**: The object detector **MUST NOT** publish motor commands directly to `/cmd_vel` or `/throttle`. It must only output semantic bounding boxes to be consumed by `semantic_behavior_node`.
