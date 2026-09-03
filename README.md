# Perception and Sensor Configuration

Welcome to the project repository. Below are the links to the individual documentation files covering the perception environment and camera configuration:


* [Main Camera Configuration Change — RGB to RGB-D (Depth) Camera](enable-vision-depth.md) — Details how the Gazebo front camera description was modified from a standard RGB camera to an RGB-D depth camera, along with verification and runtime topics.


* [ROS 2 + Python ML Environment for YOLO Perception](enable-ros-yolo.md) — Covers setting up the dedicated Python virtual environment, managing NumPy/OpenCV and PyTorch dependencies, package structures, and running the YOLO perception node.

* [VLM and YOLO 3D Target Estimation Pipeline](yolo-vlm-pipeline.md) — Explains the integration of YOLO object detections and Qwen2.5-VL for target selection, including depth calculation and TF2 frame transformations to the odom frame.