## ROS 2 + Python ML Environment for YOLO Perception

The perception package requires a Python environment that provides both the ROS 2 Python interfaces and the machine-learning dependencies used by YOLO. The environment must also remain compatible with ROS 2 Humble's `cv_bridge`.

### 1. Python Environment

A dedicated Python virtual environment is used for the machine-learning components:
```bash
python -m venv ros_ml
source /opt/ros_ml/bin/activate
```

Verify that Python resolves to the environment:
```bash
which python
python -c "import sys; print(sys.executable); print(sys.prefix)"
```

Expected:

```bash
/opt/ros_ml/bin/python
/opt/ros_ml
```

The ROS 2 Humble environment is also available through the system installation:

```bash
source /opt/ros/humble/setup.bash
```

The resulting `PATH` places the ML environment before the system Python:

```bash
echo $PATH
```

Expected to contain:

```bash
/opt/ros_ml/bin
/opt/ros/humble/bin
/usr/bin
```
### 2. Required Python ML Dependencies

The following dependencies must be installed on the environment :

* PyTorch
* CUDA-enabled PyTorch
* Ultralytics
* OpenCV
* NumPy
* `cv_bridge`

Verify the installed packages:
```bash
python -m pip check
```

Expected:
```bash
No broken requirements found.
```

Verify Installation:

* **Ultralytics version:** 
  ```bash
  python -c "import ultralytics; print('Ultralytics:', ultralytics.__version__)"
  ```
* **PyTorch & CUDA setup:** 
  ```bash
  python -c "import torch; print('Torch:', torch.__version__); print('CUDA:', torch.version.cuda); print('CUDA available:', torch.cuda.is_available()); print('GPU:', torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'None')"
  ```
---

### 3. NumPy / OpenCV Compatibility

ROS 2 Humble's installed `cv_bridge` Python module is compiled against the NumPy 1.x API. Therefore, the ML environment must use a NumPy 1.x release rather than NumPy 2.x.


```bash
NumPy 1.26.4
OpenCV 4.10.0.84
```

Install the compatible pair:

```bash
python -m pip install --force-reinstall "numpy==1.26.4" "opencv-python==4.10.0.84"
```

Verify the dependency state:

```bash
python -m pip check
```

Expected:

```bash
No broken requirements found.
```

Verify the complete Python compatibility chain:

```bash
python -c "import numpy, cv2, torch; from cv_bridge import CvBridge; print('NumPy:', numpy.__version__); print('OpenCV:', cv2.__version__); print('Torch:', torch.__version__); print('CUDA:', torch.cuda.is_available()); print('cv_bridge: OK')"
```

Expected:

```bash
NumPy: 1.26.4
OpenCV: 4.10.0
Torch: 2.13.0+cu130
CUDA: True
cv_bridge: OK
```

### 4. ROS 2 Python / ML Import Compatibility

Before running the perception node, verify that ROS 2 Python modules and ML modules can be imported from the same ros_ml environment:

```bash
python -c "import rclpy; from cv_bridge import CvBridge; from ultralytics import YOLO; import torch; print('ROS 2: OK'); print('cv_bridge: OK'); print('Ultralytics: OK'); print('Torch:', torch.__version__); print('CUDA:', torch.cuda.is_available())"
```

Expected:

```bash
ROS 2: OK
cv_bridge: OK
Ultralytics: OK
Torch: 2.13.0+cu130
CUDA: True
```
### 5. Ultralytics Installation Verification

Run the Ultralytics environment check:

```bash
yolo checks
```

Expected:

```bash
Python-3.10.12
torch-2.13.0+cu130
CUDA:0 (NVIDIA GeForce RTX 4060 Laptop GPU, 8188MiB)
Environment: Docker
GPU count: 1
CUDA: 13.0
```

### 6. ROS 2 VLLM Perception Package
The package structure for the perception components is:

```bash
src/robile_vlm_perception/
├── config/
├── launch/
├── package.xml
├── resource/
│   └── robile_vlm_perception
├── robile_vlm_perception/
│   ├── __init__.py
│   ├── vlm_node.py
│   └── yolo_node.py
├── setup.cfg
├── setup.py
└── test/
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```
Executables:

```bash
robile_vlm_perception
├── yolo_node
└── vlm_node
```

### 7. Python Interpreter Compatibility for ROS2 Executables

ROS2 Humble's ros2 command is provided by the system ROS installation:

```bash
which ros2
```
Expected:
```bash
/opt/ros/humble/bin/ros2
```
However, the ML dependencies are installed in:
```bash
/opt/ros_ml/bin/python
```
Therefore, the ROS2 executable launcher must explicitly invoke the `ros_ml` Python interpreter so that `ultralytics`, `PyTorch`, and the other `ML dependencies` are available when `ros2 run` starts the node.

The launcher is located at:

```bash
src/robile_vlm_perception/scripts/yolo_node
```

Its contents are:
```bash
#!/bin/bash
exec /opt/ros_ml/bin/python -m robile_vlm_perception.yolo_node "$@"
```

Make it executable:

```bash
chmod +x src/robile_vlm_perception/scripts/yolo_node
```

The package is configured so this script is installed as the `yolo_node` ROS2 executable.

### 8. Build and source the Perception Package 

```bash

cd /home/ros2_ws

colcon build --packages-select robile_vlm_perception --symlink-install

source /home/ros2_ws/install/setup.bash
```

Verify that the installed YOLO launcher uses the ML environment:

```bash
head -5 /home/ros2_ws/install/robile_vlm_perception/lib/robile_vlm_perception/yolo_node
```

Expected:

```bash
#!/bin/bash
exec /opt/ros_ml/bin/python -m robile_vlm_perception.yolo_node "$@"
```

### 9. YOLO Node

- The `YOLO node` subscribes to the `front RGB camera` topic: `/camera_front/image_raw`

- The incoming ROS2 `sensor_msgs/msg/Image` message is converted to an `OpenCV image` using `cv_bridge`

- The image is then passed to `YOLO11n`



The detection pipeline is therefore:

```bash
/camera_front/image_raw
        │
        ▼
    ROS 2 Image
        │
        ▼
     cv_bridge
        │
        ▼
   OpenCV BGR image
        │
        ▼
     YOLO11n
        │
        ▼
   CUDA / RTX 4060
        │
        ▼
 object detections
 ```

### 10. Runtime Verification

Start the YOLO ROS 2 node with:

```bash
ros2 run robile_vlm_perception yolo_node
```

The node should initialize with messages similar to:
```bash
[yolo_node]: YOLO node started
[yolo_node]: Model: YOLO11n
[yolo_node]: Device: CUDA
```

Once camera frames are received and processed, detections should be reported:

```bash
[yolo_node]: Frame 5: 1 objects detected
[yolo_node]:   airplane: 0.53 bbox=(0,64,54,86)
```

`The exact detected classes and bounding boxes depend on the current simulated camera scene.`

A successful runtime confirms the complete processing chain:

```bash
Robile simulated RGB camera
          │
          ▼
/camera_front/image_raw
          │
          ▼
      cv_bridge
          │
          ▼
      OpenCV image
          │
          ▼
       YOLO11n
          │
          ▼
   NVIDIA RTX 4060
          │
          ▼
    object class
    confidence
    bounding box
```

