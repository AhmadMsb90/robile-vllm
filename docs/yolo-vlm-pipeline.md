## Vision-Language Target Localization Pipeline

After converting the front camera to RGB-D, the perception pipeline must be `extended` so that the mobile robot can `interpret a natural-language navigation instruction`, `identify` the corresponding `detected object`, `estimate` its `3D position` using depth, and express that position in the global `odom` frame.

### 1. ROS2 Interface Messages

The existing `robile_interfaces` package must be extended with messages for the perception pipeline:

#### `ObjectDetection.msg`
```bash
string class_name
float32 confidence
float32 x1
float32 y1
float32 x2
float32 y2
```
`ObjectDetectionArray.msg`
```bash
std_msgs/Header header
ObjectDetection[] detections
```
`VlmTarget.msg
`
```bash
std_msgs/Header header
int32 target_id
string target_class
float32 confidence
float32 x1
float32 y1
float32 x2
float32 y2
string reason
```

The `messages` must be `added` to `robile_interfaces/CMakeLists.txt` and `rebuilt` so they are available as ROS2 interfaces.

Verification:

```bash
ros2 interface show robile_interfaces/msg/VlmTarget
```

### 2. YOLO Object Detection Node

A ROS2 node must be implemented as:
`/ros2_ws/src/robile_vlm_perception/robile_vlm_perception/yolo_node.py`

The node:

- Subscribes to /camera_front/image_raw
- Runs YOLO object detection
- Processes camera frames 
- Publishes detections as `ObjectDetectionArray` interface message on the `/yolo/detections` topic

The `detections` contain the `object class`, `confidence`, and `bounding box coordinates`.

Runtime verification:
```bash
ros2 topic info /yolo/detections
```

### 3. Qwen2.5-VL Vision-Language Reasoning

A second ROS2 node must be implemented as:
`ros2_ws/src/robile_vlm_perception/robile_vlm_perception/vlm_node.py`

The node uses the vision-language model `Qwen/Qwen2.5-VL-3B-Instruct`

The model must be downloaded from` Hugging Face` and `stored` on the host through the `shared Docker` volume rather than inside the Docker container.

This approach must be used because keeping the model on the host/shared directory makes the approximately `7 GB model weights` persistent and allows the same model files to be reused across container sessions without downloading them again and also does not incrase the size of the docker image massively.


The model must be downloaded directly into the shared directory using the Hugging Face CLI, for example:

```bash
hf download Qwen/Qwen2.5-VL-3B-Instruct \
  --local-dir /host_shared/models/Qwen2.5-VL-3B-Instruct
```

The `ROS2 perception environment` runs inside the Docker container using `a dedicated Python virtual environment`:

```bash
/opt/ros_ml
```

The `virtual environment` must contain the machine-learning dependencies required by the perception pipeline, including:
- PyTorch
- Transformers
- Accelerate
- qwen-vl-utils
- BitsAndBytes.

**Important Note:** The `container` must have `access` to the `host NVIDIA GPU` through `Docker's NVIDIA runtime (--gpus all)`. The available GPU used for this project was the NVIDIA GeForce RTX 4060 Laptop GPU. `PyTorch` inside the `ros_ml environment` must be verified to `detect` `CUDA and the NVIDIA GPU`, allowing` Qwen2.5-VL inference` to `run on the GPU` rather than purely on the CPU.

Therefore the `model` will be loaded from the `host-mounted` path:
```python
MODEL_PATH = "/host_shared/models/Qwen2.5-VL-3B-Instruct"
```
**Important Note:** Because the `full-precision mode`l required `substantial GPU memory` and caused `CPU offloading` during `inference`, the final implementation uses `4-bit quantization` with `BitsAndBytes`:

```python
quantization_config = BitsAndBytesConfig(
    load_in_4bit=True,
    bnb_4bit_quant_type="nf4",
    bnb_4bit_compute_dtype=torch.float16,
    bnb_4bit_use_double_quant=True,
)
```
The model is then loaded with automatic device placement:

```bash
self.model = Qwen2_5_VLForConditionalGeneration.from_pretrained(
    MODEL_PATH,
    quantization_config=quantization_config,
    device_map="auto",
)
```

The` 4-bit configuration` reduces GPU memory usage `from over 6 GB to 2.7 GB`, enabling practical inference on an RTX 4060. Additionally, it` eliminates CPU offloading`, cutting response times from minutes to `10–20 seconds` per prompt.

The `VLM` is `not executed continuously` on every camera frame. RGB frames are buffered in memory, while YOLO provides continuous object detections. `Qwen2.5-VL` is `invoked` only `when a natural-language instruction is received` on the topic `/vlm/instruction`.


For each instruction, the node:
- Provides the VLM with the relevant `RGB image`  
- Provides the VLM with also with the `YOLO detections` 
- Asks VLLM to select the single detection that best matches the semantic instruction, for example:

```bash
ros2 topic pub --once /vlm/instruction std_msgs/msg/String \
"{data: 'Go to the person near the table.'}"
```

The `VLM returns` a `structured target-selection` response:

```bash
TARGET_ID: 0
REASON: The YOLO detection with the highest confidence is the person near the table.
```

The `selected object` is then `published` on the topic `/vlm/target` with an interface message of type `robile_interfaces/msg/VlmTarget` which has the structure below:

```bash
std_msgs/Header header
int32 target_id
string target_class
float32 confidence
float32 x1
float32 y1
float32 x2
float32 y2
string reason
```

Instead of tracking objects frame by frame, Qwen2.5-VL acts as a high-level semantic reasoning engine. While YOLO handles continuous object detection, the VLM steps in to interpret user prompts and pinpoint the exact object needed.

### 4. Depth-Based 3D Target Localization

A third ROS2 node must be implemented as:

`/home/ros2_ws/src/robile_vlm_perception/robile_vlm_perception/depth_target_node.py`

The purpose of this node is to `convert the semantic object` selected by the VLM from a 2D image detection `into a metric 3D target position` using the `RGB-D` camera. This provides the navigation system with the physical position of the selected object.


The node subscribes to:

- `/camera_front/depth/image_raw`
- `/camera_front/depth/camera_info`
- `/vlm/target`

As mentioned earlier, the /vlm/target message contains the bounding box selected by the VLM from the YOLO detections together with the target class, confidence, target ID, and the timestamp of the corresponding camera/detection frame.

#### 4.1 Depth Frame Buffering and Target Synchronization

Because the RGB image, YOLO detection, VLM reasoning, and depth image are processed by different ROS2 callbacks, the depth frame does not necessarily arrive in the same callback cycle as the selected VLM target. To handle this, the node maintains a small temporal buffer of recent depth frames:

```python
self.depth_buffer = {} 
self.max_buffer_size = 30
```
Each depth image is stored together with its ROS2 timestamp:

```python
timestamp = (
    msg.header.stamp.sec,
    msg.header.stamp.nanosec
)
```
When a VLM target arrives, the node first looks for a depth frame with exactly the same timestamp. If an exact match is unavailable, it selects the buffered depth frame with the closest timestamp.

This provides approximate temporal synchronization between the selected YOLO target and the corresponding RGB-D measurement while avoiding the need for a more complex synchronization mechanism.

#### 4.2 Extracting the Target Region from the Depth Image

The VLM target contains the bounding box of the selected object in image coordinates:

```bash
(x1, y1) ───────────────┐
   │                    │
   │      target        │
   │                    │
   └────────────────────┘
                    (x2, y2)
```
The coordinates are clipped to the actual depth-image dimensions to prevent invalid array indexing.



Instead of using the complete bounding box, an inner region is extracted. A 20% margin is removed from each side:

```python
margin_x = max(1, int((x2 - x1) * 0.20))
margin_y = max(1, int((y2 - y1) * 0.20))
```

This is done because pixels close to the bounding-box boundaries can contain a mixture of the target object and the surrounding scene. For example, a person detector may include part of the floor, table, or background around the person's body. Using an inner region reduces the influence of these boundary pixels.

The resulting region of interest is:

```python
roi = depth[
    y1 + margin_y:y2 - margin_y + 1,
    x1 + margin_x:x2 - margin_x + 1
]
```

#### 4.3 Filtering Invalid Depth Measurements

The raw depth image can contain invalid, infinite, or otherwise unusable measurements. The node therefore keeps only depth values satisfying:

```python 
np.isfinite(roi) & (roi > 0.1) & (roi < 100.0)
```

This removes:

- NaN values
- infinite values
- values below the configured minimum depth
- unrealistic values beyond the configured maximum range

If no valid depth values remain, the target is rejected and a warning is generated.

#### 4.4 Robust Depth Estimation

The `target depth` is estimated from `all valid pixel`s in the `inner bounding-box region` rather than using only one pixel. The `median` is used:

```python 
z = float(np.median(valid_depth))
```

The `median` is selected because it is more `robust` to isolated erroneous measurements than a single center pixel or a simple extreme-value measurement. This is particularly useful around object `boundaries`, where some depth pixels may belong to the background.

During testing, the `tightly concentrated depth distribution` within the object's bounding box confirmed that the `median` provided a `stable` target distance estimate.

#### 4.5 Determining the Target Pixel

The representative `image location` of the target is taken as the `center` of the selected `bounding box`:


```python 
u = (x1 + x2) / 2.0  # horizontal image coordinate
v = (y1 + y2) / 2.0  # vertical image coordinate
```

#### 4.6 Using the RGB-D Camera Calibration

The node obtains the `camera's intrinsic parameters` from the topic `/camera_front/depth/camera_info`.

The relevant parameters are extracted from the ROS2 `CameraInfo` message:

```python
fx = self.latest_camera_info.k[0]
fy = self.latest_camera_info.k[4]
cx = self.latest_camera_info.k[2]
cy = self.latest_camera_info.k[5]
```
For the configured 320 × 240 camera, the calibrated values obtained during runtime are approximately:
```python
fx = 190.681
fy = 190.681
cx = 160.5
cy = 120.5
```

These intrinsic parameters are used to convert image coordinates and measured depth into metric 3D coordinates.

#### 4.7 Back-Projecting the Pixel into 3D
The selected pixel `(u, v)` and the` measured depth z` are `converted` into a `3D point` using the `pinhole-camera projection` equations:

```python 
x = (u - cx) * z / fx 
y = (v - cy) * z / fy 
z = z
```
The resulting coordinates are expressed in the` camera coordinate frame`:
`camera_front_link`

Thus, the pipeline convertes:

```bash
┌────────────────────────┐
│    2D Bounding Box     │
├────────────────────────┤         ┌────────────────────┐
│   Depth Measurement    │  ─────► │ 3D Target Position │
├────────────────────────┤         └────────────────────┘
│   Camera Intrinsics    │
└────────────────────────┘
```
As an example, an approximate camera-frame position produced by one verified target is as below:

```bash
x = -0.377 m
y = -0.552 m
z =  2.569 m
```

Which indicates that the selected `object` is approximately `2.57 m` away along the camera `depth axis`.

#### 4.8 Creating the ROS2 3D Target Message

The 3D result is stored in a standard ROS2 message : `geometry_msgs/msg/PointStamped`

Initially, the point is set in the `camera_front_link` frame while retaining the timestamp from the original VLM/YOLO detection:

```python 

camera_point.header.stamp = msg.header.stamp
camera_point.header.frame_id = 'camera_front_link'
```

The original timestamp is preserved so that the target position corresponds to a specific camera observation rather than an arbitrary later time.

#### 4.9 TF2 Transformation toward the Navigation Frame

The camera-frame point is subsequently transformed. The `depth node` maintains a TF2 buffer and listener:

```python 
self.tf_buffer = tf2_ros.Buffer() 
self.tf_listener = tf2_ros.TransformListener( 
    self.tf_buffer, 
    self 
)
```

The `target` is `transformed` directly from `camera_front_link` to `odom` using the `timestamp` associated with the target:

```python 
transform = self.tf_buffer.lookup_transform( 
    'odom', 
    'camera_front_link', 
    camera_point.header.stamp, 
    timeout=rclpy.duration.Duration(seconds=0.5) 
)
```
The transformation is then applied with:


```python 
odom_point = do_transform_point(
    camera_point,
    transform
)
```
The resulting message is populated in the `odom` frame while retaining the synchronized timestamp so that the final target position is expressed in a stable navigation frame:


```python 
odom_point.header.stamp = camera_point.header.stamp odom_point.header.frame_id = 'odom'
```
#### 4.10 Verified Transformation Chain

The robot TF tree is verified to contain the following chain:
```bash
odom
 └── base_footprint
      └── base_link
           └── camera_front_link
```

The` fixed camera mounting transform` was verified using the TF2 transform between `base_link` and `camera_front_link`:

```bash
ros2 run tf2_ros tf2_echo base_link camera_front_link

At time 0.0 
- Translation: [0.450, 0.000, 0.300] 
- Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000] 
- Rotation: in RPY (radian) [0.000, 0.000, 0.000] 
- Rotation: in RPY (degree) [0.000, 0.000, 0.000] 
- Matrix: 
1.000 0.000 0.000 0.450 
0.000 1.000 0.000 0.000 
0.000 0.000 1.000 0.300 
0.000 0.000 0.000 1.000

```
The output indicates there is `only translation` between `base_link` and `camera_front_link`.

```bash
Translation: 
x = 0.45 m 
y = 0.00 m 
z = 0.30 m
```
The complete target transformation is therefore handled by TF2:

```bash
┌───────────────────────┐
│   camera_front_link   │
└───────────┬───────────┘
            │
           TF2
            │
            ▼
┌───────────────────────┐
│       base_link       │
└───────────┬───────────┘
            │
           TF2
            │
            ▼
┌───────────────────────┐
│         odom          │
└───────────────────────┘
```
A complete runtime test produced the following target coordinates:

Camera frame (`camera_front_link`):

```bash
x = -0.377 m
y = -0.552 m
z =  2.569 m
```

The `camera_front_link` frame is `attached` to the` RGB-D camera`. Its `origin` is located at the `camera's calibrated mounting position` on the robot. The coordinates are expressed relative to this camera frame:

- `z` is the `camera;s optical/depth direction`, pointing `outward` from the camera into the scene.
- `x` is the camera's `horizontal` direction.
- `y` is the camera's `vertical` direction according to the ROS camera-frame convention.

The point is first calculated from the image pixel and depth value using the camera intrinsics:

```python 
x = (u - cx) * z / fx
y = (v - cy) * z / fy
z = measured depth
```

The `camera-frame point` is then transformed into the `odom` frame using the `TF2` transform chain:

```bash
camera_front_link
└── base_link
    └── base_footprint
        └── odom
```

**Note:** The `odom` frame is a `robot/world reference frame` maintained by the `robot's odometry system`. Its `origin` is established `when the odometry system starts`, and its `axes` remain `fixed` relative to that `local world frame`. The odom frame is not necessarily aligned with a global map or GPS frame, and its origin may drift over long distances because it is based on accumulated robot motion.

As mentioned earlier, the verified `camera mounting` transform relative to `base_link` is:

`camera_front_link` relative to `base_link`:

```bash
translation:
x = 0.45 m
y = 0.00 m
z = 0.30 m

rotation:
identity
```

Therefore, in this configuration, the camera is mounted 0.45 m forward and 0.30 m above the robot's `base_link`, with no additional rotation relative to that frame. The remaining transformation from `base_link` through `base_footprint` to `odom` is provided by the robot's TF2/odometry system. Any robot orientation accumulated by odometry is included in this TF2 transformation.


After applying the complete translation and rotation from camera_front_link to odom, the same example target (see above) was represented as:

```bash
Odom frame:
x = 0.073 m
y = -0.552 m
z =  2.869 m
```

The transformed result was published on the topic : `/vlm/target_point` with

```bash
header:
  frame_id: odom
```
#### 4.11 Final Result of the Depth Localization Stage

The node therefore performs the complete conversion:

```bash
                    VLM-selected YOLO bounding box
                                ↓               
                    matching RGB-D depth frame  
                                ↓               
                    inner bounding-box depth region
                                ↓               
                        valid-depth filtering     
                                ↓               
                        median target depth      
                                ↓               
                        camera intrinsics       
                                ↓               
                    3D point in camera_front_link 
                                ↓               
                        TF2 transformation      
                                ↓               
                            3D point in odom       
                                ↓               
                        /vlm/target_point       
```

**Summary:**

The /vlm/`target_point` topic outputs the 3D metric position of a selected object `in the odom` coordinate frame. For robot `navigation`, the `2D (x, y)` coordinates are used to generate movement `goals`, while the height (z) coordinate is kept strictly for 3D perception data.

### 5. Final Verified Pipeline

The complete perception pipeline is:

```bash 
RGB-D Camera
      │
      ├── RGB image ───────────────┐
      │                            ▼
      │                      YOLO detector
      │                            │
      │                            ▼
      │                     /yolo/detections
      │                            │
      └────────────────────────────┼► Qwen2.5-VL
                                   │
                                   ▼
                            /vlm/instruction
                                   │
                                   ▼
                            target selection
                                   │
                                   ▼
                              /vlm/target
                                   │
                                   ▼
                              Depth image
                                   │
                                   ▼
                         3D target estimation
                                   │
                                   ▼
                         camera_front_link
                                   │
                                   ▼
                                  TF2
                                   │
                                   ▼
                                 odom
                                   │
                                   ▼
                          /vlm/target_point
```


