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