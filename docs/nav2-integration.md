
## Navigation Frame Alignment and AMCL Initialization

After completing the RGB-D, YOLO, Qwen2.5-VL, depth localization, and TF2 stages, the next step is to connect the resulting target positions to the robot's navigation system. Before using the VLM target as a `Nav2` goal, the navigation stack itself has to be verified independently. The relationship between the simulator's `world` frame, the robot's `odom` frame, and the loaded occupancy-grid `map` frame should be verified. The robot navigation TF tree is expected to follow:

```bash
map 
└── odom 
    └── base_footprint 
        └── base_link 
            └── camera_front_link
```

The goal is therefore to verify that `AMCL` correctly initialized the robot in the `map` frame and that the below transform chain represents the actual robot position:

```bash
map 
└── odom 
    └── base_footprint 
        └── base_link 
```

### 1. Inspecting the Ground-Truth Pose

The `Gazebo` provides the robot's `ground-truth pose` through the topic `/ground_truth_pose` with respect to the `Gazebo's world frame`:

An example inspection:
```bash
ros2 topic echo /ground_truth_pose --once

header: frame_id: world 

child_frame_id: base_link 

pose: 
    position: 
    x: -1.9991205545649406 
    y: -8.484731565008724 
    z: 0.18978606370405843 

    orientation: 
    x: -1.259820634925919e-09 
    y: 5.71252041569652e-10 
    z: 0.004063475643747908 
    w: 0.9999917440487661
```

### 2 Inspecting the Odometry Pose

The odometry pose iis checked with:

```bash
ros2 topic echo /odom --once

header:
  frame_id: odom

child_frame_id: base_footprint

pose:
  position:
    x: -1.999120568174062
    y: -3.484731615614152
    z: 0.18978606370405843

  orientation:
    x: -1.2598206014676643e-09
    y: 5.712520307720002e-10
    z: 0.00406344161103693
    w: 0.9999917441870577
```

The important observation was that the `x coordinate` and `orientation` were essentially `identical` to the ground-truth pose, while the `y coordinate` differed by exactly approximately `5 m`:

```bash
ground_truth y ≈ -8.4847 m
odom y         ≈ -3.4847 m

difference ≈ 5.0 m
```

Which is consistent with the `Gazebo ground-truth` configuration used by the Robile robot model, where the ground-truth pose plugin contains:

```xml
<xyz_offset>0 -5.0 0</xyz_offset>
```

Therefore, the `5 m` difference was `not` caused by navigation `drift`. It comes from the `coordinate offset` configured for the simulator's ground-truth pose.

### 3 Inspecting the Navigation Map

The loaded `occupancy-grid map` can be inspected:

`/home/ros2_ws/src/robile_navigation/maps/map.yaml`

The `map configuration` is:

```xml
image: map.pgm
mode: trinary
resolution: 0.05
origin: [-5.09, -5.37, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```
The relevant map parameters are therefore:

```text
resolution = 0.05 m/pixel

origin:
x = -5.09 m
y = -5.37 m
yaw = 0
```

**Note:** The robot's position in the map frame is not identical to its position in the Gazebo world frame.


### 4 Determining the Robot's Initial Map Position

Using the known simulator offset and map origin, the robot's approximate map position has to be derived from the observed ground-truth position.

The ground-truth pose before navigation is:

```text
world:
x = -1.9991
y = -8.4847
```

The `simulator's ground-truth y` coordinate includes the `-5 m offset`, so the corresponding robot position relative to the local map coordinate system is approximately:

```text
x_map ≈ -1.9991 - (-5.09)
      ≈ 3.091 m

y_map ≈ -8.4847 - (-5.37)
      ≈ -3.115 m
```

The robot `orientation` before navigation was almost `aligned` with the `negative x direction` after the map-frame initialization convention was taken into account. The corresponding yaw used for AMCL initialization was approximately:

```text

yaw ≈ -3.1338 rad
```

The resulting estimated initial pose used for AMCL was therefore:

```text
x = 3.0912 m
y = -3.1356 m
yaw = -3.1338 rad
```

This was then used to initialize AMCL explicitly.

To prevent AMCL from starting with an unrelated default pose, the AMCL launch configuration has to change to explicitly define the robot's initial map pose:

```python
Node(
    package='nav2_amcl',
    executable='amcl',
    name='amcl',
    output='screen',
    parameters=[
        nav2_yaml,
        {
            'use_sim_time': True,
            'set_initial_pose': True,
            'initial_pose.x': 3.0912,
            'initial_pose.y': -3.1356,
            'initial_pose.z': 0.0,
            'initial_pose.yaw': -3.1338
        }
    ]
)
```

The important parameters are:

```python
set_initial_pose = true

initial_pose.x = 3.0912
initial_pose.y = -3.1356
initial_pose.z = 0.0
initial_pose.yaw = -3.1338
```
This makes `AMCL` start from the robot's `known initial location` in the `occupancy-grid map` instead of requiring the initial estimate to remain at an unrelated default position.

### 5. Verifying the AMCL Localization

The AMCL estimate can be checked :

```bash
ros2 topic echo /amcl_pose --once

header:
  frame_id: map

pose:
  pose:
    position:
      x: 3.0739582306005464
      y: -3.124103280650459
      z: 0.0

    orientation:
      x: 0.0
      y: 0.0
      z: -0.9999536072263803
      w: 0.009632413765507811
```

The corresponding `yaw` is approximately:
```bash
yaw ≈ -3.122 rad ≈ -178.9°
```


The estimated AMCL position will therefore be close to the configured initial pose:

```bash
configured:
x = 3.0912
y = -3.1356

AMCL:
x ≈ 3.0740
y ≈ -3.1241
```

The` small difference` is normal and the `AMCL` performs its `localization update` using the laser scan and the map `after initialization`.

The AMCL covariance will also no longer be zero. The relevant part was approximately:

```bash

x variance ≈ 0.2413
y variance ≈ 0.2402
yaw variance ≈ 0.0661
```

Wich indicates that AMCL has established an actual probabilistic pose estimate instead of simply exposing the previously observed (0,0) initialization.

### 6. Verifying the Map-to-Robot TF

The TF chain check :

```bash
ros2 run tf2_ros tf2_echo map base_link

- Translation: [3.079, -3.206, -0.000]
- Rotation: in Quaternion [-0.000, 0.000, 1.000, -0.000]
- Rotation: in RPY (radian) [0.000, 0.000, -3.141]
- Rotation: in RPY (degree) [0.000, 0.000, -179.962]
- Matrix:
 -1.000  0.001 -0.000  3.079
 -0.001 -1.000  0.000 -3.206
 -0.000  0.000  1.000 -0.000
  0.000  0.000  0.000  1.000
```
The important result iw that the direct TF query `map → base_link` agrees closely with the AMCL pose and `AMCL estimate` is successfully being propagated into the` TF tree`:

```text
AMCL:
x ≈ 3.074
y ≈ -3.124
yaw ≈ -3.122 rad

TF:
x ≈ 3.074
y ≈ -3.124
yaw ≈ -3.122 rad
```


### 7. Basic Nav2 Navigation Test

After correcting the initial AMCL pose, Nav2 can be tested:

Example: The current `robot pose` was approximately:

```text
map:
x ≈ 3.074 m
y ≈ -3.124 m
yaw ≈ -3.122 rad
```

A simple target roughly 0.5 m in front of the robot was therefore selected:

```text
goal:
x = 2.574 m
y = -3.124 m
```

The` Nav2 action` was sent with a manually specified `goal` (later to be published on the `vlm/target_poin`):

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: map}, pose: {position: {x: 2.574, y: -3.124, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.99995, w: 0.00963}}}}" \
--feedback
```

 Odometry Before and During the Nav2 Test

Before the movement (Nav2 Test), `/odom` reported approximately:

```text
x = -1.9991
y = -3.4847
yaw ≈ 0.0081 rad
```
After the Nav2 test, `/odom` reported:

```text
header:
  frame_id: odom

child_frame_id: base_footprint

pose:
  position:
    x: -1.7119219514483672
    y: -3.486532236266445
    z: 0.18978606370370757

  orientation:
    x: -1.279830887428905e-09
    y: 5.312931606991144e-10
    z: 0.035467550820779375
    w: 0.9993708284909938

twist:
  twist:
    linear:
      x: -1.05264180106723e-06
      y: -0.00022442104411086298
      z: 0.0

    angular:
      x: 0.0
      y: 0.0
      z: -0.052631578947368474
```
Compared with the initial odometry:

```
initial:
x ≈ -1.9991
y ≈ -3.4847

later:
x ≈ -1.7119
y ≈ -3.4865
```
the robot had changed its odometry position by approximately:

```text
Δx ≈ +0.287 m
Δy ≈ -0.002 m
```

Therefore the robot responds to the Nav2 command with actual translational motion.

The odometry orientation also changed from approximately:

```text
yaw ≈ 0.008 rad
```
to approximately:
```text
yaw ≈ 0.071 rad
```
indicating that the controller was also able to command rotational motion.



The avigation stack:

```bash
AMCL localization
       ↓
map → odom → base_link TF
       ↓
Nav2 planner
       ↓
Nav2 controller
       ↓
/cmd_vel
       ↓
Robile robot
```


