## Main Camera Configuration Change — RGB to RGB-D (Depth) Camera

The aim is to make the `Robile front camera` publish both **RGB image data** and **depth sensor data** in ROS2.




### 1. File Modified
`/home/ros2_ws/src/robile_description/urdf/sensors/camera_front.gazebo.xacro`

### 2. Change Summary
The Gazebo camera sensor definition was switched from a plain RGB camera to a depth-capable (RGB-D) camera by changing the sensor `type` attribute, adding a `<depth_camera>` tag inside the `<camera>` block, and extending the plugin with depth-specific parameters (`min_depth`, `max_depth`) alongside the existing `camera_name` and `frame_name` tags so the correct topics and frames are generated.

**Before:**
```xml
<sensor name="${name}" type="camera">
```

**After:**
```xml
<sensor name="${name}" type="depth">
```

Depth-camera element added inside `<camera>`:
```xml
<depth_camera></depth_camera>
```

Plugin updated with depth parameters and naming:
```xml
<camera_name>${name}</camera_name>
<frame_name>${name}_link</frame_name>
<min_depth>${min_range}</min_depth>
<max_depth>${max_range}</max_depth>
```

### 3. Resulting Full Sensor Block
This is the complete, merged sensor definition after all edits — the `type="depth"` sensor, the depth camera flag, and the plugin with both ROS remapping and depth clipping parameters:

```xml
<sensor name="${name}" type="depth">
  <update_rate>${update_rate}</update_rate>
  <pose>0 0 0 0 0 0</pose>
  <visualize>true</visualize>
  <camera name="head">
    <horizontal_fov>1.3962634</horizontal_fov>
    <image>
      <width>800</width>
      <height>800</height>
      <format>R8G8B8</format>
    </image>
    <depth_camera></depth_camera>
    <clip>
      <near>${min_range}</near>
      <far>${max_range}</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.001</stddev>
    </noise>
  </camera>
  <plugin name="${name}_controller" filename="libgazebo_ros_camera.so">
    <ros>
      <remapping>~/out:=${ros_topic}</remapping>
    </ros>
    <camera_name>${name}</camera_name>
    <frame_name>${name}_link</frame_name>
    <min_depth>${min_range}</min_depth>
    <max_depth>${max_range}</max_depth>
  </plugin>
</sensor>
```

### 4. Camera Xacro Inclusion
The gazebo config file above is pulled in via this include, and the camera macro is then instantiated with concrete parameters (topic name, update rate, min/max range, and mounting pose):

File: `/home/ros2_ws/src/robile_description/urdf/sensors/camera_front.urdf.xacro`
```xml
<xacro:include filename="$(find robile_description)/urdf/sensors/camera_front.gazebo.xacro" />
```

Instantiated in `/home/ros2_ws/src/robile_description/gazebo/gazebo_robile_laserscanner_camera.xacro`:
```xml
<xacro:camera_front
    name="camera_front"
    parent="base"
    ros_topic="/camera/image_raw"
    update_rate="30"
    min_range="0.1"
    max_range="100">
    <origin xyz="0.45 0 0.30" rpy="0 0 0"/>
</xacro:camera_front>
```

Mounting pose relative to `base_link`: `x = 0.45 m`, `y = 0.00 m`, `z = 0.30 m`.

### 5. Debugging / Verification Steps
These steps confirm the installed package points back to the edited source file, then rebuild the package and regenerate the full URDF so the change can be checked in the actual output.

**Package prefix check:**
```bash
source /opt/ros/humble/setup.bash
source /home/ros2_ws/install/setup.bash
ros2 pkg prefix robile_description
# -> /home/ros2_ws/install/robile_description
```

**Symlink resolution check (confirms the installed xacro is the edited source):**
```bash
readlink -f /home/ros2_ws/install/robile_description/share/robile_description/urdf/sensors/camera_front.gazebo.xacro
# -> /home/ros2_ws/src/robile_description/urdf/sensors/camera_front.gazebo.xacro
```

**Rebuild package:**
```bash
cd /home/ros2_ws
colcon build --packages-select robile_description --symlink-install
source /home/ros2_ws/install/setup.bash
```

**Generate full URDF from the top-level xacro file:**
```bash
xacro /home/ros2_ws/src/robile_description/gazebo/gazebo_robile_laserscanner_camera.xacro \
  platform_config:=4_wheel_config \
  movable_joints:=False > /tmp/generated_robot.urdf
```

**Confirm the depth sensor and depth_camera tag exist in the generated URDF, and that no stray LaserScan output_type leaked into the camera plugin:**
```bash
grep -n 'sensor name="camera_front"' /tmp/generated_robot.urdf
# -> 670:    <sensor name="camera_front" type="depth">

grep -n '<depth_camera>' /tmp/generated_robot.urdf
# -> 681:        <depth_camera>

grep -n 'output_type' /tmp/generated_robot.urdf
# -> 650:        <output_type>sensor_msgs/LaserScan</output_type>  (belongs to the LiDAR, not the camera)
```

### 6. Runtime Verification
The robot was launched in Gazebo and the actual published ROS 2 topics were inspected to confirm the depth pipeline is live, not just present in the URDF.

**Launch:**
```bash
ros2 launch robile_gazebo gazebo_4_wheel.launch.py
```

**List camera/depth/point-related topics:**
```bash
ros2 topic list | grep -Ei 'camera|depth|point'
```

**Result:**
```
/camera_front/camera_info
/camera_front/depth/camera_info
/camera_front/depth/image_raw
/camera_front/depth/image_raw/compressed
/camera_front/image_raw
/camera_front/image_raw/compressed
/camera_front/points
/clicked_point
```

**Inspect the RGB topic's publisher/type:**
```bash
ros2 topic info /camera_front/image_raw -v
```
```
Type: sensor_msgs/msg/Image
Publisher count: 1
Node name: camera_front_controller
```

**Echo one RGB frame to confirm encoding/resolution:**
```bash
ros2 topic echo /camera_front/image_raw --once
```
```
height: 800
width: 800
encoding: rgb8
```

### 7. Final Status

| Output | Topic |
|---|---|
| RGB image | `/camera_front/image_raw` |
| RGB camera info | `/camera_front/camera_info` | 
| Depth image | `/camera_front/depth/image_raw` |
| Depth camera info | `/camera_front/depth/camera_info` | 
| Point cloud | `/camera_front/points` | 

### 8. Essential Change

```xml
<sensor ... type="camera">   →   <sensor ... type="depth">
```
plus adding `<depth_camera></depth_camera>` and the depth plugin parameters (`min_depth`, `max_depth`). This single sensor-type switch is what causes Gazebo to publish the full RGB-D topic set (RGB image, depth image, camera info for both, and a derived point cloud) instead of just a plain RGB image.