# smorphi_semantic_mapping

Adds **semantic object labels to the SLAM map** while the Smorphi robot is mapping.
A YOLOv8 model classifies objects in the camera feed; each detection's bearing is
fused with the 2D LiDAR `/scan` to get a real `(x, y)` point, which is transformed
into the `map` frame and shown in RViz (and saved to YAML).

It deliberately reuses the project's existing label pattern (sphere + `TEXT_VIEW_FACING`
markers in the `map` frame, YAML persistence) from `waypoint_label_publisher.py`.

## How it works

```
camera image -> YOLOv8 -> bbox -> bearing -> /scan range at that bearing
   -> point in laser frame -> TF -> point in map frame
   -> merge into per-class landmark store -> /semantic_objects markers + YAML
```

A monocular camera only gives **bearing**, not distance, so the LiDAR supplies the
range. This is the recommended method for this robot (2D RPLidar + mono camera).

## Your model

These weights (`best.pt`, YOLOv8n, 80 epochs, 640px) detect **2 classes**:

| id | class name |
|----|------------|
| 0  | `Machine_01` |
| 1  | `Machine_02` |

Final validation metrics were precision 0.94 / recall 0.94 / mAP50 0.96, so a
`conf_threshold` of 0.5–0.6 works well. The node reads these names straight from the
`.pt` file, so you don't configure them anywhere — detected machines appear on the map
as `Machine_01` / `Machine_02` labels automatically.

Use `weights/best.pt` (not `last.pt`) — `best.pt` is the checkpoint with the best
validation score.

## 1. Install dependencies (on the robot, ROS 2 Humble)

```bash
pip3 install ultralytics
sudo apt install ros-humble-cv-bridge ros-humble-vision-msgs \
                 ros-humble-tf2-geometry-msgs ros-humble-v4l2-camera
```

(`v4l2-camera` is only needed if you don't already publish a camera image topic.)

## 2. Drop the package into the workspace and build

Copy this folder next to the other packages (e.g. `~/ros2_ws/src/smorphi_semantic_mapping`)
then:

```bash
cd ~/ros2_ws
colcon build --packages-select smorphi_semantic_mapping
source install/setup.bash
```

## 3. Run

Start the robot bringup + SLAM as usual, then in another terminal:

```bash
ros2 launch smorphi_semantic_mapping semantic_mapping.launch.py \
    model_path:=/home/smorphi/best.pt \
    image_topic:=/image_raw \
    scan_topic:=/scan \
    start_camera:=true
```

If your YOLO node / camera already publishes images, set `start_camera:=false` and
point `image_topic` at your existing topic.

In RViz add a **MarkerArray** display on `/semantic_objects`. Coloured spheres with
text labels appear on the map as objects are detected.

## 4. Calibrate two things for accurate placement

These two parameters decide whether a detected "chair" lands on the real chair:

- **`horizontal_fov_deg`** — your camera's horizontal field of view in degrees
  (check the datasheet; e.g. ~62° for a Pi Camera v2, ~70° for many USB webcams).
- **`camera_yaw_offset`** — yaw in **radians** from the LiDAR's 0° direction to the
  camera's optical axis. `0.0` if the camera faces the same way as LiDAR angle 0.
  If the camera points to the robot's left by 90°, set `1.5708`.

Sanity test: face a wall, detect something, and confirm the marker lands on the wall
in RViz. If it's rotated off to one side, adjust `camera_yaw_offset`. If the distance
is consistently wrong, fix `horizontal_fov_deg`.

## Key parameters

| Parameter | Default | Meaning |
|---|---|---|
| `model_path` | `~/best.pt` | Your trained YOLOv8 weights |
| `image_topic` | `/image_raw` | Camera image in |
| `scan_topic` | `/scan` | LiDAR used for ranging |
| `conf_threshold` | `0.5` | Min detection confidence to map |
| `class_allowlist` | `[]` (all) | Only map these class names, e.g. `['door','chair']` |
| `fusion_method` | `lidar` | `lidar` (use range) or `pose` (stamp in front of robot) |
| `merge_radius` | `0.6` m | Same-class detections within this distance merge into one |
| `max_fusion_range` | `8.0` m | Ignore LiDAR returns beyond this |
| `camera_yaw_offset` | `0.0` rad | Camera-vs-LiDAR yaw, see calibration |
| `horizontal_fov_deg` | `70.0` | Camera horizontal FOV |
| `save_path` | `~/smorphi_semantic_map.yaml` | Where landmarks persist |

## Output YAML

```yaml
chair_1:
  class: chair
  position: {x: 2.31, y: -0.84, z: 0.0}
  confidence: 0.91
  observations: 14
door_1:
  class: door
  position: {x: 5.02, y: 1.10, z: 0.0}
  confidence: 0.78
  observations: 6
```

`observations` is how many times the object was seen and averaged in — higher means a
more reliable position estimate.

## Optional: launch it together with mapping

Add this node to `smorphi_mapper_online_async_launch.py` so semantic mapping starts
with SLAM. Example block:

```python
start_semantic_mapper = Node(
    package='smorphi_semantic_mapping',
    executable='semantic_mapper',
    name='semantic_mapper',
    parameters=[{'model_path': '/home/smorphi/best.pt',
                 'image_topic': '/image_raw',
                 'scan_topic': '/scan'}],
    output='screen')
# ... ld.add_action(start_semantic_mapper)
```
