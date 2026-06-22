# coordinate_transformer

ROS2 package for Odin SLAM pose relocation. It subscribes to Odin odometry, looks up the latest `odom -> map` TF, compensates the planar sensor offset, and publishes a robot-center pose.

## What It Provides

- `coordinate_transformer`: online ROS2 node.
- `calibrate_sensor_offset`: offline rosbag calibration tool.
- `transformer_launch.py`: launch only the coordinate transformer node.
- `odin_transformer.launch.py`: launch Odin driver related nodes plus the coordinate transformer node.

Do not launch both `transformer_launch.py` and `odin_transformer.launch.py` at the same time unless you intentionally want two `coordinate_transformer` nodes. `odin_transformer.launch.py` already starts one.

## Node Behavior

### `coordinate_transformer`

Executable:

```bash
ros2 run coordinate_transformer coordinate_transformer
```

Main inputs:

| Input | Type | Default | Purpose |
|------|------|---------|---------|
| `/odin1/odometry_highfreq` | `nav_msgs/Odometry` | `odin_pose_topic` | Odin sensor pose in `source_frame` |
| `/tf` | `tf2_msgs/TFMessage` | TF buffer | latest transform from `source_frame` to `target_frame` |

Main output:

| Output | Type | Default | Purpose |
|------|------|---------|---------|
| `/odin1/relocation` | `geometry_msgs/PoseStamped` | `output_pose_topic` | compensated robot-center pose in `target_frame`, or `fallback_output_frame` before TF is initialized |

The odometry message `child_frame_id` is ignored. The node only uses `header.frame_id` as a sanity check and `pose.pose` as the sensor pose.

The transform chain is:

```text
T_map_sensor = T_map_odom @ T_odom_sensor
```

Then the planar lever-arm correction is:

```text
robot_xy = sensor_xy - R(yaw - sensor_offset.yaw) @ sensor_offset.xy
```

The published orientation currently stays equal to the transformed odometry orientation. The `sensor_offset.yaw` only affects the xy lever-arm direction.

Relocalization fallback:

- On startup, the node waits for the `target_frame <- source_frame` TF.
- If TF is still unavailable after `tf_initialization_timeout`, the node falls back to `/odin1/odometry_highfreq` and publishes in `fallback_output_frame` (`odom` by default).
- The terminal prints a clear warning when fallback starts, repeats a throttled warning while fallback continues, and prints an info message when TF becomes available again.
- Downstream nodes should check `/odin1/relocation.header.frame_id`: `map` means relocalized/map output, while `odom` means temporary fallback output.

### `calibrate_sensor_offset`

Executable:

```bash
ros2 run coordinate_transformer calibrate_sensor_offset <bag_dir>
```

It reads `/odin1/odometry_highfreq` from an offline rosbag and estimates `sensor_offset.x/y` from self-rotation. The idea is that during in-place rotation the sensor draws a circle, while the robot center should stay as still as possible.

Default output:

```text
src/coordinate_transformer/coordinate_transformer/output/calibrated.yaml
```

With diagnostics plot:

```bash
ros2 run coordinate_transformer calibrate_sensor_offset <bag_dir> --plot
```

This also writes:

```text
src/coordinate_transformer/coordinate_transformer/output/calibrated.png
```

The plot contains raw odometry xy, circle/ellipse fits, corrected center trajectory, radial residuals, and yaw coverage.

## Launch Files

### `transformer_launch.py`

Starts only:

| Node | Package | Executable | Purpose |
|------|---------|------------|---------|
| `coordinate_transformer` | `coordinate_transformer` | `coordinate_transformer` | online pose relocation |

Use this when Odin driver/TF sources are already running:

```bash
ros2 launch coordinate_transformer transformer_launch.py
```

### `odin_transformer.launch.py`

Starts a full Odin pipeline plus relocation:

| Node | Package | Executable | Purpose |
|------|---------|------------|---------|
| `host_sdk_sample` | `odin_ros_driver` | `host_sdk_sample` | Odin device/SLAM driver |
| `pcd2depth_ros2_node` | `odin_ros_driver` | `pcd2depth_ros2_node` | point cloud to depth image processing |
| `cloud_reprojection_ros2_node` | `odin_ros_driver` | `cloud_reprojection_ros2_node` | cloud reprojection processing |
| `image_overlay_node` | `odin_ros_driver` | `image_overlay_node` | image overlay processing |
| `rviz2` | `rviz2` | `rviz2` | visualization |
| `coordinate_transformer` | `coordinate_transformer` | `coordinate_transformer` | online pose relocation |

Use this when you want to start the Odin driver stack and relocation together:

```bash
ros2 launch coordinate_transformer odin_transformer.launch.py
```

If you see two relocation topics being published or duplicate node-name warnings, check whether another terminal already launched `coordinate_transformer`.

## Configuration

Default parameter file:

```text
src/coordinate_transformer/coordinate_transformer/config/default.yaml
```

Important parameters:

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `sensor_offset` | `[-0.395299, 0.019101, 0.0, 0.0, 0.0, pi]` | sensor pose relative to robot center: `[x, y, z, roll, pitch, yaw]` |
| `map_origin_offset` | `[0, 0, 0, 0, 0, 0]` | final position offset added after compensation |
| `source_frame` | `odom` | odometry pose frame |
| `target_frame` | `map` | output pose frame |
| `tf_timeout` | `0.05` | timeout for TF lookup |
| `mount_profile` | `yaw_180` | selected mount flow; default keeps the existing yaw 180 behavior |
| `fallback_when_tf_missing` | `true` | publish odom-frame fallback after TF initialization timeout |
| `tf_initialization_timeout` | `3.0` | seconds to wait for TF before entering fallback |
| `fallback_output_frame` | `odom` | frame id used while publishing fallback odometry output |
| `fallback_warn_period` | `5.0` | throttled warning period while fallback remains active |
| `odin_pose_topic` | `/odin1/odometry_highfreq` | odometry input topic |
| `output_pose_topic` | `/odin1/relocation` | relocated pose output topic |
| `publish_rate` | `100.0` | timer publish rate for cached latest pose |

`sensor_offset.x > 0` means the sensor is in front of the robot center. `sensor_offset.y > 0` means the sensor is to the left. In the current setup `sensor_offset.yaw = pi` means the sensor frame yaw is 180 degrees from the robot frame, but this yaw is used only for position compensation.

## Calibration Workflow

1. Put the robot approximately in-place rotation.
2. Record a rosbag containing `/odin1/odometry_highfreq`.
3. Run:

```bash
ros2 run coordinate_transformer calibrate_sensor_offset <bag_dir> --plot
```

4. Inspect `output/calibrated.png`.
5. Copy the recommended `sensor_offset` into `config/default.yaml` if the result is reasonable.

Good signs:

- `Corrected center RMSE` is small.
- The corrected center plot is compact.
- Yaw span covers at least one full rotation.
- Circle/ellipse diagnostics do not show large systematic distortion.

## Build

```bash
colcon build --packages-select coordinate_transformer
source install/setup.bash
```

## Dependencies

- ROS2 Humble
- `numpy`
- `scipy`
- `PyYAML`
- `matplotlib` for `--plot`
- `rosbag2_py` and `rosidl_runtime_py` for offline calibration
