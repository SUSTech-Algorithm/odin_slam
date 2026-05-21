# coordinate_transformer

ROS2 坐标系转换包，用于 odin SLAM 系统。将 Odin 传感器位姿转换为 map 坐标系下的机器人中心/base_link 位姿，并支持 map 原点偏移调整。

## 功能

- **odom → map 坐标变换**: 订阅 odom 坐标系下的 Odin 传感器位姿，转换到 map 坐标系
- **传感器偏移补偿**: 根据安装外参，将 Odin 传感器 pose 换算成机器人中心/base_link pose
- **map 原点偏移**: 支持移动 map 坐标系原点到任意位置
- **点云转换**: 支持点云数据的坐标系转换

## 依赖

- ROS2 Humble
- numpy
- scipy

## 构建

```bash
# 进入工作空间
cd ~/your_workspace

# 构建包
colcon build --packages-select coordinate_transformer

# source 环境
source install/setup.bash
```

## 配置

配置文件位于 `config/default.yaml`:

```yaml
coordinate_transformer:
  ros__parameters:
    # Topic 配置
    odin_pose_topic: '/odin_odom'
    output_pose_topic: '/transformed/pose'

    # 传感器在机器人中心/base_link 坐标系下的位姿 [x, y, z, roll, pitch, yaw]
    # 单位: 米 / 弧度
    sensor_offset: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # map 坐标系原点偏移 [x, y, z, roll, pitch, yaw]
    map_origin_offset: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # TF 配置
    source_frame: 'odom'
    target_frame: 'map'
    tf_timeout: 1.0

    # 发布选项
    publish_transformed_pose: True
```

### Topic 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `odin_pose_topic` | string | '/odin_odom' | odin 位姿输入话题 |
| `output_pose_topic` | string | '/transformed/pose' | 转换后位姿输出话题 |

### 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `sensor_offset` | list[6] | [0,0,0,0,0,0] | Odin 传感器坐标系在机器人中心/base_link 坐标系下的位姿 (x,y,z,m + roll,pitch,yaw,rad)，即 `T_robot_sensor` |
| `map_origin_offset` | list[6] | [0,0,0,0,0,0] | map 原点偏移量 |
| `source_frame` | string | 'odom' | 源坐标系 |
| `target_frame` | string | 'map' | 目标坐标系 |
| `tf_timeout` | float | 1.0 | TF 查询超时时间 (秒) |

## 使用

### 方式一: 使用 launch 文件

```bash
ros2 launch coordinate_transformer transformer_launch.py
```

### 方式二: 直接运行节点

```bash
ros2 run coordinate_transformer coordinate_transformer --ros-args --params-file $(find coordinate_transformer)/config/default.yaml
```

### 方式三: 运行时覆盖参数

```bash
ros2 run coordinate_transformer coordinate_transformer --ros-args \
  -p sensor_offset:="[0.3, 0.0, 0.5, 0.0, 0.0, 0.0]" \
  -p map_origin_offset:="[10.0, 5.0, 0.0, 0.0, 0.0, 0.0]"
```

## 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/odin_odom` | nav_msgs/Odometry | Odin 传感器在 odom 下的位姿 |
| `/points` | sensor_msgs/PointCloud2 | 待转换的点云 |

## 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/transformed/pose` | geometry_msgs/PoseStamped | map 坐标系下的机器人中心/base_link 位姿 |
| `/transformed/points` | sensor_msgs/PointCloud2 | 转换到 map 坐标系的点云 |

## 坐标变换原理

目标输出是 `T_map_robot`：机器人中心/base_link 坐标系在 map 坐标系下的位姿。

约定:

- `T_map_odom`: TF 中 `odom -> map` 的变换，即 map 坐标系下的 odom 位姿
- `T_odom_sensor`: Odin SLAM 输出的传感器位姿
- `T_robot_sensor`: `sensor_offset`，传感器坐标系在机器人中心/base_link 坐标系下的位姿
- `T_sensor_robot`: `T_robot_sensor` 的逆，用于从传感器位姿换算到机器人中心/base_link 位姿

完整 pose 变换链:

```
T_map_sensor = T_map_odom @ T_odom_sensor
T_map_robot  = T_map_sensor @ T_sensor_robot
T_output     = T_map_offset @ T_map_robot
```

即: 先把 Odin 传感器 pose 表达到 map 中，再根据安装外参换算成机器人中心/base_link pose。这里不会对整个 map 坐标系做共轭变换。

### 传感器偏移

如果 Odin 传感器不在机器人中心，需要测量并配置 `sensor_offset`。这个偏移描述的是传感器坐标系在机器人中心/base_link 坐标系下的位姿。机器人原地自转时，原始 Odin 传感器位置可能绕机器人中心画圆；转换后的机器人中心位置应基本保持不动，主要只有 yaw 变化。

## 标定方法

### 传感器偏移标定

1. 将机器人放置在已知位置
2. 以机器人中心/base_link 为参考，测量传感器原点的位置 (x, y, z)
3. 测量传感器坐标系相对于机器人中心/base_link 坐标系的安装角 (roll, pitch, yaw)
4. 将测量值填入 `sensor_offset`

当前 `default.yaml` 中的 `sensor_offset` 是根据一段原地自转 rosbag 初步拟合出的值，用于减少旋转时的半径残差。它不应替代实际机械测量；如果传感器安装位置变化，需要重新标定。

### map 原点标定

1. 确定你想要的 map 原点位置 (例如场地左下角)
2. 测量该位置相对于 SLAM 原点的偏移
3. 将偏移值填入 `map_origin_offset`

## 示例配置

### 示例 1: 基本配置

```yaml
coordinate_transformer:
  ros__parameters:
    sensor_offset: [0.3, 0.0, 0.2, 0.0, 0.0, 0.0]
    map_origin_offset: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

### 示例 2: 带 map 原点偏移

```yaml
coordinate_transformer:
  ros__parameters:
    sensor_offset: [0.3, 0.0, 0.2, 0.0, 0.0, 0.0]
    map_origin_offset: [10.0, 5.0, 0.0, 0.0, 0.0, 0.0]
```

### 示例 3: 带角度偏移

```yaml
coordinate_transformer:
  ros__parameters:
    sensor_offset: [0.3, 0.0, 0.2, 0.0, 0.0, 1.5708]  # 90度安装偏差
    map_origin_offset: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

## API

### Python API

```python
from coordinate_transformer.transformer import OffsetTransformer, PoseTransformer
import numpy as np

# 初始化
transformer = OffsetTransformer(
    sensor_offset=(0.3, 0.0, 0.2, 0.0, 0.0, 0.0),
    map_origin_offset=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
)

# 位姿变换
odom_pose = (1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0)  # x,y,z,qx,qy,qz,qw
tf_odom_to_map = np.eye(4)  # 从 TF 树获取
map_pose = transformer.odom_to_map_with_offset(odom_pose, tf_odom_to_map)

# 点变换
point = (1.0, 2.0, 3.0)
transformed_point = transformer.transform_point(point, tf_matrix)
```
