#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import numpy as np

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformListener, Buffer
from tf2_py import LookupException, ExtrapolationException, ConnectivityException

from scipy.spatial.transform import Rotation as R

class CoordinateTransformer(Node):
    """
    坐标转换节点

    功能:
    1. 订阅 odom 坐标系下的 Odin 传感器位姿，转换为 map 坐标系
    2. 支持根据安装外参换算机器人中心/base_link 位姿
    3. 支持 map 原点偏移调整
    """

    def __init__(self):
        super().__init__('coordinate_transformer')

        # TF2 缓冲区和监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 从参数服务器加载配置
        self.declare_parameters(
            namespace='coordinate_transformer',
            parameters=[
                ('sensor_offset', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                ('map_origin_offset', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                ('source_frame', 'odom'),
                ('target_frame', 'map'),
                ('tf_timeout', 1.0),
                ('odom_orientation_frame', 'planar'),
                ('odin_pose_topic', '/odin_odom'),
                ('output_pose_topic', '/transformed/pose'),
                ('publish_transformed_pose', True),
                ('publish_rate', 10.0),
            ]
        )

        self.sensor_offset = self.get_parameter('coordinate_transformer.sensor_offset').value
        self.map_origin_offset = self.get_parameter('coordinate_transformer.map_origin_offset').value
        self.source_frame = self.get_parameter('coordinate_transformer.source_frame').value
        self.target_frame = self.get_parameter('coordinate_transformer.target_frame').value
        self.tf_timeout = self.get_parameter('coordinate_transformer.tf_timeout').value
        odom_orientation_frame = self.get_parameter(
            'coordinate_transformer.odom_orientation_frame'
        ).value
        self.odin_pose_topic = self.get_parameter('coordinate_transformer.odin_pose_topic').value
        self.output_pose_topic = self.get_parameter(
            'coordinate_transformer.output_pose_topic'
        ).value
        publish_pose = self.get_parameter('coordinate_transformer.publish_transformed_pose').value
        publish_rate = self.get_parameter('coordinate_transformer.publish_rate').value

        # 缓存最新收到的位姿
        self.latest_map_position = None
        self.latest_map_quaternion = None
        self.latest_stamp = None        

        self.get_logger().info(f"""
        Parameters loaded:
            Sensor offset: {self.sensor_offset}
            Map origin offset: {self.map_origin_offset}
            Transform: {self.source_frame} -> {self.target_frame}
            Odom orientation frame: {odom_orientation_frame}
            Odin pose topic: {self.odin_pose_topic}
            Output pose topic: {self.output_pose_topic}
            Publish rate: {publish_rate}
            """)

        # 订阅 odin 位姿 (来自 SLAM 的 odom 输出)
        self.odin_sub = self.create_subscription(
            Odometry,
            self.odin_pose_topic,
            self.odin_pose_callback,
            10
        )

        # 发布转换后的位姿
        self.pose_pub = None
        if publish_pose:
            self.pose_pub = self.create_publisher(
                PoseStamped,
                self.output_pose_topic,
                10
            )
            # 创建定时器控制发布频率
            self.publish_timer = self.create_timer(
                1.0 / publish_rate,
                self.publish_timer_callback
            )

        self.get_logger().info('Coordinate transformer initialized')

    def _lookup_latest_transform_matrix(self):
        """Look up the latest T_target_source matrix."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout)
            )
            return self._transform_to_matrix(transform)
        except LookupException as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=5)
        except ExtrapolationException as e:
            self.get_logger().warn(f'TF extrapolation failed: {e}', throttle_duration_sec=5)
        except ConnectivityException as e:
            self.get_logger().warn(f'TF connectivity failed: {e}', throttle_duration_sec=5)
        self.latest_map_position = None
        self.latest_map_quaternion = None
        self.latest_stamp = None
        return None

    def _transform_to_matrix(self, transform):
        """Convert a TransformStamped to a 4x4 transform matrix."""
        t = transform.transform.translation
        q = transform.transform.rotation
        return self.pose_to_matrix(t.x, t.y, t.z, q.x, q.y, q.z, q.w)

    def pose_to_matrix(self, x, y, z, qx, qy, qz, qw):
        """
        位姿 (坐标 + 四元数) 转 4x4 变换矩阵 (SE(3) 群元素)
        """
        T = np.eye(4)
        T[:3, :3] = R.from_quat([qx, qy, qz, qw]).as_matrix()
        T[:3, 3] = [x, y, z]
        return T

    def matrix_to_pose(self, T):
        """
        4x4 变换矩阵 转位姿 (坐标 + 四元数)
        """
        x, y, z = T[:3, 3]
        quat = R.from_matrix(T[:3, :3]).as_quat()
        qx, qy, qz, qw = quat
        return x, y, z, qx, qy, qz, qw

    def _transform_from_odom_to_map(self, map_pose_position: list, map_pose_yaw: float):
        radius = np.sqrt(self.sensor_offset[0]**2 + self.sensor_offset[1]**2)
        if radius < 1e-9:
            return [
                map_pose_position[0] + self.map_origin_offset[0],
                map_pose_position[1] + self.map_origin_offset[1],
                map_pose_position[2],
            ]
        cos_alpha = -self.sensor_offset[0] / radius
        sin_alpha = self.sensor_offset[1] / radius
        position = [0.0, 0.0, 0.0]
        position[0] = map_pose_position[0] - radius*(cos_alpha - np.cos(map_pose_yaw)) + self.map_origin_offset[0]
        position[1] = map_pose_position[1] - radius*(sin_alpha + np.sin(map_pose_yaw)) + self.map_origin_offset[1]
        position[2] = map_pose_position[2]
        return position

    def odin_pose_callback(self, msg: Odometry):
        """处理 odin 位姿，计算并缓存 map 坐标系下的结果"""
        if msg.header.frame_id and msg.header.frame_id != self.source_frame:
            self.get_logger().warn(
                f'Odom message frame_id "{msg.header.frame_id}" does not match '
                f'source_frame "{self.source_frame}"',
                throttle_duration_sec=5
            )

        tf_matrix = self._lookup_latest_transform_matrix()
        if tf_matrix is None:
            return

        # 提取 SLAM 套件/传感器在 source_frame 下的位姿；child_frame_id 不参与数学计算。
        odom_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        odom_pose_matrix = self.pose_to_matrix(*odom_pose)
        map_pose_matrix = tf_matrix @ odom_pose_matrix 
        map_pose = self.matrix_to_pose(map_pose_matrix)
        map_pose_yaw = R.from_quat([map_pose[3], map_pose[4], map_pose[5], map_pose[6]]).as_euler('ZYX')[0]
        map_pose_position = [-map_pose[0], -map_pose[1], map_pose[2]]
        self.latest_map_position = self._transform_from_odom_to_map(map_pose_position, map_pose_yaw)
        self.latest_map_quaternion = [map_pose[3], map_pose[4], map_pose[5], map_pose[6]]
        self.latest_stamp = msg.header.stamp

    def publish_timer_callback(self):
        """定时器回调，按固定频率发布缓存的位姿"""
        if (
            self.latest_map_position is None
            or self.latest_map_quaternion is None
            or self.pose_pub is None
        ):
            return

        transformed_pose = PoseStamped()
        transformed_pose.header.frame_id = self.target_frame
        transformed_pose.header.stamp = self.latest_stamp
        transformed_pose.pose.position.x = self.latest_map_position[0]
        transformed_pose.pose.position.y = self.latest_map_position[1]
        transformed_pose.pose.position.z = self.latest_map_position[2]
        transformed_pose.pose.orientation.x = self.latest_map_quaternion[0]
        transformed_pose.pose.orientation.y = self.latest_map_quaternion[1]
        transformed_pose.pose.orientation.z = self.latest_map_quaternion[2]
        transformed_pose.pose.orientation.w = self.latest_map_quaternion[3]

        self.pose_pub.publish(transformed_pose)

def main(args=None):
    rclpy.init(args=args)
    node = CoordinateTransformer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
