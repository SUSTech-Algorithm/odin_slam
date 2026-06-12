#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import numpy as np

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformListener, Buffer
from tf2_py import LookupException, ExtrapolationException, ConnectivityException

from .transformer import OffsetTransformer, PoseTransformer


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
                ('odom_orientation_frame', 'base'),
                ('odin_pose_topic', '/odin_odom'),
                ('output_pose_topic', '/transformed/pose'),
                ('publish_transformed_pose', True),
                ('publish_rate', 10.0),
            ]
        )

        sensor_offset = self.get_parameter('coordinate_transformer.sensor_offset').value
        map_origin_offset = self.get_parameter('coordinate_transformer.map_origin_offset').value
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
        self.latest_map_pose = None
        self.latest_stamp = None

        # 初始化变换器
        self.transformer = OffsetTransformer(
            sensor_offset,
            map_origin_offset,
            odom_orientation_frame=odom_orientation_frame,
        )
        self.pose_transformer = PoseTransformer()

        self.get_logger().info(f"""
        Parameters loaded:
            Sensor offset: {sensor_offset}
            Map origin offset: {map_origin_offset}
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
        self.latest_map_pose = None
        self.latest_stamp = None
        return None

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

        self.latest_map_pose = self.transformer.odom_to_map_with_offset(
            odom_pose,
            tf_matrix
        )
        self.latest_stamp = msg.header.stamp

    def publish_timer_callback(self):
        """定时器回调，按固定频率发布缓存的位姿"""
        if (
            self.latest_map_pose is None
            or self.pose_pub is None
        ):
            return

        transformed_pose = PoseStamped()
        transformed_pose.header.frame_id = self.target_frame
        transformed_pose.header.stamp = self.latest_stamp
        transformed_pose.pose.position.x = self.latest_map_pose[0]
        transformed_pose.pose.position.y = self.latest_map_pose[1]
        transformed_pose.pose.position.z = self.latest_map_pose[2]
        transformed_pose.pose.orientation.x = self.latest_map_pose[3]
        transformed_pose.pose.orientation.y = self.latest_map_pose[4]
        transformed_pose.pose.orientation.z = self.latest_map_pose[5]
        transformed_pose.pose.orientation.w = self.latest_map_pose[6]

        self.pose_pub.publish(transformed_pose)

    def _transform_to_matrix(self, transform: TransformStamped) -> np.ndarray:
        """将 TF 消息转换为 4x4 矩阵"""
        t = transform.transform.translation
        q = transform.transform.rotation

        return self.pose_transformer.pose_to_matrix(t.x, t.y, t.z, q.x, q.y, q.z, q.w)


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
