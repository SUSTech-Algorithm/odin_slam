import numpy as np
from scipy.spatial.transform import Rotation as R


class PoseTransformer:
    """
    姿态变换类 - 使用 scipy 的 Rotation 类
    坐标系约定: FLU (front-X, left-Y, up-Z)
    欧拉角顺序: ZYX (yaw-pitch-roll, 先绕 Z 转, 再 pitch, 最后 roll)
    """

    def __init__(self):
        pass

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

    def apply_transform(self, current_pose, transform_matrix):
        """
        应用变换矩阵到当前位姿
        """
        T_current = self.pose_to_matrix(*current_pose)
        T_new = transform_matrix @ T_current
        return self.matrix_to_pose(T_new)


class OffsetTransformer:
    """
    专门处理 odin SLAM 的坐标偏移
    坐标系约定: FLU (front-X, left-Y, up-Z)
    欧拉角顺序: ZYX (yaw-pitch-roll)

    sensor_offset 表示 Odin 传感器坐标系在机器人中心/base_link 坐标系下的位姿，
    即 T_base_sensor。odom_orientation_frame 控制 odometry orientation 的语义:
    - planar: 平面机器人模型，y 固定为 0，位置和 yaw 用显式公式补偿。
    - sensor: odometry pose 是传感器坐标系位姿，输出姿态会应用完整外参旋转。
    - base: odometry orientation 已经是 base_link 朝向，只修正传感器原点平移。
    """

    def __init__(
        self,
        sensor_offset,
        map_origin_offset=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        odom_orientation_frame='planar',
    ):
        self.pose_transformer = PoseTransformer()
        if odom_orientation_frame not in ('planar', 'sensor', 'base'):
            raise ValueError('odom_orientation_frame must be "planar", "sensor", or "base"')

        # T_base_sensor: 传感器坐标系在机器人中心/base_link 坐标系下的位姿。
        self.T_base_sensor = self._build_transform(sensor_offset)
        self.T_sensor_base = self._inverse_transform(self.T_base_sensor)
        self.t_base_sensor = self.T_base_sensor[:3, 3].copy()

        # 兼容旧字段名，避免外部调试脚本直接访问属性时报错。
        self.T_robot_sensor = self.T_base_sensor
        self.T_sensor_robot = self.T_sensor_base
        self.T_sensor_to_robot = self.T_base_sensor
        self.T_robot_to_sensor = self.T_sensor_base

        self.sensor_offset = sensor_offset
        self.map_origin_offset = map_origin_offset
        self.odom_orientation_frame = odom_orientation_frame

    @staticmethod
    def _build_transform(offset):
        """
        从 (x, y, z, roll, pitch, yaw) 构建 4x4 变换矩阵。
        """
        x, y, z, roll, pitch, yaw = offset
        T = np.eye(4)
        T[:3, :3] = R.from_euler('ZYX', [yaw, pitch, roll]).as_matrix()
        T[:3, 3] = [x, y, z]
        return T

    @staticmethod
    def _inverse_transform(T):
        """
        计算 SE(3) 矩阵的精确逆 (解析逆)
        """
        T_inv = np.eye(4)
        T_inv[:3, :3] = T[:3, :3].T
        T_inv[:3, 3] = -T[:3, :3].T @ T[:3, 3]
        return T_inv

    def odom_to_map_with_offset(self, odom_pose, tf_odom_to_map):
        """
        将 Odin 传感器位姿转换为 map 坐标系下的机器人中心/base_link 位姿。

        变换链:
            T_map_sensor = T_map_odom @ T_odom_sensor
            T_map_base = T_map_sensor @ T_sensor_base
        """
        T_odom_sensor = self.pose_transformer.pose_to_matrix(*odom_pose)

        # T_map_sensor = T_map_odom @ T_odom_sensor
        T_map_sensor = tf_odom_to_map @ T_odom_sensor

        if self.odom_orientation_frame == 'planar':
            T_map_base = self._planar_sensor_to_base(T_map_sensor)
        elif self.odom_orientation_frame == 'base':
            T_map_base = T_map_sensor.copy()
            T_map_base[:3, 3] = (
                T_map_sensor[:3, 3]
                - T_map_sensor[:3, :3] @ self.t_base_sensor
            )
        else:
            # T_map_base = T_map_sensor @ T_sensor_base
            T_map_base = T_map_sensor @ self.T_sensor_base

        T_mo = self._build_transform(self.map_origin_offset)
        T_final = T_mo @ T_map_base

        x, y, z, qx, qy, qz, qw = self.pose_transformer.matrix_to_pose(T_final)

        return x, y, z, qx, qy, qz, qw

    def _planar_sensor_to_base(self, T_map_sensor):
        """
        平面机器人专用补偿。

        sensor_offset 的 y 视为 0。yaw_offset 只用于计算传感器杆臂方向，
        不改变最终输出的机器人 yaw。
        """
        x_offset = float(self.sensor_offset[0])
        z_offset = float(self.sensor_offset[2])
        yaw_offset = float(self.sensor_offset[5])

        yaw_sensor = R.from_matrix(T_map_sensor[:3, :3]).as_euler('ZYX')[0]
        yaw_for_translation = yaw_sensor - yaw_offset
        R_map_translation = R.from_euler(
            'ZYX',
            [yaw_for_translation, 0.0, 0.0]
        ).as_matrix()

        T_map_base = np.eye(4)
        T_map_base[:3, :3] = T_map_sensor[:3, :3]
        T_map_base[:3, 3] = (
            T_map_sensor[:3, 3]
            - R_map_translation @ np.array([x_offset, 0.0, z_offset])
        )
        return T_map_base

    def transform_point(self, point, tf_matrix):
        p_homogeneous = np.array([point[0], point[1], point[2], 1.0])
        p_transformed = tf_matrix @ p_homogeneous
        return p_transformed[:3]

    def transform_point_cloud(self, points, tf_matrix):
        points_homogeneous = np.hstack([points, np.ones((points.shape[0], 1))])
        transformed = (tf_matrix @ points_homogeneous.T).T
        return transformed[:, :3]
