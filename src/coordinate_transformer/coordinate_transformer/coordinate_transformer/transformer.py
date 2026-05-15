import numpy as np
from scipy.spatial.transform import Rotation as R


class PoseTransformer:
    """
    姿态变换类 - 使用 scipy 的 Rotation 类
    兼容参考代码 forest_searching/transformation/robot_pose/transformer.py 的实现
    """

    def __init__(self):
        pass

    def pose_to_matrix(self, x, y, z, qx, qy, qz, qw):
        """
        位姿 (坐标 + 四元数) 转 4x4 变换矩阵

        :param x, y, z: 位置 (米)
        :param qx, qy, qz, qw: 四元数
        :return: 4x4 numpy 数组
        """
        T = np.eye(4)
        T[:3, :3] = R.from_quat([qx, qy, qz, qw]).as_matrix()
        T[:3, 3] = [x, y, z]
        return T

    def matrix_to_pose(self, T):
        """
        4x4 变换矩阵 转位姿 (坐标 + 四元数)

        :param T: 4x4 numpy 数组
        :return: (x, y, z, qx, qy, qz, qw)
        """
        x, y, z = T[:3, 3]
        quat = R.from_matrix(T[:3, :3]).as_quat()
        qx, qy, qz, qw = quat
        return x, y, z, qx, qy, qz, qw

    def apply_transform(self, current_pose, transform_matrix):
        """
        应用变换矩阵到当前位姿

        :param current_pose: 元组 (x, y, z, qx, qy, qz, qw)
        :param transform_matrix: 4x4 numpy 数组
        :return: 变换后的新位姿 (x, y, z, qx, qy, qz, qw)
        """
        T_current = self.pose_to_matrix(*current_pose)
        T_new = transform_matrix @ T_current
        return self.matrix_to_pose(T_new)


class OffsetTransformer:
    """
    专门处理 odin SLAM 的坐标偏移

    由于 odin (传感器) 不在机器人中心，需要处理两类偏移:
    1. 传感器到机器人中心的偏移 (T_sensor_to_robot)
    2. map 坐标系原点的偏移 (T_map_origin_offset)
    """

    def __init__(self, sensor_offset, map_origin_offset=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)):
        """
        :param sensor_offset: 传感器相对机器人中心的偏移 (x, y, z, roll, pitch, yaw)
        :param map_origin_offset: map 原点偏移 (x, y, z, roll, pitch, yaw)
        """
        self.pose_transformer = PoseTransformer()
        self.T_sensor_to_robot = self._build_transform(sensor_offset)
        self.T_map_origin_offset = self._build_transform(map_origin_offset)
        self.T_robot_to_sensor = self._inverse_transform(self.T_sensor_to_robot)

    @staticmethod
    def _build_transform(offset):
        """
        从 (x, y, z, roll, pitch, yaw) 构建 4x4 变换矩阵

        :param offset: (x, y, z, roll, pitch, yaw) - 单位: 米 / 弧度
        :return: 4x4 numpy 数组
        """
        x, y, z, roll, pitch, yaw = offset
        T = np.eye(4)
        T[:3, :3] = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
        T[:3, 3] = [x, y, z]
        return T

    @staticmethod
    def _inverse_transform(T):
        """
        计算 4x4 变换矩阵的逆

        :param T: 4x4 numpy 数组
        :return: T 的逆矩阵
        """
        T_inv = np.eye(4)
        T_inv[:3, :3] = T[:3, :3].T
        T_inv[:3, 3] = -T[:3, :3].T @ T[:3, 3]
        return T_inv

    def odom_to_map_with_offset(self, odom_pose, tf_odom_to_map):
        """
        完整坐标变换: odom -> map，并考虑传感器偏移和 map 原点偏移

        变换链: P_robot_in_map = T_mo @ T_om @ T_ro @ P_robot_in_odom

        其中 P_robot_in_odom = P_sensor_in_odom @ inv(T_sensor_to_robot)
        因为 SLAM 给出的是传感器位姿，需要通过传感器偏移的逆变换得到机器人位姿

        :param odom_pose: odin 在 odom 坐标系下的位姿 (x, y, z, qx, qy, qz, qw)
        :param tf_odom_to_map: odom 到 map 的 tf 变换矩阵 (从 TF 树获取)
        :return: 变换后的 map 坐标系下的位姿 (x, y, z, qx, qy, qz, qw)
        """
        T_sensor_in_odom = self.pose_transformer.pose_to_matrix(*odom_pose)
        T_robot_in_odom = T_sensor_in_odom @ self.T_robot_to_sensor
        T_total = self.T_map_origin_offset @ tf_odom_to_map @ T_robot_in_odom
        return self.pose_transformer.matrix_to_pose(T_total)

    def transform_point(self, point, tf_matrix):
        """
        变换单个点 (无旋转分量)

        :param point: (x, y, z)
        :param tf_matrix: 4x4 变换矩阵
        :return: 变换后的点 (x, y, z)
        """
        p_homogeneous = np.array([point[0], point[1], point[2], 1.0])
        p_transformed = tf_matrix @ p_homogeneous
        return p_transformed[:3]

    def transform_point_cloud(self, points, tf_matrix):
        """
        批量变换点云

        :param points: Nx3 numpy 数组
        :param tf_matrix: 4x4 变换矩阵
        :return: 变换后的点云 Nx3
        """
        points_homogeneous = np.hstack([points, np.ones((points.shape[0], 1))])
        transformed = (tf_matrix @ points_homogeneous.T).T
        return transformed[:, :3]
