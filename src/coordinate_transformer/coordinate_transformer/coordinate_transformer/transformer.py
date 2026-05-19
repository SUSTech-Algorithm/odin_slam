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
    """

    def __init__(self, sensor_offset, map_origin_offset=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)):
        self.pose_transformer = PoseTransformer()
        
        # T_sensor_to_robot: 描述传感器在机器人底盘坐标系中的位姿 (即 T_B^S)
        self.T_sensor_to_robot = self._build_transform(sensor_offset)
        # T_robot_to_sensor: 描述底盘在传感器坐标系中的位姿 (即 (T_B^S)^-1)
        self.T_robot_to_sensor = self._inverse_transform(self.T_sensor_to_robot)
        
        self.sensor_offset = sensor_offset
        self.map_origin_offset = map_origin_offset

    @staticmethod
    def _build_transform(offset):
        """
        从 (x, y, z, roll, pitch, yaw) 构建 4x4 变换矩阵。
        """
        x, y, z, roll, pitch, yaw = offset
        T = np.eye(4)
        
        # 【核心修复 1】: Scipy API 传参顺序修正
        # 当指定轴序为 'ZYX' 时，传入的数组必须严格按照 [yaw, pitch, roll] 顺序
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
        终极坐标变换：将 SLAM 颠倒的全局坐标系，彻底翻转并映射为符合机器人底盘直觉的 User Map。
        """
        # 1. 提取传感器在 Odom 下的局部相对位姿
        T_sensor_in_odom = self.pose_transformer.pose_to_matrix(*odom_pose)

        # 2. 组合 TF，得到传感器在 SLAM Map 中的绝对位姿 (T_slam_map_to_sensor)
        T_sensor_in_slam_map = tf_odom_to_map @ T_sensor_in_odom

        # 3. 【核心数学重构：全局共轭变换】
        # 将整个 SLAM 的绝对位姿，通过外参共轭映射，强行转换到以车头为基准的世界坐标系中。
        # 公式: T_user_map = T_base_to_sensor * T_slam_pose * T_sensor_to_base
        T_robot_in_user_map = self.T_sensor_to_robot @ T_sensor_in_slam_map @ self.T_robot_to_sensor

        # 4. 应用可能的用户自定义 Map 原点偏移
        T_mo = self._build_transform(self.map_origin_offset)
        T_final = T_mo @ T_robot_in_user_map

        x, y, z, qx, qy, qz, qw = self.pose_transformer.matrix_to_pose(T_final)

        return x, y, z, qx, qy, qz, qw

    def transform_point(self, point, tf_matrix):
        p_homogeneous = np.array([point[0], point[1], point[2], 1.0])
        p_transformed = tf_matrix @ p_homogeneous
        return p_transformed[:3]

    def transform_point_cloud(self, points, tf_matrix):
        points_homogeneous = np.hstack([points, np.ones((points.shape[0], 1))])
        transformed = (tf_matrix @ points_homogeneous.T).T
        return transformed[:, :3]