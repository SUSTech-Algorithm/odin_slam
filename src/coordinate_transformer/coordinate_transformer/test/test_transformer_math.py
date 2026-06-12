import numpy as np
from scipy.spatial.transform import Rotation as R

from coordinate_transformer.calibrate_sensor_x import calibrate_x
from coordinate_transformer.transformer import OffsetTransformer, PoseTransformer


def make_pose_matrix(x, y, yaw):
    transformer = PoseTransformer()
    quat = R.from_euler('ZYX', [yaw, 0.0, 0.0]).as_quat()
    return transformer.pose_to_matrix(x, y, 0.0, *quat)


def test_sensor_x_offset_keeps_rotating_base_center_stable():
    sensor_offset = [0.4, 0.0, 0.0, 0.0, 0.0, 0.0]
    transformer = OffsetTransformer(sensor_offset)
    pose_transformer = PoseTransformer()
    t_base_sensor = OffsetTransformer._build_transform(sensor_offset)

    centers = []
    for yaw in np.linspace(-np.pi, np.pi, 25):
        t_map_base = make_pose_matrix(1.2, -0.7, yaw)
        t_map_sensor = t_map_base @ t_base_sensor
        odom_pose = pose_transformer.matrix_to_pose(t_map_sensor)
        output_pose = transformer.odom_to_map_with_offset(odom_pose, np.eye(4))
        centers.append(output_pose[:3])

    centers = np.array(centers)
    assert np.allclose(centers[:, 0], 1.2, atol=1e-9)
    assert np.allclose(centers[:, 1], -0.7, atol=1e-9)
    assert np.allclose(centers[:, 2], 0.0, atol=1e-9)


def test_calibrate_x_recovers_known_sensor_offset():
    sensor_offset = [0.37, 0.0, 0.0, 0.0, 0.0, np.pi]
    t_base_sensor = OffsetTransformer._build_transform(sensor_offset)

    samples = []
    for yaw in np.linspace(-1.5 * np.pi, 1.5 * np.pi, 60):
        t_map_base = make_pose_matrix(-0.2, 0.3, yaw)
        samples.append(t_map_base @ t_base_sensor)

    result = calibrate_x(np.stack(samples), sensor_offset)
    assert np.isclose(result['x'], sensor_offset[0], atol=1e-9)
    assert result['corrected_position_rmse'] < 1e-9
