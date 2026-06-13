#!/usr/bin/env python3
"""Calibrate planar sensor_offset x/y from an offline ROS 2 bag."""

import argparse
from pathlib import Path

import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R


def default_package_root():
    """Return the source/install package root that contains config/."""
    return Path(__file__).resolve().parents[1]


def pose_to_matrix(pose):
    """Convert a geometry_msgs Pose to a 4x4 matrix."""
    p = pose.position
    q = pose.orientation
    matrix = np.eye(4)
    matrix[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    matrix[:3, 3] = [p.x, p.y, p.z]
    return matrix


def read_template(params_file):
    """Read the parameter template and return its parameter dictionary."""
    with open(params_file, 'r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream)

    try:
        params = data['coordinate_transformer']['ros__parameters']
    except (TypeError, KeyError) as exc:
        raise ValueError(
            f'Invalid coordinate_transformer params file: {params_file}'
        ) from exc

    return data, params


def read_odometry_samples(bag_dir, odom_topic, source_frame):
    """Read odometry pose matrices from a ROS 2 bag."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )

    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    if odom_topic not in topic_types:
        raise ValueError(f'Missing odometry topic in bag: {odom_topic}')

    odom_type = get_message(topic_types[odom_topic])
    samples = []
    frame_warnings = 0

    while reader.has_next():
        topic, data, _bag_stamp = reader.read_next()
        if topic != odom_topic:
            continue

        msg = deserialize_message(data, odom_type)
        if msg.header.frame_id and msg.header.frame_id != source_frame:
            frame_warnings += 1
        samples.append(pose_to_matrix(msg.pose.pose))

    if frame_warnings:
        print(
            f'Warning: {frame_warnings} odometry messages had frame_id different '
            f'from source_frame "{source_frame}". child_frame_id was ignored.'
        )
    if not samples:
        raise ValueError(f'No odometry samples found on {odom_topic}')

    return np.stack(samples)


def rotation_for_offset(yaw, yaw_offset, odom_orientation_frame):
    """Return the planar rotation used to map sensor_offset xy into odom/map xy."""
    if odom_orientation_frame in ('planar', 'sensor'):
        yaw_for_translation = yaw - yaw_offset
    elif odom_orientation_frame == 'base':
        yaw_for_translation = yaw
    else:
        raise ValueError('odom_orientation_frame must be "planar", "sensor", or "base"')

    return R.from_euler('ZYX', [yaw_for_translation, 0.0, 0.0]).as_matrix()[:2, :2]


def calibrate_x(odom_samples, sensor_offset, odom_orientation_frame='planar'):
    """Estimate planar sensor_offset x/y and self-rotation center."""
    sensor_offset = np.array(sensor_offset, dtype=float)
    if len(sensor_offset) != 6:
        raise ValueError('sensor_offset must contain 6 values')
    if len(odom_samples) < 3:
        raise ValueError('At least 3 odometry samples are required')

    yaw_offset = float(sensor_offset[5])
    positions = odom_samples[:, :2, 3]
    yaws = np.array([
        R.from_matrix(sample[:3, :3]).as_euler('ZYX')[0]
        for sample in odom_samples
    ])

    a_rows = []
    b_rows = []
    rotations = []
    for position, yaw in zip(positions, yaws):
        rotation = rotation_for_offset(yaw, yaw_offset, odom_orientation_frame)
        rotations.append(rotation)
        a_rows.append([1.0, 0.0, rotation[0, 0], rotation[0, 1]])
        b_rows.append(position[0])
        a_rows.append([0.0, 1.0, rotation[1, 0], rotation[1, 1]])
        b_rows.append(position[1])

    solution, *_unused = np.linalg.lstsq(
        np.array(a_rows),
        np.array(b_rows),
        rcond=None,
    )
    center_xy = solution[:2]
    offset_xy = solution[2:4]

    centers = np.array([
        position - rotation @ offset_xy
        for position, rotation in zip(positions, rotations)
    ])
    residuals = centers - centers.mean(axis=0)
    residual_rmse = float(np.sqrt(np.mean(np.sum(residuals * residuals, axis=1))))
    raw_centered = positions - positions.mean(axis=0)
    raw_rmse = float(np.sqrt(np.mean(np.sum(raw_centered * raw_centered, axis=1))))
    yaw_span = float(np.ptp(np.unwrap(yaws)))

    return {
        'x': float(offset_xy[0]),
        'y': float(offset_xy[1]),
        'center_x': float(center_xy[0]),
        'center_y': float(center_xy[1]),
        'samples': int(len(odom_samples)),
        'yaw_span_rad': yaw_span,
        'yaw_span_deg': float(np.degrees(yaw_span)),
        'raw_position_rmse': raw_rmse,
        'corrected_center_rmse': residual_rmse,
    }


def fit_circle_and_ellipse(odom_samples):
    """Fit circle/ellipse diagnostics to odometry xy trajectory."""
    positions = odom_samples[:, :2, 3]
    if len(positions) < 5:
        raise ValueError('At least 5 odometry samples are required for ellipse fitting')

    center0 = positions.mean(axis=0)
    radius0 = np.linalg.norm(positions - center0, axis=1).mean()

    def circle_residual(values):
        center_x, center_y, radius = values
        radii = np.linalg.norm(positions - [center_x, center_y], axis=1)
        return radii - radius

    circle_solution = least_squares(
        circle_residual,
        [center0[0], center0[1], radius0],
    )
    circle_x, circle_y, circle_radius = circle_solution.x
    circle_residuals = circle_residual(circle_solution.x)
    circle_rmse = float(np.sqrt(np.mean(circle_residuals * circle_residuals)))

    centered = positions - center0
    covariance = np.cov(centered.T)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    order = np.argsort(eigenvalues)[::-1]
    eigenvalues = eigenvalues[order]
    eigenvectors = eigenvectors[:, order]
    theta0 = float(np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0]))
    semi_major0, semi_minor0 = np.sqrt(np.maximum(eigenvalues * 2.0, 1e-12))
    if semi_major0 < semi_minor0:
        semi_major0, semi_minor0 = semi_minor0, semi_major0
        theta0 += np.pi / 2.0

    def ellipse_residual(values):
        center_x, center_y, log_a, log_b, theta = values
        semi_major = np.exp(log_a)
        semi_minor = np.exp(log_b)
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        x = positions[:, 0] - center_x
        y = positions[:, 1] - center_y
        x_rot = cos_theta * x + sin_theta * y
        y_rot = -sin_theta * x + cos_theta * y
        return (x_rot / semi_major) ** 2 + (y_rot / semi_minor) ** 2 - 1.0

    ellipse_solution = least_squares(
        ellipse_residual,
        [
            center0[0],
            center0[1],
            np.log(semi_major0),
            np.log(semi_minor0),
            theta0,
        ],
        max_nfev=20000,
    )
    ellipse_x, ellipse_y, log_a, log_b, theta = ellipse_solution.x
    semi_major = float(np.exp(log_a))
    semi_minor = float(np.exp(log_b))
    if semi_minor > semi_major:
        semi_major, semi_minor = semi_minor, semi_major
        theta += np.pi / 2.0

    eccentricity = float(
        np.sqrt(max(0.0, 1.0 - (semi_minor * semi_minor) / (semi_major * semi_major)))
    )
    axis_ratio = float(semi_minor / semi_major)

    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    x = positions[:, 0] - ellipse_x
    y = positions[:, 1] - ellipse_y
    x_rot = cos_theta * x + sin_theta * y
    y_rot = -sin_theta * x + cos_theta * y
    angle = np.arctan2(y_rot, x_rot)
    ellipse_radius = 1.0 / np.sqrt(
        (np.cos(angle) / semi_major) ** 2
        + (np.sin(angle) / semi_minor) ** 2
    )
    point_radius = np.sqrt(x_rot * x_rot + y_rot * y_rot)
    ellipse_radial_residuals = point_radius - ellipse_radius
    ellipse_radial_rmse = float(
        np.sqrt(np.mean(ellipse_radial_residuals * ellipse_radial_residuals))
    )

    return {
        'circle_center_x': float(circle_x),
        'circle_center_y': float(circle_y),
        'circle_radius': float(circle_radius),
        'circle_radial_rmse': circle_rmse,
        'circle_relative_rmse': float(circle_rmse / abs(circle_radius)),
        'ellipse_center_x': float(ellipse_x),
        'ellipse_center_y': float(ellipse_y),
        'ellipse_semi_major': semi_major,
        'ellipse_semi_minor': semi_minor,
        'ellipse_axis_ratio': axis_ratio,
        'ellipse_eccentricity': eccentricity,
        'ellipse_major_axis_rad': float(theta),
        'ellipse_major_axis_deg': float(np.degrees(theta)),
        'ellipse_radial_rmse': ellipse_radial_rmse,
    }


def write_calibrated_yaml(data, params, result, output_path, write_map_origin):
    """Write a complete calibrated ROS 2 params file."""
    sensor_offset = list(params.get('sensor_offset', [0.0] * 6))
    if len(sensor_offset) != 6:
        raise ValueError('sensor_offset must contain 6 values')

    sensor_offset[0] = result['x']
    sensor_offset[1] = result['y']
    params['sensor_offset'] = [float(value) for value in sensor_offset]

    if write_map_origin:
        map_origin_offset = list(params.get('map_origin_offset', [0.0] * 6))
        if len(map_origin_offset) != 6:
            raise ValueError('map_origin_offset must contain 6 values')
        map_origin_offset[0] = -result['center_x']
        map_origin_offset[1] = -result['center_y']
        params['map_origin_offset'] = [float(value) for value in map_origin_offset]

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, 'w', encoding='utf-8') as stream:
        yaml.safe_dump(data, stream, sort_keys=False, allow_unicode=True)


def parse_args():
    """Parse command-line arguments."""
    package_root = default_package_root()
    default_params = package_root / 'config' / 'default.yaml'
    default_output = package_root / 'config' / 'calibrated.yaml'

    parser = argparse.ArgumentParser(
        description='Calibrate planar sensor_offset x/y from self-rotation odometry.'
    )
    parser.add_argument('bag_dir', type=Path, help='Path to a rosbag2 directory')
    parser.add_argument('--params-file', type=Path, default=default_params)
    parser.add_argument('--output', type=Path, default=default_output)
    parser.add_argument('--odom-topic', default='/odin1/odometry_highfreq')
    parser.add_argument('--source-frame', default=None)
    parser.add_argument(
        '--odom-orientation-frame',
        choices=['planar', 'sensor', 'base'],
        default=None,
    )
    parser.add_argument(
        '--write-map-origin',
        action='store_true',
        help='Also write -rotation_center into map_origin_offset x/y.',
    )
    return parser.parse_args()


def main():
    """Run planar sensor_offset calibration and write calibrated.yaml."""
    args = parse_args()
    data, params = read_template(args.params_file)
    source_frame = args.source_frame or params.get('source_frame', 'odom')
    odom_orientation_frame = (
        args.odom_orientation_frame
        or params.get('odom_orientation_frame', 'planar')
    )

    sensor_offset = params.get('sensor_offset', [0.0] * 6)
    if len(sensor_offset) != 6:
        raise ValueError('sensor_offset must contain 6 values')

    odom_samples = read_odometry_samples(
        args.bag_dir,
        args.odom_topic,
        source_frame,
    )
    result = calibrate_x(
        odom_samples,
        sensor_offset,
        odom_orientation_frame=odom_orientation_frame,
    )
    shape = fit_circle_and_ellipse(odom_samples)
    write_calibrated_yaml(
        data,
        params,
        result,
        args.output,
        args.write_map_origin,
    )

    calibrated_offset = list(sensor_offset)
    calibrated_offset[0] = result['x']
    calibrated_offset[1] = result['y']

    print('Recommended sensor_offset:')
    print(
        '['
        + ', '.join(f'{float(value):.6f}' for value in calibrated_offset)
        + ']'
    )
    print(f'Calibrated sensor_offset.x: {result["x"]:.6f} m')
    print(f'Calibrated sensor_offset.y: {result["y"]:.6f} m')
    print(
        'Estimated self-rotation center in odom xy: '
        f'({result["center_x"]:.6f}, {result["center_y"]:.6f}) m'
    )
    print(
        'If you want this center to become the origin, use map_origin_offset xy: '
        f'({-result["center_x"]:.6f}, {-result["center_y"]:.6f}) m'
    )
    print(f'Samples used: {result["samples"]}')
    print(
        f'Yaw span: {result["yaw_span_rad"]:.3f} rad '
        f'({result["yaw_span_deg"]:.1f} deg)'
    )
    print(f'Raw position RMSE: {result["raw_position_rmse"]:.6f} m')
    print(f'Corrected center RMSE: {result["corrected_center_rmse"]:.6f} m')
    print('Trajectory shape diagnostics:')
    print(
        '  Circle center/radius: '
        f'({shape["circle_center_x"]:.6f}, {shape["circle_center_y"]:.6f}), '
        f'r={shape["circle_radius"]:.6f} m'
    )
    print(
        '  Circle radial RMSE: '
        f'{shape["circle_radial_rmse"]:.6f} m '
        f'({shape["circle_relative_rmse"]:.3%} of radius)'
    )
    print(
        '  Ellipse center: '
        f'({shape["ellipse_center_x"]:.6f}, {shape["ellipse_center_y"]:.6f})'
    )
    print(
        '  Ellipse semi axes: '
        f'a={shape["ellipse_semi_major"]:.6f} m, '
        f'b={shape["ellipse_semi_minor"]:.6f} m, '
        f'b/a={shape["ellipse_axis_ratio"]:.6f}'
    )
    print(f'  Ellipse eccentricity: {shape["ellipse_eccentricity"]:.6f}')
    print(
        '  Ellipse major axis angle: '
        f'{shape["ellipse_major_axis_rad"]:.6f} rad '
        f'({shape["ellipse_major_axis_deg"]:.1f} deg)'
    )
    print(f'  Ellipse radial RMSE: {shape["ellipse_radial_rmse"]:.6f} m')
    print(f'Wrote calibrated params: {args.output}')


if __name__ == '__main__':
    main()
