#!/usr/bin/env python3
"""Calibrate the sensor x offset from an offline ROS 2 bag."""

import argparse
from bisect import bisect_left
from pathlib import Path

import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation as R

from .transformer import OffsetTransformer, PoseTransformer


def stamp_to_ns(stamp):
    """Convert a ROS stamp to integer nanoseconds."""
    return stamp.sec * 1000000000 + stamp.nanosec


def default_package_root():
    """Return the source/install package root that contains config/."""
    return Path(__file__).resolve().parents[1]


def transform_to_matrix(transform):
    """Convert a TransformStamped transform field to a 4x4 matrix."""
    t = transform.translation
    q = transform.rotation
    return PoseTransformer().pose_to_matrix(t.x, t.y, t.z, q.x, q.y, q.z, q.w)


def pose_to_matrix(pose):
    """Convert a geometry_msgs Pose to a 4x4 matrix."""
    p = pose.position
    q = pose.orientation
    return PoseTransformer().pose_to_matrix(p.x, p.y, p.z, q.x, q.y, q.z, q.w)


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


def read_bag_samples(args):
    """Read odometry samples and map<-source TF samples from a ROS 2 bag."""
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(
        uri=str(args.bag_dir),
        storage_id='sqlite3',
    )
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    required = [args.odom_topic, args.tf_topic]
    missing = [topic for topic in required if topic not in topic_types]
    if missing:
        raise ValueError(f'Missing topic(s) in bag: {", ".join(missing)}')

    odom_type = get_message(topic_types[args.odom_topic])
    tf_type = get_message(topic_types[args.tf_topic])

    odom_samples = []
    tf_samples = []
    frame_warnings = 0

    while reader.has_next():
        topic, data, bag_stamp = reader.read_next()
        if topic == args.odom_topic:
            msg = deserialize_message(data, odom_type)
            if msg.header.frame_id and msg.header.frame_id != args.source_frame:
                frame_warnings += 1
            stamp = stamp_to_ns(msg.header.stamp) or bag_stamp
            odom_samples.append((stamp, pose_to_matrix(msg.pose.pose)))
        elif topic == args.tf_topic:
            msg = deserialize_message(data, tf_type)
            for transform in msg.transforms:
                parent = transform.header.frame_id
                child = transform.child_frame_id
                matrix = transform_to_matrix(transform.transform)
                if parent == args.target_frame and child == args.source_frame:
                    tf_matrix = matrix
                elif parent == args.source_frame and child == args.target_frame:
                    tf_matrix = OffsetTransformer._inverse_transform(matrix)
                else:
                    continue
                stamp = stamp_to_ns(transform.header.stamp) or bag_stamp
                tf_samples.append((stamp, tf_matrix))

    if frame_warnings:
        print(
            f'Warning: {frame_warnings} odometry messages had frame_id different '
            f'from source_frame "{args.source_frame}". child_frame_id was ignored.'
        )

    if not odom_samples:
        raise ValueError(f'No odometry samples found on {args.odom_topic}')
    if not tf_samples:
        raise ValueError(
            f'No usable {args.source_frame}<->{args.target_frame} transforms '
            f'found on {args.tf_topic}'
        )

    tf_samples.sort(key=lambda item: item[0])
    return odom_samples, tf_samples


def nearest_tf(stamp, tf_stamps, tf_matrices, max_gap_ns):
    """Return the nearest TF matrix for a stamp, or None when too far away."""
    index = bisect_left(tf_stamps, stamp)
    candidates = []
    if index < len(tf_stamps):
        candidates.append(index)
    if index > 0:
        candidates.append(index - 1)
    if not candidates:
        return None

    best = min(candidates, key=lambda item: abs(tf_stamps[item] - stamp))
    if abs(tf_stamps[best] - stamp) > max_gap_ns:
        return None
    return tf_matrices[best]


def build_map_sensor_samples(odom_samples, tf_samples, max_tf_gap):
    """Combine odometry and TF samples into T_map_sensor matrices."""
    tf_stamps = [stamp for stamp, _matrix in tf_samples]
    tf_matrices = [matrix for _stamp, matrix in tf_samples]
    max_gap_ns = int(max_tf_gap * 1000000000)
    map_sensor = []

    for stamp, t_odom_sensor in odom_samples:
        t_map_odom = nearest_tf(stamp, tf_stamps, tf_matrices, max_gap_ns)
        if t_map_odom is None:
            continue
        map_sensor.append(t_map_odom @ t_odom_sensor)

    if not map_sensor:
        raise ValueError(
            'No odometry samples could be matched with TF. '
            'Try increasing --max-tf-gap.'
        )

    return np.stack(map_sensor)


def calibrate_x(t_map_sensor_samples, sensor_offset, odom_orientation_frame='planar'):
    """Estimate sensor_offset.x by minimizing robot-center position variance."""
    if odom_orientation_frame not in ('planar', 'sensor', 'base'):
        raise ValueError('odom_orientation_frame must be "planar", "sensor", or "base"')

    sensor_offset = np.array(sensor_offset, dtype=float)
    fixed_offset = sensor_offset.copy()
    fixed_offset[0] = 0.0

    t_base_sensor_fixed = OffsetTransformer._build_transform(fixed_offset)
    fixed_translation = t_base_sensor_fixed[:3, 3]
    x_axis = np.array([1.0, 0.0, 0.0])
    if odom_orientation_frame in ('planar', 'sensor'):
        r_sensor_base = t_base_sensor_fixed[:3, :3].T
        fixed_rotation = r_sensor_base
    else:
        fixed_rotation = np.eye(3)

    q_values = []
    v_values = []
    raw_positions = []
    yaws = []

    for t_map_sensor in t_map_sensor_samples:
        r_map_sensor = t_map_sensor[:3, :3]
        p_map_sensor = t_map_sensor[:3, 3]
        q_values.append(p_map_sensor - r_map_sensor @ fixed_rotation @ fixed_translation)
        v_values.append(r_map_sensor @ fixed_rotation @ x_axis)
        raw_positions.append(p_map_sensor)
        yaws.append(R.from_matrix(r_map_sensor).as_euler('ZYX')[0])

    q_values = np.array(q_values)
    v_values = np.array(v_values)
    q_centered = q_values - q_values.mean(axis=0)
    v_centered = v_values - v_values.mean(axis=0)
    denominator = float(np.sum(v_centered * v_centered))
    if denominator < 1e-12:
        raise ValueError('Not enough rotation to estimate sensor_offset.x')

    x_estimate = float(np.sum(q_centered * v_centered) / denominator)
    corrected_positions = q_values - x_estimate * v_values
    residuals = corrected_positions - corrected_positions.mean(axis=0)
    residual_rmse = float(np.sqrt(np.mean(np.sum(residuals * residuals, axis=1))))

    raw_positions = np.array(raw_positions)
    raw_centered = raw_positions - raw_positions.mean(axis=0)
    raw_rmse = float(np.sqrt(np.mean(np.sum(raw_centered * raw_centered, axis=1))))
    yaw_span = float(np.ptp(np.unwrap(np.array(yaws))))

    return {
        'x': x_estimate,
        'samples': int(len(t_map_sensor_samples)),
        'yaw_span_rad': yaw_span,
        'yaw_span_deg': float(np.degrees(yaw_span)),
        'raw_position_rmse': raw_rmse,
        'corrected_position_rmse': residual_rmse,
    }


def write_calibrated_yaml(data, params, result, output_path):
    """Write a complete calibrated ROS 2 params file."""
    sensor_offset = list(params.get('sensor_offset', [0.0] * 6))
    if len(sensor_offset) != 6:
        raise ValueError('sensor_offset must contain 6 values')

    sensor_offset[0] = result['x']
    params['sensor_offset'] = [float(value) for value in sensor_offset]

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, 'w', encoding='utf-8') as stream:
        yaml.safe_dump(data, stream, sort_keys=False, allow_unicode=True)


def parse_args():
    """Parse command-line arguments."""
    package_root = default_package_root()
    default_params = package_root / 'config' / 'default.yaml'
    default_output = package_root / 'config' / 'calibrated.yaml'

    parser = argparse.ArgumentParser(
        description='Calibrate coordinate_transformer sensor_offset.x from a bag.'
    )
    parser.add_argument('bag_dir', type=Path, help='Path to a rosbag2 directory')
    parser.add_argument('--params-file', type=Path, default=default_params)
    parser.add_argument('--output', type=Path, default=default_output)
    parser.add_argument('--odom-topic', default='/odin1/odometry_highfreq')
    parser.add_argument('--tf-topic', default='/tf')
    parser.add_argument('--source-frame', default=None)
    parser.add_argument('--target-frame', default=None)
    parser.add_argument(
        '--odom-orientation-frame',
        choices=['planar', 'sensor', 'base'],
        default=None,
    )
    parser.add_argument('--max-tf-gap', type=float, default=0.2)
    return parser.parse_args()


def main():
    """Run x-offset calibration and write calibrated.yaml."""
    args = parse_args()
    data, params = read_template(args.params_file)
    args.source_frame = args.source_frame or params.get('source_frame', 'odom')
    args.target_frame = args.target_frame or params.get('target_frame', 'map')
    odom_orientation_frame = (
        args.odom_orientation_frame
        or params.get('odom_orientation_frame', 'planar')
    )

    sensor_offset = params.get('sensor_offset', [0.0] * 6)
    if len(sensor_offset) != 6:
        raise ValueError('sensor_offset must contain 6 values')

    odom_samples, tf_samples = read_bag_samples(args)
    t_map_sensor_samples = build_map_sensor_samples(
        odom_samples,
        tf_samples,
        args.max_tf_gap,
    )
    result = calibrate_x(
        t_map_sensor_samples,
        sensor_offset,
        odom_orientation_frame=odom_orientation_frame,
    )
    write_calibrated_yaml(data, params, result, args.output)

    print(f'Calibrated sensor_offset.x: {result["x"]:.6f} m')
    print(f'Samples used: {result["samples"]}')
    print(
        f'Yaw span: {result["yaw_span_rad"]:.3f} rad '
        f'({result["yaw_span_deg"]:.1f} deg)'
    )
    print(f'Raw position RMSE: {result["raw_position_rmse"]:.6f} m')
    print(
        'Corrected center RMSE: '
        f'{result["corrected_position_rmse"]:.6f} m'
    )
    print(f'Wrote calibrated params: {args.output}')


if __name__ == '__main__':
    main()
