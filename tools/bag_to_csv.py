#!/usr/bin/env python3
"""Export pose/TF/IMU data from a ROS 2 bag to CSV.

This script is intentionally independent from colcon entry points so it can be
run directly after sourcing the ROS 2 environment:

    python3 tools/bag_to_csv.py path/to/rosbag2_dir

It also recomputes the configured odom -> map pose using the repository's
coordinate_transformer implementation, which is useful for checking whether a
published relocation topic matches the algorithm.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import math
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation as R

try:
    import yaml
except ImportError:  # pragma: no cover - ROS installs usually include PyYAML.
    yaml = None


POSE_FIELDS = [
    "topic",
    "msg_type",
    "bag_time",
    "header_time",
    "frame_id",
    "child_frame_id",
    "x",
    "y",
    "z",
    "qx",
    "qy",
    "qz",
    "qw",
    "roll",
    "pitch",
    "yaw",
    "linear_x",
    "linear_y",
    "linear_z",
    "angular_x",
    "angular_y",
    "angular_z",
]

TF_FIELDS = [
    "topic",
    "bag_time",
    "header_time",
    "parent_frame",
    "child_frame",
    "x",
    "y",
    "z",
    "qx",
    "qy",
    "qz",
    "qw",
    "roll",
    "pitch",
    "yaw",
]

IMU_FIELDS = [
    "topic",
    "bag_time",
    "header_time",
    "frame_id",
    "qx",
    "qy",
    "qz",
    "qw",
    "roll",
    "pitch",
    "yaw",
    "angular_velocity_x",
    "angular_velocity_y",
    "angular_velocity_z",
    "linear_acceleration_x",
    "linear_acceleration_y",
    "linear_acceleration_z",
]

RECOMPUTED_FIELDS = [
    "input_topic",
    "bag_time",
    "header_time",
    "input_frame_id",
    "input_child_frame_id",
    "input_x",
    "input_y",
    "input_z",
    "input_qx",
    "input_qy",
    "input_qz",
    "input_qw",
    "input_roll",
    "input_pitch",
    "input_yaw",
    "tf_time",
    "tf_parent_frame",
    "tf_child_frame",
    "tf_x",
    "tf_y",
    "tf_z",
    "tf_qx",
    "tf_qy",
    "tf_qz",
    "tf_qw",
    "tf_roll",
    "tf_pitch",
    "tf_yaw",
    "recomputed_x",
    "recomputed_y",
    "recomputed_z",
    "recomputed_qx",
    "recomputed_qy",
    "recomputed_qz",
    "recomputed_qw",
    "recomputed_roll",
    "recomputed_pitch",
    "recomputed_yaw",
    "compare_topic",
    "compare_bag_time",
    "compare_header_time",
    "compare_x",
    "compare_y",
    "compare_z",
    "compare_qx",
    "compare_qy",
    "compare_qz",
    "compare_qw",
    "compare_roll",
    "compare_pitch",
    "compare_yaw",
    "compare_minus_recomputed_x",
    "compare_minus_recomputed_y",
    "compare_minus_recomputed_z",
    "compare_minus_recomputed_roll",
    "compare_minus_recomputed_pitch",
    "compare_minus_recomputed_yaw",
    "compare_minus_recomputed_xy",
    "compare_minus_recomputed_xyz",
]


PoseTuple = Tuple[float, float, float, float, float, float, float]


def stamp_to_sec(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def bag_time_to_sec(timestamp_ns: int) -> float:
    return float(timestamp_ns) * 1e-9


def normalize_frame(frame: str) -> str:
    return frame[1:] if frame.startswith("/") else frame


def quat_to_rpy(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-12:
        return float("nan"), float("nan"), float("nan")
    q = [qx / norm, qy / norm, qz / norm, qw / norm]
    roll, pitch, yaw = R.from_quat(q).as_euler("xyz", degrees=False)
    return float(roll), float(pitch), float(yaw)


def angle_diff(a: float, b: float) -> float:
    if math.isnan(a) or math.isnan(b):
        return float("nan")
    return math.atan2(math.sin(a - b), math.cos(a - b))


def pose_to_matrix(pose: PoseTuple) -> np.ndarray:
    x, y, z, qx, qy, qz, qw = pose
    matrix = np.eye(4)
    matrix[:3, :3] = R.from_quat([qx, qy, qz, qw]).as_matrix()
    matrix[:3, 3] = [x, y, z]
    return matrix


def inverse_transform(matrix: np.ndarray) -> np.ndarray:
    inv = np.eye(4)
    inv[:3, :3] = matrix[:3, :3].T
    inv[:3, 3] = -matrix[:3, :3].T @ matrix[:3, 3]
    return inv


def get_repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def default_config_path(repo_root: Path) -> Path:
    return repo_root / "src/coordinate_transformer/coordinate_transformer/config/default.yaml"


def load_config(path: Optional[Path]) -> Dict[str, Any]:
    if path is None or not path.exists() or yaml is None:
        return {}
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    return data.get("coordinate_transformer", {}).get("ros__parameters", {})


def load_offset_transformer(repo_root: Path):
    package_source = repo_root / "src/coordinate_transformer/coordinate_transformer"
    sys.path.insert(0, str(package_source))
    from coordinate_transformer.transformer import OffsetTransformer  # pylint: disable=import-error

    return OffsetTransformer


def open_bag_reader(bag_dir: Path) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)
    return reader


def pose_tuple_from_odometry(msg: Any) -> PoseTuple:
    pose = msg.pose.pose
    return (
        float(pose.position.x),
        float(pose.position.y),
        float(pose.position.z),
        float(pose.orientation.x),
        float(pose.orientation.y),
        float(pose.orientation.z),
        float(pose.orientation.w),
    )


def pose_tuple_from_pose_stamped(msg: Any) -> PoseTuple:
    pose = msg.pose
    return (
        float(pose.position.x),
        float(pose.position.y),
        float(pose.position.z),
        float(pose.orientation.x),
        float(pose.orientation.y),
        float(pose.orientation.z),
        float(pose.orientation.w),
    )


def pose_tuple_from_pose_with_covariance_stamped(msg: Any) -> PoseTuple:
    pose = msg.pose.pose
    return (
        float(pose.position.x),
        float(pose.position.y),
        float(pose.position.z),
        float(pose.orientation.x),
        float(pose.orientation.y),
        float(pose.orientation.z),
        float(pose.orientation.w),
    )


def pose_tuple_from_transform(transform: Any) -> PoseTuple:
    t = transform.transform.translation
    q = transform.transform.rotation
    return (
        float(t.x),
        float(t.y),
        float(t.z),
        float(q.x),
        float(q.y),
        float(q.z),
        float(q.w),
    )


def pose_values(prefix: str, pose: PoseTuple) -> Dict[str, float]:
    x, y, z, qx, qy, qz, qw = pose
    roll, pitch, yaw = quat_to_rpy(qx, qy, qz, qw)
    return {
        f"{prefix}x": x,
        f"{prefix}y": y,
        f"{prefix}z": z,
        f"{prefix}qx": qx,
        f"{prefix}qy": qy,
        f"{prefix}qz": qz,
        f"{prefix}qw": qw,
        f"{prefix}roll": roll,
        f"{prefix}pitch": pitch,
        f"{prefix}yaw": yaw,
    }


def pose_row(topic: str, msg_type: str, bag_time: float, msg: Any) -> Optional[Dict[str, Any]]:
    if msg_type == "nav_msgs/msg/Odometry":
        pose = pose_tuple_from_odometry(msg)
        twist = msg.twist.twist
        row = {
            "topic": topic,
            "msg_type": msg_type,
            "bag_time": bag_time,
            "header_time": stamp_to_sec(msg.header.stamp),
            "frame_id": msg.header.frame_id,
            "child_frame_id": msg.child_frame_id,
            "linear_x": float(twist.linear.x),
            "linear_y": float(twist.linear.y),
            "linear_z": float(twist.linear.z),
            "angular_x": float(twist.angular.x),
            "angular_y": float(twist.angular.y),
            "angular_z": float(twist.angular.z),
        }
    elif msg_type == "geometry_msgs/msg/PoseStamped":
        pose = pose_tuple_from_pose_stamped(msg)
        row = {
            "topic": topic,
            "msg_type": msg_type,
            "bag_time": bag_time,
            "header_time": stamp_to_sec(msg.header.stamp),
            "frame_id": msg.header.frame_id,
            "child_frame_id": "",
            "linear_x": "",
            "linear_y": "",
            "linear_z": "",
            "angular_x": "",
            "angular_y": "",
            "angular_z": "",
        }
    elif msg_type == "geometry_msgs/msg/PoseWithCovarianceStamped":
        pose = pose_tuple_from_pose_with_covariance_stamped(msg)
        row = {
            "topic": topic,
            "msg_type": msg_type,
            "bag_time": bag_time,
            "header_time": stamp_to_sec(msg.header.stamp),
            "frame_id": msg.header.frame_id,
            "child_frame_id": "",
            "linear_x": "",
            "linear_y": "",
            "linear_z": "",
            "angular_x": "",
            "angular_y": "",
            "angular_z": "",
        }
    else:
        return None

    row.update(pose_values("", pose))
    return row


def tf_rows(topic: str, bag_time: float, msg: Any) -> List[Dict[str, Any]]:
    rows = []
    for transform in msg.transforms:
        pose = pose_tuple_from_transform(transform)
        row = {
            "topic": topic,
            "bag_time": bag_time,
            "header_time": stamp_to_sec(transform.header.stamp),
            "parent_frame": transform.header.frame_id,
            "child_frame": transform.child_frame_id,
        }
        row.update(pose_values("", pose))
        rows.append(row)
    return rows


def imu_row(topic: str, bag_time: float, msg: Any) -> Dict[str, Any]:
    q = msg.orientation
    roll, pitch, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)
    return {
        "topic": topic,
        "bag_time": bag_time,
        "header_time": stamp_to_sec(msg.header.stamp),
        "frame_id": msg.header.frame_id,
        "qx": float(q.x),
        "qy": float(q.y),
        "qz": float(q.z),
        "qw": float(q.w),
        "roll": roll,
        "pitch": pitch,
        "yaw": yaw,
        "angular_velocity_x": float(msg.angular_velocity.x),
        "angular_velocity_y": float(msg.angular_velocity.y),
        "angular_velocity_z": float(msg.angular_velocity.z),
        "linear_acceleration_x": float(msg.linear_acceleration.x),
        "linear_acceleration_y": float(msg.linear_acceleration.y),
        "linear_acceleration_z": float(msg.linear_acceleration.z),
    }


def find_direct_tf(
    msg: Any,
    target_frame: str,
    source_frame: str,
) -> Optional[Dict[str, Any]]:
    target = normalize_frame(target_frame)
    source = normalize_frame(source_frame)

    for transform in msg.transforms:
        parent = normalize_frame(transform.header.frame_id)
        child = normalize_frame(transform.child_frame_id)
        pose = pose_tuple_from_transform(transform)
        matrix = pose_to_matrix(pose)

        if parent == target and child == source:
            return {
                "time": stamp_to_sec(transform.header.stamp),
                "parent_frame": transform.header.frame_id,
                "child_frame": transform.child_frame_id,
                "pose": pose,
                "matrix": matrix,
            }
        if parent == source and child == target:
            inv = inverse_transform(matrix)
            inv_pose = matrix_to_pose(inv)
            return {
                "time": stamp_to_sec(transform.header.stamp),
                "parent_frame": target_frame,
                "child_frame": source_frame,
                "pose": inv_pose,
                "matrix": inv,
            }
    return None


def matrix_to_pose(matrix: np.ndarray) -> PoseTuple:
    x, y, z = matrix[:3, 3]
    qx, qy, qz, qw = R.from_matrix(matrix[:3, :3]).as_quat()
    return float(x), float(y), float(z), float(qx), float(qy), float(qz), float(qw)


def get_header_time(msg: Any) -> float:
    return stamp_to_sec(msg.header.stamp)


def parse_six_floats(values: Optional[Sequence[str]], fallback: Sequence[float]) -> List[float]:
    if values is None:
        return [float(v) for v in fallback]
    if len(values) != 6:
        raise argparse.ArgumentTypeError("Expected exactly 6 numbers.")
    return [float(v) for v in values]


def should_keep(counter: int, sample_every: int) -> bool:
    return sample_every <= 1 or (counter - 1) % sample_every == 0


def nearest_compare(
    compare_times: Sequence[float],
    compare_records: Sequence[Dict[str, Any]],
    bag_time: float,
    max_dt: float,
) -> Optional[Dict[str, Any]]:
    if not compare_records:
        return None

    idx = bisect.bisect_left(compare_times, bag_time)
    candidates = []
    if idx < len(compare_records):
        candidates.append(compare_records[idx])
    if idx > 0:
        candidates.append(compare_records[idx - 1])

    best = min(candidates, key=lambda record: abs(record["bag_time"] - bag_time))
    if abs(best["bag_time"] - bag_time) <= max_dt:
        return best
    return None


def add_compare_columns(
    record: Dict[str, Any],
    compare: Optional[Dict[str, Any]],
) -> Dict[str, Any]:
    if compare is None:
        for field in RECOMPUTED_FIELDS:
            record.setdefault(field, "")
        return record

    compare_pose = (
        compare["x"],
        compare["y"],
        compare["z"],
        compare["qx"],
        compare["qy"],
        compare["qz"],
        compare["qw"],
    )
    record["compare_topic"] = compare["topic"]
    record["compare_bag_time"] = compare["bag_time"]
    record["compare_header_time"] = compare["header_time"]
    record.update(pose_values("compare_", compare_pose))

    dx = compare["x"] - record["recomputed_x"]
    dy = compare["y"] - record["recomputed_y"]
    dz = compare["z"] - record["recomputed_z"]
    droll = angle_diff(compare["roll"], record["recomputed_roll"])
    dpitch = angle_diff(compare["pitch"], record["recomputed_pitch"])
    dyaw = angle_diff(compare["yaw"], record["recomputed_yaw"])
    record["compare_minus_recomputed_x"] = dx
    record["compare_minus_recomputed_y"] = dy
    record["compare_minus_recomputed_z"] = dz
    record["compare_minus_recomputed_roll"] = droll
    record["compare_minus_recomputed_pitch"] = dpitch
    record["compare_minus_recomputed_yaw"] = dyaw
    record["compare_minus_recomputed_xy"] = math.hypot(dx, dy)
    record["compare_minus_recomputed_xyz"] = math.sqrt(dx * dx + dy * dy + dz * dz)
    return record


def write_csv(path: Path, fieldnames: Sequence[str], rows: Iterable[Dict[str, Any]]) -> int:
    count = 0
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
            count += 1
    return count


def write_summary(path: Path, lines: Sequence[str]) -> None:
    with path.open("w", encoding="utf-8") as f:
        f.write("\n".join(lines))
        f.write("\n")


def existing_topics(topic_types: Dict[str, str], candidates: Iterable[str]) -> List[str]:
    return [topic for topic in candidates if topic in topic_types]


def main() -> int:
    repo_root = get_repo_root()
    config_default = default_config_path(repo_root)

    parser = argparse.ArgumentParser(
        description="Export odin ROS 2 bag pose, TF, and IMU data to CSV."
    )
    parser.add_argument("bag", type=Path, help="Path to a rosbag2 directory.")
    parser.add_argument(
        "-o",
        "--output-dir",
        type=Path,
        default=None,
        help="CSV output directory. Defaults to <bag>/csv_export.",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=config_default if config_default.exists() else None,
        help="coordinate_transformer YAML config.",
    )
    parser.add_argument(
        "--pose-topics",
        nargs="*",
        default=None,
        help="Pose-like topics to export. Defaults to all Odometry/PoseStamped topics.",
    )
    parser.add_argument(
        "--odom-topic",
        default=None,
        help="Odometry topic used for offline recomputation.",
    )
    parser.add_argument(
        "--compare-topic",
        action="append",
        default=None,
        help="Published output pose topic to compare against. May be repeated.",
    )
    parser.add_argument(
        "--tf-topics",
        nargs="*",
        default=["/tf", "/tf_static"],
        help="TF topics to export and use.",
    )
    parser.add_argument(
        "--imu-topics",
        nargs="*",
        default=None,
        help="IMU topics to export. Defaults to all sensor_msgs/msg/Imu topics.",
    )
    parser.add_argument(
        "--source-frame",
        default=None,
        help="Source frame for TF lookup, usually odom.",
    )
    parser.add_argument(
        "--target-frame",
        default=None,
        help="Target frame for TF lookup, usually map.",
    )
    parser.add_argument(
        "--sensor-offset",
        nargs=6,
        type=float,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        default=None,
        help="Override sensor offset from config.",
    )
    parser.add_argument(
        "--map-origin-offset",
        nargs=6,
        type=float,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        default=None,
        help="Override map origin offset from config.",
    )
    parser.add_argument(
        "--compare-window",
        type=float,
        default=0.05,
        help="Max time difference in seconds for nearest output-pose comparison.",
    )
    parser.add_argument(
        "--sample-every",
        type=int,
        default=1,
        help="Keep every Nth pose/IMU input message. TF rows are always kept.",
    )
    parser.add_argument(
        "--no-recompute",
        action="store_true",
        help="Only export raw CSV files; skip offline coordinate recomputation.",
    )
    args = parser.parse_args()

    bag_dir = args.bag.expanduser().resolve()
    if not bag_dir.exists():
        parser.error(f"Bag path does not exist: {bag_dir}")
    if args.sample_every < 1:
        parser.error("--sample-every must be >= 1")

    config = load_config(args.config.expanduser().resolve() if args.config else None)
    sensor_offset = parse_six_floats(
        args.sensor_offset,
        config.get("sensor_offset", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    )
    map_origin_offset = parse_six_floats(
        args.map_origin_offset,
        config.get("map_origin_offset", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    )
    source_frame = args.source_frame or config.get("source_frame", "odom")
    target_frame = args.target_frame or config.get("target_frame", "map")
    odom_topic = args.odom_topic or config.get("odin_pose_topic", "/odin1/odometry_highfreq")

    reader = open_bag_reader(bag_dir)
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    msg_type_cache = {topic: get_message(msg_type) for topic, msg_type in topic_types.items()}

    pose_types = {
        "nav_msgs/msg/Odometry",
        "geometry_msgs/msg/PoseStamped",
        "geometry_msgs/msg/PoseWithCovarianceStamped",
    }
    pose_topics = (
        args.pose_topics
        if args.pose_topics is not None
        else [topic for topic, msg_type in topic_types.items() if msg_type in pose_types]
    )
    imu_topics = (
        args.imu_topics
        if args.imu_topics is not None
        else [topic for topic, msg_type in topic_types.items() if msg_type == "sensor_msgs/msg/Imu"]
    )

    compare_candidates = []
    if args.compare_topic:
        compare_candidates.extend(args.compare_topic)
    else:
        configured_output = config.get("output_pose_topic")
        if configured_output:
            compare_candidates.append(configured_output)
        compare_candidates.extend(["/odin1/relocation", "/odin1/relocation_1", "/transformed/pose"])
    compare_topics = existing_topics(topic_types, compare_candidates)

    output_dir = args.output_dir or (bag_dir / "csv_export")
    output_dir.mkdir(parents=True, exist_ok=True)

    OffsetTransformer = None
    transformer = None
    if not args.no_recompute:
        try:
            OffsetTransformer = load_offset_transformer(repo_root)
            transformer = OffsetTransformer(sensor_offset, map_origin_offset)
        except Exception as exc:  # pragma: no cover - useful on machines without package deps.
            print(f"Warning: offline recomputation disabled: {exc}", file=sys.stderr)

    pose_records: List[Dict[str, Any]] = []
    tf_records: List[Dict[str, Any]] = []
    imu_records: List[Dict[str, Any]] = []
    recomputed_records: List[Dict[str, Any]] = []
    compare_records: List[Dict[str, Any]] = []
    raw_topic_counts: Dict[str, int] = {}
    kept_topic_counts: Dict[str, int] = {}
    latest_tf: Optional[Dict[str, Any]] = None

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        raw_topic_counts[topic] = raw_topic_counts.get(topic, 0) + 1
        msg_type = topic_types[topic]
        bag_time = bag_time_to_sec(timestamp_ns)

        if topic not in pose_topics and topic not in args.tf_topics and topic not in imu_topics:
            if topic not in compare_topics and topic != odom_topic:
                continue

        msg = deserialize_message(data, msg_type_cache[topic])

        if topic in args.tf_topics and msg_type == "tf2_msgs/msg/TFMessage":
            rows = tf_rows(topic, bag_time, msg)
            tf_records.extend(rows)
            kept_topic_counts[topic] = kept_topic_counts.get(topic, 0) + len(rows)
            direct_tf = find_direct_tf(msg, target_frame, source_frame)
            if direct_tf is not None:
                latest_tf = direct_tf
            continue

        keep_sample = should_keep(raw_topic_counts[topic], args.sample_every)

        if topic in pose_topics and msg_type in pose_types and keep_sample:
            row = pose_row(topic, msg_type, bag_time, msg)
            if row is not None:
                pose_records.append(row)
                kept_topic_counts[topic] = kept_topic_counts.get(topic, 0) + 1
                if topic in compare_topics:
                    compare_records.append(row)

        if topic in compare_topics and topic not in pose_topics and msg_type in pose_types:
            row = pose_row(topic, msg_type, bag_time, msg)
            if row is not None:
                compare_records.append(row)

        if topic in imu_topics and msg_type == "sensor_msgs/msg/Imu" and keep_sample:
            imu_records.append(imu_row(topic, bag_time, msg))
            kept_topic_counts[topic] = kept_topic_counts.get(topic, 0) + 1

        if (
            transformer is not None
            and topic == odom_topic
            and msg_type == "nav_msgs/msg/Odometry"
            and latest_tf is not None
            and keep_sample
        ):
            input_pose = pose_tuple_from_odometry(msg)
            recomputed_pose = transformer.odom_to_map_with_offset(input_pose, latest_tf["matrix"])
            record = {
                "input_topic": topic,
                "bag_time": bag_time,
                "header_time": get_header_time(msg),
                "input_frame_id": msg.header.frame_id,
                "input_child_frame_id": msg.child_frame_id,
                "tf_time": latest_tf["time"],
                "tf_parent_frame": latest_tf["parent_frame"],
                "tf_child_frame": latest_tf["child_frame"],
            }
            record.update(pose_values("input_", input_pose))
            record.update(pose_values("tf_", latest_tf["pose"]))
            record.update(pose_values("recomputed_", recomputed_pose))
            recomputed_records.append(record)

    compare_records.sort(key=lambda row: row["bag_time"])
    compare_times = [row["bag_time"] for row in compare_records]
    recomputed_records = [
        add_compare_columns(
            record,
            nearest_compare(compare_times, compare_records, record["bag_time"], args.compare_window),
        )
        for record in recomputed_records
    ]

    topic_rows = [
        {"topic": topic, "msg_type": msg_type}
        for topic, msg_type in sorted(topic_types.items(), key=lambda item: item[0])
    ]

    counts = {
        "topics.csv": write_csv(output_dir / "topics.csv", ["topic", "msg_type"], topic_rows),
        "poses.csv": write_csv(output_dir / "poses.csv", POSE_FIELDS, pose_records),
        "tf.csv": write_csv(output_dir / "tf.csv", TF_FIELDS, tf_records),
        "imu.csv": write_csv(output_dir / "imu.csv", IMU_FIELDS, imu_records),
        "recomputed_pose.csv": write_csv(
            output_dir / "recomputed_pose.csv",
            RECOMPUTED_FIELDS,
            recomputed_records,
        ),
    }

    summary_lines = [
        f"bag: {bag_dir}",
        f"output_dir: {output_dir}",
        f"config: {args.config if args.config else ''}",
        f"sensor_offset: {sensor_offset}",
        f"map_origin_offset: {map_origin_offset}",
        f"recompute_input_topic: {odom_topic}",
        f"tf_lookup: {target_frame} <- {source_frame}",
        f"compare_topics: {compare_topics}",
        f"compare_window_sec: {args.compare_window}",
        f"sample_every: {args.sample_every}",
        "",
        "rows_written:",
    ]
    summary_lines.extend(f"  {name}: {count}" for name, count in counts.items())
    summary_lines.append("")
    summary_lines.append("kept_topic_counts:")
    summary_lines.extend(
        f"  {topic}: {count}" for topic, count in sorted(kept_topic_counts.items())
    )
    summary_lines.append("")
    summary_lines.append("raw_topic_counts_seen:")
    summary_lines.extend(
        f"  {topic}: {count}" for topic, count in sorted(raw_topic_counts.items())
    )
    write_summary(output_dir / "summary.txt", summary_lines)

    for line in summary_lines[:14]:
        print(line)
    print(f"Wrote CSV files to: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
