#!/usr/bin/env python3
"""
偏移旋转仿真器（动态动画版）

用于模拟物体（传感器）不在旋转中心时的运动情况，
实时验证坐标偏移补偿逻辑的正确性。
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import argparse
from scipy.spatial.transform import Rotation as R

from transformer import OffsetTransformer, PoseTransformer


def simulate_rotation_animation(
    rotation_center_x=0.0,
    rotation_center_y=0.0,
    rotation_radius=1.0,
    num_steps=72,
    sensor_offset=(0.35, 0.0, 0.0, 0.0, 0.0, 0.0),
    map_origin_offset=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    interval=50,
):
    """
    动态仿真：机器人绕固定点旋转，实时显示偏移补偿效果。

    :param interval: 每帧间隔（毫秒），值越小动画越快
    """
    pose_transformer = PoseTransformer()
    offset_transformer = OffsetTransformer(sensor_offset, map_origin_offset)

    angles = np.linspace(0, 2 * np.pi, num_steps, endpoint=False)

    robot_center_world = []
    sensor_world = []
    compensated = []

    tf_odom_to_map = np.eye(4)
    T_world_to_map = offset_transformer._build_transform(map_origin_offset)

    for angle in angles:
        robot_x = rotation_center_x + rotation_radius * np.cos(angle)
        robot_y = rotation_center_y + rotation_radius * np.sin(angle)
        robot_yaw = angle + np.pi / 2

        offset_x, offset_y, offset_z = sensor_offset[0], sensor_offset[1], sensor_offset[2]
        cos_yaw, sin_yaw = np.cos(robot_yaw), np.sin(robot_yaw)

        sensor_x = robot_x + offset_x * cos_yaw - offset_y * sin_yaw
        sensor_y = robot_y + offset_x * sin_yaw + offset_y * cos_yaw
        sensor_z = offset_z

        offset_yaw = sensor_offset[5]
        total_yaw = robot_yaw + offset_yaw
        sensor_q = R.from_euler('z', total_yaw).as_quat()
        sqx, sqy, sqz, sqw = sensor_q

        robot_center_world.append([robot_x, robot_y, 0.0])
        sensor_world.append([sensor_x, sensor_y, sensor_z])

        result = offset_transformer.odom_to_map_with_offset(
            (sensor_x, sensor_y, sensor_z, sqx, sqy, sqz, sqw),
            tf_odom_to_map
        )
        compensated.append(result)

    robot_center_world = np.array(robot_center_world)
    sensor_world = np.array(sensor_world)
    compensated = np.array(compensated)

    ground_truth_in_map = offset_transformer.transform_point_cloud(
        robot_center_world, T_world_to_map
    )

    error_compensated = np.sqrt(
        (compensated[:, 0] - ground_truth_in_map[:, 0])**2 +
        (compensated[:, 1] - ground_truth_in_map[:, 1])**2
    )
    sensor_in_map = sensor_world.copy()
    error_naive = np.sqrt(
        (sensor_in_map[:, 0] - ground_truth_in_map[:, 0])**2 +
        (sensor_in_map[:, 1] - ground_truth_in_map[:, 1])**2
    )

    map_origin = map_origin_offset[:3]
    map_center_x = rotation_center_x + map_origin[0]
    map_center_y = rotation_center_y + map_origin[1]

    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    fig.suptitle(
        f'Rotation Simulation (radius={rotation_radius}m, sensor_offset=({sensor_offset[0]}, {sensor_offset[1]}m))',
        fontsize=13
    )

    # ---- 图1：2D 轨迹 ----
    ax1 = axes[0]
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_title('2D Trajectory')
    ax1.grid(True, alpha=0.3)

    # 预计算数据范围，防止 blit+equal 模式下 autoscaling 失效
    all_x = np.concatenate([robot_center_world[:, 0], sensor_world[:, 0], compensated[:, 0], ground_truth_in_map[:, 0]])
    all_y = np.concatenate([robot_center_world[:, 1], sensor_world[:, 1], compensated[:, 1], ground_truth_in_map[:, 1]])
    x_margin = (np.ptp(all_x) * 0.15) + 0.5
    y_margin = (np.ptp(all_y) * 0.15) + 0.5
    ax1.set_xlim(all_x.min() - x_margin, all_x.max() + x_margin)
    ax1.set_ylim(all_y.min() - y_margin, all_y.max() + y_margin)

    (line_robot_gt,) = ax1.plot([], [], 'b-', linewidth=2, alpha=0.8, label='Robot center (ground truth)')
    (line_sensor_gt,) = ax1.plot([], [], 'r--', linewidth=1.5, alpha=0.7, label='Sensor (ground truth, no compensation)')
    (line_comp,) = ax1.plot([], [], 'm-', linewidth=2, alpha=0.9, label='Compensated (output)')
    (line_gt_map,) = ax1.plot([], [], 'c:', linewidth=2, alpha=0.9, label='Ground truth (map)')

    ax1.scatter([], [], c='black', s=250, marker='+', linewidths=3, label='Rotation center')
    if map_origin[0] != 0 or map_origin[1] != 0:
        ax1.scatter([], [], c='green', s=250, marker='+', linewidths=3, label='Map origin offset')

    (sc_robot_curr,) = ax1.plot([], [], 'bo', ms=10, zorder=10, label='Robot current')
    (sc_sensor_curr,) = ax1.plot([], [], 'r^', ms=8, zorder=10, label='Sensor current')
    (sc_comp_curr,) = ax1.plot([], [], 'ms', ms=8, zorder=10, label='Compensated current')

    # 偏移向量箭头（固定在四分之一位置）
    arrow_idx = num_steps // 4
    arrow = ax1.annotate('',
        xy=(robot_center_world[arrow_idx, 0], robot_center_world[arrow_idx, 1]),
        xytext=(sensor_world[arrow_idx, 0], sensor_world[arrow_idx, 1]),
        arrowprops=dict(arrowstyle='->', color='orange', lw=2))
    ax1.text(0, 0, 'Sensor offset', color='orange', fontsize=9, ha='center')

    ax1.legend(loc='upper right', fontsize=8)

    # ---- 图2：半径分析 ----
    ax2 = axes[1]
    ax2.set_xlabel('Rotation angle (deg)')
    ax2.set_ylabel('Distance to rotation center (m)')
    ax2.set_title('Radius Analysis')
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim(-5, 365)
    ax2.set_ylim(0, max(rotation_radius * 1.3, abs(sensor_offset[0]) * 1.3) if sensor_offset[0] != 0 else rotation_radius * 1.3)

    dist_robot = np.sqrt(
        (robot_center_world[:, 0] - rotation_center_x)**2 +
        (robot_center_world[:, 1] - rotation_center_y)**2
    )
    dist_sensor = np.sqrt(
        (sensor_world[:, 0] - rotation_center_x)**2 +
        (sensor_world[:, 1] - rotation_center_y)**2
    )
    dist_compensated = np.sqrt(
        (compensated[:, 0] - map_center_x)**2 +
        (compensated[:, 1] - map_center_y)**2
    )

    ax2.plot(np.degrees(angles), dist_robot, 'b-', linewidth=2, label='Robot center (truth)')
    ax2.plot(np.degrees(angles), dist_sensor, 'r--', linewidth=1.5, alpha=0.7, label='Sensor (truth)')
    ax2.plot(np.degrees(angles), dist_compensated, 'm-', linewidth=2, alpha=0.9, label='Compensated (output)')
    ax2.legend(fontsize=9)

    (vline,) = ax2.plot([], [], 'k-', linewidth=2)
    (dot_r,) = ax2.plot([], [], 'bo', ms=8)
    (dot_s,) = ax2.plot([], [], 'r^', ms=7)
    (dot_c,) = ax2.plot([], [], 'ms', ms=7)

    # ---- 图3：误差分析 ----
    ax3 = axes[2]
    ax3.set_xlabel('Rotation angle (deg)')
    ax3.set_ylabel('Position error (cm)')
    ax3.set_title('Position Error vs Ground Truth')
    ax3.grid(True, alpha=0.3)
    ax3.set_xlim(-5, 365)

    max_err = max(np.max(error_naive), np.max(error_compensated)) * 100
    ax3.set_ylim(-max_err * 0.05, max_err * 1.15)

    ax3.plot(np.degrees(angles), error_naive * 100, 'r-', linewidth=2,
             label=f'Without compensation (max={np.max(error_naive)*100:.2f}cm)')
    ax3.plot(np.degrees(angles), error_compensated * 100, 'm-', linewidth=2,
             label=f'With compensation (max={np.max(error_compensated)*100:.2f}cm)')
    ax3.axhline(y=0, color='black', linestyle='-', linewidth=0.5)
    ax3.legend(fontsize=9)

    (vline3,) = ax3.plot([], [], 'k-', linewidth=2)
    (dot_naive3,) = ax3.plot([], [], 'ro', ms=8)
    (dot_comp3,) = ax3.plot([], [], 'ms', ms=8)

    # ---- 文字信息面板 ----
    info_text = ax1.text(0.02, 0.02, '', transform=ax1.transAxes,
                         fontsize=9, verticalalignment='bottom',
                         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

    def init():
        line_robot_gt.set_data([], [])
        line_sensor_gt.set_data([], [])
        line_comp.set_data([], [])
        line_gt_map.set_data([], [])
        sc_robot_curr.set_data([], [])
        sc_sensor_curr.set_data([], [])
        sc_comp_curr.set_data([], [])
        vline.set_data([], [])
        vline3.set_data([], [])
        info_text.set_text('')
        return [
            line_robot_gt, line_sensor_gt, line_comp, line_gt_map,
            sc_robot_curr, sc_sensor_curr, sc_comp_curr,
            vline, vline3, info_text
        ]

    def animate(i):
        i_wrap = i % num_steps  # 循环播放

        robot_trail = robot_center_world[:i_wrap + 1]
        sensor_trail = sensor_world[:i_wrap + 1]
        comp_trail = compensated[:i_wrap + 1]

        line_robot_gt.set_data(robot_center_world[:i_wrap + 1, 0],
                               robot_center_world[:i_wrap + 1, 1])
        line_sensor_gt.set_data(sensor_world[:i_wrap + 1, 0],
                                sensor_world[:i_wrap + 1, 1])
        line_comp.set_data(compensated[:i_wrap + 1, 0],
                           compensated[:i_wrap + 1, 1])
        line_gt_map.set_data(ground_truth_in_map[:i_wrap + 1, 0],
                             ground_truth_in_map[:i_wrap + 1, 1])

        sc_robot_curr.set_data([robot_center_world[i_wrap, 0]],
                               [robot_center_world[i_wrap, 1]])
        sc_sensor_curr.set_data([sensor_world[i_wrap, 0]],
                               [sensor_world[i_wrap, 1]])
        sc_comp_curr.set_data([compensated[i_wrap, 0]],
                              [compensated[i_wrap, 1]])

        vline.set_data([np.degrees(angles[i_wrap]), np.degrees(angles[i_wrap])], [0, 1])
        dot_r.set_data([np.degrees(angles[i_wrap])], [dist_robot[i_wrap]])
        dot_s.set_data([np.degrees(angles[i_wrap])], [dist_sensor[i_wrap]])
        dot_c.set_data([np.degrees(angles[i_wrap])], [dist_compensated[i_wrap]])

        vline3.set_data([np.degrees(angles[i_wrap]), np.degrees(angles[i_wrap])], [0, max_err * 1.1])
        dot_naive3.set_data([np.degrees(angles[i_wrap])], [error_naive[i_wrap] * 100])
        dot_comp3.set_data([np.degrees(angles[i_wrap])], [error_compensated[i_wrap] * 100])

        info_text.set_text(
            f'Angle: {np.degrees(angles[i_wrap]):.1f}°\n'
            f'Comp error: {error_compensated[i_wrap]*100:.3f} cm\n'
            f'Naive error: {error_naive[i_wrap]*100:.3f} cm'
        )

        return [
            line_robot_gt, line_sensor_gt, line_comp, line_gt_map,
            sc_robot_curr, sc_sensor_curr, sc_comp_curr,
            vline, dot_r, dot_s, dot_c,
            vline3, dot_naive3, dot_comp3,
            info_text
        ]

    ani = animation.FuncAnimation(
        fig, animate,
        init_func=init,
        frames=num_steps,
        interval=interval,
        blit=True
    )

    plt.tight_layout()
    plt.show()

    # ---- 运行结束后打印统计 ----
    print("\n" + "="*65)
    print("SIMULATION SUMMARY")
    print("="*65)
    print(f"Rotation center:        ({rotation_center_x}, {rotation_center_y}) m")
    print(f"Rotation radius:        {rotation_radius} m")
    print(f"Sensor offset:           (x={sensor_offset[0]}, y={sensor_offset[1]}) m")
    print(f"Map origin offset:      {map_origin_offset}")
    print(f"Number of samples:       {num_steps}")
    print()
    print(f"{'Metric':<30} {'Naive (no offset)':<20} {'Compensated':<20}")
    print("-"*65)
    print(f"{'Max error (cm)':<30} {np.max(error_naive)*100:>18.2f}   {np.max(error_compensated)*100:>18.2f}")
    print(f"{'Mean error (cm)':<30} {np.mean(error_naive)*100:>18.2f}   {np.mean(error_compensated)*100:>18.2f}")
    print(f"{'Std error (cm)':<30} {np.std(error_naive)*100:>18.2f}   {np.std(error_compensated)*100:>18.2f}")
    print("="*65)

    improvement = (np.mean(error_naive) - np.mean(error_compensated)) / np.mean(error_naive) * 100
    if improvement > 0:
        print(f"Compensation IMPROVES accuracy by {improvement:.1f}%")
    else:
        print(f"Compensation worsens accuracy by {-improvement:.1f}%")
    print("="*65)


def main():
    parser = argparse.ArgumentParser(
        description='Offset Rotation Simulator (Animated) - Validate sensor offset compensation'
    )
    parser.add_argument('--center-x', type=float, default=0.0,
                        help='Rotation center X (default: 0)')
    parser.add_argument('--center-y', type=float, default=0.0,
                        help='Rotation center Y (default: 0)')
    parser.add_argument('--radius', type=float, default=1.0,
                        help='Rotation radius in meters (default: 1.0)')
    parser.add_argument('--offset-x', type=float, default=0.35,
                        help='Sensor offset X from robot center (default: 0.35)')
    parser.add_argument('--offset-y', type=float, default=0.0,
                        help='Sensor offset Y from robot center (default: 0.0)')
    parser.add_argument('--offset-z', type=float, default=0.0,
                        help='Sensor offset Z from robot center (default: 0.0)')
    parser.add_argument('--yaw', type=float, default=0.0,
                        help='Sensor yaw offset from robot center (rad, default: 0)')
    parser.add_argument('--steps', type=int, default=72,
                        help='Number of sampling points (default: 72)')
    parser.add_argument('--map-offset', type=float, nargs=3, default=[0, 0, 0],
                        metavar=('X', 'Y', 'Z'),
                        help='Map origin offset xyz (default: 0 0 0)')
    parser.add_argument('--interval', type=int, default=50,
                        help='Animation frame interval in ms (default: 50, smaller=faster)')

    args = parser.parse_args()

    sensor_offset = (args.offset_x, args.offset_y, args.offset_z,
                     0.0, 0.0, args.yaw)
    map_origin_offset = (*args.map_offset, 0.0, 0.0, 0.0)

    simulate_rotation_animation(
        rotation_center_x=args.center_x,
        rotation_center_y=args.center_y,
        rotation_radius=args.radius,
        num_steps=args.steps,
        sensor_offset=sensor_offset,
        map_origin_offset=map_origin_offset,
        interval=args.interval,
    )


if __name__ == '__main__':
    main()
