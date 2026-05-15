import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    odin_driver_pkg = get_package_share_directory('odin_ros_driver')
    transformer_pkg = get_package_share_directory('coordinate_transformer')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(odin_driver_pkg, 'config', 'control_command.yaml'),
        description='Path to the control config YAML file'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(odin_driver_pkg, 'config', 'odin_ros2.rviz'),
        description='Path to RViz2 config file'
    )

    host_sdk_node = Node(
        package='odin_ros_driver',
        executable='host_sdk_sample',
        name='host_sdk_sample',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file')
        }]
    )

    pcd2depth_config_path = os.path.join(odin_driver_pkg, 'config', 'control_command.yaml')
    with open(pcd2depth_config_path, 'r') as f:
        pcd2depth_params = yaml.safe_load(f)
    pcd2depth_calib_path = os.path.join(odin_driver_pkg, 'config', 'calib.yaml')
    pcd2depth_params['calib_file_path'] = pcd2depth_calib_path
    pcd2depth_node = Node(
        package='odin_ros_driver',
        executable='pcd2depth_ros2_node',
        name='pcd2depth_ros2_node',
        output='screen',
        parameters=[pcd2depth_params]
    )

    reprojection_config_path = os.path.join(odin_driver_pkg, 'config', 'control_command.yaml')
    with open(reprojection_config_path, 'r') as f:
        reprojection_params = yaml.safe_load(f)
    reprojection_calib_path = os.path.join(odin_driver_pkg, 'config', 'calib.yaml')
    reprojection_params['calib_file_path'] = reprojection_calib_path
    cloud_reprojection_node = Node(
        package='odin_ros_driver',
        executable='cloud_reprojection_ros2_node',
        name='cloud_reprojection_ros2_node',
        output='screen',
        parameters=[reprojection_params]
    )

    overlay_config_path = os.path.join(odin_driver_pkg, 'config', 'control_command.yaml')
    with open(overlay_config_path, 'r') as f:
        overlay_params = yaml.safe_load(f)
    image_overlay_node = Node(
        package='odin_ros_driver',
        executable='image_overlay_node',
        name='image_overlay_node',
        output='screen',
        parameters=[overlay_params]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )

    transformer_config = os.path.join(transformer_pkg, 'config', 'default.yaml')
    coordinate_transformer_node = Node(
        package='coordinate_transformer',
        executable='coordinate_transformer',
        name='coordinate_transformer',
        output='screen',
        parameters=[transformer_config],
    )

    return LaunchDescription([
        config_file_arg,
        rviz_config_arg,
        host_sdk_node,
        pcd2depth_node,
        cloud_reprojection_node,
        image_overlay_node,
        rviz_node,
        coordinate_transformer_node,
    ])
