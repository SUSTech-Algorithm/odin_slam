from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('coordinate_transformer')
    config_file = os.path.join(pkg_dir, 'config', 'default.yaml')

    coordinate_transformer_node = Node(
        package='coordinate_transformer',
        executable='coordinate_transformer',
        name='coordinate_transformer',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([
        coordinate_transformer_node,
    ])
