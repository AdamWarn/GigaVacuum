import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the share directory of your package containing the map (or use an absolute path)
    map_dir = get_package_share_directory('nav2_config')
    map_yaml_file = os.path.join(map_dir, 'config', 'map.yaml')

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml_file}]
        )
    ])
