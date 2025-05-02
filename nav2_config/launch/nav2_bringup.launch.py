from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Path to the default nav2_bringup launch file
    nav2_bringup_path = os.path.join(
        '/opt/ros/jazzy/share/nav2_bringup/launch',
        'navigation_launch.py'
    )
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_bringup_path),
            launch_arguments={
                'params_file': '/home/adam/ros2_ws/src/nav2_config/config/nav2_params.yaml',
                'use_sim_time': 'true'  # Add other arguments if necessary
            }.items(),
        ),
    ])

def generate_amcl_node():
    return Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'scan_topic': '/scan',
            # You can include other AMCL tuning parameters as needed.
        }]
    )