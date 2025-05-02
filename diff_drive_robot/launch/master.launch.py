from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command

def generate_launch_description():
    package_dir = get_package_share_directory('diff_drive_robot')
    urdf_path = os.path.join(package_dir, 'urdf', 'robot.urdf.xacro')
    map_path = os.path.join(package_dir, 'maps', 'my_map.yaml')

    nav2_launch = os.path.join(
        '/opt/ros/jazzy/share/nav2_bringup/launch',
        'navigation_launch.py'
    )
    slam_launch = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch', 'online_async_launch.py'
    )

    # Include Nav2 bringup without composition/autostart
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'params_file': os.path.join(package_dir, 'config', 'nav2_params.yaml'),
            'use_sim_time': 'false',
            'autostart': 'false',
            'use_composition': 'False'
        }.items()
    )

    # Launch controller_server under GDB in current terminal
    controller_server_gdb = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        prefix='gdb --args',  # run under GDB in this shell
        output='screen',
        parameters=[os.path.join(package_dir, 'config', 'nav2_params.yaml')]
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        launch_arguments={
            'use_sim_time': 'false',
            'slam_params_file': os.path.join(package_dir, 'config', 'slam_toolbox_config.yaml')
        }.items()
    )

    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 460800,
            'frame_id': 'laser_frame',
            'inverted': False,
            'angle_compensate': True
        }],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(package_dir, 'rviz', 'default.rviz')],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': False,
            'robot_description': Command(['xacro ', urdf_path])
        }],
        output='screen'
    )

    motor_controller = Node(
        package='motor_controller',
        executable='motor_controller',
        parameters=[{'velocity_topic': '/cmd_vel'}],
        output='screen'
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        parameters=[os.path.join(package_dir, 'config', 'lifecycle_manager.yaml')],
        output='screen'
    )

    configure_slam = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
                output='screen'
            )
        ]
    )

    activate_slam = TimerAction(
        period=30.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        nav2_bringup,
        #controller_server_gdb,
        slam_toolbox,
        lidar_node,
        rviz_node,
        robot_state_publisher,
        motor_controller,
        lifecycle_manager,
        configure_slam,
        activate_slam,
    ])
