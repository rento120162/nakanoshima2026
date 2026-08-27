import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    cur_dir = get_package_share_directory('g1_navigation')
    nav2_dir = get_package_share_directory('nav2_gps_waypoint_follower_demo')
    #realsense_dir = get_package_share_directory('realsense_driver')
    #rl_params_file = os.path.join(gps_wpf_dir, "config", "dual_ekf_navsat_params.yaml")

    #realsense_cmd = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(
    #        os.path.join(realsense_dir, 'launch', 'launch.py')
    #    )
    #)
    gnss_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cur_dir, 'launch', 'septentrio_gnss_driver', 'g1_gnss.launch.py')
        )
    )

    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'gps_waypoint_follower.launch.py')
        )
    )

    pc2laser_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cur_dir, 'launch', 'obstacle_cloud_to_scan', 'obstacle_cloud_to_scan.launch.py')
        )
    )

    return LaunchDescription([

    Node(
        package='unitree_ros2_example',
        executable='g1_high_level_ros2',
        name='cmdvel_control',
    ),

    Node(
        package='g1_navigation',
        executable='gps_odom_filter',
        name='gps_odom_filter',
    ),

    Node(
        package='g1_navigation',
        executable='ros2serial',
        name='emrgency_switch',
    ),
    
    TimerAction(
        period=3.0,
        actions=[gnss_cmd]
    ),

    TimerAction(
        period=6.0,
        actions=[pc2laser_cmd]
    ),

    TimerAction(
        period=9.0,
        actions=[nav2_cmd]
    )

])