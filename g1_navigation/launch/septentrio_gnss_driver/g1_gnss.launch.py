import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time}: [{name}] [{severity}]\t{message}'
# Verbose log:
#os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time}: [{name}] [{severity}]\t{message} ({function_name}() at {file_name}:{line_number})'

def generate_launch_description():
    package_path = get_package_share_directory('g1_navigation')
    default_config_path = os.path.join(package_path, 'config', 'septentrio_gnss_driver')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    

    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='gnss.yaml',
        description='Config file'
    )

    composable_node = ComposableNode(
        name='septentrio_gnss_driver',
        package='septentrio_gnss_driver', 
        plugin='rosaic_node::ROSaicNode',
        #emulate_tty=True,
        parameters=[PathJoinSubstitution([config_path, config_file])]
    )

    container = ComposableNodeContainer(
        name='septentrio_gnss_driver_container',
        namespace='septentrio_gnss_driver',
        package='rclcpp_components',
        executable='component_container_isolated',
        emulate_tty=True,
        sigterm_timeout = '20',
        composable_node_descriptions=[composable_node],
        output='screen'
    )

    return LaunchDescription([
        declare_config_path_cmd,
        declare_config_file_cmd,
        container
    ])