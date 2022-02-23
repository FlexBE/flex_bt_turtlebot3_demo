import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

def configure_hokuyo(context):
    urg_node_dir = get_package_share_directory('urg_node')
    param_file = os.path.join(urg_node_dir, 'launch',
        'urg_node_' + context.launch_configurations['sensor_interface'] + '.yaml')
    if os.path.exists(param_file):
        return [SetLaunchConfiguration('param', param_file)]

def generate_launch_description():
    turtlebot_bringup_dir = get_package_share_directory('flex_bt_turtlebot_demo_bringup')

    # Configure Kobuki base node
    params_file = os.path.join(turtlebot_bringup_dir, 'param', 'kobuki_node_params.yaml')
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['kobuki_ros_node']['ros__parameters']
    kobuki_ros_node = Node(package='kobuki_node',
                           executable='kobuki_ros_node',
                           output='screen',
                           parameters=[params],
                           remappings=[('commands/velocity', 'cmd_vel')])

    launch_description = LaunchDescription([
        DeclareLaunchArgument('sensor_interface', default_value='serial',
            description='sensor_interface: supported: serial, ethernet')])

    # Configure Hokuyo LiDAR node
    param_file_path = OpaqueFunction(function=configure_hokuyo)
    launch_description.add_action(param_file_path)

    hokuyo_node = Node(package='urg_node', executable='urg_node_driver', output='screen',
                       parameters=[LaunchConfiguration('param')],
                       remappings=[('scan', 'hokuyo_node/scan_raw')])

    # Confiugre laser filter for Hokuyo LiDAR
    filters = os.path.join(turtlebot_bringup_dir, 'param', 'laser_filters.yaml')
    laser_filters = Node(package="laser_filters", executable="scan_to_scan_filter_chain",
        output="screen", name="laser_filter",
        remappings=[("scan", 'hokuyo_node/scan_raw'), ("scan_filtered", "hokuyo_node/scan")],
        parameters=[filters]
    )

    # Static transform between Kobuki base and Hokuyo LiDAR
    node = Node(package = "tf2_ros",
                       executable = "static_transform_publisher",
                       arguments = ["0 0 0.3 0 0 0 base_link laser"])

    # Add all the nodes and then launch
    launch_description.add_action(kobuki_ros_node)
    launch_description.add_action(hokuyo_node)
    launch_description.add_action(laser_filters)
    return launch_description
