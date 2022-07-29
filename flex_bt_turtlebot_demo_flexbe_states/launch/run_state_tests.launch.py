from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


flexbe_testing_dir = get_package_share_directory('flexbe_testing')
flex_bt_demo_states_test_dir = get_package_share_directory('flex_bt_turtlebot_demo_flexbe_states')

path = flex_bt_demo_states_test_dir + "/test"

testcases  = path + "/charging_state.test \n"
testcases += path + "/check_battery_life_state.test \n"
testcases += path + "/get_waypoints_state.test \n"
testcases += path + "/send_waypoints_state.test"


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("compact_format", default_value="True"),
        DeclareLaunchArgument("testcases", default_value=testcases),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(flexbe_testing_dir + "/launch/flexbe_testing.launch.py"),
            launch_arguments={
                'compact_format': LaunchConfiguration("compact_format"),
                'testcases': LaunchConfiguration("testcases"),
            }.items()
        )
    ])
