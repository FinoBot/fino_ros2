from ament_index_python import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Camera launch file definition
    depthai_filters = get_package_share_directory('depthai_filters')

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(depthai_filters, "launch", "spatial_bb.launch.py")
        ),
        # launch_arguments={
        #     "params_file": params_file,
        # }.items(),
    )

    # Node definitions
    driver = Node(
        package='fino_ros2',
        executable='finobot_driver',
        name='finobot_driver',
        output='screen'
    )

    movement_controller = Node(
        package='fino_ros2',
        executable='movement_controller',
        name='movement_controller',
        output='screen'
    )

    human_detector = Node(
        package='fino_ros2',
        executable='human_detector',
        name='human_detector',
        output='screen'
    )
    
    state_manager = Node(
        package='fino_ros2',
        executable='state_manager',
        name='state_manager',
        output='screen'
    )
    
    
    return LaunchDescription([
        camera_launch,
        driver,
        human_detector,
        state_manager,
        movement_controller,
    ])