from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fino_ros2',
            executable='finobot_driver',
            name='finobot_driver',
            output='screen'
        ),
        Node(
            package='fino_ros2',
            executable='movement_controller',
            name='movement_controller',
            output='screen'
        ),
        Node(
            package='fino_ros2',
            executable='human_detector',
            name='human_detector',
            output='screen'
        ),
        Node(
            package='fino_ros2',
            executable='state_manager',
            name='state_manager',
            output='screen'
        )
        # Node(
        #     package='fino_ros2',
        #     executable='joy_driver',
        #     name='joy_controler',
        #     output='screen'
        # )
    ])