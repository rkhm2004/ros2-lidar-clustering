from launch import LaunchDescription
from launch_ros.actions import Node

"""
COMPLETED: Complete this launch file to:
1. Launch the 'talker' node from package 'ros2_launch_demo' with:
   - Parameter 'message_prefix' set to 'ROS2'
2. Launch the 'listener' node from package 'ros2_launch_demo'
"""

def generate_launch_description():
    return LaunchDescription(
        [
            # 1. Add talker node with message_prefix parameter
            Node(
                package='ros2_launch_demo',
                executable='talker',
                name='talker',
                parameters=[
                    {'message_prefix': 'ROS2'}
                ]
            ),
            
            # 2. Add listener node
            Node(
                package='ros2_launch_demo',
                executable='listener',
                name='listener'
            )
        ]
    )