from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='openpose_hand_publisher',
            executable='publish_hand_node.py',
            name='hand_publisher_node',
            output='screen'
        )
    ])
