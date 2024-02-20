from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    webcam_pub = Node(
        package="auv",
        executable="webcam_pub"
    )

    webcam_sub = Node(
        package="auv",
        executable="webcam_sub"
    )

    ld.add_action(webcam_pub)
    ld.add_action(webcam_sub)

    return ld