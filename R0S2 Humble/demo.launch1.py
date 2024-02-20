from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    draw_circle = Node(
        package="auv",
        executable="draw_circle"
    )
    talker_node = Node(
        package="demo_nodes_cpp",
        executable="talker",
        name="my_talker",
        remappings=[
            ("chatter", "my_chatter")
        ],
        parameters=[
            {"param_name": "value"},
            {"param2:19"}
        ]
    )
    
    listener_node = Node(
        package="demo_nodes_py",
        executable="listener",
        remappings=[
            ("chatter", "my_chatter")
        ]
    )

    turtlesim = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )
    
    ld.add_action(turtlesim)
    ld.add_action(talker_node)
    ld.add_action(listener_node)
    ld.add_action(draw_circle)
    

    return ld
    