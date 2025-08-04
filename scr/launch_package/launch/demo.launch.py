from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():


    ld = LaunchDescription()

    talker_node = Node(
        package = "demo_nodes_cpp",
        executable= "talker",
        name="talker_node",
        remappings=[
            ("/chatter", "/my_chatter"),
        ]
    )

    listener_node = Node(
        package="demo_nodes_py",
        executable="listener",
        remappings=[
            ("/chatter", "/my_chatter"),]
    )


    ld.add_action(talker_node)

    ld.add_action(listener_node)

    return ld