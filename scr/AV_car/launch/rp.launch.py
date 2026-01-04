import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    
#     # urdf_path = '/home/saeed/ros2_ws/src/my_bot/description/my_bot.urdf.xacro'
#     # rviz2_path = '/home/saeed/ros2_ws/src/my_bot/config/view_bot.rviz'

    pkg_find = get_package_share_directory('small_bot')
    rviz2_path = os.path.join(pkg_find, 'config', 'view_bot.rviz')

    urdf_path =os.path.join(pkg_find,'design','my_bot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ' + urdf_path]), value_type=str)



    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time':use_sim_time,'robot_description':robot_description}]
        # parameters=[{'robot_description':robot_description}]
    )

    node_joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    node_rviz2 = Node(
        package ='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d',rviz2_path]
        )

    return LaunchDescription([
        node_robot_state_publisher,
        node_rviz2,
        node_joint_state_publisher,
        declare_use_sim_time
    ])


# import os
# from launch import LaunchDescription
# from launch.substitutions import Command
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# from launch_ros.parameter_descriptions import ParameterValue

# def generate_launch_description():
#     # Paths
#     pkg_share = get_package_share_directory('small_bot')
#     urdf_path = os.path.join(pkg_share, 'design', 'my_bot.urdf.xacro')
#     robot_rviz_path = os.path.join(pkg_share, 'config', 'view_bot.rviz')

#     # Robot State Publisher Node
#     node_robot_state_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         parameters=[{
#             'robot_description': ParameterValue(
#                 Command(['xacro ' + urdf_path]),
#                 value_type=str
#             )
#         }]
#     )

#     # Joint State Publisher GUI Node
#     node_joint_state_publisher_gui = Node(
#         package='joint_state_publisher_gui',
#         executable='joint_state_publisher_gui',
#         output='screen'
#     )

#     # RViz2 Node
#     node_rviz2 = Node(
#         package='rviz2',
#         executable='rviz2',
#         output='screen',
#         arguments=['-d', robot_rviz_path]
#     )

#     return LaunchDescription([
#         node_robot_state_publisher,
#         node_joint_state_publisher_gui,
#         node_rviz2
#     ])
