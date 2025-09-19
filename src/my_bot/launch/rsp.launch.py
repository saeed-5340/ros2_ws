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
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    
    urdf_path = '/home/saeed/ros2_ws/src/my_bot/description/my_bot.urdf.xacro'
    rviz2_path = '/home/saeed/ros2_ws/src/my_bot/config/view_bot.rviz'

    # pkg_find = get_package_share_directory('my_bot')
    # rviz2_path = os.path.join(pkg_find, 'rviz', 'view_bot.rviz')

    # urdf_path =os.path.join(pkg_find,'description','my_bot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ' + urdf_path]), value_type=str)



    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time':use_sim_time,'robot_description':robot_description}]
    )

    node_rviz2 = Node(
        package ='rviz2',
        executable='rviz2',
        arguments=['-d',rviz2_path]
        )

    return LaunchDescription([
        node_robot_state_publisher,
        node_rviz2,
        declare_use_sim_time
    ])