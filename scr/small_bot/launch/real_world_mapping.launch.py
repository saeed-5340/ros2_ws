import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'small_bot'
    
    # Get URDF path
    urdf_path = os.path.join(
        get_package_share_directory(package_name),
        'design',
        'my_bot.urdf.xacro'
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': f"$(command 'xacro {urdf_path}')",
            'use_sim_time': False
        }]
    )
    
    # Real LiDAR Node
    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        parameters=[{
            'serial_port': '/dev/serial/by-path/pci-0000:00:14.0-usb-0:5:1.0-port0',
            'frame_id': 'lidar_link',
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }],
        output='screen'
    )
    
    # SLAM Toolbox for mapping
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'base_frame': 'base_footprint',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'scan_topic': '/scan',
            'mode': 'mapping'
        }]
    )
    
    # RViz for visualization
    rviz_config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'odom_fix.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}]
    )
    
    return LaunchDescription([
        robot_state_publisher,
        lidar_node,
        slam_toolbox,
        rviz_node
    ])