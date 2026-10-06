import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('yaskawa_workcell_description').find('yaskawa_workcell_description')
    default_model_path = os.path.join(pkg_share, 'urdf', 'robot_only.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'robot_only.rviz')

    doc = xacro.process_file(default_model_path)
    robot_description_config = doc.toxml()

    map_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_base_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
    )

    world_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='rvizconfig',
            default_value=default_rviz_config_path,
            description='Absolute path to rviz config file'
        ),
        map_to_base_tf,
        world_to_base_tf,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_config}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
            parameters=[{'robot_description': robot_description_config}]
        )
    ])
