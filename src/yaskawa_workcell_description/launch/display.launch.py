import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('yaskawa_workcell_description').find('yaskawa_workcell_description')
    default_model_path = os.path.join(pkg_share, 'urdf', 'workcell.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'workcell.rviz')

    doc = xacro.process_file(default_model_path)
    robot_description_config = doc.toxml()

    map_to_world_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_world_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'world']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='rvizconfig',
            default_value=default_rviz_config_path,
            description='Absolute path to rviz config file'
        ),
        map_to_world_tf,
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
