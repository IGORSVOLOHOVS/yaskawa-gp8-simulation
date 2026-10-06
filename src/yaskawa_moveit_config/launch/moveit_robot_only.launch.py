import os
import xacro
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def load_yaml(package_name, file_path):
    package_path = FindPackageShare(package_name).find(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    pkg_desc = FindPackageShare('yaskawa_workcell_description').find('yaskawa_workcell_description')
    pkg_moveit = FindPackageShare('yaskawa_moveit_config').find('yaskawa_moveit_config')

    urdf_path = os.path.join(pkg_desc, 'urdf', 'robot_only.urdf.xacro')
    srdf_path = os.path.join(pkg_moveit, 'config', 'yaskawa_workcell.srdf')

    doc = xacro.process_file(urdf_path)
    robot_description_config = doc.toxml()
    robot_description = {'robot_description': robot_description_config}

    with open(srdf_path, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    kinematics_yaml = load_yaml('yaskawa_moveit_config', 'config/kinematics.yaml')
    ompl_yaml = load_yaml('yaskawa_moveit_config', 'config/ompl_planning.yaml')
    joint_limits_yaml = load_yaml('yaskawa_moveit_config', 'config/joint_limits.yaml')
    moveit_controllers_yaml = load_yaml('yaskawa_moveit_config', 'config/moveit_controllers.yaml')

    ros2_controllers_path = os.path.join(pkg_moveit, 'config', 'ros2_controllers.yaml')

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

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_yaml,
            joint_limits_yaml,
            moveit_controllers_yaml,
            {'publish_robot_description_semantic': True},
            {'planning_plugin': 'ompl_interface/OMPLPlanner'}
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_controllers_path],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager']
    )

    rviz_config_file = os.path.join(pkg_moveit, 'rviz', 'moveit.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_yaml
        ]
    )

    return LaunchDescription([
        map_to_base_tf,
        world_to_base_tf,
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        move_group_node,
        rviz_node
    ])
