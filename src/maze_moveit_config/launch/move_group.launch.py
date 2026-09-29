import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    robot_ip = LaunchConfiguration('robot_ip')
    base_x = LaunchConfiguration('base_x')
    base_y = LaunchConfiguration('base_y')
    base_z = LaunchConfiguration('base_z')
    base_yaw = LaunchConfiguration('base_yaw')
    use_rviz = LaunchConfiguration('use_rviz')

    declared_arguments = [
        DeclareLaunchArgument('use_sim', default_value='true'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false'),
        DeclareLaunchArgument('robot_ip', default_value=''),
        DeclareLaunchArgument('base_x', default_value='0.0'),
        DeclareLaunchArgument('base_y', default_value='0.0'),
        DeclareLaunchArgument('base_z', default_value='0.0'),
        DeclareLaunchArgument('base_yaw', default_value='0.0'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
    ]

    xacro_path = os.path.join(
        get_package_share_directory('maze_description'), 'urdf', 'maze_robot.urdf.xacro')

    moveit_config = (
        MoveItConfigsBuilder('maze_robot', package_name='maze_moveit_config')
        .robot_description(
            file_path=xacro_path,
            mappings={
                'use_sim': use_sim,
                'use_fake_hardware': use_fake_hardware,
                'robot_ip': robot_ip,
                'base_x': base_x,
                'base_y': base_y,
                'base_z': base_z,
                'base_yaw': base_yaw,
            })
        .robot_description_semantic()
        .robot_description_kinematics()
        .joint_limits()
        .planning_pipelines(pipelines=['ompl'])
        .trajectory_execution()
        .planning_scene_monitor()
        .to_moveit_configs()
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': use_sim},
        ],
    )

    rviz_config_path = os.path.join(
        get_package_share_directory('maze_moveit_config'), 'rviz', 'maze_solver.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_rviz),
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': use_sim},
        ],
    )

    return LaunchDescription(declared_arguments + [move_group_node, rviz_node])
