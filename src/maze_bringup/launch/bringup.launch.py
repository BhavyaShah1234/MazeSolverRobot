import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    robot_ip = LaunchConfiguration('robot_ip')
    base_x = LaunchConfiguration('base_x')
    base_y = LaunchConfiguration('base_y')
    base_z = LaunchConfiguration('base_z')
    base_yaw = LaunchConfiguration('base_yaw')

    declared_arguments = [
        DeclareLaunchArgument('use_sim', default_value='true',
                               description='true: Gazebo; false: real hardware or mock hardware'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false',
                               description='Only meaningful when use_sim is false: mock_components/GenericSystem instead of the real robot'),
        DeclareLaunchArgument('robot_ip', default_value='',
                               description='Only meaningful when use_sim and use_fake_hardware are both false'),
        DeclareLaunchArgument('base_x', default_value='0.0'),
        DeclareLaunchArgument('base_y', default_value='0.0'),
        DeclareLaunchArgument('base_z', default_value='0.0'),
        DeclareLaunchArgument('base_yaw', default_value='0.0'),
    ]

    xacro_path = os.path.join(
        get_package_share_directory('maze_description'), 'urdf', 'maze_robot.urdf.xacro')
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_path,
        ' use_sim:=', use_sim,
        ' use_fake_hardware:=', use_fake_hardware,
        ' robot_ip:=', robot_ip,
        ' base_x:=', base_x,
        ' base_y:=', base_y,
        ' base_z:=', base_z,
        ' base_yaw:=', base_yaw,
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    bringup_config = get_package_share_directory('maze_bringup')
    controller_manager_yaml = os.path.join(bringup_config, 'config', 'controller_manager.yaml')
    jtc_sim_yaml = os.path.join(bringup_config, 'config', 'joint_trajectory_controller_sim.yaml')
    jtc_hardware_yaml = os.path.join(bringup_config, 'config', 'joint_trajectory_controller_hardware.yaml')

    # mock_components/GenericSystem exposes position/velocity/effort command
    # interfaces alike, so it uses the same position-based controller config
    # as Gazebo; only a real robot needs the effort+PID config
    # (franka_hardware/FrankaHardwareInterface is fundamentally torque-
    # controlled, per franka_fr3_moveit_config's own reference).
    is_position_based = PythonExpression(
        ['"true" if "', use_sim, '" == "true" or "', use_fake_hardware, '" == "true" else "false"'])
    is_effort_based = PythonExpression(
        ['"true" if "', use_sim, '" == "false" and "', use_fake_hardware, '" == "false" else "false"'])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': use_sim}],
    )

    # --- Simulation path ---
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('maze_gazebo'), 'launch', 'world.launch.py'])),
        condition=IfCondition(use_sim),
    )
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'fr3'],
        output='screen',
        condition=IfCondition(use_sim),
    )

    # --- Real/mock hardware path: Gazebo's own gz_ros2_control plugin embeds
    # controller_manager in-process (it must: GazeboSimSystem reads/writes
    # Gazebo's own simulation state directly, so it can only run inside the
    # gz-sim process, not as a separate node) -- a standalone ros2_control_node
    # is only needed, and only possible, when there's no Gazebo process to
    # embed it in. ---
    controller_manager_standalone = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_manager_yaml, {'use_sim_time': False}],
        output='screen',
        condition=UnlessCondition(use_sim),
    )

    # joint_state_broadcaster's type is already declared in whichever
    # controller_manager we end up with (Franka's own hardcoded example-
    # controllers config for the Gazebo-embedded case; controller_manager.yaml
    # above for the standalone case would need it too -- but spawner sets it
    # here explicitly either way so this file doesn't depend on that).
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '30'],
        output='screen',
    )

    # See joint_trajectory_controller_sim.yaml's own comment: this is what
    # adds joint_trajectory_controller to a Gazebo-embedded controller_manager
    # that never heard of it at startup, without touching the URDF.
    spawn_joint_trajectory_controller_sim = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '-p', jtc_sim_yaml, '--controller-manager-timeout', '30'],
        output='screen',
        condition=IfCondition(is_position_based),
    )
    spawn_joint_trajectory_controller_hardware = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '-p', jtc_hardware_yaml, '--controller-manager-timeout', '30'],
        output='screen',
        condition=IfCondition(is_effort_based),
    )

    return LaunchDescription(declared_arguments + [
        robot_state_publisher,
        gazebo_world,
        spawn_robot,
        controller_manager_standalone,
        spawn_joint_state_broadcaster,
        spawn_joint_trajectory_controller_sim,
        spawn_joint_trajectory_controller_hardware,
    ])
