"""Launches robot bring-up: state publishing, controllers, and Gazebo.

Processes the robot's xacro into ``robot_description``, starts
``robot_state_publisher``, and brings up either the Gazebo simulation path
or the real/mock hardware path, selected by the ``use_sim`` argument.
"""

# os.path, used to build config/xacro file paths.
import os

# Resolves this and other packages' installed share directories.
from ament_index_python.packages import get_package_share_directory
# Base class for a launch file's returned description.
from launch import LaunchDescription
# Declares launch arguments and includes another launch file.
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# Conditions that gate whether an action runs, based on a launch argument's value.
from launch.conditions import IfCondition, UnlessCondition
# Tells IncludeLaunchDescription the included file is a Python launch file.
from launch.launch_description_sources import PythonLaunchDescriptionSource
# Substitutions: values resolved at launch time rather than at file-parse time.
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression)
# Node action: launches a single ROS2 node/process.
from launch_ros.actions import Node
# Wraps a substitution's result with an explicit parameter type (here, string).
from launch_ros.parameter_descriptions import ParameterValue
# Resolves an installed package's share directory as a launch-time substitution.
from launch_ros.substitutions import FindPackageShare


# Entry point ros2 launch calls to build this launch file's description.
def generate_launch_description() -> LaunchDescription:
    """Builds the launch description for robot bring-up.

    Returns:
        The launch description: the declared arguments, robot_state_publisher,
        the Gazebo simulation path, and the real/mock hardware path, each
        gated by ``use_sim``/``use_fake_hardware``.
    """
    # Resolves to whatever the use_sim argument is given.
    use_sim = LaunchConfiguration('use_sim')
    # Resolves to whatever the use_fake_hardware argument is given.
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    # Resolves to whatever the robot_ip argument is given.
    robot_ip = LaunchConfiguration('robot_ip')
    # Resolves to whatever the base_x argument is given.
    base_x = LaunchConfiguration('base_x')
    # Resolves to whatever the base_y argument is given.
    base_y = LaunchConfiguration('base_y')
    # Resolves to whatever the base_z argument is given.
    base_z = LaunchConfiguration('base_z')
    # Resolves to whatever the base_yaw argument is given.
    base_yaw = LaunchConfiguration('base_yaw')

    # This launch file's full set of overridable arguments.
    declared_arguments = [
        # Selects Gazebo vs. real/mock hardware.
        DeclareLaunchArgument('use_sim', default_value='true',
                               description='true: Gazebo; false: real hardware or mock hardware'),
        # Only meaningful outside simulation: swaps in mock hardware instead of the real robot.
        DeclareLaunchArgument('use_fake_hardware', default_value='false',
                               description='Only meaningful when use_sim is false: mock_components/GenericSystem instead of the real robot'),
        # The real FR3's network address, needed only for the genuine-hardware path.
        DeclareLaunchArgument('robot_ip', default_value='',
                               description='Only meaningful when use_sim and use_fake_hardware are both false'),
        # Robot base mount X position.
        DeclareLaunchArgument('base_x', default_value='0.0'),
        # Robot base mount Y position.
        DeclareLaunchArgument('base_y', default_value='0.0'),
        # Robot base mount Z position.
        DeclareLaunchArgument('base_z', default_value='0.0'),
        # Robot base mount yaw.
        DeclareLaunchArgument('base_yaw', default_value='0.0'),
    ]

    # Path to the robot's top-level xacro file.
    xacro_path = os.path.join(
        get_package_share_directory('maze_description'), 'urdf', 'maze_robot.urdf.xacro')
    # Build the shell command that runs xacro with all the relevant arguments forwarded.
    robot_description_content = Command([
        # The xacro executable itself.
        FindExecutable(name='xacro'), ' ', xacro_path,
        # Forward the sim/hardware switch.
        ' use_sim:=', use_sim,
        # Forward the fake-hardware switch.
        ' use_fake_hardware:=', use_fake_hardware,
        # Forward the robot's IP.
        ' robot_ip:=', robot_ip,
        # Forward the base X position.
        ' base_x:=', base_x,
        # Forward the base Y position.
        ' base_y:=', base_y,
        # Forward the base Z position.
        ' base_z:=', base_z,
        # Forward the base yaw.
        ' base_yaw:=', base_yaw,
    ])
    # Wrap the xacro command's output as the robot_description parameter (a string).
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # This package's installed config directory.
    bringup_config = get_package_share_directory('maze_bringup')
    # Path to the manager-wide controller_manager settings.
    controller_manager_yaml = os.path.join(bringup_config, 'config', 'controller_manager.yaml')
    # Path to the position-based (sim/mock) joint_trajectory_controller config.
    jtc_sim_yaml = os.path.join(bringup_config, 'config', 'joint_trajectory_controller_sim.yaml')
    # Path to the effort-based (real hardware) joint_trajectory_controller config.
    jtc_hardware_yaml = os.path.join(bringup_config, 'config', 'joint_trajectory_controller_hardware.yaml')

    # mock_components/GenericSystem exposes position/velocity/effort command
    # interfaces alike, so it uses the same position-based controller config
    # as Gazebo; only a real robot needs the effort+PID config
    # (franka_hardware/FrankaHardwareInterface is fundamentally torque-
    # controlled, per franka_fr3_moveit_config's own reference).
    # True whenever the position-based controller config should be used (sim or fake hardware).
    is_position_based = PythonExpression(
        ['"true" if "', use_sim, '" == "true" or "', use_fake_hardware, '" == "true" else "false"'])
    # True only for genuine real hardware (neither sim nor fake).
    is_effort_based = PythonExpression(
        ['"true" if "', use_sim, '" == "false" and "', use_fake_hardware, '" == "false" else "false"'])

    # Publishes /robot_description and /tf from the URDF and joint states.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': use_sim}],
    )

    # --- Simulation path ---
    # Includes maze_gazebo's world launch file, only when simulating.
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('maze_gazebo'), 'launch', 'world.launch.py'])),
        condition=IfCondition(use_sim),
    )
    # Spawns the robot model into Gazebo from the published robot_description, only when simulating.
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
    # Runs a standalone controller_manager, only when NOT simulating.
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
    # Loads and activates joint_state_broadcaster, regardless of sim/hardware path.
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '30'],
        output='screen',
    )

    # See joint_trajectory_controller_sim.yaml's own comment: this is what
    # adds joint_trajectory_controller to a Gazebo-embedded controller_manager
    # that never heard of it at startup, without touching the URDF.
    # Loads the position-based joint_trajectory_controller, only for sim/fake hardware.
    spawn_joint_trajectory_controller_sim = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '-p', jtc_sim_yaml, '--controller-manager-timeout', '30'],
        output='screen',
        condition=IfCondition(is_position_based),
    )
    # Loads the effort-based joint_trajectory_controller, only for real hardware.
    spawn_joint_trajectory_controller_hardware = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '-p', jtc_hardware_yaml, '--controller-manager-timeout', '30'],
        output='screen',
        condition=IfCondition(is_effort_based),
    )

    # Return the full launch description: arguments plus every node/include above.
    return LaunchDescription(declared_arguments + [
        robot_state_publisher,
        gazebo_world,
        spawn_robot,
        controller_manager_standalone,
        spawn_joint_state_broadcaster,
        spawn_joint_trajectory_controller_sim,
        spawn_joint_trajectory_controller_hardware,
    ])
