"""Launches MoveIt2's move_group and, optionally, RViz.

Assembles the full MoveIt2 configuration from this package's own SRDF,
kinematics, joint-limits, and OMPL config files via ``MoveItConfigsBuilder``,
starts ``move_group``, and starts RViz preloaded with the robot, planning
scene, and this project's maze-solving topics.
"""

# os.path, used to build xacro/rviz file paths.
import os

# Resolves this and other packages' installed share directories.
from ament_index_python.packages import get_package_share_directory
# Base class for a launch file's returned description.
from launch import LaunchDescription
# Declares a launch argument that can be overridden from the command line.
from launch.actions import DeclareLaunchArgument
# Gates whether an action runs, based on a launch argument's value.
from launch.conditions import IfCondition
# Reference to a launch argument's value, resolved at launch time.
from launch.substitutions import LaunchConfiguration
# Node action: launches a single ROS2 node/process.
from launch_ros.actions import Node
# Assembles a complete MoveIt2 configuration from this package's config files.
from moveit_configs_utils import MoveItConfigsBuilder


# Entry point ros2 launch calls to build this launch file's description.
def generate_launch_description() -> LaunchDescription:
    """Builds the launch description for move_group and RViz.

    Returns:
        The launch description: the declared arguments, the move_group
        node, and the (conditionally launched) RViz node.
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
    # Resolves to whatever the use_rviz argument is given.
    use_rviz = LaunchConfiguration('use_rviz')

    # This launch file's full set of overridable arguments.
    declared_arguments = [
        # Selects Gazebo vs. real/mock hardware.
        DeclareLaunchArgument('use_sim', default_value='true'),
        # Only meaningful outside simulation: swaps in mock hardware instead of the real robot.
        DeclareLaunchArgument('use_fake_hardware', default_value='false'),
        # The real FR3's network address.
        DeclareLaunchArgument('robot_ip', default_value=''),
        # Robot base mount X position.
        DeclareLaunchArgument('base_x', default_value='0.0'),
        # Robot base mount Y position.
        DeclareLaunchArgument('base_y', default_value='0.0'),
        # Robot base mount Z position.
        DeclareLaunchArgument('base_z', default_value='0.0'),
        # Robot base mount yaw.
        DeclareLaunchArgument('base_yaw', default_value='0.0'),
        # Whether to also open RViz alongside move_group.
        DeclareLaunchArgument('use_rviz', default_value='true'),
    ]

    # Path to the robot's top-level xacro file.
    xacro_path = os.path.join(
        get_package_share_directory('maze_description'), 'urdf', 'maze_robot.urdf.xacro')

    # Build the full MoveIt2 configuration from this package's own config files.
    moveit_config = (
        # Named after this package's SRDF/config file base name.
        MoveItConfigsBuilder('maze_robot', package_name='maze_moveit_config')
        # Processes the robot's xacro into a URDF, forwarding the same launch arguments as bringup.
        .robot_description(
            file_path=xacro_path,
            mappings={
                # Forward the sim/hardware switch.
                'use_sim': use_sim,
                # Forward the fake-hardware switch.
                'use_fake_hardware': use_fake_hardware,
                # Forward the robot's IP.
                'robot_ip': robot_ip,
                # Forward the base X position.
                'base_x': base_x,
                # Forward the base Y position.
                'base_y': base_y,
                # Forward the base Z position.
                'base_z': base_z,
                # Forward the base yaw.
                'base_yaw': base_yaw,
            })
        # Loads maze_robot.srdf.
        .robot_description_semantic()
        # Loads kinematics.yaml.
        .robot_description_kinematics()
        # Loads joint_limits.yaml.
        .joint_limits()
        # Only the OMPL planning pipeline is configured for this project.
        .planning_pipelines(pipelines=['ompl'])
        # Loads moveit_controllers.yaml and other execution settings.
        .trajectory_execution()
        # Configures the planning scene monitor's default topics.
        .planning_scene_monitor()
        # Finalize into a single MoveItConfigs object.
        .to_moveit_configs()
    )

    # Launches move_group, MoveIt2's central planning/execution node.
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            # The full assembled MoveIt2 configuration.
            moveit_config.to_dict(),
            # Use simulation time when running under Gazebo.
            {'use_sim_time': use_sim},
        ],
    )

    # Path to this project's RViz configuration.
    rviz_config_path = os.path.join(
        get_package_share_directory('maze_moveit_config'), 'rviz', 'maze_solver.rviz')

    # Launches RViz, preloaded with the robot, planning scene, and the maze-solving topics.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        # Only launched when use_rviz is true.
        condition=IfCondition(use_rviz),
        parameters=[
            # Needed so RViz's RobotModel display can render the arm.
            moveit_config.robot_description,
            # Needed for the MotionPlanning plugin's planning-group awareness.
            moveit_config.robot_description_semantic,
            # Needed for the MotionPlanning plugin's IK queries.
            moveit_config.robot_description_kinematics,
            # Needed for the MotionPlanning plugin's planner selection.
            moveit_config.planning_pipelines,
            # Needed for the MotionPlanning plugin's joint-limit awareness.
            moveit_config.joint_limits,
            # Use simulation time when running under Gazebo.
            {'use_sim_time': use_sim},
        ],
    )

    # Return the full launch description: arguments, move_group, and RViz.
    return LaunchDescription(declared_arguments + [move_group_node, rviz_node])
