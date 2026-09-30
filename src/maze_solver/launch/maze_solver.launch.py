# Base class for a launch file's returned description.
from launch import LaunchDescription
# Declares a launch argument that can be overridden from the command line.
from launch.actions import DeclareLaunchArgument
# Reference to a launch argument's value, resolved at launch time.
from launch.substitutions import LaunchConfiguration
# Node action: launches a single ROS2 node process.
from launch_ros.actions import Node


# Entry point ros2 launch calls to build this launch file's description.
def generate_launch_description():
    # Resolves to whatever value the use_sim_time argument is given.
    use_sim_time = LaunchConfiguration('use_sim_time')
    # Return the full description: one launch argument plus the three solver nodes.
    return LaunchDescription([
        # Defaults to true (simulation); pass false for real hardware.
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # Launch perception_node, forwarding use_sim_time.
        Node(package='maze_solver', executable='perception_node', output='screen', parameters=[{'use_sim_time': use_sim_time}]),
        # Launch planning_node, forwarding use_sim_time.
        Node(package='maze_solver', executable='planning_node', output='screen', parameters=[{'use_sim_time': use_sim_time}]),
        # Launch control_node, forwarding use_sim_time.
        Node(package='maze_solver', executable='control_node', output='screen', parameters=[{'use_sim_time': use_sim_time}]),
    ])
