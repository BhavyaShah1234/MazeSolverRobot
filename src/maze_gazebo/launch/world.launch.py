"""Launches the Gazebo world, overhead camera, and referee node.

Starts Gazebo with the maze-solving world, spawns the overhead camera model
at its configured pose, bridges its topics and the simulation clock onto
ROS2, publishes the camera's static TF transform, and launches the referee
node that drives the continuous spawn/solve/respawn loop.
"""

# os.path/os.environ, used for file paths and setting GZ_SIM_RESOURCE_PATH.
import os

# YAML parsing for scene.yaml.
import yaml
# Resolves this and other packages' installed share directories.
from ament_index_python.packages import get_package_share_directory
# Base class for a launch file's returned description.
from launch import LaunchDescription
# Includes another package's launch file (ros_gz_sim's gz_sim.launch.py).
from launch.actions import IncludeLaunchDescription
# Tells IncludeLaunchDescription the included file is a Python launch file.
from launch.launch_description_sources import PythonLaunchDescriptionSource
# Node action: launches a single ROS2 node/process.
from launch_ros.actions import Node


# Load and parse this package's scene.yaml (camera pose, maze layout/sizing).
def get_scene() -> dict:
    """Loads and parses this package's scene.yaml.

    Returns:
        The parsed scene configuration (camera pose, maze layout/sizing).
    """
    # This package's installed share directory.
    share_dir = get_package_share_directory('maze_gazebo')
    # Open scene.yaml from the installed config directory.
    with open(os.path.join(share_dir, 'config', 'scene.yaml'), 'r') as f:
        # Parse and return the whole scene document.
        return yaml.safe_load(f)


# Entry point ros2 launch calls to build this launch file's description.
def generate_launch_description() -> LaunchDescription:
    """Builds the launch description for the Gazebo world and camera.

    Returns:
        The launch description: the Gazebo world, clock/camera bridges, the
        camera spawn and its static TF, and the referee node.
    """
    # This package's installed share directory.
    share_dir = get_package_share_directory('maze_gazebo')
    # The parsed scene configuration.
    scene = get_scene()
    # Just the overhead-camera section of the scene config.
    camera_cfg = scene['overhead_camera']
    # Assemble the camera's actual spawn pose from the config values.
    camera_pose = {
        # Camera X position.
        'x': camera_cfg['x'],
        # Camera Y position.
        'y': camera_cfg['y'],
        # Camera height = robot's highest point + configured clearance above it.
        'z': camera_cfg['robot_height_m'] + camera_cfg['clearance_m'],
        # Camera roll.
        'roll': camera_cfg['roll'],
        # Camera pitch.
        'pitch': camera_cfg['pitch'],
        # Camera yaw.
        'yaw': camera_cfg['yaw'],
    }

    # Point Gazebo's resource search path at franka_description's parent, so mesh references resolve.
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(
        get_package_share_directory('franka_description'))
    # ros_gz_sim's installed share directory, needed to include its own launch file.
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    # Path to this package's world SDF file.
    world_file = os.path.join(share_dir, 'worlds', 'empty_with_sensors.sdf')
    # Include ros_gz_sim's launch file to actually start Gazebo with this world, running (-r).
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'{world_file} -r'}.items(),
    )

    # Bridges Gazebo's simulated clock onto ROS2's /clock topic.
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # Path to the overhead camera's own model SDF.
    overhead_camera_model = os.path.join(share_dir, 'models', 'overhead_camera', 'model.sdf')
    # Spawns the overhead camera model into Gazebo at the configured pose.
    spawn_overhead_camera = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            # The model file to spawn.
            '-file', overhead_camera_model,
            # The entity name to give it in Gazebo.
            '-name', 'overhead_camera',
            # Spawn X position.
            '-x', str(camera_pose['x']),
            # Spawn Y position.
            '-y', str(camera_pose['y']),
            # Spawn Z position.
            '-z', str(camera_pose['z']),
            # Spawn roll.
            '-R', str(camera_pose['roll']),
            # Spawn pitch.
            '-P', str(camera_pose['pitch']),
            # Spawn yaw.
            '-Y', str(camera_pose['yaw']),
        ],
        output='screen',
    )

    # Bridges the camera's Gazebo topics onto their ROS2 equivalents.
    overhead_camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Color image stream.
            '/overhead_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            # Camera intrinsics stream.
            '/overhead_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            # Depth image stream.
            '/overhead_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            # Point cloud stream.
            '/overhead_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
        ],
        output='screen',
    )

    # The camera is a standalone Gazebo model spawned outside the robot's
    # URDF, so nothing publishes its pose to /tf on its own. Since it's
    # fixed, a static transform using the exact spawn pose is all that's
    # needed. Child frame matches the sensor data's actual header.frame_id
    # (Gazebo's own scoped naming, "overhead_camera/link/overhead_rgbd_camera")
    # -- maze_solver's perception node reads this straight from each image's
    # own header rather than hardcoding it, so it isn't tied to this
    # particular naming convention.
    # Publishes the camera's fixed pose as a static TF transform.
    overhead_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            # Translation X.
            '--x', str(camera_pose['x']),
            # Translation Y.
            '--y', str(camera_pose['y']),
            # Translation Z.
            '--z', str(camera_pose['z']),
            # Rotation roll.
            '--roll', str(camera_pose['roll']),
            # Rotation pitch.
            '--pitch', str(camera_pose['pitch']),
            # Rotation yaw.
            '--yaw', str(camera_pose['yaw']),
            # Parent frame.
            '--frame-id', 'world',
            # Child frame, matching Gazebo's own scoped sensor frame name.
            '--child-frame-id', 'overhead_camera/link/overhead_rgbd_camera',
        ],
        output='screen',
    )

    # Launches the referee node that spawns mazes and drives the solve/reset loop.
    referee = Node(
        package='maze_gazebo',
        executable='referee_node.py',
        output='screen',
    )

    # Return the full launch description: world, bridges, camera spawn/TF, and the referee.
    return LaunchDescription([
        gazebo_world,
        clock_bridge,
        spawn_overhead_camera,
        overhead_camera_bridge,
        overhead_camera_tf,
        referee,
    ])
