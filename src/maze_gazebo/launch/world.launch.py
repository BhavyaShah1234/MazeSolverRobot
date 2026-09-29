import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def get_scene():
    share_dir = get_package_share_directory('maze_gazebo')
    with open(os.path.join(share_dir, 'config', 'scene.yaml'), 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    share_dir = get_package_share_directory('maze_gazebo')
    scene = get_scene()
    camera_cfg = scene['overhead_camera']
    camera_pose = {
        'x': camera_cfg['x'],
        'y': camera_cfg['y'],
        'z': camera_cfg['robot_height_m'] + camera_cfg['clearance_m'],
        'roll': camera_cfg['roll'],
        'pitch': camera_cfg['pitch'],
        'yaw': camera_cfg['yaw'],
    }

    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(
        get_package_share_directory('franka_description'))
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world_file = os.path.join(share_dir, 'worlds', 'empty_with_sensors.sdf')
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'{world_file} -r'}.items(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    overhead_camera_model = os.path.join(share_dir, 'models', 'overhead_camera', 'model.sdf')
    spawn_overhead_camera = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', overhead_camera_model,
            '-name', 'overhead_camera',
            '-x', str(camera_pose['x']),
            '-y', str(camera_pose['y']),
            '-z', str(camera_pose['z']),
            '-R', str(camera_pose['roll']),
            '-P', str(camera_pose['pitch']),
            '-Y', str(camera_pose['yaw']),
        ],
        output='screen',
    )

    overhead_camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/overhead_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/overhead_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/overhead_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
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
    overhead_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', str(camera_pose['x']),
            '--y', str(camera_pose['y']),
            '--z', str(camera_pose['z']),
            '--roll', str(camera_pose['roll']),
            '--pitch', str(camera_pose['pitch']),
            '--yaw', str(camera_pose['yaw']),
            '--frame-id', 'world',
            '--child-frame-id', 'overhead_camera/link/overhead_rgbd_camera',
        ],
        output='screen',
    )

    referee = Node(
        package='maze_gazebo',
        executable='referee_node.py',
        output='screen',
    )

    return LaunchDescription([
        gazebo_world,
        clock_bridge,
        spawn_overhead_camera,
        overhead_camera_bridge,
        overhead_camera_tf,
        referee,
    ])
