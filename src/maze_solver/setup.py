# os.path.join, used to build the launch-file install destination portably.
import os
# glob, used to find every *.launch.py file to install.
from glob import glob

# setuptools' package discovery and setup entry point.
from setuptools import find_packages, setup

# This package's name, reused throughout the setup() call below.
package_name = 'maze_solver'

# Standard ament_python setup() call.
setup(
    # Package name.
    name=package_name,
    # Package version.
    version='0.1.0',
    # Auto-discover the Python packages under this directory, excluding tests.
    packages=find_packages(exclude=['test']),
    # Non-Python files to install alongside the package.
    data_files=[
        # Registers this package with the ament resource index.
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Installs package.xml into the package's share directory.
        ('share/' + package_name, ['package.xml']),
        # Installs every launch file into the package's share/launch directory.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    # Runtime Python dependency (setuptools itself).
    install_requires=['setuptools'],
    # Safe to install as a zip archive.
    zip_safe=True,
    # Package maintainer name.
    maintainer='Bhavya Shah',
    # Package maintainer email.
    maintainer_email='bshah43@asu.edu',
    # Short description shown by package-listing tools.
    description='Hardware-agnostic maze perception, planning, and control',
    # License this package is distributed under.
    license='Apache 2.0',
    # Test dependency (pytest).
    tests_require=['pytest'],
    # Console-script entry points, one per node executable.
    entry_points={
        # This package's three runnable nodes.
        'console_scripts': [
            # Maps `ros2 run maze_solver perception_node` to perception_node.py's main().
            'perception_node = maze_solver.perception_node:main',
            # Maps `ros2 run maze_solver planning_node` to planning_node.py's main().
            'planning_node = maze_solver.planning_node:main',
            # Maps `ros2 run maze_solver control_node` to control_node.py's main().
            'control_node = maze_solver.control_node:main',
        ],
    },
)
