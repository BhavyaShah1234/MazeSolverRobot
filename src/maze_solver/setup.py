import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'maze_solver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bhavya Shah',
    maintainer_email='bshah43@asu.edu',
    description='Hardware-agnostic maze perception, planning, and control',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_node = maze_solver.perception_node:main',
            'planning_node = maze_solver.planning_node:main',
            'control_node = maze_solver.control_node:main',
        ],
    },
)
