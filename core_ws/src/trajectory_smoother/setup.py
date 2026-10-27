import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'trajectory_smoother'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lu',
    maintainer_email='lu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'smoother_node=trajectory_smoother.smoother_node:main',
            'perception_node=trajectory_smoother.perception_node:main',
            'usv_simulator=trajectory_smoother.usv_simulator:main',
            'kinematic_vessel_node=trajectory_smoother.kinematic_vessel_node:main',
            'odom_tf_broadcaster=trajectory_smoother.odom_tf_broadcaster:main',
            'online_trajectory_visualizer=trajectory_smoother.online_trajectory_visualizer:main',
            'robot_description_publisher=trajectory_smoother.robot_description_publisher:main',
            'demo_replay_node=trajectory_smoother.demo_replay_node:main',
        ],
    },
)
