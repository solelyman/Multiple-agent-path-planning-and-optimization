import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    tracking_pkg_dir = get_package_share_directory('tracking_control')
    config_file = os.path.join(tracking_pkg_dir, 'config', 'vehicle_params.yaml')

    sf_controller = Node(
        package='tracking_control',
        executable='sf_local_tracking',
        name='sf_controller',
        output='screen',
        parameters=[config_file],
        remappings=[('odom', '/model/eca_a9/odometry')],
    )

    thruster_allocator = Node(
        package='tracking_control',
        executable='thruster_allocator',
        name='thruster_allocator',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([sf_controller, thruster_allocator])
