import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    topology_share = get_package_share_directory('topology_planner')
    smoother_param_dir = os.path.join(
        get_package_share_directory('trajectory_smoother'),
        'config',
        'parameter.yaml')
    guidance_param_dir = os.path.join(
        get_package_share_directory('guidance_planner'),
        'config',
        'ros2_params.yaml')
    rviz_config = os.path.join(topology_share, 'rviz', 'planner.rviz')
    use_rviz = LaunchConfiguration('use_rviz')
    use_models = LaunchConfiguration('use_models')

    agent_config = [
        {'id': 1, 'init_pose': [-22.0, -16.0, 0.0], 'target_offset': [-6.0, -6.0, 0.0]},
        {'id': 2, 'init_pose': [-22.0, -4.0, 0.0], 'target_offset': [-6.0, 6.0, 0.0]},
        {'id': 3, 'init_pose': [-10.0, -16.0, 0.0], 'target_offset': [6.0, -6.0, 0.0]},
        {'id': 4, 'init_pose': [-10.0, -4.0, 0.0], 'target_offset': [6.0, 6.0, 0.0]},
    ]

    obstacle_vessels = [
        {'name': 'vessel_e1', 'type': 'e', 'init_pose': [18.0, -24.0, 0.2, 1.5708], 'velocity': [0.0, 1.0], 'target': [132.0, 30.0, 0.0]},
        {'name': 'vessel_f1', 'type': 'f', 'init_pose': [58.0, 18.0, 0.2, 3.14159], 'velocity': [-0.90, 0.0], 'target': [12.0, -20.0, 0.0]},
        {'name': 'vessel_e2', 'type': 'e', 'init_pose': [42.0, -26.0, 0.2, 1.5708], 'velocity': [0.0, 1.0], 'target': [134.0, 34.0, 0.0]},
        {'name': 'vessel_f2', 'type': 'f', 'init_pose': [76.0, 26.0, 0.2, 3.14159], 'velocity': [-0.85, 0.0], 'target': [18.0, -24.0, 0.0]},
        {'name': 'vessel_e3', 'type': 'e', 'init_pose': [68.0, -28.0, 0.2, 1.5708], 'velocity': [0.0, 0.95], 'target': [138.0, 18.0, 0.0]},
        {'name': 'vessel_f3', 'type': 'f', 'init_pose': [104.0, -8.0, 0.2, 3.14159], 'velocity': [-0.90, 0.0], 'target': [26.0, 22.0, 0.0]},
    ]
    dynamic_obstacle_topics = [f"/{vessel['name']}/odom" for vessel in obstacle_vessels]
    dynamic_obstacle_names = [vessel['name'] for vessel in obstacle_vessels]
    dynamic_obstacle_radii = [3.2 for _ in obstacle_vessels]

    launch_entities = [
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_models', default_value='true'),
    ]

    pkg_model = get_package_share_directory('model')
    heron_xacro = os.path.join(pkg_model, 'urdf', 'heron_dave.urdf.xacro')

    for agent in agent_config:
        agent_id = agent['id']
        agent_ns = f'usv_{agent_id}'

        doc = xacro.process_file(heron_xacro, mappings={'namespace': agent_ns})
        robot_description = doc.toxml()

        rsp_ = Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', namespace=agent_ns, output='screen',
            parameters=[{'robot_description': robot_description}],
            remappings=[('tf', '/tf'), ('tf_static', '/tf_static')],
            condition=IfCondition(use_models),
        )

        usv_sim_ = Node(
            package='trajectory_smoother', executable='usv_simulator',
            name='usv_simulator', namespace=agent_ns, output='screen',
            parameters=[{'init_pose': agent['init_pose'], 'speed': 1.9, 'publish_rate': 20.0}],
        )

        prm_ = Node(
            package='topology_planner', executable='prm_node',
            name='prm_node', namespace=agent_ns, output='screen',
            parameters=[
                smoother_param_dir,
                guidance_param_dir,
                {'agent_id': agent_id, 'offset': agent['init_pose'], 'target_offset': agent['target_offset'], 'clock_frequency': 10.0},
            ],
        )

        smooth_ = Node(
            package='trajectory_smoother', executable='smoother_node',
            name='smoother_node', namespace=agent_ns, output='screen',
            parameters=[smoother_param_dir, {'agent_id': agent_id, 'num_usvs': 4}],
        )

        trajectory_visualizer_ = Node(
            package='trajectory_smoother', executable='online_trajectory_visualizer',
            name='online_trajectory_visualizer', namespace=agent_ns, output='screen',
            parameters=[smoother_param_dir, {
                'agent_id': agent_id,
                'frame_id': 'odom',
                'publish_rate': 10.0,
                'stale_timeout_sec': 1.0,
                'selected_hold_sec': 1.2,
                'dynamic_obstacle_topics': dynamic_obstacle_topics,
                'dynamic_obstacle_names': dynamic_obstacle_names,
                'dynamic_obstacle_radii': dynamic_obstacle_radii,
            }],
        )

        perception_ = Node(
            package='trajectory_smoother', executable='perception_node',
            name='perception_node', namespace=agent_ns, output='screen',
            parameters=[smoother_param_dir, {'offset': agent['init_pose']}],
        )

        launch_entities.extend([rsp_, usv_sim_, prm_, smooth_, trajectory_visualizer_, perception_])

    for vessel in obstacle_vessels:
        vessel_motion = Node(
            package='trajectory_smoother', executable='kinematic_vessel_node',
            name=f"{vessel['name']}_motion", output='screen',
            parameters=[{
                'world_name': 'dave_ocean_waves_fixed',
                'model_name': vessel['name'],
                'odom_topic': f"/{vessel['name']}/odom",
                'child_frame_id': f"{vessel['name']}/base_link",
                'initial_pose': vessel['init_pose'],
                'linear_velocity': vessel['velocity'],
                'target': vessel['target'],
                'static_obstacle_x': [30.0, 48.0, 65.0, 78.0, 92.0, 108.0, 120.0, 55.0, 85.0, 100.0],
                'static_obstacle_y': [18.0, -10.0, 8.0, 22.0, -6.0, 14.0, -16.0, 34.0, -18.0, 32.0],
                'static_obstacle_radii': [5.0, 4.5, 5.5, 4.0, 5.0, 4.5, 4.0, 5.0, 4.5, 4.5],
                'update_rate': 10.0,
            }],
        )
        launch_entities.append(vessel_motion)

        vessel_xacro = os.path.join(pkg_model, 'urdf', f"vessel_{vessel['type']}.urdf.xacro")
        vessel_doc = xacro.process_file(vessel_xacro, mappings={'namespace': vessel['name']})
        vessel_description = vessel_doc.toxml()
        vessel_rsp = Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name=f"{vessel['name']}_state_publisher", output='screen',
            parameters=[{'robot_description': vessel_description}],
            remappings=[('tf', '/tf'), ('tf_static', '/tf_static')],
            condition=IfCondition(use_models),
        )
        launch_entities.append(vessel_rsp)

        vessel_tf = Node(
            package='trajectory_smoother', executable='odom_tf_broadcaster',
            name='odom_tf_broadcaster', namespace=vessel['name'], output='screen',
            parameters=[{'publish_rate': 10.0}],
            condition=IfCondition(use_models),
        )
        launch_entities.append(vessel_tf)

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2_planner',
        arguments=['-d', rviz_config], output='screen',
        condition=IfCondition(use_rviz),
    )
    launch_entities.append(rviz_node)
    return LaunchDescription(launch_entities)
