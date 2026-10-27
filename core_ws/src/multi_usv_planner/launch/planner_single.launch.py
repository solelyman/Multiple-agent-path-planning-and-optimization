import os
import random
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, TextSubstitution
from launch_ros.actions import Node


def _jitter_pose(rng, pose, xy=1.0):
    jittered = list(pose)
    if len(jittered) >= 1:
        jittered[0] = pose[0] + rng.uniform(-xy, xy)
    if len(jittered) >= 2:
        jittered[1] = pose[1] + rng.uniform(-xy, xy)
    return jittered


def _jitter_velocity(rng, vel, s=0.12):
    return [vel[0] + rng.uniform(-s, s), vel[1] + rng.uniform(-s, s)]


def generate_launch_description():
    acados_source = '/home/lu/.local/share/acados'
    planner_share = get_package_share_directory('multi_usv_planner')
    model_share = get_package_share_directory('model')
    guidance_share = get_package_share_directory('guidance_planner')

    rviz_config = LaunchConfiguration('rviz_config')
    guidance_config = os.path.join(guidance_share, 'config', 'ros2_params.yaml')
    planner_config = os.path.join(planner_share, 'config', 'usv_params.yaml')

    use_rviz = LaunchConfiguration('use_rviz')
    use_models = LaunchConfiguration('use_models')
    use_guidance = LaunchConfiguration('use_guidance')
    use_topology_selector = LaunchConfiguration('use_topology_selector')
    ros_domain_id = LaunchConfiguration('ros_domain_id')
    seed_value = int(os.environ.get('MULTI_USV_SEED', '0'))
    rng = random.Random(seed_value)

    usv = {
        'id': 1,
        'init_pose': [-24.0, -14.0, 0.0],
        'target_offset': [-4.0, -3.0, 0.0],
        'goal': [58.0, 8.0],
    }

    obstacle_vessels = [
        {'name': 'vessel_e1', 'type': 'e', 'radius': 3.0, 'init_pose': _jitter_pose(rng, [4.0, -8.2, 0.2, 1.5708], 0.20),
         'velocity': _jitter_velocity(rng, [0.0, 0.92], 0.02), 'target': [4.0, 9.5, 0.0]},
        {'name': 'vessel_e2', 'type': 'e', 'radius': 3.0, 'init_pose': _jitter_pose(rng, [16.0, -7.6, 0.2, 1.5708], 0.20),
         'velocity': _jitter_velocity(rng, [0.0, 0.94], 0.02), 'target': [16.0, 10.5, 0.0]},
        {'name': 'vessel_f1', 'type': 'f', 'radius': 3.5, 'init_pose': _jitter_pose(rng, [10.0, 11.2, 0.2, -1.5708], 0.20),
         'velocity': _jitter_velocity(rng, [0.0, -0.88], 0.02), 'target': [10.0, -6.0, 0.0]},
        {'name': 'vessel_f2', 'type': 'f', 'radius': 3.5, 'init_pose': _jitter_pose(rng, [22.0, 12.0, 0.2, -1.5708], 0.20),
         'velocity': _jitter_velocity(rng, [0.0, -0.86], 0.02), 'target': [22.0, -5.5, 0.0]},
    ]

    launch_entities = [
        SetEnvironmentVariable('ACADOS_SOURCE_DIR', acados_source),
        SetEnvironmentVariable('ACADOS_PYTHON', f'{acados_source}/venv/bin/python'),
        SetEnvironmentVariable(
            'LD_LIBRARY_PATH',
            [
                TextSubstitution(text=f'{acados_source}/lib:'),
                TextSubstitution(text=f'{acados_source}/build/acados:'),
                TextSubstitution(text=f'{acados_source}/build/external/hpipm:'),
                TextSubstitution(text=f'{acados_source}/build/external/blasfeo:'),
                EnvironmentVariable('LD_LIBRARY_PATH', default_value=''),
            ],
        ),
        SetEnvironmentVariable(
            'PYTHONPATH',
            [
                TextSubstitution(text=f'{acados_source}/interfaces/acados_template:'),
                EnvironmentVariable('PYTHONPATH', default_value=''),
            ],
        ),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_models', default_value='true'),
        DeclareLaunchArgument('use_guidance', default_value='true'),
        DeclareLaunchArgument('use_topology_selector', default_value='false'),
        DeclareLaunchArgument('ros_domain_id', default_value='43'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', ros_domain_id),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='/home/lu/paper2/mpc_planner-main/mpc_planner_dingo/rviz/ros2.rviz'),
    ]

    heron_xacro = os.path.join(model_share, 'urdf', 'heron_dave.urdf.xacro')
    ns = 'usv_1'
    doc = xacro.process_file(heron_xacro, mappings={'namespace': ns})
    robot_desc = doc.toxml()

    launch_entities.append(Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', namespace=ns, output='screen',
        parameters=[{'robot_description': robot_desc}],
        remappings=[('tf', '/tf'), ('tf_static', '/tf_static')],
        condition=IfCondition(use_models),
    ))

    launch_entities.append(Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='map_to_odom', output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        condition=IfCondition(use_models),
    ))

    launch_entities.append(Node(
        package='trajectory_smoother', executable='usv_simulator',
        name='usv_simulator', namespace=ns, output='screen',
        parameters=[{
            'init_pose': usv['init_pose'],
            'speed': 1.35,
            'publish_rate': 20.0,
        }],
    ))

    launch_entities.append(Node(
        package='topology_planner', executable='prm_node',
        name='prm_node', namespace=ns, output='screen',
        parameters=[guidance_config, {
            'agent_id': usv['id'],
            'offset': usv['init_pose'],
            'target_offset': usv['target_offset'],
            'target': [usv['goal'][0], usv['goal'][1], 0.0],
            'clock_frequency': 10.0,
            'enable_visualization': True,
            'enable_info_log': False,
            'enable_debug_report': False,
            'exit_clearance_margin': 2.5,
            'exit_hold_count': 10,
            'static_obstacles_x': [0.0],
            'static_obstacles_y': [0.0],
            'static_obstacles_r': [0.0],
        }],
        condition=IfCondition(use_guidance),
    ))

    launch_entities.append(Node(
        package='multi_usv_planner', executable='usv_planner_node_exe',
        name='usv_planner_node', namespace=ns, output='screen',
        parameters=[planner_config, {
            'agent_id': usv['id'],
            'use_selected_reference': True,
            'use_original_like_reference_mode': True,
            'neighbors': [0],
            'goal.x': usv['goal'][0],
            'goal.y': usv['goal'][1],
        }],
    ))

    launch_entities.append(Node(
        package='multi_usv_planner', executable='topology_selector_node',
        name='topology_selector_node', namespace=ns, output='screen',
        parameters=[{
            'agent_id': usv['id'],
            'neighbors': [0],
            'selection_hold_sec': 1.5,
            'consistency_penalty': 10.0,
            'topology_switch_penalty': 8.0,
            'weight_topology_length': 0.2,
            'goal_x': usv['goal'][0],
            'goal_y': usv['goal'][1],
            'local_goal_horizon': 10.0,
            'weight_global_consistency': 25.0,
            'weight_initial_turn': 20.0,
            'obstacle_safe_margin': 4.0,
            'static_obstacles_x': [0.0],
            'static_obstacles_y': [0.0],
            'static_obstacles_r': [0.0],
        }],
        condition=IfCondition(use_topology_selector),
    ))

    launch_entities.append(Node(
        package='trajectory_smoother', executable='online_trajectory_visualizer',
        name='online_trajectory_visualizer', namespace=ns, output='screen',
        parameters=[guidance_config, {
            'agent_id': usv['id'],
            'target': [usv['goal'][0], usv['goal'][1], 0.0],
        }],
        condition=IfCondition(use_guidance),
    ))

    for vessel in obstacle_vessels:
        launch_entities.append(Node(
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
                'static_obstacle_x': [0.0],
                'static_obstacle_y': [0.0],
                'static_obstacle_radii': [0.0],
                'update_rate': 10.0,
            }],
        ))

        vessel_xacro_path = os.path.join(model_share, 'urdf', f"vessel_{vessel['type']}.urdf.xacro")
        vessel_doc = xacro.process_file(vessel_xacro_path, mappings={'namespace': vessel['name']})
        vessel_desc = vessel_doc.toxml()

        launch_entities.append(Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', namespace=vessel['name'], output='screen',
            parameters=[{'robot_description': vessel_desc}],
            remappings=[('tf', '/tf'), ('tf_static', '/tf_static')],
            condition=IfCondition(use_models),
        ))

        launch_entities.append(Node(
            package='trajectory_smoother', executable='odom_tf_broadcaster',
            name='odom_tf_broadcaster', namespace=vessel['name'], output='screen',
            parameters=[{'publish_rate': 10.0}],
            condition=IfCondition(use_models),
        ))

    launch_entities.append(Node(
        package='multi_usv_planner', executable='scene_visualizer_node',
        name='scene_visualizer', output='screen',
    ))

    launch_entities.append(Node(
        package='rviz2', executable='rviz2', name='rviz2_planner',
        arguments=['-d', rviz_config], output='screen',
        condition=IfCondition(use_rviz),
    ))

    return LaunchDescription(launch_entities)
