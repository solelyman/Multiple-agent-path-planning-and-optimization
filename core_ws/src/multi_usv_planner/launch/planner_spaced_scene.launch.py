"""
Spaced-scene launch — old-repo scenario stretched longitudinally.

Layout (old-repo style, goal pulled far right):
  - 4 Heron USVs start at x≈-38..-14, goals at x=110
  - 4 dynamic vessels (e1,e2,f1,f2) concentrated near the entrance (x≈-2..10)
  - 4 static obstacles at old-repo positions
  - Deep stretch: obstacle zone remains near entry, goal at x=110
"""
import json
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

    rviz_config = os.path.join(planner_share, 'rviz', 'spaced_scene.rviz')
    guidance_config = os.path.join(guidance_share, 'config', 'ros2_params.yaml')
    planner_config = os.path.join(planner_share, 'config', 'usv_params.yaml')

    use_rviz = LaunchConfiguration('use_rviz')
    use_models = LaunchConfiguration('use_models')
    use_guidance = LaunchConfiguration('use_guidance')
    enable_topology_rerank = LaunchConfiguration('enable_topology_rerank')
    seed_value = int(os.environ.get('MULTI_USV_SEED', '17'))
    num_agents = int(os.environ.get('MULTI_USV_AGENTS', '4'))
    rng = random.Random(seed_value)

    # ── Old-repo-style USV configs, stretched ─────────────────────
    all_usv_configs = [
        {'id': 1, 'init_pose': [-30.0, -10.5, 0.0], 'target_offset': [-8.0, 8.0, 0.0],
         'color': [1.0, 0.3, 0.3], 'goal': [110.0, 2.0]},
        {'id': 3, 'init_pose': [-38.0, -10.5, 0.0], 'target_offset': [12.0, 8.0, 0.0],
         'color': [0.3, 0.3, 1.0], 'goal': [110.0, 6.0]},
        {'id': 2, 'init_pose': [-22.0, -10.5, 0.0], 'target_offset': [3.0, 8.0, 0.0],
         'color': [0.3, 1.0, 0.3], 'goal': [110.0, 14.0]},
        {'id': 4, 'init_pose': [-14.0, -10.5, 0.0], 'target_offset': [18.0, 8.0, 0.0],
         'color': [1.0, 0.7, 0.2], 'goal': [110.0, 22.0]},
    ]
    usv_configs = all_usv_configs[:num_agents]
    active_ids = [c['id'] for c in usv_configs]

    # ── Old-repo-style 4 dynamic vessels, concentrated near entrance ──
    obstacle_vessels = [
        {'name': 'vessel_e1', 'type': 'e', 'radius': 2.2,
         'init_pose': _jitter_pose(rng, [8.0, -12.0, 0.2, 1.5708], 0.20),
         'velocity': _jitter_velocity(rng, [0.0, 0.42], 0.02),
         'target': [8.0, 10.0, 0.0]},
        {'name': 'vessel_e2', 'type': 'e', 'radius': 2.2,
         'init_pose': _jitter_pose(rng, [24.0, -12.0, 0.2, 1.5708], 0.20),
         'velocity': _jitter_velocity(rng, [0.0, 0.42], 0.02),
         'target': [24.0, 10.0, 0.0]},
        {'name': 'vessel_e3', 'type': 'e', 'radius': 2.2,
         'init_pose': _jitter_pose(rng, [46.0, -12.0, 0.2, 1.5708], 0.20),
         'velocity': _jitter_velocity(rng, [0.0, 0.40], 0.02),
         'target': [46.0, 10.0, 0.0]},
        {'name': 'vessel_f1', 'type': 'f', 'radius': 2.4,
         'init_pose': _jitter_pose(rng, [16.0, 10.0, 0.2, -1.5708], 0.20),
         'velocity': _jitter_velocity(rng, [0.0, -0.40], 0.02),
         'target': [16.0, -12.0, 0.0]},
        {'name': 'vessel_f2', 'type': 'f', 'radius': 2.4,
         'init_pose': _jitter_pose(rng, [34.0, 10.0, 0.2, -1.5708], 0.20),
         'velocity': _jitter_velocity(rng, [0.0, -0.40], 0.02),
         'target': [34.0, -12.0, 0.0]},
        {'name': 'vessel_f3', 'type': 'f', 'radius': 2.4,
         'init_pose': _jitter_pose(rng, [56.0, 10.0, 0.2, -1.5708], 0.20),
         'velocity': _jitter_velocity(rng, [0.0, -0.38], 0.02),
         'target': [56.0, -12.0, 0.0]},
    ]

    # ── Old-repo static obstacles, but stretched in x to open the passage ──
    static_obs_x = [14.0, 34.0, 22.0, 42.0]
    static_obs_y = [-11.5, -2.5, 4.5, 12.0]
    static_obs_r = [0.75, 0.85, 0.68, 0.75]

    max_obs = 10  # 6 vessels + up to 3 neighbors + margin

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
        DeclareLaunchArgument('enable_topology_rerank', default_value='true'),
        DeclareLaunchArgument('debris_mesh', default_value='file://' + os.path.join(model_share, 'models', 'meshes', 'ultimate_realistic_wooden_crates_pack.glb')),
    ]

    # map -> odom static transform
    launch_entities.append(Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='map_to_odom', output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        condition=IfCondition(use_models),
    ))

    heron_xacro = os.path.join(model_share, 'urdf', 'heron_dave.urdf.xacro')

    # ═══════════════════════════════════════════════════
    # Each Heron USV
    # ═══════════════════════════════════════════════════
    for usv in usv_configs:
        uid = usv['id']
        ns = f'usv_{uid}'

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
            package='trajectory_smoother', executable='usv_simulator',
            name='usv_simulator', namespace=ns, output='screen',
            parameters=[{
                'init_pose': usv['init_pose'],
                'speed': 1.35,
                'publish_rate': 20.0,
            }],
        ))

        # PRM guidance path
        launch_entities.append(Node(
            package='topology_planner', executable='prm_node',
            name='prm_node', namespace=ns, output='screen',
            parameters=[guidance_config, {
                'agent_id': uid,
                'offset': usv['init_pose'],
                'target_offset': usv['target_offset'],
                'target': [usv['goal'][0], usv['goal'][1], 0.0],
                'clock_frequency': 3.0,
                'enable_visualization': True,
                'enable_info_log': False,
                'enable_debug_report': False,
                'exit_clearance_margin': 2.5,
                'exit_hold_count': 10,
                'static_obstacles_x': static_obs_x,
                'static_obstacles_y': static_obs_y,
                'static_obstacles_r': static_obs_r,
                'dynamic_obstacle_names': [v['name'] for v in obstacle_vessels] + [f'neighbor_{n}' for n in active_ids if n != uid],
                'dynamic_obstacle_radii': [v['radius'] + 1.0 for v in obstacle_vessels] + [3.0 for n in active_ids if n != uid],
                'dynamic_obstacle_topics': [f'/{v["name"]}/odom' for v in obstacle_vessels] + [f'/usv_{n}/odom' for n in active_ids if n != uid],
            }],
            condition=IfCondition(use_guidance),
        ))

        # USV MPC planner — use corridor-style high-performance params
        launch_entities.append(Node(
            package='multi_usv_planner', executable='usv_planner_node_exe',
            name='usv_planner_node', namespace=ns, output='screen',
            parameters=[planner_config, {
                'agent_id': uid,
                'use_selected_reference': True,
                'use_original_like_reference_mode': True,
                'enable_topology_rerank': enable_topology_rerank,
                'neighbors': [i for i in active_ids if i != uid],
                'goal.x': usv['goal'][0],
                'goal.y': usv['goal'][1],
                'static_obstacles_x': static_obs_x,
                'static_obstacles_y': static_obs_y,
                'static_obstacles_r': static_obs_r,
                'pass_x_threshold': 20.0,
                'episode_timeout_s': 150.0,
                'colregs_enabled': True,
                'mpc.N': 6,
                'mpc.dt': 0.5,
                'mpc.max_vel': 1.9,
                'mpc.max_acc': 3.0,
                'mpc.desired_speed': 1.9,
                'mpc.acados_max_obstacles': max_obs,
                'mpc.max_iter': 8,
                'mpc.weight_goal': 15.0,
                'mpc.weight_obstacle': 8.0,
                'mpc.weight_neighbor': 8.0,
                'mpc.weight_progress': 15.0,
                'mpc.weight_backtrack': 60.0,
                'mpc.min_progress_step': 0.25,
                'mpc.obstacle_clearance': 1.2,
                'mpc.dynamic_obstacle_growth': 0.02,
                'mpc.dynamic_speed_dist_threshold': 2.0,
                'mpc.dynamic_speed_scale_denom': 50.0,
                'mpc.topology_violation_obstacle_weight': 10.0,
                'mpc.topology_violation_neighbor_weight': 10.0,
                'mpc.topology_soft_obstacle_weight': 2.0,
                'mpc.topology_soft_neighbor_weight': 3.0,
                'mpc.topology_goal_progress_weight': 0.5,
                'mpc.topology_soft_neighbor_threshold': 2.0,
                'mpc.topology_soft_obstacle_threshold': 1.5,
                'mpc.topology_curvature_threshold': 3.0,
                'mpc.topology_curvature_weight': 2.0,
                'mpc.acados_weight_velocity': 8.0,
                'mpc.acados_weight_acc': 0.01,
                'mpc.acados_weight_angvel': 0.10,
                'mpc.acados_weight_contour': 0.15,
                'mpc.acados_weight_lag': 0.30,
            }],
        ))

    # ═══════════════════════════════════════════════════
    # Dynamic obstacle vessels (old-repo style: simple target-seeking)
    # ═══════════════════════════════════════════════════
    launch_entities.append(Node(
        package='multi_usv_planner', executable='dynamic_obstacle_node.py',
        name='dynamic_obstacle_node', output='screen',
        parameters=[{
            'update_rate': 10.0,
            'marker_topic': '/spaced_scene/dynamic_vessels',
            'publish_corridor_walls': False,
            'vessels': json.dumps([
                {
                    'name': v['name'],
                    'radius': v['radius'],
                    'init_pose': list(v['init_pose']),
                    'velocity': list(v['velocity']),
                    'target': list(v['target']),
                    'mesh_resource': 'file://' + os.path.join(model_share, 'models', 'vessel_e' if v['type'] == 'e' else 'vessel_f', 'meshes', 'Boat05.dae' if v['type'] == 'e' else 'Boat06.dae'),
                }
                for v in obstacle_vessels
            ]),
        }],
    ))

    # Vessel robot model publishers
    for vessel in obstacle_vessels:
        vessel_xacro = os.path.join(model_share, 'urdf', f"vessel_{vessel['type']}.urdf.xacro")
        vessel_doc = xacro.process_file(vessel_xacro, mappings={'namespace': vessel['name']})
        vessel_description = vessel_doc.toxml()
        launch_entities.append(Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', namespace=vessel['name'], output='screen',
            parameters=[{'robot_description': vessel_description}],
            remappings=[('tf', '/tf'), ('tf_static', '/tf_static')],
            condition=IfCondition(use_models),
        ))
        launch_entities.append(Node(
            package='trajectory_smoother', executable='odom_tf_broadcaster',
            name='odom_tf_broadcaster', namespace=vessel['name'], output='screen',
            parameters=[{'publish_rate': 10.0, 'parent_frame': 'odom'}],
            condition=IfCondition(use_models),
        ))

    # ═══════════════════════════════════════════════════
    # Floating debris — recover old-scene visual density
    launch_entities.append(Node(
        package='multi_usv_planner', executable='floating_debris_node.py',
        name='floating_debris_node', output='screen',
        parameters=[{
            'num_debris': 8,
            'update_rate': 10.0,
            'seed': seed_value,
            'frame_id': 'odom',
            'marker_topic': '/spaced_scene/debris_models',
            'mesh_resource': 'file://' + os.path.join(model_share, 'models', 'meshes', 'ultimate_realistic_wooden_crates_pack.glb'),
            'mesh_scale_x': 0.82,
            'mesh_scale_y': 0.52,
            'mesh_scale_z': 0.22,
        }],
    ))

    # Scene markers (Python node, parameterized for this scene)
    launch_entities.append(Node(
        package='multi_usv_planner', executable='spaced_scene_markers.py',
        name='spaced_scene_markers', output='screen',
        parameters=[{
            'usv_configs': json.dumps([
                {'init_pose': u['init_pose'], 'goal': u['goal']}
                for u in usv_configs
            ]),
            'static_obstacles_x': static_obs_x,
            'static_obstacles_y': static_obs_y,
            'static_obstacles_r': static_obs_r,
            'marker_topic': '/spaced_scene/scene_markers',
            'static_mesh_resource': 'file://' + os.path.join(model_share, 'models', 'meshes', 'ice_floes.glb'),
            'use_static_mesh': False,
            'publish_rate': 2.0,
        }],
    ))

    # ═══════════════════════════════════════════════════
    # RViz
    # ═══════════════════════════════════════════════════
    launch_entities.append(Node(
        package='rviz2', executable='rviz2', name='rviz2_scene',
        arguments=['-d', rviz_config], output='screen',
        condition=IfCondition(use_rviz),
    ))

    return LaunchDescription(launch_entities)
