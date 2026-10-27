"""
Corridor experiment launch file — multi-USV, bounded corridor, dynamic vessels.

Scenario:
  - 3 Heron USVs navigating a straight corridor (y ∈ [-20, 20])
  - Two black boundary walls prevent TEB-style cheating
  - Multiple Vessel_e/f moving back-and-forth across the corridor (y-direction)
  - USVs start at x=-10, spread across y, goal at x=50
  - Pass threshold: x > 25 (USV clears the obstacle zone)

Difficulty modes (set via MULTI_USV_DEBRIS_COUNT env):
  4 → easy   (4 vessels)
  6 → medium (6 vessels)
  8 → hard   (8 vessels)

Usage:
  # Visualisation
  ros2 launch multi_usv_planner planner_corridor.launch.py

  # Headless batch run
  MULTI_USV_DEBRIS_COUNT=6 MULTI_USV_SEED=42 \
    ros2 launch multi_usv_planner planner_corridor.launch.py use_rviz:=false
"""

import json
import os
import random
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, TextSubstitution, PythonExpression
from launch_ros.actions import Node


def _jitter_pose(rng, pose, xy=1.5):
    jittered = list(pose)
    if len(jittered) >= 1:
        jittered[0] = pose[0] + rng.uniform(-xy, xy)
    if len(jittered) >= 2:
        jittered[1] = pose[1] + rng.uniform(-xy, xy)
    return jittered


def _jitter_speed(rng, base, s=0.05):
    return base + rng.uniform(-s, s)


def generate_launch_description():
    acados_source = '/home/lu/.local/share/acados'
    planner_share = get_package_share_directory('multi_usv_planner')
    model_share = get_package_share_directory('model')
    guidance_share = get_package_share_directory('guidance_planner')

    rviz_config = os.path.join(planner_share, 'rviz', 'corridor.rviz')
    guidance_config = os.path.join(guidance_share, 'config', 'ros2_params.yaml')
    planner_config_default = os.path.join(planner_share, 'config', 'usv_params.yaml')
    planner_config = os.environ.get('MULTI_USV_PLANNER_CONFIG', planner_config_default)

    use_rviz = LaunchConfiguration('use_rviz')
    use_models = LaunchConfiguration('use_models')
    use_guidance = LaunchConfiguration('use_guidance')
    seed_value = int(os.environ.get('MULTI_USV_SEED', '0'))
    num_vessels = int(os.environ.get('MULTI_USV_DEBRIS_COUNT', '4'))
    num_agents = int(os.environ.get('MULTI_USV_AGENTS', '3'))
    obstacle_aware_sampling = os.environ.get('MULTI_USV_OBS_AWARE', '1') == '1'
    vprm_no_neighbor = os.environ.get('MULTI_USV_VPRM_NO_NEIGHBOR', '0') == '1'
    colregs_enabled = os.environ.get('MULTI_USV_COLREGS', '1') == '1'
    enable_rerank = os.environ.get('MULTI_USV_ENABLE_RERANK', '1') == '1'

    # ── Difficulty-dependent max obstacles ──
    max_obs = min(num_vessels + 2, 14)

    rng = random.Random(seed_value)

    # ── Multi-USV configuration ──
    # 3 USVs spread across y, all heading to x=50 at different y offsets
    usv_configs = [
        {'id': 1, 'init_pose': [-10.0, -6.0, 0.0], 'goal': [80.0, -6.0], 'color': [1.0, 0.3, 0.3]},
        {'id': 2, 'init_pose': [-10.0,  0.0, 0.0], 'goal': [80.0,  0.0], 'color': [0.3, 1.0, 0.3]},
        {'id': 3, 'init_pose': [-10.0,  6.0, 0.0], 'goal': [80.0,  6.0], 'color': [0.3, 0.3, 1.0]},
    ]
    active_ids = [c['id'] for c in usv_configs]

    # ── Corridor vessel generation ──
    CORRIDOR_Y_MIN = -18.0
    CORRIDOR_Y_MAX = 18.0
    WALL_STATIC_R = 0.0  # walls enforced via acados lbx/ubx, not as soft obstacles
    VESSEL_Y_RANGE = 14.0  # vessel 活动范围缩小到 ±14，保持对 USV 的干扰
    VESSEL_BASE_SPEED = 0.65
    VESSEL_RADIUS = 4.0  # 4m 直径，对应视觉显示尺寸

    vessel_templates = []
    if num_vessels >= 1:
        vessel_templates.append({'name': 'vessel_e1', 'type': 'e', 'x': -2.0, 'start_side': 'bottom'})
    if num_vessels >= 2:
        vessel_templates.append({'name': 'vessel_e2', 'type': 'e', 'x': 6.0, 'start_side': 'top'})
    if num_vessels >= 3:
        vessel_templates.append({'name': 'vessel_f1', 'type': 'f', 'x': 14.0, 'start_side': 'bottom'})
    if num_vessels >= 4:
        vessel_templates.append({'name': 'vessel_f2', 'type': 'f', 'x': 22.0, 'start_side': 'top'})
    if num_vessels >= 5:
        vessel_templates.append({'name': 'vessel_e3', 'type': 'e', 'x': 2.0, 'start_side': 'bottom'})
    if num_vessels >= 6:
        vessel_templates.append({'name': 'vessel_f3', 'type': 'f', 'x': 18.0, 'start_side': 'top'})
    if num_vessels >= 7:
        vessel_templates.append({'name': 'vessel_e4', 'type': 'e', 'x': -4.0, 'start_side': 'bottom'})
    if num_vessels >= 8:
        vessel_templates.append({'name': 'vessel_f4', 'type': 'f', 'x': 26.0, 'start_side': 'top'})
    if num_vessels >= 9:
        vessel_templates.append({'name': 'vessel_e5', 'type': 'e', 'x': -6.0, 'start_side': 'bottom'})
    if num_vessels >= 10:
        vessel_templates.append({'name': 'vessel_f5', 'type': 'f', 'x': 30.0, 'start_side': 'top'})
    if num_vessels >= 11:
        vessel_templates.append({'name': 'vessel_e6', 'type': 'e', 'x': 10.0, 'start_side': 'bottom'})
    if num_vessels >= 12:
        vessel_templates.append({'name': 'vessel_f6', 'type': 'f', 'x': 34.0, 'start_side': 'top'})

    obstacle_vessels = []
    for vt in vessel_templates:
        if vt['start_side'] == 'bottom':
            y_start = -VESSEL_Y_RANGE
            y_end = VESSEL_Y_RANGE
        else:
            y_start = VESSEL_Y_RANGE
            y_end = -VESSEL_Y_RANGE

        speed = _jitter_speed(rng, VESSEL_BASE_SPEED)

        mesh_path = (
            f"file:///home/lu/paper2/core_ws/src/model/models/vessel_{vt['type']}/meshes/Boat05.dae"
            if vt['type'] == 'e' else
            f"file:///home/lu/paper2/core_ws/src/model/models/vessel_{vt['type']}/meshes/Boat06.dae"
        )

        obstacle_vessels.append({
            'name': vt['name'],
            'type': vt['type'],
            'radius': VESSEL_RADIUS,
            'init_pose': _jitter_pose(rng, [vt['x'], y_start, 0.2, 0.0], 0.3),
            'velocity': [0.0, speed if vt['start_side'] == 'bottom' else -speed],
            'path': [[vt['x'], y_start], [vt['x'], y_end]],
            'loop': True,
            'mesh_resource': mesh_path,
        })

    # ── Launch entities ──
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
    ]

    # map → odom static transform (only one needed)
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

        # Heron robot model
        doc = xacro.process_file(heron_xacro, mappings={'namespace': ns})
        robot_desc = doc.toxml()

        launch_entities.append(Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', namespace=ns, output='screen',
            parameters=[{'robot_description': robot_desc}],
            remappings=[('tf', '/tf'), ('tf_static', '/tf_static')],
            condition=IfCondition(use_models),
        ))

        # USV motion simulator
        launch_entities.append(Node(
            package='trajectory_smoother', executable='usv_simulator',
            name='usv_simulator', namespace=ns, output='screen',
            parameters=[{'init_pose': usv['init_pose'], 'speed': 1.35, 'publish_rate': 20.0}],
        ))

        # PRM guidance path
        launch_entities.append(Node(
            package='topology_planner', executable='prm_node',
            name='prm_node', namespace=ns, output='screen',
            parameters=[guidance_config, {
                'agent_id': uid,
                'offset': usv['init_pose'],
                'target_offset': [0.0, 0.0, 0.0],
                'target': [usv['goal'][0], usv['goal'][1], 0.0],
                'clock_frequency': 3.0,
                'enable_visualization': True,
                'enable_info_log': False,
                'enable_debug_report': False,
                'exit_clearance_margin': 2.5,
                'exit_hold_count': 10,
                'static_obstacles_x': [0.0],
                'static_obstacles_y': [0.0],
                'static_obstacles_r': [0.0],
                'corridor_y_min': CORRIDOR_Y_MIN,
                'corridor_y_max': CORRIDOR_Y_MAX,
            }] + ([{
                'dynamic_obstacle_names': [v['name'] for v in obstacle_vessels] + ([f'neighbor_{n}' for n in active_ids if n != uid] if not vprm_no_neighbor else []),
                'dynamic_obstacle_radii': [v['radius'] + 1.0 for v in obstacle_vessels] + ([3.0 for n in active_ids if n != uid] if not vprm_no_neighbor else []),
                'dynamic_obstacle_topics': [f'/{v["name"]}/odom' for v in obstacle_vessels] + ([f'/usv_{n}/odom' for n in active_ids if n != uid] if not vprm_no_neighbor else []),
            }] if obstacle_aware_sampling else []),
            condition=IfCondition(use_guidance),
        ))

        # USV MPC planner
        launch_entities.append(Node(
            package='multi_usv_planner', executable='usv_planner_node_exe',
            name='usv_planner_node', namespace=ns, output='screen',
            parameters=[planner_config, {
                'agent_id': uid,
                'use_selected_reference': True,
                'use_original_like_reference_mode': True,
                'enable_topology_rerank': enable_rerank,
                'neighbors': [i for i in active_ids if i != uid],
                'goal.x': usv['goal'][0],
                'goal.y': usv['goal'][1],
                'static_obstacles_x': [0.0],
                'static_obstacles_y': [0.0],
                'static_obstacles_r': [0.0],
                'pass_x_threshold': 30.0,
                'episode_timeout_s': 120.0,
                'colregs_enabled': colregs_enabled,
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
    # Dynamic obstacle vessels
    # ═══════════════════════════════════════════════════
    launch_entities.append(Node(
        package='multi_usv_planner', executable='dynamic_obstacle_node.py',
        name='dynamic_obstacle_node', output='screen',
        parameters=[{
            'update_rate': 10.0,
            'corridor_y_min': CORRIDOR_Y_MIN,
            'corridor_y_max': CORRIDOR_Y_MAX,
            'vessels': json.dumps([
                {
                    'name': v['name'],
                    'radius': v['radius'],
                    'init_pose': list(v['init_pose']),
                    'velocity': list(v['velocity']),
                    'path': v['path'],
                    'loop': v['loop'],
                    'mesh_resource': v.get('mesh_resource', ''),
                }
                for v in obstacle_vessels
            ]),
        }],
    ))

    # ── Vessel robot model publishers ──
    for vessel in obstacle_vessels:
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

    # ═══════════════════════════════════════════════════
    # Scene visualizer (walls + start/goal)
    # ═══════════════════════════════════════════════════
    launch_entities.append(Node(
        package='multi_usv_planner', executable='scene_visualizer_node',
        name='scene_visualizer', output='screen',
    ))

    # ═══════════════════════════════════════════════════
    # RViz
    # ═══════════════════════════════════════════════════
    launch_entities.append(Node(
        package='rviz2', executable='rviz2', name='rviz2_planner',
        arguments=['-d', rviz_config], output='screen',
        condition=IfCondition(use_rviz),
    ))

    # NOTE: exit_monitor and collision_monitor are no longer embedded in the
    # launch file. run_exp.py starts them as standalone subprocesses
    # (experiments_v2/shared/exit_monitor.py, shared/collision_monitor.py).
    # This keeps the timing/logic code out of the workspace entirely.

    return LaunchDescription(launch_entities)
