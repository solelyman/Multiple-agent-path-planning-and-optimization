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

    rviz_config = os.path.join(planner_share, 'rviz', 'planner.rviz')
    guidance_config = os.path.join(guidance_share, 'config', 'ros2_params.yaml')
    # 选择与 debris 数量匹配的参数文件
    num_debris = int(os.environ.get('MULTI_USV_DEBRIS_COUNT', '12'))
    if num_debris <= 4:
        profile = 'easy'
    elif num_debris <= 8:
        profile = 'medium'
    else:
        profile = 'hard'
    planner_config = os.path.join(planner_share, 'config', f'usv_params_{profile}.yaml')

    use_rviz = LaunchConfiguration('use_rviz')
    use_models = LaunchConfiguration('use_models')
    use_guidance = LaunchConfiguration('use_guidance')
    enable_topology_rerank = LaunchConfiguration('enable_topology_rerank')
    debris_mesh = LaunchConfiguration('debris_mesh')
    seed_value = int(os.environ.get('MULTI_USV_SEED', '0'))
    num_agents = int(os.environ.get('MULTI_USV_AGENTS', '4'))
    rng = random.Random(seed_value)

    # ── Heron USV 配置 ────────────────────
    # 目标点远离障碍物区域（障碍物集中在 x=7~31, y=-3.8~5.8）
    all_usv_configs = [
        {'id': 1, 'init_pose': [-24.0, -10.5, 0.0], 'target_offset': [-8.0, 7.0, 0.0],
         'color': [1.0, 0.3, 0.3], 'goal': [74.0, 2.0]},
        {'id': 3, 'init_pose': [-32.0, -10.5, 0.0], 'target_offset': [12.0, 7.0, 0.0],
         'color': [0.3, 0.3, 1.0], 'goal': [74.0, 6.0]},
        {'id': 2, 'init_pose': [-16.0, -10.5, 0.0], 'target_offset': [3.0, 7.0, 0.0],
         'color': [0.3, 1.0, 0.3], 'goal': [74.0, 14.0]},
    ]
    usv_configs = all_usv_configs[:num_agents]
    active_ids = [c['id'] for c in usv_configs]

    # ── 动态障碍船舶 ────────────────────────
    obstacle_vessels = [
        {'name': 'vessel_e1', 'type': 'e', 'radius': 3.0, 'init_pose': _jitter_pose(rng, [-2.0, -11.0, 0.2, 1.5708], 0.20),
         'velocity': _jitter_velocity(rng, [0.0, 0.52], 0.02), 'target': [-2.0, 8.5, 0.0]},
        {'name': 'vessel_e2', 'type': 'e', 'radius': 3.0, 'init_pose': _jitter_pose(rng, [6.0, -10.8, 0.2, 1.5708], 0.20),
         'velocity': _jitter_velocity(rng, [0.0, 0.54], 0.02), 'target': [6.0, 9.5, 0.0]},
        {'name': 'vessel_f1', 'type': 'f', 'radius': 3.5, 'init_pose': _jitter_pose(rng, [2.0, 8.4, 0.2, -1.5708], 0.20),
         'velocity': _jitter_velocity(rng, [0.0, -0.48], 0.02), 'target': [2.0, -4.5, 0.0]},
        {'name': 'vessel_f2', 'type': 'f', 'radius': 3.5, 'init_pose': _jitter_pose(rng, [10.0, 9.0, 0.2, -1.5708], 0.20),
         'velocity': _jitter_velocity(rng, [0.0, -0.46], 0.02), 'target': [10.0, -4.0, 0.0]},
    ]

    # ── 启动项 ────────────────────────────────────────
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
        DeclareLaunchArgument('debris_mesh', default_value='file:///home/lu/paper2/core_ws/src/model/models/meshes/ultimate_realistic_wooden_crates_pack.glb'),
    ]

    heron_xacro = os.path.join(model_share, 'urdf', 'heron_dave.urdf.xacro')

    # ═══════════════════════════════════════════════════
    # 每个 Heron USV 的启动组
    # ═══════════════════════════════════════════════════
    for usv in usv_configs:
        uid = usv['id']
        ns = f'usv_{uid}'

        # Heron 模型
        doc = xacro.process_file(heron_xacro, mappings={'namespace': ns})
        robot_desc = doc.toxml()

        rsp = Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', namespace=ns, output='screen',
            parameters=[{'robot_description': robot_desc}],
            remappings=[('tf', '/tf'), ('tf_static', '/tf_static')],
            condition=IfCondition(use_models),
        )
        launch_entities.append(rsp)

        # USV 运动仿真
        usv_sim = Node(
            package='trajectory_smoother', executable='usv_simulator',
            name='usv_simulator', namespace=ns, output='screen',
            parameters=[{
                'init_pose': usv['init_pose'],
                'speed': 1.35,
                'publish_rate': 20.0,
            }],
        )
        launch_entities.append(usv_sim)

        # PRM 路径规划 (提供 reference_path)
        prm_node = Node(
            package='topology_planner', executable='prm_node',
            name='prm_node', namespace=ns, output='screen',
            parameters=[guidance_config, {
                'agent_id': uid,
                'offset': usv['init_pose'],
                'target_offset': usv['target_offset'],
                'target': [usv['goal'][0], usv['goal'][1], 0.0],
                'clock_frequency': 10.0,
                'enable_visualization': True,
                'enable_info_log': False,
                'enable_debug_report': False,
                'exit_clearance_margin': 2.5,
                'exit_hold_count': 10,
                'static_obstacles_x': [6.5, 17.5, 9.5, 14.5],
                'static_obstacles_y': [-11.5, -2.5, 4.5, 12.0],
                'static_obstacles_r': [0.85, 0.95, 0.73, 0.82],
            }],
            condition=IfCondition(use_guidance),
        )
        launch_entities.append(prm_node)

        # Multi-USV 规划器 (COLREGs 硬约束, DecompUtil 走廊, 单次求解)
        planner_node = Node(
            package='multi_usv_planner', executable='usv_planner_node_exe',
            name='usv_planner_node', namespace=ns, output='screen',
            parameters=[planner_config, {
                'agent_id': uid,
                'enable_topology_rerank': enable_topology_rerank,
                'neighbors': [i for i in active_ids if i != uid],
                'goal.x': usv['goal'][0],
                'goal.y': usv['goal'][1],
            }],
        )
        launch_entities.append(planner_node)

        # 拓扑选择层
        topo_selector_node = Node(
            package='multi_usv_planner', executable='topology_selector_node',
            name='topology_selector_node', namespace=ns, output='screen',
            parameters=[{
                'agent_id': uid,
                'neighbors': [i for i in active_ids if i != uid],
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
                'static_obstacles_x': [6.5, 17.5, 9.5, 14.5],
                'static_obstacles_y': [-11.5, -2.5, 4.5, 12.0],
                'static_obstacles_r': [0.85, 0.95, 0.73, 0.82],
            }],
            condition=IfCondition(use_guidance),
        )
        launch_entities.append(topo_selector_node)

        # 在线拓扑可视化
        topo_viz_node = Node(
            package='trajectory_smoother', executable='online_trajectory_visualizer',
            name='online_trajectory_visualizer', namespace=ns, output='screen',
            parameters=[guidance_config, {
                'agent_id': uid,
                'target': [usv['goal'][0], usv['goal'][1], 0.0],
            }],
            condition=IfCondition(use_guidance),
        )
        launch_entities.append(topo_viz_node)


    # ═══════════════════════════════════════════════════
    # 动态障碍物 (vessel_e/f) — 条件启用
    # ═══════════════════════════════════════════════════
    use_dynamic_obs = os.environ.get('MULTI_USV_DYNAMIC_OBS', '0') == '1'
    if use_dynamic_obs:
        dynamic_obs_node = Node(
        package='multi_usv_planner', executable='dynamic_obstacle_node.py',
        name='dynamic_obstacle_node', output='screen',
        parameters=[{
            'update_rate': 10.0,
            'vessels': json.dumps([
                {'name': 'vessel_e1', 'radius': 2.8,
                 'init_pose': list(obstacle_vessels[0]['init_pose']),
                 'velocity': list(obstacle_vessels[0]['velocity']),
                 'target': list(obstacle_vessels[0]['target'])},
                {'name': 'vessel_e2', 'radius': 2.8,
                 'init_pose': list(obstacle_vessels[1]['init_pose']),
                 'velocity': list(obstacle_vessels[1]['velocity']),
                 'target': list(obstacle_vessels[1]['target'])},
                {'name': 'vessel_f1', 'radius': 3.0,
                 'init_pose': list(obstacle_vessels[2]['init_pose']),
                 'velocity': list(obstacle_vessels[2]['velocity']),
                 'target': list(obstacle_vessels[2]['target'])},
                {'name': 'vessel_f2', 'radius': 3.0,
                 'init_pose': list(obstacle_vessels[3]['init_pose']),
                 'velocity': list(obstacle_vessels[3]['velocity']),
                 'target': list(obstacle_vessels[3]['target'])},
            ]),
        }],
    )
        launch_entities.append(dynamic_obs_node)

        for vessel in obstacle_vessels:
            vessel_xacro = os.path.join(model_share, 'urdf', f"vessel_{vessel['type']}.urdf.xacro")
            vessel_doc = xacro.process_file(vessel_xacro, mappings={'namespace': vessel['name']})
            vessel_description = vessel_doc.toxml()
            vessel_rsp = Node(
                package='robot_state_publisher', executable='robot_state_publisher',
                name='robot_state_publisher', namespace=vessel['name'], output='screen',
                parameters=[{'robot_description': vessel_description}],
                remappings=[('tf', '/tf'), ('tf_static', '/tf_static')],
                condition=IfCondition(use_models),
            )
            launch_entities.append(vessel_rsp)

            vessel_tf = Node(
                package='trajectory_smoother', executable='odom_tf_broadcaster',
                name='odom_tf_broadcaster', namespace=vessel['name'], output='screen',
                parameters=[{'publish_rate': 10.0, 'parent_frame': 'odom'}],
                condition=IfCondition(use_models),
            )
            launch_entities.append(vessel_tf)

        # 漂浮垃圾 debris（通过环境变量 MULTI_USV_DEBRIS_COUNT 控制数量）
        debris_node = Node(
            package='multi_usv_planner', executable='floating_debris_node.py',
            name='floating_debris_node', output='screen',
            parameters=[{
                'num_debris': num_debris,
                'update_rate': 10.0,
                'seed': seed_value,
                'frame_id': 'odom',
                'mesh_resource': debris_mesh,
                'mesh_scale_x': 0.75,
                'mesh_scale_y': 0.48,
                'mesh_scale_z': 0.18,
            }],
        )
        launch_entities.append(debris_node)

    # ═══════════════════════════════════════════════════
    # 轻量场景可视化：小岛礁 + 起终点 + 中轴线
    # ═══════════════════════════════════════════════════
    scene_node = Node(
        package='multi_usv_planner', executable='scene_visualizer_node',
        name='scene_visualizer', output='screen',
    )
    launch_entities.append(scene_node)

    # ═══════════════════════════════════════════════════
    # RViz
    # ═══════════════════════════════════════════════════
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2_planner',
        arguments=['-d', rviz_config], output='screen',
        condition=IfCondition(use_rviz),
    )
    launch_entities.append(rviz_node)

    return LaunchDescription(launch_entities)
