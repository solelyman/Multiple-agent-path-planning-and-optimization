import numpy as np


PARAMETER_DEFAULTS = {
    "comm_radius": 60.0,
    "target": [46.0, 12.0, 0.0],
    "formation_coupling_gain": 0.15,
    "comm_delay_sec": 0.0,
    "avoidance_decision_hold_sec": 3.5,
    "local_avoid_trigger_margin": 10.0,
    "safety_buffer_radius": 2.0,
    "static_obstacle_safety_margin": 3.0,
    "shared_obstacle_distance_margin": 8.0,
    "shared_obstacle_front_margin": 9.0,
    "shared_obstacle_z_margin": 30.0,
    "obstacle_priority_distance": 35.0,
    "obstacle_critical_clearance": 8.0,
    "neighbor_progress_margin": 6.0,
    "avoidance_entry_count": 2,
    "avoidance_exit_count": 5,
    "neighbor_discussion_distance": 40.0,
    "emergency_lock_margin": 6.0,
    "stop_radius": 5.0,
    "exit_clearance_margin": 1.8,
    "exit_hold_count": 6,
    "collision_penalty": 1.0e5,
    "colregs_penalty": 2.5e2,
    "consistency_penalty": 15.0,
    "path_switch_penalty": 20.0,
    "mode_switch_hysteresis": 15.0,
    "single_path_yield_penalty": 5.0e4,
    "soft_conflict_gain": 60.0,
    "path_sample_count": 40,
    "head_on_heading_tol_deg": 35.0,
    "head_on_bearing_tol_deg": 25.0,
    "overtaking_heading_tol_deg": 30.0,
    "overtaking_bearing_deg": 112.5,
    "crossing_progress_margin": 0.05,
    "decision_settle_sec": 0.25,
    "max_game_rounds": 3,
    "formation_path_points": 50,
    "route_lock_repeat_count": 2,
    "candidate_freeze_sec": 0.8,
    "dynamic_obstacle_topics": [""],
    "dynamic_obstacle_names": [""],
    "dynamic_obstacle_radii": [0.85, 2.70],
    "dynamic_obstacle_timeout_sec": 0.5,
    "static_obstacle_x": [0.0],
    "static_obstacle_y": [0.0],
    "static_obstacle_radii": [0.0],
    "usv_perception_radius": 55.0,
    "usv_collision_radius": 2.0,
    "vessel_safety_margin": 2.0,
    "spatiotemporal_horizon_sec": 18.0,
    "spatiotemporal_soft_margin": 4.0,
    "relative_closing_speed_threshold": 0.15,
    "min_pairwise_ttc_sec": 1.0,
    "usv_non_neighbor_colregs_gain": 300.0,
}


def default_neighbors(agent_id):
    topology = {
        1: [2, 3],
        2: [1, 4],
        3: [1, 4],
        4: [2, 3],
    }
    return topology.get(int(agent_id), [])


def declare_smoother_parameters(node, agent_id):
    for name, default in PARAMETER_DEFAULTS.items():
        node.declare_parameter(name, default)
    node.declare_parameter(f"usv_{agent_id}_offset", [0.0, 0.0, 0.0])
    node.declare_parameter(f"usv_{agent_id}_neighbors", default_neighbors(agent_id))


def get_float(node, name):
    return float(node.get_parameter(name).value)


def get_int(node, name):
    return int(node.get_parameter(name).value)


def get_list(node, name):
    return list(node.get_parameter(name).value)


def get_static_obstacles(node):
    static_x = get_list(node, "static_obstacle_x")
    static_y = get_list(node, "static_obstacle_y")
    static_radii = get_list(node, "static_obstacle_radii")
    obstacles = []
    for i in range(min(len(static_x), len(static_y), len(static_radii))):
        obstacles.append((float(static_x[i]), float(static_y[i]), float(static_radii[i])))
    return obstacles


def get_colregs_angles(node):
    return {
        "head_on_heading_tol": np.deg2rad(get_float(node, "head_on_heading_tol_deg")),
        "head_on_bearing_tol": np.deg2rad(get_float(node, "head_on_bearing_tol_deg")),
        "overtaking_heading_tol": np.deg2rad(get_float(node, "overtaking_heading_tol_deg")),
        "overtaking_bearing": np.deg2rad(get_float(node, "overtaking_bearing_deg")),
    }
