#include <multi_usv_planner/usv_planner.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <decomp_util/ellipsoid_decomp.h>
#include <decomp_geometry/polyhedron.h>

#include <algorithm>
#include <numeric>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <iomanip>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <cstdlib>

using namespace std::chrono_literals;
using namespace MultiUSV;

static void applyLateralShift(ReferencePath &path, double shift_meters)
{
    if (path.points.size() < 3 || std::abs(shift_meters) < 0.01)
        return;

    const double shift = std::clamp(shift_meters, -2.0, 2.0);

    for (size_t i = 0; i < path.points.size(); ++i)
    {
        size_t i_prev = (i == 0) ? 0 : i - 1;
        size_t i_next = (i == path.points.size() - 1) ? i : i + 1;
        Eigen::Vector2d dir = path.points[i_next] - path.points[i_prev];
        double hdg = std::atan2(dir.y(), dir.x());
        double nx = -std::sin(hdg);
        double ny =  std::cos(hdg);
        path.points[i].x() += nx * shift;
        path.points[i].y() += ny * shift;
    }
}

double accumulatePathLength(const ReferencePath &path)
{
    if (path.points.size() < 2)
        return 0.0;

    double total = 0.0;
    for (size_t i = 1; i < path.points.size(); ++i)
        total += (path.points[i] - path.points[i - 1]).norm();
    return total;
}

double sampledPathDistance(const ReferencePath &a,
                           const ReferencePath &b,
                           double horizon = 24.0,
                           int samples = 8)
{
    if (a.points.size() < 2 || b.points.size() < 2 || samples <= 0)
        return 0.0;

    const double a_len = a.length();
    const double b_len = b.length();
    const double probe = std::max(1.0, std::min(horizon, std::max(a_len, b_len)));
    double accum = 0.0;
    for (int i = 0; i < samples; ++i)
    {
        const double alpha = (samples == 1)
            ? 0.0
            : static_cast<double>(i) / static_cast<double>(samples - 1);
        const double sa = std::min(alpha * probe, a_len);
        const double sb = std::min(alpha * probe, b_len);
        accum += (a.sample(sa) - b.sample(sb)).norm();
    }
    return accum / static_cast<double>(samples);
}

double sampledHeadingGap(const ReferencePath &a,
                         const ReferencePath &b,
                         double horizon = 10.0,
                         int samples = 4)
{
    if (a.points.size() < 2 || b.points.size() < 2 || samples <= 0)
        return 0.0;

    const double a_len = a.length();
    const double b_len = b.length();
    const double probe = std::max(1.0, std::min(horizon, std::max(a_len, b_len)));
    const double heading_step = 1.2;
    double accum = 0.0;
    for (int i = 0; i < samples; ++i)
    {
        const double alpha = (samples == 1)
            ? 0.0
            : static_cast<double>(i) / static_cast<double>(samples - 1);
        const double sa = std::min(alpha * probe, a_len);
        const double sb = std::min(alpha * probe, b_len);
        const double ha = a.headingBetween(sa, std::min(sa + heading_step, a_len));
        const double hb = b.headingBetween(sb, std::min(sb + heading_step, b_len));
        accum += std::abs(Pose2D::normalizeAngle(ha - hb));
    }
    return accum / static_cast<double>(samples);
}

double topologyDiversityScore(const ReferencePath &a, const ReferencePath &b)
{
    return sampledPathDistance(a, b) + 1.6 * sampledHeadingGap(a, b);
}

double clearanceDangerRamp(double clearance, double stop_clearance, double slow_clearance)
{
    if (!std::isfinite(clearance))
        return 0.0;
    if (clearance <= stop_clearance)
        return 1.0;
    if (clearance >= slow_clearance)
        return 0.0;
    return 1.0 - (clearance - stop_clearance) /
        std::max(slow_clearance - stop_clearance, 1e-3);
}

double stageDangerWeight(int stage, int stage_count)
{
    if (stage_count <= 1)
        return 1.0;
    const double alpha = static_cast<double>(stage) /
                         static_cast<double>(stage_count - 1);
    return std::clamp(1.0 - 0.85 * alpha, 0.15, 1.0);
}

double dynamicStopClearance(double stop_radius)
{
    return std::max(stop_radius * 0.72, 1.8);
}

double dynamicSlowClearance(double obstacle_clearance)
{
    return std::max(obstacle_clearance + 3.8, 7.8);
}

double dynamicGuardCap(double nearest_dynamic)
{
    if (!std::isfinite(nearest_dynamic) || nearest_dynamic <= 0.0)
        return 0.0;
    return std::clamp(0.12 * nearest_dynamic + 0.38, 0.0, 0.95);
}

double dynamicGuardFromClearance(double nearest_dynamic,
                                 double stop_radius,
                                 double obstacle_clearance)
{
    const double stop_clearance = dynamicStopClearance(stop_radius);
    const double slow_clearance = dynamicSlowClearance(obstacle_clearance);
    const double danger = clearanceDangerRamp(
        nearest_dynamic, stop_clearance, slow_clearance);
    const double raw_guard = 1.05 * danger;
    return std::min(raw_guard, dynamicGuardCap(nearest_dynamic));
}

USVPlanner::USVPlanner()
    : Node("usv_planner_node")
{
    declareParams();
    loadParams();
    initCommunication();

    {
        AcadosContouringSolver::Params ap;
        const auto &mp = mpc_solver_.params();
        ap.N = mp.N;
        ap.dt = mp.dt;
        ap.desired_speed = mp.desired_speed;
        ap.obstacle_clearance = mp.obstacle_clearance;
        ap.weight_acc = mp.acados_weight_acc;
        ap.weight_angvel = mp.acados_weight_angvel;
        ap.weight_velocity = mp.acados_weight_velocity;
        ap.weight_contour = mp.acados_weight_contour;
        ap.weight_lag = mp.acados_weight_lag;
        ap.weight_terminal_angle = mp.acados_weight_terminal_angle;
        ap.weight_terminal_contour = mp.acados_weight_terminal_contouring;
        ap.warmstart_with_previous = false;
        acados_solver_.setParams(ap);
        bool acados_ok = acados_solver_.initialize();
        if (acados_ok)
            RCLCPP_INFO(this->get_logger(), "USV-%d acados contouring solver ready (5-state, EXTERNAL+EXACT+MIRROR, ellipsoid soft penalty[N_ELL=%d])", agent_id_, AcadosContouringSolver::N_ELLIPSOIDS);
        else
            RCLCPP_WARN(this->get_logger(), "USV-%d acados init failed: %s",
                        agent_id_, acados_solver_.statusMessage().c_str());
    }

    RCLCPP_INFO(this->get_logger(),
        "USV-%d 启动 | K=%d 邻居 | MPC N=%d | COLREGs 硬约束 | CTRL=%s | ILOS lh=%.1f",
        agent_id_, neighbor_selector_.k,
        mpc_solver_.params().N,
        control_mode_.c_str(),
        controller_.params().lookahead_h);
}

#define P_(name, val) this->declare_parameter(#name, val)

void USVPlanner::declareParams()
{
    P_(agent_id, 1);      P_(neighbors, (std::vector<int64_t>{2, 3, 4}));
    P_(goal.x, 120.0);    P_(goal.y, 30.0);
    P_(use_direct_cmd, false);  P_(use_selected_reference, true);  P_(use_original_like_reference_mode, false);  P_(control_mode, "los");
    P_(enable_topology_rerank, true);  P_(topology_rerank_candidates, 4);  P_(topology_rerank_switch_margin, 0.45);
    P_(static_obstacles_x, std::vector<double>{});  P_(static_obstacles_y, std::vector<double>{});  P_(static_obstacles_r, std::vector<double>{});

    // MPC
    P_(mpc.N, 10);        P_(mpc.dt, 0.4);         P_(mpc.max_vel, 2.0);
    P_(mpc.max_omega, 1.57);  P_(mpc.max_acc, 1.0);  P_(mpc.desired_speed, 1.5);
    P_(mpc.max_iter, 50);     P_(mpc.weight_goal, 8.0);  P_(mpc.weight_path, 60.0);
    P_(mpc.weight_obstacle, 30.0);  P_(mpc.weight_smooth, 1.0);  P_(mpc.weight_speed, 1.0);
    P_(mpc.weight_guidance, 50.0);  P_(mpc.guidance_margin, 0.8);
    P_(mpc.weight_progress, 10.0);  P_(mpc.weight_backtrack, 40.0);  P_(mpc.min_progress_step, 0.15);
    P_(mpc.weight_terminal_angle, 12.0);  P_(mpc.weight_terminal_contouring, 3.0);
    P_(mpc.weight_seed_alignment, 3.0);
    P_(mpc.weight_neighbor, 25.0);
    P_(mpc.obstacle_clearance, 2.0);
    P_(mpc.dynamic_obstacle_growth, 0.12);
    P_(mpc.acados_weight_acc, 0.25);
    P_(mpc.acados_weight_angvel, 0.50);
    P_(mpc.acados_weight_velocity, 0.30);
    P_(mpc.acados_weight_contour, 0.05);
    P_(mpc.acados_weight_lag, 0.10);
    P_(mpc.acados_weight_terminal_angle, 100.0);
    P_(mpc.acados_weight_terminal_contouring, 10.0);

    // Neighbor
    P_(neighbor.k, 3);  P_(neighbor.prediction_window, 5.0);  P_(neighbor.danger_distance, 8.0);

    // ILOS
    P_(controller.m11, 28.0);  P_(controller.m22, 28.0);   P_(controller.Izz, 10.0);
    P_(controller.d_u, -16.45);  P_(controller.d_r, -6.0); P_(controller.lookahead_h, 8.0);
    P_(controller.k_x, 2.0);   P_(controller.k_u, 5.0);    P_(controller.k_psi, 1.0);
    P_(controller.k_y, 0.5);   P_(controller.k_r, 5.0);    P_(controller.max_u, 2.0);
    P_(controller.max_r, 1.0); P_(controller.filter_omega_n, 2.2);  P_(controller.filter_zeta, 0.85);
    P_(controller.stop_radius, 2.0);
    P_(pass_x_threshold, 55.0);
    P_(episode_timeout_s, 600.0);
    P_(colregs_enabled, true);
}

#undef P_

void USVPlanner::loadParams()
{
    agent_id_ = this->get_parameter("agent_id").as_int();
    auto nb_vec = this->get_parameter("neighbors").as_integer_array();
    for (auto id : nb_vec) neighbor_ids_.push_back((int)id);

    goal_x_ = this->get_parameter("goal.x").as_double();
    goal_y_ = this->get_parameter("goal.y").as_double();
    has_goal_ = true;

    use_direct_cmd_ = this->get_parameter("use_direct_cmd").as_bool();
    use_selected_reference_ = this->get_parameter("use_selected_reference").as_bool();
    use_original_like_reference_mode_ = this->get_parameter("use_original_like_reference_mode").as_bool();
    control_mode_ = this->get_parameter("control_mode").as_string();
    enable_topology_rerank_ = this->get_parameter("enable_topology_rerank").as_bool();
    topology_rerank_candidates_ = std::max(
        1,
        static_cast<int>(this->get_parameter("topology_rerank_candidates").as_int()));
    topology_rerank_switch_margin_ = this->get_parameter("topology_rerank_switch_margin").as_double();

    auto obs_x = this->get_parameter("static_obstacles_x").as_double_array();
    auto obs_y = this->get_parameter("static_obstacles_y").as_double_array();
    auto obs_r = this->get_parameter("static_obstacles_r").as_double_array();
    int n_obs = std::min({(int)obs_x.size(), (int)obs_y.size(), (int)obs_r.size()});
    for (int i = 0; i < n_obs; i++)
        static_obstacles_.push_back(Obstacle{obs_x[i], obs_y[i], obs_r[i]});

    auto load_double = [&](const char *k) { return this->get_parameter(k).as_double(); };
    auto load_int = [&](const char *k) { return this->get_parameter(k).as_int(); };

    // MPC
    {
        auto &p = mpc_solver_.params();
        p.N = load_int("mpc.N");
        p.dt = load_double("mpc.dt");
        p.max_vel = load_double("mpc.max_vel");
        p.max_omega = load_double("mpc.max_omega");
        p.max_acc = load_double("mpc.max_acc");
        p.desired_speed = load_double("mpc.desired_speed");
        p.max_iter = load_int("mpc.max_iter");
        p.weight_goal = load_double("mpc.weight_goal");
        p.weight_path = load_double("mpc.weight_path");
        p.weight_obstacle = load_double("mpc.weight_obstacle");
        p.weight_smooth = load_double("mpc.weight_smooth");
        p.weight_speed = load_double("mpc.weight_speed");
        p.weight_guidance = load_double("mpc.weight_guidance");
        p.guidance_margin = load_double("mpc.guidance_margin");
        p.weight_progress = load_double("mpc.weight_progress");
        p.weight_backtrack = load_double("mpc.weight_backtrack");
        p.min_progress_step = load_double("mpc.min_progress_step");
        p.weight_terminal_angle = load_double("mpc.weight_terminal_angle");
        p.weight_terminal_contouring = load_double("mpc.weight_terminal_contouring");
        p.weight_seed_alignment = load_double("mpc.weight_seed_alignment");
        p.obstacle_clearance = load_double("mpc.obstacle_clearance");
        p.dynamic_obstacle_growth = load_double("mpc.dynamic_obstacle_growth");
        p.acados_weight_acc = load_double("mpc.acados_weight_acc");
        p.acados_weight_angvel = load_double("mpc.acados_weight_angvel");
        p.acados_weight_velocity = load_double("mpc.acados_weight_velocity");
        p.acados_weight_contour = load_double("mpc.acados_weight_contour");
        p.acados_weight_lag = load_double("mpc.acados_weight_lag");
        p.acados_weight_terminal_angle = load_double("mpc.acados_weight_terminal_angle");
        p.acados_weight_terminal_contouring = load_double("mpc.acados_weight_terminal_contouring");
    }

    // Neighbor selector
    neighbor_selector_.k = load_int("neighbor.k");
    neighbor_selector_.prediction_window = load_double("neighbor.prediction_window");
    neighbor_selector_.danger_distance = load_double("neighbor.danger_distance");

    // Behavioral layer
    colregs_enabled_ = this->get_parameter("colregs_enabled").as_bool();

    // Controller
    {
        USVController::Params cp;
        cp.m11 = load_double("controller.m11");
        cp.m22 = load_double("controller.m22");
        cp.Izz = load_double("controller.Izz");
        cp.d_u = load_double("controller.d_u");
        cp.d_r = load_double("controller.d_r");
        cp.lookahead_h = load_double("controller.lookahead_h");
        cp.k_x = load_double("controller.k_x");
        cp.k_u = load_double("controller.k_u");
        cp.k_psi = load_double("controller.k_psi");
        cp.k_y = load_double("controller.k_y");
        cp.k_r = load_double("controller.k_r");
        cp.max_u = load_double("controller.max_u");
        cp.max_r = load_double("controller.max_r");
        cp.filter_omega_n = load_double("controller.filter_omega_n");
        cp.filter_zeta = load_double("controller.filter_zeta");
        cp.stop_radius = load_double("controller.stop_radius");
    pass_x_threshold_ = load_double("pass_x_threshold");
    episode_timeout_s_ = load_double("episode_timeout_s");
    controller_.setParams(cp);
    }
}

void USVPlanner::initCommunication()
{
    const auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::QoS(10),
        [this](const nav_msgs::msg::Odometry::SharedPtr m) { odomCallback(m); });

    sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
        "reference_path", latched_qos,
        [this](const nav_msgs::msg::Path::SharedPtr m) { pathCallback(m); });

    sub_topo_path_ = this->create_subscription<topology_interfaces::msg::TopologicalPathArray>(
        "topological_path", 10,
        [this](const topology_interfaces::msg::TopologicalPathArray::SharedPtr m) { topoPathCallback(m); });

    sub_goal_ = this->create_subscription<geometry_msgs::msg::Point>(
        "final_goal", 10,
        [this](const geometry_msgs::msg::Point::SharedPtr m) { goalCallback(m); });

    const std::vector<std::pair<std::string, double>> dynamic_obstacle_specs = {
        {"vessel_e1", 2.8}, {"vessel_e2", 2.8},
        {"vessel_f1", 3.0}, {"vessel_f2", 3.0},
    };
    for (const auto &[name, radius] : dynamic_obstacle_specs)
    {
        auto cb = [this, name, radius](const nav_msgs::msg::Odometry::SharedPtr m)
        { dynamicObstacleCallback(m, name, radius); };
        sub_dynamic_obstacles_.push_back(
            this->create_subscription<nav_msgs::msg::Odometry>(
                "/" + name + "/odom", 10, cb));
    }
    // Floating debris (debris_0 ~ debris_11)
    for (int i = 0; i < 12; i++)
    {
        std::string name = "debris_" + std::to_string(i);
        double radius = 0.45;
        auto cb = [this, name, radius](const nav_msgs::msg::Odometry::SharedPtr m)
        { dynamicObstacleCallback(m, name, radius); };
        sub_dynamic_obstacles_.push_back(
            this->create_subscription<nav_msgs::msg::Odometry>(
                "/" + name + "/odom", 10, cb));
    }

    // Trajectory for neighbors
    pub_broadcast_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/usv_comms/usv_" + std::to_string(agent_id_), 10);

    for (int nb_id : neighbor_ids_)
    {
        auto cb = [this, nb_id](const std_msgs::msg::Float64MultiArray::SharedPtr m)
        { neighborCallback(m, nb_id); };
        sub_neighbors_.push_back(
            this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/usv_comms/usv_" + std::to_string(nb_id), 10, cb));
    }

    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    pub_ref_path_ = this->create_publisher<nav_msgs::msg::Path>("smooth_trajectory", 10);

    // Visualization
    pub_viz_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "trajectory_markers", 10);
    pub_topo_viz_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "topology_candidates", 10);
    const std::string ns_prefix = "/usv_" + std::to_string(agent_id_);
    pub_odom_alias_ = this->create_publisher<nav_msgs::msg::Odometry>(
        ns_prefix + "/odom_visual", 10);
    pub_planned_traj_alias_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        ns_prefix + "/planned_trajectory", 10);
    pub_contouring_alias_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        ns_prefix + "/contouring", 10);
    pub_contouring_current_alias_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        ns_prefix + "/contouring/current", 10);
    pub_contouring_path_alias_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        ns_prefix + "/contouring/path", 10);
    pub_contouring_points_alias_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        ns_prefix + "/contouring/points", 10);
    pub_goal_module_alias_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        ns_prefix + "/goal_module", 10);
    pub_dynamic_obs_alias_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        ns_prefix + "/obstacles_3d", 10);

    timer_ = this->create_wall_timer(100ms, [this]() { controlLoop(); });
    RCLCPP_INFO(this->get_logger(), "USV-%d control loop: 10 Hz", agent_id_);
}

void USVPlanner::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    ego_.x = msg->pose.pose.position.x;
    ego_.y = msg->pose.pose.position.y;

    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    double roll, pitch;
    tf2::Matrix3x3(q).getRPY(roll, pitch, ego_.psi);

    ego_.v = msg->twist.twist.linear.x;
    ego_.omega = msg->twist.twist.angular.z;
    ego_.received = true;
    has_odom_ = true;
    if (pub_odom_alias_)
        pub_odom_alias_->publish(*msg);

    if (metrics_written_ && has_goal_)
    {
        double dxg = msg->pose.pose.position.x - goal_x_;
        double dyg = msg->pose.pose.position.y - goal_y_;
        double dist_to_goal = std::sqrt(dxg * dxg + dyg * dyg);
        bool still_far = dist_to_goal > 40.0;
        if (still_far)
        {
            metrics_written_ = false;
            metrics_initialized_ = false;
            done_ = false;
            collision_dynamic_ = false;
            collision_static_ = false;
            collision_neighbor_ = false;
            min_dynamic_clearance_ = std::numeric_limits<double>::max();
            min_static_clearance_ = std::numeric_limits<double>::max();
            min_neighbor_clearance_ = std::numeric_limits<double>::max();
            episode_start_time_ = this->now();
            metrics_initialized_ = true;
            RCLCPP_WARN(this->get_logger(),
                "USV-%d episode reset (was done but still far from goal, dist=%.1f)",
                agent_id_, dist_to_goal);
            reset_guard_until_ = this->now() + rclcpp::Duration::from_seconds(20.0);
            near_goal_cooldown_ = 200;
            return;
        }
    }

    if (first_run_)
    {
        double dxg = ego_.x - goal_x_;
        double dyg = ego_.y - goal_y_;
        double dist2goal = std::sqrt(dxg * dxg + dyg * dyg);
        if (ego_.x > pass_x_threshold_ || dist2goal < 40.0) {
            RCLCPP_WARN(this->get_logger(),
                "USV-%d spawn past threshold (x=%.1f, dist2goal=%.1f), skipping",
                agent_id_, ego_.x, dist2goal);
            first_run_ = false;
            metrics_written_ = true;
            return;
        }
        controller_.reset(ego_.omega);
        last_time_ = this->now();
        episode_start_time_ = last_time_;
        reset_guard_until_ = this->now() + rclcpp::Duration::from_seconds(20.0);
        near_goal_cooldown_ = 200;
        metrics_initialized_ = true;
        RCLCPP_INFO(this->get_logger(),
            "USV-%d 初始: [%.2f, %.2f, %.1f°] cooldown=%d",
            agent_id_, ego_.x, ego_.y, ego_.psi * 180.0 / M_PI, near_goal_cooldown_);
        first_run_ = false;
        return;
    }
}

void USVPlanner::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    if (!use_selected_reference_)
        return;
    if (enable_topology_rerank_ && !use_original_like_reference_mode_)
        return;

    ReferencePath received_path;
    for (const auto &pose : msg->poses)
    {
        received_path.points.push_back(
            Eigen::Vector2d(pose.pose.position.x, pose.pose.position.y));
    }
    received_path.total_length = accumulatePathLength(received_path);
    has_reference_path_ = received_path.points.size() >= 2;
    if (has_reference_path_)
    {
        if (use_original_like_reference_mode_)
        {
            smooth_ref_path_ = std::move(received_path);
            selected_reference_active_ = true;
            topo_ref_path_.clear();
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "USV-%d received reference_path with %zu poses", agent_id_, msg->poses.size());
            return;
        }

        const bool freeze_switching = freezeReferenceSwitching();
        if (freeze_switching && smooth_ref_path_.points.size() >= 2)
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1500,
                "USV-%d selected reference update frozen by nearby dynamics",
                agent_id_);
            return;
        }
        if (selected_reference_active_ &&
            smooth_ref_path_.points.size() >= 2 &&
            topo_ref_path_.points.size() >= 2 &&
            !referenceHandoffReady(received_path, smooth_ref_path_))
        {
            if (freeze_switching)
            {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1500,
                    "USV-%d selected reference rebased but frozen by nearby dynamics",
                    agent_id_);
                return;
            }
            selected_reference_active_ = false;
            mpc_solver_.clearWarmstart();
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1500,
                "USV-%d selected reference rebased, hold topo and reset warmstart",
                agent_id_);
        }

        smooth_ref_path_ = std::move(received_path);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "USV-%d received selected reference_path with %zu poses", agent_id_, msg->poses.size());
    }
    else
    {
        smooth_ref_path_.clear();
        selected_reference_active_ = false;
        if (use_original_like_reference_mode_)
            topo_ref_path_.clear();
    }
}

void USVPlanner::topoPathCallback(const topology_interfaces::msg::TopologicalPathArray::SharedPtr msg)
{
    if (msg->topological_paths.empty()) return;

    topo_candidates_.clear();

    auto signature_key = [](const std::vector<double> &sig) {
        if (sig.empty())
            return 0.0;
        return std::round(sig.front() * 1e6) / 1e6;
    };

    const bool show_local_candidates = shouldShowLocalCandidates();
    if (!show_local_candidates)
    {
        clearTopologyCandidateMarkers();
    }

    visualization_msgs::msg::MarkerArray markers;

    // 先删除之前所有 marker
    for (int i = 0; i < prev_topo_marker_count_; i++)
    {
        visualization_msgs::msg::Marker del;
        del.header = msg->header;
        del.ns = "topology_candidates";
        del.id = i;
        del.action = visualization_msgs::msg::Marker::DELETE;
        markers.markers.push_back(del);
    }

    std::vector<double> kept_signatures;
    std::vector<TopologyCandidate> all_candidates;
    all_candidates.reserve(msg->topological_paths.size());
    int marker_id = 0;
    for (const auto &topo : msg->topological_paths)
    {
        const double key = signature_key(topo.h_signature);
        const bool duplicate = std::any_of(
            kept_signatures.begin(), kept_signatures.end(),
            [&](double prev) { return std::abs(prev - key) < 1e-6; });
        if (duplicate)
            continue;

        ReferencePath candidate_path;
        candidate_path.points.reserve(topo.path.poses.size());
        for (const auto &pose : topo.path.poses)
        {
            candidate_path.points.push_back(Eigen::Vector2d(
                pose.pose.position.x, pose.pose.position.y));
        }
        candidate_path.total_length = accumulatePathLength(candidate_path);
        if (candidate_path.points.size() < 2)
            continue;

        all_candidates.push_back(TopologyCandidate{candidate_path, topo.h_signature});
        kept_signatures.push_back(key);
    }

    if (!all_candidates.empty())
    {
        const int keep_count = std::min<int>(
            topology_rerank_candidates_, static_cast<int>(all_candidates.size()));
        std::vector<int> chosen_indices;
        chosen_indices.reserve(keep_count);
        chosen_indices.push_back(0);

        while (static_cast<int>(chosen_indices.size()) < keep_count)
        {
            int best_idx = -1;
            double best_score = -std::numeric_limits<double>::infinity();
            for (int idx = 1; idx < static_cast<int>(all_candidates.size()); ++idx)
            {
                if (std::find(chosen_indices.begin(), chosen_indices.end(), idx) != chosen_indices.end())
                    continue;

                double min_diversity = std::numeric_limits<double>::infinity();
                for (int chosen : chosen_indices)
                {
                    min_diversity = std::min(
                        min_diversity,
                        topologyDiversityScore(
                            all_candidates[idx].path,
                            all_candidates[chosen].path));
                }

                const double rank_bias = 0.05 / (1.0 + static_cast<double>(idx));
                const double candidate_score = min_diversity + rank_bias;
                if (candidate_score > best_score)
                {
                    best_score = candidate_score;
                    best_idx = idx;
                }
            }

            if (best_idx < 0)
                break;
            chosen_indices.push_back(best_idx);
        }

        topo_candidates_.reserve(chosen_indices.size());
        for (int order = 0; order < static_cast<int>(chosen_indices.size()); ++order)
        {
            const int idx = chosen_indices[order];
            topo_candidates_.push_back(all_candidates[idx]);

            visualization_msgs::msg::Marker line;
            line.header = msg->header;
            line.ns = "topology_candidates";
            line.id = marker_id++;
            line.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line.action = visualization_msgs::msg::Marker::ADD;
            line.scale.x = (order == 0) ? 0.20 : 0.12;
            line.color.a = (order == 0) ? 0.92 : 0.56;
            static const std::array<std::array<float, 3>, 6> candidate_colors = {{
                {{0.12f, 0.74f, 0.36f}},
                {{0.78f, 0.10f, 0.42f}},
                {{0.16f, 0.46f, 0.92f}},
                {{0.92f, 0.62f, 0.12f}},
                {{0.54f, 0.22f, 0.82f}},
                {{0.10f, 0.72f, 0.78f}}
            }};
            const auto &rgb = candidate_colors[order % candidate_colors.size()];
            line.color.r = rgb[0];
            line.color.g = rgb[1];
            line.color.b = rgb[2];
            for (const auto &pt : all_candidates[idx].path.points)
            {
                geometry_msgs::msg::Point p;
                p.x = pt.x();
                p.y = pt.y();
                p.z = 0.0;
                line.points.push_back(p);
            }
            markers.markers.push_back(line);
        }
    }
    prev_topo_marker_count_ = marker_id;
    topology_candidates_visible_ = show_local_candidates && marker_id > 0;

    pub_topo_viz_->publish(markers);

    if (use_original_like_reference_mode_)
        return;

    if (topo_candidates_.empty())
        return;

    ReferencePath new_topo_path = topo_candidates_.front().path;

    if (freezeReferenceSwitching() && topo_ref_path_.points.size() >= 2)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1500,
            "USV-%d topo reference update frozen by nearby dynamics", agent_id_);
        return;
    }

    bool topo_basin_changed = false;
    if (topo_ref_path_.points.size() >= 2 && new_topo_path.points.size() >= 2)
    {
        const ReferencePath new_local = buildLocalReferenceWindow(new_topo_path);
        const ReferencePath old_local = buildLocalReferenceWindow(topo_ref_path_);
        const bool local_continuous = referenceHandoffReady(new_local, old_local);

        if (!local_continuous)
        {
            const Eigen::Vector2d ego_pos(ego_.x, ego_.y);
            double s_new = 0.0;
            double s_old = 0.0;
            new_local.findClosest(ego_pos, s_new);
            old_local.findClosest(ego_pos, s_old);

            const double heading_probe = 1.6;
            const double new_heading = new_local.headingBetween(
                s_new, std::min(s_new + heading_probe, new_local.length()));
            const double old_heading = old_local.headingBetween(
                s_old, std::min(s_old + heading_probe, old_local.length()));
            const double heading_gap = std::abs(Pose2D::normalizeAngle(new_heading - old_heading));

            const Eigen::Vector2d new_p = new_local.sample(std::min(s_new + 1.6, new_local.length()));
            const Eigen::Vector2d old_p = old_local.sample(std::min(s_old + 1.6, old_local.length()));
            const double corridor_gap = (new_p - old_p).norm();

            topo_basin_changed = heading_gap > 1.05 || corridor_gap > 2.4;
        }
    }

    topo_ref_path_ = std::move(new_topo_path);

    if (topo_basin_changed && !selected_reference_active_)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1500,
            "USV-%d topo corridor changed (keep warmstart)", agent_id_);
    }
}

void USVPlanner::goalCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
    goal_x_ = msg->x;
    goal_y_ = msg->y;
    has_goal_ = true;
    RCLCPP_INFO(this->get_logger(), "USV-%d 目标: [%.1f, %.1f]", agent_id_, goal_x_, goal_y_);
}

ReferencePath USVPlanner::buildLocalReferenceWindow(const ReferencePath &path) const
{
    if (!has_odom_ || path.points.size() < 2)
        return path;

    const double total = path.length();
    if (total < 1e-6)
        return path;

    const Eigen::Vector2d ego_pos(ego_.x, ego_.y);
    double s_near = 0.0;
    path.findClosest(ego_pos, s_near);
    s_near = std::clamp(s_near, 0.0, total);

    const double back_margin = 0.35;
    const double start_s = std::max(0.0, s_near - back_margin);
    const double step = std::clamp(
        mpc_solver_.params().desired_speed * mpc_solver_.params().dt,
        0.5, 1.2);

    ReferencePath local;
    local.nominal_speed = path.nominal_speed;
    local.points.push_back(path.sample(start_s));

    for (double s = start_s + step; s < total; s += step)
    {
        Eigen::Vector2d sample = path.sample(s);
        if ((sample - local.points.back()).norm() > 1e-3)
            local.points.push_back(sample);
    }

    const Eigen::Vector2d tail = path.sample(total);
    if ((tail - local.points.back()).norm() > 1e-3)
        local.points.push_back(tail);

    if (local.points.size() < 2)
        return path;

    local.total_length = accumulatePathLength(local);
    return local;
}

ReferencePath USVPlanner::buildContouringReferenceWindow(const ReferencePath &path) const
{
    if (!has_odom_ || path.points.size() < 2)
        return path;

    const double total = path.length();
    if (total < 1e-6)
        return path;

    const Eigen::Vector2d ego_pos(ego_.x, ego_.y);
    double s_near = 0.0;
    path.findClosest(ego_pos, s_near);
    s_near = std::clamp(s_near, 0.0, total);

    const double stop_clearance = dynamicStopClearance(controller_.params().stop_radius);
    const double slow_clearance = dynamicSlowClearance(mpc_solver_.params().obstacle_clearance);
    const double danger = clearanceDangerRamp(
        currentNearestDynamicDistance(), stop_clearance, slow_clearance);

    const double back_margin = 0.35 + 0.30 * danger;
    const double start_s = std::max(0.0, s_near - back_margin);
    const double step = std::clamp(
        mpc_solver_.params().desired_speed * mpc_solver_.params().dt,
        0.40, 0.80);
    const double forward_horizon = std::clamp(
        mpc_solver_.params().desired_speed * mpc_solver_.params().dt *
            static_cast<double>(mpc_solver_.params().N + 5) +
            5.0 * danger,
        12.0, 21.0);
    const double end_s = std::min(total, start_s + forward_horizon);

    ReferencePath local;
    local.nominal_speed = path.nominal_speed;
    local.points.push_back(path.sample(start_s));

    for (double s = start_s + step; s < end_s; s += step)
    {
        const Eigen::Vector2d sample = path.sample(s);
        if ((sample - local.points.back()).norm() > 1e-3)
            local.points.push_back(sample);
    }

    const Eigen::Vector2d tail = path.sample(end_s);
    if ((tail - local.points.back()).norm() > 1e-3)
        local.points.push_back(tail);

    if (local.points.size() < 2)
        return path;

    local.total_length = accumulatePathLength(local);
    return local;
}

bool USVPlanner::referenceHandoffReady(const ReferencePath &candidate,
                                       const ReferencePath &current) const
{
    if (!has_odom_ || current.points.size() < 2)
        return true;
    if (candidate.points.size() < 2)
        return false;

    const Eigen::Vector2d ego_pos(ego_.x, ego_.y);
    double s_candidate = 0.0;
    double s_current = 0.0;
    candidate.findClosest(ego_pos, s_candidate);
    current.findClosest(ego_pos, s_current);

    const double candidate_total = candidate.length();
    const double current_total = current.length();
    const double remain_floor = std::max(2.5, 3.0 * mpc_solver_.params().desired_speed * mpc_solver_.params().dt);
    const double candidate_remain = std::max(candidate_total - s_candidate, 0.0);
    const double current_remain = std::max(current_total - s_current, 0.0);
    if (candidate_remain < remain_floor)
        return false;
    if (candidate_remain + 1.5 < std::min(current_remain, remain_floor + 1.5))
        return false;

    const std::array<double, 5> probes{{0.0, 0.8, 1.6, 2.4, 3.2}};
    double avg_dist = 0.0;
    double max_dist = 0.0;
    for (double probe : probes)
    {
        const Eigen::Vector2d p_candidate =
            candidate.sample(std::min(s_candidate + probe, candidate_total));
        const Eigen::Vector2d p_current =
            current.sample(std::min(s_current + probe, current_total));
        const double dist = (p_candidate - p_current).norm();
        avg_dist += dist;
        max_dist = std::max(max_dist, dist);
    }
    avg_dist /= static_cast<double>(probes.size());

    const double heading_probe = 1.8;
    const double heading_candidate = candidate.headingBetween(
        s_candidate, std::min(s_candidate + heading_probe, candidate_total));
    const double heading_current = current.headingBetween(
        s_current, std::min(s_current + heading_probe, current_total));
    const double heading_err =
        std::abs(Pose2D::normalizeAngle(heading_candidate - heading_current));
    const double ego_heading_err =
        std::abs(Pose2D::normalizeAngle(heading_candidate - ego_.psi));

    return avg_dist < 0.85 && max_dist < 1.6 && heading_err < 0.55 && ego_heading_err < 1.35;
}

double USVPlanner::currentNearestDynamicDistance() const
{
    double nearest = std::numeric_limits<double>::infinity();
    const double now_sec = rclcpp::Time(this->now()).seconds();
    for (const auto &pair : dynamic_obstacles_)
    {
        const auto &obs = pair.second;
        if (!obs.received)
            continue;
        if (now_sec - obs.last_update_sec > 1.5)
            continue;
        nearest = std::min(nearest,
            std::hypot(ego_.x - obs.x, ego_.y - obs.y) - obs.radius);
    }
    return nearest;
}

bool USVPlanner::freezeReferenceSwitching() const
{
    const double nearest = currentNearestDynamicDistance();
    return std::isfinite(nearest) && nearest < reference_switch_freeze_distance_;
}

bool USVPlanner::shouldShowLocalCandidates() const
{
    const double nearest = currentNearestDynamicDistance();
    return std::isfinite(nearest) && nearest < topology_candidate_show_distance_;
}

void USVPlanner::clearTopologyCandidateMarkers()
{
    if (!topology_candidates_visible_ && prev_topo_marker_count_ == 0)
        return;

    visualization_msgs::msg::MarkerArray markers;
    for (int i = 0; i < prev_topo_marker_count_; ++i)
    {
        visualization_msgs::msg::Marker del;
        del.header.frame_id = "odom";
        del.header.stamp = this->now();
        del.ns = "topology_candidates";
        del.id = i;
        del.action = visualization_msgs::msg::Marker::DELETE;
        markers.markers.push_back(del);
    }
    pub_topo_viz_->publish(markers);
    prev_topo_marker_count_ = 0;
    topology_candidates_visible_ = false;
}

void USVPlanner::rebuildActiveReference()
{
    if (use_original_like_reference_mode_)
    {
        if (!(has_reference_path_ && smooth_ref_path_.points.size() >= 2))
        {
            selected_reference_active_ = false;
            ref_path_.clear();
            return;
        }

        selected_reference_active_ = true;
        ref_path_ = buildLocalReferenceWindow(smooth_ref_path_);
        return;
    }

    const bool has_selected = has_reference_path_ && smooth_ref_path_.points.size() >= 2;
    const bool has_topo = topo_ref_path_.points.size() >= 2;
    const bool freeze_switching = freezeReferenceSwitching();
    const ReferencePath *base_path = nullptr;
    bool engage_selected = false;

    if (has_selected)
    {
        if (selected_reference_active_)
        {
            engage_selected = true;
            base_path = &smooth_ref_path_;
        }
        else if (!freeze_switching &&
                 (!has_topo || (!topo_reference_locked_ && referenceHandoffReady(smooth_ref_path_, topo_ref_path_))))
        {
            engage_selected = true;
            base_path = &smooth_ref_path_;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1500,
                "USV-%d selected reference handoff accepted", agent_id_);
        }
        else
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1500,
                freeze_switching
                    ? "USV-%d selected reference frozen, keep topo basin"
                    : "USV-%d selected reference pending, keep topo basin",
                agent_id_);
        }
    }

    if (!base_path && has_topo)
        base_path = &topo_ref_path_;
    if (!base_path && has_selected)
    {
        engage_selected = true;
        base_path = &smooth_ref_path_;
    }

    selected_reference_active_ = engage_selected;

    if (!base_path)
    {
        ref_path_.clear();
        return;
    }

    ref_path_ = buildLocalReferenceWindow(*base_path);
}

void USVPlanner::neighborCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg, int neighbor_id)
{
    if (msg->data.size() < 4) return;

    auto &nb = neighbors_[neighbor_id];
    nb.id = neighbor_id;
    nb.current_state.x = msg->data[0];
    nb.current_state.y = msg->data[1];
    nb.current_state.psi = msg->data[2];
    nb.current_state.v = msg->data[3];
    nb.current_state.received = true;

    nb.predicted_traj.clear();
    nb.predicted_traj.dt = 0.4;
    for (size_t i = 4; i + 4 < msg->data.size(); i += 5)
    {
        nb.predicted_traj.add(msg->data[i], msg->data[i + 1], msg->data[i + 2],
                              msg->data[i + 3], msg->data[i + 4]);
    }
    nb.received = true;
}

void USVPlanner::dynamicObstacleCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg, const std::string &name, double radius)
{
    auto &obs = dynamic_obstacles_[name];
    obs.name = name;
    obs.x = msg->pose.pose.position.x;
    obs.y = msg->pose.pose.position.y;
    obs.vx = msg->twist.twist.linear.x;
    obs.vy = msg->twist.twist.linear.y;
    obs.radius = radius;
    obs.received = true;
    obs.last_update_sec = rclcpp::Time(msg->header.stamp).seconds();
}

void USVPlanner::controlLoop()
{
    // [DBG] entry
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "[DBG_ENTER] USV-%d controlLoop has_odom=%d has_goal=%d metrics_written=%d ego=(%.1f,%.1f)",
        agent_id_, has_odom_, has_goal_, metrics_written_, ego_.x, ego_.y);
    if (!has_odom_ || !has_goal_)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "[DBG_SKIP] USV-%d skip: odom=%d goal=%d", agent_id_, has_odom_, has_goal_);
        return;
    }

    last_time_ = this->now();
    updateEpisodeMetrics();

    double dxg = ego_.x - goal_x_;
    double dyg = ego_.y - goal_y_;
    double dist_to_goal = std::sqrt(dxg * dxg + dyg * dyg);
    double episode_elapsed = metrics_initialized_ ? (this->now() - episode_start_time_).seconds() : 0.0;

    if (!std::isfinite(last_progress_dist_to_goal_))
    {
        last_progress_dist_to_goal_ = dist_to_goal;
        last_progress_time_ = this->now();
    }
    else if (dist_to_goal < last_progress_dist_to_goal_ - 0.8)
    {
        last_progress_dist_to_goal_ = dist_to_goal;
        last_progress_time_ = this->now();
    }

    const double stagnation_s = (this->now() - last_progress_time_).seconds();
    if (!done_ && dist_to_goal > 6.0 && stagnation_s > 6.0)
    {
        stuck_recovery_active_ = true;
        stuck_recovery_until_ = this->now() + rclcpp::Duration::from_seconds(4.0);
        topology_hold_until_ = this->now() + rclcpp::Duration::from_seconds(4.0);
        last_progress_time_ = this->now();
        RCLCPP_WARN(this->get_logger(),
            "STUCK_RECOVERY usv=%d dist=%.2f stagnation=%.1fs cand=%d",
            agent_id_, dist_to_goal, stagnation_s, selected_topology_candidate_idx_);
    }
    if (stuck_recovery_active_ && this->now() > stuck_recovery_until_)
        stuck_recovery_active_ = false;

    std::vector<Obstacle> planning_obstacles = static_obstacles_;
    double nearest_dynamic = std::numeric_limits<double>::max();
    int dynamic_count = 0;
    const double now_sec = rclcpp::Time(this->now()).seconds();
    for (const auto &pair : dynamic_obstacles_)
    {
        const auto &obs = pair.second;
        if (!obs.received) continue;
        dynamic_count++;
        nearest_dynamic = std::min(
            nearest_dynamic, std::hypot(ego_.x - obs.x, ego_.y - obs.y) - obs.radius);
    }

    double neighbor_clearance = std::numeric_limits<double>::max();
    for (const auto &[id, nb] : neighbors_)
    {
        (void)id;
        if (!nb.received) continue;
        double clearance = std::hypot(ego_.x - nb.current_state.x, ego_.y - nb.current_state.y) - 2.0;
        neighbor_clearance = std::min(neighbor_clearance, clearance);
    }

    const double stop_clearance = dynamicStopClearance(controller_.params().stop_radius);
    const double slow_clearance = dynamicSlowClearance(mpc_solver_.params().obstacle_clearance);
    const double extra_dynamic_guard = dynamicGuardFromClearance(
        nearest_dynamic,
        controller_.params().stop_radius,
        mpc_solver_.params().obstacle_clearance);

    for (const auto &pair : dynamic_obstacles_)
    {
        const auto &obs = pair.second;
        if (!obs.received) continue;

        const double staleness = now_sec - obs.last_update_sec;
        const double dvx = (staleness > 1.2) ? 0.0 : obs.vx;
        const double dvy = (staleness > 1.2) ? 0.0 : obs.vy;

        for (int step = 0; step < 4; ++step)
        {
            const double t = 0.9 * static_cast<double>(step);
            const double px = obs.x + dvx * t;
            const double py = obs.y + dvy * t;
            const double rr = obs.radius + 0.18 * step +
                              extra_dynamic_guard * stageDangerWeight(step, 4);
            planning_obstacles.push_back(Obstacle{px, py, rr});
        }
    }
    if (dynamic_count > 0)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "DYNOBS cnt=%d nearest=%.2f guard=%.2f total_obs=%zu",
            dynamic_count, nearest_dynamic, extra_dynamic_guard,
            planning_obstacles.size());
    }

    if (near_goal_cooldown_ > 0) {
        near_goal_cooldown_--;
    }

    bool near_goal = (dist_to_goal < 2.6)
                     && (this->now() > reset_guard_until_)
                     && (near_goal_cooldown_ == 0);
    if (near_goal && !metrics_written_)
    {
        RCLCPP_INFO(this->get_logger(), "USV-%d arrived (dist=%.2f near=%d) ✓",
            agent_id_, dist_to_goal, (int)near_goal);
        done_ = true;
        writeEpisodeMetricsIfNeeded(false);

        std::string done_path = "/home/lu/paper2/core_ws/trial_done_" + std::to_string(agent_id_) + ".flag";
        std::ofstream done_flag(done_path);
        if (done_flag.is_open())
            done_flag << "arrived\n";
    }

    if (!metrics_written_ && episode_elapsed > episode_timeout_s_)
    {
        RCLCPP_WARN(this->get_logger(), "EP timeout usv=%d elapsed=%.1f x=%.1f dist=%.2f — forcing stop",
            agent_id_, episode_elapsed, ego_.x, dist_to_goal);
        done_ = true;
        writeEpisodeMetricsIfNeeded(true);

        std::string done_path = "/home/lu/paper2/core_ws/trial_done_" + std::to_string(agent_id_) + ".flag";
        std::ofstream done_flag(done_path);
        if (done_flag.is_open())
            done_flag << "timeout\n";
    }

    if (!metrics_written_)
    {
        if (optimizing_.exchange(true))
        {
            auto stuck_duration = std::chrono::steady_clock::now() - solver_start_time_;
            if (stuck_duration > std::chrono::seconds(5))
            {
                RCLCPP_WARN(this->get_logger(), "USV-%d solver STUCK for %.1fs — force reset",
                    agent_id_, std::chrono::duration<double>(stuck_duration).count());
                acados_solver_was_reset_ = true;
                optimizing_ = false;
            }
            else if (last_output_.size() > 0)
            {
                publishCommand(last_output_);
                publishTrajectoryPath(last_output_);
            }
            return;
        }
        solver_start_time_ = std::chrono::steady_clock::now();
        acados_solver_was_reset_ = false;

        std::vector<NeighborTrajectory> nb_list;
    for (const auto &pair : neighbors_)
        nb_list.push_back(pair.second);
    auto active_ids = neighbor_selector_.select(ego_, nb_list);

    const bool has_selected_ref = has_reference_path_ && smooth_ref_path_.points.size() >= 2;
    const bool has_topo_ref = !use_original_like_reference_mode_ && topo_ref_path_.points.size() >= 2;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "[DBG_REF] USV-%d has_selected_ref=%d has_topo_ref=%d ref_pts=%zu topo_pts=%zu metrics_written=%d",
        agent_id_, has_selected_ref, has_topo_ref,
        smooth_ref_path_.points.size(), topo_ref_path_.points.size(),
        metrics_written_);
    if (!has_selected_ref && !has_topo_ref)
    {
        mpc_solver_.clearWarmstart();
        if (last_output_.size() > 0)
            last_output_.clear();
        last_predicted_traj_.clear();
        auto cmd = geometry_msgs::msg::Twist();
        pub_cmd_vel_->publish(cmd);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1500,
            "USV-%d waiting reference path, hold position", agent_id_);
        return;
    }

    auto min_obstacle_margin = [&](const Trajectory &traj) -> double
    {
        double best = std::numeric_limits<double>::max();
        const size_t limit = std::min<size_t>(6, traj.points.size());
        for (size_t i = 0; i < limit; ++i)
        {
            const auto &pt = traj.points[i];
            for (const auto &obs : planning_obstacles)
            {
                const double safe = obs.radius + mpc_solver_.params().obstacle_clearance;
                const double margin = std::hypot(pt.x - obs.x, pt.y - obs.y) - safe;
                best = std::min(best, margin);
            }
        }
        return best;
    };

    auto min_neighbor_margin = [&](const Trajectory &traj) -> double
    {
        double best = std::numeric_limits<double>::max();
        const size_t limit = std::min<size_t>(6, traj.points.size());
        for (const auto &nb : nb_list)
        {
            if (!nb.received)
                continue;

            const double nb_dt = nb.predicted_traj.dt > 1e-6
                ? nb.predicted_traj.dt
                : std::max(0.05, mpc_solver_.params().dt);
            for (size_t i = 0; i < limit; ++i)
            {
                const auto &pt = traj.points[i];
                const double t = static_cast<double>(i) * std::max(0.05, traj.dt);
                double nx = nb.current_state.x;
                double ny = nb.current_state.y;
                if (nb.predicted_traj.size() > 0)
                {
                    int look = std::clamp(
                        static_cast<int>(std::round(t / std::max(0.05, nb_dt))),
                        0, nb.predicted_traj.size() - 1);
                    nx = nb.predicted_traj.points[look].x;
                    ny = nb.predicted_traj.points[look].y;
                }
                else
                {
                    nx += nb.current_state.v * std::cos(nb.current_state.psi) * t;
                    ny += nb.current_state.v * std::sin(nb.current_state.psi) * t;
                }
                const double margin = std::hypot(pt.x - nx, pt.y - ny) -
                                      2.0 * controller_.params().stop_radius;
                best = std::min(best, margin);
            }
        }
        return best;
    };

    auto run_mpc_for_ref = [&](const ReferencePath &active_ref) -> Trajectory
    {
        ReferencePath contour_ref = buildContouringReferenceWindow(active_ref);

        StageConstraintSet stage_cons = constraint_builder_.build(
            contour_ref, ego_, dynamic_obstacles_, neighbors_,
            static_obstacles_, active_ids,
            agent_id_, acados_solver_.params().robot_radius,
            acados_solver_.params().obstacle_clearance,
            acados_solver_.params().dt, acados_solver_.params().N);

        const double original_desired_speed = acados_solver_.params().desired_speed;
        
        if (colregs_enabled_)
        {
        auto colregs = colregs_engine_.compute(ego_, neighbors_, agent_id_);
        last_colregs_rule_ = colregs.rule;
        if (colregs.rule != COLREGsEngine::NONE)
        {
            double new_speed;
            if (colregs.speed_factor > 1.0)
            {
                // STAND-ON: 加速拉开距离，上限 desired × 1.20
                new_speed = std::min(
                    original_desired_speed * colregs.speed_factor,
                    original_desired_speed * 1.20);
            }
            else
            {
                // GIVE-WAY / HEAD-ON: 减速
                new_speed = std::clamp(
                    original_desired_speed * colregs.speed_factor, 0.15, original_desired_speed);
            }
            acados_solver_.params_mutable().desired_speed = new_speed;

            if (std::abs(new_speed - original_desired_speed) > 0.01)
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                    "COLREGS usv=%d rule=%d factor=%.2f speed=%.2f→%.2f",
                    agent_id_, (int)colregs.rule, colregs.speed_factor,
                    original_desired_speed, new_speed);
        }
        }

        double dyn_nearest = currentNearestDynamicDistance();
        if (dyn_nearest < 25.0 && dyn_nearest > 0.0)
        {
            double dyn_scale = std::clamp((dyn_nearest - 2.0) / 23.0, 0.30, 1.0);
            double new_speed = original_desired_speed * dyn_scale;
            new_speed = std::clamp(new_speed, 0.20, original_desired_speed);
            if (new_speed < acados_solver_.params().desired_speed)
            {
                acados_solver_.params_mutable().desired_speed = new_speed;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "DYN_SPEED usv=%d nearest=%.1f scale=%.2f desired=%.2f→%.2f",
                    agent_id_, dyn_nearest, dyn_scale,
                    original_desired_speed, new_speed);
            }
        }

        Trajectory traj = acados_solver_.solve(ego_, contour_ref, stage_cons);
        last_predicted_traj_ = traj;
        acados_solver_.params_mutable().desired_speed = original_desired_speed;
        return traj;
    };

    struct CandidateEval
    {
        bool valid = false;
        int candidate_index = -1;
        ReferencePath local_ref;
        Trajectory mpc_traj;
        Trajectory final_traj;
        double obstacle_margin = std::numeric_limits<double>::max();
        double neighbor_margin = std::numeric_limits<double>::max();
        double goal_remain = std::numeric_limits<double>::max();
        double goal_progress = 0.0;
        double obstacle_violation = 0.0;
        double neighbor_violation = 0.0;
        double obstacle_soft_penalty = 0.0;
        double neighbor_soft_penalty = 0.0;
        double diversity = 0.0;
        double heading_gap = 0.0;
        double score = std::numeric_limits<double>::max();
    };

    Trajectory mpc_traj;
    Trajectory final_traj;
    bool reranked = false;

    if (enable_topology_rerank_ &&
        !use_original_like_reference_mode_ &&
        static_cast<int>(topo_candidates_.size()) >= 2)
    {
        std::vector<CandidateEval> evals;
        evals.reserve(std::min<int>(topology_rerank_candidates_, topo_candidates_.size()));

        for (int idx = 0; idx < std::min<int>(topology_rerank_candidates_, topo_candidates_.size()); ++idx)
        {
            CandidateEval eval;
            eval.candidate_index = idx;
            eval.local_ref = buildLocalReferenceWindow(topo_candidates_[idx].path);
            if (eval.local_ref.points.size() < 2)
                continue;

            mpc_solver_.clearWarmstart();
            eval.mpc_traj = run_mpc_for_ref(eval.local_ref);
            if (eval.mpc_traj.size() == 0)
                continue;

            eval.final_traj = eval.mpc_traj;
            eval.obstacle_margin = min_obstacle_margin(eval.final_traj);
            eval.neighbor_margin = min_neighbor_margin(eval.final_traj);
            const auto &tail = eval.final_traj.points.back();
            eval.goal_remain = std::hypot(tail.x - goal_x_, tail.y - goal_y_);
            eval.goal_progress = std::max(0.0, dist_to_goal - eval.goal_remain);
            eval.obstacle_violation = std::max(0.0, -eval.obstacle_margin);
            eval.neighbor_violation = std::max(0.0, -eval.neighbor_margin);
            eval.obstacle_soft_penalty = std::max(0.0, 0.8 - eval.obstacle_margin);
            eval.neighbor_soft_penalty = std::max(0.0, 0.6 - eval.neighbor_margin);
            if (idx > 0 && !topo_candidates_.empty())
            {
                eval.diversity = sampledPathDistance(
                    topo_candidates_[idx].path,
                    topo_candidates_.front().path);
                eval.heading_gap = sampledHeadingGap(
                    topo_candidates_[idx].path,
                    topo_candidates_.front().path);
            }
            // COLREGs bias: compute lateral deviation of this path
            double colregs_bias = 0.0;
            if (colregs_enabled_ && last_colregs_rule_ != COLREGsEngine::NONE)
            {
                // Compute average signed lateral deviation of the candidate path
                const double cos_psi = std::cos(ego_.psi);
                const double sin_psi = std::sin(ego_.psi);
                double avg_cross = 0.0;
                int n_pts = 0;
                for (const auto &pt : topo_candidates_[idx].path.points)
                {
                    double dx = pt.x() - ego_.x;
                    double dy = pt.y() - ego_.y;
                    double cross = -dx * sin_psi + dy * cos_psi;  // + = left, - = right
                    avg_cross += cross;
                    ++n_pts;
                }
                avg_cross /= std::max(n_pts, 1);
                const bool path_goes_left = (avg_cross > 1.0);
                const bool path_goes_right = (avg_cross < -1.0);
                switch (last_colregs_rule_)
                {
                case COLREGsEngine::HEAD_ON:
                    // 对头：左右绕都加分，直行扣分
                    if (path_goes_left || path_goes_right)
                        colregs_bias = -1.5;
                    else
                        colregs_bias = 1.5;
                    break;
                case COLREGsEngine::CROSSING_GIVE_WAY:
                    // 让路：右绕加分（给对方让路），左绕或直行扣分
                    if (path_goes_right)
                        colregs_bias = -2.0;
                    else if (path_goes_left)
                        colregs_bias = 1.0;
                    else
                        colregs_bias = 0.5;
                    break;
                case COLREGsEngine::CROSSING_STAND_ON:
                    // 直行船：保持航向，左右绕扣分
                    if (!path_goes_left && !path_goes_right)
                        colregs_bias = -1.0;
                    else
                        colregs_bias = 1.0;
                    break;
                case COLREGsEngine::OVERTAKING:
                    // 超车：左绕加速超，右绕扣分
                    if (path_goes_left)
                        colregs_bias = -2.5;
                    else if (path_goes_right)
                        colregs_bias = 1.0;
                    else
                        colregs_bias = 0.5;
                    break;
                default:
                    break;
                }
            }
            eval.score = eval.goal_remain
                + 18.0 * eval.obstacle_violation
                + 14.0 * eval.neighbor_violation
                + 1.6 * eval.obstacle_soft_penalty
                + 1.2 * eval.neighbor_soft_penalty
                - 0.90 * eval.goal_progress
                + colregs_bias;
            eval.valid = true;
            evals.push_back(eval);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1500,
                "RERANK_EVAL usv=%d cand=%d div=%.2f hdg=%.2f obs=%.2f nb=%.2f prog=%.2f osoft=%.2f nsoft=%.2f score=%.2f",
                agent_id_, eval.candidate_index,
                eval.diversity, eval.heading_gap,
                eval.obstacle_margin, eval.neighbor_margin,
                eval.goal_progress,
                eval.obstacle_soft_penalty, eval.neighbor_soft_penalty,
                eval.score);
        }

        if (!evals.empty())
        {
            auto best_it = std::min_element(
                evals.begin(), evals.end(),
                [](const CandidateEval &a, const CandidateEval &b) { return a.score < b.score; });
            auto chosen_it = best_it;
            if (selected_topology_candidate_idx_ >= 0)
            {
                auto current_it = std::find_if(
                    evals.begin(), evals.end(),
                    [&](const CandidateEval &eval) {
                        return eval.candidate_index == selected_topology_candidate_idx_;
                    });
                if (current_it != evals.end() &&
                    (this->now() < topology_hold_until_ ||
                     current_it->score <= best_it->score + topology_rerank_switch_margin_ + (stuck_recovery_active_ ? 1.2 : 0.4)))
                {
                    chosen_it = current_it;
                }
            }

            smooth_ref_path_ = topo_candidates_[chosen_it->candidate_index].path;
            has_reference_path_ = true;
            selected_reference_active_ = true;
            selected_topology_candidate_idx_ = chosen_it->candidate_index;
            ref_path_ = chosen_it->local_ref;
            mpc_traj = chosen_it->mpc_traj;
            final_traj = chosen_it->final_traj;
            reranked = true;

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1500,
                "RERANK usv=%d cand=%d obs=%.2f nb=%.2f goal=%.2f prog=%.2f score=%.2f sel_prev=%d switch_margin=%.2f",
                agent_id_, chosen_it->candidate_index,
                chosen_it->obstacle_margin, chosen_it->neighbor_margin,
                chosen_it->goal_remain, chosen_it->goal_progress, chosen_it->score,
                selected_topology_candidate_idx_, topology_rerank_switch_margin_);
        }
    }

    if (!reranked)
    {
        rebuildActiveReference();
        if (ref_path_.points.size() < 2)
        {
            mpc_solver_.clearWarmstart();
            optimizing_ = false;
            return;
        }

        mpc_traj = runMPC();
        if (mpc_traj.size() == 0)
        {
            optimizing_ = false;
            return;
        }

        final_traj = mpc_traj;
    }

    for (int i = 0; i < final_traj.size() && i < mpc_traj.size(); ++i)
    {
        final_traj.points[i].v = std::min(final_traj.points[i].v, mpc_traj.points[i].v);
        final_traj.points[i].v = std::min(final_traj.points[i].v, mpc_solver_.params().max_vel);
    }

    static int cmp_log_count = 0;
    if (cmp_log_count++ < 20 && mpc_traj.size() > 1 && final_traj.size() > 1)
    {
        const auto &m1 = mpc_traj.points[1];
        const auto &f1 = final_traj.points[1];
        RCLCPP_INFO(this->get_logger(),
            "PATHCMP ctrl=%s | p1 mpc(%.2f,%.2f,%.2f,%.2f,%.2f) final(%.2f,%.2f,%.2f,%.2f,%.2f) | neighbors=%zu",
            control_mode_.c_str(),
            m1.x, m1.y, m1.psi, m1.v, mpc_traj.points[0].omega,
            f1.x, f1.y, f1.psi, f1.v, final_traj.points[0].omega,
            active_ids.size());
    }

    if (final_traj.size() >= 2)
    {
        const auto &tail = final_traj.points.back();
        const auto &prev = final_traj.points[final_traj.size() - 2];
        double goal_heading = std::atan2(goal_y_ - tail.y, goal_x_ - tail.x);
        double tail_seg_heading = std::atan2(tail.y - prev.y, tail.x - prev.x);
        double heading_err = Pose2D::normalizeAngle(tail_seg_heading - goal_heading);
        if (std::abs(heading_err) > 0.6)
        {
            final_traj.points.back().psi = goal_heading;
            final_traj.points.back().omega = 0.0;
        }
    }

    double nearest_obstacle_clearance = std::numeric_limits<double>::max();
    for (const auto &obs : planning_obstacles)
    {
        double clearance = std::hypot(ego_.x - obs.x, ego_.y - obs.y) -
                           (obs.radius + controller_.params().stop_radius);
        nearest_obstacle_clearance = std::min(nearest_obstacle_clearance, clearance);
    }

    const double hazard_clearance = std::min(nearest_dynamic - extra_dynamic_guard, nearest_obstacle_clearance);
    if (final_traj.size() > 0)
    {
        double min_speed_scale = 1.0;
        double min_path_clearance = std::numeric_limits<double>::max();
        const size_t limit = std::min<size_t>(4, final_traj.points.size());

        for (size_t i = 0; i < limit; ++i)
        {
            const auto &pt = final_traj.points[i];
            double point_clearance = std::numeric_limits<double>::max();
            for (const auto &obs : planning_obstacles)
            {
                double clearance = std::hypot(pt.x - obs.x, pt.y - obs.y) -
                                   (obs.radius + controller_.params().stop_radius);
                point_clearance = std::min(point_clearance, clearance);
            }
            for (const auto &[id, nb] : neighbors_)
            {
                (void)id;
                if (!nb.received) continue;
                double clearance = std::hypot(pt.x - nb.current_state.x,
                                              pt.y - nb.current_state.y) - 2.0;
                point_clearance = std::min(point_clearance, clearance);
            }

            if (!std::isfinite(point_clearance))
                continue;

            min_path_clearance = std::min(min_path_clearance, point_clearance);
            double point_scale = 1.0;
            if (point_clearance <= 0.0)
            {
                point_scale = 0.25;
            }
            else if (point_clearance <= stop_clearance * 0.5)
            {
                point_scale = 0.25 + 0.10 * static_cast<double>(i);
                point_scale = std::min(point_scale, 0.50);
            }
            else if (point_clearance <= stop_clearance)
            {
                point_scale = (i == 0) ? 0.20 : 0.30 + 0.10 * static_cast<double>(i);
                point_scale = std::min(point_scale, 0.65);
            }
            else if (point_clearance < slow_clearance)
            {
                point_scale = std::clamp(
                    (point_clearance - stop_clearance) / std::max(slow_clearance - stop_clearance, 1e-3),
                    0.35, 1.0);
            }
            min_speed_scale = std::min(min_speed_scale, point_scale);
        }

        if (std::isfinite(hazard_clearance))
        {
            if (hazard_clearance <= stop_clearance && min_speed_scale > 0.25)
                min_speed_scale = std::max(0.25, min_speed_scale * 0.50);
            else if (hazard_clearance < slow_clearance)
                min_speed_scale = std::min(min_speed_scale, std::clamp(
                    (hazard_clearance - stop_clearance) / std::max(slow_clearance - stop_clearance, 1e-3),
                    0.40, 1.0));
        }

        {
            double lead_speed = 999.0;
            Eigen::Vector2d ego_forward(std::cos(ego_.psi), std::sin(ego_.psi));
            for (const auto &[id, nb] : neighbors_)
            {
                (void)id;
                if (!nb.received) continue;
                Eigen::Vector2d to_nb(nb.current_state.x - ego_.x, nb.current_state.y - ego_.y);
                double along_track = to_nb.dot(ego_forward);
                double dist = to_nb.norm();
                if (along_track > 0.0 && dist > 5.0 && dist < 30.0 &&
                    nb.current_state.v < acados_solver_.params().desired_speed * 0.70)
                {
                    lead_speed = std::min(lead_speed, nb.current_state.v);
                }
            }
            if (lead_speed < 900.0)
            {
                double follow_speed = std::max(lead_speed, acados_solver_.params().desired_speed * 0.25);
                double current_speed = ego_.v;
                if (current_speed > follow_speed + 0.2)
                {
                    double follow_scale = follow_speed / std::max(current_speed, 0.01);
                    min_speed_scale = std::min(min_speed_scale, std::clamp(follow_scale, 0.30, 0.90));
                }
            }
        }

        {
            bool is_stand_on = (last_colregs_rule_ == COLREGsEngine::CROSSING_STAND_ON);
            if (!is_stand_on)
            {
                double neighbor_yield_scale = 1.0;
                Eigen::Vector2d ego_forward(std::cos(ego_.psi), std::sin(ego_.psi));
                for (const auto &[id, nb] : neighbors_)
                {
                    (void)id;
                    if (!nb.received) continue;
                    Eigen::Vector2d to_nb(nb.current_state.x - ego_.x, nb.current_state.y - ego_.y);
                    double dist = to_nb.norm();
                    double ahead = to_nb.dot(ego_forward);
                    if (ahead > 0.0 && dist < neighbor_selector_.danger_distance)
                    {
                        double scale = std::clamp(dist / std::max(neighbor_selector_.danger_distance, 1e-3), 0.35, 1.0);
                        neighbor_yield_scale = std::min(neighbor_yield_scale, scale);
                    }
                }
                min_speed_scale = std::min(min_speed_scale, neighbor_yield_scale);
            }
        }

        if (min_speed_scale < 1.0)
        {
            double speed_floor = 0.25;
            for (size_t i = 0; i < limit; ++i)
            {
                double ramp = (i == 0) ? min_speed_scale :
                    std::max(min_speed_scale, 0.35 + 0.15 * static_cast<double>(i));
                double point_scale = std::clamp(ramp, speed_floor, 1.0);
                final_traj.points[i].v *= point_scale;
            }
        }
    }

    last_output_ = final_traj;
    publishCommand(final_traj);
    publishTrajectoryPath(final_traj);
    publishVisualization(final_traj);

    double log_clr_dyn = (std::isfinite(nearest_dynamic) && nearest_dynamic < 900.0) ? nearest_dynamic : 999.0;
    double log_clr_st = (std::isfinite(nearest_obstacle_clearance) && nearest_obstacle_clearance < 900.0) ? nearest_obstacle_clearance : 999.0;
    double log_clr_nb = (std::isfinite(neighbor_clearance) && neighbor_clearance < 900.0) ? neighbor_clearance : 999.0;
    {
        const char *csv_env = std::getenv("MULTI_USV_RESULTS_CSV");
        std::string csv_path = csv_env ? std::string(csv_env) : "/home/lu/paper2/core_ws/results_multi_usv.csv";
        int fd = ::open(csv_path.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0644);
        if (fd >= 0) {
            ::close(fd);
            std::ofstream hdr(csv_path, std::ios::app);
            if (hdr.is_open())
                hdr << "t,agent,colregs,coll_dyn,coll_st,coll_nb,clr_dyn,clr_st,clr_nb,speed\n";
        }
        std::ofstream csv(csv_path, std::ios::app);
        if (csv.is_open()) {
            csv << std::fixed << std::setprecision(3) << this->now().seconds() << ',' << agent_id_ << ','
                << (int)last_colregs_rule_ << ','
                << (collision_dynamic_?1:0) << ',' << (collision_static_?1:0) << ',' << (collision_neighbor_?1:0) << ','
                << log_clr_dyn << ',' << log_clr_st << ',' << log_clr_nb << ','
                << ego_.v << '\n';
        }
    }

    if (agent_id_ <= 4 && mpc_traj.size() > 1)
    {
        const auto &p1 = mpc_traj.points[1];
        double min_dist_barrier = 1e9;
        for (const auto &obs : planning_obstacles)
        {
            double d = std::hypot(p1.x - obs.x, p1.y - obs.y);
            double safe = obs.radius + mpc_solver_.params().obstacle_clearance;
            double margin = d - safe;
            min_dist_barrier = std::min(min_dist_barrier, margin);
        }
    }


    optimizing_ = false;
    } // end of if (!metrics_written_)
}

void USVPlanner::updateEpisodeMetrics()
{
    for (const auto &obs : static_obstacles_)
    {
        double clearance = std::hypot(ego_.x - obs.x, ego_.y - obs.y) - obs.radius;
        min_static_clearance_ = std::min(min_static_clearance_, clearance);
        if (clearance <= 0.0)  // center inside obstacle → collision
            collision_static_ = true;
    }

    for (const auto &pair : dynamic_obstacles_)
    {
        const auto &obs = pair.second;
        if (!obs.received) continue;
        double clearance = std::hypot(ego_.x - obs.x, ego_.y - obs.y) - (obs.radius + 1.0);
        min_dynamic_clearance_ = std::min(min_dynamic_clearance_, clearance);
        if (clearance <= 0.0)
            collision_dynamic_ = true;
    }

    for (const auto &[id, nb] : neighbors_)
    {
        if (!nb.received) continue;
        double clearance = std::hypot(ego_.x - nb.current_state.x, ego_.y - nb.current_state.y) - 2.0;  // sum of vehicle radii
        min_neighbor_clearance_ = std::min(min_neighbor_clearance_, clearance);
        if (clearance <= 0.0)
            collision_neighbor_ = true;
    }
}

void USVPlanner::writeEpisodeMetricsIfNeeded(bool timeout)
{
    if (!metrics_initialized_ || metrics_written_) return;
    metrics_written_ = true;

    const double duration = (this->now() - episode_start_time_).seconds();
    const char *csv_env = std::getenv("MULTI_USV_RESULTS_CSV");
    const std::string path = csv_env ? std::string(csv_env) : "/home/lu/paper2/core_ws/results_multi_usv.csv";
    std::ofstream out(path, std::ios::app);
    if (!out.is_open()) return;
    int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0644);
    if (fd >= 0) {
        ::close(fd);
        out << "agent_id,duration_s,timeout,collision_dynamic,collision_static,collision_neighbor,min_dynamic_clearance,min_static_clearance,min_neighbor_clearance\n";
    }
    out << agent_id_ << ',' << duration << ',' << (timeout ? 1 : 0) << ','
        << (collision_dynamic_ ? 1 : 0) << ',' << (collision_static_ ? 1 : 0) << ',' << (collision_neighbor_ ? 1 : 0) << ','
        << min_dynamic_clearance_ << ',' << min_static_clearance_ << ',' << min_neighbor_clearance_ << '\n';
}

Trajectory USVPlanner::runMPC()
{
    rebuildActiveReference();
    if (ref_path_.points.size() < 2)
        return Trajectory();

    ReferencePath contour_ref = buildContouringReferenceWindow(ref_path_);

    std::vector<NeighborTrajectory> nb_list;
    for (const auto &pair : neighbors_)
        nb_list.push_back(pair.second);
    auto active_ids = neighbor_selector_.select(ego_, nb_list);

    StageConstraintSet stage_cons = constraint_builder_.build(
        contour_ref, ego_, dynamic_obstacles_, neighbors_,
        static_obstacles_, active_ids,
        agent_id_, acados_solver_.params().robot_radius,
        acados_solver_.params().obstacle_clearance,
        acados_solver_.params().dt, acados_solver_.params().N);

    const double ods = acados_solver_.params().desired_speed;
    if (colregs_enabled_)
    {
    auto colregs = colregs_engine_.compute(ego_, neighbors_, agent_id_);
    last_colregs_rule_ = colregs.rule;
    if (colregs.rule != COLREGsEngine::NONE)
    {
        double new_speed = std::clamp(
            ods * colregs.speed_factor, 0.15, ods);
        acados_solver_.params_mutable().desired_speed = new_speed;
        if (new_speed < ods * 0.99)
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "COLREGS usv=%d rule=%d scale=%.2f speed=%.2f→%.2f",
                agent_id_, (int)colregs.rule, colregs.speed_factor,
                ods, new_speed);
    }
    }

    double dyn_nearest = currentNearestDynamicDistance();
    if (dyn_nearest < 25.0 && dyn_nearest > 0.0)
    {
        double dyn_scale = std::clamp((dyn_nearest - 2.0) / 23.0, 0.30, 1.0);
        double new_speed = std::clamp(ods * dyn_scale, 0.20, ods);
        if (new_speed < acados_solver_.params().desired_speed)
            acados_solver_.params_mutable().desired_speed = new_speed;
    }

    Trajectory traj = acados_solver_.solve(ego_, contour_ref, stage_cons);

    last_predicted_traj_ = traj;

    acados_solver_.params_mutable().desired_speed = ods;
    return traj;
}

void USVPlanner::publishCommand(const Trajectory &traj)
{
    auto cmd = geometry_msgs::msg::Twist();

    if (use_direct_cmd_ && traj.size() > 0)
    {
        cmd.linear.x = std::clamp(traj.points[0].v, 0.0, controller_.params().max_u);
        cmd.angular.z = std::clamp(traj.points[0].omega, -controller_.params().max_r, controller_.params().max_r);
    }
    else
    {
        double dt_seconds = 0.1;
        auto [u_cmd, r_cmd] = controller_.compute(traj, ego_, dt_seconds);
        cmd.linear.x = u_cmd;
        cmd.angular.z = r_cmd;
    }

    if (colregs_enabled_)
    {
        auto colregs = colregs_engine_.compute(ego_, neighbors_, agent_id_);
        if (colregs.rule != COLREGsEngine::NONE)
        {
            cmd.linear.x *= colregs.speed_factor;
            cmd.linear.x = std::clamp(cmd.linear.x, 0.15, controller_.params().max_u);
        }
    }

    if (stuck_recovery_active_)
    {
        cmd.linear.x = std::max(cmd.linear.x, 0.28);
        cmd.angular.z = std::clamp(cmd.angular.z, -0.45, 0.45);
    }

    pub_cmd_vel_->publish(cmd);

    auto broadcast = std_msgs::msg::Float64MultiArray();
    broadcast.data.reserve(4 + traj.size() * 5);
    broadcast.data.push_back(ego_.x);
    broadcast.data.push_back(ego_.y);
    broadcast.data.push_back(ego_.psi);
    broadcast.data.push_back(ego_.v);
    for (const auto &pt : traj.points)
    {
        broadcast.data.push_back(pt.x);
        broadcast.data.push_back(pt.y);
        broadcast.data.push_back(pt.psi);
        broadcast.data.push_back(pt.v);
        broadcast.data.push_back(pt.omega);
    }
    pub_broadcast_->publish(broadcast);
}


void USVPlanner::publishTrajectoryPath(const Trajectory &traj)
{
    (void)traj;
    auto path = nav_msgs::msg::Path();
    path.header.stamp = this->now();
    path.header.frame_id = "odom";

    const ReferencePath *global_path = nullptr;
    if (smooth_ref_path_.points.size() >= 2)
        global_path = &smooth_ref_path_;
    else if (topo_ref_path_.points.size() >= 2)
        global_path = &topo_ref_path_;
    else if (ref_path_.points.size() >= 2)
        global_path = &ref_path_;

    if (!global_path)
    {
        pub_ref_path_->publish(path);
        return;
    }

    for (const auto &rp : global_path->points)
    {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = path.header;
        ps.pose.position.x = rp.x();
        ps.pose.position.y = rp.y();
        path.poses.push_back(ps);
    }
    pub_ref_path_->publish(path);
}

void USVPlanner::publishVisualization(const Trajectory &traj)
{
    visualization_msgs::msg::MarkerArray markers;
    const auto stamp = this->now();

    auto agent_color = [&](float alpha = 1.0f) {
        std_msgs::msg::ColorRGBA c;
        if (agent_id_ == 1) {
            c.r = 0.95f; c.g = 0.30f; c.b = 0.30f;
        } else if (agent_id_ == 2) {
            c.r = 0.25f; c.g = 0.90f; c.b = 0.35f;
        } else if (agent_id_ == 3) {
            c.r = 0.25f; c.g = 0.45f; c.b = 0.95f;
        } else {
            c.r = 0.95f; c.g = 0.80f; c.b = 0.20f;
        }
        c.a = alpha;
        return c;
    };

    visualization_msgs::msg::Marker line;
    line.header.stamp = stamp;
    line.header.frame_id = "odom";
    line.ns = "trajectory";
    line.id = agent_id_ * 10;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.22;
    line.color = agent_color(0.88f);

    visualization_msgs::msg::Marker nodes;
    nodes.header = line.header;
    nodes.ns = "trajectory_nodes";
    nodes.id = agent_id_ * 10 + 1;
    nodes.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    nodes.action = visualization_msgs::msg::Marker::ADD;
    nodes.pose.orientation.w = 1.0;
    nodes.scale.x = 0.34;
    nodes.scale.y = 0.34;
    nodes.scale.z = 0.10;
    nodes.color = agent_color(0.0f);

    visualization_msgs::msg::Marker heading;
    heading.header = line.header;
    heading.ns = "trajectory_heading";
    heading.id = agent_id_ * 10 + 2;
    heading.type = visualization_msgs::msg::Marker::LINE_LIST;
    heading.action = visualization_msgs::msg::Marker::ADD;
    heading.pose.orientation.w = 1.0;
    heading.scale.x = 0.08;
    heading.color = agent_color(0.55f);

    for (size_t i = 0; i < traj.points.size(); ++i)
    {
        const auto &pt = traj.points[i];
        const float t = traj.points.size() > 1 ? static_cast<float>(i) / static_cast<float>(traj.points.size() - 1) : 1.0f;

        geometry_msgs::msg::Point p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = 0.03;
        line.points.push_back(p);
        nodes.points.push_back(p);
        nodes.colors.push_back(agent_color(0.15f + 0.75f * t));

        geometry_msgs::msg::Point p2 = p;
        p2.x += 0.65 * std::cos(pt.psi);
        p2.y += 0.65 * std::sin(pt.psi);
        heading.points.push_back(p);
        heading.points.push_back(p2);
    }
    markers.markers.push_back(line);
    markers.markers.push_back(nodes);
    markers.markers.push_back(heading);

    pub_viz_->publish(markers);

    if (pub_planned_traj_alias_)
        pub_planned_traj_alias_->publish(markers);

    if (ref_path_.points.size() >= 2)
    {
        visualization_msgs::msg::MarkerArray contour_path;
        visualization_msgs::msg::Marker path_line;
        path_line.header.stamp = stamp;
        path_line.header.frame_id = "odom";
        path_line.ns = "contouring_path";
        path_line.id = 0;
        path_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_line.action = visualization_msgs::msg::Marker::ADD;
        path_line.pose.orientation.w = 1.0;
        path_line.scale.x = 0.12;
        path_line.color = agent_color(0.42f);

        visualization_msgs::msg::Marker path_points = path_line;
        path_points.ns = "contouring_points";
        path_points.id = 1;
        path_points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        path_points.scale.x = 0.18;
        path_points.scale.y = 0.18;
        path_points.scale.z = 0.05;
        path_points.color = agent_color(0.0f);

        for (size_t i = 0; i < ref_path_.points.size(); ++i)
        {
            const auto &rp = ref_path_.points[i];
            const float t = ref_path_.points.size() > 1 ? static_cast<float>(i) / static_cast<float>(ref_path_.points.size() - 1) : 1.0f;
            geometry_msgs::msg::Point p;
            p.x = rp.x();
            p.y = rp.y();
            p.z = 0.02;
            path_line.points.push_back(p);
            path_points.points.push_back(p);
            path_points.colors.push_back(agent_color(0.08f + 0.45f * t));
        }
        contour_path.markers.push_back(path_line);
        contour_path.markers.push_back(path_points);
        if (pub_contouring_alias_)
            pub_contouring_alias_->publish(contour_path);
        if (pub_contouring_path_alias_)
            pub_contouring_path_alias_->publish(contour_path);
        if (pub_contouring_points_alias_)
        {
            visualization_msgs::msg::MarkerArray contour_points_only;
            contour_points_only.markers.push_back(path_points);
            pub_contouring_points_alias_->publish(contour_points_only);
        }
    }

    visualization_msgs::msg::MarkerArray contour_current;
    visualization_msgs::msg::Marker current;
    current.header.stamp = stamp;
    current.header.frame_id = "odom";
    current.ns = "contouring_current";
    current.id = 0;
    current.type = visualization_msgs::msg::Marker::ARROW;
    current.action = visualization_msgs::msg::Marker::ADD;
    current.scale.x = 0.9;
    current.scale.y = 0.22;
    current.scale.z = 0.22;
    current.color.r = 0.95f;
    current.color.g = 0.75f;
    current.color.b = 0.15f;
    current.color.a = 0.95f;
    current.pose.position.x = ego_.x;
    current.pose.position.y = ego_.y;
    current.pose.position.z = 0.05;
    current.pose.orientation.z = std::sin(ego_.psi * 0.5);
    current.pose.orientation.w = std::cos(ego_.psi * 0.5);
    contour_current.markers.push_back(current);
    if (pub_contouring_current_alias_)
        pub_contouring_current_alias_->publish(contour_current);

    if (pub_goal_module_alias_ && has_goal_)
    {
        visualization_msgs::msg::MarkerArray goal_markers;

        visualization_msgs::msg::Marker goal;
        goal.header.stamp = stamp;
        goal.header.frame_id = "odom";
        goal.ns = "goal_module";
        goal.id = 0;
        goal.type = visualization_msgs::msg::Marker::SPHERE;
        goal.action = visualization_msgs::msg::Marker::ADD;
        goal.pose.position.x = goal_x_;
        goal.pose.position.y = goal_y_;
        goal.pose.position.z = 0.15;
        goal.pose.orientation.w = 1.0;
        goal.scale.x = 0.8;
        goal.scale.y = 0.8;
        goal.scale.z = 0.8;
        goal.color.r = 0.2f;
        goal.color.g = 0.95f;
        goal.color.b = 0.2f;
        goal.color.a = 0.95f;
        visualization_msgs::msg::Marker link = goal;
        link.ns = "goal_module_link";
        link.id = 1;
        link.type = visualization_msgs::msg::Marker::LINE_STRIP;
        link.scale.x = 0.08;
        link.color.r = 0.9f;
        link.color.g = 0.9f;
        link.color.b = 0.2f;
        link.color.a = 0.75f;
        link.points.clear();
        geometry_msgs::msg::Point p0;
        p0.x = ego_.x;
        p0.y = ego_.y;
        p0.z = 0.06;
        geometry_msgs::msg::Point p1;
        p1.x = goal_x_;
        p1.y = goal_y_;
        p1.z = 0.06;
        link.points.push_back(p0);
        link.points.push_back(p1);
        goal_markers.markers.push_back(link);

        pub_goal_module_alias_->publish(goal_markers);
    }

    if (pub_dynamic_obs_alias_)
    {
        visualization_msgs::msg::MarkerArray dyn_markers;

        visualization_msgs::msg::Marker clear_all;
        clear_all.header.stamp = stamp;
        clear_all.header.frame_id = "odom";
        clear_all.action = visualization_msgs::msg::Marker::DELETEALL;
        dyn_markers.markers.push_back(clear_all);

        int marker_id = 0;
        const double now_sec = rclcpp::Time(this->now()).seconds();
        const float alphas[5] = {0.40f, 0.32f, 0.24f, 0.16f, 0.08f};
        for (const auto &pair : dynamic_obstacles_)
        {
            const auto &obs = pair.second;
            if (!obs.received)
                continue;
            if (now_sec - obs.last_update_sec > 1.5)
                continue;

            auto obstacle_color = [&](const std::string &name, float alpha) {
                std_msgs::msg::ColorRGBA c;
                const std::size_t h = std::hash<std::string>{}(name) % 6;
                switch (h)
                {
                case 0: c.r = 0.78f; c.g = 0.28f; c.b = 0.78f; break;
                case 1: c.r = 0.22f; c.g = 0.72f; c.b = 0.95f; break;
                case 2: c.r = 0.30f; c.g = 0.88f; c.b = 0.48f; break;
                case 3: c.r = 0.96f; c.g = 0.58f; c.b = 0.18f; break;
                case 4: c.r = 0.95f; c.g = 0.32f; c.b = 0.42f; break;
                default: c.r = 0.82f; c.g = 0.72f; c.b = 0.22f; break;
                }
                c.a = alpha;
                return c;
            };

            visualization_msgs::msg::Marker body;
            body.header.stamp = stamp;
            body.header.frame_id = "odom";
            body.ns = "pedestrians";
            body.type = visualization_msgs::msg::Marker::CYLINDER;
            body.action = visualization_msgs::msg::Marker::ADD;
            body.pose.orientation.w = 1.0;
            body.scale.x = 2.0 * obs.radius;
            body.scale.y = 2.0 * obs.radius;
            body.scale.z = 0.08;

            for (int k = 0; k < 5; ++k)
            {
                const double t = 0.45 * static_cast<double>(k);
                visualization_msgs::msg::Marker disc = body;
                disc.id = marker_id++;
                disc.pose.position.x = obs.x + obs.vx * t;
                disc.pose.position.y = obs.y + obs.vy * t;
                disc.pose.position.z = 0.03 + 0.006 * k;
                disc.scale.x = 2.0 * obs.radius * (1.0 + 0.06 * k);
                disc.scale.y = 2.0 * obs.radius * (1.0 + 0.06 * k);
                disc.color = obstacle_color(pair.first, alphas[k]);
                dyn_markers.markers.push_back(disc);
            }
        }
        pub_dynamic_obs_alias_->publish(dyn_markers);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<USVPlanner>());
    rclcpp::shutdown();
    return 0;
}
