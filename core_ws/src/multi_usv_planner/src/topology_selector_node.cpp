#include <multi_usv_planner/topology_selector.h>

#include <geometry_msgs/msg/point.hpp>
#include <cmath>
#include <limits>
#include <algorithm>
#include <future>

namespace MultiUSV
{

TopologySelectorNode::TopologySelectorNode()
    : Node("topology_selector_node")
{
    declareParams();
    initCommunication();
    hold_until_ = this->now();
}

void TopologySelectorNode::declareParams()
{
    this->declare_parameter("agent_id", 1);
    this->declare_parameter("neighbors", std::vector<int64_t>{2, 3, 4});
    this->declare_parameter("selection_hold_sec", 1.5);
    this->declare_parameter("consistency_penalty", 10.0);
    this->declare_parameter("topology_switch_penalty", 8.0);
    this->declare_parameter("weight_topology_length", 0.2);
    this->declare_parameter("obstacle_cost_gain", 80.0);
    this->declare_parameter("obstacle_safe_margin", 4.0);
    this->declare_parameter("static_obstacles_x", std::vector<double>{});
    this->declare_parameter("static_obstacles_y", std::vector<double>{});
    this->declare_parameter("static_obstacles_r", std::vector<double>{});
    this->declare_parameter("goal_x", 0.0);
    this->declare_parameter("goal_y", 0.0);
    this->declare_parameter("local_goal_horizon", 10.0);
    this->declare_parameter("weight_global_consistency", 25.0);
    this->declare_parameter("weight_initial_turn", 20.0);

    agent_id_ = this->get_parameter("agent_id").as_int();
    auto nb = this->get_parameter("neighbors").as_integer_array();
    for (auto id : nb) neighbor_ids_.push_back(static_cast<int>(id));
    selection_hold_sec_ = this->get_parameter("selection_hold_sec").as_double();
    consistency_penalty_ = this->get_parameter("consistency_penalty").as_double();
    topology_switch_penalty_ = this->get_parameter("topology_switch_penalty").as_double();
    weight_topology_length_ = this->get_parameter("weight_topology_length").as_double();
    obstacle_cost_gain_ = this->get_parameter("obstacle_cost_gain").as_double();
    obstacle_safe_margin_ = this->get_parameter("obstacle_safe_margin").as_double();
    goal_x_ = this->get_parameter("goal_x").as_double();
    goal_y_ = this->get_parameter("goal_y").as_double();
    local_goal_horizon_ = this->get_parameter("local_goal_horizon").as_double();
    weight_global_consistency_ = this->get_parameter("weight_global_consistency").as_double();
    weight_initial_turn_ = this->get_parameter("weight_initial_turn").as_double();
    static_obstacles_x_ = this->get_parameter("static_obstacles_x").as_double_array();
    static_obstacles_y_ = this->get_parameter("static_obstacles_y").as_double_array();
    static_obstacles_r_ = this->get_parameter("static_obstacles_r").as_double_array();
}

void TopologySelectorNode::initCommunication()
{
    const auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { odomCallback(msg); });
    sub_topo_ = this->create_subscription<topology_interfaces::msg::TopologicalPathArray>(
        "topological_path", 10, [this](const topology_interfaces::msg::TopologicalPathArray::SharedPtr msg) { topoCallback(msg); });

    const std::vector<std::pair<std::string, double>> dynamic_obstacle_specs = {
        {"vessel_e1", 2.8}, {"vessel_e2", 2.8}, {"vessel_f1", 3.0}, {"vessel_f2", 3.0},
    };
    for (const auto &[name, radius] : dynamic_obstacle_specs)
    {
        dynamic_obstacle_radii_[name] = radius;
        auto cb = [this, name, radius](const nav_msgs::msg::Odometry::SharedPtr msg)
        { dynamicObstacleCallback(msg, name, radius); };
        sub_dynamic_obstacles_.push_back(this->create_subscription<nav_msgs::msg::Odometry>(
            "/" + name + "/odom", 10, cb));
    }
    // Floating debris (debris_0 ~ debris_11)
    for (int i = 0; i < 12; i++)
    {
        std::string name = "debris_" + std::to_string(i);
        double radius = 0.45;
        dynamic_obstacle_radii_[name] = radius;
        auto cb = [this, name, radius](const nav_msgs::msg::Odometry::SharedPtr msg)
        { dynamicObstacleCallback(msg, name, radius); };
        sub_dynamic_obstacles_.push_back(this->create_subscription<nav_msgs::msg::Odometry>(
            "/" + name + "/odom", 10, cb));
    }

    pub_smooth_path_ = this->create_publisher<nav_msgs::msg::Path>("reference_path", latched_qos);
    pub_decision_ = this->create_publisher<topology_interfaces::msg::Decision>("topology_decision", 10);
    pub_candidate_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("topology_candidates", 10);
    pub_mode_ = this->create_publisher<std_msgs::msg::String>("planner_mode", 10);
}

void TopologySelectorNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    ego_odom_ = *msg;
    has_odom_ = true;
}

void TopologySelectorNode::dynamicObstacleCallback(const nav_msgs::msg::Odometry::SharedPtr msg, const std::string &name, double)
{
    dynamic_obstacles_[name] = *msg;
}

TopologySelectorNode::SignatureKey TopologySelectorNode::signatureKey(const std::vector<double> &sig) const
{
    if (sig.empty()) return SignatureKey{0.0};
    return SignatureKey{std::round(sig.front() * 1e6) / 1e6};
}

void TopologySelectorNode::topoCallback(const topology_interfaces::msg::TopologicalPathArray::SharedPtr msg)
{
    latest_candidates_ = *msg;
    has_candidates_ = !msg->topological_paths.empty();
    evaluateAndPublish();
}

void TopologySelectorNode::ensurePlannerPool(size_t count)
{
    if (planners_.size() < count)
        planners_.resize(count);
    for (auto &planner : planners_)
        planner.matched = false;
}

ReferencePath TopologySelectorNode::pathMsgToReference(const nav_msgs::msg::Path &path) const
{
    const nav_msgs::msg::Path clean = sanitizePath(path);
    ReferencePath ref;
    ref.points.reserve(clean.poses.size());
    for (const auto &pose : clean.poses)
        ref.points.emplace_back(pose.pose.position.x, pose.pose.position.y);
    ref.total_length = 0.0;
    for (size_t i = 1; i < ref.points.size(); ++i)
        ref.total_length += (ref.points[i] - ref.points[i - 1]).norm();
    ref.nominal_speed = 1.5;
    return ref;
}

nav_msgs::msg::Path TopologySelectorNode::sanitizePath(const nav_msgs::msg::Path &path, const USVState *ego) const
{
    nav_msgs::msg::Path clean = path;
    if (path.poses.size() < 2)
        return clean;

    std::vector<Eigen::Vector2d> pts;
    pts.reserve(path.poses.size() + 1);
    if (ego)
        pts.emplace_back(ego->x, ego->y);
    for (const auto &pose : path.poses)
    {
        Eigen::Vector2d p(pose.pose.position.x, pose.pose.position.y);
        if (pts.empty() || (p - pts.back()).norm() > 0.15)
            pts.push_back(p);
    }

    if (pts.size() < 2)
        return clean;

    size_t anchor_idx = std::min<size_t>(pts.size() - 1, 4);
    Eigen::Vector2d forward = pts[anchor_idx] - pts.front();
    if (forward.norm() < 1e-6 && ego)
        forward = Eigen::Vector2d(std::cos(ego->psi), std::sin(ego->psi));
    if (forward.norm() < 1e-6)
        forward = Eigen::Vector2d(1.0, 0.0);
    forward.normalize();

    std::vector<Eigen::Vector2d> pruned;
    pruned.reserve(pts.size());
    pruned.push_back(pts.front());
    double last_proj = 0.0;
    for (size_t i = 1; i < pts.size(); ++i)
    {
        double proj = (pts[i] - pts.front()).dot(forward);
        bool local_backtrack = (i <= 6 && proj < last_proj - 0.05);
        if (local_backtrack)
            continue;
        if ((pts[i] - pruned.back()).norm() > 0.15)
        {
            pruned.push_back(pts[i]);
            last_proj = proj;
        }
    }
    if (pruned.size() < 2)
        pruned = pts;

    std::vector<double> arc(pruned.size(), 0.0);
    for (size_t i = 1; i < pruned.size(); ++i)
        arc[i] = arc[i - 1] + (pruned[i] - pruned[i - 1]).norm();
    const double total = arc.back();
    if (total < 1e-6)
        return clean;

    std::vector<Eigen::Vector2d> resampled;
    const double ds = 0.6;
    for (double s = 0.0; s < total; s += ds)
    {
        size_t hi = 1;
        while (hi < arc.size() && arc[hi] < s)
            ++hi;
        if (hi >= arc.size())
            break;
        size_t lo = hi - 1;
        double seg = std::max(arc[hi] - arc[lo], 1e-6);
        double t = (s - arc[lo]) / seg;
        resampled.push_back(pruned[lo] + t * (pruned[hi] - pruned[lo]));
    }
    resampled.push_back(pruned.back());

    clean.poses.clear();
    clean.poses.reserve(resampled.size());
    for (const auto &p : resampled)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = p.x();
        pose.pose.position.y = p.y();
        pose.pose.orientation.w = 1.0;
        clean.poses.push_back(pose);
    }
    return clean;
}

Eigen::Vector2d TopologySelectorNode::computeLocalGoal(const ReferencePath &ref, const USVState &ego) const
{
    if (ref.points.empty()) return Eigen::Vector2d(goal_x_, goal_y_);
    double s0 = 0.0;
    ref.findClosest(Eigen::Vector2d(ego.x, ego.y), s0);
    double s_goal = std::min(s0 + local_goal_horizon_, ref.length());
    return ref.sample(s_goal);
}

double TopologySelectorNode::computeGlobalConsistencyCost(const nav_msgs::msg::Path &path, const USVState &ego) const
{
    if (path.poses.size() < 2) return 1e9;
    Eigen::Vector2d goal(goal_x_, goal_y_);
    Eigen::Vector2d ego_pos(ego.x, ego.y);
    Eigen::Vector2d task_dir = goal - ego_pos;
    double task_norm = task_dir.norm();
    if (task_norm < 1e-6) return 0.0;
    task_dir /= task_norm;

    const auto &p1 = path.poses[1].pose.position;
    Eigen::Vector2d cand_dir(p1.x - ego.x, p1.y - ego.y);
    double cand_norm = cand_dir.norm();
    if (cand_norm < 1e-6) return weight_global_consistency_;
    cand_dir /= cand_norm;

    double heading_penalty = weight_global_consistency_ * std::max(0.0, 1.0 - cand_dir.dot(task_dir));
    const auto &pend = path.poses.back().pose.position;
    double end_dist = std::hypot(pend.x - goal_x_, pend.y - goal_y_);
    return heading_penalty + 0.05 * end_dist;
}

double TopologySelectorNode::computeInitialTurnCost(const nav_msgs::msg::Path &path, const USVState &ego) const
{
    if (path.poses.size() < 2) return weight_initial_turn_;
    const auto &p1 = path.poses[1].pose.position;
    double path_heading = std::atan2(p1.y - ego.y, p1.x - ego.x);
    double dpsi = Pose2D::normalizeAngle(path_heading - ego.psi);
    return weight_initial_turn_ * std::abs(dpsi);
}

std::vector<Obstacle> TopologySelectorNode::buildPlanningObstacles() const
{
    std::vector<Obstacle> out;
    const int static_count = std::min({static_cast<int>(static_obstacles_x_.size()),
                                       static_cast<int>(static_obstacles_y_.size()),
                                       static_cast<int>(static_obstacles_r_.size())});
    for (int i = 0; i < static_count; ++i)
        out.push_back(Obstacle{static_obstacles_x_[i], static_obstacles_y_[i], static_obstacles_r_[i]});
    for (const auto &[name, odom] : dynamic_obstacles_)
        out.push_back(Obstacle{odom.pose.pose.position.x, odom.pose.pose.position.y, dynamic_obstacle_radii_.at(name)});
    return out;
}

double TopologySelectorNode::solveRepresentativeCost(LocalPlanner &planner, const nav_msgs::msg::Path &path, bool allow_warmstart) const
{
    if (!has_odom_ || path.poses.size() < 2)
        return std::numeric_limits<double>::max();

    USVState ego;
    ego.x = ego_odom_.pose.pose.position.x;
    ego.y = ego_odom_.pose.pose.position.y;
    const auto &q = ego_odom_.pose.pose.orientation;
    ego.psi = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    ego.v = std::hypot(ego_odom_.twist.twist.linear.x, ego_odom_.twist.twist.linear.y);

    const nav_msgs::msg::Path clean_path = sanitizePath(path, &ego);
    auto ref = pathMsgToReference(clean_path);
    planner.solver.setCurrentHeading(ego.psi);
    planner.solver.setReferencePath(ref);
    if (!allow_warmstart)
        planner.solver.clearWarmstart();
    const auto obstacles = buildPlanningObstacles();
    StageObstacleSet obstacles_by_stage(planner.solver.params().N, obstacles);
    Eigen::Vector2d local_goal = computeLocalGoal(ref, ego);
    planner.last_traj = planner.solver.solve(ego, obstacles_by_stage, local_goal.x(), local_goal.y());
    planner.last_cost = planner.solver.lastCost();
    return planner.last_cost;
}

std::vector<TopologySelectorNode::CandidateGroup> TopologySelectorNode::buildCandidateGroups() const
{
    std::map<SignatureKey, CandidateGroup> grouped;
    for (size_t i = 0; i < latest_candidates_.topological_paths.size(); ++i)
    {
        const auto &topo = latest_candidates_.topological_paths[i];
        const auto key = signatureKey(topo.h_signature);
        auto &group = grouped[key];
        group.key = key;
        group.raw_indices.push_back(i);
    }

    std::vector<CandidateGroup> groups;
    groups.reserve(grouped.size());
    for (const auto &[key, group] : grouped)
        groups.push_back(group);

    const_cast<TopologySelectorNode *>(this)->ensurePlannerPool(groups.size());

    USVState ego;
    ego.x = ego_odom_.pose.pose.position.x;
    ego.y = ego_odom_.pose.pose.position.y;
    const auto &q = ego_odom_.pose.pose.orientation;
    ego.psi = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    ego.v = std::hypot(ego_odom_.twist.twist.linear.x, ego_odom_.twist.twist.linear.y);

    for (auto &planner : planners_)
        planner.matched = false;

    for (auto &group : groups)
    {
        LocalPlanner *slot = nullptr;
        for (auto &planner : planners_)
        {
            if (planner.occupied && !planner.matched && planner.signature == group.key)
            {
                planner.matched = true;
                planner.reuse_warmstart = true;
                slot = &planner;
                break;
            }
        }
        if (!slot)
        {
            for (auto &planner : planners_)
            {
                if (!planner.occupied || !planner.matched)
                {
                    planner.occupied = true;
                    planner.matched = true;
                    planner.reuse_warmstart = false;
                    planner.signature = group.key;
                    slot = &planner;
                    break;
                }
            }
        }
        if (!slot)
            continue;

        std::vector<std::future<std::pair<size_t, double>>> futures;
        futures.reserve(group.raw_indices.size());
        for (size_t raw_idx : group.raw_indices)
        {
            futures.emplace_back(std::async(std::launch::async, [this, &group, slot, raw_idx, ego]() {
                const auto clean_path = sanitizePath(latest_candidates_.topological_paths[raw_idx].path, &ego);
                LocalPlanner local = *slot;
                local.signature = group.key;
                double solve_cost = solveRepresentativeCost(local, clean_path, slot->reuse_warmstart);
                double total_cost = solve_cost
                    + weight_topology_length_ * computeLengthCost(clean_path)
                    + computeGlobalConsistencyCost(clean_path, ego)
                    + computeInitialTurnCost(clean_path, ego);
                if (has_previous_selection_ && group.key != previous_signature_)
                    total_cost += topology_switch_penalty_;
                return std::make_pair(raw_idx, total_cost);
            }));
        }
        for (auto &f : futures)
        {
            auto [raw_idx, total_cost] = f.get();
            if (total_cost < group.representative_cost)
            {
                group.representative_cost = total_cost;
                group.representative_index = raw_idx;
            }
        }
        if (group.representative_index < latest_candidates_.topological_paths.size())
        {
            const auto best_path = sanitizePath(latest_candidates_.topological_paths[group.representative_index].path, &ego);
            solveRepresentativeCost(*slot, best_path, slot->reuse_warmstart);
        }
    }
    return groups;
}

double TopologySelectorNode::computeLengthCost(const nav_msgs::msg::Path &path) const
{
    if (path.poses.size() < 2) return 1e9;
    double total = 0.0;
    for (size_t i = 1; i < path.poses.size(); ++i)
    {
        const auto &a = path.poses[i - 1].pose.position;
        const auto &b = path.poses[i].pose.position;
        total += std::hypot(b.x - a.x, b.y - a.y);
    }
    return total;
}

double TopologySelectorNode::computeObstacleCost(const nav_msgs::msg::Path &path) const
{
    double cost = 0.0;
    for (const auto &pose : path.poses)
    {
        const auto &p = pose.pose.position;
        const int static_count = std::min({static_cast<int>(static_obstacles_x_.size()),
                                           static_cast<int>(static_obstacles_y_.size()),
                                           static_cast<int>(static_obstacles_r_.size())});
        for (int i = 0; i < static_count; ++i)
        {
            double dist = std::hypot(p.x - static_obstacles_x_[i], p.y - static_obstacles_y_[i]);
            double safe = static_obstacles_r_[i] + obstacle_safe_margin_;
            if (dist < safe) cost += obstacle_cost_gain_ * (safe - dist) * 1.5;
        }
        for (const auto &[name, odom] : dynamic_obstacles_)
        {
            const auto &o = odom.pose.pose.position;
            double dist = std::hypot(p.x - o.x, p.y - o.y);
            double safe = dynamic_obstacle_radii_.at(name) + obstacle_safe_margin_;
            if (dist < safe) cost += obstacle_cost_gain_ * (safe - dist);
        }
    }
    return cost;
}

double TopologySelectorNode::computePathCost(const nav_msgs::msg::Path &path) const
{
    return computeLengthCost(path) + computeObstacleCost(path);
}

void TopologySelectorNode::publishSelectedPath(const nav_msgs::msg::Path &path)
{
    USVState ego;
    if (has_odom_)
    {
        ego.x = ego_odom_.pose.pose.position.x;
        ego.y = ego_odom_.pose.pose.position.y;
        const auto &q = ego_odom_.pose.pose.orientation;
        ego.psi = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }
    previous_selected_path_ = sanitizePath(path, has_odom_ ? &ego : nullptr);
    pub_smooth_path_->publish(previous_selected_path_);
}

void TopologySelectorNode::publishDecision(size_t index, double base_cost, bool locked)
{
    topology_interfaces::msg::Decision msg;
    msg.header.stamp = this->now();
    msg.agent_id = agent_id_;
    msg.base_cost = base_cost;
    msg.is_locked = locked;
    if (index < latest_candidates_.topological_paths.size())
    {
        msg.h_signature = latest_candidates_.topological_paths[index].h_signature;
        USVState ego;
        const USVState *ego_ptr = nullptr;
        if (has_odom_)
        {
            ego.x = ego_odom_.pose.pose.position.x;
            ego.y = ego_odom_.pose.pose.position.y;
            const auto &q = ego_odom_.pose.pose.orientation;
            ego.psi = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            ego_ptr = &ego;
        }
        msg.planned_path = sanitizePath(latest_candidates_.topological_paths[index].path, ego_ptr);
    }
    pub_decision_->publish(msg);
}

void TopologySelectorNode::publishCandidateMarkers(ssize_t selected_index)
{
    visualization_msgs::msg::MarkerArray markers;
    for (size_t i = 0; i < latest_candidates_.topological_paths.size(); ++i)
    {
        const auto &topo = latest_candidates_.topological_paths[i];
        if (selected_index >= 0 && static_cast<ssize_t>(i) != selected_index)
            continue;

        visualization_msgs::msg::Marker line;
        line.header = latest_candidates_.header;
        line.ns = "topology_candidates";
        line.id = static_cast<int>(i);
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.pose.orientation.w = 1.0;
        line.scale.x = (static_cast<ssize_t>(i) == selected_index) ? 0.28 : 0.12;
        line.color.a = (static_cast<ssize_t>(i) == selected_index) ? 0.98 : 0.45;
        line.color.r = (static_cast<ssize_t>(i) == selected_index) ? 0.10 : 0.55;
        line.color.g = (static_cast<ssize_t>(i) == selected_index) ? 0.90 : 0.55;
        line.color.b = (static_cast<ssize_t>(i) == selected_index) ? 0.20 : 0.60;
        for (const auto &pose : topo.path.poses) line.points.push_back(pose.pose.position);
        markers.markers.push_back(line);
    }
    pub_candidate_markers_->publish(markers);
}

void TopologySelectorNode::evaluateAndPublish()
{
    if (!has_odom_ || !has_candidates_ || latest_candidates_.topological_paths.empty()) return;

    const auto groups = buildCandidateGroups();
    if (groups.empty()) return;

    const auto now = this->now();
    ssize_t selected_index = -1;
    double best_cost = std::numeric_limits<double>::max();

    if (has_previous_selection_ && now < hold_until_)
    {
        for (const auto &group : groups)
        {
            const size_t rep = group.representative_index;
            if (signatureKey(latest_candidates_.topological_paths[rep].h_signature) == previous_signature_)
            {
                selected_index = static_cast<ssize_t>(rep);
                best_cost = group.representative_cost;
                break;
            }
        }
    }

    if (selected_index < 0)
    {
        for (const auto &group : groups)
        {
            const size_t rep = group.representative_index;
            double cost = group.representative_cost;
            if (has_previous_selection_ && signatureKey(latest_candidates_.topological_paths[rep].h_signature) != previous_signature_)
                cost += consistency_penalty_;
            if (cost < best_cost)
            {
                best_cost = cost;
                selected_index = static_cast<ssize_t>(rep);
            }
        }
    }

    if (selected_index < 0) return;

    const auto &selected = latest_candidates_.topological_paths[static_cast<size_t>(selected_index)];
    const SignatureKey selected_signature = signatureKey(selected.h_signature);
    const bool keep_previous_path =
        has_previous_selection_ &&
        selected_signature == previous_signature_ &&
        !previous_selected_path_.poses.empty();

    previous_signature_ = selected_signature;
    has_previous_selection_ = true;
    hold_until_ = now + rclcpp::Duration::from_seconds(selection_hold_sec_);

    if (keep_previous_path)
        pub_smooth_path_->publish(previous_selected_path_);
    else
        publishSelectedPath(selected.path);
    publishDecision(static_cast<size_t>(selected_index), best_cost, true);
    publishCandidateMarkers(selected_index);

    std_msgs::msg::String mode;
    mode.data = "TOPOLOGY";
    pub_mode_->publish(mode);
}

} // namespace MultiUSV

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiUSV::TopologySelectorNode>());
    rclcpp::shutdown();
    return 0;
}
