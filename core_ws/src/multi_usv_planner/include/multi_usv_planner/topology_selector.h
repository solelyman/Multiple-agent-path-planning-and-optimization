#ifndef MULTI_USV_TOPOLOGY_SELECTOR_H
#define MULTI_USV_TOPOLOGY_SELECTOR_H

#include <multi_usv_planner/mpc_solver.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <topology_interfaces/msg/topological_path_array.hpp>
#include <topology_interfaces/msg/decision.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <vector>
#include <map>
#include <tuple>
#include <limits>

namespace MultiUSV
{

class TopologySelectorNode : public rclcpp::Node
{
public:
    TopologySelectorNode();

private:
    using SignatureKey = std::tuple<double>;

    struct CandidateGroup
    {
        SignatureKey key;
        std::vector<size_t> raw_indices;
        size_t representative_index = 0;
        double representative_cost = std::numeric_limits<double>::max();
    };

    struct LocalPlanner
    {
        SignatureKey signature{0.0};
        bool occupied = false;
        bool matched = false;
        bool reuse_warmstart = false;
        double last_cost = std::numeric_limits<double>::max();
        Trajectory last_traj;
        MPCSolver solver;
    };

    void declareParams();
    void initCommunication();

    void topoCallback(const topology_interfaces::msg::TopologicalPathArray::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void dynamicObstacleCallback(const nav_msgs::msg::Odometry::SharedPtr msg, const std::string &name, double radius);

    void evaluateAndPublish();
    std::vector<CandidateGroup> buildCandidateGroups() const;
    void ensurePlannerPool(size_t count);
    ReferencePath pathMsgToReference(const nav_msgs::msg::Path &path) const;
    nav_msgs::msg::Path sanitizePath(const nav_msgs::msg::Path &path, const USVState *ego = nullptr) const;
    Eigen::Vector2d computeLocalGoal(const ReferencePath &ref, const USVState &ego) const;
    double computeGlobalConsistencyCost(const nav_msgs::msg::Path &path, const USVState &ego) const;
    double computeInitialTurnCost(const nav_msgs::msg::Path &path, const USVState &ego) const;
    std::vector<Obstacle> buildPlanningObstacles() const;
    double solveRepresentativeCost(LocalPlanner &planner, const nav_msgs::msg::Path &path, bool allow_warmstart) const;
    double computePathCost(const nav_msgs::msg::Path &path) const;
    double computeLengthCost(const nav_msgs::msg::Path &path) const;
    double computeObstacleCost(const nav_msgs::msg::Path &path) const;
    SignatureKey signatureKey(const std::vector<double> &sig) const;
    void publishDecision(size_t index, double base_cost, bool locked);
    void publishSelectedPath(const nav_msgs::msg::Path &path);
    void publishCandidateMarkers(ssize_t selected_index);

    int agent_id_ = 1;
    std::vector<int> neighbor_ids_;
    bool has_odom_ = false;
    bool has_candidates_ = false;

    double consistency_penalty_ = 10.0;
    double topology_switch_penalty_ = 8.0;
    double weight_topology_length_ = 0.2;
    double obstacle_cost_gain_ = 80.0;
    double obstacle_safe_margin_ = 4.0;
    double selection_hold_sec_ = 1.5;
    double local_goal_horizon_ = 10.0;
    double weight_global_consistency_ = 25.0;
    double weight_initial_turn_ = 20.0;
    double goal_x_ = 0.0;
    double goal_y_ = 0.0;

    nav_msgs::msg::Odometry ego_odom_;
    topology_interfaces::msg::TopologicalPathArray latest_candidates_;
    std::vector<double> static_obstacles_x_;
    std::vector<double> static_obstacles_y_;
    std::vector<double> static_obstacles_r_;
    std::map<std::string, nav_msgs::msg::Odometry> dynamic_obstacles_;
    std::map<std::string, double> dynamic_obstacle_radii_;
    mutable std::vector<LocalPlanner> planners_;

    bool has_previous_selection_ = false;
    SignatureKey previous_signature_{0.0};
    nav_msgs::msg::Path previous_selected_path_;
    rclcpp::Time hold_until_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<topology_interfaces::msg::TopologicalPathArray>::SharedPtr sub_topo_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> sub_dynamic_obstacles_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_smooth_path_;
    rclcpp::Publisher<topology_interfaces::msg::Decision>::SharedPtr pub_decision_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_candidate_markers_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_mode_;
};

} // namespace MultiUSV

#endif
