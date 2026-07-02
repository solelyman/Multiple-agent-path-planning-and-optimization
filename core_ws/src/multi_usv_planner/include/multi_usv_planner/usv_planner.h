#ifndef MULTI_USV_PLANNER_H
#define MULTI_USV_PLANNER_H

#include <multi_usv_planner/types.h>
#include <multi_usv_planner/mpc_solver.h>
#include <multi_usv_planner/acados_contouring_solver.h>
#include <multi_usv_planner/controller.h>
#include <multi_usv_planner/constraint_builder.h>
#include <multi_usv_planner/colregs_engine.h>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <topology_interfaces/msg/topological_path_array.hpp>

#include <vector>
#include <string>
#include <map>
#include <atomic>
#include <fstream>
#include <limits>

namespace MultiUSV
{
class USVPlanner : public rclcpp::Node
{
public:
    USVPlanner();

private:
    struct TopologyCandidate
    {
        ReferencePath path;
        std::vector<double> signature;
    };

    void declareParams();
    void loadParams();
    void initCommunication();
    void controlLoop();

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void topoPathCallback(const topology_interfaces::msg::TopologicalPathArray::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::Point::SharedPtr msg);
    void neighborCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg,
                          int neighbor_id);
    void dynamicObstacleCallback(const nav_msgs::msg::Odometry::SharedPtr msg,
                                 const std::string &name, double radius);

    void publishCommand(const Trajectory &traj);
    void publishVisualization(const Trajectory &traj);
    void publishTrajectoryPath(const Trajectory &traj);
    void updateEpisodeMetrics();
    void writeEpisodeMetricsIfNeeded(bool timeout = false);
    void rebuildActiveReference();
    ReferencePath buildLocalReferenceWindow(const ReferencePath &path) const;
    ReferencePath buildContouringReferenceWindow(const ReferencePath &path) const;
    bool referenceHandoffReady(const ReferencePath &candidate,
                               const ReferencePath &current) const;
    double currentNearestDynamicDistance() const;
    bool freezeReferenceSwitching() const;
    bool shouldShowLocalCandidates() const;
    void clearTopologyCandidateMarkers();
    Trajectory runMPC();

    //State
    int agent_id_ = 0;
    USVState ego_;
    std::vector<int> neighbor_ids_;
    double goal_x_ = 100.0, goal_y_ = 0.0;
    bool has_goal_ = false;
    bool has_odom_ = false;
    bool first_run_ = true;
    rclcpp::Time reset_guard_until_{0, 0, RCL_ROS_TIME};
    int near_goal_cooldown_ = 0;
    bool use_direct_cmd_ = false;
    bool use_selected_reference_ = true;
    bool use_original_like_reference_mode_ = false;
    std::string control_mode_ = "los";
    rclcpp::Time last_time_;

    //Obstacle list
    std::vector<Obstacle> static_obstacles_;
    std::map<std::string, DynamicObstacle> dynamic_obstacles_;

    //Neighbor list
    std::map<int, NeighborTrajectory> neighbors_;
    NeighborSelector neighbor_selector_;

    // Solver & Controller
    MPCSolver mpc_solver_;
    AcadosContouringSolver acados_solver_;
    ReferencePath ref_path_;
    ReferencePath topo_ref_path_;
    ReferencePath smooth_ref_path_;
    std::vector<TopologyCandidate> topo_candidates_;
    bool has_reference_path_ = false;
    bool selected_reference_active_ = false;
    bool topo_reference_locked_ = false;
    double reference_switch_freeze_distance_ = 8.0;
    bool enable_topology_rerank_ = true;
    int topology_rerank_candidates_ = 2;
    double topology_rerank_switch_margin_ = 0.45;
    int selected_topology_candidate_idx_ = -1;
    rclcpp::Time topology_hold_until_{0, 0, RCL_ROS_TIME};
    double last_progress_dist_to_goal_ = std::numeric_limits<double>::infinity();
    rclcpp::Time last_progress_time_{0, 0, RCL_ROS_TIME};
    bool stuck_recovery_active_ = false;
    rclcpp::Time stuck_recovery_until_{0, 0, RCL_ROS_TIME};
    USVController controller_;

    ConstraintBuilder constraint_builder_;
    COLREGsEngine colregs_engine_;
    COLREGsEngine::Rule last_colregs_rule_ = COLREGsEngine::NONE;

    std::atomic<bool> optimizing_{false};
    std::chrono::steady_clock::time_point solver_start_time_;
    bool acados_solver_was_reset_ = false;

private:
    Trajectory last_output_;
    Trajectory last_predicted_traj_;  // full MPC prediction for DR anchor projection
    rclcpp::Time episode_start_time_;
    bool metrics_initialized_ = false;
    bool metrics_written_ = false;
    bool done_ = false;  // true after passing obstacle zone → x>pass_x_threshold
    double pass_x_threshold_ = 55.0;
    bool collision_dynamic_ = false;
    bool collision_static_ = false;
    bool collision_neighbor_ = false;
    double min_dynamic_clearance_ = std::numeric_limits<double>::max();
    double min_static_clearance_ = std::numeric_limits<double>::max();
    double min_neighbor_clearance_ = std::numeric_limits<double>::max();
    double episode_timeout_s_ = 600.0;  // generous — DYN_SPEED may crawl near obstacles

    //Publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_ref_path_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_broadcast_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_viz_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_topo_viz_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_alias_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_planned_traj_alias_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_contouring_alias_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_contouring_current_alias_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_contouring_path_alias_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_contouring_points_alias_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_goal_module_alias_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_dynamic_obs_alias_;
    int prev_topo_marker_count_ = 0;
    bool topology_candidates_visible_ = false;
    double topology_candidate_show_distance_ = 10.0;
    bool colregs_enabled_{true};

    //Subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
    rclcpp::Subscription<topology_interfaces::msg::TopologicalPathArray>::SharedPtr sub_topo_path_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_goal_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr> sub_neighbors_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> sub_dynamic_obstacles_;

    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace MultiUSV

#endif
