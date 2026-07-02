#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <topology_interfaces/msg/topological_path_array.hpp>
#include <topology_interfaces/msg/topological_path.hpp>

#include <guidance_planner/global_guidance.h>
#include <ros_tools/ros2_wrappers.h>
#include <ros_tools/visuals.h>
#include <ros_tools/profiling.h>
#include <ros_tools/spline.h>

#include <memory>
#include <algorithm>
#include <cmath>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <chrono>

using namespace GuidancePlanner;

class PRMNode : public rclcpp::Node
{
public:
    PRMNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(false))
        : Node("prm_node", options)
    {
        // Initialize static pointers for ROS tools
        STATIC_NODE_POINTER.init(this);
        VISUALS.init(this);
        guidance_ = std::make_unique<GlobalGuidance>();
        if (!this->has_parameter("offset"))
        {
            this->declare_parameter<std::vector<double>>("offset", std::vector<double>{0.0, 0.0, 0.0});
        }
        if (!this->has_parameter("reference_path_width"))
        {
            this->declare_parameter<double>("reference_path_width", 30.0);
        }
        if (!this->has_parameter("local_goal_distance_scale"))
        {
            this->declare_parameter<double>("local_goal_distance_scale", 1.0);
        }
        if (!this->has_parameter("min_local_goal_distance"))
        {
            this->declare_parameter<double>("min_local_goal_distance", 10.0);
        }
        if (!this->has_parameter("target"))
        {
            this->declare_parameter<std::vector<double>>("target", std::vector<double>{120.0, 60.0, 0.0});
        }
        if (!this->has_parameter("enable_visualization"))
        {
            this->declare_parameter<bool>("enable_visualization", false);
        }
        if (!this->has_parameter("enable_info_log"))
        {
            this->declare_parameter<bool>("enable_info_log", false);
        }
        if (!this->has_parameter("enable_debug_report"))
        {
            this->declare_parameter<bool>("enable_debug_report", false);
        }
        if (!this->has_parameter("static_obstacles_x"))
        {
            this->declare_parameter<std::vector<double>>("static_obstacles_x", std::vector<double>{});
            this->declare_parameter<std::vector<double>>("static_obstacles_y", std::vector<double>{});
            this->declare_parameter<std::vector<double>>("static_obstacles_r", std::vector<double>{});
        }
        const auto offset = this->get_parameter("offset").as_double_array();
        const auto target = this->get_parameter("target").as_double_array();
        reference_path_width_ = this->get_parameter("reference_path_width").as_double();
        local_goal_distance_scale_ = this->get_parameter("local_goal_distance_scale").as_double();
        min_local_goal_distance_ = this->get_parameter("min_local_goal_distance").as_double();
        enable_visualization_ = this->get_parameter("enable_visualization").as_bool();
        enable_info_log_ = this->get_parameter("enable_info_log").as_bool();
        enable_debug_report_ = this->get_parameter("enable_debug_report").as_bool();
        static_obstacles_x_ = this->get_parameter("static_obstacles_x").as_double_array();
        static_obstacles_y_ = this->get_parameter("static_obstacles_y").as_double_array();
        static_obstacles_r_ = this->get_parameter("static_obstacles_r").as_double_array();
        if (offset.size() >= 2)
        {
            current_pose_.position.x = offset[0];
            current_pose_.position.y = offset[1];
            current_pose_.position.z = offset.size() >= 3 ? offset[2] : 0.0;
            current_pose_.orientation.w = 1.0;
            has_odom_ = true;
        }
        if (target.size() >= 2)
        {
            current_goal_.position.x = target[0];
            current_goal_.position.y = target[1];
            current_goal_.position.z = target.size() >= 3 ? target[2] : 0.0;
            current_goal_.orientation.w = 1.0;
            has_goal_ = true;
        }

        // Publishers
        path_pub_ = this->create_publisher<topology_interfaces::msg::TopologicalPathArray>("topological_path", 10);
        reference_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "reference_path", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&PRMNode::OdomCallback, this, std::placeholders::_1));
        
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&PRMNode::GoalCallback, this, std::placeholders::_1));
        obstacle_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "detected_obstacle", 10, std::bind(&PRMNode::ObstacleCallback, this, std::placeholders::_1));

        const std::vector<std::pair<std::string, double>> dynamic_obstacle_specs = {
            {"vessel_e1", 2.8}, {"vessel_e2", 2.8}, {"vessel_f1", 3.0}, {"vessel_f2", 3.0},
        };
        for (const auto &[name, radius] : dynamic_obstacle_specs)
        {
            auto cb = [this, name, radius](const nav_msgs::msg::Odometry::SharedPtr msg)
            { DynamicObstacleCallback(msg, name, radius); };
            dynamic_obstacle_subs_.push_back(this->create_subscription<nav_msgs::msg::Odometry>(
                "/" + name + "/odom", 10, cb));
        }

        // Timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&PRMNode::TimerCallback, this));

        RCLCPP_INFO(this->get_logger(), "PRM Node initialized. visuals=%s info_log=%s", enable_visualization_ ? "on" : "off", enable_info_log_ ? "on" : "off");
        if (enable_debug_report_)
        {
            LoadDebugEnv();
        }
    }

private:
    void LoadDebugEnv()
    {
        std::ifstream env_file("/home/lu/paper2/.dbg/ros-prm-visualization.env");
        if (!env_file.is_open())
        {
            return;
        }
        std::string line;
        while (std::getline(env_file, line))
        {
            if (line.rfind("DEBUG_SERVER_URL=", 0) == 0)
            {
                debug_url_ = line.substr(std::string("DEBUG_SERVER_URL=").size());
            }
            else if (line.rfind("DEBUG_SESSION_ID=", 0) == 0)
            {
                debug_session_id_ = line.substr(std::string("DEBUG_SESSION_ID=").size());
            }
        }
    }

    void ReportDebug(const std::string &hypothesis_id, const std::string &location, const std::string &msg, const std::string &data_json)
    {
        if (debug_url_.empty() || !enable_debug_report_)
        {
            return;
        }
        const auto escaped_msg = EscapeJson(msg);
        const auto escaped_location = EscapeJson(location);
        const auto escaped_session = EscapeJson(debug_session_id_);
        std::ostringstream payload;
        payload << "{\"sessionId\":\"" << escaped_session << "\","
                << "\"runId\":\"pre-fix\","
                << "\"hypothesisId\":\"" << hypothesis_id << "\","
                << "\"location\":\"" << escaped_location << "\","
                << "\"msg\":\"" << escaped_msg << "\","
                << "\"data\":" << data_json << "}";
        const std::string command = "python3 - <<'PY'\n"
            "import json, urllib.request\n"
            "url = " + PythonQuote(debug_url_) + "\n"
            "payload = " + PythonQuote(payload.str()) + "\n"
            "try:\n"
            "    req = urllib.request.Request(url, data=payload.encode(), headers={'Content-Type': 'application/json'})\n"
            "    urllib.request.urlopen(req, timeout=0.2).read()\n"
            "except Exception:\n"
            "    pass\n"
            "PY";
        std::system(command.c_str());
    }

    static std::string EscapeJson(const std::string &input)
    {
        std::string out;
        out.reserve(input.size() + 8);
        for (const char ch : input)
        {
            switch (ch)
            {
            case '\\': out += "\\\\"; break;
            case '"': out += "\\\""; break;
            case '\n': out += "\\n"; break;
            default: out += ch; break;
            }
        }
        return out;
    }

    static std::string PythonQuote(const std::string &input)
    {
        return "r'''" + input + "'''";
    }

    struct ObstacleTrack
    {
        Eigen::Vector2d current_position = Eigen::Vector2d::Zero();
        Eigen::Vector2d previous_position = Eigen::Vector2d::Zero();
        double current_time = 0.0;
        double previous_time = 0.0;
        double last_seen_time = 0.0;
        double radius = 12.0;
        bool has_history = false;
    };

    void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = msg->pose.pose;
        current_twist_ = msg->twist.twist;
        has_odom_ = true;
        // #region debug-point B:prm-odom
        ReportDebug("B", "prm_node.cpp:OdomCallback", "[DEBUG] PRM received odom", "{\"x\":" + std::to_string(current_pose_.position.x) + ",\"y\":" + std::to_string(current_pose_.position.y) + "}");
        // #endregion
    }

    void GoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_goal_ = msg->pose;
        has_goal_ = true;
        RCLCPP_INFO(this->get_logger(), "New goal received: (%.2f, %.2f)", current_goal_.position.x, current_goal_.position.y);
        // #region debug-point B:prm-goal
        ReportDebug("B", "prm_node.cpp:GoalCallback", "[DEBUG] PRM received goal", "{\"x\":" + std::to_string(current_goal_.position.x) + ",\"y\":" + std::to_string(current_goal_.position.y) + "}");
        // #endregion
    }

    void ObstacleCallback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        UpsertObstacleTrack(Eigen::Vector2d(msg->x, msg->y), std::max(1.0, static_cast<double>(msg->z)));
    }

    void DynamicObstacleCallback(const nav_msgs::msg::Odometry::SharedPtr msg, const std::string &, double radius)
    {
        const auto &p = msg->pose.pose.position;
        UpsertObstacleTrack(Eigen::Vector2d(p.x, p.y), radius);
    }

    void UpsertObstacleTrack(const Eigen::Vector2d &observation, double radius)
    {
        const double now = this->now().seconds();

        auto same_track = [&](const ObstacleTrack &track) {
            return (track.current_position - observation).norm() < obstacle_track_merge_distance_;
        };

        auto it = std::find_if(obstacle_tracks_.begin(), obstacle_tracks_.end(), same_track);
        if (it == obstacle_tracks_.end())
        {
            ObstacleTrack track;
            track.current_position = observation;
            track.previous_position = observation;
            track.current_time = now;
            track.previous_time = now;
            track.last_seen_time = now;
            track.radius = radius;
            obstacle_tracks_.push_back(track);
            return;
        }

        it->previous_position = it->current_position;
        it->previous_time = it->current_time;
        it->current_position = observation;
        it->current_time = now;
        it->last_seen_time = now;
        it->radius = radius;
        it->has_history = true;
        // #region debug-point B:prm-obstacle
        ReportDebug("B", "prm_node.cpp:ObstacleCallback", "[DEBUG] PRM received obstacle observation", "{\"x\":" + std::to_string(observation.x()) + ",\"y\":" + std::to_string(observation.y()) + ",\"r\":" + std::to_string(radius) + "}");
        // #endregion
    }

    void PruneObstacleTracks()
    {
        const double now = this->now().seconds();
        obstacle_tracks_.erase(
            std::remove_if(
                obstacle_tracks_.begin(),
                obstacle_tracks_.end(),
                [&](const ObstacleTrack &track) {
                    return (now - track.last_seen_time) > obstacle_track_timeout_;
                }),
            obstacle_tracks_.end());
    }

    std::vector<Obstacle> BuildDynamicObstacles() const
    {
        std::vector<Obstacle> obstacles;
        obstacles.reserve(obstacle_tracks_.size());

        for (const auto &track : obstacle_tracks_)
        {
            Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
            if (track.has_history)
            {
                const double dt = std::max(1e-3, track.current_time - track.previous_time);
                velocity = (track.current_position - track.previous_position) / dt;
            }

            std::vector<Eigen::Vector2d> predicted_positions;
            predicted_positions.reserve(Config::N + 1);
            for (int k = 0; k <= Config::N; ++k)
            {
                predicted_positions.push_back(track.current_position + velocity * (k * Config::DT));
            }

            obstacles.emplace_back(-1, predicted_positions, track.radius);
        }

        return obstacles;
    }

    void AppendStaticObstacles(std::vector<Obstacle> &obstacles) const
    {
        const int n = std::min({static_cast<int>(static_obstacles_x_.size()),
                                static_cast<int>(static_obstacles_y_.size()),
                                static_cast<int>(static_obstacles_r_.size())});
        for (int i = 0; i < n; ++i)
        {
            std::vector<Eigen::Vector2d> positions;
            positions.reserve(Config::N + 1);
            const Eigen::Vector2d center(static_obstacles_x_[i], static_obstacles_y_[i]);
            for (int k = 0; k <= Config::N; ++k)
                positions.push_back(center);
            obstacles.emplace_back(-1000 - i, positions, static_obstacles_r_[i] + 0.8);
        }
    }

    double ComputeLocalPlanningDistance(double speed) const
    {
        const double horizon_distance = std::max(8.0, speed * Config::DT * static_cast<double>(Config::N));
        const double scaled_distance = std::max(min_local_goal_distance_, horizon_distance * local_goal_distance_scale_);
        return scaled_distance;
    }

    Eigen::Vector2d ComputeLocalGoal(const Eigen::Vector2d &start, const Eigen::Vector2d &goal, double local_distance) const
    {
        const Eigen::Vector2d delta = goal - start;
        const double distance_to_goal = delta.norm();
        if (distance_to_goal <= std::max(1e-3, local_distance))
        {
            return goal;
        }

        return start + delta.normalized() * local_distance;
    }

    void TimerCallback()
    {
        // #region debug-point B:prm-tick
        ReportDebug("B", "prm_node.cpp:TimerCallback", "[DEBUG] PRM timer tick", "{\"has_odom\":" + std::string(has_odom_ ? "true" : "false") + ",\"has_goal\":" + std::string(has_goal_ ? "true" : "false") + "}");
        // #endregion
        if (!has_odom_ || !has_goal_) return;

        PruneObstacleTracks();

        // Set start state
        double yaw = 2.0 * atan2(current_pose_.orientation.z, current_pose_.orientation.w);
        double measured_vel = std::hypot(current_twist_.linear.x, current_twist_.linear.y);
        double planning_vel = 1.9;
        const Eigen::Vector2d start(current_pose_.position.x, current_pose_.position.y);
        const Eigen::Vector2d global_goal(current_goal_.position.x, current_goal_.position.y);
        const double distance_to_goal = (global_goal - start).norm();
        if (distance_to_goal < 1e-3)
        {
            return;
        }

        const double local_distance = ComputeLocalPlanningDistance(planning_vel);
        const Eigen::Vector2d local_goal = ComputeLocalGoal(start, global_goal, local_distance);
        const Eigen::Vector2d direction = (local_goal - start).norm() > 1e-6 ? (local_goal - start).normalized() : Eigen::Vector2d(1.0, 0.0);
        std::vector<double> ref_x;
        std::vector<double> ref_y;
        const int ref_points = 6;
        ref_x.reserve(ref_points);
        ref_y.reserve(ref_points);
        for (int i = 0; i < ref_points; ++i)
        {
            const double alpha = static_cast<double>(i) / static_cast<double>(ref_points - 1);
            const Eigen::Vector2d point = start + alpha * local_distance * direction;
            ref_x.push_back(point.x());
            ref_y.push_back(point.y());
        }
        auto reference_path = std::make_shared<RosTools::Spline2D>(ref_x, ref_y);

        guidance_->SetStart(start, yaw, planning_vel);
        guidance_->LoadReferencePath(0.0, reference_path, reference_path_width_);

        std::vector<Obstacle> obstacles = BuildDynamicObstacles();
        AppendStaticObstacles(obstacles);
        std::vector<Halfspace> static_obstacles;
        guidance_->LoadObstacles(obstacles, static_obstacles);

        // Update guidance planner
        const auto update_begin = std::chrono::steady_clock::now();
        bool success = guidance_->Update();
        const double update_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - update_begin).count();

        if (success)
        {
            topology_interfaces::msg::TopologicalPathArray path_array_msg;
            path_array_msg.header.stamp = this->now();
            path_array_msg.header.frame_id = "odom";

            int num_trajs = guidance_->NumberOfGuidanceTrajectories();
            if (num_trajs <= 0)
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "Guidance update succeeded but produced no trajectories. start=(%.2f, %.2f) local_goal=(%.2f, %.2f) global_goal=(%.2f, %.2f) dynamic_obs=%zu",
                    current_pose_.position.x, current_pose_.position.y,
                    local_goal.x(), local_goal.y(),
                    current_goal_.position.x, current_goal_.position.y,
                    obstacles.size());
            }
            for (int i = 0; i < num_trajs; i++)
            {
                auto& traj = guidance_->GetGuidanceTrajectory(i);
                
                topology_interfaces::msg::TopologicalPath topo_path;
                // 只在需要调试 PRM 内部结构时显示 spline，默认关闭，避免 RViz/内存压力
                if (enable_visualization_)
                {
                    traj.spline.Visualize();
                }
                
                // Get the trajectory points
                RosTools::Spline2D spline_traj = traj.spline.GetTrajectory();
                
                // We use topology_class as the h_signature for downstream identification
                topo_path.h_signature.push_back(static_cast<double>(traj.topology_class));

                topo_path.path.header = path_array_msg.header;
                
                // Sample points from the continuous spline
                const double horizon = std::max(20.0, Config::N * Config::DT * 1.8);
                for (double t = 0.0; t <= horizon; t += Config::DT)
                {
                    Eigen::Vector2d pos = spline_traj.getPoint(t);
                    geometry_msgs::msg::PoseStamped pose_stamped;
                    pose_stamped.header = path_array_msg.header;
                    pose_stamped.pose.position.x = pos.x();
                    pose_stamped.pose.position.y = pos.y();
                    topo_path.path.poses.push_back(pose_stamped);
                }
                
                path_array_msg.topological_paths.push_back(topo_path);
            }

            path_pub_->publish(path_array_msg);
            if (!path_array_msg.topological_paths.empty())
            {
                reference_path_pub_->publish(path_array_msg.topological_paths.front().path);
            }
            // #region debug-point B:prm-publish
            ReportDebug("B", "prm_node.cpp:TimerCallback", "[DEBUG] PRM published topological paths", "{\"count\":" + std::to_string(num_trajs) + ",\"dynamic_obs\":" + std::to_string(obstacles.size()) + "}");
            // #endregion
            if (enable_info_log_)
            {
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 10000,
                    "Published %d topological trajectories in %.1f ms. start=(%.2f, %.2f) local_goal=(%.2f, %.2f) global_goal=(%.2f, %.2f) dynamic_obs=%zu",
                    num_trajs,
                    update_ms,
                    current_pose_.position.x, current_pose_.position.y,
                    local_goal.x(), local_goal.y(),
                    current_goal_.position.x, current_goal_.position.y,
                    obstacles.size());
            }
            if (enable_visualization_)
            {
                guidance_->Visualize();
            }
        }
        else
        {
            // #region debug-point B:prm-fail
            ReportDebug("B", "prm_node.cpp:TimerCallback", "[DEBUG] PRM update failed", "{\"dynamic_obs\":" + std::to_string(obstacles.size()) + ",\"measured_speed\":" + std::to_string(measured_vel) + ",\"planning_speed\":" + std::to_string(planning_vel) + "}");
            // #endregion
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Guidance update failed after %.1f ms. start=(%.2f, %.2f) local_goal=(%.2f, %.2f) global_goal=(%.2f, %.2f) dynamic_obs=%zu",
                update_ms,
                current_pose_.position.x, current_pose_.position.y,
                local_goal.x(), local_goal.y(),
                current_goal_.position.x, current_goal_.position.y,
                obstacles.size());
        }
    }

    rclcpp::Publisher<topology_interfaces::msg::TopologicalPathArray>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_path_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr obstacle_sub_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> dynamic_obstacle_subs_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<GlobalGuidance> guidance_;
    std::vector<ObstacleTrack> obstacle_tracks_;

    geometry_msgs::msg::Pose current_pose_;
    geometry_msgs::msg::Twist current_twist_;
    geometry_msgs::msg::Pose current_goal_;

    bool has_odom_ = false;
    bool has_goal_ = false;
    bool enable_visualization_ = false;
    bool enable_info_log_ = false;
    bool enable_debug_report_ = false;
    double obstacle_track_merge_distance_ = 4.0;
    double obstacle_track_timeout_ = 1.5;
    double reference_path_width_ = 30.0;
    double local_goal_distance_scale_ = 0.9;
    double min_local_goal_distance_ = 6.0;
    std::vector<double> static_obstacles_x_;
    std::vector<double> static_obstacles_y_;
    std::vector<double> static_obstacles_r_;
    std::string debug_url_ = "http://127.0.0.1:7777/event";
    std::string debug_session_id_ = "ros-prm-visualization";
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PRMNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
