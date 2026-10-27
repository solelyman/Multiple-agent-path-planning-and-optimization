#ifndef MULTI_USV_PLANNER_HERON_SINGLE_NODE_H
#define MULTI_USV_PLANNER_HERON_SINGLE_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <multi_usv_planner/types.h>
#include <multi_usv_planner/mpc_solver.h>
#include <multi_usv_planner/acados_solver.h>
#include <multi_usv_planner/controller.h>

namespace MultiUSV
{

class HeronSingleNode : public rclcpp::Node
{
public:
    HeronSingleNode();

private:
    void declareParams();
    void loadParams();
    void initCommunication();
    void controlLoop();

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::Point::SharedPtr msg);

    Trajectory runPlanner();
    void publishCommand(const Trajectory &traj);
    void publishTrajectoryPath(const Trajectory &traj);

    bool has_odom_ = false;
    bool has_goal_ = false;
    bool first_run_ = true;
    bool use_acados_ = false;

    double goal_x_ = 40.0;
    double goal_y_ = 0.0;
    double stop_radius_ = 2.0;

    USVState ego_;
    ReferencePath ref_path_;
    MPCSolver mpc_solver_;
    AcadosSolver acados_solver_;
    USVController controller_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_goal_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_traj_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_viz_;

    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace MultiUSV

#endif
