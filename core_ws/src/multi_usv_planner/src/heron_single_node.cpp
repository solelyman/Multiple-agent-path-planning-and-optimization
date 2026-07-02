#include <multi_usv_planner/heron_single_node.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;
using namespace MultiUSV;

HeronSingleNode::HeronSingleNode()
    : Node("heron_single_node")
{
    declareParams();
    loadParams();
    initCommunication();

    RCLCPP_INFO(this->get_logger(),
        "Heron 单艇节点启动 | acados=%s | status=%s | goal=(%.1f, %.1f)",
        use_acados_ ? "on" : "off",
        acados_solver_.statusMessage().c_str(),
        goal_x_, goal_y_);
}

void HeronSingleNode::declareParams()
{
    this->declare_parameter("goal.x", 40.0);
    this->declare_parameter("goal.y", 0.0);
    this->declare_parameter("use_acados", false);

    this->declare_parameter("acados.N", 20);
    this->declare_parameter("acados.dt", 0.2);
    this->declare_parameter("acados.m11", 28.0);
    this->declare_parameter("acados.Izz", 10.0);
    this->declare_parameter("acados.d_u", 16.45);
    this->declare_parameter("acados.d_r", 6.0);
    this->declare_parameter("acados.max_u", 2.0);
    this->declare_parameter("acados.max_r", 1.0);
    this->declare_parameter("acados.max_tau_u", 20.0);
    this->declare_parameter("acados.max_tau_r", 10.0);
    this->declare_parameter("acados.weight_pos", 10.0);
    this->declare_parameter("acados.weight_psi", 2.0);
    this->declare_parameter("acados.weight_u", 1.0);
    this->declare_parameter("acados.weight_r", 1.0);
    this->declare_parameter("acados.weight_tau_u", 0.1);
    this->declare_parameter("acados.weight_tau_r", 0.1);

    this->declare_parameter("mpc.N", 15);
    this->declare_parameter("mpc.dt", 0.4);
    this->declare_parameter("mpc.max_vel", 2.0);
    this->declare_parameter("mpc.max_omega", 1.57);
    this->declare_parameter("mpc.max_acc", 1.0);
    this->declare_parameter("mpc.desired_speed", 1.5);
    this->declare_parameter("mpc.max_iter", 0);
    this->declare_parameter("mpc.weight_goal", 1.0);
    this->declare_parameter("mpc.weight_path", 0.0);
    this->declare_parameter("mpc.weight_obstacle", 0.0);
    this->declare_parameter("mpc.weight_smooth", 1.0);
    this->declare_parameter("mpc.weight_speed", 8.0);
    this->declare_parameter("mpc.obstacle_clearance", 1.5);

    this->declare_parameter("controller.m11", 28.0);
    this->declare_parameter("controller.m22", 28.0);
    this->declare_parameter("controller.Izz", 10.0);
    this->declare_parameter("controller.d_u", -16.45);
    this->declare_parameter("controller.d_r", -6.0);
    this->declare_parameter("controller.lookahead_h", 8.0);
    this->declare_parameter("controller.k_x", 2.0);
    this->declare_parameter("controller.k_u", 5.0);
    this->declare_parameter("controller.k_psi", 1.0);
    this->declare_parameter("controller.k_y", 0.5);
    this->declare_parameter("controller.k_r", 5.0);
    this->declare_parameter("controller.max_u", 2.0);
    this->declare_parameter("controller.max_r", 1.0);
    this->declare_parameter("controller.filter_omega_n", 2.2);
    this->declare_parameter("controller.filter_zeta", 0.85);
    this->declare_parameter("controller.stop_radius", 2.0);
}

void HeronSingleNode::loadParams()
{
    goal_x_ = this->get_parameter("goal.x").as_double();
    goal_y_ = this->get_parameter("goal.y").as_double();
    use_acados_ = this->get_parameter("use_acados").as_bool();

    auto load_double = [&](const char *k) { return this->get_parameter(k).as_double(); };
    auto load_int = [&](const char *k) { return this->get_parameter(k).as_int(); };

    auto &ap = acados_solver_.params();
    ap.N = load_int("acados.N");
    ap.dt = load_double("acados.dt");
    ap.m11 = load_double("acados.m11");
    ap.Izz = load_double("acados.Izz");
    ap.d_u = load_double("acados.d_u");
    ap.d_r = load_double("acados.d_r");
    ap.max_u = load_double("acados.max_u");
    ap.max_r = load_double("acados.max_r");
    ap.max_tau_u = load_double("acados.max_tau_u");
    ap.max_tau_r = load_double("acados.max_tau_r");
    ap.weight_pos = load_double("acados.weight_pos");
    ap.weight_psi = load_double("acados.weight_psi");
    ap.weight_u = load_double("acados.weight_u");
    ap.weight_r = load_double("acados.weight_r");
    ap.weight_tau_u = load_double("acados.weight_tau_u");
    ap.weight_tau_r = load_double("acados.weight_tau_r");

    if (use_acados_)
    {
        acados_solver_.initialize();
    }

    auto &mp = mpc_solver_.params();
    mp.N = load_int("mpc.N");
    mp.dt = load_double("mpc.dt");
    mp.max_vel = load_double("mpc.max_vel");
    mp.max_omega = load_double("mpc.max_omega");
    mp.max_acc = load_double("mpc.max_acc");
    mp.desired_speed = load_double("mpc.desired_speed");
    mp.max_iter = load_int("mpc.max_iter");
    mp.weight_goal = load_double("mpc.weight_goal");
    mp.weight_path = load_double("mpc.weight_path");
    mp.weight_obstacle = load_double("mpc.weight_obstacle");
    mp.weight_smooth = load_double("mpc.weight_smooth");
    mp.weight_speed = load_double("mpc.weight_speed");
    mp.obstacle_clearance = load_double("mpc.obstacle_clearance");

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
    stop_radius_ = cp.stop_radius;
    controller_.setParams(cp);
}

void HeronSingleNode::initCommunication()
{
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr m) { odomCallback(m); });

    sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
        "reference_path", 10, [this](const nav_msgs::msg::Path::SharedPtr m) { pathCallback(m); });

    sub_goal_ = this->create_subscription<geometry_msgs::msg::Point>(
        "final_goal", 10, [this](const geometry_msgs::msg::Point::SharedPtr m) { goalCallback(m); });

    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    pub_traj_ = this->create_publisher<nav_msgs::msg::Path>("smooth_trajectory", 10);
    pub_viz_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("planner_visualization", 10);

    timer_ = this->create_wall_timer(100ms, std::bind(&HeronSingleNode::controlLoop, this));
}

void HeronSingleNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    ego_.x = msg->pose.pose.position.x;
    ego_.y = msg->pose.pose.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    ego_.psi = yaw;
    ego_.v = msg->twist.twist.linear.x;
    ego_.omega = msg->twist.twist.angular.z;

    has_odom_ = true;
    if (first_run_)
    {
        controller_.reset(ego_.omega);
        first_run_ = false;
    }
}

void HeronSingleNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    ref_path_.clear();
    for (const auto &pose : msg->poses)
    {
        ref_path_.points.push_back(Eigen::Vector2d(pose.pose.position.x, pose.pose.position.y));
    }
}

void HeronSingleNode::goalCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
    goal_x_ = msg->x;
    goal_y_ = msg->y;
    has_goal_ = true;
}

Trajectory HeronSingleNode::runPlanner()
{
    if (use_acados_)
    {
        auto traj = acados_solver_.solve(ego_, ref_path_, {}, goal_x_, goal_y_);
        if (traj.size() > 0)
        {
            return traj;
        }
    }

    if (!ref_path_.empty())
    {
        mpc_solver_.setReferencePath(ref_path_);
    }
    StageObstacleSet empty_obstacles_by_stage(mpc_solver_.params().N);
    return mpc_solver_.solve(ego_, empty_obstacles_by_stage, goal_x_, goal_y_);
}

void HeronSingleNode::publishCommand(const Trajectory &traj)
{
    if (traj.size() == 0) return;

    auto [u_cmd, r_cmd] = controller_.compute(traj, ego_, 0.1);
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = u_cmd;
    cmd.angular.z = r_cmd;
    pub_cmd_vel_->publish(cmd);
}

void HeronSingleNode::publishTrajectoryPath(const Trajectory &traj)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = "odom";
    path.header.stamp = this->now();
    for (const auto &p : traj.points)
    {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = path.header;
        ps.pose.position.x = p.x;
        ps.pose.position.y = p.y;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, p.psi);
        ps.pose.orientation = tf2::toMsg(q);
        path.poses.push_back(ps);
    }
    pub_traj_->publish(path);
}

void HeronSingleNode::controlLoop()
{
    if (!has_odom_ || !has_goal_) return;

    double dx = ego_.x - goal_x_;
    double dy = ego_.y - goal_y_;
    if (std::sqrt(dx * dx + dy * dy) < stop_radius_)
    {
        geometry_msgs::msg::Twist cmd;
        pub_cmd_vel_->publish(cmd);
        return;
    }

    auto traj = runPlanner();
    if (traj.size() > 1)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Heron acados=%s | p0(%.2f,%.2f,%.2f,%.2f) p1(%.2f,%.2f,%.2f,%.2f) goal=(%.1f,%.1f)",
            use_acados_ ? "on" : "off",
            traj.points[0].x, traj.points[0].y, traj.points[0].psi, traj.points[0].v,
            traj.points[1].x, traj.points[1].y, traj.points[1].psi, traj.points[1].v,
            goal_x_, goal_y_);
    }
    publishCommand(traj);
    publishTrajectoryPath(traj);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HeronSingleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
