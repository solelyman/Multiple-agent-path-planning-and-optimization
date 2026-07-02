#include <cmath>
#include <algorithm>
#include <memory>
#include <limits>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class CommandFilter{
private:
    double omegan_, zetan_, limit_, x1_, x2_;
public:
    CommandFilter() : omegan_(1.0), zetan_(0.7), limit_(1.0), x1_(0.0), x2_(0.0) {}
    CommandFilter(double omegan, double zetan, double limit): 
        omegan_(omegan), zetan_(zetan), limit_(limit), x1_(0.0), x2_(0.0){}

    void reset(double val){
        x1_ = std::clamp(val, -limit_, limit_);
        x2_ = 0.0;
    }
    
    void update(double input, double dt){
        double input_bounded = std::clamp(input, -limit_, limit_);
        double error = x1_ - input_bounded;
        double x2_dot = -2.0 * zetan_ * omegan_ * x2_ - omegan_ * omegan_ * error;
        x2_ += x2_dot * dt;
        x1_ += x2_ * dt;
    }
    
    double getSpeed() const {return x1_;}
    double getRate() const {return x2_;}
};

class SFController : public rclcpp::Node
{
private:
    CommandFilter yaw_filter_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_final_goal_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
    
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_left_thrust_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_right_thrust_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    rclcpp::Time last_time_; 
    nav_msgs::msg::Path path_;
    bool has_path_ = false;
    
    double x_, y_, psi_;
    double u_, v_, r_;
    bool has_final_goal_ = false;
    geometry_msgs::msg::Point final_goal_;

    // Parameters
    double last_yaw_ref_ = 0.0;
    double omegan_, zetan_;
    double delta_h_, k_x_, k_u_, k_psi_, k_r_;
    double stop_radius_;
    
    double m11_, m22_, I_zz_;
    double d_u_, d_r_;
    double max_tau_u_, max_tau_r_;
    double max_u_, max_r_;
    std::string output_mode_;
    double cmd_vel_linear_scale_;
    double cmd_vel_angular_scale_;
    double heron_half_width_;
    double heron_max_forward_thrust_;
    double heron_max_reverse_thrust_;
    double thrust_forward_command_scale_;
    double thrust_reverse_command_scale_;
    
    bool has_odom_ = false;
    bool first_run_ = true;

    void declare_params(){
        this->declare_parameter("vehicle_model.mass_11", 28.0);
        this->declare_parameter("vehicle_model.mass_22", 28.0);
        this->declare_parameter("vehicle_model.inertia_zz", 10.0);
        this->declare_parameter("vehicle_model.damping_11", -16.45);
        this->declare_parameter("vehicle_model.damping_66", -6.0);
        
        this->declare_parameter("dynamics_limits.max_thrust", 300.0);
        this->declare_parameter("dynamics_limits.max_torque", 100.0);
        this->declare_parameter("dynamics_limits.max_surge_speed", 2.0);
        this->declare_parameter("dynamics_limits.max_yaw_rate", 1.0);
        
        this->declare_parameter("controller_gains.lookahead_h", 8.0);
        this->declare_parameter("controller_gains.k_x", 2.0);
        this->declare_parameter("controller_gains.k_u", 5.0);
        this->declare_parameter("controller_gains.k_psi", 1.0);
        this->declare_parameter("controller_gains.k_r", 5.0);
        this->declare_parameter("controller_gains.stop_radius", 2.0);
        
        this->declare_parameter("filter.omega_n", 2.2);
        this->declare_parameter("filter.zeta_n", 0.85);

        this->declare_parameter("output.mode", "thrust");
        this->declare_parameter("output.cmd_vel_linear_scale", 1.0);
        this->declare_parameter("output.cmd_vel_angular_scale", 1.0);
        this->declare_parameter("output.heron_half_width", 0.37795);
        this->declare_parameter("output.heron_max_forward_thrust", 45.0);
        this->declare_parameter("output.heron_max_reverse_thrust", 25.0);
        this->declare_parameter("output.thrust_forward_command_scale", 300.0);
        this->declare_parameter("output.thrust_reverse_command_scale", 300.0);
    }

    void load_params_and_init(){
        max_tau_u_ = this->get_parameter("dynamics_limits.max_thrust").as_double();
        max_tau_r_ = this->get_parameter("dynamics_limits.max_torque").as_double();
        max_u_ = this->get_parameter("dynamics_limits.max_surge_speed").as_double();
        max_r_ = this->get_parameter("dynamics_limits.max_yaw_rate").as_double();
        
        omegan_ = this->get_parameter("filter.omega_n").as_double();
        zetan_ = this->get_parameter("filter.zeta_n").as_double();
        yaw_filter_ = CommandFilter(omegan_, zetan_, max_r_);
        
        delta_h_ = this->get_parameter("controller_gains.lookahead_h").as_double();
        
        m11_ = this->get_parameter("vehicle_model.mass_11").as_double();
        m22_ = this->get_parameter("vehicle_model.mass_22").as_double();
        I_zz_ = this->get_parameter("vehicle_model.inertia_zz").as_double();
        
        d_u_ = this->get_parameter("vehicle_model.damping_11").as_double(); 
        d_r_ = this->get_parameter("vehicle_model.damping_66").as_double();
        
        k_x_ = this->get_parameter("controller_gains.k_x").as_double();
        k_u_ = this->get_parameter("controller_gains.k_u").as_double();
        k_psi_ = this->get_parameter("controller_gains.k_psi").as_double();
        k_r_ = this->get_parameter("controller_gains.k_r").as_double();
        stop_radius_ = this->get_parameter("controller_gains.stop_radius").as_double();

        output_mode_ = this->get_parameter("output.mode").as_string();
        cmd_vel_linear_scale_ = this->get_parameter("output.cmd_vel_linear_scale").as_double();
        cmd_vel_angular_scale_ = this->get_parameter("output.cmd_vel_angular_scale").as_double();
        heron_half_width_ = this->get_parameter("output.heron_half_width").as_double();
        heron_max_forward_thrust_ = this->get_parameter("output.heron_max_forward_thrust").as_double();
        heron_max_reverse_thrust_ = this->get_parameter("output.heron_max_reverse_thrust").as_double();
        thrust_forward_command_scale_ = this->get_parameter("output.thrust_forward_command_scale").as_double();
        thrust_reverse_command_scale_ = this->get_parameter("output.thrust_reverse_command_scale").as_double();
        if (output_mode_ != "thrust" && output_mode_ != "cmd_vel") {
            RCLCPP_WARN(this->get_logger(), "Unknown output.mode '%s', fallback to thrust.", output_mode_.c_str());
            output_mode_ = "thrust";
        }
        heron_half_width_ = std::max(heron_half_width_, 1e-3);
        heron_max_forward_thrust_ = std::max(heron_max_forward_thrust_, 1e-3);
        heron_max_reverse_thrust_ = std::max(heron_max_reverse_thrust_, 1e-3);
        thrust_forward_command_scale_ = std::max(thrust_forward_command_scale_, 0.0);
        thrust_reverse_command_scale_ = std::max(thrust_reverse_command_scale_, 0.0);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        
        tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                          msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        double roll, pitch;
        tf2::Matrix3x3(q).getRPY(roll, pitch, psi_);
        
        u_ = msg->twist.twist.linear.x;
        v_ = msg->twist.twist.linear.y;
        r_ = msg->twist.twist.angular.z;
        has_odom_ = true;
    }

    double normalize_angle(double angle){
        while(angle > M_PI) angle -= 2.0 * M_PI;
        while(angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void publish_cmd_vel(double surge_cmd, double yaw_rate_cmd){
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = std::clamp(surge_cmd * cmd_vel_linear_scale_, 0.0, max_u_);
        cmd.angular.z = std::clamp(yaw_rate_cmd * cmd_vel_angular_scale_, -max_r_, max_r_);
        pub_cmd_vel_->publish(cmd);
    }

    double saturate_thruster_force(double thrust_cmd) const {
        if (thrust_cmd >= 0.0) {
            return std::min(thrust_cmd, heron_max_forward_thrust_);
        }
        return std::max(thrust_cmd, -heron_max_reverse_thrust_);
    }

    double force_to_command(double thrust_cmd) const {
        if (thrust_cmd >= 0.0) {
            return heron_max_forward_thrust_ > 0.0
                ? (thrust_cmd / heron_max_forward_thrust_) * thrust_forward_command_scale_
                : 0.0;
        }
        return heron_max_reverse_thrust_ > 0.0
            ? (thrust_cmd / heron_max_reverse_thrust_) * thrust_reverse_command_scale_
            : 0.0;
    }

    void publish_thrust(double surge_force, double yaw_torque){
        // Allocate differential thrust in a Heron-like way:
        // keep the requested yaw authority first, then fit the remaining surge.
        double tau_z_max = 2.0 * heron_half_width_ * heron_max_reverse_thrust_;
        double tau_z = std::clamp(yaw_torque, -tau_z_max, tau_z_max);
        double base_left = -tau_z / (2.0 * heron_half_width_);
        double base_right = tau_z / (2.0 * heron_half_width_);
        double feasible_surge = surge_force;

        if (tau_z >= 0.0) {
            if (feasible_surge >= 0.0) {
                feasible_surge = std::min(feasible_surge, 2.0 * (heron_max_forward_thrust_ - base_right));
            } else {
                feasible_surge = std::max(feasible_surge, 2.0 * (-heron_max_reverse_thrust_ - base_left));
            }
        } else {
            if (feasible_surge >= 0.0) {
                feasible_surge = std::min(feasible_surge, 2.0 * (heron_max_forward_thrust_ - base_left));
            } else {
                feasible_surge = std::max(feasible_surge, 2.0 * (-heron_max_reverse_thrust_ - base_right));
            }
        }

        double left_force = saturate_thruster_force(base_left + feasible_surge / 2.0);
        double right_force = saturate_thruster_force(base_right + feasible_surge / 2.0);

        auto msg_left = std_msgs::msg::Float64();
        msg_left.data = force_to_command(left_force);
        pub_left_thrust_->publish(msg_left);

        auto msg_right = std_msgs::msg::Float64();
        msg_right.data = force_to_command(right_force);
        pub_right_thrust_->publish(msg_right);
    }

    void control_loop(){
        if(!has_odom_ || !has_path_) return;

        if (path_.poses.empty()) return;

        geometry_msgs::msg::Point local_goal_;
        double lookahead_dist_ = delta_h_; 
        int min_idx = 0;
        double min_dist = std::numeric_limits<double>::infinity();
        
        for (size_t i = 0; i < path_.poses.size(); i++){
            double dx_tmp = x_ - path_.poses[i].pose.position.x;
            double dy_tmp = y_ - path_.poses[i].pose.position.y;
            double dist = std::sqrt(dx_tmp*dx_tmp + dy_tmp*dy_tmp);
            
            if(dist < min_dist){
                min_dist = dist;
                min_idx = i;
            }
        }
        
        bool found_lookahead = false;
        for(size_t i = min_idx; i < path_.poses.size(); i++){
            double dx_tmp = x_ - path_.poses[i].pose.position.x;
            double dy_tmp = y_ - path_.poses[i].pose.position.y;
            double dist = std::sqrt(dx_tmp*dx_tmp + dy_tmp*dy_tmp);
            
            if(dist > lookahead_dist_){
                local_goal_ = path_.poses[i].pose.position;
                found_lookahead = true;
                break;
            }
        }
        if (!found_lookahead) {
            local_goal_ = path_.poses.back().pose.position;
        }

        rclcpp::Time current_time = this->get_clock()->now();
        if (first_run_) {
            last_time_ = current_time;
            first_run_ = false;
            yaw_filter_.reset(r_);
            return;
        }
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;
        dt = std::clamp(dt, 0.001, 0.1);

        double x_ref = local_goal_.x;
        double y_ref = local_goal_.y;
        double u_ref = 1.0; 

        double yaw_ref = std::atan2(y_ref - y_, x_ref - x_);

        double dx = x_ - x_ref;
        double dy = y_ - y_ref;

        double cy = cos(yaw_ref); double sy = sin(yaw_ref);
        
        // 2D LOS 误差计算
        double xe = cy * dx + sy * dy;
        double ye = -sy * dx + cy * dy;

        double psi_los = atan2(-ye, delta_h_); 
        double psi_des = yaw_ref + psi_los;

        double r_feedforward = normalize_angle(psi_des - last_yaw_ref_) / dt;
        last_yaw_ref_ = psi_des;

        double u_des = u_ref - k_x_ * xe;
        u_des = std::clamp(u_des, -max_u_, max_u_);
        if (u_ref >= 0.0 && u_des < 0.0) {
            u_des = 0.0;
        }
        
        bool in_deadzone = false;
        if (has_final_goal_) {
            double dxg = x_ - final_goal_.x;
            double dyg = y_ - final_goal_.y;
            double dist_to_goal = std::sqrt(dxg * dxg + dyg * dyg);
            if (dist_to_goal < stop_radius_) {
                in_deadzone = true;
                u_des = 0.0;
                r_feedforward = 0.0;
            }
        }
        
        double u_error = u_ - u_des;
        
        // 纯 2D 控制律
        double tau_u = - m22_ * v_ * r_ 
                       - d_u_ * u_des               
                       - k_u_ * u_error             
                       - k_x_*k_x_ * m11_ * xe;
        
        if (in_deadzone) {
            tau_u = 0.0;
        }

        
        tau_u = std::clamp(tau_u, 0.0, max_tau_u_);

        double psi_error = normalize_angle(psi_ - psi_des);
        double alpha_r = r_feedforward - k_psi_ * psi_error;
        if (in_deadzone) alpha_r = 0.0;
        
        yaw_filter_.update(alpha_r, dt);
        double r_des = yaw_filter_.getSpeed();
        double r_dot_des = yaw_filter_.getRate();
        double r_error = r_ - r_des;
        
        double tau_r = - d_r_ * r_des + I_zz_ * r_dot_des - k_r_ * r_error;
        tau_r = std::clamp(tau_r, -max_tau_r_, max_tau_r_);
        
        if (in_deadzone) {
            tau_r = 0.0;
        }

        if (output_mode_ == "cmd_vel") {
            publish_cmd_vel(in_deadzone ? 0.0 : u_des, in_deadzone ? 0.0 : r_des);
            return;
        }

        publish_thrust(tau_u, tau_r);
    }

public:
    SFController(): Node("sf_controller")
    {
        declare_params();
        load_params_and_init(); 
    
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::SensorDataQoS(), std::bind(&SFController::odom_callback, this, std::placeholders::_1)
        );

        sub_final_goal_ = this->create_subscription<geometry_msgs::msg::Point>(
            "final_goal", 10, [this](const geometry_msgs::msg::Point::SharedPtr msg){
                final_goal_ = *msg;
                has_final_goal_ = true;
            }
        );

        sub_path_ = this->create_subscription<nav_msgs::msg::Path>("smooth_trajectory",10,[this](const nav_msgs::msg::Path::SharedPtr msg){
            path_=*msg;
            has_path_=true;
        });

        pub_left_thrust_ = this->create_publisher<std_msgs::msg::Float64>("thruster_left/cmd_thrust", 10);
        pub_right_thrust_ = this->create_publisher<std_msgs::msg::Float64>("thruster_right/cmd_thrust", 10);
        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
        timer_ = this->create_wall_timer(20ms, std::bind(&SFController::control_loop, this));
    
        RCLCPP_INFO(
            this->get_logger(),
            "SF 2D Controller Ready. output_mode=%s, cmd_vel_scales=(%.2f, %.2f), heron_half_width=%.4f",
            output_mode_.c_str(), cmd_vel_linear_scale_, cmd_vel_angular_scale_, heron_half_width_);
    }
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SFController>());
    rclcpp::shutdown();
    return 0;
}
