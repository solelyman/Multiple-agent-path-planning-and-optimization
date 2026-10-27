#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cmath>
#include <string>
#include <vector>

class CorridorVisualizer : public rclcpp::Node
{
public:
    CorridorVisualizer() : Node("scene_visualizer")
    {
        pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("scene_markers", 1);
        wall_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("corridor_walls", 1);
        timer_ = this->create_wall_timer(std::chrono::seconds(2), [this]() { publishScene(); });
        publishScene(); // immediate first publish
    }

private:
    void publishScene()
    {
        visualization_msgs::msg::MarkerArray arr;
        visualization_msgs::msg::MarkerArray wall_arr;
        int id = 0;

        // ── Corridor boundary walls: publish on dedicated topic ──
        addSimpleWall(wall_arr, id++, "corridor_walls", 37.5, -14.0, 105.0, 0.8);
        addSimpleWall(wall_arr, id++, "corridor_walls", 37.5,  14.0, 105.0, 0.8);

        // ── Multi-USV start markers (green spheres) ──
        addSphere(arr, id++, "start", -10.0, -6.0, 0.2, 0.8, 0.2, 1.2);
        addSphere(arr, id++, "start", -10.0,  0.0, 0.2, 0.8, 0.2, 1.2);
        addSphere(arr, id++, "start", -10.0,  6.0, 0.2, 0.8, 0.2, 1.2);

        // ── Multi-USV goal markers (red rings) ──
        addGoalCylinder(arr, id++, "goal", 80.0, -6.0);
        addGoalCylinder(arr, id++, "goal", 80.0,  0.0);
        addGoalCylinder(arr, id++, "goal", 80.0,  6.0);

        // ── Obstacle zone markers (faint dashed lines at x=-5 and x=25) ──
        addZoneLine(arr, id++, "zone_entry", -5.0, 0.6, 0.4, 0.1);
        addZoneLine(arr, id++, "zone_exit", 25.0, 0.4, 0.6, 0.1);

        // ── Finish/Stop line at x=25 (pass_threshold) ──
        addFinishLine(arr, id++, "finish_line", 25.0);

        wall_pub_->publish(wall_arr);
        pub_->publish(arr);
    }

    void addSimpleWall(visualization_msgs::msg::MarkerArray &arr, int id,
                       const std::string &ns, double cx, double cy, double len_x, double width_y)
    {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "odom";
        m.header.stamp = now();
        m.ns = ns;
        m.id = id;
        m.type = visualization_msgs::msg::Marker::CUBE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = cx;
        m.pose.position.y = cy;
        m.pose.position.z = 0.25;
        m.pose.orientation.w = 1.0;
        m.scale.x = len_x;
        m.scale.y = width_y;
        m.scale.z = 0.5;
        m.color.r = 0.02;
        m.color.g = 0.02;
        m.color.b = 0.02;
        m.color.a = 1.0;
        m.lifetime = rclcpp::Duration::from_seconds(3.0);
        arr.markers.push_back(m);
    }

    void addFinishLine(visualization_msgs::msg::MarkerArray &arr, int id,
                       const std::string &ns, double x)
    {
        // ── Solid green bar across corridor ──
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "odom";
        m.header.stamp = now();
        m.ns = ns;
        m.id = id;
        m.type = visualization_msgs::msg::Marker::CUBE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = x;
        m.pose.position.y = 0.0;
        m.pose.position.z = 0.05;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.15;
        m.scale.y = 50.0;   // full corridor width y=-25 to y=25
        m.scale.z = 0.08;
        m.color.r = 0.1;
        m.color.g = 0.9;
        m.color.b = 0.1;
        m.color.a = 0.45;   // semi-transparent green
        m.lifetime = rclcpp::Duration::from_seconds(3.0);
        arr.markers.push_back(m);

        // ── Bright outline lines ──
        visualization_msgs::msg::Marker m2;
        m2.header = m.header;
        m2.ns = ns + "_outline";
        m2.id = id + 1000;
        m2.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m2.action = visualization_msgs::msg::Marker::ADD;
        m2.pose.orientation.w = 1.0;
        m2.scale.x = 0.08;
        m2.color.r = 0.0;
        m2.color.g = 0.95;
        m2.color.b = 0.0;
        m2.color.a = 0.9;
        geometry_msgs::msg::Point p1, p2;
        p1.x = x; p1.y = -25.0; p1.z = 0.05;
        p2.x = x; p2.y = 25.0;  p2.z = 0.05;
        m2.points.push_back(p1);
        m2.points.push_back(p2);
        m2.lifetime = rclcpp::Duration::from_seconds(3.0);
        arr.markers.push_back(m2);
    }

    void addWall(visualization_msgs::msg::MarkerArray &arr, int id,
                 const std::string &ns,
                 const geometry_msgs::msg::Point &p1,
                 const geometry_msgs::msg::Point &p2)
    {
        // ── Solid wall segment (wide flat box) ──
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "odom";
        m.header.stamp = now();
        m.ns = ns;
        m.id = id;
        m.type = visualization_msgs::msg::Marker::CUBE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = (p1.x + p2.x) / 2.0;
        m.pose.position.y = (p1.y + p2.y) / 2.0;
        m.pose.position.z = 0.1;
        m.pose.orientation.w = 1.0;
        m.scale.x = std::abs(p2.x - p1.x);
        m.scale.y = 0.35; // wall thickness
        m.scale.z = 0.3;  // wall height
        m.color.r = 0.0;
        m.color.g = 0.0;
        m.color.b = 0.0;
        m.color.a = 0.95;
        m.lifetime = rclcpp::Duration::from_seconds(3.0);
        arr.markers.push_back(m);

        // ── Outline line overlay ──
        visualization_msgs::msg::Marker m2;
        m2.header = m.header;
        m2.ns = ns + "_outline";
        m2.id = id + 1000;
        m2.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m2.action = visualization_msgs::msg::Marker::ADD;
        m2.pose.orientation.w = 1.0;
        m2.scale.x = 0.05;
        m2.color.r = 0.05;
        m2.color.g = 0.05;
        m2.color.b = 0.05;
        m2.color.a = 1.0;
        m2.points.push_back(p1);
        m2.points.push_back(p2);
        m2.lifetime = rclcpp::Duration::from_seconds(3.0);
        arr.markers.push_back(m2);
    }

    void addSphere(visualization_msgs::msg::MarkerArray &arr, int id,
                   const std::string &ns, double x, double y,
                   float r, float g, float b, double size)
    {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "odom";
        m.header.stamp = now();
        m.ns = ns;
        m.id = id;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = x;
        m.pose.position.y = y;
        m.pose.position.z = 0.2;
        m.pose.orientation.w = 1.0;
        m.scale.x = size;
        m.scale.y = size;
        m.scale.z = size;
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        m.color.a = 0.95;
        m.lifetime = rclcpp::Duration::from_seconds(3.0);
        arr.markers.push_back(m);
    }

    void addGoalCylinder(visualization_msgs::msg::MarkerArray &arr, int id,
                         const std::string &ns, double x, double y)
    {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "odom";
        m.header.stamp = now();
        m.ns = ns;
        m.id = id;
        m.type = visualization_msgs::msg::Marker::CYLINDER;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = x;
        m.pose.position.y = y;
        m.pose.position.z = 0.05;
        m.pose.orientation.w = 1.0;
        m.scale.x = 3.0;  // diameter
        m.scale.y = 3.0;
        m.scale.z = 0.15; // flat
        m.color.r = 0.9;
        m.color.g = 0.15;
        m.color.b = 0.15;
        m.color.a = 0.50; // semi-transparent red
        m.lifetime = rclcpp::Duration::from_seconds(3.0);
        arr.markers.push_back(m);

        // Outline ring
        m.id = id + 1000;
        m.type = visualization_msgs::msg::Marker::LINE_STRIP;
        m.scale.x = 0.08;
        m.color.a = 0.85;
        m.points.clear();
        const int n_seg = 32;
        for (int i = 0; i <= n_seg; ++i)
        {
            double angle = 2.0 * M_PI * i / n_seg;
            geometry_msgs::msg::Point p;
            p.x = x + 1.5 * std::cos(angle);
            p.y = y + 1.5 * std::sin(angle);
            p.z = 0.1;
            m.points.push_back(p);
        }
        arr.markers.push_back(m);
    }

    void addZoneLine(visualization_msgs::msg::MarkerArray &arr, int id,
                     const std::string &ns, double x,
                     float r, float g, float b)
    {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "odom";
        m.header.stamp = now();
        m.ns = ns;
        m.id = id;
        m.type = visualization_msgs::msg::Marker::LINE_LIST;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.06;
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        m.color.a = 0.35;

        // Dashed line from y=-14 to y=14
        for (double y = -14.0; y <= 13.0; y += 1.5)
        {
            geometry_msgs::msg::Point p1, p2;
            p1.x = x; p1.y = y;       p1.z = 0.01;
            p2.x = x; p2.y = y + 0.7; p2.z = 0.01;
            m.points.push_back(p1);
            m.points.push_back(p2);
        }
        m.lifetime = rclcpp::Duration::from_seconds(3.0);
        arr.markers.push_back(m);
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wall_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CorridorVisualizer>());
    rclcpp::shutdown();
    return 0;
}
