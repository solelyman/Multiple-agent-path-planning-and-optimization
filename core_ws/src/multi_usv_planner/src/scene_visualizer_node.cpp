#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <cmath>
#include <string>
#include <vector>

struct Island
{
    double x, y, r;
};

static geometry_msgs::msg::Quaternion quatFromRollPitchYaw(double roll, double pitch, double yaw)
{
    geometry_msgs::msg::Quaternion q;
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return q;
}

class SceneVisualizer : public rclcpp::Node
{
public:
    SceneVisualizer() : Node("scene_visualizer")
    {
        pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("scene_markers", rclcpp::QoS(1).transient_local());
        pub_static_alias_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/pedestrian_simulator/static_obstacles", rclcpp::QoS(1).transient_local());
        pub_roadmap_alias_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/roadmap/reference_visual", rclcpp::QoS(1).transient_local());
        timer_ = create_wall_timer(std::chrono::milliseconds(1000), [this]() { publishScene(); });
    }

private:
    void publishScene()
    {
        visualization_msgs::msg::MarkerArray arr;
        int id = 0;

        addStartGoalMarkers(arr, id);
        addCenterlines(arr, id);
        addIslands(arr, id);

        pub_->publish(arr);
        pub_static_alias_->publish(arr);
        pub_roadmap_alias_->publish(arr);
    }

    void addStartGoalMarkers(visualization_msgs::msg::MarkerArray &arr, int &id)
    {
        const std::vector<std::pair<double, double>> starts = {
            {-24.0, -14.0}, {-24.0, -6.0}, {-16.0, -14.0}, {-16.0, -6.0}
        };
        const std::vector<std::pair<double, double>> goals = {
            {42.0, 8.0}, {42.0, 16.0}, {50.0, 8.0}, {50.0, 16.0}
        };

        for (size_t i = 0; i < starts.size(); ++i)
        {
            addSphere(arr, id++, "starts", starts[i].first, starts[i].second, 0.8, 0.1, 0.1, 1.0);
            addSphere(arr, id++, "goals", goals[i].first, goals[i].second, 0.1, 0.8, 0.1, 1.2);
        }
    }

    void addCenterlines(visualization_msgs::msg::MarkerArray &arr, int &id)
    {
        const std::vector<std::vector<std::pair<double, double>>> lanes = {
            {{-32.0, -14.0}, {-14.0, -11.2}, {4.0, -8.2}, {20.0, -3.0}, {38.0, 3.6}, {60.0, 10.0}},
            {{-32.0, -6.0},  {-14.0, -3.2},  {4.0, -0.2}, {20.0, 5.0},  {38.0, 11.6}, {60.0, 18.0}}
        };

        for (size_t i = 0; i < lanes.size(); ++i)
        {
            visualization_msgs::msg::Marker line;
            line.header.frame_id = "odom";
            line.header.stamp = now();
            line.ns = "centerline";
            line.id = id++;
            line.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line.action = visualization_msgs::msg::Marker::ADD;
            line.pose.orientation.w = 1.0;
            line.scale.x = 0.12;
            line.color.r = 0.15;
            line.color.g = 0.35;
            line.color.b = 0.85;
            line.color.a = (i == 0) ? 0.30 : 0.22;
            for (const auto &pt : lanes[i])
            {
                geometry_msgs::msg::Point p;
                p.x = pt.first;
                p.y = pt.second;
                p.z = 0.01;
                line.points.push_back(p);
            }
            arr.markers.push_back(line);
        }
    }
    void addIslands(visualization_msgs::msg::MarkerArray &arr, int &id)
    {
        const std::vector<Island> islands = {
            {6.5, -11.5, 1.10},
            {14.5, -2.5, 1.18},
            {7.5, 4.5, 1.05},
            {15.5, 12.0, 1.12}
        };

        const std::string iceberg_mesh = "file:///home/lu/paper2/core_ws/src/model/models/meshes/ice_floes.glb";

        for (size_t i = 0; i < islands.size(); ++i)
        {
            const auto &island = islands[i];
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "odom";
            m.header.stamp = now();
            m.ns = "small_islands";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.mesh_resource = iceberg_mesh;
            m.mesh_use_embedded_materials = true;
            m.pose.position.x = island.x;
            m.pose.position.y = island.y;
            m.pose.position.z = -0.28;
            m.pose.orientation = quatFromRollPitchYaw(0.0, 0.0, (i % 2 == 0) ? 0.18 : -0.22);
            m.scale.x = island.r * 0.72;
            m.scale.y = island.r * 0.72;
            m.scale.z = island.r * 0.72;
            m.color.r = 1.0;
            m.color.g = 1.0;
            m.color.b = 1.0;
            m.color.a = 1.0;
            arr.markers.push_back(m);
        }
    }

    void addSphere(visualization_msgs::msg::MarkerArray &arr, int id, const std::string &ns, double x, double y, float r, float g, float b, double size)
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
        arr.markers.push_back(m);
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_static_alias_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_roadmap_alias_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SceneVisualizer>());
    rclcpp::shutdown();
    return 0;
}
