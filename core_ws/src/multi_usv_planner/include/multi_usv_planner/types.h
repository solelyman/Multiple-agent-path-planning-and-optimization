#ifndef MULTI_USV_TYPES_H
#define MULTI_USV_TYPES_H

#include <Eigen/Dense>
#include <vector>
#include <cmath>

namespace MultiUSV
{

//2D Pose with heading
struct Pose2D
{
    double x = 0.0, y = 0.0, theta = 0.0;
    Pose2D() = default;
    Pose2D(double x, double y, double theta) : x(x), y(y), theta(theta) {}

    Eigen::Vector2d pos() const { return Eigen::Vector2d(x, y); }
    void setPos(const Eigen::Vector2d &p) { x = p.x(); y = p.y(); }
    static double normalizeAngle(double a)
    {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }
};

// Trajectory point
struct TrajectoryPoint
{
    double x = 0.0, y = 0.0, psi = 0.0, v = 0.0, omega = 0.0;
    double s = 0.0;  // progress along reference path
};

struct ControlPoint
{
    double a = 0.0, omega = 0.0;
};

// Trajectory
struct Trajectory
{
    std::vector<TrajectoryPoint> points;
    std::vector<ControlPoint> controls;
    double dt = 0.2;
    int n = 0;

    void clear() { points.clear(); controls.clear(); n = 0; }
    int size() const { return (int)points.size(); }
    void add(double x, double y, double psi, double v, double omega)
    {
        points.push_back({x, y, psi, v, omega, 0.0});
        n++;
    }
};

// Static obstacle
struct Obstacle
{
    double x = 0.0, y = 0.0, radius = 1.0;
};

struct DynamicObstacle
{
    std::string name;
    double x = 0.0, y = 0.0;
    double vx = 0.0, vy = 0.0;
    double radius = 1.0;
    bool received = false;
    double last_update_sec = 0.0;  // ROS epoch seconds of last odometry callback
};

// USV state
struct USVState
{
    double x = 0.0, y = 0.0, psi = 0.0;
    double v = 0.0, omega = 0.0;
    bool received = false;

    Eigen::Vector2d pos() const { return Eigen::Vector2d(x, y); }
};

// Neighbor trajectory
struct NeighborTrajectory
{
    int id = -1;
    USVState current_state;
    Trajectory predicted_traj;
    bool received = false;
};

// Neighbor selector
struct NeighborSelector
{
    int k = 2;                       
    double prediction_window = 5.0; 
    double danger_distance = 8.0;  

    std::vector<int> select(const USVState &ego,
                            const std::vector<NeighborTrajectory> &neighbors) const
    {
        struct Scored { int idx; double dist; };
        std::vector<Scored> scored;
        for (size_t i = 0; i < neighbors.size(); i++)
        {
            if (!neighbors[i].received) continue;

            // Judge by the distance to the end of the prediction window 
            double dist = -1.0;
            if (neighbors[i].predicted_traj.size() > 0)
            {
                int look_idx = std::min((int)(prediction_window / neighbors[i].predicted_traj.dt + 0.5),
                                        neighbors[i].predicted_traj.size() - 1);
                auto &pt = neighbors[i].predicted_traj.points[look_idx];
                dist = (ego.pos() - Eigen::Vector2d(pt.x, pt.y)).norm();
            }
            else
            {
                const double t = std::max(0.0, prediction_window);
                const double px = neighbors[i].current_state.x +
                                  neighbors[i].current_state.v *
                                      std::cos(neighbors[i].current_state.psi) * t;
                const double py = neighbors[i].current_state.y +
                                  neighbors[i].current_state.v *
                                      std::sin(neighbors[i].current_state.psi) * t;
                dist = (ego.pos() - Eigen::Vector2d(px, py)).norm();
            }
            if (dist < danger_distance)
                scored.push_back({(int)i, dist});
        }
        std::sort(scored.begin(), scored.end(),
                  [](const Scored &a, const Scored &b) { return a.dist < b.dist; });

        std::vector<int> result;
        for (size_t i = 0; i < scored.size() && (int)i < k; i++)
            result.push_back(scored[i].idx);
        return result;
    }
};

} // namespace MultiUSV

#endif
