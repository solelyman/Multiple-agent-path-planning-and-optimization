#include <multi_usv_planner/constraint_builder.h>
#include <multi_usv_planner/acados_contouring_solver.h>

namespace MultiUSV
{

/// @brief Predict neighbor position @p tk seconds into the future.
static Eigen::Vector2d predictNeighbor(
    const NeighborTrajectory &nb, double tk)
{
    if (nb.predicted_traj.points.size() >= 3)
    {
        double t_step = nb.predicted_traj.dt;
        int idx = static_cast<int>(std::round(tk / t_step));
        idx = std::clamp(idx, 0, static_cast<int>(nb.predicted_traj.points.size()) - 1);
        const auto &pt = nb.predicted_traj.points[idx];
        return {pt.x, pt.y};
    }
    const double speed = nb.current_state.v;
    const double psi = nb.current_state.psi;
    return {
        nb.current_state.x + std::cos(psi) * speed * tk,
        nb.current_state.y + std::sin(psi) * speed * tk
    };
}

/// @brief Get reference path position at time @p tk.
static Eigen::Vector2d refPos(const ReferencePath &ref, double tk)
{
    if (ref.points.empty())
        return Eigen::Vector2d::Zero();
    double total = ref.length();
    double arclen = std::min(tk * 1.2, total);
    double accum = 0.0;
    for (size_t i = 0; i + 1 < ref.points.size(); ++i)
    {
        const Eigen::Vector2d &a = ref.points[i];
        const Eigen::Vector2d &b = ref.points[i + 1];
        double seg = (b - a).norm();
        if (accum + seg >= arclen || i + 2 >= ref.points.size())
        {
            double t = (arclen - accum) / std::max(seg, 1e-6);
            t = std::clamp(t, 0.0, 1.0);
            return a + t * (b - a);
        }
        accum += seg;
    }
    return ref.points.back();
}

StageConstraintSet ConstraintBuilder::build(
    const ReferencePath &contour_ref,
    const USVState & /*ego*/,
    const std::map<std::string, DynamicObstacle> &dynamic_obstacles,
    const std::map<int, NeighborTrajectory> &neighbors,
    const std::vector<Obstacle> &static_obstacles,
    const std::vector<int> & /*active_ids*/,
    int /*agent_id*/,
    double robot_radius,
    double obstacle_clearance,
    double dt,
    int N)
{
    constexpr double NEIGHBOR_RADIUS = 1.0;  // ~half Heron beam
    StageConstraintSet sc_set;
    sc_set.reserve(N);

    for (int k = 0; k < N; ++k)
    {
        double tk = dt * (k + 1);
        const Eigen::Vector2d ego_pred = refPos(contour_ref, tk);

        struct Candidate {
            double ox, oy, r;
            double dist;  // distance to predicted ego position
        };
        std::vector<Candidate> cands;
        cands.reserve(32);

        // 1) Static obstacles (islands) — fixed position, no velocity
        for (const auto &obs : static_obstacles)
        {
            if (obs.radius < 0.01) continue;
            double d = (Eigen::Vector2d{obs.x, obs.y} - ego_pred).norm();
            cands.push_back({
                obs.x, obs.y,
                obs.radius + robot_radius + obstacle_clearance,
                d
            });
        }

        // 2) Dynamic obstacles (debris, vessel_e/f) — constant-velocity prediction
        for (const auto &pair : dynamic_obstacles)
        {
            const auto &obs = pair.second;
            if (!obs.received) continue;
            double ox = obs.x + obs.vx * tk;
            double oy = obs.y + obs.vy * tk;
            double d = (Eigen::Vector2d{ox, oy} - ego_pred).norm();
            cands.push_back({
                ox, oy,
                obs.radius + robot_radius + obstacle_clearance,
                d
            });
        }

        // 3) Neighbor USVs — trajectory prediction
        for (const auto &pair : neighbors)
        {
            const auto &nb = pair.second;
            if (!nb.received) continue;
            const Eigen::Vector2d pred = predictNeighbor(nb, tk);
            double d = (pred - ego_pred).norm();
            cands.push_back({
                pred.x(), pred.y(),
                NEIGHBOR_RADIUS + robot_radius + obstacle_clearance,
                d
            });
        }

        // Sort by distance to predicted ego, take N_ELLIPSOIDS
        std::sort(cands.begin(), cands.end(), [](const Candidate &a, const Candidate &b) {
                return a.dist < b.dist;
            });

        StageConstraints sc;
        int n_to_take = std::min(static_cast<int>(cands.size()),
                                 AcadosContouringSolver::N_ELLIPSOIDS);
        for (int i = 0; i < n_to_take; ++i)
        {
            EllipsoidObstacle ell;
            ell.ox = cands[i].ox;
            ell.oy = cands[i].oy;
            ell.r  = cands[i].r;
            sc.ellipsoids.push_back(ell);
        }

        // Pad remaining slots with dummies (r=0 → no cost)
        while (static_cast<int>(sc.ellipsoids.size()) < AcadosContouringSolver::N_ELLIPSOIDS)
        {
            EllipsoidObstacle ell;
            ell.ox = 0.0; ell.oy = 0.0; ell.r = 0.0;
            sc.ellipsoids.push_back(ell);
        }

        sc_set.push_back(std::move(sc));
    }

    return sc_set;
}

}  // namespace MultiUSV
