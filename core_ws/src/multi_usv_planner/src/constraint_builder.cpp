#include <multi_usv_planner/constraint_builder.h>
#include <multi_usv_planner/acados_contouring_solver.h>
#include <multi_usv_planner/decomp_corridor.h>

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
    double dynamic_obstacle_growth,
    double dt,
    int N,
    double corridor_y_min,
    double corridor_y_max)
{
    constexpr double NEIGHBOR_RADIUS = 1.0;  // ~half Heron beam
    StageConstraintSet sc_set;
    sc_set.reserve(N);

    // ──  DecompUtil corridor  ──
    DecompCorridor decomp_corridor;
    bool decomp_ok = false;
    if (contour_ref.points.size() >= 2)
    {
        vec_Vec2f occ;
        occ.reserve(static_obstacles.size() + 2);
        for (const auto &obs : static_obstacles)
        {
            if (obs.radius > 0.01)
                occ.push_back(Vec2f(obs.x, obs.y));
        }
        vec_Vec2f path;
        path.reserve(contour_ref.points.size());
        for (const auto &pt : contour_ref.points)
            path.push_back(Vec2f(pt.x(), pt.y()));
        decomp_corridor.setObstacles(occ);
        decomp_corridor.dilate(path);
        decomp_ok = decomp_corridor.segments() > 0;
    }

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

        // Corridor walls — DecompUtil-aware

        // 1) Static obstacles (islands) — fixed position, no velocity
        for (const auto &obs : static_obstacles)
        {
            if (obs.radius < 0.01) continue;
            double d = (Eigen::Vector2d{obs.x, obs.y} - ego_pred).norm();
            cands.push_back({
                obs.x, obs.y,
                obs.radius + robot_radius + obstacle_clearance +
                    std::max(0.0, dynamic_obstacle_growth) * tk,
                d
            });
        }

        {
            double yp = ego_pred.y();
            double y_min = corridor_y_min;
            double y_max = corridor_y_max;
            double wall_margin = robot_radius + obstacle_clearance;

            if (decomp_ok)
            {
                double total_arclen = 0.0;
                std::vector<double> seg_lens;
                seg_lens.reserve(contour_ref.points.size());
                for (size_t i = 1; i < contour_ref.points.size(); ++i)
                {
                    double ds = (contour_ref.points[i] - contour_ref.points[i-1]).norm();
                    seg_lens.push_back(ds);
                    total_arclen += ds;
                }
                double frac = std::min(tk / (dt * N), 1.0);
                double target_arclen = frac * total_arclen;
                double accum = 0.0;
                int seg_idx = static_cast<int>(seg_lens.size()) - 1;
                for (size_t i = 0; i < seg_lens.size(); ++i)
                {
                    accum += seg_lens[i];
                    if (accum >= target_arclen)
                    {
                        seg_idx = static_cast<int>(i);
                        break;
                    }
                }
                auto bounds = decomp_corridor.getYBounds(seg_idx);
                y_min = std::max(y_min, bounds.first);
                y_max = std::min(y_max, bounds.second);
            }

            double dist_bottom = std::abs(yp - y_min);
            double dist_top    = std::abs(yp - y_max);
            cands.push_back({ego_pred.x(), y_min, wall_margin, dist_bottom});
            cands.push_back({ego_pred.x(), y_max, wall_margin, dist_top});
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
                NEIGHBOR_RADIUS + robot_radius + obstacle_clearance +
                    0.5 * std::max(0.0, dynamic_obstacle_growth) * tk,
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
