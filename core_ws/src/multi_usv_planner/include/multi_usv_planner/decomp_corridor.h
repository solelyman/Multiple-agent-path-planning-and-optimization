#ifndef MULTI_USV_DECOMP_CORRIDOR_H
#define MULTI_USV_DECOMP_CORRIDOR_H

#include <decomp_util/ellipsoid_decomp.h>

#include <Eigen/Dense>
#include <vector>
#include <utility>

namespace MultiUSV
{

/**
 * Thin wrapper around EllipsoidDecomp2D.
 *
 * 1) set_obs(occupied_points)
 * 2) dilate(ref_path)   → computes Safe Flight Corridor
 * 3) getCorridorBounds(stage) → returns (y_low, y_high) at that path segment
 */
class DecompCorridor
{
public:
    DecompCorridor()
    {
        decomp_.set_local_bbox(Vec2f(20.0, 20.0));
    }

    /// Feed obstacle occupied positions (from PRM / dynamic obstacles)
    void setObstacles(const vec_Vec2f &obs)
    {
        occ_pos_ = obs;
        decomp_.set_obs(obs);
    }

    /// Dilate along the reference path (must have at least 2 points)
    void dilate(const vec_Vec2f &path)
    {
        if (path.size() < 2) return;
        decomp_.dilate(path, 0.0);
        path_ = path;
        constraints_ = decomp_.get_constraints();
    }

    /// Number of segments = path.size()-1
    int segments() const { return static_cast<int>(constraints_.size()); }

    /// Get approximate corridor y-bounds at the *midpoint* of segment idx
    /// Returns (y_min, y_max) or (path_y±6) fallback
    std::pair<double, double> getYBounds(int idx) const
    {
        if (idx < 0 || idx >= static_cast<int>(constraints_.size()))
            return {-12.0, 12.0};

        // The polyhedron has half-planes: a1*x + a2*y ≤ b
        // Find the max/min y feasible within the corridor at the path point
        const auto &c = constraints_[idx];
        double y_low = -100.0, y_high = 100.0;
        double path_y = (idx < static_cast<int>(path_.size())) ? path_[idx].y() : 0.0;

        for (int i = 0; i < c.A_.rows(); ++i)
        {
            double a1 = c.A_.row(i)[0];
            double a2 = c.A_.row(i)[1];
            double b  = c.b_(i);

            // Half-plane: a1*x + a2*y ≤ b
            // At path x, solve for y bound
            double path_x = (idx < static_cast<int>(path_.size())) ? path_[idx].x() : 0.0;
            if (std::abs(a2) > 1e-6)
            {
                double y_bound = (b - a1 * path_x) / a2;
                if (a2 > 0) // y ≤ bound
                    y_high = std::min(y_high, y_bound);
                else        // y ≥ bound
                    y_low = std::max(y_low, y_bound);
            }
        }

        if (y_low < -50.0) y_low = path_y - 6.0;
        if (y_high > 50.0) y_high = path_y + 6.0;
        return {y_low, y_high};
    }

    /// Get the polyhedron constraint for segment idx (for reference)
    const LinearConstraint<2> &getConstraint(int idx) const
    {
        static LinearConstraint<2> empty;
        if (idx < 0 || idx >= static_cast<int>(constraints_.size()))
            return empty;
        return constraints_[idx];
    }

    const vec_E<Polyhedron<2>> &polyhedrons() const { return decomp_.get_polyhedrons(); }

private:
    EllipsoidDecomp2D decomp_;
    vec_Vec2f occ_pos_;
    vec_Vec2f path_;
    vec_E<LinearConstraint<2>> constraints_;
};

}  // namespace MultiUSV

#endif  // MULTI_USV_DECOMP_CORRIDOR_H
