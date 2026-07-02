#ifndef MULTI_USV_CONSTRAINT_BUILDER_H
#define MULTI_USV_CONSTRAINT_BUILDER_H

#include <multi_usv_planner/types.h>
#include <multi_usv_planner/acados_contouring_solver.h>

#include <Eigen/Dense>
#include <vector>
#include <map>
#include <string>
#include <utility>
#include <cmath>
#include <algorithm>

namespace MultiUSV
{

/**
 * Generates per-stage ellipsoid constraints for dynamic obstacles.
 *
 * Each dynamic obstacle → EllipsoidObstacle (circle):
 *   (x-ox)² + (y-oy)² - r² <= 0
 * where r = obs.radius + robot_radius + clearance.
 *
 * Linear motion prediction: ox(t) = x₀ + vx*t, oy(t) = y₀ + vy*t.
 * Bounded con_h_expr constraints in acados solver, handled by multi-RTI loop.
 */
class ConstraintBuilder
{
public:
    StageConstraintSet build(
        const ReferencePath &contour_ref,
        const USVState &ego,
        const std::map<std::string, DynamicObstacle> &dynamic_obstacles,
        const std::map<int, NeighborTrajectory> &neighbors,
        const std::vector<Obstacle> &static_obstacles,
        const std::vector<int> &active_ids,
        int agent_id,
        double robot_radius,
        double obstacle_clearance,
        double dt,
        int N);
};

}  // namespace MultiUSV

#endif  // MULTI_USV_CONSTRAINT_BUILDER_H
