#ifndef MULTI_USV_COLREGS_ENGINE_H
#define MULTI_USV_COLREGS_ENGINE_H

#include <multi_usv_planner/types.h>

#include <map>
#include <cmath>

namespace MultiUSV
{

/**
 * Deep module: COLREGs rule classification and speed scaling.
 *
 * Interface:  compute(ego, neighbors, agent_id) → Verdict
 * Implementation (deep):  bearing/heading geometry → 5-rule classifier →
 * lateral offset → speed factor.
 *
 * No ROS dependency — pure geometry.
 */
class COLREGsEngine
{
public:
    enum Rule : int { NONE = 0, HEAD_ON, CROSSING_GIVE_WAY, CROSSING_STAND_ON, OVERTAKING };

    /// Verdict: speed reduction + optional lateral shift within corridor.
    /// lateral_shift > 0 = starboard, < 0 = port (meters, ±2.0 max).
    /// When clearance is sufficient, lateral_shift = 0 (pure speed reduction).
    struct Verdict
    {
        Rule rule = NONE;
        int neighbor_id = -1;
        double speed_factor = 1.0;     // speed multiplier, <1 = slow down
        double lateral_shift = 0.0;    // lateral nudge (meters), 0 = no shift
    };

    /// Configuration (exposed for parameter loading)
    double activation_dist = 40.0;
    double give_way_speed = 0.6;
    double stand_on_speed = 0.9;

    /**
     * Compute COLREGs verdict for this USV based on neighbor states.
     * Thread-safe — no mutable state used.
     */
    Verdict compute(const USVState &ego,
                    const std::map<int, NeighborTrajectory> &neighbors,
                    int agent_id) const;
};

}  // namespace MultiUSV

#endif  // MULTI_USV_COLREGS_ENGINE_H
