#include <multi_usv_planner/colregs_engine.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

namespace MultiUSV
{

static double normalizeAngle(double a)
{
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

COLREGsEngine::Verdict COLREGsEngine::compute(
    const USVState &ego,
    const std::map<int, NeighborTrajectory> &neighbors,
    int agent_id) const
{
    // 多邻居聚合：取最保守策略
    //   GIVE-WAY/HEAD-ON (factor<1.0)：取最小 factor
    //   STAND-ON (factor>1.0)：仅当无冲突时允许加速
    double min_factor = 1.0;
    bool has_conflict = false;
    COLREGsEngine::Rule governing_rule = NONE;
    double best_standon_dist = 1e9;

    for (const auto &pair : neighbors)
    {
        if (pair.first == agent_id) continue;
        const auto &nb = pair.second;
        const auto &ns = nb.current_state;

        // Range gating
        double dx = ns.x - ego.x;
        double dy = ns.y - ego.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist > activation_dist || dist < 0.5)
            continue;

        // brg: neighbor bearing relative to ego heading
        double brg = std::atan2(dy, dx) - ego.psi;
        brg = normalizeAngle(brg);
        double abs_brg = std::abs(brg);

        // hdg:  neighbor heading relative to ego heading
        double hdg = ns.psi - ego.psi;
        hdg = normalizeAngle(hdg);
        double abs_hdg = std::abs(hdg);

        // along-track:  neighbor's position projected onto ego's heading
        // positive = neighbor is ahead of ego along direction of travel
        double along_track = dx * std::cos(ego.psi) + dy * std::sin(ego.psi);

        // cross-track: lateral offset perpendicular to ego's heading
        double cross_track = std::abs(-dx * std::sin(ego.psi) + dy * std::cos(ego.psi));

        // Bearing from neighbor back to ego (neighbor's frame)
        double bearing_from_nb = std::atan2(-dy, -dx) - ns.psi;
        bearing_from_nb = normalizeAngle(bearing_from_nb);

        // Relative closing speed (positive = getting closer)
        double rel_speed = ns.v * std::cos(hdg) - ego.v;

        Verdict v;
        v.neighbor_id = pair.first;

        // COLREGs Rule Classification (5 rules)

        const double HEAD_ON_HDG_THRESH = 2.618;  // 150° — nearly opposing courses
        const double PARALLEL_THRESH    = 0.525;  // 30°  — similar courses
        const double CROSSING_THRESH_LOW = 0.525;  // 30°
        const double CROSSING_THRESH_HI  = 2.618;  // 150°
        const double SECTOR_AHEAD       = 1.047;   // 60°  — forward sector for approaching detection

        // 1) HEAD-ON — Rule 14
        //   Opposing courses (abs_hdg > 150°), bearing roughly ahead
        if (abs_hdg > HEAD_ON_HDG_THRESH && abs_brg < SECTOR_AHEAD)
        {
            v.rule = HEAD_ON;
            // Both vessels slow equally
            double ramp = std::clamp((activation_dist - dist) / activation_dist, 0.0, 1.0);
            v.speed_factor = 1.0 - 0.30 * ramp;
            // 不横向偏移 — 纯让速防止推入障碍物
        }
        // 2) OVERTAKING — Rule 13
        //    Similar course (abs_hdg < 30°)
        else if (abs_hdg < PARALLEL_THRESH && cross_track < 25.0)
        {
            if (along_track > 0.0)
            {
                // We are BEHIND → 检查前车是否在避障减速
                // 如果前车速度<0.8 m/s（约desired的65%），说明在避障
                // 此时不应超车——超车会把我们推进障碍物区
                if (ns.v < 0.8)
                {
                    v.rule = NONE;
                    v.speed_factor = 1.0;  // 不触发超车，后续跟速逻辑接管
                }
                else
                {
                    // We are BEHIND → GIVE-WAY (overtaking vessel): 减速
                    v.rule = OVERTAKING;
                    double ramp = std::clamp((activation_dist * 0.5 - along_track) / (activation_dist * 0.5), 0.0, 1.0);
                    if (ramp > 0.0)
                        v.speed_factor = std::max(give_way_speed, 1.0 - 0.50 * ramp);
                    else
                        v.speed_factor = 1.0;
                }
            }
            else
            {
                // We are AHEAD → STAND-ON (being overtaken): 加速拉开距离
                // 但如果我们自己也在避障减速（速度<0.8），则不应加速
                if (ego.v < 0.8)
                {
                    v.rule = NONE;
                    v.speed_factor = 1.0;
                }
                else
                {
                    v.rule = CROSSING_STAND_ON;
                    v.speed_factor = 1.20;  // +20%, 上限由节点 clamp
                }
            }
        }
        // 3) CROSSING — Rule 15
        //    Courses between 30° and 150°
        else if (abs_hdg > CROSSING_THRESH_LOW && abs_hdg < CROSSING_THRESH_HI)
        {
            if (dist < 30.0 && rel_speed > 0.0)
            {
                if (bearing_from_nb > -SECTOR_AHEAD && bearing_from_nb < SECTOR_AHEAD)
                {
                    if (brg > 0.0)
                    {
                        // Starboard approach → GIVE-WAY: 减速
                        v.rule = CROSSING_GIVE_WAY;
                        double ramp = std::clamp((activation_dist - dist) / (activation_dist - 10.0),
                                                  0.0, 1.0);
                        v.speed_factor = std::max(give_way_speed, 1.0 - 0.50 * ramp);
                    }
                    else
                    {
                        // Port approach → STAND-ON: 加速拉开距离
                        v.rule = CROSSING_STAND_ON;
                        v.speed_factor = 1.15;  // +15%
                    }
                }
                else
                {
                    v.rule = NONE;
                    v.speed_factor = 1.0;
                }
            }
            else
            {
                v.rule = NONE;
                v.speed_factor = 1.0;
            }
        }
        else
        {
            v.rule = NONE;
            v.speed_factor = 1.0;
        }

        if (v.speed_factor < 1.0)
        {
            has_conflict = true;
            if (v.speed_factor < min_factor)
            {
                min_factor = v.speed_factor;
                governing_rule = v.rule;
            }
        }
        else if (!has_conflict && v.speed_factor > 1.0 && dist < best_standon_dist)
        {
            min_factor = v.speed_factor;
            governing_rule = v.rule;
            best_standon_dist = dist;
        }
    }  // end for neighbors

    Verdict result;
    result.rule = governing_rule;
    result.speed_factor = min_factor;
    result.lateral_shift = 0.0;
    return result;
}

}  // namespace MultiUSV
