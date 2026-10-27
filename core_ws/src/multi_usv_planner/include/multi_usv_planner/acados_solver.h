#ifndef MULTI_USV_PLANNER_ACADOS_SOLVER_H
#define MULTI_USV_PLANNER_ACADOS_SOLVER_H

#include <multi_usv_planner/types.h>
#include <multi_usv_planner/mpc_solver.h>
#include <string>

struct heron_single_solver_capsule;

namespace MultiUSV
{

class AcadosSolver
{
public:
    struct Params
    {
        int N = 20;
        double dt = 0.2;

        double m11 = 28.0;
        double Izz = 10.0;
        double d_u = 16.45;
        double d_r = 6.0;

        double max_u = 2.0;
        double max_r = 1.0;
        double max_tau_u = 20.0;
        double max_tau_r = 10.0;

        double weight_pos = 10.0;
        double weight_psi = 2.0;
        double weight_u = 1.0;
        double weight_r = 1.0;
        double weight_tau_u = 0.1;
        double weight_tau_r = 0.1;
    };

    AcadosSolver() = default;

    void setParams(const Params &p) { params_ = p; }
    Params &params() { return params_; }
    const Params &params() const { return params_; }

    bool initialize();
    bool available() const { return initialized_; }
    const std::string &statusMessage() const { return status_message_; }

    Trajectory solve(const USVState &state,
                     const ReferencePath &ref_path,
                     const std::vector<Obstacle> &obstacles,
                     double goal_x,
                     double goal_y);

private:
    Params params_;
    bool initialized_ = false;
    std::string status_message_ = "acados not initialized";
    ::heron_single_solver_capsule *capsule_ = nullptr;
};

} // namespace MultiUSV

#endif
