#include <multi_usv_planner/acados_solver.h>

#include <cstdlib>
#include <cstdio>
#include <array>
#include <sstream>
#include <algorithm>
#include <vector>
#include <limits>

extern "C" {
#include "acados_solver_heron_single.h"
}

namespace MultiUSV
{

namespace
{
std::string runCommand(const char *cmd)
{
    std::array<char, 256> buffer{};
    std::string result;
    FILE *raw = popen(cmd, "r");
    if (!raw) return result;
    while (fgets(buffer.data(), static_cast<int>(buffer.size()), raw) != nullptr)
    {
        result += buffer.data();
    }
    pclose(raw);
    while (!result.empty() && (result.back() == '\n' || result.back() == '\r'))
    {
        result.pop_back();
    }
    return result;
}
}

bool AcadosSolver::initialize()
{
    initialized_ = false;

    const char *src_dir = std::getenv("ACADOS_SOURCE_DIR");
    const std::string lib_path = runCommand(
        "find ${ACADOS_SOURCE_DIR:-/tmp} -path '*/libacados.so' 2>/dev/null | head -1");
    const char *py_bin = std::getenv("ACADOS_PYTHON");
    std::string python_cmd = py_bin && std::string(py_bin).size() ? std::string(py_bin) : std::string("python3");
    const std::string py_check = runCommand(
        (python_cmd + " - <<'PY'\n"
        "try:\n"
        "    import acados_template\n"
        "    print('OK')\n"
        "except Exception as e:\n"
        "    print('MISSING:' + str(e))\n"
        "PY").c_str());

    std::ostringstream oss;
    if (!src_dir || std::string(src_dir).empty())
    {
        oss << "ACADOS_SOURCE_DIR not set";
        status_message_ = oss.str();
        return false;
    }
    if (lib_path.empty())
    {
        oss << "libacados.so not found";
        status_message_ = oss.str();
        return false;
    }
    if (py_check.rfind("OK", 0) != 0)
    {
        oss << "acados_template unavailable: " << py_check;
        status_message_ = oss.str();
        return false;
    }

    if (capsule_)
    {
        heron_single_acados_free(capsule_);
        heron_single_acados_free_capsule(capsule_);
        capsule_ = nullptr;
    }

    capsule_ = heron_single_acados_create_capsule();
    if (!capsule_)
    {
        status_message_ = "failed to create acados capsule";
        return false;
    }

    int status = heron_single_acados_create(capsule_);
    if (status != 0)
    {
        status_message_ = "heron_single_acados_create failed";
        return false;
    }

    initialized_ = true;
    status_message_ = "acados solver ready";
    return true;
}

Trajectory AcadosSolver::solve(const USVState &state,
                               const ReferencePath &ref_path,
                               const std::vector<Obstacle> &obstacles,
                               double goal_x,
                               double goal_y)
{
    (void)obstacles;

    Trajectory traj;
    traj.dt = params_.dt;

    if (!initialized_ || !capsule_)
        return traj;

    ocp_nlp_config *nlp_config = heron_single_acados_get_nlp_config(capsule_);
    ocp_nlp_dims *nlp_dims = heron_single_acados_get_nlp_dims(capsule_);
    ocp_nlp_in *nlp_in = heron_single_acados_get_nlp_in(capsule_);
    ocp_nlp_out *nlp_out = heron_single_acados_get_nlp_out(capsule_);

    double x0[HERON_SINGLE_NX] = {state.x, state.y, state.psi, state.v, state.omega};
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", x0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", x0);

    double ref_length = ref_path.length();
    double s_start = 0.0;
    bool has_ref = !ref_path.points.empty() && ref_length > 1e-6;
    if (has_ref)
        ref_path.findClosest(Eigen::Vector2d(state.x, state.y), s_start);

    auto speed_ref_at = [&](double s_ref)
    {
        double remain = has_ref ? std::max(ref_length - s_ref, 0.0) : std::max(std::hypot(goal_x - state.x, goal_y - state.y) - s_ref, 0.0);
        double v_scale = std::clamp(remain / 8.0, 0.25, 1.0);
        return params_.max_u * 0.7 * v_scale;
    };

    for (int i = 0; i < HERON_SINGLE_N; ++i)
    {
        double xr = state.x;
        double yr = state.y;
        double psi_ref = std::atan2(goal_y - state.y, goal_x - state.x);
        double u_ref = params_.max_u * 0.7;

        if (has_ref)
        {
            double s_ref = std::min(s_start + params_.max_u * 0.7 * params_.dt * (i + 1), ref_length);
            Eigen::Vector2d p_ref = ref_path.sample(s_ref);
            xr = p_ref.x();
            yr = p_ref.y();
            psi_ref = ref_path.heading(s_ref);
            u_ref = speed_ref_at(s_ref);
        }
        else
        {
            double alpha = static_cast<double>(i + 1) / static_cast<double>(HERON_SINGLE_N);
            double s_ref = alpha * std::hypot(goal_x - state.x, goal_y - state.y);
            xr = state.x + alpha * (goal_x - state.x);
            yr = state.y + alpha * (goal_y - state.y);
            u_ref = speed_ref_at(s_ref);
        }

        double yref[HERON_SINGLE_NY] = {xr, yr, psi_ref, u_ref, 0.0, 0.0, 0.0};
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "x", x0);
        double u_guess[HERON_SINGLE_NU] = {0.0, 0.0};
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "u", u_guess);
    }

    double goal_heading = std::atan2(goal_y - state.y, goal_x - state.x);
    if (has_ref)
        goal_heading = ref_path.heading(ref_path.length());
    double yref_e[HERON_SINGLE_NYN] = {goal_x, goal_y, goal_heading, 0.0, 0.0};
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, HERON_SINGLE_N, "yref", yref_e);
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, HERON_SINGLE_N, "x", x0);

    int status = heron_single_acados_solve(capsule_);
    if (status != 0)
    {
        status_message_ = "acados solve failed";
        return traj;
    }

    for (int i = 0; i <= HERON_SINGLE_N; ++i)
    {
        double x[HERON_SINGLE_NX] = {0.0};
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "x", x);

        double tau[HERON_SINGLE_NU] = {0.0, 0.0};
        if (i < HERON_SINGLE_N)
        {
            ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "u", tau);
        }

        traj.add(x[0], x[1], x[2], x[3], x[4]);
    }

    status_message_ = "acados solve ok";
    return traj;
}

} // namespace MultiUSV
