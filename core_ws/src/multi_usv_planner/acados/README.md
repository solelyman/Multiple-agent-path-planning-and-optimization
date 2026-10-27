# Heron single-vessel acados integration

This directory is reserved for the minimal acados-based single-vessel route in
`multi_usv_planner`, using the Heron model context only.

## Scope
- keep the current ROS 2 Jazzy project structure
- do not copy the whole `single` project
- do not add `ros_tools` as a new dependency here
- use a simplified planar Euler-Lagrange surge-yaw model

## Current optimization model
States:
- `x`, `y`, `psi`, `u`, `r`

Controls:
- `tau_u`, `tau_r`

Dynamics:
- `x_dot = u cos(psi)`
- `y_dot = u sin(psi)`
- `psi_dot = r`
- `u_dot = (tau_u - d_u u) / m11`
- `r_dot = (tau_r - d_r r) / Izz`

## Minimal environment requirements
Required:
- `casadi`
- `acados_template`
- `libacados.so`
- `ACADOS_SOURCE_DIR`

Current host status is checked by `AcadosSolver::initialize()`.

## Recommended installation style
Prefer an external/local installation of acados, then export:
- `ACADOS_SOURCE_DIR`
- `LD_LIBRARY_PATH`
- `PYTHONPATH`

Do not commit generated acados binaries until the pipeline is stable.
Use `generated/` only as a transient output directory.

## Planned next step
1. install acados minimally
2. generate Heron single-vessel OCP solver
3. connect generated solver into `acados_solver.cpp`
4. validate in `heron_single_node`
