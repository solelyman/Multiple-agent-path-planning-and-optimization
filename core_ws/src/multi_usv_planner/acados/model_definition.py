"""
Heron single-vessel acados OCP generator.

Simplified planar Euler-Lagrange surge-yaw model:
    x_dot   = u * cos(psi)
    y_dot   = u * sin(psi)
    psi_dot = r
    u_dot   = (tau_u - d_u * u) / m11
    r_dot   = (tau_r - d_r * r) / Izz
"""

from dataclasses import dataclass
from pathlib import Path

import casadi as ca
import numpy as np
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver


@dataclass
class HeronAcadosParams:
    N: int = 20
    Tf: float = 4.0
    m11: float = 28.0
    Izz: float = 10.0
    d_u: float = 16.45
    d_r: float = 6.0
    max_u: float = 2.0
    max_r: float = 1.0
    max_tau_u: float = 20.0
    max_tau_r: float = 10.0
    weight_pos: float = 10.0
    weight_psi: float = 2.0
    weight_u: float = 1.0
    weight_r: float = 1.0
    weight_tau_u: float = 0.1
    weight_tau_r: float = 0.1


def build_model(params: HeronAcadosParams) -> AcadosModel:
    model = AcadosModel()
    model.name = 'heron_single'

    x = ca.SX.sym('x')
    y = ca.SX.sym('y')
    psi = ca.SX.sym('psi')
    u = ca.SX.sym('u')
    r = ca.SX.sym('r')
    states = ca.vertcat(x, y, psi, u, r)

    tau_u = ca.SX.sym('tau_u')
    tau_r = ca.SX.sym('tau_r')
    controls = ca.vertcat(tau_u, tau_r)

    xdot = ca.SX.sym('xdot')
    ydot = ca.SX.sym('ydot')
    psidot = ca.SX.sym('psidot')
    udot = ca.SX.sym('udot')
    rdot = ca.SX.sym('rdot')
    states_dot = ca.vertcat(xdot, ydot, psidot, udot, rdot)

    f_expl = ca.vertcat(
        u * ca.cos(psi),
        u * ca.sin(psi),
        r,
        (tau_u - params.d_u * u) / params.m11,
        (tau_r - params.d_r * r) / params.Izz,
    )

    model.x = states
    model.xdot = states_dot
    model.u = controls
    model.f_expl_expr = f_expl
    model.f_impl_expr = states_dot - f_expl
    return model


def build_ocp(params: HeronAcadosParams, export_dir: Path) -> AcadosOcp:
    model = build_model(params)
    ocp = AcadosOcp()
    ocp.model = model
    ocp.code_export_directory = str(export_dir)

    nx = model.x.rows()
    nu = model.u.rows()
    ny = nx + nu
    ny_e = nx

    ocp.dims.N = params.N
    ocp.solver_options.tf = params.Tf

    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'

    ocp.cost.W = np.diag([
        params.weight_pos,
        params.weight_pos,
        params.weight_psi,
        params.weight_u,
        params.weight_r,
        params.weight_tau_u,
        params.weight_tau_r,
    ])
    ocp.cost.W_e = np.diag([
        params.weight_pos,
        params.weight_pos,
        params.weight_psi,
        params.weight_u,
        params.weight_r,
    ])

    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[0, 0] = 1.0
    ocp.cost.Vx[1, 1] = 1.0
    ocp.cost.Vx[2, 2] = 1.0
    ocp.cost.Vx[3, 3] = 1.0
    ocp.cost.Vx[4, 4] = 1.0

    ocp.cost.Vu = np.zeros((ny, nu))
    ocp.cost.Vu[5, 0] = 1.0
    ocp.cost.Vu[6, 1] = 1.0

    ocp.cost.Vx_e = np.eye(nx)

    ocp.cost.yref = np.zeros((ny,))
    ocp.cost.yref_0 = np.zeros((ny,))
    ocp.cost.yref_e = np.zeros((ny_e,))

    ocp.constraints.lbu = np.array([-params.max_tau_u, -params.max_tau_r])
    ocp.constraints.ubu = np.array([ params.max_tau_u,  params.max_tau_r])
    ocp.constraints.idxbu = np.array([0, 1])

    ocp.constraints.lbx = np.array([-1.0e9, -1.0e9, -1.0e9, 0.0, -params.max_r])
    ocp.constraints.ubx = np.array([ 1.0e9,  1.0e9,  1.0e9, params.max_u, params.max_r])
    ocp.constraints.idxbx = np.array([0, 1, 2, 3, 4])

    ocp.constraints.x0 = np.zeros((nx,))

    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'

    return ocp


def main():
    root = Path(__file__).resolve().parent
    export_dir = root / 'generated'
    export_dir.mkdir(parents=True, exist_ok=True)

    params = HeronAcadosParams()
    ocp = build_ocp(params, export_dir)
    AcadosOcpSolver(ocp, json_file=str(export_dir / 'heron_single_ocp.json'))
    print(f'Generated acados solver in: {export_dir}')


if __name__ == '__main__':
    main()
