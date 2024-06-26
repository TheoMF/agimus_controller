#!/usr/bin/env python
from math import pi
from agimus_controller.ocps.ocp_croco_hpp import OCPCrocoHPP
from agimus_controller.mpc import MPC
from agimus_controller.hpp_interface import HppInterface
from agimus_controller.utils.plots import MPCPlots
import time


if __name__ == "__main__":
    hpp_interface = HppInterface()
    q_init = [pi / 6, -pi / 2, pi / 2, 0, 0, 0, -0.2, 0, 0.02, 0, 0, 0, 1]
    hpp_interface.set_ur3_problem_solver(q_init)
    ball_init_pose = [-0.2, 0, 0.02, 0, 0, 0, 1]
    hpp_path = hpp_interface.ps.client.basic.problem.getPath(
        hpp_interface.ps.numberPaths() - 1
    )
    x_plan, a_plan, whole_traj_T = hpp_interface.get_hpp_plan(1e-2, 6, hpp_path)
    ocp = OCPCrocoHPP("ur3")
    chc = MPC(ocp, x_plan, a_plan, ocp.robot.model)
    start = time.time()
    chc.ocp.set_weights(10**4, 1, 10**-3, 0)
    chc.simulate_mpc(100, True)
    end = time.time()
    u_plan = chc.ocp.get_uref(x_plan, a_plan)
    mpc_plots = MPCPlots(
        chc.croco_xs,
        chc.croco_us,
        x_plan,
        u_plan,
        ocp.robot.model,
        chc.ocp.DT,
        "wrist_3_joint",
        hpp_interface.vf,
        ball_init_pose,
    )
    print("mpc duration ", end - start)
