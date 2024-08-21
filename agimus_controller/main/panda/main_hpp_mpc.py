import time
import numpy as np
from agimus_controller.hpp_interface import HppInterface
from agimus_controller.mpc import MPC
from agimus_controller.visualization.plots import MPCPlots
from agimus_controller.robot_model.panda_model import PandaRobotModel
from agimus_controller.ocps.ocp_croco_hpp import OCPCrocoHPP


def main():
    panda_model = PandaRobotModel.load_model()
    rmodel = panda_model.get_reduced_robot_model()
    cmodel = panda_model.get_reduced_collision_model()
    ee_frame_name = panda_model._params.ee_frame_name

    hpp_interface = HppInterface()
    q_init, q_goal = hpp_interface.get_panda_q_init_q_goal()
    hpp_interface.set_panda_planning(q_init, q_goal, use_gepetto_gui=True)
    ps = hpp_interface.get_problem_solver()
    viewer = hpp_interface.get_viewer()
    x_plan, a_plan, _ = hpp_interface.get_hpp_x_a_planning(
        1e-2, 7, ps.client.problem.getPath(ps.numberPaths() - 1)
    )

    armature = np.zeros(rmodel.nq)
    ocp = OCPCrocoHPP(rmodel, cmodel, use_constraints=False, armature=armature)
    mpc = MPC(ocp, x_plan, a_plan, rmodel, cmodel)

    start = time.time()
    mpc.ocp.set_weights(10**4, 1, 10**-3, 0)
    mpc.simulate_mpc(T=100, save_predictions=False)
    end = time.time()
    print("Time of solving: ", end - start)
    u_plan = mpc.ocp.get_u_plan(x_plan, a_plan)
    MPCPlots(
        croco_xs=mpc.croco_xs,
        croco_us=mpc.croco_us,
        whole_x_plan=x_plan,
        whole_u_plan=u_plan,
        rmodel=rmodel,
        DT=mpc.ocp.DT,
        ee_frame_name=ee_frame_name,
        viewer=viewer,
    )
    return True


if __name__ == "__main__":
    main()
