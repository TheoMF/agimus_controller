import time
import numpy as np
from agimus_controller.hpp_interface import HppInterface
from agimus_controller.mpc import MPC
from agimus_controller.utils.path_finder import get_package_path
from agimus_controller.visualization.plots import MPCPlots
from agimus_controller.ocps.ocp_pose_ref import OCPPoseRef
from agimus_controller.robot_model.panda_model import (
    PandaRobotModel,
    PandaRobotModelParameters,
)
from agimus_controller.utils.pin_utils import get_ee_pose_from_configuration
from agimus_controller.main.servers import Servers
from agimus_controller.mpc_search import MPCSearch


class APP(object):
    def main(self, use_gui=False, spawn_servers=False):
        if spawn_servers:
            self.servers = Servers()
            self.servers.spawn_servers(use_gui)

        panda_params = PandaRobotModelParameters()
        panda_params.collision_as_capsule = True
        panda_params.self_collision = False
        agimus_demos_description_dir = get_package_path("agimus_demos_description")
        collision_file_path = (
            agimus_demos_description_dir / "pick_and_place" / "obstacle_params.yaml"
        )
        pandawrapper = PandaRobotModel.load_model(
            params=panda_params, env=collision_file_path
        )

        rmodel = pandawrapper.get_reduced_robot_model()
        cmodel = pandawrapper.get_reduced_collision_model()
        ee_frame_name = panda_params.ee_frame_name

        hpp_interface = HppInterface()
        q_init, q_goal = hpp_interface.get_panda_q_init_q_goal()
        hpp_interface.set_panda_planning(q_init, q_goal, use_gepetto_gui=use_gui)
        viewer = hpp_interface.get_viewer()
        # x_plan, a_plan, _ = hpp_interface.get_hpp_x_a_planning(1e-2)
        x0 = q_init + [0] * 7
        length = 49
        x_plan = np.array(x0 * length)
        x_plan = np.reshape(x_plan, (length, 14))
        print("xplan shape", x_plan.shape)
        a_plan = np.zeros((length, 7))
        armature = np.zeros(rmodel.nq)
        effector_frame_name = "panda_hand_tcp"
        effector_frame_id = rmodel.getFrameId(effector_frame_name)
        rdata = rmodel.createData()
        des_pose = get_ee_pose_from_configuration(
            rmodel,
            rdata,
            effector_frame_id,
            np.array(q_goal),
        )
        ocp = OCPPoseRef(
            rmodel,
            cmodel,
            armature=armature,
            effector_frame_name=effector_frame_name,
            des_pose=des_pose,
            use_callbacks=False,
        )

        self.mpc = MPC(ocp, x_plan, a_plan, rmodel, cmodel)
        # self.mpc_search = MPCSearch(
        #    mpc=self.mpc, rmodel=rmodel, ee_frame_name=effector_frame_name
        # )
        # self.mpc_search.search_best_costs(
        #    use_constraints=True, configuration_traj=False
        # )
        start = time.time()
        self.mpc.simulate_mpc(save_predictions=True)
        max_kkt = max(self.mpc.mpc_data["kkt_norm"])
        index = self.mpc.mpc_data["kkt_norm"].index(max_kkt)
        print(f"max kkt {max_kkt} index {index}")
        end = time.time()
        print("Time of solving: ", end - start)
        self.mpc_plots = MPCPlots(
            croco_xs=self.mpc.croco_xs,
            croco_us=self.mpc.croco_us,
            whole_x_plan=x_plan,
            whole_u_plan=np.zeros((399, 7)),
            rmodel=rmodel,
            vmodel=pandawrapper.get_reduced_visual_model(),
            cmodel=cmodel,
            DT=self.mpc.ocp.DT,
            ee_frame_name=ee_frame_name,
            viewer=viewer,
        )

        if use_gui:
            self.mpc_plots.display_path_gepetto_gui()
        return True


def main():
    return APP().main(use_gui=False, spawn_servers=False)


if __name__ == "__main__":

    app = APP()
    app.main(use_gui=True, spawn_servers=True)
