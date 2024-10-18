import crocoddyl
import pinocchio as pin
import numpy as np
import mim_solvers

from colmpc import ResidualDistanceCollision


class OCPPoseRef:
    def __init__(
        self,
        rmodel: pin.Model,
        cmodel: pin.GeometryModel = None,
        armature: np.ndarray = None,
        effector_frame_name: str = "panda_hand_tcp",
        des_pose: pin.SE3 = None,
        use_callbacks: bool = False,
    ) -> None:
        """Class to define the OCP linked witha HPP generated trajectory.

        Args:
            rmodel (pin.Model): Pinocchio model of the robot.
            cmodel (pin.GeometryModel): Pinocchio geometry model of the robot. Must have been convexified for the collisions to work.
            use_constraints : boolean to activate collision avoidance constraints.
            armature : armature of the robot.

        Raises:
            Exception: Unkown robot.
        """
        self.use_callbacks = use_callbacks
        self.use_constraints = True
        # Robot models
        self._rmodel = rmodel
        self._cmodel = cmodel

        # Data of the model
        self._rdata = self._rmodel.createData()

        self.des_pose = des_pose

        # Obtaining the gripper frame id
        self._effector_frame_id = self._rmodel.getFrameId(effector_frame_name)
        # Weights of the different costs
        # comb = [2.51188643150958, 0.00039810717055349735, 0.31622776601683794]
        self._weight_u_reg = 1e-3
        self._weight_ee_placement = 4
        self._weight_vel_reg = 2

        # Using the constraints ?
        # Safety marging for the collisions
        self._safety_margin = 1.5e-1

        # Creating the state and actuation models
        self.state = crocoddyl.StateMultibody(self._rmodel)
        self.actuation = crocoddyl.ActuationModelFull(self.state)

        # Setting up variables necessary for the OCP
        self.armature = armature
        self.DT = 1e-2  # Time step of the OCP
        self.nq = self._rmodel.nq  # Number of joints of the robot
        self.nv = self._rmodel.nv  # Dimension of the speed of the robot
        self.T = 30

        # Creating the running and terminal models
        self.running_models = None
        self.terminal_model = None

        # Solver used for the OCP
        self.solver = None

    def get_grav_compensation(
        self,
        q: np.ndarray,
    ) -> np.ndarray:
        """Return the reference of control u_plan that compensates gravity."""
        pin.computeGeneralizedGravity(
            self._rmodel,
            self._rdata,
            q,
        )
        return self._rdata.g.copy()

    def get_inverse_dynamic_control(self, x, a):
        """Return inverse dynamic control for a given state and acceleration."""
        return pin.rnea(self._rmodel, self._rdata, x[: self.nq], x[self.nq :], a).copy()

    def set_weights(
        self,
        weight_ee_placement: float,
        weight_x_reg: float,
        weight_u_reg: float,
        weight_vel_reg: float,
    ):
        """Set weights of the ocp.

        Args:
            weight_ee_placement (float): Weight of the placement of the end effector with regards to the target.
            weight_x_reg (float): Weight of the state regularization.
            weight_u_reg (float): Weight of the control regularization.
            weight_vel_reg (float): Weight of the velocity regularization.
        """
        self._weight_ee_placement = weight_ee_placement
        self._weight_x_reg = weight_x_reg
        self._weight_u_reg = weight_u_reg
        self._weight_vel_reg = weight_vel_reg

    def get_model(self, x_ref, u_ref, des_pose):

        running_cost_model = crocoddyl.CostModelSum(self.state)
        u_reg_cost = self.get_control_residual(u_ref)
        ugrav_reg_cost = self.get_control_grav_residual()
        x_reg_cost = self.get_state_residual(x_ref)
        vel_reg_cost = self.get_velocity_residual()
        placement_reg_cost = self.get_placement_residual(des_pose)
        running_cost_model.addCost("uReg", u_reg_cost, self._weight_u_reg)
        # running_cost_model.addCost("ugravReg", ugrav_reg_cost, self._weight_u_reg)
        running_cost_model.addCost(
            "gripperPose", placement_reg_cost, self._weight_ee_placement
        )
        running_cost_model.addCost("velReg", vel_reg_cost, self._weight_vel_reg)
        running_cost_model.addCost("xReg", x_reg_cost, 0)

        constraints = self.get_constraints()
        running_DAM = crocoddyl.DifferentialActionModelFreeFwdDynamics(
            self.state, self.actuation, running_cost_model, constraints
        )
        running_DAM.armature = self.armature
        return crocoddyl.IntegratedActionModelEuler(running_DAM, self.DT)

    def get_constraints(self):
        constraint_model_manager = crocoddyl.ConstraintModelManager(self.state, self.nq)
        if len(self._cmodel.collisionPairs) != 0:
            for col_idx in range(len(self._cmodel.collisionPairs)):
                collision_constraint = self._get_collision_constraint(
                    col_idx, self._safety_margin
                )
                # Adding the constraint to the constraint manager
                constraint_model_manager.addConstraint(
                    "col_term_" + str(col_idx), collision_constraint
                )
        return constraint_model_manager

    def _get_collision_constraint(
        self, col_idx: int, safety_margin: float
    ) -> "crocoddyl.ConstraintModelResidual":
        """Returns the collision constraint that will be in the constraint model manager.

        Args:
            col_idx (int): index of the collision pair.
            safety_margin (float): Lower bound of the constraint, ie the safety margin.

        Returns:
            _type_: _description_
        """
        obstacleDistanceResidual = ResidualDistanceCollision(
            self.state, 7, self._cmodel, col_idx
        )

        # Creating the inequality constraint
        constraint = crocoddyl.ConstraintModelResidual(
            self.state,
            obstacleDistanceResidual,
            np.array([safety_margin]),
            np.array([np.inf]),
        )
        return constraint

    def get_placement_residual(self, placement_ref):
        """Return placement residual with desired reference for end effector placement."""
        return crocoddyl.CostModelResidual(
            self.state,
            crocoddyl.ResidualModelFramePlacement(
                self.state, self._effector_frame_id, placement_ref
            ),
        )

    def get_velocity_residual(self):
        """Return velocity residual of desired joint."""
        vref = pin.Motion.Zero()
        return crocoddyl.CostModelResidual(
            self.state,
            crocoddyl.ResidualModelFrameVelocity(
                self.state,
                self._effector_frame_id,
                vref,
                pin.WORLD,
            ),
        )

    def get_control_residual(self, uref):
        """Return control residual with uref the control reference."""
        return crocoddyl.CostModelResidual(
            self.state, crocoddyl.ResidualModelControl(self.state, uref)
        )

    def get_control_grav_residual(self):
        """Return control residual with uref the control reference."""
        return crocoddyl.CostModelResidual(
            self.state, crocoddyl.ResidualModelControlGrav(self.state)
        )

    def get_state_residual(self, xref):
        """Return state residual with xref the state reference."""
        return crocoddyl.CostModelResidual(
            self.state,  # x_reg_weights,
            crocoddyl.ResidualModelState(self.state, xref, self.actuation.nu),
        )

    def update_cost(self, model, new_model, cost_name, update_weight=True):
        """Update model's cost reference and weight by copying new_model's cost."""
        model.differential.costs.costs[cost_name].cost.residual.reference = (
            new_model.differential.costs.costs[cost_name].cost.residual.reference.copy()
        )
        if update_weight:
            new_weight = new_model.differential.costs.costs[cost_name].weight
            model.differential.costs.costs[cost_name].weight = new_weight

    def update_model(self, model, new_model, update_weight):
        """update model's costs by copying new_model's costs."""
        self.update_cost(model, new_model, "gripperPose", update_weight)
        self.update_cost(model, new_model, "uReg", update_weight)

    def reset_ocp(self, x, x_ref: np.ndarray, u_plan: np.ndarray, placement_ref):
        """Reset ocp problem using next reference in state and control."""
        self.solver.problem.x0 = x
        last_u_plan = self.solver.us
        u_grav = self.get_grav_compensation(x[: self.nq])
        model = self.get_model(x, u_grav, self.des_pose)
        runningModels = list(self.solver.problem.runningModels)
        for node_idx in range(len(runningModels)):

            self.update_model(runningModels[node_idx], model, True)
        self.update_model(self.solver.problem.terminalModel, model, True)

    def build_ocp_from_plannif(self, x_plan, a_plan, x0):
        u_grav = self.get_grav_compensation(x0[: self.nq])
        model = self.get_model(x0, u_grav, self.des_pose)
        # print("x0 ", x0)
        # print("model ", model)
        self.u_plan = np.array(list(u_grav) * (self.T - 1))
        self.u_plan = np.reshape(self.u_plan, (self.T - 1, 7))
        return crocoddyl.ShootingProblem(x0, [model] * (self.T - 1), model)

    def run_solver(self, problem, xs_init, us_init, max_iter, max_qp_iter):
        """
        Run FDDP or CSQP solver
        problem : crocoddyl ocp problem.
        xs_init : xs warm start.
        us_init : us warm start.
        max_iter : max number of iteration for the solver
        set_callback : activate solver callback
        """
        # Creating the solver for this OC problem, defining a logger
        solver = mim_solvers.SolverCSQP(problem)
        solver.use_filter_line_search = True
        solver.termination_tolerance = 1e-4
        solver.max_qp_iters = max_qp_iter
        if self.use_callbacks:
            solver.with_callbacks = True
        solver.solve(xs_init, us_init, max_iter)
        self.solver = solver
