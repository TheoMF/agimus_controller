# agimus_controller

## Dependencies

In order to build this package one need various complexe codes:

- Humanoid Path Planner
- Agimus software
- Croccodyl

## Installation

All of these dependencies are built in a single docker:
gitlab.laas.fr:4567/agimus-project/agimus_dev_container:noetic-devel

One can simply use this package in order to use the docker in the VSCode
development editor.
https://gitlab.laas.fr/agimus-project/agimus_dev_container

## Usage

We use the Humanoid Path Planner in order to get a path and then build a whole-body model predictive controller that tracks the planned trajectory.

### Simulation

#### Without ROS

you can launch one of these mains :
    - ur3 scripts :
        - `python3 -m agimus_controller.main.ur3.main_hpp_mpc -N=1`
    - panda scripts :
        -  `python3 -m agimus_controller.main.panda.main_hpp_mpc_buffer`
        -  `python3 -im agimus_controller.main.panda.main_hpp_mpc`
        -  `python3 -m agimus_controller.main.panda.main_meshcat_display`
        -  `python3 -m agimus_controller.main.panda.main_optim_traj`
        -  `python3 -m agimus_controller.main.panda.main_scenes`


#### Using ROS

For one to simply run the node individually.

```bash
rosrun agimus_controller agimus_controller_node
```

```bash

roslaunch agimus_controller hpp_agimus_controller.launch
```

## Experiment
For a more complete setup see the
https://github.com/agimus-project/agimus_pick_and_place
package.

To run the simulation with ros one can launch:
```bash
roslaunch panda_torque_mpc simulation.launch arm_id:=panda simulate_camera:=false headless:=true
roslaunch panda_torque_mpc sim_controllers.launch controller:=ctrl_mpc_linearized
roslaunch agimus_controller agimus_controller.launch
roslaunch agimus-hpp start_hpp.launch
cd ~/ros_ws/src/agimus-demos/franka/manipulation/
python -i script_hpp.py
q_init, p = GrabAndDrop(robot, ps, binPicking, <your pose acquisition method>)
q_init, p = GrabAndDrop(robot, ps, binPicking, q_init, ros_bridge_config,vision_listener)
rostopic pub /hpp/target/read_path std_msgs/UInt32 0
rostopic pub /hpp/target/publish std_msgs/Empty
```
export ROS_MASTER_URI=http://172.17.1.1:11311 ROS_IP=172.17.1.1
Start on the robot:
```bash
ROS_MASTER_URI=http://172.17.1.1:11311 ROS_IP=172.17.1.1 roslaunch panda_torque_mpc real_controllers.launch controller:=ctrl_mpc_linearized robot_ip:=172.17.1.3 robot:=panda
ROS_MASTER_URI=http://172.17.1.1:11311 ROS_IP=172.17.1.1 roslaunch agimus_controller hpp_agimus_controller.launch
```

rosservice call /hpp/target/set_joint_names "names: 
- 'pandas/panda_joint1'
- 'pandas/panda_joint2'
- 'pandas/panda_joint3'
- 'pandas/panda_joint4'
- 'pandas/panda_joint5'
- 'pandas/panda_joint6'
- 'pandas/panda_joint7'"