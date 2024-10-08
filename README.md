# agimus_controller

## Getting started

In order to build this package one need various complexe codes:

- Humanoid Path Planner
- Agimus software
- Croccodyl

All of these are built in a single docker:
gitlab.laas.fr:4567/agimus-project/agimus_dev_container:noetic-devel

One can simply use this package in order to use the docker in the VSCode
development editor.
https://gitlab.laas.fr/agimus-project/agimus_dev_container

## Run one application.

### Pick an place scenario using HPP

We use the Humanoid Path Planner in order to get a path and then build a whole-body model predictive controller that tracks the planned trajectory.

Once the code is built, you can run the mpc either with or without ros, without ros, one can run these in several terminals inside the docker :
- `hppcorbaserver`
- `gepetto-gui`
- `roscore`
then you can you choose one of the mains :
    - ur3 scripts :
        - `python3 -m agimus_controller.main.ur3.main_hpp_mpc -N=1`
    - panda scripts :
        -  `python3 -m agimus_controller.main.panda.main_hpp_mpc_buffer -N=1`
        -  `python3 -m agimus_controller.main.panda.main_hpp_mpc -N=1`
        -  `python3 -m agimus_controller.main.panda.main_meshcat_display -N=1`
        -  `python3 -m agimus_controller.main.panda.main_optim_traj -N=1`
        -  `python3 -m agimus_controller.main.panda.main_scenes -N=1`


### Using ROS

If build with ROS one can run nodes that are tracking a planned trajectory from HPP:

For one to simply run the node individually.

```bash
rosrun agimus_controller agimus_controller_node
```

```bash

roslaunch agimus_controller hpp_agimus_controller.launch
```

For a more complete setup see the
https://github.com/agimus-project/agimus_pick_and_place
package.

To run the simulation with ros one can launch:
```bash
roslaunch panda_torque_mpc simulation.launch arm_id:=panda simulate_camera:=false
roslaunch panda_torque_mpc sim_controllers.launch controller:=ctrl_mpc_linearized
roslaunch agimus_controller hpp_agimus_controller.launch
```

Start on the robot:
```bash
ROS_MASTER_URI=http://172.17.1.1:11311 ROS_IP=172.17.1.1 roslaunch panda_torque_mpc real_controllers.launch controller:=ctrl_mpc_linearized robot_ip:=172.17.1.3 robot:=panda
ROS_MASTER_URI=http://172.17.1.1:11311 ROS_IP=172.17.1.1 roslaunch agimus_controller hpp_agimus_controller.launch
```
