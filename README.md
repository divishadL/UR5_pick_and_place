# UR5_Octomap
This repository demonstrates a simple pick and place task executed by a UR5 robot arm with 2 finger gripper from Robotiq. All the dependent packages needed to run this demo are included in the workspace.

This example is compatible and tested with **ROS Noetic** and Ubuntu 20.04.

## Steps to launch the example:

- At first launch the UR5 arm in Gazebo. The referred package is [UR5_Gazebo](https://github.com/utecrobotics/ur5). The referred repository is built for older versions of ROS and will give some build errors for Noetic, so cloning the package [UR5](https://github.com/divishadL/UR5_pick_and_place/tree/main/src/ur5) in current repo will work fine for Noetic systems. Moreover, an Intel Realsense camera is added in the model, to make use of the Point clouds to generate an octomap with moveit.

`roslaunch ur5_gazebo ur5_cubes.launch`
> Here remember to unpause the simulation in Gazebo. It is paused by default.

- Secondly, launch the move_group node and Rviz visualisation in order to visualise the octomap and the planning scene.

`roslaunch ur5_moveit_config execution.launch`

- Finallz, run the following node, which adds the concerned box in the planning scene and allows the collision between gripper and box and later attaches the box to gripper.

> Here even after removing the collision object, it is still remaining attached with the gripper(Issue to be resolved)

`rosrun ur5_moveit_config octomap_pp`

