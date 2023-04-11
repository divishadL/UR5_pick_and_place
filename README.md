# UR5_Octomap
This repository demonstrates a simple pick and place task executed by a UR5 robot arm with 2 finger gripper from Robotiq. All the dependent packages needed to run this demo are included in the workspace.

This example is compatible and tested with ROS Noetic and Ubuntu 20.04.

##Steps to launch the example:

- At first launch the UR5 arm in Gazebo. The referred package is [UR5_Gazebo](https://github.com/utecrobotics/ur5). The referred repository is built for older versions of ROS and will give some build errors for Noetic, so cloning the package [UR5](https://github.com/divishadL/UR5_pick_and_place/tree/main/src/ur5) in current repo will work fine for Noetic systems.

`roslaunch ur5_gazebo ur5_cubes.launch`
