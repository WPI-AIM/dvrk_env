Notification
====================

All the models have been designed at WPI from scratch.

Description
====================
This repository contains RViz and Gazebo simulations of the da Vinci Surgical System controlled using ROS. 

# Authors

Ankur Agrawal:asagrawal@wpi.edu
Radian Azhar Gondokaryono:ragondokaryono@wpi.edu

# Maintainer

Adnan Munawar: amunawar@wpi.edu

# Install
* download & compile dvrk_env

```sh

# cd to catkin ws src dir
cd /PATH/TO/CATKIN_WS/src

# clone repo
git clone https://github.com/WPI-AIM/dvrk_env.git

# copy models to .gazebo/models
cd dvrk_env/dvrk_description
./install.sh 

# build
cd /PATH/TO/CATKIN_WS
catkin_make
```
# Packages

This is a short description of the packages in this repository. The detailed explanations and instructions are available in the packages itself.

dvrk_gazebo: Launch files to launch PSM, MTM, ECM, SUJ Cart and the full dvrk models in gazebo. Src includes model plugin which provides interface between the dvrk Gazebo simulation and ROS.

dvrk_gazebo_control: Example codes to control the gazebo simulation.

dvrk_description: CAD models, URDFs, SDFs of PSM, ECM, MTM, SUJ, and dVRk. Additionally it has launch files for all of the models in RViz. 

# Launching 
```sh
# Run the Surgical System simulation
roslaunch dvrk_gazebo dvrk_gazebo.launch
```
# Dependencies

Gazebo 7, ROS kinetic or ROS indigo, gazebo_ros_pkgs. If ROS-indigo is to be used with Gazebo 7, keep gazebo_ros_pkgs (https://github.com/ros-simulation/gazebo_ros_pkgs/tree/indigo-devel) in your src folder.
