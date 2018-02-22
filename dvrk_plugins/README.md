# Description

This package contains code for the model plugin written specifically for the daVinci Research Kit simulated in Gazebo. This provides an interface for using the simulated Gazebo models to be controlled using ROS messages.

The package spawns ROS topics for each active joints in the model for setting position kinematically, setting a target position to use the position controller and to set the effort for the joints. Furthermore another topic /joints/states provides state of position, velocity and effort for each active joints.  

# How to use

Setting Position: Publish to the "/SetPosition" topic to set the position kinematically. This can be done only for the joints that are not involved in a closed loop chain.

Setting Target Position: Publish to the "/SetPostionTarget" topic to set the target position for the position controller. The position controller uses the P, I, D values from the ROS parameter servers. If no such values are provided, it gives an error and cannot be used for the particular joint.

Setting Effort: Publish to the "/SetEffort" topic to set the desired effort for the particular joint.

Reading states:  Subscribe to the "/joint/states" topic to read the position, velocity, effort applied on all active joints.  
