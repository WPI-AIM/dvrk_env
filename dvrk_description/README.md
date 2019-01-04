# Description

This package provides the cad models, URDF, SDF for the Patient Side Manipulator (psm), Master Tool Manipulator (mtm), Endocopic Camera Manipulator (ecm), Set up joint cart (cart), and Laprotek Master for the daVinci Research Kit.

Moreover, the whole integrated dvrk setup for simulation using RViz and Gazebo is also provided here. The launch files for gazebo are in dvrk_gazebo package. The launch files for RViz are in the respective folder here.

# Note

For the integrated models, nesting of different models has been done. For this, the models need to be present in the .gazebo folder of your system for Gazebo to recognize the models that are nested.

We provide a simple script to take care of that: Just execute the **install** file as follows:

`./install.sh`

# Usage
## ROS (Running in only RViz)

All the launch files are located in the launch folder. Make sure that your **ros_ws** that contains this package is sourced. Remember, you can either source the package for the current terminal session using `source **ros_ws**/devel/setup.bash` or add it to your **~/.bashrc** file. Afterwards run:

`roslaunch dvrk_description <arm_type>_rviz.launch`

Where `<arm_type>` is one of the following `mtm, psm, ecm, suj, dvrk`.

