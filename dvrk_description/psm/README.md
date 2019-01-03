## PSM Launch Files

### Description and Usage
1. The "**psm.urdf.xacro**" file is the meat and bones of the PSM's urdf file
    * This file requires the "**prefix**" and the **initial pose** of the base link
2. The "**psm.launch.urdf.xacro**" file in the entry point of using the "**psm.urdf.xacro**"
    * This file sets the "**prefix**" and the **initial pose** of the base link
    * You can play around with this file to generate **PSM1**, **PSM2** and **PSM3**
3. To generate a **.urdf** file, run the following command on Linux command-line
   * `rosrun xacro xacro psm_launch.urdf.xacro > <file_to_save_to>`
   * As mentioned in the previous point, you play around with the **prefix** and **xyz** **rpy** to change
    the base pose and robot's prefix
4. The "**.sdf**" files are for gazebo and self explanatory
