## MTM Launch Files

### Description and Usage
1. The "**mtm.urdf.xacro**" file is the meat and bones of the MTM's urdf file
    * This file requires the "**prefix**" and the **initial pose** of the base link
2. The "**mtm.launch.urdf.xacro**" file in the entry point of using the "**mtm.urdf.xacro**"
    * This file sets the "**prefix**" and the **initial pose** of the base link
    * You can play around with this file to generate **MTMR** and **MTML**
3. To generate a **.urdf** file, run the following command on Linux command-line
   * `rosrun xacro xacro mtm_launch.urdf.xacro > <file_to_save_to>`
   * As mentioned in the previous point, you play around with the **prefix** and **xyz** **rpy** to change
    the base pose and robot's prefix
4. The "**.sdf**" files are for gazebo and self explanatory
