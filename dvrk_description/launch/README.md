## Launch Files

### Description
These launch files launch the relevant **<robot>_launch.urdf.xacro** files in the `../<robot>/` folders. These
launch files are purposefully launching **MTM**'s and **PSM**'s without any suffix. **SUJ** and **ECM** have no suffix since there is onyl 
one of each. You can easily change the suffix for any robot in either the 
**<robot>_launch.urdf.xacro** file by setting the **prefix** string or in the launch file itself
