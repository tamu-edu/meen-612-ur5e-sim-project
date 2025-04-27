# ur3e model

The UR3e models were originally imported from <https://github.com/ros-industrial/universal_robot/tree/melodic-devel/ur_description>,
exact commit unknown but probably near
<https://github.com/ros-industrial/universal_robot/commit/c8c27c15579fad1d817cc6cfac7a8e62a3da081d>.

High-level changes:
- Converted the mesh format from DAE to OBJ to glTF.
- Added two new sets of collision geometries (which can be selected via xacro):
  - Cylinders + spheres 
  - Spheres only (which works better w/ some collision checking frameworks)

This folder was directly copies from https://github.com/RobotLocomotion/models/tree/master

the ur5e.urdf was generated from this repo https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/humble

I used this commaned in the urdf directory of the above repo
 `xacro -o ur5e.urdf ur.urdf.xacro name:=ur5e ur_type:=ur5e`

 and edited the config yaml files to generate the .obj meshes
 the .dae and .stl meshes were converted to .obj using meshlab
