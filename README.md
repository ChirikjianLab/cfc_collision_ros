# CFC-collision.app
Motion planning application package for collision detection using closed-form contact space (CFC) parameterization.

## Dependencies
- [CFC-collision](https://github.com/ChirikjianLab/cfc-collision.git): core library for CFC-based collision detection
- ROS Noetic (Ubuntu 20.04)
- MoveIt

## Installation
- Install ROS Noetic and MoveIt using official instructions
- Generate catkin workspace and download the source code
```
mkdir -p ~/catkin_ws/src/
cd ~/catkin_ws/src/
git clone --recurse-submodules https://github.com/ChirikjianLab/cfc_collision_app.git
```
- Build the package
```
cd ../
catkin build
```

## Usage
### Visualize collision using interactivity marker
Show contact points as described in [MoveIt tutorial -- visualizing collisions](https://ros-planning.github.io/moveit_tutorials/doc/visualizing_collisions/visualizing_collisions_tutorial.html)
```
roslaunch cfc_collision_app moveit_visualize_collision.launch
```

### Planning using OMPL (not MoveIt plugin)
```
roslaunch cfc_collision_app ompl_planning.launch 
```
Note: 
- Please change launch file for different robots and environment types.
- For testing on real robot, use `ompl_planning_real.launch`

### Solve inverse kinematics
Given a set of SE(3) poses, solve inverse kinematics using MoveIt feature referred in [moveit::core::RobotState::setFromIK](http://docs.ros.org/en/indigo/api/moveit_core/html/classmoveit_1_1core_1_1RobotState.html#ab816880027ef7e63bbdef22a0497cc78)
```
roslaunch cfc_collision_app moveit_ik_solver.launch
```
Note:
- For testing on real robot, use `moveit_ik_solver_real.launch`

## Status
### Features
- Add CFC-based collision checker as a plugin for MoveIt: ==In progress==
- Visualizing collision via interactive markers: ==Done==
- Direct interface with OMPL using CFC-based collision checker: ==Done==

### Supported robots
- Panda arm (7 DOF)
- Panda dual-arm (14 DOF)
- Snake-like manipulator (15 DOF)

### Available environment types
- Sparse
- Cluttered
- Narrow
- Dense
- Real lab experiments

### Supported start/goal types
- Joint space (C-space): ==Done==
- Workspace: ==In progress==
