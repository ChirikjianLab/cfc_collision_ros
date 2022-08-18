# cfc_collision_ros
[![ROS Noetic CI](https://github.com/ChirikjianLab/cfc_collision_ros/actions/workflows/ros_noetic_ci.yml/badge.svg)](https://github.com/ChirikjianLab/cfc_collision_ros/actions/workflows/ros_noetic_ci.yml)

ROS package for CFC-based collision detection framework, with applications on motion planning.

## Introduction
This is the ROS package for the collision detection algorithms using closed-form contact space (CFC) parameterization. It includes visualizations and applications on robot motion planning. Direct interface with OMPL is implemented.

- Paper: [IEEE Robotics and Automation Letters (RA-L)](https://ieeexplore.ieee.org/document/9829274)
- Project page: [https://chirikjianlab.github.io/cfc-collision/](https://chirikjianlab.github.io/cfc-collision/)
- C++ core library: [https://github.com/ChirikjianLab/cfc-collision/](https://github.com/ChirikjianLab/cfc-collision/)

## Authors
[Sipu Ruan](https://ruansp.github.io), Xiaoli Wang and [Gregory S. Chirikjian](https://scholar.google.com/citations?user=qoIuyMoAAAAJ&hl=en)

## Dependencies
- [`CFC-collision`](https://github.com/ChirikjianLab/cfc-collision): Core C++ library for CFC-based collision detection, fetch as a submodule (see installation instructions).
- [`ROS Noetic (Ubuntu 20.04)`](https://www.ros.org/)
- [`Ceres solver (>= 2.0)`](http://ceres-solver.org/installation.html)

### Required ROS packages
- [`MoveIt 1.0`](https://moveit.ros.org/): Binary installation is recommended.
- [`panda_moveit_config`](https://github.com/ros-planning/panda_moveit_config)
- `ompl`
- `moveit_visual_tools`
- `eigen_conversions`

Installation command
```sh
[sudo] apt install ros-noetic-moveit ros-noetic-panda-moveit-config ros-noetic-ompl ros-noetic-moveit-visual-tools ros-noetic-eigen-conversions
```

## Installation
- Install required dependencies
- Generate catkin workspace and download the source code
```sh
mkdir -p ~/catkin_ws/src/
cd ~/catkin_ws/src/
git clone --recurse-submodules https://github.com/ChirikjianLab/cfc_collision_ros.git
```
- Build the package
```sh
cd ~/catkin_ws/
catkin build
```

## Usage
### Visualize collision using interactivity marker
Show contact points as described in [MoveIt tutorial -- visualizing collisions](https://ros-planning.github.io/moveit_tutorials/doc/visualizing_collisions/visualizing_collisions_tutorial.html). 
```sh
roslaunch cfc_collision_ros moveit_visualize_collision.launch
```
**Additional feature**: when the object and robot are close to each other, a line segment connecting the closest witness points are shown.

### Planning using OMPL
```sh
roslaunch cfc_collision_ros ompl_planning.launch
```
**Note**
- Please change the parameters for different robots ([here](/launch/ompl_planning.launch#L4)) and environment types ([here](/launch/ompl_planning.launch#L5)). Can also specify as
```sh
roslaunch cfc_collision_ros ompl_planning.launch robot_name:=panda_arm env_type:=sparse
```
- For real Panda robot, use [`ompl_planning_real.launch`](/launch/ompl_planning_real.launch)
- For end points in workspace (only supports `panda_arm` robot model), use [`ompl_planning_ee.launch`](/launch/ompl_planning_ee.launch). Before planning, please run IK solver launch file first (instructed below).

**Reproduction of figures in paper**

To reproduce figures in the paper (Figure 6), please use the following arguments when launching `ompl_planning.launch`:
| Figure | Robot name | Environment type | Argument |
|--------|------------|------------------|----------|
| Fig. 6(a) | Snake | Narrow | `robot_name:=snake env_type:=narrow` |
| Fig. 6(b) | Panda single arm | Dense | `robot_name:=panda_arm env_type:=dense` |
| Fig. 6(c) | Panda dual arm | Sparse | `robot_name:=panda_dual_arm env_type:=sparse` |

### Solve inverse kinematics
Given a set of SE(3) poses, solve inverse kinematics using the MoveIt feature ([moveit::core::RobotState::setFromIK](http://docs.ros.org/en/indigo/api/moveit_core/html/classmoveit_1_1core_1_1RobotState.html#ab816880027ef7e63bbdef22a0497cc78))
```
roslaunch cfc_collision_ros moveit_ik_solver.launch
```
**Note**
- For testing on real scenes, use [`moveit_ik_solver_real.launch`](/launch/moveit_ik_solver_real.launch)

## Status
### Features
- Visualize collision and distance via interactive markers
- Direct interface with OMPL using CFC-based collision checker

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
- Joint space (C-space): For all robots
- Workspace (end effector poses): Only for "panda_arm"

## Reference
If you find our work useful in your research, please consider citing:

- S. Ruan, X. Wang and G. S. Chirikjian, "Collision Detection for Unions of Convex Bodies With Smooth Boundaries Using Closed-Form Contact Space Parameterization," in IEEE Robotics and Automation Letters, vol. 7, no. 4, pp. 9485-9492, Oct. 2022, doi: 10.1109/LRA.2022.3190629.

- BibTeX
```tex
@ARTICLE{ruan2022collision,
  author={Ruan, Sipu and Wang, Xiaoli and Chirikjian, Gregory S.},
  journal={IEEE Robotics and Automation Letters}, 
  title={Collision Detection for Unions of Convex Bodies With Smooth Boundaries Using Closed-Form Contact Space Parameterization}, 
  year={2022},
  volume={7},
  number={4},
  pages={9485-9492},
  doi={10.1109/LRA.2022.3190629}}
```
