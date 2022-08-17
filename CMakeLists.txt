cmake_minimum_required(VERSION 3.5)
project(cfc_collision_app VERSION 0.1.0 LANGUAGES CXX)

set(default_build_type "Release")
add_compile_options(-std=c++14)

# Packages
find_package(catkin REQUIRED COMPONENTS
    interactive_markers
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    pcl_ros
    eigen_conversions
    message_generation
    ompl
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(Ceres 2.0 REQUIRED)

add_service_files(
  FILES
  SolveIK.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
    CATKIN_DEPENDS
      moveit_core
      moveit_visual_tools
      moveit_ros_planning_interface
      interactive_markers
    DEPENDS
      EIGEN3
)

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(include cfc-collision/include)

link_directories(${catkin_LIBRARY_DIRS})

set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/data)
file(MAKE_DIRECTORY data/result/)

# Subdirectory
add_subdirectory(src)
