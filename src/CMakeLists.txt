add_subdirectory(util)
add_subdirectory(planning_app)

add_executable(moveit_visualize_collision moveit_visualize_collision.cpp)
target_link_libraries(moveit_visualize_collision
                      collision_common
                      ${interactive_markers_LIBRARIES}
                      interact_utils
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES})

add_executable(test_ompl_planner test_ompl_planner.cpp)
target_link_libraries(test_ompl_planner
                      ompl_planner
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES})

add_executable(environment_publisher environment_publisher.cpp)
target_link_libraries(environment_publisher
                      ompl_planner
                      visual_utils
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES})

add_executable(robot_visualizer robot_visualizer.cpp)
target_link_libraries(robot_visualizer
                      visual_utils
                      parsers
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES})

add_executable(test_ik_solver test_ik_solver.cpp)
target_link_libraries(test_ik_solver
                      ompl_planner
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES})

add_executable(test_ik_solver_service test_ik_solver_service.cpp)
target_link_libraries(test_ik_solver_service
                      ompl_planner
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES})
