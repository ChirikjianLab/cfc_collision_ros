add_library(collision_common
            collision_common.cpp
            collision_helper.cpp)
target_link_libraries(collision_common
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${CERES_LIBRARIES})

add_library(ompl_planner ompl_planner.cpp)
target_link_libraries(ompl_planner
                      collision_common
                      geometry_utils
                      parsers
                      ${OMPL_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${MOVEIT_LIB_NAME})
