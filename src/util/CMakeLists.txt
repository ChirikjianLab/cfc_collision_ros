# Source files
add_library(geometry_utils MultiBodyTree3D.cpp)

add_library(parsers ParsePlanningSettings.cpp
                    ParseURDF.cpp
                    Parse2dCsvFile.cpp)
target_link_libraries(parsers
                      geometry_utils
                      ${catkin_LIBRARIES})

add_library(interact_utils interactive_robot.cpp
                           imarker.cpp
                           pose_string.cpp)
target_link_libraries(interact_utils
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES})

add_library(visual_utils VisualizationUtils.cpp)
target_link_libraries(visual_utils ${catkin_LIBRARIES})
