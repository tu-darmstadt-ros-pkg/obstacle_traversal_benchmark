cmake_minimum_required(VERSION 3.0.2)
project(obstacle_traversal_benchmark)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  hector_pose_prediction_interface
  hector_stability_metrics
  nav_msgs
  rosbag
  sensor_msgs
  tf2_msgs
  sdf_contact_estimation
  moveit_core
  moveit_ros_planning
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES obstacle_traversal_benchmark
  CATKIN_DEPENDS eigen_conversions geometry_msgs hector_pose_prediction_interface hector_stability_metrics nav_msgs rosbag sensor_msgs tf2_msgs sdf_contact_estimation moveit_core moveit_ros_planning
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(HEADERS
  include/${PROJECT_NAME}/bag_reader.h
  include/${PROJECT_NAME}/checkpoint.h
  include/${PROJECT_NAME}/imu_datapoint.h
  include/${PROJECT_NAME}/obstacle_traversal_benchmark.h
  include/${PROJECT_NAME}/stability_datapoint.h
  include/${PROJECT_NAME}/trial.h
  include/obstacle_traversal_benchmark/types.h
)

set(SOURCES
  src/bag_reader.cpp
  src/obstacle_traversal_benchmark.cpp
  src/trial.cpp
  include/obstacle_traversal_benchmark/util.cpp
  include/obstacle_traversal_benchmark/util.h
)

add_library(${PROJECT_NAME} ${HEADERS} ${SOURCES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/obstacle_traversal_benchmark_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
