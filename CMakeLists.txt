cmake_minimum_required(VERSION 3.0.2)
project(target_detector)

############
## Config ##
############

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
	roscpp
	message_generation
	tf2
	tf2_ros
	geometry_msgs
	visualization_msgs
	ar_track_alvar_msgs
	dynamic_reconfigure
)

include_directories(
	include
	${catkin_INCLUDE_DIRS})

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(cfg/target_detector.cfg)

# Package messages and services
add_message_files(
	DIRECTORY msg
	FILES
		Detection.msg
		Detections.msg
)
add_service_files(
	DIRECTORY srv
	FILES Detector.srv
)
generate_messages(
	DEPENDENCIES
		geometry_msgs
)

catkin_package(CATKIN_DEPENDS
	roscpp
	message_generation
	tf2
	tf2_ros
	geometry_msgs
	visualization_msgs
	ar_track_alvar_msgs
	dynamic_reconfigure
)

###########
## Build ##
###########

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

## ROS nodes
add_executable(${PROJECT_NAME}_lidar_2d
	src/cluster.cpp
	src/detector_reflector.cpp
	src/detector_column.cpp
	src/node_base.cpp
	src/node_lidar_2d.cpp
	src/main_lidar_2d.cpp
)
add_dependencies(${PROJECT_NAME}_lidar_2d
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_lidar_2d
	${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_alvar
	src/node_base.cpp
	src/node_alvar.cpp
	src/main_alvar.cpp
)
add_dependencies(${PROJECT_NAME}_alvar
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_alvar
	${catkin_LIBRARIES}
)


## Tests
add_executable(test_cluster
	src/cluster.cpp
	src/tests/test_cluster.cpp
)

## Tests
add_executable(test_detector_reflector
	src/cluster.cpp
	src/detector_reflector.cpp
	src/tests/test_detector_reflector.cpp
)

add_executable(test_detector_column
	src/cluster.cpp
	src/detector_column.cpp
	src/tests/test_detector_column.cpp
)

#############
## Install ##
#############

install(
	TARGETS ${PROJECT_NAME}_lidar_2d ${PROJECT_NAME}_alvar
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
	TARGETS test_cluster test_detector_reflector test_detector_column
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch/
)

install(DIRECTORY config/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config/
)

install(DIRECTORY alvar_markers/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/alvar_markers/
)

install(DIRECTORY rviz/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz/
)
