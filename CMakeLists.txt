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

## ROS node
add_executable(${PROJECT_NAME}
	src/cluster.cpp
	src/detector_reflector.cpp
	src/detector_column.cpp
	src/node.cpp
	src/main.cpp
)
add_dependencies(${PROJECT_NAME}
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
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
	TARGETS target_detector
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
