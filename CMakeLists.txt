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
	tf2_eigen
	geometry_msgs
	reflector_finder
	ar_track_alvar_msgs
)

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
	tf2_eigen
	geometry_msgs
	reflector_finder
	ar_track_alvar_msgs
)

###########
## Build ##
###########

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

## ROS node
add_executable(target_detector
	src/target_detector.cpp
	src/target_detector_main.cpp
)
add_dependencies(target_detector
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)
target_link_libraries(target_detector
	${catkin_LIBRARIES}
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

install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch/
)

install(DIRECTORY config/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config/
)
