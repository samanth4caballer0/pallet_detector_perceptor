cmake_minimum_required(VERSION 3.0.2)
project(target_detector)

############
## Config ##
############

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
	roscpp
	tf
	geometry_msgs
	sensor_msgs
	visualization_msgs
	sick_safetyscanners
)

catkin_package(CATKIN_DEPENDS
	roscpp
	tf
	geometry_msgs
	sensor_msgs
	visualization_msgs
	sick_safetyscanners
)

###########
## Build ##
###########

include_directories(include
	${catkin_INCLUDE_DIRS}
)

##  Library
add_library(${PROJECT_NAME}
	src/marker_reflector/cluster.cpp
	src/marker_reflector/marker_reflector.cpp
)
#target_link_libraries(${PROJECT_NAME}
#	xxxxx
#)

## ROS node
add_executable(${PROJECT_NAME}_node
	src/target_detector.cpp
	src/main.cpp
)
add_dependencies(${PROJECT_NAME}_node
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
	${PROJECT_NAME}
)
target_link_libraries(${PROJECT_NAME}_node
	${catkin_LIBRARIES}
	${PROJECT_NAME}
)


#############
## Install ##
#############

install(
	TARGETS ${PROJECT_NAME}	${PROJECT_NAME}_node
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

install(DIRECTORY rviz/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz/
)
