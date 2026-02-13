cmake_minimum_required(VERSION 3.0.2)
project(target_detector)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package( catkin REQUIRED COMPONENTS
	roscpp
	message_generation
	geometry_msgs
	sensor_msgs
	tf2
	tf2_ros
	visualization_msgs
	std_msgs
	ar_track_alvar_msgs
	sick_safetyscanners
	tmk_uwb
)

add_message_files(
	DIRECTORY msg
	FILES
		Detection.msg
		Detections.msg
)
add_service_files(
	DIRECTORY srv
	FILES
		DetectorEnable.srv
)
generate_messages(
	DEPENDENCIES
		geometry_msgs
)

catkin_package(
	INCLUDE_DIRS
		include
	CATKIN_DEPENDS
		roscpp
		message_generation
		geometry_msgs
		sensor_msgs
		tf2
		tf2_ros
		visualization_msgs
		std_msgs
		ar_track_alvar_msgs
		sick_safetyscanners
		tmk_uwb
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_library(detectors
	src/detectors/reflector_detector.cpp
	src/detectors/cluster.cpp
)
add_dependencies(detectors
	${catkin_EXPORTED_TARGETS}
	${target_detector_EXPORTED_TARGETS}
)
target_link_libraries(detectors
	${catkin_LIBRARIES}
)

add_executable(reflector_perceptor
	src/reflector_perceptor_main.cpp
	src/reflector_perceptor.cpp
)
add_dependencies(reflector_perceptor
	detectors
	${target_detector_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)
target_link_libraries(reflector_perceptor
	detectors
	${catkin_LIBRARIES}
)

add_executable(baseline_pair_perceptor
	src/baseline_pair_perceptor_main.cpp
	src/baseline_pair_perceptor.cpp
)
add_dependencies(baseline_pair_perceptor
	${target_detector_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)
target_link_libraries(baseline_pair_perceptor
	${catkin_LIBRARIES}
)

add_executable(alvar_perceptor
	src/alvar_perceptor_main.cpp
	src/alvar_perceptor.cpp
)
add_dependencies(alvar_perceptor
	${target_detector_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)
target_link_libraries(alvar_perceptor
	${catkin_LIBRARIES}
)

add_executable(tmk_uwb_perceptor
	src/tmk_uwb_perceptor_main.cpp
	src/tmk_uwb_perceptor.cpp
)
add_dependencies(tmk_uwb_perceptor
	${target_detector_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)
target_link_libraries(tmk_uwb_perceptor
	${catkin_LIBRARIES}
)

install(
	TARGETS
		reflector_perceptor
		baseline_pair_perceptor
		alvar_perceptor
		tmk_uwb_perceptor
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS detectors
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/detectors/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
)
