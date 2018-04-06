cmake_minimum_required(VERSION 2.8.3)
project(usc_utilities)

find_package(catkin REQUIRED COMPONENTS
  rosconsole
  roscpp
  tf
  kdl_parser
  sensor_msgs
  geometry_msgs
  visualization_msgs
  message_generation
  rosbag
  cmake_modules
)

find_package(orocos_kdl)
#find_package(bspline)


add_message_files(
  FILES
  AccumulatedTrialStatistics.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  #INCLUDE_DIRS include include/${PACKAGE_NAME}/
  INCLUDE_DIRS include
  LIBRARIES usc_utilities
  CATKIN_DEPENDS rosconsole roscpp tf kdl_parser sensor_msgs geometry_msgs visualization_msgs rosbag
  DEPENDS system_lib
)

find_package(Eigen REQUIRED)
include_directories(
  include
  ${EIGEN_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
add_definitions(${EIGEN_DEFINITIONS})

add_library(usc_utilities
	src/accumulator.cpp
	src/kdl_chain_wrapper.cpp
	src/rviz_marker_manager.cpp
	src/rviz_publisher.cpp
	src/sl_config_file_handler.cpp
)

add_dependencies(usc_utilities
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(usc_utilities_test
	test/asserts_enabled_test.cpp
	test/asserts_disabled_test.cpp
	test/param_server_test.cpp
	test/accumulator_test.cpp
	test/test_main.cpp
)
target_link_libraries(usc_utilities_test
  usc_utilities
  ${catkin_LIBRARIES}
)
target_link_libraries(usc_utilities_test gtest
  ${catkin_LIBRARIES}
)

