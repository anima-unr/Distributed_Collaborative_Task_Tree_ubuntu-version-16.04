cmake_minimum_required(VERSION 2.8.3)
project(remote_mutex)

find_package(catkin REQUIRED COMPONENTS
	roscpp
 	rospy
  std_msgs
  robotics_task_tree_msgs
  timeseries_recording_toolkit
  message_generation
)

# find_package(Boost REQUIRED COMPONENTS system)

add_service_files(
  FILES
  remote_mutex_msg.srv
)

generate_messages(
  DEPENDENCIES  std_msgs robotics_task_tree_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES remote_mutex
  DEPENDS timeseries_recording_toolkit robotics_task_tree_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(remote_mutex
  src/${PROJECT_NAME}/remote_mutex.cpp
)

add_executable(remote_mutex_service
  src/${PROJECT_NAME}/remote_mutex_service.cpp
)

add_dependencies(remote_mutex_service
  remote_mutex
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
)

add_dependencies(remote_mutex
  ${PROJECT_NAME}_generate_messages_cpp
  timeseries_recording_toolkit
)

target_link_libraries(remote_mutex_service
  ${catkin_LIBRARIES}
  #timeseries_recording_toolkit
)
target_link_libraries(remote_mutex
  ${catkin_LIBRARIES}
)


## Mark executables and/or libraries for installation
install(TARGETS remote_mutex remote_mutex_service
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".hg" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
