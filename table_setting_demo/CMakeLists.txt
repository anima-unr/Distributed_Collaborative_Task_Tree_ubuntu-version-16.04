cmake_minimum_required(VERSION 2.8.3)
project(table_setting_demo)

set(CMAKE_CXX_FLAGS "--std=c++0x -g3")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  geometry_msgs
  active_vision_msgs
  moveit_ros_planning_interface
  robotics_task_tree_eval
  tf
)
if ($ENV{ROBOT} MATCHES "pr2")
  option(BUILD_TABLE_SETTING_DEMO "build table setting demo" OFF)
else ($ENV{ROBOT} MATCHES "pr2")
  option(BUILD_TABLE_SETTING_DEMO "build table setting demo" ON)
endif ($ENV{ROBOT} MATCHES "pr2")

if (${BUILD_TABLE_SETTING_DEMO})
  message("-- BUILD: TABLE_SETTING_DEMO")
  
  find_package(catkin REQUIRED COMPONENTS
    robotics_task_tree_eval
    remote_mutex
    message_generation
    #tabletop_segmenter
    moveit_ros_planning_interface
    robotics_task_tree_eval
    tf
  )
  find_package(PCL REQUIRED COMPONENTS
    common
    io
  )
  INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})

  #  find_package(Eigen3 REQUIRED)
  #  INCLUDE_DIRECTORIES(${EIGEN_INVLUDE_DIR})
endif (${BUILD_TABLE_SETTING_DEMO})
find_package(Boost REQUIRED COMPONENTS 
  date_time
  thread
)

option(LOG_INFO_PRINTING "Output Debugging logging information" 0)
if (${LOG_INFO_PRINTING})
  add_definitions(-DLOG_INFO_PRINTING)
endif()
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  pick_and_place.srv
  pick_and_place_state.srv
  pick_and_place_stop.srv
  object_request.srv
  object_position.srv
  ObjectTransformation.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES table_setting_demo
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)


## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
execute_process(
  COMMAND rosversion -d
  OUTPUT_VARIABLE ROS_VERSION
)
set(QR_SEARCHES_ $ENV{HOME}/.local/include /usr/local/include)
set(QR_NAMES qr_detect quad_tree)
set(QR_LIB_SEARCH $ENV{HOME}/.local/lib /usr/local/lib)
find_path(QR_INCLUDE_DIRS NAMES qr_detect.h quad_tree.h
  PATHS ${QR_SEARCHES_}
  PATH_SUFFIXES qr_detect
)
find_library(QR_LIBRARY
  NAMES qr_detect
  PATHS ${QR_LIB_SEARCH}
)

find_library(Quad_LIBRARY
  NAMES quad_tree
  PATHS ${QR_LIB_SEARCH}
)
set(QR_LIBRARIES ${Quad_LIBRARY} ${QR_LIBRARY})
if (EXISTS ${QR_LIBRARY} AND EXISTS ${Quad_LIBRARY})
  set(QR_FOUND True)
else()
  set(QR_FOUND False)
endif()


#JB: This used to be only groovy to build on the pr2, but since we are running it in 
#    simulation now, we need to build it on our host machine!
# if (${ROS_VERSION} MATCHES "groovy")
  find_package(catkin REQUIRED COMPONENTS
    # arm_navigation
    # pr2_controllers
    tf
    actionlib
    moveit_ros_planning_interface
  )

  set(message_INCLUDE_DIRS
# JB: TODO: The arm_navigation does not exist in indigo, it is replaced by MoveIt!
#           So will need to more than a simple change here!!!!
#    /opt/ros/groovy/stacks/arm_navigation/arm_navigation_msgs/msg_gen/cpp/include/

    # /opt/ros/groovy/stacks/pr2_controllers/pr2_controllers_msgs/msg_gen/cpp/include/
    /opt/ros/indigo/stacks/pr2_controllers/pr2_controllers_msgs/msg_gen/cpp/include/

  )
  option(BUILD_PR2_PICKPLACE "Build PR2 pick and place service" 1)
  if (${BUILD_PR2_PICKPLACE})
    include_directories(
      ${message_INCLUDE_DIRS}
    )
    add_executable(pr2_pick_service
      src/pick_and_place/pr2_pick_and_place_service.cc
      src/pick_and_place/pick_and_place_service.cc
    )
    target_link_libraries(pr2_pick_service
      ${catkin_LIBRARIES}
    )
    add_dependencies(pr2_pick_service
    table_setting_demo_generate_messages_cpp
  )
  endif (${BUILD_PR2_PICKPLACE})
# endif (${ROS_VERSION} MATCHES "groovy")
################################################################################
################################################################################
if (${BUILD_TABLE_SETTING_DEMO})


  #find_package(OpenCV REQUIRED)


  find_package(catkin REQUIRED COMPONENTS
    std_msgs
    sensor_msgs
    image_transport
    cv_bridge
    robotics_task_tree_eval
    remote_mutex
    pcl_ros
    tf
    moveit_ros_planning_interface

  )

  option(BUILD_TEST "Build Testing Modules" 1)

  if (${BUILD_TEST})

    add_library(table_object_behavior
      src/table_object_behavior.cpp
    )
    # DFS: took out because the testing directory is not compiling with tf correctly
    #add_subdirectory(testing)
  endif ()

  add_executable(table_setting_demo
    src/table_setting_demo.cpp
    src/table_object_behavior.cpp
  )
  add_dependencies(table_setting_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

  target_link_libraries(table_setting_demo
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
  )

# multi 
  add_executable(multi_robot_demo
    src/multi_robot_network.cc
    src/table_object_behavior.cpp
  )
  target_link_libraries(multi_robot_demo
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
  )
  add_dependencies(multi_robot_demo
    table_setting_demo_generate_messages_cpp
  )



  # BUILD OBJECT TRANSFORMATION service
  #add_executable(object_trans_service
   # src/3d_object_transformation_service/3d_object_trans_service.cc
   # src/3d_object_transformation_service/3d_object_transformation_service.cc
  #)
  #target_link_libraries(object_trans_service
  #  ${Boost_LIBRARIES}
  #  ${catkin_LIBRARIES}
  # ${OpenCV_LIBS}
  # ${PCL_COMMON_LIBRARIES}
  # ${PCL_IO_LIBRARIES}
  #)

  #add_dependencies(object_trans_service
  #  table_setting_demo_generate_messages_cpp
  #)

  #if (NOT ${QR_FOUND})
   # message( FATAL_ERROR "QR Library not found cannot build table demo")
  #endif()

  #include_directories(${QR_INCLUDE_DIRS})
  #add_executable(qr_object_track_service
  #  src/qr/qr_object_service.cc
  #  src/qr/qr_object_detect.cc
  #)

  #target_link_libraries(qr_object_track_service
  #  ${QR_LIBRARIES}
  #  ${catkin_LIBRARIES}
  #  ${OpenCV_LIBS}
  #)
  #add_dependencies(qr_object_track_service
  #  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  #  ${catkin_EXPORTED_TARGETS}
  #)
  add_dependencies(table_setting_demo
    table_setting_demo_generate_messages_cpp
  )
endif (${BUILD_TABLE_SETTING_DEMO})
