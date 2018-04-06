cmake_minimum_required(VERSION 2.8.3)
project(unr_object_manipulation)

include(cmake/CPlusPlusCompiler.cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  moveit_core
  geometry_msgs
  moveit_msgs
)
#find_package(GTest REQUIRED)
find_package(Eigen)
find_package(Boost REQUIRED COMPONENTS
  thread
  python
  program_options
  system
  filesystem
)
find_package(PythonLibs 2.7 REQUIRED)

include_directories(
  ${Eigen_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)
# message("PYTHON INCLUDES ${PYTHON_INCLUDES")

catkin_python_setup()

add_service_files(
  FILES
  object_request.srv
  pick_and_place.srv
  pick_and_place_state.srv
  pick_and_place_stop.srv
)

# add_action_files(
#   FILES
# )

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  # INCLUDE_DIRS
  # LIBRARIES
  CATKIN_DEPENDS
    moveit_core
  # DPENDS
)

# cmake options
option(LOGGING_ENABLED "Enable Log info printing" OFF)
if(${LOGGING_ENABLED})
  set(LOG_INFO_PRINTING ON)
endif()

if("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
  set(DEBUG ON)
endif()


# Setup configuration files
include(cmake/ManipulationGenHeaders.cmake)

# determine which rosdistro is being used
execute_process(
  COMMAND rosversion -d
  OUTPUT_VARIABLE ROS_VERSION
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

################################################################################
########################## BUILD PYTHON BINDINGS TOOLS #########################
################################################################################
set(PYTHON_BINDINGS_LIB_NAME py_bindings_tools)
add_library(${PYTHON_BINDINGS_LIB_NAME}
  src/py_bindings_tools/roscpp_initializer.cc
)
target_link_libraries(${PYTHON_BINDINGS_LIB_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PYTHON_LIBRARIES}
)

add_library(${PYTHON_BINDINGS_LIB_NAME}_python
  src/py_bindings_tools/wrap_roscpp_initializer_python.cc
)
target_link_libraries(${PYTHON_BINDINGS_LIB_NAME}_python
  ${PYTHON_BINDINGS_LIB_NAME}
  ${PYTHON_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
################################################################################
############################ Build Main Library API ############################
################################################################################
set(GRASP_LIBRARY grasp_server_library)
add_library(${GRASP_LIBRARY}
  src/grasp_server_library/grasp_server.cc
)

target_link_libraries(${GRASP_LIBRARY}
  ${catkin_LIBRARIES}
)
################################################################################
################################################################################

################################################################################
######################## Build Python Main Library API #########################
################################################################################
add_library(${GRASP_LIBRARY}_python
  src/grasp_server_library/wrap_grasp_server_python.cc
)
target_link_libraries(${GRASP_LIBRARY}_python
  ${GRASP_LIBRARY}
  ${PYTHON_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PYTHON_BINDINGS_LIB_NAME}
)
set_target_properties(${GRASP_LIBRARY}_python
  PROPERTIES
    OUTPUT_NAME grasp_server PREFIX "")
set_target_properties(${GRASP_LIBRARY}_python
  PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY 
      ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
################################################################################
################################################################################


################################################################################
########################### GRASP COMMANDER SERVICE ############################
################################################################################
add_executable(grasp_commander_service
  src/grasp_commander/grasp_commander_service.cc
)
target_link_libraries(grasp_commander_service
  ${catkin_LIBRARIES}
)
add_dependencies(grasp_commander_service
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
################################################################################
################################################################################

add_subdirectory(baxter_table_setting)

################################################################################
################################### TESTING ####################################
################################################################################
add_executable(grasp_server_test
  src/grasp_server_library/grasp_server_test.cc
)

target_link_libraries(grasp_server_test
  grasp_server_library
  ${GTEST_LIBRARIES}
  pthread
)

enable_testing()
add_test(
  NAME server_test
  COMMAND grasp_server_test
)
################################################################################
################################################################################