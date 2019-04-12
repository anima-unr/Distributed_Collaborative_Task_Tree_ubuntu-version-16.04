configure_file("${CMAKE_SOURCE_DIR}/Distributed_Collaborative_Task_Tree/unr_object_manipulation/cmake/template/uomconfig.h.in" "${CMAKE_BINARY_DIR}/unr_object_manipulation/uomconfig.h")
include_directories(${CMAKE_BINARY_DIR})
# configure_file("${MANIP_SOURCE_DIR}/cmake/template/uomconfig.h.in" "${CONFIG_INCLUDE_DIR}/uomconfig.h")
install(FILES "${CMAKE_BINARY_DIR}/uomconfig.h" DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
