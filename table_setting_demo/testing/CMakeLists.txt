
find_package(Boost REQUIRED COMPONENTS
  thread
)

include_directories(include)

add_executable(collaborative_test
  src/collaborative_test.cc
  src/collaborative_behavior.cc
)

target_link_libraries(collaborative_test
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  table_object_behavior
)