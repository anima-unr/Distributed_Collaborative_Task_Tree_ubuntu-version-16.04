CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
project(object-recognition)
set(BUILD_SHARED_LIBS ON)
INCLUDE_DIRECTORIES(include)
option(LOG_INFO_PRINTING "Add logging to builds executable for debugging")
if( ${LOG_INFO_PRINTING} )
  set(LOG_INFO_PRINTING 1)
  add_definitions(-DLOG_INFO_PRINTING=${LOG_INFO_PRINTING})
endif( ${LOG_INFO_PRINTING} )

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED)
option(BUILD_TESTS "Add test build functionality")

add_library(quad_tree
  src/quad_tree.cc
)

add_library(qr_detect
  src/qr_detect.cc
)

target_link_libraries(qr_detect
  quad_tree
  ${OpenCV_LIBS}
)

if( ${BUILD_TESTS} )
  add_executable(qr_test
    src/qr_test.cc
    src/qr_detect.cc
  )
  target_link_libraries(qr_test
    ${OpenCV_LIBS}
    quad_tree
  )

  add_executable(qt_test
    src/qt_test.cc
  )
  target_link_libraries(qt_test
    quad_tree
    ${OpenCV_LIBS}
  )

  add_executable(qr_video_test
    src/qr_video_test.cc
    src/qr_detect.cc
  )
  target_link_libraries(qr_video_test
    quad_tree
    ${OpenCV_LIBS}
  )
endif( ${BUILD_TESTS} )

option(BUILD_UTILS "Build utility programs" True)
if( ${BUILD_UTILS} )
  add_subdirectory(utils/)
endif( ${BUILD_UTILS} )

set(CMAKE_INSTALL_PREFIX $ENV{HOME}/.local)

# Install the qr library locally
install(TARGETS qr_detect quad_tree
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION ~/.bin
  ARCHIVE DESTINATION lib/static
)
install(DIRECTORY include/
  DESTINATION include/qr_detect
)
