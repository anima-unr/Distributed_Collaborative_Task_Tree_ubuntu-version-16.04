
# find_package(OpenCV REQUIRED)



add_executable(vec-merge
  src/vec_merge.cc
)

file(COPY scripts/vec_merge.py DESTINATION .)
# target_link_libraries(vec-merge
#   ${OpenCV_LIBS}
# )