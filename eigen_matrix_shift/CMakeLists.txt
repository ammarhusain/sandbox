find_package(Eigen3 REQUIRED)
add_executable(block-shift main.cpp)
target_include_directories(block-shift PRIVATE ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(block-shift ${OpenCV_LIBS})