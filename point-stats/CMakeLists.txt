find_package(Eigen3 REQUIRED)
message("Eigen: ${EIGEN3_INCLUDE_DIRS}")
add_executable(point-stats main.cpp)
target_include_directories(point-stats PRIVATE ${EIGEN3_INCLUDE_DIRS})
