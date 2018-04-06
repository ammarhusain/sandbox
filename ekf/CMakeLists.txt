find_package(Eigen3 REQUIRED)
message("Eigen: ${EIGEN3_INCLUDE_DIRS}")
add_executable(kalman-filter main.cpp)
target_include_directories(kalman-filter PRIVATE ${EIGEN3_INCLUDE_DIRS})
