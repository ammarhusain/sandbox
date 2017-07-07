find_package(Eigen3 REQUIRED)
message("Eigen: ${EIGEN3_INCLUDE_DIRS}")
add_executable(connected-components main.cpp)
target_include_directories(connected-components PRIVATE ${EIGEN3_INCLUDE_DIRS})
