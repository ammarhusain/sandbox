set (CMAKE_C_COMPILER             "/usr/bin/clang")
set (CMAKE_CXX_COMPILER             "/usr/bin/clang++")

project(ceres-tutorial)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(CMAKE_BUILD_TYPE Debug)


add_compile_options(-std=c++11)

find_package(Ceres 1.2.3 REQUIRED)

add_executable(helloworld helloworld.cpp)
target_include_directories(helloworld PRIVATE ${CERES_INCLUDE_DIRS})
target_link_libraries(helloworld ${CERES_LIBRARIES})

add_executable(curve-fitting curve-fitting.cpp)
target_include_directories(curve-fitting PRIVATE ${CERES_INCLUDE_DIRS})
target_link_libraries(curve-fitting ${CERES_LIBRARIES})

add_executable(bundle-adjuster bundle-adjuster.cpp)
target_include_directories(bundle-adjuster PRIVATE ${CERES_INCLUDE_DIRS})
target_link_libraries(bundle-adjuster ${CERES_LIBRARIES})

add_executable(robot-pose-mle robot-pose-mle.cpp)
target_include_directories(robot-pose-mle PUBLIC ${CERES_INCLUDE_DIRS})
target_link_libraries(robot-pose-mle ${CERES_LIBRARIES} ${GFLAGS_LIBRARIES})
message("gf ${GFLAGS_LIBRARIES}")