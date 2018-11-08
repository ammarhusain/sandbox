set (CMAKE_C_COMPILER             "/usr/bin/clang")
set (CMAKE_CXX_COMPILER             "/usr/bin/clang++")

project(ceres-tutorial)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(CMAKE_BUILD_TYPE Debug)


add_compile_options(-std=c++11)

find_package(Ceres 1.2.3 REQUIRED)

message("*********in ${CERES_INCLUDE_DIRS}")
message("*******lib ${CERES_LIBRARIES}")
add_executable(ceres-tutorials helloworld.cpp)
target_include_directories(ceres-tutorials PRIVATE ${CERES_INCLUDE_DIRS})
target_link_libraries(ceres-tutorials ${CERES_LIBRARIES})
