set (CMAKE_C_COMPILER             "/usr/bin/clang")
set (CMAKE_CXX_COMPILER             "/usr/bin/clang++")

project(SfM)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(CMAKE_BUILD_TYPE Debug)


add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

add_executable(SfM main.cpp common.h common.cpp optical_flow.h optical_flow.cpp rich_feature.h rich_feature.cpp camera_matrices.h camera_matrices.cpp triangulation.h triangulation.cpp reconstruction.h reconstruction.cpp)

target_link_libraries(SfM ${OpenCV_LIBS} ${PCL_LIBRARIES})
target_include_directories(SfM PUBLIC ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
