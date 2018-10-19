set (CMAKE_C_COMPILER             "/usr/bin/clang")
set (CMAKE_CXX_COMPILER             "/usr/bin/clang++")

project(SfM)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(CMAKE_BUILD_TYPE Debug)


add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_GLIBCXX_DEBUG")
message ("cxxflags ${CMAKE_CXX_FLAGS}")
#add_compile_options(-D_GLIBCXX_DEBUG)

find_package(OpenCV REQUIRED)

add_executable(SfM main.cpp common.h common.cpp optical_flow.h optical_flow.cpp)

target_link_libraries(SfM ${OpenCV_LIBS})