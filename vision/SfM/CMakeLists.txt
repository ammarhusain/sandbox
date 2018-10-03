project(SfM)
cmake_minimum_required(VERSION 2.8)

add_compile_options(-std=c++11)
find_package(OpenCV REQUIRED)

add_executable(SfM main.cpp common.h common.cpp)

message ("ocvl: ${OpenCV_LIBS}")
target_link_libraries(SfM ${OpenCV_LIBS})