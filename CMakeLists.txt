# Copyright (C) 2016 Ammar Husain. All Rights Reserved.
cmake_minimum_required(VERSION 2.6)

#set(CMAKE_CXX_FLAGS "-Wall -g")

project(algorithm-sandbox)

enable_testing()

add_definitions("-c")

set(CMAKE_CXX_FLAGS "-Wall -std=c++14 -g")

add_subdirectory(connected-components)
add_subdirectory(point-stats)
add_subdirectory(pcd-to-json)