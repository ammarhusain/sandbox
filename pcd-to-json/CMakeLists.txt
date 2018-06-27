find_package(Eigen3 REQUIRED)
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(pcd-to-json main.cpp jsoncpp.cpp json/json.h json/json-forwards.h tinyxml2/tinyxml2.h tinyxml2/tinyxml2.cpp)
target_include_directories(pcd-to-json PRIVATE ${EIGEN3_INCLUDE_DIRS})
target_link_libraries (pcd-to-json ${PCL_LIBRARIES})
