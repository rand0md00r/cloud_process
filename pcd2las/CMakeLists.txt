cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcd2las)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -pthread -g -march=native")

find_package(LASlib REQUIRED)
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcd2las pcd2las.cpp)
target_link_libraries (pcd2las ${PCL_LIBRARIES} LASlib)
