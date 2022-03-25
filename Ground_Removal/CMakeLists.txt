cmake_minimum_required(VERSION 3.0.2)
project(ground_removal)

set(CMAKE_BUILD_TYPE "Debug")
add_compile_options(-std=c++11)

find_package(PCL REQUIRED)
include_directories(
    ${PCL_INCLUDE_DIRS}
    include/
)

# link_libraries(${PCL_LIBRARIES_DIRS})

add_executable(ground_removal src/main.cpp src/Gauss.cpp src/GroundRemoval.cpp)
target_link_libraries(ground_removal ${PCL_LIBRARIES})