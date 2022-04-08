cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(NORMALS)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (NORMALS normals.cpp)
target_link_libraries (NORMALS ${PCL_LIBRARIES})
