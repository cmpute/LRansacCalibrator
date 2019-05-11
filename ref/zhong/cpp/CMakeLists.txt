cmake_minimum_required(VERSION 3.5.0)
set(CMAKE_CXX_STANDARD 11)

project(plane_finder)
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# support cython/c api
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_LIST_DIR})

add_executable(detector test_regiongrow.cpp)
target_link_libraries(detector ${PCL_LIBRARIES})

