cmake_minimum_required (VERSION 2.8.11)
project(OITK) # Open ICV Toolkit

# Add cmake macros and functions
set(CMAKE_MODULE_PATH "${OITK_SOURCE_DIR}/cmake/modules/" ${CMAKE_MODULE_PATH})
include(${OITK_SOURCE_DIR}/cmake/common.cmake)

# Search for dependencies
if(NOT DEFINED EIGEN_FOUND)
    find_package(Eigen REQUIRED)
    # Use SYSTEM to enable <xxx> include style
    include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})
endif()

# Add submodules
include_directories(${OITK_SOURCE_DIR}/include)
add_subdirectory(source/calibration)

# Add command line interface
add_executable(oitk entry.cxx)
target_link_libraries(oitk PRIVATE oitk_calibration)
