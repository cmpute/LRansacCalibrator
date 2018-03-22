cmake_minimum_required (VERSION 2.8.11)
project(OITK) # Open ICV Toolkit

# Add cmake macros and functions
set(CMAKE_MODULE_PATH "${OITK_SOURCE_DIR}/cmake/modules/" ${CMAKE_MODULE_PATH})
include(${OITK_SOURCE_DIR}/cmake/common.cmake)

# Find dependencies
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL COMPONENTS common io sample_consensus search features segmentation visualization REQUIRED)
# Use SYSTEM to enable <xxx> include style
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

# Add submodules
include_directories(${OITK_SOURCE_DIR}/include)
include_directories(${OITK_SOURCE_DIR}/source) # for *.hxx
add_subdirectory(source/calibration)

# Add command line interface
add_executable(oitk entry.cxx)
add_dependencies(oitk oitk_calibration)
target_link_libraries(oitk PRIVATE oitk_calibration)
target_link_libraries(oitk ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
