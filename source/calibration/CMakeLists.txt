oitk_set_module_vars(calibration)

if(NOT DEFINED EIGEN_FOUND)
    find_package(Eigen REQUIRED)
    # Use SYSTEM to enable <xxx> include style
    include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})
endif()

add_library(${MODULE_LIB_NAME} ${${MODULE_SRC_NAME}} ${${MODULE_HDR_NAME}})
