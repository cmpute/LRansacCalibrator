oitk_set_module_vars(calibration)

add_library(${MODULE_LIB_NAME} ${${MODULE_SRC_NAME}} ${${MODULE_HDR_NAME}})
target_link_libraries(${MODULE_LIB_NAME} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
