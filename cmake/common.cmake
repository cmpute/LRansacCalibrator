# Set up essential names for a module
macro(oitk_set_module_names MNAME)
    set(MODULE_NAME ${MNAME})                       # Module name
    string(TOUPPER ${MNAME} MODULE_NAME_UPPER)      # Module name (upper case)
    set(MODULE_LIB_NAME oitk_${MNAME})              # Module library name
    set(MODULE_SRC_NAME ${MODULE_NAME_UPPER}_SRCS)  # List of module sources
    set(MODULE_HDR_NAME ${MODULE_NAME_UPPER}_HDRS)  # List of module headers
endmacro()

macro(oitk_set_module_vars MNAME)
    oitk_set_module_names(${MNAME})
    file(GLOB ${MODULE_SRC_NAME} *.cxx *.hxx)
    file(GLOB ${MODULE_HDR_NAME} ../../include/${MNAME}/*.h)
endmacro()
