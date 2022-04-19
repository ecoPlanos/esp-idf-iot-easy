idf_component_register(
    SRCS "bsec_integration.c"
    INCLUDE_DIRS include
)
add_library(algobsec STATIC IMPORTED GLOBAL)
add_dependencies(algobsec algobsec_build)

set_target_properties(algobsec PROPERTIES IMPORTED_LOCATION
     ${COMPONENT_DIR}/lib/libalgobsec.a)
set_target_properties(algobsec PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
     ${COMPONENT_DIR}/include)

target_link_libraries(${COMPONENT_TARGET} "-L${CMAKE_CURRENT_LIST_DIR}/lib")
target_link_libraries(${COMPONENT_TARGET} algobsec)
