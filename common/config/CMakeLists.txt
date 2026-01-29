project(qmmf_config VERSION 1.0)

add_library(qmmf_config SHARED
  qmmf_config.cc
)

target_link_libraries(qmmf_config
  qmmf_utils
  ${CMAKE_DL_LIBS}
)

target_compile_definitions(
  qmmf_config PRIVATE
  QMMF_CONFIG_FILE=\"${CAM_SERVER_CONFIG_DIR}/cam-server.ini\"
)

set_target_properties(qmmf_config PROPERTIES
 VERSION ${PROJECT_VERSION}
 SOVERSION ${PROJECT_VERSION_MAJOR}
)

install(TARGETS qmmf_config DESTINATION lib OPTIONAL)

# Configure and install pkg-config file
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/qmmf_config.pc.in
  ${CMAKE_CURRENT_BINARY_DIR}/qmmf_config.pc
  @ONLY
)

install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/qmmf_config.pc
  DESTINATION lib/pkgconfig
)
