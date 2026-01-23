project(qmmf_config)

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

install(TARGETS qmmf_config DESTINATION lib OPTIONAL)
