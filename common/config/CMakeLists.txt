project(qmmf_config)

add_library(qmmf_config SHARED
  qmmf_config.cc
)

find_package(PkgConfig)

pkg_check_modules(GLIB glib-2.0)

# Include GLib headers
target_include_directories(qmmf_config
  PRIVATE ${GLIB_INCLUDE_DIRS}
)

# Link GLib libraries
target_link_libraries(qmmf_config
  qmmf_utils
  ${GLIB_LIBRARIES}
  ${CMAKE_DL_LIBS}
)

target_compile_definitions(
  qmmf_config PRIVATE
  QMMF_CONFIG_FILE=\"${CAM_SERVER_CONFIG_DIR}/cam-server.ini\"
)

install(TARGETS qmmf_config DESTINATION lib OPTIONAL)
