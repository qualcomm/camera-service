project(qmmf_propertyvault)

add_library(qmmf_propertyvault SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/qmmf_propertyvault.cc
)

find_package(PkgConfig)

pkg_check_modules(PROPERTY_VAULT property-vault)

string(REGEX MATCH "^[0-9]+" PROPERTY_VAULT_VERSION_MAJOR ${PROPERTY_VAULT_VERSION})

target_compile_definitions(
  qmmf_propertyvault PRIVATE
  PROPERTY_VAULT_VER="${PROPERTY_VAULT_VERSION_MAJOR}"
)

add_dependencies(qmmf_camera_adaptor qmmf_utils)

target_include_directories(qmmf_propertyvault
 PRIVATE ${TOP_DIRECTORY})

target_include_directories(qmmf_propertyvault
 PRIVATE ${TOP_DIRECTORY}/common/memory)

target_include_directories(qmmf_propertyvault
 PRIVATE ${TOP_DIRECTORY}/common/utils)

target_link_libraries(qmmf_propertyvault qmmf_utils ${CMAKE_DL_LIBS})

install(TARGETS qmmf_propertyvault DESTINATION lib OPTIONAL)
