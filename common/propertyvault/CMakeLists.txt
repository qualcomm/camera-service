cmake_minimum_required(VERSION 3.1)

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

target_link_libraries(qmmf_propertyvault ${CMAKE_DL_LIBS})

install(TARGETS qmmf_propertyvault DESTINATION lib OPTIONAL)
