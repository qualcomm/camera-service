cmake_minimum_required(VERSION 3.1)

project(qmmf_propertyvault)

add_library(qmmf_propertyvault SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/qmmf_propertyvault.cc
)

target_link_libraries(qmmf_propertyvault ${CMAKE_DL_LIBS})

install(TARGETS qmmf_propertyvault DESTINATION lib OPTIONAL)
