project(qmmf_common_resizer_neon)

if (NOT RESIZER_NEON_ENABLED)
set(exclude EXCLUDE_FROM_ALL)
endif()

add_library(qmmf_common_resizer_neon SHARED ${exclude}
${CMAKE_CURRENT_SOURCE_DIR}/res_neon.cc
${CMAKE_CURRENT_SOURCE_DIR}/qmmf_resizer_neon.cc
)

target_include_directories(qmmf_common_resizer_neon
 PRIVATE ${TOP_DIRECTORY})

target_include_directories(qmmf_common_resizer_neon
 PRIVATE ${TOP_DIRECTORY}/common/memory)

install(TARGETS qmmf_common_resizer_neon DESTINATION lib OPTIONAL)
target_link_libraries(qmmf_common_resizer_neon pthread qmmf_utils)
if(HAVE_ANDROID_UTILS)
target_link_libraries(qmmf_common_resizer_neon log utils cutils)
else()
target_link_libraries(qmmf_common_resizer_neon qmmf_config)
endif()
