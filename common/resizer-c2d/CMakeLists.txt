project(qmmf_common_resizer_c2d)

add_library(qmmf_common_resizer_c2d SHARED ${exclude}
${CMAKE_CURRENT_SOURCE_DIR}/qmmf_resizer_c2d.cc
)

target_include_directories(qmmf_common_resizer_c2d
 PRIVATE ${TOP_DIRECTORY})

target_include_directories(qmmf_common_resizer_c2d
 PRIVATE ${TOP_DIRECTORY}/common/memory)

install(TARGETS qmmf_common_resizer_c2d DESTINATION lib OPTIONAL)
target_link_libraries(qmmf_common_resizer_c2d pthread dl C2D2)
if(HAVE_ANDROID_UTILS)
target_link_libraries(qmmf_common_resizer_c2d log utils cutils)
endif()
