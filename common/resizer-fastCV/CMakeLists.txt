cmake_minimum_required(VERSION 3.1)

project(qmmf_common_resizer_fastcv)

if (NOT RESIZER_FASTCV_ENABLED)
set(exclude EXCLUDE_FROM_ALL)
endif()

add_library(qmmf_common_resizer_fastcv SHARED ${exclude}
${CMAKE_CURRENT_SOURCE_DIR}/qmmf_resizer_fastCV.cc
)
add_dependencies(qmmf_common_resizer_fastcv qmmf_memory_interface)

target_include_directories(qmmf_common_resizer_fastcv
 PRIVATE ${TOP_DIRECTORY})

target_include_directories(qmmf_common_resizer_fastcv
 PRIVATE $<BUILD_INTERFACE:${KERNEL_INCDIR}/usr/include>)

target_include_directories(qmmf_common_resizer_fastcv
 PRIVATE ${TOP_DIRECTORY}/common/memory)

install(TARGETS qmmf_common_resizer_fastcv DESTINATION lib OPTIONAL)
target_link_libraries(qmmf_common_resizer_fastcv pthread dl fastcvopt qmmf_memory_interface)
target_link_libraries(qmmf_common_resizer_fastcv qmmf_propertyvault)
if(HAVE_ANDROID_UTILS)
target_link_libraries(qmmf_common_resizer_fastcv log utils cutils)
endif()
