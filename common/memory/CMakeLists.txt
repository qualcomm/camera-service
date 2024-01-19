cmake_minimum_required(VERSION 3.1)

project(qmmf_memory_interface)

if (NOT MEMORY_ENABLED)
set(exclude EXCLUDE_FROM_ALL)
endif()

if(TARGET_USES_GRALLOC1)
add_library(qmmf_memory_interface SHARED ${exclude}
qmmf_memory_interface.cc
qmmf_gralloc1_interface.cc)

elseif(TARGET_USES_GRALLOC2)
add_library(qmmf_memory_interface SHARED ${exclude}
qmmf_memory_interface.cc
qmmf_gralloc2_interface.cc)

elseif(TARGET_USES_GBM)
add_library(qmmf_memory_interface SHARED ${exclude}
qmmf_memory_interface.cc
qmmf_gbm_interface.cc)

else()
add_library(qmmf_memory_interface SHARED ${exclude}
qmmf_memory_interface.cc
qmmf_gralloc_interface.cc)
endif()
add_dependencies(qmmf_memory_interface qmmf_utils)

target_include_directories(qmmf_memory_interface
 PRIVATE ${TOP_DIRECTORY})

target_include_directories(qmmf_memory_interface
 PRIVATE ${TOP_DIRECTORY}/common/memory)

target_include_directories(qmmf_memory_interface
 PRIVATE ${TOP_DIRECTORY}/common/utils)

install(TARGETS qmmf_memory_interface DESTINATION lib OPTIONAL)
target_link_libraries(qmmf_memory_interface hardware qmmf_utils dl)
if(HAVE_ANDROID_UTILS)
target_link_libraries(qmmf_memory_interface log utils cutils)
endif()
if(TARGET_USES_GBM)
target_link_libraries(qmmf_memory_interface gbm)
endif()