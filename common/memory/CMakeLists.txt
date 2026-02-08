project(qmmf_memory_interface VERSION 1.0)

find_package(PkgConfig REQUIRED)

pkg_check_modules(QMMF_CAMERA_METADATA
  REQUIRED qmmf_camera_metadata)
pkg_check_modules(QMMF_PROTO
  REQUIRED qmmf_proto)
pkg_check_modules(QMMF_UTILS
  REQUIRED qmmf_utils)

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

elseif(USE_LIBGBM)
add_library(qmmf_memory_interface SHARED ${exclude}
qmmf_memory_interface.cc
qmmf_gbm_interface.cc)
elseif(NOT HAVE_ANDROID_UTILS)
add_library(qmmf_memory_interface SHARED ${exclude}
qmmf_memory_interface.cc
qmmf_dmabuf_interface.cc)
else()
add_library(qmmf_memory_interface SHARED ${exclude}
qmmf_memory_interface.cc
qmmf_gralloc_interface.cc)
endif()

target_include_directories(qmmf_memory_interface
 PRIVATE ${TOP_DIRECTORY}
         ${QMMF_UTILS_INCLUDE_DIRS}
         ${QMMF_CONFIG_INCLUDE_DIRS}
         ${CAMX_INCLUDE_DIRS}
 )

target_include_directories(qmmf_memory_interface
 PRIVATE ${TOP_DIRECTORY}/common/memory)

target_include_directories(qmmf_memory_interface
 PRIVATE ${TOP_DIRECTORY}/common/utils)

set_target_properties(qmmf_memory_interface PROPERTIES
 VERSION ${PROJECT_VERSION}
 SOVERSION ${PROJECT_VERSION_MAJOR}
 OUTPUT_NAME ${PROJECT_NAME}${TARGET_SUFFIX}
)

install(TARGETS qmmf_memory_interface DESTINATION lib OPTIONAL)
target_link_libraries(qmmf_memory_interface ${QMMF_UTILS_LIBRARIES} dl)
if(HAVE_ANDROID_UTILS)
target_link_libraries(qmmf_memory_interface log utils cutils hardware)
endif()
if(USE_LIBGBM)
target_link_libraries(qmmf_memory_interface gbm)
else()
target_link_libraries(qmmf_memory_interface ${QMMF_CONFIG_LIBRARIES})
endif()
