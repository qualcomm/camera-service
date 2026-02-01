project(qmmf_recorder_service VERSION 1.0)

find_package(PkgConfig REQUIRED)

pkg_check_modules(QMMF_CAMERA_METADATA
  REQUIRED qmmf_camera_metadata)
pkg_check_modules(QMMF_PROTO
  REQUIRED qmmf_proto)
pkg_check_modules(QMMF_CONFIG
  REQUIRED qmmf_config)
pkg_check_modules(QMMF_UTILS
  REQUIRED qmmf_utils)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DAEC_WAIT_TIMEOUT=500000000")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-c++11-narrowing")

if (NOT RECORDER_SERVICE_ENABLED)
  set(exclude EXCLUDE_FROM_ALL)
endif()

if (ENABLE_OFFLINE_JPEG)
list(APPEND jpeg "${CMAKE_CURRENT_SOURCE_DIR}/qmmf_offline_proc_impl.cc")
endif()

if (NOT CAMERA_HAL1_SUPPORT)
add_library(qmmf_recorder_service SHARED ${exclude}
  ${CMAKE_CURRENT_SOURCE_DIR}/qmmf_recorder_service.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/qmmf_recorder_impl.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/qmmf_remote_cb.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/qmmf_camera_source.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/qmmf_camera_context.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/qmmf_camera_rescaler.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/qmmf_camera_frc.cc
  ${jpeg}
)
else()
add_library(qmmf_recorder_service SHARED ${exclude}
  ${CMAKE_CURRENT_SOURCE_DIR}/qmmf_recorder_service.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/qmmf_recorder_impl.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/qmmf_remote_cb.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/qmmf_camera_source.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/qmmf_camera_context_hal1.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/qmmf_camera_rescaler.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/qmmf_camera_frc.cc
)
endif()
if (NOT CAMERA_HAL1_SUPPORT)
add_dependencies(
  qmmf_recorder_service
  qmmf_camera_adaptor
)
if (RESIZER_C2D_ENABLED)
add_dependencies(
  qmmf_recorder_service
  qmmf_common_resizer_c2d
)
endif()
if (RESIZER_FASTCV_ENABLED)
add_dependencies(
  qmmf_recorder_service
  qmmf_common_resizer_fastcv
)
endif()
if (RESIZER_NEON_ENABLED)
add_dependencies(
  qmmf_recorder_service
  qmmf_common_resizer_neon
)
endif()
else()
add_dependencies(
  qmmf_recorder_service
  qmmf_common_resizer_neon
  qmmf_common_resizer_c2d
)
endif()
if (NOT DISABLE_RESCALER_COLORSPACE)
add_dependencies(qmmf_recorder_service qdMetaData)
endif()

target_include_directories(qmmf_recorder_service
 PRIVATE ${TOP_DIRECTORY}
         ${QMMF_UTILS_INCLUDE_DIRS}
         ${QMMF_CONFIG_INCLUDE_DIRS}
         ${QMMF_PROTO_INCLUDE_DIRS}
         ${CAMX_INCLUDE_DIRS}
 )

target_include_directories(qmmf_recorder_service
 PRIVATE ${TOP_DIRECTORY}/common/memory)

set_target_properties(qmmf_recorder_service PROPERTIES
 VERSION ${PROJECT_VERSION}
 SOVERSION ${PROJECT_VERSION_MAJOR}
 OUTPUT_NAME ${PROJECT_NAME}${TARGET_SUFFIX}
)

install(TARGETS qmmf_recorder_service DESTINATION lib OPTIONAL)
if (NOT CAMERA_HAL1_SUPPORT)
target_link_libraries(qmmf_recorder_service dl pthread
  ${QMMF_UTILS_LIBRARIES} qmmf_camera_adaptor qmmf_camera_interface)

if(HAVE_ANDROID_UTILS)
target_link_libraries(qmmf_recorder_service log utils cutils binder)
else()
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -lpthread -lrt -lm -lglib-2.0 -ldl -latomic")
target_link_libraries(qmmf_recorder_service
 ${QMMF_CONFIG_LIBRARIES}
 ${QMMF_PROTO_LIBRARIES}
 )
endif()

if (RESIZER_FASTCV_ENABLED)
target_link_libraries(
  qmmf_recorder_service
  qmmf_common_resizer_fastcv
)
endif()
if (RESIZER_C2D_ENABLED)
target_link_libraries(
  qmmf_recorder_service
  qmmf_common_resizer_c2d
)
endif()
if (RESIZER_NEON_ENABLED)
target_link_libraries(
  qmmf_recorder_service
  qmmf_common_resizer_neon
)
endif()
else()
target_link_libraries(qmmf_recorder_service dl log utils cutils binder
  pthread hardware ${QMMF_UTILS_LIBRARIES}
  qmmf_memory_interface qmmf_common_resizer_neon qmmf_common_resizer_c2d)
endif()

# TODO remove this hack when camx issue with propagating c and cpp glags is solved
if(HAVE_ANDROID_UTILS)
target_link_libraries(qmmf_recorder_service ion qmmf_camera_metadata)
else()
target_link_libraries(qmmf_recorder_service qmmf_camera_metadata)
endif()

if (NOT HAVE_BINDER_H)
target_link_libraries(qmmf_recorder_service ${QMMF_CONFIG_LIBRARIES})

target_compile_definitions(qmmf_recorder_service PRIVATE
  "-DQMMF_CURRENT_INTERFACE_VER=\"${QMMF_CURRENT_INTERFACE_VER}\"")

include(CMakePrintHelpers)
cmake_print_properties(
    TARGETS qmmf_recorder_service
    PROPERTIES MANUALLY_ADDED_DEPENDENCIES INCLUDE_DIRECTORIES
)
endif()
