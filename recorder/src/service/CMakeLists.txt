cmake_minimum_required(VERSION 3.1)

project(qmmf_recorder_service)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DAEC_WAIT_TIMEOUT=500000000")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-c++11-narrowing")

if (NOT RECORDER_SERVICE_ENABLED)
  set(exclude EXCLUDE_FROM_ALL)
endif()

if (ENABLE_OFFLINE_JPEG)
  list(APPEND jpeg "${CMAKE_CURRENT_SOURCE_DIR}/qmmf_offline_jpegenc_impl.cc")
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
  qmmf_utils
  qmmf_camera_adaptor
  qmmf_recorder_client
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
  qmmf_utils
  qmmf_common_resizer_neon
  qmmf_common_resizer_c2d
  qmmf_recorder_client
)
endif()
if (NOT DISABLE_RESCALER_COLORSPACE)
add_dependencies(qmmf_recorder_service qdMetaData)
endif()

target_include_directories(qmmf_recorder_service
 PRIVATE ${TOP_DIRECTORY})

target_include_directories(qmmf_recorder_service
 PRIVATE $<BUILD_INTERFACE:${KERNEL_INCDIR}/usr/include>)

# Headers path for msm-5.4 media on qrbx210 targets.
target_include_directories(qmmf_recorder_service
 PRIVATE $<BUILD_INTERFACE:${KERNEL_INCDIR}/usr/include/vidc>)

target_include_directories(qmmf_recorder_service
 PRIVATE ${TOP_DIRECTORY}/common/memory)

install(TARGETS qmmf_recorder_service DESTINATION lib OPTIONAL)
if (NOT CAMERA_HAL1_SUPPORT)
target_link_libraries(qmmf_recorder_service dl log utils cutils binder
  pthread camera_metadata hardware qmmf_recorder_client
  qmmf_utils qmmf_camera_adaptor )
if(NOT CAMERA_CLIENT_DISABLED)
target_link_libraries(qmmf_recorder_service camera_client )
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
  pthread hardware qmmf_recorder_client qmmf_utils
  qmmf_memory_interface qmmf_common_resizer_neon qmmf_common_resizer_c2d)
if(NOT CAMERA_CLIENT_DISABLED)
target_link_libraries(qmmf_recorder_service camera_client )
endif()
endif()

# TODO remove this hack when camx issue with propagating c and cpp glags is solved
target_link_libraries(qmmf_recorder_service ion qmmf_camera_metadata)
