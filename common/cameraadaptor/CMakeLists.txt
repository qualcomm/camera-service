project(qmmf_camera_adaptor_${PLATFORM} VERSION 1.0)

find_package(PkgConfig REQUIRED)

pkg_check_modules(QMMF_PROTO
  REQUIRED qmmf_proto)
pkg_check_modules(QMMF_CONFIG
  REQUIRED qmmf_config)
pkg_check_modules(QMMF_UTILS
  REQUIRED qmmf_utils)

if (NOT CAMERAADAPTOR_ENABLED)
set(exclude EXCLUDE_FROM_ALL)
endif()

add_library(qmmf_camera_adaptor_${PLATFORM} SHARED ${exclude}
${CMAKE_CURRENT_SOURCE_DIR}/qmmf_camera3_device_client.cc
${CMAKE_CURRENT_SOURCE_DIR}/qmmf_camera3_monitor.cc
${CMAKE_CURRENT_SOURCE_DIR}/qmmf_camera3_request_handler.cc
${CMAKE_CURRENT_SOURCE_DIR}/qmmf_camera3_prepare_handler.cc
${CMAKE_CURRENT_SOURCE_DIR}/qmmf_camera3_stream.cc
${CMAKE_CURRENT_SOURCE_DIR}/qmmf_camera3_utils.cc
${CMAKE_CURRENT_SOURCE_DIR}/qmmf_camera3_smooth_zoom.cc
)

if (HAVE_BINDER_H)
add_dependencies(qmmf_camera_adaptor_${PLATFORM} qmmf_utils)
else()
add_dependencies(qmmf_camera_adaptor_${PLATFORM} qmmf_proto
  qmmf_utils
  qmmf_config
)
endif()

target_include_directories(qmmf_camera_adaptor_${PLATFORM}
 PRIVATE ${TOP_DIRECTORY}
         ${QMMF_UTILS_INCLUDE_DIRS}
         ${QMMF_CONFIG_INCLUDE_DIRS}
         ${QMMF_PROTO_INCLUDE_DIRS}
         ${CAMX_INCLUDE_DIRS}
)

target_include_directories(qmmf_camera_adaptor_${PLATFORM}
 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_include_directories(qmmf_camera_adaptor_${PLATFORM}
 PRIVATE ${TOP_DIRECTORY}/common/memory)

target_compile_definitions(qmmf_camera_adaptor_${PLATFORM} PRIVATE
  $<$<BOOL:${HAVE_MMM_COLOR_FMT_H}>:HAVE_MMM_COLOR_FMT_H>
)

set_target_properties(qmmf_camera_adaptor_${PLATFORM} PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
  OUTPUT_NAME qmmf_camera_adaptor${TARGET_SUFFIX}
)

install(TARGETS qmmf_camera_adaptor_${PLATFORM} DESTINATION lib OPTIONAL)

target_link_libraries(qmmf_camera_adaptor_${PLATFORM}
  pthread
  ${CMAKE_DL_LIBS}
  qmmf_memory_interface_${PLATFORM}
  qmmf_camera_interface
  qmmf_camera_metadata
  ${QMMF_UTILS_LIBRARIES}
)

if(HAVE_ANDROID_UTILS)
target_link_libraries(qmmf_camera_adaptor_${PLATFORM} log binder utils cutils hardware)
else()
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -lpthread -lrt -lm -lglib-2.0 -ldl -latomic")
target_link_libraries(qmmf_camera_adaptor_${PLATFORM} ${QMMF_CONFIG_LIBRARIES})
endif()

if(USE_LIBGBM)
target_link_libraries(qmmf_camera_adaptor_${PLATFORM} gbm)
endif()
