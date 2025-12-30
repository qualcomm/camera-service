project(qmmf_camera_adaptor VERSION 1.0)

if (NOT CAMERAADAPTOR_ENABLED)
set(exclude EXCLUDE_FROM_ALL)
endif()

add_library(qmmf_camera_adaptor SHARED ${exclude}
${CMAKE_CURRENT_SOURCE_DIR}/qmmf_camera3_device_client.cc
${CMAKE_CURRENT_SOURCE_DIR}/qmmf_camera3_monitor.cc
${CMAKE_CURRENT_SOURCE_DIR}/qmmf_camera3_request_handler.cc
${CMAKE_CURRENT_SOURCE_DIR}/qmmf_camera3_prepare_handler.cc
${CMAKE_CURRENT_SOURCE_DIR}/qmmf_camera3_stream.cc
${CMAKE_CURRENT_SOURCE_DIR}/qmmf_camera3_utils.cc
${CMAKE_CURRENT_SOURCE_DIR}/qmmf_camera3_smooth_zoom.cc
)

if (HAVE_BINDER_H)
add_dependencies(qmmf_camera_adaptor qmmf_utils)
else()
add_dependencies(qmmf_camera_adaptor qmmf_proto qmmf_utils)
endif()


target_include_directories(qmmf_camera_adaptor
 PRIVATE ${TOP_DIRECTORY})

target_include_directories(qmmf_camera_adaptor
 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_include_directories(qmmf_camera_adaptor
 PRIVATE ${TOP_DIRECTORY}/common/memory)

target_compile_definitions(qmmf_camera_adaptor PRIVATE
  $<$<BOOL:${HAVE_MMM_COLOR_FMT_H}>:HAVE_MMM_COLOR_FMT_H>
)

set_target_properties(qmmf_camera_adaptor PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
)

install(TARGETS qmmf_camera_adaptor DESTINATION lib OPTIONAL)
target_link_libraries(qmmf_camera_adaptor qmmf_camera_metadata camera_metadata qmmf_utils)
target_link_libraries(qmmf_camera_adaptor pthread ${CMAKE_DL_LIBS} qmmf_memory_interface qmmf_camera_interfaces)
if(HAVE_ANDROID_UTILS)
target_link_libraries(qmmf_camera_adaptor log binder utils cutils hardware)
else()
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -lpthread -lrt -lm -lglib-2.0 -ldl -latomic")
target_link_libraries(qmmf_camera_adaptor qmmf_config camera_hardware)
endif()

if(USE_LIBGBM)
target_link_libraries(qmmf_camera_adaptor gbm)
endif()
