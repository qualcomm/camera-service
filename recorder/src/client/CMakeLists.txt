project(qmmf_recorder_client)

if (NOT RECORDER_CLIENT_ENABLED)
  set(exclude EXCLUDE_FROM_ALL)
else()
  #EXPORT_HEADERS
  install(FILES
    ${TOP_DIRECTORY}/include/qmmf-sdk/qmmf_recorder.h
    ${TOP_DIRECTORY}/include/qmmf-sdk/qmmf_recorder_params.h
    ${TOP_DIRECTORY}/include/qmmf-sdk/qmmf_recorder_extra_param.h
    ${TOP_DIRECTORY}/include/qmmf-sdk/qmmf_recorder_extra_param_tags.h
    ${TOP_DIRECTORY}/include/qmmf-sdk/qmmf_buffer.h
    ${TOP_DIRECTORY}/include/qmmf-sdk/qmmf_offline_jpeg_params.h
    ${TOP_DIRECTORY}/include/qmmf-sdk/qmmf_offline_camera_params.h
    DESTINATION include/qmmf-sdk)
endif()

add_library(qmmf_recorder_client SHARED ${exclude}
${CMAKE_CURRENT_SOURCE_DIR}/qmmf_recorder.cc
${CMAKE_CURRENT_SOURCE_DIR}/qmmf_recorder_client.cc
${CMAKE_CURRENT_SOURCE_DIR}/qmmf_recorder_extra_param.cc
)

find_package(PkgConfig)

pkg_check_modules(GBM gbm)

string(REGEX MATCH "^[0-9]+" GBM_VERSION_MAJOR ${GBM_VERSION})

target_compile_definitions(qmmf_recorder_client PRIVATE GBM_VER="${GBM_VERSION_MAJOR}")

target_include_directories(qmmf_recorder_client
 PRIVATE ${TOP_DIRECTORY})

target_include_directories(qmmf_recorder_client
 PRIVATE ${TOP_DIRECTORY}/common/memory)

set_target_properties(qmmf_recorder_client PROPERTIES
 VERSION ${PROJECT_VERSION}
 SOVERSION ${PROJECT_VERSION_MAJOR}
)

install(TARGETS qmmf_recorder_client DESTINATION lib OPTIONAL)
target_link_libraries(qmmf_recorder_client dl qmmf_camera_metadata)
if(HAVE_ANDROID_UTILS)
target_link_libraries(qmmf_recorder_client log binder utils cutils ion)
else()
target_link_libraries(qmmf_recorder_client ${PROTOBUF_LIBRARIES} qmmf_config)
endif()

if(USE_LIBGBM)
target_link_libraries(qmmf_recorder_client ${CMAKE_DL_LIBS})
endif()

if(USE_LIBGBM)
target_link_libraries(qmmf_recorder_client gbm)
endif()

if(NOT HAVE_BINDER_H)
target_link_libraries(qmmf_recorder_client qmmf_proto)
endif()
