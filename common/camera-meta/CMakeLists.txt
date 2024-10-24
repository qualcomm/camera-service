cmake_minimum_required(VERSION 3.1)

project(qmmf_camera_metadata)

add_library(qmmf_camera_metadata SHARED ${exclude}
  ${CMAKE_CURRENT_SOURCE_DIR}/qmmf_vendor_tag_descriptor.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/qmmf_camera_metadata.cpp
)

target_include_directories(qmmf_camera_metadata
 PRIVATE ${TOP_DIRECTORY})

include_directories(${SYSROOT_INCDIR}/camx-api/camx/service)
#EXPORT_HEADERS
install(FILES
  ${TOP_DIRECTORY}/include/qmmf-sdk/qmmf_camera_metadata.h
  ${TOP_DIRECTORY}/include/qmmf-sdk/qmmf_vendor_tag_descriptor.h
  DESTINATION include/qmmf-sdk)

install(TARGETS qmmf_camera_metadata DESTINATION lib OPTIONAL)

target_link_libraries(qmmf_camera_metadata pthread ${CMAKE_DL_LIBS})
if(HAVE_ANDROID_UTILS)
target_link_libraries(qmmf_camera_metadata log binder utils cutils)
else()
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -lrt -lm -lglib-2.0 -ldl -latomic")
target_link_libraries(qmmf_camera_metadata propertyvault qmmf_utils)
endif()