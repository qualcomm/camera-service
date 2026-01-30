project(qmmf_camera_metadata VERSION 1.0)

find_package(PkgConfig REQUIRED)

pkg_check_modules(CAMERA_METADATA REQUIRED camera_metadata)

add_library(qmmf_camera_metadata SHARED ${exclude}
  ${CMAKE_CURRENT_SOURCE_DIR}/qmmf_vendor_tag_descriptor.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/qmmf_camera_metadata.cpp
)

target_include_directories(qmmf_camera_metadata
 PRIVATE ${TOP_DIRECTORY}
 PUBLIC ${CAMERA_METADATA_INCLUDE_DIRS}
)

#EXPORT_HEADERS
install(FILES
  ${TOP_DIRECTORY}/include/qmmf-sdk/qmmf_camera_metadata.h
  ${TOP_DIRECTORY}/include/qmmf-sdk/qmmf_vendor_tag_descriptor.h
  DESTINATION include/qmmf-sdk)

set_target_properties(qmmf_camera_metadata PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
)

install(TARGETS qmmf_camera_metadata DESTINATION lib OPTIONAL)

# Configure and install pkg-config file
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/qmmf_camera_metadata.pc.in
  ${CMAKE_CURRENT_BINARY_DIR}/qmmf_camera_metadata.pc
  @ONLY
)

install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/qmmf_camera_metadata.pc
  DESTINATION lib/pkgconfig
)

target_link_libraries(qmmf_camera_metadata pthread ${CMAKE_DL_LIBS})
if(HAVE_ANDROID_UTILS)
target_link_libraries(qmmf_camera_metadata log binder utils cutils)
else()
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -lrt -lm -lglib-2.0 -ldl -latomic")
target_link_libraries(qmmf_camera_metadata qmmf_utils)
endif()
