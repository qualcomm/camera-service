project(qmmf_utils VERSION 1.0)

if (NOT QMMF_UTILS_ENABLED)
set(exclude EXCLUDE_FROM_ALL)
endif()

add_library(qmmf_utils SHARED ${exclude}
${CMAKE_CURRENT_SOURCE_DIR}/qmmf_condition.cc
${CMAKE_CURRENT_SOURCE_DIR}/qmmf_thread.cc
${CMAKE_CURRENT_SOURCE_DIR}/qmmf_extra_param.cc
)

target_include_directories(qmmf_utils PUBLIC ${TOP_DIRECTORY})
target_include_directories(qmmf_utils PUBLIC ${TOP_DIRECTORY}/common/memory)

set_target_properties(qmmf_utils PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
)

# Add destination of compiletime pkg-config file to PKG_CONFIG_PATH
set(ENV{PKG_CONFIG_PATH} "${CMAKE_CURRENT_BINARY_DIR}:$ENV{PKG_CONFIG_PATH}")

install(TARGETS qmmf_utils DESTINATION lib OPTIONAL)

# Configure and install pkg-config file
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/qmmf_utils.pc.in
  ${CMAKE_CURRENT_BINARY_DIR}/qmmf_utils.pc
  @ONLY
)

install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/qmmf_utils.pc
  DESTINATION lib/pkgconfig
)
target_link_libraries(qmmf_utils pthread)
if(HAVE_ANDROID_UTILS)
target_link_libraries(qmmf_utils log)
else()
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -lpthread -lrt -lm -lglib-2.0 -ldl -latomic")
endif()
