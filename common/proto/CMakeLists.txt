project(qmmf_proto VERSION 1.0)

find_package(absl)
find_package(Protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIR})

add_library(qmmf_proto SHARED ${exclude} qmmf.pb.cc)

file(MD5 "${CMAKE_CURRENT_SOURCE_DIR}/qmmf.proto" PROTO_FILE_MD5)

message(STATUS "MD5(qmmf.proto) = ${PROTO_FILE_MD5}")

set(QMMF_CURRENT_INTERFACE_VER "${PROTO_FILE_MD5}" PARENT_SCOPE)

protobuf_generate(TARGET qmmf_proto
    PROTOC_OUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}
    PROTOS qmmf.proto)

# Add destination of compiletime pkg-config file to PKG_CONFIG_PATH
set(ENV{PKG_CONFIG_PATH} "${CMAKE_CURRENT_BINARY_DIR}:$ENV{PKG_CONFIG_PATH}")

# Install the generated header so parent recipes can include it
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/qmmf.pb.h
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/proto"
)

set_target_properties(qmmf_proto PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
  )

install(TARGETS qmmf_proto DESTINATION lib OPTIONAL)

# Configure and install pkg-config file
configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/qmmf_proto.pc.in
  ${CMAKE_CURRENT_BINARY_DIR}/qmmf_proto.pc
  @ONLY
)

install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/qmmf_proto.pc
  DESTINATION lib/pkgconfig
)

target_link_libraries(qmmf_proto
  ${PROTOBUF_LIBRARIES}
  $<$<TARGET_EXISTS:absl::log_internal_message>:absl::log_internal_message>
  $<$<TARGET_EXISTS:absl::log_internal_check_op>:absl::log_internal_check_op>
)
