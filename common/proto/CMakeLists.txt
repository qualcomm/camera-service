cmake_minimum_required(VERSION 3.1)

project(qmmf_proto VERSION 1.0)

find_package(Protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIR})

add_library(qmmf_proto SHARED ${exclude} qmmf.pb.cc)

protobuf_generate(TARGET qmmf_proto
    PROTOC_OUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}
    PROTOS qmmf.proto)

set_target_properties(qmmf_proto PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
  )

install(TARGETS qmmf_proto DESTINATION lib OPTIONAL)

target_link_libraries(qmmf_proto ${PROTOBUF_LIBRARIES})