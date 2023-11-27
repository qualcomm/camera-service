cmake_minimum_required(VERSION 3.1)

project(qmmf_proto)

find_package(Protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIR})

add_library(qmmf_proto SHARED ${exclude} qmmf.pb.cc)

protobuf_generate(TARGET qmmf_proto
    PROTOC_OUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}
    PROTOS qmmf.proto)

install(TARGETS qmmf_proto DESTINATION lib OPTIONAL)

target_link_libraries(qmmf_proto ${PROTOBUF_LIBRARIES})