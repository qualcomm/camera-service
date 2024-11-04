cmake_minimum_required(VERSION 3.1)

project(cam-server)

if (NOT QMMF_SERVER_ENABLED)
  set(exclude EXCLUDE_FROM_ALL)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHFR_THRESHOLD=30 -DAEC_WAIT_TIMEOUT=500000000")

add_executable(cam-server ${exclude} qmmf_server_main.cc)
if (HAVE_BINDER_H)
add_dependencies(cam-server qmmf_recorder_service)
else()
add_dependencies(cam-server qmmf_proto qmmf_recorder_service)
endif()

target_include_directories(cam-server
 PRIVATE ${TOP_DIRECTORY})

target_include_directories(cam-server
 PRIVATE $<BUILD_INTERFACE:${KERNEL_INCDIR}/usr/include>)

target_include_directories(cam-server
 PRIVATE ${TOP_DIRECTORY}/common/memory)

install(TARGETS cam-server DESTINATION bin OPTIONAL)
target_link_libraries(cam-server qmmf_recorder_service)
if(HAVE_ANDROID_UTILS)
target_link_libraries(cam-server log cutils utils binder)
else()
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -lpthread -lrt -lm -lglib-2.0 -ldl -latomic")
target_link_libraries(cam-server qmmf_propertyvault)
endif()

if (QMMF_SERVER_ENABLED)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/service/cam-server-base.service DESTINATION ${CAM_SERVER_SYSTEMD_DIR} RENAME cam-server.service)
endif()
