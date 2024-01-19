cmake_minimum_required(VERSION 3.1)

project(qmmf-server)

if (NOT QMMF_SERVER_ENABLED)
  set(exclude EXCLUDE_FROM_ALL)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHFR_THRESHOLD=30 -DAEC_WAIT_TIMEOUT=500000000")

add_executable(qmmf-server ${exclude} qmmf_server_main.cc)
add_dependencies(qmmf-server qmmf_recorder_service)

target_include_directories(qmmf-server
 PRIVATE ${TOP_DIRECTORY})

target_include_directories(qmmf-server
 PRIVATE $<BUILD_INTERFACE:${KERNEL_INCDIR}/usr/include>)

# Headers path for msm-5.4 media on qrbx210 targets.
target_include_directories(qmmf-server
 PRIVATE $<BUILD_INTERFACE:${KERNEL_INCDIR}/usr/include/vidc>)

target_include_directories(qmmf-server
 PRIVATE ${TOP_DIRECTORY}/common/memory)

install(TARGETS qmmf-server DESTINATION bin OPTIONAL)
target_link_libraries(qmmf-server qmmf_recorder_service)
if(HAVE_ANDROID_UTILS)
target_link_libraries(qmmf-server log cutils utils binder)
else()
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -lpthread -lrt -lm -lglib-2.0 -ldl -latomic")
target_link_libraries(qmmf-server propertyvault)
endif()

if (TARGET_BOARD_PLATFORM STREQUAL "apq8009")
    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/service/qmmf-server-cam-arch-v1.service DESTINATION ${QMMF_SYSTEMD_DIR} RENAME qmmf-server.service)
elseif (TARGET_BOARD_PLATFORM STREQUAL "apq8053")
    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/service/qmmf-server-cam-arch-v1.service DESTINATION ${QMMF_SYSTEMD_DIR} RENAME qmmf-server.service)
elseif (TARGET_BOARD_PLATFORM STREQUAL "qcs605")
    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/service/qmmf-server-cam-arch-v2.service DESTINATION ${QMMF_SYSTEMD_DIR} RENAME qmmf-server.service)
elseif (TARGET_BOARD_PLATFORM STREQUAL "sdmsteppe")
    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/service/qmmf-server-cam-arch-v2-usrgrp.service DESTINATION ${QMMF_SYSTEMD_DIR} RENAME qmmf-server.service)
elseif (TARGET_BOARD_PLATFORM STREQUAL "qrb5165" AND TARGET_PRODUCT_PLATFORM STREQUAL "ubuntu")
    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/service/qmmf-server-cam-arch-v2-xdg.service DESTINATION ${QMMF_SYSTEMD_DIR} RENAME qmmf-server.service)
elseif (TARGET_BOARD_PLATFORM STREQUAL "qrb5165")
    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/service/qmmf-server-cam-arch-v2-usrgrp.service DESTINATION ${QMMF_SYSTEMD_DIR} RENAME qmmf-server.service)
elseif (TARGET_BOARD_PLATFORM STREQUAL "qrbx210")
    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/service/qmmf-server-cam-arch-v2-usrgrp.service DESTINATION ${QMMF_SYSTEMD_DIR} RENAME qmmf-server.service)
elseif (TARGET_BOARD_PLATFORM STREQUAL "qcs6490" AND TARGET_PRODUCT_PLATFORM STREQUAL "ubuntu")
    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/service/qmmf-server-cam-arch-v2-usrgrp.service DESTINATION ${QMMF_SYSTEMD_DIR} RENAME qmmf-server.service)
elseif (TARGET_BOARD_PLATFORM STREQUAL "kalama")
    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/service/qmmf-server-cam-arch-v2-usrgrp.service DESTINATION ${QMMF_SYSTEMD_DIR} RENAME qmmf-server.service)
else()
    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/service/qmmf-server-base.service DESTINATION ${QMMF_SYSTEMD_DIR} RENAME qmmf-server.service)
endif()
