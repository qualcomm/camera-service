cmake_minimum_required(VERSION 3.1)

project(recorder-gtest)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_FILE_OFFSET_BITS=64")

if (NOT RECORDER_GTEST_ENABLED)
  set(exclude EXCLUDE_FROM_ALL)
endif()

add_executable(qmmf_recorder_gtest ${exclude}
  qmmf_gtest_common.cc
  qmmf_gtest.cc
)

add_dependencies(qmmf_recorder_gtest qmmf_recorder_client qmmf_utils)

target_include_directories(qmmf_recorder_gtest
 PRIVATE ${TOP_DIRECTORY})

target_include_directories(qmmf_recorder_gtest
 PRIVATE $<BUILD_INTERFACE:${KERNEL_INCDIR}/usr/include>)

target_include_directories(qmmf_recorder_gtest
 PRIVATE ${TOP_DIRECTORY}/recorder/test/gtest)

target_include_directories(qmmf_recorder_gtest
 PRIVATE ${TOP_DIRECTORY}/common/memory)

install(TARGETS qmmf_recorder_gtest DESTINATION bin OPTIONAL)
target_link_libraries(qmmf_recorder_gtest gtest gtest_main qmmf_camera_metadata qmmf_recorder_client)
if(HAVE_ANDROID_UTILS)
target_link_libraries(qmmf_recorder_gtest log utils cutils)
else()
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -lpthread -lrt -lm -lglib-2.0 -ldl -latomic")
target_link_libraries(qmmf_recorder_gtest qmmf_propertyvault)
endif()
if(NOT CAMERA_CLIENT_DISABLED)
target_link_libraries(qmmf_recorder_gtest camera_client)
endif()
