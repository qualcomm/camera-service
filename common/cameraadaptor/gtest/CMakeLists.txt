project(qmmf_camera_adaptor_gtest)

set(CMAKE_CXX_STANDARD 17)

if (NOT CAMERAADAPTOR_GTEST_ENABLED)
set(exclude EXCLUDE_FROM_ALL)
endif()

add_executable(qmmf_camera_adaptor_gtest ${exclude} qmmf_camera_adaptor_gtest.cc)
add_dependencies(qmmf_camera_adaptor_gtest qmmf_camera_adaptor)

set_target_properties(qmmf_camera_adaptor_gtest PROPERTIES
  OUTPUT_NAME ${PROJECT_NAME}${TARGET_SUFFIX}
)

target_include_directories(qmmf_camera_adaptor_gtest
 PRIVATE ${TOP_DIRECTORY})

target_include_directories(qmmf_camera_adaptor_gtest
 PRIVATE ${TOP_DIRECTORY}/common/cameraadaptor)

target_include_directories(qmmf_camera_adaptor_gtest
 PRIVATE ${TOP_DIRECTORY}/common/memory)

install(TARGETS qmmf_camera_adaptor_gtest DESTINATION bin OPTIONAL)
target_link_libraries(qmmf_camera_adaptor_gtest gtest
  gtest_main
  qmmf_camera_adaptor
  qmmf_camera_interfaces
)
if(HAVE_ANDROID_UTILS)
target_link_libraries(qmmf_camera_adaptor_gtest log cutils utils)
endif()

add_executable(qmmf_ccamera_dual_adaptor_gtest ${exclude} qmmf_dual_camera_adaptor_gtest.cc)
add_dependencies(qmmf_ccamera_dual_adaptor_gtest qmmf_camera_adaptor)

target_include_directories(qmmf_ccamera_dual_adaptor_gtest
 PRIVATE ${TOP_DIRECTORY})

target_include_directories(qmmf_ccamera_dual_adaptor_gtest
 PRIVATE ${TOP_DIRECTORY}/common/cameraadaptor)

target_include_directories(qmmf_ccamera_dual_adaptor_gtest
 PRIVATE ${TOP_DIRECTORY}/common/memory)

install(TARGETS qmmf_ccamera_dual_adaptor_gtest DESTINATION bin OPTIONAL)
target_link_libraries(qmmf_ccamera_dual_adaptor_gtest gtest
  gtest_main
  qmmf_camera_adaptor
  qmmf_camera_interfaces
)
if(HAVE_ANDROID_UTILS)
target_link_libraries(qmmf_ccamera_dual_adaptor_gtest log cutils utils)
endif()
