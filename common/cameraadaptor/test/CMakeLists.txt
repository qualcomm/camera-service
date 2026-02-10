project(camera_factory_test)

set(CMAKE_CXX_STANDARD 14)

add_executable(camera_factory_test camera_factory_test.cc)
add_dependencies(camera_factory_test qmmf_camera_adaptor)

set_target_properties(camera_factory_test PROPERTIES
  OUTPUT_NAME ${PROJECT_NAME}${TARGET_SUFFIX}
)

target_include_directories(camera_factory_test
 PRIVATE ${TOP_DIRECTORY})

target_include_directories(camera_factory_test
 PRIVATE ${TOP_DIRECTORY}/common/cameraadaptor)

target_include_directories(camera_factory_test
 PRIVATE ${TOP_DIRECTORY}/common/memory)

install(TARGETS camera_factory_test DESTINATION bin)

target_link_libraries(camera_factory_test qmmf_camera_adaptor_${PLATFORM} qmmf_camera_interface)
