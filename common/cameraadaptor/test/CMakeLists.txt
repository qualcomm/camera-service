project(camera_factory_test)

set(CMAKE_CXX_STANDARD 14)

add_executable(camera_factory_test camera_factory_test.cc)
add_dependencies(camera_factory_test qmmf_camera_adaptor)

target_include_directories(camera_factory_test
 PRIVATE ${TOP_DIRECTORY})

target_include_directories(camera_factory_test
 PRIVATE ${TOP_DIRECTORY}/common/cameraadaptor)

target_include_directories(camera_factory_test
 PRIVATE ${TOP_DIRECTORY}/common/memory)

install(TARGETS camera_factory_test DESTINATION bin)

target_link_libraries(camera_factory_test camera_metadata qmmf_camera_adaptor qmmf_camera_interfaces)
