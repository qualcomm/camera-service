add_library(qmmf_camera_interface INTERFACE)

target_include_directories(qmmf_camera_interface
 INTERFACE
 ${CMAKE_CURRENT_SOURCE_DIR}
)

add_library(qmmf_recorder_service_interface INTERFACE)

target_include_directories(qmmf_recorder_service_interface
 INTERFACE
 ${CMAKE_CURRENT_SOURCE_DIR}
)
