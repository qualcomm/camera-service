cmake_minimum_required(VERSION 3.1)

project(qmmf_utils)

if (NOT QMMF_UTILS_ENABLED)
set(exclude EXCLUDE_FROM_ALL)
endif()

add_library(qmmf_utils SHARED ${exclude}
${CMAKE_CURRENT_SOURCE_DIR}/qmmf_condition.cc
${CMAKE_CURRENT_SOURCE_DIR}/qmmf_thread.cc
)

target_include_directories(qmmf_utils PRIVATE ${TOP_DIRECTORY})
target_include_directories(qmmf_utils PRIVATE ${TOP_DIRECTORY}/common/memory)

install(TARGETS qmmf_utils DESTINATION lib OPTIONAL)
target_link_libraries(qmmf_utils pthread)
if(HAVE_ANDROID_UTILS)
target_link_libraries(qmmf_utils log)
else()
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -lpthread -lrt -lm -lglib-2.0 -ldl -latomic")
target_link_libraries(qmmf_utils propertyvault)
endif()