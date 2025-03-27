cmake_minimum_required(VERSION 3.1)

include_directories(${SYSROOT_INCDIR})

link_directories(${SYSROOT_LIBDIR})

project(qmmf-sdk)

set(CMAKE_CXX_STANDARD 17)

set(TOP_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})

include_directories(${TOP_DIRECTORY}/include)

# Check whether MMM color format header is present.
include(CheckIncludeFile)
check_include_file("display/media/mmm_color_fmt.h" HAVE_MMM_COLOR_FMT_H)

# Check whether C2D header is present.
include(CheckIncludeFile)
check_include_file("adreno/c2d2.h" RESIZER_C2D_ENABLED)

# Check whether android utilities are present.
check_include_file("cutils/properties.h" HAVE_ANDROID_UTILS)

# Check whether JPEGEncoder header is present.
include(CheckIncludeFileCXX)
check_include_file_cxx("chicdk/chiofflinepostprocintf.h" HAVE_CHI_OFFLINE_POST_PROC_INTF_H)

# Check whether IBinder header is present.
include(CheckIncludeFileCXX)
check_include_file_cxx("binder/IBinder.h" HAVE_BINDER_H)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fexceptions -fno-short-enums -O2")
# uncomment to generate debug information
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} -Wl,-no-undefined")
set(CMAKE_SHARED_LINKER_FLAGS
    "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-no-undefined")
set(CMAKE_STATIC_LINKER_FLAGS
    "${CMAKE_STATIC_LINKER_FLAGS} -Wl,-no-undefined")

add_subdirectory(config/${TARGET_BOARD_PLATFORM})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${PLATFORM_CXX_FLAGS}")

if (RESIZER_FASTCV_ENABLED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DENABLE_RESCALER_FASTCV")
endif()

if (HAVE_BINDER_H)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHAVE_BINDER")
endif()

if (RESIZER_C2D_ENABLED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DENABLE_RESCALER_C2D")
endif()

if (RESIZER_NEON_ENABLED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DENABLE_RESCALER_NEON")
endif()

if (ENABLE_OFFLINE_JPEG)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DENABLE_OFFLINE_JPEG")
endif()

if(HAVE_ANDROID_UTILS)
add_compile_definitions(HAVE_ANDROID_UTILS)
include_directories(${SYSROOT_INCDIR}/mm-core)
else()
# required for camera headers
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DLE_CAMERA")
include_directories(${SYSROOT_INCDIR}/camx-api/camx/service)
include_directories(${SYSROOT_INCDIR}/camx-api/camx/service/hardware)
include_directories(${SYSROOT_INCDIR}/camx-api/camx/service/system)
include_directories(${SYSROOT_INCDIR}/glib-2.0)
include_directories(${SYSROOT_LIBDIR}/glib-2.0/include)
endif()

include_directories(${TOP_DIRECTORY}/include)

add_subdirectory(qmmf-server)

add_subdirectory(common/camera-meta)

add_subdirectory(recorder/src/service)

add_subdirectory(recorder/test/gtest)

add_subdirectory(recorder/src/client)

add_subdirectory(common/memory)

add_subdirectory(common/cameraadaptor/gtest)
add_subdirectory(common/cameraadaptor/test)

add_subdirectory(common/cameraadaptor)

if (RESIZER_FASTCV_ENABLED)
add_subdirectory(common/resizer-fastCV)
endif()

if (RESIZER_C2D_ENABLED)
add_subdirectory(common/resizer-c2d)
endif()

if (RESIZER_NEON_ENABLED)
add_subdirectory(common/resizer-neon)
endif()

add_subdirectory(common/propertyvault)

add_subdirectory(common/utils)

if (NOT HAVE_BINDER_H)
add_subdirectory(common/proto)
endif()
