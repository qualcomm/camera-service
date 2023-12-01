cmake_minimum_required(VERSION 3.1)

include_directories(${SYSROOT_INCDIR})
include_directories(${SYSROOT_INCDIR}/mm-core)
link_directories(${SYSROOT_LIBDIR})

project(qmmf-sdk)

set(CMAKE_CXX_STANDARD 14)

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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fexceptions -fno-short-enums -O2 -DANDROID")
# uncomment to generate debug information
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} -Wl,-no-undefined")
set(CMAKE_SHARED_LINKER_FLAGS
    "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-no-undefined")
set(CMAKE_STATIC_LINKER_FLAGS
    "${CMAKE_STATIC_LINKER_FLAGS} -Wl,-no-undefined")

add_subdirectory(config/LE-${TARGET_BOARD_PLATFORM})

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

if(HAVE_ANDROID_UTILS)
add_compile_definitions(HAVE_ANDROID_UTILS)
endif()

add_subdirectory(qmmf-server)

add_subdirectory(common/camera-meta)

add_subdirectory(recorder/src/service)

add_subdirectory(recorder/test/gtest)

add_subdirectory(recorder/src/client)

add_subdirectory(common/memory)

if (NOT CAMERA_HAL1_SUPPORT)
add_subdirectory(common/cameraadaptor/gtest)

add_subdirectory(common/cameraadaptor)
endif()

if (RESIZER_FASTCV_ENABLED)
add_subdirectory(common/resizer-fastCV)
endif()

if (RESIZER_C2D_ENABLED)
add_subdirectory(common/resizer-c2d)
endif()

if (RESIZER_NEON_ENABLED)
add_subdirectory(common/resizer-neon)
endif()

add_subdirectory(common/utils)
