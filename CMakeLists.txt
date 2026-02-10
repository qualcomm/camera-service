cmake_minimum_required(VERSION 3.16)

project(qmmf-sdk)

include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 17)

set(BUILD_CATEGORY "ALL" CACHE STRING "Build Category")

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
if (HAVE_ANDROID_UTILS)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DANDROID")
endif()
# uncomment to generate debug information
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} -Wl,-no-undefined")
set(CMAKE_SHARED_LINKER_FLAGS
    "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-no-undefined")
set(CMAKE_STATIC_LINKER_FLAGS
    "${CMAKE_STATIC_LINKER_FLAGS} -Wl,-no-undefined")

#Add default configuration
add_subdirectory(config/common)

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

if (QMMF_SERVER_ENABLED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DQMMF_SERVER_ENABLED")
endif()

if(HAVE_ANDROID_UTILS)
add_compile_definitions(HAVE_ANDROID_UTILS)
include_directories(${CMAKE_SYSROOT}${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}/mm-core)
else()
# required for camera headers
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DLE_CAMERA")
endif()

include_directories(${TOP_DIRECTORY}/include)

# === Common libs ===
if (NOT HAVE_BINDER_H)
add_subdirectory(common/proto)
endif()

add_subdirectory(common/camera-meta)

if (NOT HAVE_ANDROID_UTILS)
add_subdirectory(common/config)
endif()

add_subdirectory(common/utils)

add_subdirectory(common/interfaces)

# === Client ===
if(RECORDER_CLIENT_ENABLED)
add_subdirectory(recorder/src/client)
endif()

# === Server ===
if(QMMF_SERVER_ENABLED)
add_subdirectory(qmmf-server)
endif()

# Using function to limit TARGET variables scope
function(build_target TARGET_SUFFIX PLATFORM CAMX_INCLUDE_DIRS)

  # Use the two-arg form to give this subdir a unique binary dir per variant.
  # Prevents CMake from reusing one build dir for different sources (avoids
  # “binary directory already used to build a source directory” errors).
  set(BINROOT "${CMAKE_BINARY_DIR}/${PLATFORM}")

  #Load platform specific configuration
  add_subdirectory("config/${PLATFORM}" "${BINROOT}/config/${PLATFORM}" )

  add_subdirectory("recorder/src/service" "${BINROOT}/recorder/src/service")
  add_subdirectory("common/memory" "${BINROOT}/common/memory")
  add_subdirectory("common/cameraadaptor" "${BINROOT}/common/cameraadaptor")
endfunction()

# === Server Libs ===
if(CAMERAADAPTOR_ENABLED AND RECORDER_SERVICE_ENABLED AND MEMORY_ENABLED)

  # If TARGET_BOARD_PLATFORM is set, build specifically for
  # that platform (qli 1.x)
  if(DEFINED TARGET_BOARD_PLATFORM)
    build_target("" ${TARGET_BOARD_PLATFORM} "")
  else()
    # Mainline build
    # Camx include paths are defined in common config
    build_target("_kodiak" "qcm6490" "${CAMX_KODIAK_INCLUDE_DIRS}")
    # Using lemans config as common target
    build_target("" "qcs9100" "${CAMX_TARGET_INCLUDE_DIRS}")
  endif()

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
# === Tests ===
if (NOT CAMERA_HAL1_SUPPORT AND CAMERAADAPTOR_GTEST_ENABLED)
add_subdirectory(common/cameraadaptor/gtest)
endif()

if(RECORDER_GTEST_ENABLED)
add_subdirectory(recorder/test/gtest)
endif()

if(ENABLE_CAMERA_FACTORY_TEST)
add_subdirectory(common/cameraadaptor/test)
endif()
