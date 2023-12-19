LOCAL_PATH := $(call my-dir)

QMMF_SDK_TOP_SRCDIR := $(LOCAL_PATH)/../..

include $(QMMF_SDK_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

# Build qmmf camera hal reprocess library
# libqmmf_common_resizer_fastcv.so

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_SRC_FILES := qmmf_resizer_fastCV.cc

LOCAL_SHARED_LIBRARIES += libfastcvopt libcamera_metadata libqmmf_memory_interface

LOCAL_MODULE = libqmmf_common_resizer_fastcv

include $(BUILD_SHARED_LIBRARY)

endif # BUILD_QMMMF
