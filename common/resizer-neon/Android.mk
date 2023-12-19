LOCAL_PATH := $(call my-dir)

QMMF_ALG_TOP_SRCDIR := $(LOCAL_PATH)/../..

#=======================================================================
#             Build res_neon library
#=======================================================================
include $(CLEAR_VARS)

ifeq ($(32_BIT_FLAG), true)
LOCAL_32_BIT_ONLY := true
endif

include $(QMMF_ALG_TOP_SRCDIR)/common.mk

LOCAL_CFLAGS += -fexceptions

LOCAL_SRC_FILES := res_neon.cc
LOCAL_SRC_FILES += qmmf_resizer_neon.cc

LOCAL_MODULE := libqmmf_common_resizer_neon

LOCAL_SHARED_LIBRARIES += libcamera_metadata libqmmf_memory_interface
LOCAL_SHARED_LIBRARIES += libqmmf_utils

include $(BUILD_SHARED_LIBRARY)
