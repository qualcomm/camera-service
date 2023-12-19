LOCAL_PATH := $(call my-dir)

QMMF_SDK_TOP_SRCDIR := $(LOCAL_PATH)/../../..

include $(QMMF_SDK_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

# Build recorder gtest application binary

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(TOP)/system/core/base/include

LOCAL_SRC_FILES := qmmf_gtest_common.cc
LOCAL_SRC_FILES += qmmf_gtest.cc

LOCAL_SHARED_LIBRARIES += libqmmf_recorder_client libqmmf_memory_interface
LOCAL_SHARED_LIBRARIES += libcamera_client

LOCAL_MODULE = qmmf_recorder_gtest

ifeq ($(LOCAL_VENDOR_MODULE),true)
LOCAL_VENDOR_MODULE := false
endif

include $(BUILD_NATIVE_TEST)

endif
