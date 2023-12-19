LOCAL_PATH := $(call my-dir)

QMMF_SDK_TOP_SRCDIR := $(LOCAL_PATH)/../../..

include $(QMMF_SDK_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

# Build qmmf recorder client library
# libqmmf_recorder_client.so

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(MEDIA_HAL_PATH)

LOCAL_SRC_FILES := qmmf_recorder.cc
LOCAL_SRC_FILES += qmmf_recorder_client.cc
LOCAL_SRC_FILES += qmmf_recorder_extra_param.cc

LOCAL_SHARED_LIBRARIES += libcamera_metadata libcamera_client
LOCAL_SHARED_LIBRARIES += libqmmf_camera_adaptor libbinder libqmmf_memory_interface

LOCAL_MODULE = libqmmf_recorder_client

LOCAL_EXPORT_C_INCLUDE_DIRS := $(QMMF_SDK_TOP_SRCDIR)/include

include $(BUILD_SHARED_LIBRARY)

endif # BUILD_QMMMF
