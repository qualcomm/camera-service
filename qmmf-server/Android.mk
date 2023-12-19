LOCAL_PATH := $(call my-dir)

QMMF_SDK_TOP_SRCDIR := $(LOCAL_PATH)/..

include $(QMMF_SDK_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

# Build qmmf-server binary

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(MEDIA_HAL_PATH)

LOCAL_SRC_FILES := qmmf_server_main.cc

LOCAL_SHARED_LIBRARIES += libqmmf_memory_interface

LOCAL_SHARED_LIBRARIES += libqmmf_recorder_service

LOCAL_SHARED_LIBRARIES += libbinder

LOCAL_MODULE = qmmf-server

include $(BUILD_EXECUTABLE)

endif # BUILD_QMMMF
