LOCAL_PATH := $(call my-dir)

QMMF_SDK_TOP_SRCDIR := $(LOCAL_PATH)/../..

include $(QMMF_SDK_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

# Build libqmmf_camera_adaptor.so

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(QMMF_SDK_TOP_SRCDIR)/common/utils
ifeq ($(IS_ANDROID_O_OR_ABOVE),true)
LOCAL_C_INCLUDES += $(TOP)/system/core/base/include
endif
ifeq ($(TARGET_USES_GRALLOC1),true)
LOCAL_C_INCLUDES += $(TOP)/system/core/libgrallocusage/include
endif


LOCAL_SRC_FILES := qmmf_memory_interface.cc
ifeq ($(TARGET_USES_GRALLOC1),true)
LOCAL_SRC_FILES += qmmf_gralloc1_interface.cc
else
ifeq ($(TARGET_USES_GRALLOC2),true)
LOCAL_SRC_FILES += qmmf_gralloc2_interface.cc
else
ifeq ($(TARGET_USES_GBM),true)
LOCAL_SRC_FILES += qmmf_gbm_interface.cc
else
LOCAL_SRC_FILES += qmmf_gralloc1_interface.cc
endif
endif
endif

LOCAL_SHARED_LIBRARIES += libhardware
ifeq ($(TARGET_USES_GRALLOC1), true)
LOCAL_SHARED_LIBRARIES += libgrallocusage_vendor
endif

LOCAL_MODULE = libqmmf_memory_interface
LOCAL_EXPORT_C_INCLUDE_DIRS := $(QMMF_SDK_TOP_SRCDIR)/common/memory
LOCAL_EXPORT_C_INCLUDE_DIRS += $(QMMF_SDK_TOP_SRCDIR)/include
ifeq ($(TARGET_USES_GRALLOC1),true)
LOCAL_EXPORT_C_INCLUDE_DIRS += $(TOP)/system/core/libgrallocusage/include
endif

include $(BUILD_SHARED_LIBRARY)
endif
