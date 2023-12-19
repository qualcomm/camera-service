LOCAL_PATH := $(call my-dir)

QMMF_SDK_TOP_SRCDIR := $(LOCAL_PATH)/../../..

include $(QMMF_SDK_TOP_SRCDIR)/build.mk

ifneq (,$(BUILD_QMMMF))

# Build qmmf recorder service library
# libqmmf_recorder_service.so

include $(CLEAR_VARS)

include $(QMMF_SDK_TOP_SRCDIR)/common.mk

LOCAL_C_INCLUDES += $(MEDIA_HAL_PATH)

LOCAL_SRC_FILES := qmmf_recorder_service.cc
LOCAL_SRC_FILES += qmmf_recorder_impl.cc
LOCAL_SRC_FILES += qmmf_remote_cb.cc
LOCAL_SRC_FILES += qmmf_camera_source.cc
LOCAL_SRC_FILES += qmmf_camera_frc.cc
LOCAL_SRC_FILES += qmmf_camera_context.cc
LOCAL_SRC_FILES += qmmf_camera_rescaler.cc

LOCAL_SHARED_LIBRARIES += libqmmf_utils
LOCAL_SHARED_LIBRARIES += libqmmf_recorder_client libqmmf_camera_adaptor
LOCAL_SHARED_LIBRARIES += libqmmf_memory_interface
LOCAL_SHARED_LIBRARIES += libcamera_client libbinder libhardware
LOCAL_SHARED_LIBRARIES += libqmmf_common_resizer_fastcv
LOCAL_SHARED_LIBRARIES += libfastcvopt
LOCAL_SHARED_LIBRARIES += libqmmf_common_resizer_c2d
LOCAL_SHARED_LIBRARIES += libqmmf_common_resizer_neon

LOCAL_MODULE = libqmmf_recorder_service

include $(BUILD_SHARED_LIBRARY)

endif # BUILD_QMMMF
