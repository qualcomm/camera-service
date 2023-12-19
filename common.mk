LOCAL_CPP_EXTENSION := .cc

LOCAL_CFLAGS := -Wall -Wextra -Werror -std=c++14 -fexceptions
# TODO functions have unused input parameters
LOCAL_CFLAGS += -Wno-unused-parameter
# Suppress unused variable caused by assert only for release variant
ifeq (userdebug,$(TARGET_BUILD_VARIANT))
LOCAL_CFLAGS += -UNDEBUG
else
LOCAL_CFLAGS += -Wno-unused-variable
endif

# ANDROID version check
ANDROID_MAJOR_VERSION :=$(shell echo $(PLATFORM_VERSION) | cut -f1 -d.)
IS_ANDROID_O_OR_ABOVE :=$(shell test $(ANDROID_MAJOR_VERSION) -gt 8 -o $(ANDROID_MAJOR_VERSION) -eq 8 && echo true)
ifeq ($(IS_ANDROID_O_OR_ABOVE),true)
LOCAL_CFLAGS += -DANDROID_O_OR_ABOVE
endif #ANDROID version check

LOCAL_C_INCLUDES := $(QMMF_SDK_TOP_SRCDIR)/include
LOCAL_C_INCLUDES += $(QMMF_SDK_TOP_SRCDIR)
LOCAL_C_INCLUDES += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr/include

# Header files required for O MR1
ifeq ($(IS_ANDROID_O_OR_ABOVE),true)
LOCAL_C_INCLUDES += $(TOP)/frameworks/native/libs/nativewindow/include
LOCAL_C_INCLUDES += $(TOP)/frameworks/native/libs/nativebase/include
LOCAL_C_INCLUDES += $(TOP)/frameworks/native/libs/arect/include
endif

LOCAL_ADDITIONAL_DEPENDENCIES += $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ/usr

LOCAL_SHARED_LIBRARIES := libcutils libutils libdl liblog

LOCAL_EXPORT_C_INCLUDE_DIRS := $(QMMF_SDK_TOP_SRCDIR)/include

LOCAL_32_BIT_ONLY := true

# Enable libs/bins installation into vendor
ifeq ($(IS_ANDROID_O_OR_ABOVE),true)
LOCAL_VENDOR_MODULE := true
endif #LOCAL_VENDOR_MODULE

# Enable CAM_ARCH_V2
ifeq ($(TARGET_BOARD_PLATFORM),qcs605)
CAM_ARCH_V2 := 1
LOCAL_CFLAGS += -DCAM_ARCH_V2
endif #CAM_ARCH_V2

# Disable Op Modes
ifeq ($(TARGET_BOARD_PLATFORM),qcs605)
LOCAL_CFLAGS += -DDISABLE_OP_MODES
endif #DISABLE_OP_MODES

# Enable Gralloc1 support
ifeq ($(TARGET_USES_GRALLOC1),true)
LOCAL_CFLAGS += -DTARGET_USES_GRALLOC1
endif #TARGET_USES_GRALLOC1

# Set HFR Threshold values based on platform
ifeq ($(TARGET_BOARD_PLATFORM),qcs605)
LOCAL_CFLAGS += -DHFR_THRESHOLD=90.0f
else
LOCAL_CFLAGS += -DHFR_THRESHOLD=30.0f
endif #HFR_THRESHOLD

ifeq ($(TARGET_BOARD_PLATFORM),qcs605)
LOCAL_CFLAGS += -DUSE_FPS_IDX
endif #USE_FPS_IDX

# Jpeg Blob offset
ifeq ($(TARGET_BOARD_PLATFORM),qcs605)
LOCAL_CFLAGS += -DJPEG_BLOB_OFFSET=0
else
LOCAL_CFLAGS += -DJPEG_BLOB_OFFSET=1
endif #JPEG_BLOB_OFFSET

# AEC timeout value in ms (if not set here, default value is set in src)
ifeq ($(TARGET_BOARD_PLATFORM),qcs605)
LOCAL_CFLAGS += -DAEC_WAIT_TIMEOUT=750000000
else
LOCAL_CFLAGS += -DAEC_WAIT_TIMEOUT=500000000
endif #AEC_WAIT_TIMEOUT

# FLUSH_RESTART_NOTAVAILABLE
ifeq ($(TARGET_BOARD_PLATFORM),qcs605)
LOCAL_CFLAGS += -DFLUSH_RESTART_NOTAVAILABLE
endif #FLUSH_RESTART_NOTAVAILABLE

# Enable local QCamera3 tags support
ifeq ($(TARGET_BOARD_PLATFORM),qcs605)
LOCAL_CFLAGS += -DQCAMERA3_TAG_LOCAL_COPY
endif #QCAMERA3_TAG_LOCAL_COPY

# Disable Rescaler Colorspace
ifeq ($(TARGET_BOARD_PLATFORM),qcs605)
DISABLE_RESCALER_COLORSPACE := 1
LOCAL_CFLAGS += -DDISABLE_RESCALER_COLORSPACE
endif #DISABLE_RESCALER_COLORSPACE

# Set hal paths
ifeq ($(PRODUCT_BRAND),Things)
CAMERA_HAL_PATH := $(TOP)/hardware/qcom/camera/$(TARGET_BOARD_PLATFORM)
MEDIA_HAL_PATH := $(TOP)/hardware/qcom/media/$(TARGET_BOARD_PLATFORM)
else
CAMERA_HAL_PATH := $(TOP)/hardware/qcom/camera
MEDIA_HAL_PATH := $(TOP)/hardware/qcom/media
endif
