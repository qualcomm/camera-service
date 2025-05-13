/*
 * Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Changes from Qualcomm Technologies, Inc. are provided under the following license:
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#ifdef __LIBGBM__
#include <system/graphics.h>
#elif HAVE_ANDROID_UTILS
#include <hardware/gralloc.h>
#endif
#include <fcntl.h>

#ifdef HAVE_BINDER
#include "common/propertyvault/qmmf_propertyvault.h"
#endif // HAVE_BINDER
#include "qmmf_gbm_interface.h"

#ifndef LOG_TAG
#define LOG_TAG "GBM Allocator"
#endif


const std::unordered_map<int32_t, int32_t> GBMUsage::usage_flag_map_ = {
  // TODO: keep this map updated with GBM enhancements
  {IMemAllocUsage::kHwCameraZsl,          0},
  {IMemAllocUsage::kPrivateAllocUbwc,     GBM_BO_USAGE_UBWC_ALIGNED_QTI},
  {IMemAllocUsage::kPrivateAllocP010,     GBM_BO_USAGE_10BIT_QTI},
  {IMemAllocUsage::kPrivateAllocTP10,     GBM_BO_USAGE_10BIT_TP_QTI},
  {IMemAllocUsage::kPrivateIommUHeap,     GBM_BO_ALLOC_IOMMU_HEAP_QTI},
  {IMemAllocUsage::kPrivateMmHeap,        GBM_BO_ALLOC_MM_HEAP_QTI},
  {IMemAllocUsage::kPrivateUncached,      GBM_BO_USAGE_UNCACHED_QTI},
  {IMemAllocUsage::kProtected,            GBM_BO_USAGE_PROTECTED_QTI},
  {IMemAllocUsage::kSwReadOften,          GBM_BO_USAGE_CPU_READ_QTI},
  {IMemAllocUsage::kSwWriteOften,         GBM_BO_USAGE_CPU_WRITE_QTI},
  {IMemAllocUsage::kVideoEncoder,         GBM_BO_USAGE_VIDEO_ENCODER_QTI},
  {IMemAllocUsage::kHwFb,                 0},
  {IMemAllocUsage::kHwTexture,            0},
  {IMemAllocUsage::kHwRender,             GBM_BO_USAGE_HW_RENDERING_QTI},
  {IMemAllocUsage::kHwComposer,           GBM_BO_USAGE_HW_COMPOSER_QTI},
  {IMemAllocUsage::kHwCameraRead,         GBM_BO_USAGE_CAMERA_READ_QTI},
  {IMemAllocUsage::kHwCameraWrite,        GBM_BO_USAGE_CAMERA_WRITE_QTI},
#ifdef GBM_BO_USAGE_PRIVATE_HEIF
  {IMemAllocUsage::kPrivateAllocHEIF,     GBM_BO_USAGE_PRIVATE_HEIF},
#else
  {IMemAllocUsage::kPrivateAllocHEIF,     0},
#endif
  {IMemAllocUsage::kFlex2Batch,           0},
  {IMemAllocUsage::kFlex4Batch,           0},
  {IMemAllocUsage::kFlex8Batch,           0},
};

const std::unordered_map<int32_t, int32_t> GBMUsage::gralloc_usage_flag_map_ = {
  //TODO: remove when repacking to buffer_handle_t is no longer needed
  {IMemAllocUsage::kHwCameraZsl,          GRALLOC_USAGE_HW_CAMERA_ZSL},
  {IMemAllocUsage::kPrivateAllocUbwc,     GRALLOC_USAGE_PRIVATE_ALLOC_UBWC},
  {IMemAllocUsage::kPrivateAllocP010,     GRALLOC_USAGE_PRIVATE_ALLOC_10BIT},
  {IMemAllocUsage::kPrivateAllocTP10,     GRALLOC_USAGE_PRIVATE_ALLOC_10BIT},
  {IMemAllocUsage::kPrivateUncached,      GRALLOC_USAGE_PRIVATE_UNCACHED},
  {IMemAllocUsage::kProtected,            GRALLOC_USAGE_PROTECTED},
  {IMemAllocUsage::kSwReadOften,          GRALLOC_USAGE_SW_READ_OFTEN},
  {IMemAllocUsage::kSwWriteOften,         GRALLOC_USAGE_SW_WRITE_OFTEN},
  {IMemAllocUsage::kVideoEncoder,         GRALLOC_USAGE_HW_VIDEO_ENCODER},
  {IMemAllocUsage::kHwFb,                 GRALLOC_USAGE_HW_FB},
  {IMemAllocUsage::kHwTexture,            GRALLOC_USAGE_HW_TEXTURE},
  {IMemAllocUsage::kHwRender,             GRALLOC_USAGE_HW_RENDER},
  {IMemAllocUsage::kHwComposer,           GRALLOC_USAGE_HW_COMPOSER},
  {IMemAllocUsage::kHwCameraRead,         GRALLOC_USAGE_HW_CAMERA_READ},
  {IMemAllocUsage::kHwCameraWrite,        GRALLOC_USAGE_HW_CAMERA_WRITE},
  {IMemAllocUsage::kPrivateAllocHEIF,     GRALLOC_USAGE_PRIVATE_HEIF},
  {IMemAllocUsage::kFlex2Batch,           0},
  {IMemAllocUsage::kFlex4Batch,           0},
  {IMemAllocUsage::kFlex8Batch,           0},
};

GBMDevice* GBMDevice::gbm_device_obj_ = nullptr;
int32_t GBMDevice::ref_count_ = 0;
std::mutex GBMDevice::gbm_device_mutex_;

int32_t GBMUsage::ToLocal(int32_t common) const {
  int32_t local_usage = 0;
  for (auto &it : usage_flag_map_) {
    if (it.first & common) {
      local_usage |= it.second;
    }
  }
  QMMF_VERBOSE("%s: local_usage(0x%x)", __func__, local_usage);
  return local_usage;
}

int32_t GBMUsage::ToLocal(MemAllocFlags common) const {
  int32_t local_usage = 0;
  for (auto &it : usage_flag_map_) {
    if (it.first & common.flags) {
      local_usage |= it.second;
    }
  }
  QMMF_VERBOSE("%s: local_usage(0x%x)", __func__, local_usage);
  return local_usage;
}

int32_t GBMUsage::ToGralloc(MemAllocFlags common) const {
  int32_t gralloc;
  gralloc = 0;
  for (auto &it : gralloc_usage_flag_map_) {
    if (it.first & common.flags) {
      gralloc |= it.second;
    }
  }
  QMMF_VERBOSE("%s: gralloc(0x%x)", __func__, gralloc);
  return gralloc;
}

MemAllocFlags GBMUsage::ToCommon(int32_t local) const {
  MemAllocFlags common;
  common.flags = 0;
  for (auto &it : usage_flag_map_) {
    if (it.second & local) {
      common.flags |= it.first;
    }
  }
  QMMF_VERBOSE("%s: common.flags(0x%x)", __func__, common.flags);
  return common;
}

MemAllocFlags GBMUsage::GrallocToCommon(int32_t gralloc) const {
  MemAllocFlags common;
  common.flags = 0;
  for (auto &it : gralloc_usage_flag_map_) {
    // Work around because camera uses the same flag for TP10 and P010 format
    if (it.second & gralloc & GRALLOC_USAGE_PRIVATE_ALLOC_10BIT) {
      if (gralloc & GRALLOC_USAGE_PRIVATE_ALLOC_UBWC) {
        common.flags |= IMemAllocUsage::kPrivateAllocTP10;
      } else {
        common.flags |= IMemAllocUsage::kPrivateAllocP010;
      }
    } else if (it.second & gralloc) {
      common.flags |= it.first;
    }
  }
  QMMF_VERBOSE("%s: common.flags(0x%x)", __func__, common.flags);
  return common;
}

int32_t GBMUsage::LocalToGralloc(int32_t local) const {
  MemAllocFlags common = ToCommon(local);
  QMMF_VERBOSE("%s: gralloc(0x%x)", __func__, ToGralloc(common));
  return (ToGralloc(common));
}

int32_t GBMUsage::GrallocToLocal(int32_t gralloc) const {
  MemAllocFlags common = GrallocToCommon(gralloc);
  QMMF_VERBOSE("%s: local(0x%x)", __func__, ToLocal(common));
  return (ToLocal(common));
}

const std::unordered_map<uint32_t, uint32_t> GBMBuffer::to_gbm_ = {
  // TODO: keep this map updated with GBM enhancements
  {HAL_PIXEL_FORMAT_BGRA_8888,               GBM_FORMAT_BGRA8888},
  {HAL_PIXEL_FORMAT_RGB_565,                 GBM_FORMAT_RGB565},
  {HAL_PIXEL_FORMAT_RGB_888,                 GBM_FORMAT_RGB888},
  {HAL_PIXEL_FORMAT_RGBA_1010102,            GBM_FORMAT_RGBA1010102},
  {HAL_PIXEL_FORMAT_RGBA_8888,               GBM_FORMAT_RGBA8888},
  {HAL_PIXEL_FORMAT_RGBX_8888,               GBM_FORMAT_RGBX8888},

  {HAL_PIXEL_FORMAT_BLOB,                    GBM_FORMAT_BLOB},
  {HAL_PIXEL_FORMAT_RAW8,                    0},
  {HAL_PIXEL_FORMAT_RAW10,                   GBM_FORMAT_RAW10},
  {HAL_PIXEL_FORMAT_RAW12,                   GBM_FORMAT_RAW12},
  {HAL_PIXEL_FORMAT_RAW16,                   GBM_FORMAT_RAW16},

  {HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED,  GBM_FORMAT_IMPLEMENTATION_DEFINED},

  {HAL_PIXEL_FORMAT_NV12_ENCODEABLE,         GBM_FORMAT_NV12_ENCODEABLE},
  {HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS,      GBM_FORMAT_YCbCr_420_SP_VENUS},
  {HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC, GBM_FORMAT_YCbCr_420_SP_VENUS_UBWC},
  {HAL_PIXEL_FORMAT_YCbCr_422_I_10BIT,       GBM_FORMAT_YCbCr_420_P010_VENUS},
  {HAL_PIXEL_FORMAT_YCbCr_420_TP10_UBWC,     GBM_FORMAT_YCbCr_420_TP10_UBWC},

  {HAL_PIXEL_FORMAT_YCbCr_420_888,           GBM_FORMAT_YCbCr_420_888},
  {HAL_PIXEL_FORMAT_YCbCr_422_SP,            GBM_FORMAT_YCbCr_422_SP},
  {HAL_PIXEL_FORMAT_YCbCr_422_I,             GBM_FORMAT_YCrCb_422_I},
  {HAL_PIXEL_FORMAT_CbYCrY_422_I,            GBM_FORMAT_UYVY},
  {HAL_PIXEL_FORMAT_YCrCb_420_SP,            GBM_FORMAT_YCrCb_420_SP},
  {HAL_PIXEL_FORMAT_YV12,                    0},
  {HAL_PIXEL_FORMAT_YCbCr_422_888,           0},

  {HAL_PIXEL_FORMAT_NV21_ZSL,                GBM_FORMAT_NV21_ZSL},
  {HAL_PIXEL_FORMAT_NV12_HEIF,               GBM_FORMAT_NV12_HEIF},
};

const std::unordered_map<int32_t, int32_t> GBMBuffer::from_gbm_ = {
  // TODO: keep this map updated with GBM enhancements
  {GBM_FORMAT_BGRA8888,                 HAL_PIXEL_FORMAT_BGRA_8888},
  {GBM_FORMAT_RGB565,                   HAL_PIXEL_FORMAT_RGB_565},
  {GBM_FORMAT_RGB888,                   HAL_PIXEL_FORMAT_RGB_888},
  {GBM_FORMAT_RGBA1010102,              HAL_PIXEL_FORMAT_RGBA_1010102},
  {GBM_FORMAT_RGBA8888,                 HAL_PIXEL_FORMAT_RGBA_8888},
  {GBM_FORMAT_RGBX8888,                 HAL_PIXEL_FORMAT_RGBX_8888},

  {GBM_FORMAT_BLOB,                     HAL_PIXEL_FORMAT_BLOB},
  {GBM_FORMAT_RAW10,                    HAL_PIXEL_FORMAT_RAW10},
  {GBM_FORMAT_RAW12,                    HAL_PIXEL_FORMAT_RAW12},
  {GBM_FORMAT_RAW16,                    HAL_PIXEL_FORMAT_RAW16},

  {GBM_FORMAT_IMPLEMENTATION_DEFINED,   HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED},

  {GBM_FORMAT_NV12_UBWC_FLEX,           HAL_PIXEL_FORMAT_NV12_UBWC_FLEX},
  {GBM_FORMAT_NV12_UBWC_FLEX_2_BATCH,   HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_2_BATCH},
  {GBM_FORMAT_NV12_UBWC_FLEX_4_BATCH,   HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_4_BATCH},
  {GBM_FORMAT_NV12_UBWC_FLEX_8_BATCH,   HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_8_BATCH},

  {GBM_FORMAT_NV12_FLEX,                HAL_PIXEL_FORMAT_NV12_FLEX},
  {GBM_FORMAT_NV12_FLEX_2_BATCH,        HAL_PIXEL_FORMAT_NV12_FLEX_2_BATCH},
  {GBM_FORMAT_NV12_FLEX_4_BATCH,        HAL_PIXEL_FORMAT_NV12_FLEX_4_BATCH},
  {GBM_FORMAT_NV12_FLEX_8_BATCH,        HAL_PIXEL_FORMAT_NV12_FLEX_8_BATCH},

  {GBM_FORMAT_YCbCr_420_P010_FLEX,              HAL_PIXEL_FORMAT_P010_FLEX},
  {GBM_FORMAT_YCbCr_420_P010_FLEX_2_BATCH,      HAL_PIXEL_FORMAT_P010_FLEX_2_BATCH},
  {GBM_FORMAT_YCbCr_420_P010_FLEX_4_BATCH,      HAL_PIXEL_FORMAT_P010_FLEX_4_BATCH},
  {GBM_FORMAT_YCbCr_420_P010_FLEX_8_BATCH,      HAL_PIXEL_FORMAT_P010_FLEX_8_BATCH},

  {GBM_FORMAT_YCbCr_420_TP10_UBWC_FLEX,         HAL_PIXEL_FORMAT_TP10_UBWC_FLEX},
  {GBM_FORMAT_YCbCr_420_TP10_UBWC_FLEX_2_BATCH, HAL_PIXEL_FORMAT_TP10_UBWC_FLEX_2_BATCH},
  {GBM_FORMAT_YCbCr_420_TP10_UBWC_FLEX_4_BATCH, HAL_PIXEL_FORMAT_TP10_UBWC_FLEX_4_BATCH},
  {GBM_FORMAT_YCbCr_420_TP10_UBWC_FLEX_8_BATCH, HAL_PIXEL_FORMAT_TP10_UBWC_FLEX_8_BATCH},

  {GBM_FORMAT_NV12_ENCODEABLE,          HAL_PIXEL_FORMAT_NV12_ENCODEABLE},
  {GBM_FORMAT_YCbCr_420_SP_VENUS,       HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS},
  {GBM_FORMAT_YCbCr_420_SP_VENUS_UBWC,  HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC},
  {GBM_FORMAT_YCbCr_420_P010_VENUS,     HAL_PIXEL_FORMAT_YCbCr_422_I_10BIT},
  {GBM_FORMAT_YCbCr_420_TP10_UBWC,      HAL_PIXEL_FORMAT_YCbCr_420_TP10_UBWC},

  {GBM_FORMAT_YCbCr_422_SP,             HAL_PIXEL_FORMAT_YCbCr_422_SP},
  {GBM_FORMAT_YCbCr_420_888,            HAL_PIXEL_FORMAT_YCbCr_420_888},
  {GBM_FORMAT_YCrCb_420_SP,             HAL_PIXEL_FORMAT_YCrCb_420_SP},
  {GBM_FORMAT_YCrCb_422_I,              HAL_PIXEL_FORMAT_YCbCr_422_I},
  {GBM_FORMAT_UYVY,                     HAL_PIXEL_FORMAT_CbYCrY_422_I},

  {GBM_FORMAT_NV21_ZSL,                 HAL_PIXEL_FORMAT_NV21_ZSL},
  {GBM_FORMAT_NV12_HEIF,                HAL_PIXEL_FORMAT_NV12_HEIF},
};

GBMBuffer::~GBMBuffer() {

  uint32_t duplicated = 0;

  if (!imported_) {
    gbm_bo_destroy(generic_handle_);

#ifdef GBM_PERFORM_GET_FD_WITH_NEW
    gbm_perform(GBM_PERFORM_GET_FD_WITH_NEW, &duplicated);
#endif // GBM_PERFORM_GET_FD_WITH_NEW

    if (duplicated != 0) {
      // The BO FD has been duplicated, we have to close it.
      close(fd_);
    }
  }

  if (nullptr != gralloc_handle_) {
    delete gralloc_handle_;
  }
}

struct gbm_bo *GBMBuffer::GetNativeHandle() const{
  return generic_handle_;
}

void GBMBuffer::SetNativeHandle(struct gbm_bo *bo) {
  generic_handle_ = bo;
  fd_ = gbm_bo_get_fd(bo);
  imported_ = false;
}

void GBMBuffer::ImportBuffer(struct gbm_bo *bo, int fd) {
  generic_handle_ = bo;
  fd_ = fd;
  imported_ = true;
}

buffer_handle_t &GBMBuffer::RepackToGralloc()
{
  if (!gralloc_handle_) {
    private_handle_t *priv_hnd = new private_handle_t(GetFD(), GetSize(),
      (int)GBMUsage().ToGralloc(GetUsage()), 0, GetFormat(), GetWidth(),
      GetHeight());
    assert(priv_hnd != NULL);
    gralloc_handle_ = static_cast<buffer_handle_t>(priv_hnd);
  }

  return gralloc_handle_;
}

int GBMBuffer::GetUsage ()
{
  MemAllocFlags cmn = GBMUsage().ToCommon(generic_handle_->usage_flags);
  return cmn.flags;
}

int GBMBuffer::GetFD() { return fd_; }

int GBMBuffer::GetFormat() {
  int format = 0;
  uint32_t gbm_format = 0;

  assert(nullptr != generic_handle_);
  gbm_format = gbm_bo_get_format(generic_handle_);
  QMMF_VERBOSE("%s: gbm_format = 0x%x", __func__, gbm_format);

  for (auto &it : from_gbm_) {
    uint32_t first = (uint32_t)it.first;
    if ((first == gbm_format) && (first != GBM_FORMAT_NOT_DEFIEND)) {
      format = (int)it.second;
      break;
    }
  }

  if (!format) {
    QMMF_ERROR("%s: Format not found!", __func__);
  }
  return format;
}

uint32_t GBMBuffer::GetSize() {
  size_t bo_size = 0;
  int ret = gbm_perform(GBM_PERFORM_GET_BO_SIZE, generic_handle_, &bo_size);
  assert(ret == GBM_ERROR_NONE);
  return (uint32_t)bo_size;
}

uint32_t GBMBuffer::GetWidth() { return gbm_bo_get_width(generic_handle_); }

uint32_t GBMBuffer::GetHeight() { return gbm_bo_get_height(generic_handle_); }

uint32_t GBMBuffer::GetLocalFormat (int common)
{
  uint32_t gbm_format = 0;
  for (auto &it : to_gbm_) {
    if ((int)it.first == common) {
      gbm_format = it.second;
    }
  }
  return gbm_format;
}

GBMDevice* GBMDevice::CreateGBMDevice() {
  std::lock_guard<std::mutex> lk(gbm_device_mutex_);
  if (!gbm_device_obj_) {
    gbm_device_obj_ = new GBMDevice;
    if (gbm_device_obj_ == nullptr) {
      QMMF_ERROR("%s: Failed to create GBM device", __func__);
      assert(false);
    }
  }
  ref_count_++;
  QMMF_DEBUG("%s: GBM device(%p) ref count: %d", __func__, gbm_device_obj_,
             ref_count_);
  return gbm_device_obj_;
}

void GBMDevice::DestroyGBMDevice() {
  std::lock_guard<std::mutex> lk(gbm_device_mutex_);
  if (gbm_device_obj_) {
    if (ref_count_ - 1 == 0) {
      delete gbm_device_obj_;
      gbm_device_obj_ = nullptr;
    }
    ref_count_--;
  }
  QMMF_DEBUG("%s: GBM device(%p) ref count: %d", __func__, gbm_device_obj_,
             ref_count_);
}

GBMDevice::GBMDevice() {
  gbm_fd_ = open("/dev/dri/renderD128", O_RDWR);
  if (gbm_fd_ < 0) {
    QMMF_WARN("%s: Falling back to /dev/dma_heap/qcom,system \n", __func__);
    gbm_fd_ = open("/dev/dma_heap/qcom,system", O_RDONLY | O_CLOEXEC);
  }
  if (gbm_fd_ < 0) {
    QMMF_WARN("%s: Falling back to /dev/ion \n", __func__);
    gbm_fd_ = open("/dev/ion", O_RDONLY | O_CLOEXEC);
  }
  assert(gbm_fd_ >= 0);
  gbm_device_ = gbm_create_device(gbm_fd_);
  assert(gbm_device_ != nullptr);
}

GBMDevice::~GBMDevice() { gbm_device_destroy(gbm_device_); close(gbm_fd_); }

gbm_device *GBMDevice::GetDevice() const {
  return (gbm_device_);
}

// items corresponding to complex flag combinations need to be placed
// before those of simple combinations especially when
// the simple flag combination is a subset of the complex combination
// for example, item with flag (kPrivateAllocUbwc | kPrivateAllocTP10)
// must be placed ahead of item with flag (kPrivateAllocUbwc)
GBMFormatTranslateEntry GBMDevice::gbm_format_translate_table_[] = {
    // flex2 UBWC, TP10 & P010
    {
      IMemAllocUsage::kFlex2Batch |
      IMemAllocUsage::kPrivateAllocUbwc |
      IMemAllocUsage::kPrivateAllocTP10,
      GBM_FORMAT_IMPLEMENTATION_DEFINED,
      GBM_FORMAT_YCbCr_420_TP10_UBWC_FLEX_2_BATCH
    },

    {
      IMemAllocUsage::kFlex2Batch |
      IMemAllocUsage::kPrivateAllocUbwc,
      GBM_FORMAT_IMPLEMENTATION_DEFINED,
      GBM_FORMAT_NV12_UBWC_FLEX_2_BATCH
    },

    {
      IMemAllocUsage::kFlex2Batch |
      IMemAllocUsage::kPrivateAllocP010,
      GBM_FORMAT_IMPLEMENTATION_DEFINED,
      GBM_FORMAT_YCbCr_420_P010_FLEX_2_BATCH
    },

    {
      IMemAllocUsage::kFlex2Batch,
      GBM_FORMAT_IMPLEMENTATION_DEFINED,
      GBM_FORMAT_NV12_FLEX_2_BATCH
    },

    // flex4 UBWC, TP10 & P010
    {
      IMemAllocUsage::kFlex4Batch |
      IMemAllocUsage::kPrivateAllocUbwc |
      IMemAllocUsage::kPrivateAllocTP10,
      GBM_FORMAT_IMPLEMENTATION_DEFINED,
      GBM_FORMAT_YCbCr_420_TP10_UBWC_FLEX_4_BATCH
    },

    {
      IMemAllocUsage::kFlex4Batch |
      IMemAllocUsage::kPrivateAllocUbwc,
      GBM_FORMAT_IMPLEMENTATION_DEFINED,
      GBM_FORMAT_NV12_UBWC_FLEX_4_BATCH
    },

    {
      IMemAllocUsage::kFlex4Batch |
      IMemAllocUsage::kPrivateAllocP010,
      GBM_FORMAT_IMPLEMENTATION_DEFINED,
      GBM_FORMAT_YCbCr_420_P010_FLEX_4_BATCH
    },

    {
      IMemAllocUsage::kFlex4Batch,
      GBM_FORMAT_IMPLEMENTATION_DEFINED,
      GBM_FORMAT_NV12_FLEX_4_BATCH
    },

    // flex8 UBWC, TP10 & P010
    {
      IMemAllocUsage::kFlex8Batch |
      IMemAllocUsage::kPrivateAllocUbwc |
      IMemAllocUsage::kPrivateAllocTP10,
      GBM_FORMAT_IMPLEMENTATION_DEFINED,
      GBM_FORMAT_YCbCr_420_TP10_UBWC_FLEX_8_BATCH
    },

    {
      IMemAllocUsage::kFlex8Batch |
      IMemAllocUsage::kPrivateAllocUbwc,
      GBM_FORMAT_IMPLEMENTATION_DEFINED,
      GBM_FORMAT_NV12_UBWC_FLEX_8_BATCH
    },

    {
      IMemAllocUsage::kFlex8Batch |
      IMemAllocUsage::kPrivateAllocP010,
      GBM_FORMAT_IMPLEMENTATION_DEFINED,
      GBM_FORMAT_YCbCr_420_P010_FLEX_8_BATCH
    },

    {
      IMemAllocUsage::kFlex8Batch,
      GBM_FORMAT_IMPLEMENTATION_DEFINED,
      GBM_FORMAT_NV12_FLEX_8_BATCH
    },

    // flex16 UBWC, TP10 & P010
    {
      IMemAllocUsage::kFlexBatch |
      IMemAllocUsage::kPrivateAllocUbwc |
      IMemAllocUsage::kPrivateAllocTP10,
      GBM_FORMAT_IMPLEMENTATION_DEFINED,
      GBM_FORMAT_YCbCr_420_TP10_UBWC_FLEX
    },

    {
      IMemAllocUsage::kFlexBatch |
      IMemAllocUsage::kPrivateAllocUbwc,
      GBM_FORMAT_IMPLEMENTATION_DEFINED,
      GBM_FORMAT_NV12_UBWC_FLEX
    },

    {
      IMemAllocUsage::kFlexBatch |
      IMemAllocUsage::kPrivateAllocP010,
      GBM_FORMAT_IMPLEMENTATION_DEFINED,
      GBM_FORMAT_YCbCr_420_P010_FLEX
    },

    {
      IMemAllocUsage::kFlexBatch,
      GBM_FORMAT_IMPLEMENTATION_DEFINED,
      GBM_FORMAT_NV12_FLEX
    },

    // TP10
    {
      IMemAllocUsage::kPrivateAllocTP10 |
      IMemAllocUsage::kPrivateAllocUbwc,
      GBM_FORMAT_IMPLEMENTATION_DEFINED,
      GBM_FORMAT_YCbCr_420_TP10_UBWC
    },

    // UBWC
    {
      IMemAllocUsage::kPrivateAllocUbwc,
      GBM_FORMAT_IMPLEMENTATION_DEFINED,
      GBM_FORMAT_YCbCr_420_SP_VENUS_UBWC
    },

    // P010
    {
      IMemAllocUsage::kPrivateAllocP010,
      GBM_FORMAT_IMPLEMENTATION_DEFINED,
      GBM_FORMAT_YCbCr_420_P010_VENUS
    },

    // HEIF
    {
      IMemAllocUsage::kPrivateAllocHEIF,
      GBM_FORMAT_YCbCr_420_888,
      GBM_FORMAT_NV12_HEIF
    },
};

MemAllocError GBMDevice::AllocBuffer(IBufferHandle& handle, int32_t width,
#ifdef HAVE_BINDER
  int32_t height, int32_t format,
#else
  int32_t height, int32_t format, int32_t override_format,
#endif // HAVE_BINDER
  MemAllocFlags usage, uint32_t *stride)
{
  handle = new GBMBuffer;
  GBMBuffer* gbm_hnd = static_cast<GBMBuffer*>(handle);
  struct gbm_bo *bo;
  uint32_t local_usage = 0;
  uint32_t gbm_format = GBM_FORMAT_NOT_DEFIEND;

  local_usage = GBMUsage().ToLocal(usage);
  gbm_format = gbm_hnd->GetLocalFormat(format);

  uint32_t table_size =
      (sizeof(gbm_format_translate_table_) / sizeof(gbm_format_translate_table_[0]));

  for (uint32_t i = 0; i < table_size; i++) {
    if ((gbm_format == gbm_format_translate_table_[i].input_format) &&
            ((gbm_format_translate_table_[i].usage_flags == 0) ||
                (usage.Matches(gbm_format_translate_table_[i].usage_flags)))) {
        gbm_format = gbm_format_translate_table_[i].output_format;

        QMMF_INFO("%s: index = %d, flags = 0x%x input format (%x) "
            "to output format (%x)",
            __func__, i,
            gbm_format_translate_table_[i].usage_flags,
            gbm_format_translate_table_[i].input_format,
            gbm_format);

        break;
    }
  }

  if (gbm_format == GBM_FORMAT_NOT_DEFIEND) {
    QMMF_ERROR("%s: GBM Format not defined !\n", __func__);
    return MemAllocError::kAllocFail;
  }

  // TODO: to be updated in to_gbm_ map post confirmation
  // from camera team for all LE SPs
  if (gbm_format == GBM_FORMAT_YCbCr_420_888) {
    gbm_format = GBM_FORMAT_NV21_ZSL;
  }

  bo = gbm_bo_create(gbm_device_, (uint32_t)width, (uint32_t)height,
    gbm_format, local_usage);
  if (!bo) {
    QMMF_ERROR("%s: Error in Allocate\n", __func__);
    return MemAllocError::kAllocFail;
  }

  char prop[PROPERTY_VALUE_MAX];
#ifdef HAVE_BINDER
  property_get("persist.qmmf.mem.color.space", prop, "ITU_R_601");
#else
  qmmf_property_get("persist.qmmf.mem.color.space", prop, "ITU_R_601");
#endif // HAVE_BINDER
  std::string colorspace(prop);

  QMMF_INFO("%s: Using color space: %s", __func__, colorspace.c_str());

  int32_t value = 0;
#ifdef HAVE_ANDROID_UTILS
  ColorMetaData colormeta = {};
#else
  GBM_ColorMetaData colormeta = {};
#endif // HAVE_ANDROID_UTILS

  auto ret = gbm_perform(GBM_PERFORM_GET_METADATA, bo,
                         GBM_METADATA_GET_COLOR_METADATA, &colormeta);
  if (ret != GBM_ERROR_NONE) {
    QMMF_ERROR("%s: Get metadata color space failed.", __func__);
    return MemAllocError::kAllocFail;
  }

#ifdef HAVE_ANDROID_UTILS
  if (colorspace == "ITU_R_601") {
    value = GBM_METADATA_COLOR_SPACE_ITU_R_601;
    colormeta.colorPrimaries = ColorPrimaries_BT601_6_625;
    colormeta.range = Range_Full;
    colormeta.transfer = Transfer_SMPTE_170M;
    colormeta.matrixCoefficients = MatrixCoEff_BT601_6_625;
  } else if (colorspace == "ITU_R_601_FR") {
    value = GBM_METADATA_COLOR_SPACE_ITU_R_601_FR;
    colormeta.colorPrimaries = ColorPrimaries_BT601_6_525;
    colormeta.range = Range_Full;
    colormeta.transfer = Transfer_SMPTE_170M;
    colormeta.matrixCoefficients = MatrixCoEff_BT601_6_525;
  } else if (colorspace == "ITU_R_709") {
    value = GBM_METADATA_COLOR_SPACE_ITU_R_709;
    colormeta.colorPrimaries = ColorPrimaries_BT709_5;
    colormeta.range = Range_Full;
    colormeta.transfer = Transfer_sRGB;
    colormeta.matrixCoefficients = MatrixCoEff_BT709_5;
  } else {
    QMMF_ERROR("%s: Unsupported color space, using ITU_R_709", __func__);
    value = GBM_METADATA_COLOR_SPACE_ITU_R_709;
    colormeta.colorPrimaries = ColorPrimaries_BT709_5;
    colormeta.range = Range_Full;
    colormeta.transfer = Transfer_sRGB;
    colormeta.matrixCoefficients = MatrixCoEff_BT709_5;
  }
#else
  if (colorspace == "ITU_R_601") {
    value = GBM_METADATA_COLOR_SPACE_ITU_R_601;
    colormeta.colorPrimaries = GBM_ColorPrimaries_BT601_6_625;
    colormeta.range = GBM_Range_Full;
    colormeta.transfer = GBM_Transfer_SMPTE_170M;
    colormeta.matrixCoefficients = GBM_MatrixCoEff_BT601_6_625;
  } else if (colorspace == "ITU_R_601_FR") {
    value = GBM_METADATA_COLOR_SPACE_ITU_R_601_FR;
    colormeta.colorPrimaries = GBM_ColorPrimaries_BT601_6_525;
    colormeta.range = GBM_Range_Full;
    colormeta.transfer = GBM_Transfer_SMPTE_170M;
    colormeta.matrixCoefficients = GBM_MatrixCoEff_BT601_6_525;
  } else if (colorspace == "ITU_R_709") {
    value = GBM_METADATA_COLOR_SPACE_ITU_R_709;
    colormeta.colorPrimaries = GBM_ColorPrimaries_BT709_5;
    colormeta.range = GBM_Range_Full;
    colormeta.transfer = GBM_Transfer_sRGB;
    colormeta.matrixCoefficients = GBM_MatrixCoEff_BT709_5;
  } else {
    QMMF_ERROR("%s: Unsupported color space, using ITU_R_709", __func__);
    value = GBM_METADATA_COLOR_SPACE_ITU_R_709;
    colormeta.colorPrimaries = GBM_ColorPrimaries_BT709_5;
    colormeta.range = GBM_Range_Full;
    colormeta.transfer = GBM_Transfer_sRGB;
    colormeta.matrixCoefficients = GBM_MatrixCoEff_BT709_5;
  }
#endif // HAVE_ANDROID_UTILS
  ret = gbm_perform(GBM_PERFORM_SET_METADATA, bo,
                    GBM_METADATA_SET_COLOR_SPACE, (void *)&value);
  if (ret != GBM_ERROR_NONE) {
    QMMF_ERROR("%s: Set color space failed.", __func__);
    return MemAllocError::kAllocFail;
  }

  ret = gbm_perform(GBM_PERFORM_SET_METADATA, bo,
                    GBM_METADATA_SET_COLOR_METADATA, &colormeta);
  if (ret != GBM_ERROR_NONE) {
    QMMF_ERROR("%s: Set metadata color space failed.", __func__);
    return MemAllocError::kAllocFail;
  }

  ret = gbm_perform(GBM_PERFORM_GET_METADATA, bo,
                    GBM_METADATA_GET_COLOR_METADATA, &colormeta);
  if (ret != GBM_ERROR_NONE) {
    QMMF_ERROR("%s: Get metadata color space failed.", __func__);
    return MemAllocError::kAllocFail;
  }

  QMMF_INFO("%s: Color Primaries %d, Color Range %d, Gamma Transfer %d, "
      "Matrix Coefficients %d", __func__, colormeta.colorPrimaries,
      colormeta.range, colormeta.transfer, colormeta.matrixCoefficients);

  gbm_hnd->SetNativeHandle(bo);
  *stride = gbm_bo_get_stride(bo);

  size_t size = 0;
  ret = gbm_perform(GBM_PERFORM_GET_BO_SIZE, bo, &size);
  if (ret != GBM_ERROR_NONE) {
    QMMF_WARN("%s: Failed to get size", __func__);
  }

  uint32_t align_width = 0;
  ret = gbm_perform(GBM_PERFORM_GET_BO_ALIGNED_WIDTH, bo, &align_width);
  if (ret != GBM_ERROR_NONE) {
    QMMF_WARN("%s: Failed to get align width", __func__);
  }

  uint32_t align_height = 0;
  ret = gbm_perform(GBM_PERFORM_GET_BO_ALIGNED_HEIGHT, bo, &align_height);
  if (ret != GBM_ERROR_NONE) {
    QMMF_WARN("%s: Failed to get align height", __func__);
  }

  QMMF_DEBUG ("%s: GBM format: 0x%x aligned dim: %dx%d stride: %d size: %d",
    __func__, gbm_format, align_width, align_height, *stride, size);

  return MemAllocError::kAllocOk;
}

MemAllocError GBMDevice::ImportBuffer(IBufferHandle& handle,
                                      void* buffer_handle, int fd) {
  handle = new GBMBuffer;
  GBMBuffer* gbm_hnd = static_cast<GBMBuffer*>(handle);
  struct gbm_bo *bo = static_cast<struct gbm_bo *>(buffer_handle);

  gbm_hnd->ImportBuffer(bo, fd);

  return MemAllocError::kAllocOk;
}

MemAllocError GBMDevice::FreeBuffer(IBufferHandle handle) {
  delete handle;
  return MemAllocError::kAllocOk;
}

MemAllocError GBMDevice::Perform(const IBufferHandle& handle,
                                 AllocDeviceAction action, void* result) {
  GBMBuffer *bo = static_cast<GBMBuffer *>(handle);
  if (nullptr == bo) {
    return MemAllocError::kAllocFail;
  }
  switch (action) {
    case AllocDeviceAction::GetHeight: {
      *static_cast<int32_t*>(result) = gbm_bo_get_height(bo->GetNativeHandle());
      return MemAllocError::kAllocOk;
    }
    case AllocDeviceAction::GetStride: {
      *static_cast<int32_t*>(result) = gbm_bo_get_stride(bo->GetNativeHandle());
      return MemAllocError::kAllocOk;
    }
    case AllocDeviceAction::GetAlignedWidth: {
      uint32_t align_width;
      auto ret = gbm_perform(GBM_PERFORM_GET_BO_ALIGNED_WIDTH,
                             bo->GetNativeHandle(), &align_width);
      if(ret == GBM_ERROR_NONE) {
        *static_cast<int32_t*>(result) = align_width;
        return MemAllocError::kAllocOk;
      } else {
        QMMF_ERROR("%s: Get aligned width action failed.", __func__);
        return MemAllocError::kAllocFail;
      }
    }
    case AllocDeviceAction::GetAlignedHeight: {
      uint32_t align_height;
      auto ret = gbm_perform(GBM_PERFORM_GET_BO_ALIGNED_HEIGHT,
                             bo->GetNativeHandle(), &align_height);
      if (ret == GBM_ERROR_NONE) {
        *static_cast<int32_t*>(result) = align_height;
        return MemAllocError::kAllocOk;
      } else {
        QMMF_ERROR("%s: Get aligned height action failed.", __func__);
        return MemAllocError::kAllocFail;
      }
    }
    case AllocDeviceAction::GetMetaFd: {
      int32_t metafd;
      auto ret = gbm_perform(GBM_PERFORM_GET_METADATA_ION_FD,
                             bo->GetNativeHandle(), &metafd);
      if (ret == GBM_ERROR_NONE) {
        *static_cast<int32_t*>(result) = metafd;
        return MemAllocError::kAllocOk;
      } else {
        QMMF_ERROR("%s: Get meta FD action failed.", __func__);
        return MemAllocError::kAllocFail;
      }
    }
    default:
      QMMF_ERROR("%s: Unrecognized action to perform.", __func__);
      return MemAllocError::kAllocFail;
  }

}
