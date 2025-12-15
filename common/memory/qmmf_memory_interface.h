/*
 * Copyright (c) 2018, 2019, The Linux Foundation. All rights reserved.
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

#pragma once

#include "qmmf_memory_interface_defs.h"

#include <string>
#ifdef TARGET_USES_GRALLOC1
#include <grallocusage/GrallocUsageConversion.h>
#include <libgralloc1/gralloc_priv.h>
#elif USE_LIBGBM
#include <gbm.h>
#include <gbm_priv.h>

#define GBM_FORMAT_NOT_DEFIEND                          0

#ifndef GBM_FORMAT_NV12_UBWC_FLEX
#define GBM_FORMAT_NV12_UBWC_FLEX                       GBM_FORMAT_NOT_DEFIEND
#endif

#ifndef GBM_FORMAT_NV12_UBWC_FLEX_2_BATCH
#define GBM_FORMAT_NV12_UBWC_FLEX_2_BATCH               GBM_FORMAT_NOT_DEFIEND
#endif

#ifndef GBM_FORMAT_NV12_UBWC_FLEX_4_BATCH
#define GBM_FORMAT_NV12_UBWC_FLEX_4_BATCH               GBM_FORMAT_NOT_DEFIEND
#endif

#ifndef GBM_FORMAT_NV12_UBWC_FLEX_8_BATCH
#define GBM_FORMAT_NV12_UBWC_FLEX_8_BATCH               GBM_FORMAT_NOT_DEFIEND
#endif

#ifndef GBM_FORMAT_NV12_FLEX
#define GBM_FORMAT_NV12_FLEX                            GBM_FORMAT_NOT_DEFIEND
#endif

#ifndef GBM_FORMAT_NV12_FLEX_2_BATCH
#define GBM_FORMAT_NV12_FLEX_2_BATCH                    GBM_FORMAT_NOT_DEFIEND
#endif

#ifndef GBM_FORMAT_NV12_FLEX_4_BATCH
#define GBM_FORMAT_NV12_FLEX_4_BATCH                    GBM_FORMAT_NOT_DEFIEND
#endif

#ifndef GBM_FORMAT_NV12_FLEX_8_BATCH
#define GBM_FORMAT_NV12_FLEX_8_BATCH                    GBM_FORMAT_NOT_DEFIEND
#endif

#ifndef GBM_FORMAT_YCbCr_420_P010_FLEX
#define GBM_FORMAT_YCbCr_420_P010_FLEX                  GBM_FORMAT_NOT_DEFIEND
#endif

#ifndef GBM_FORMAT_YCbCr_420_P010_FLEX_2_BATCH
#define GBM_FORMAT_YCbCr_420_P010_FLEX_2_BATCH          GBM_FORMAT_NOT_DEFIEND
#endif

#ifndef GBM_FORMAT_YCbCr_420_P010_FLEX_4_BATCH
#define GBM_FORMAT_YCbCr_420_P010_FLEX_4_BATCH          GBM_FORMAT_NOT_DEFIEND
#endif

#ifndef GBM_FORMAT_YCbCr_420_P010_FLEX_8_BATCH
#define GBM_FORMAT_YCbCr_420_P010_FLEX_8_BATCH          GBM_FORMAT_NOT_DEFIEND
#endif

#ifndef GBM_FORMAT_YCbCr_420_TP10_UBWC_FLEX
#define GBM_FORMAT_YCbCr_420_TP10_UBWC_FLEX             GBM_FORMAT_NOT_DEFIEND
#endif

#ifndef GBM_FORMAT_YCbCr_420_TP10_UBWC_FLEX_2_BATCH
#define GBM_FORMAT_YCbCr_420_TP10_UBWC_FLEX_2_BATCH     GBM_FORMAT_NOT_DEFIEND
#endif

#ifndef GBM_FORMAT_YCbCr_420_TP10_UBWC_FLEX_4_BATCH
#define GBM_FORMAT_YCbCr_420_TP10_UBWC_FLEX_4_BATCH     GBM_FORMAT_NOT_DEFIEND
#endif

#ifndef GBM_FORMAT_YCbCr_420_TP10_UBWC_FLEX_8_BATCH
#define GBM_FORMAT_YCbCr_420_TP10_UBWC_FLEX_8_BATCH     GBM_FORMAT_NOT_DEFIEND
#endif

#ifdef __LIBGBM__
#include <hardware/camera.h>
#else
#include <system/window.h>
#endif
#elif defined(HAVE_BINDER) && !defined(USE_LIBGBM)
#include <qcom/display/gralloc_priv.h>
#else
#include <hardware/graphics.h>
#include <hardware/native_handle.h>
#endif
#include <unordered_map>

// todo: add and move to platform specific header
#define HAL_PIXEL_FORMAT_RAW8                    0x123
#define HAL_PIXEL_FORMAT_NV12_ENCODEABLE         0x102
#define HAL_PIXEL_FORMAT_NV21_ZSL                0x113
#define HAL_PIXEL_FORMAT_NV12_HEIF               0x00000116 // HEIF video YUV420 format
#define HAL_PIXEL_FORMAT_CbYCrY_422_I            0x120
#define HAL_PIXEL_FORMAT_NV12_UBWC_FLEX          0x126
#define HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_2_BATCH  0x128
#define HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_4_BATCH  0x129
#define HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_8_BATCH  0x130
#define HAL_PIXEL_FORMAT_NV12_FLEX               0x125
#define HAL_PIXEL_FORMAT_NV12_FLEX_2_BATCH       0x140
#define HAL_PIXEL_FORMAT_NV12_FLEX_4_BATCH       0x141
#define HAL_PIXEL_FORMAT_NV12_FLEX_8_BATCH       0x142
#define HAL_PIXEL_FORMAT_P010_FLEX               0x143
#define HAL_PIXEL_FORMAT_P010_FLEX_2_BATCH       0x144
#define HAL_PIXEL_FORMAT_P010_FLEX_4_BATCH       0x145
#define HAL_PIXEL_FORMAT_P010_FLEX_8_BATCH       0x146
#define HAL_PIXEL_FORMAT_TP10_UBWC_FLEX          0x147
#define HAL_PIXEL_FORMAT_TP10_UBWC_FLEX_2_BATCH  0x148
#define HAL_PIXEL_FORMAT_TP10_UBWC_FLEX_4_BATCH  0x149
#define HAL_PIXEL_FORMAT_TP10_UBWC_FLEX_8_BATCH  0x14a
#ifndef HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS
#define HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS      0x7FA30C04
#endif //HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS
#define HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC 0x7FA30C06
#define HAL_PIXEL_FORMAT_YCbCr_422_I_10BIT       0x4C595559
#define HAL_PIXEL_FORMAT_YCbCr_420_TP10_UBWC     0x7FA30C09

#define GRALLOC_USAGE_PROTECTED                  0x00004000
#define GRALLOC_USAGE_SW_READ_OFTEN              0x00000003
#define GRALLOC_USAGE_SW_WRITE_OFTEN             0x00000030
#define GRALLOC_USAGE_HW_FB                      0x00001000
#define GRALLOC_USAGE_HW_CAMERA_ZSL              0x00060000
#define GRALLOC_USAGE_HW_TEXTURE                 0x00000100
#define GRALLOC_USAGE_HW_RENDER                  0x00000200
#define GRALLOC_USAGE_HW_COMPOSER                0x00000800
#define GRALLOC_USAGE_HW_VIDEO_ENCODER           0x00010000
#define GRALLOC_USAGE_HW_CAMERA_WRITE            0x00020000
#define GRALLOC_USAGE_HW_CAMERA_READ             0x00040000
#define GRALLOC_USAGE_PRIVATE_HEIF               0x08000000
#define GRALLOC_USAGE_PRIVATE_ALLOC_UBWC         0x10000000 // GRALLOC_USAGE_PRIVATE_0
#define GRALLOC_USAGE_PRIVATE_ALLOC_10BIT        0x40000000 // GRALLOC_USAGE_PRIVATE_2
#define GRALLOC_USAGE_PRIVATE_UNCACHED           0x02000000
#define GRALLOC_USAGE_PRIVATE_IOMMU_HEAP         0x0
#define GRALLOC_USAGE_PRIVATE_MM_HEAP            0x0
#define GRALLOC_USAGE_PRIVATE_SNAPSHOT           0x00400000

#ifdef __LIBGBM__
struct private_handle_t : public gbm_bo {
#else
struct private_handle_t : public native_handle {
#endif
  enum {
      PRIV_FLAGS_FRAMEBUFFER = 0x00000001,
      PRIV_FLAGS_VIDEO_ENCODER = 0x00010000
  };

  int fd;
  int flags;
  unsigned int  size;
  unsigned int  offset;
  int bufferType;
  int format;
  int width;   // holds aligned width of the actual buffer allocated
  int height;  // holds aligned height of the  actual buffer allocated
  int unaligned_width;   // holds width client asked to allocate
  int unaligned_height;  // holds height client asked to allocate

  static const int sNumFds = 2;
  static inline int sNumInts() {
#ifdef __LIBGBM__
      return (((sizeof(private_handle_t) - sizeof(struct gbm_bo*)) /
              sizeof(int)) - sNumFds);
#else
      return (((sizeof(private_handle_t) - sizeof(native_handle_t)) /
              sizeof(int)) - sNumFds);
#endif
  }

  private_handle_t(int fd, unsigned int size, int flags, int bufferType,
      int format, int width, int height) :
      fd(fd), flags(flags), size(size), offset(0), bufferType(bufferType),
      format(format), width(width), height(height), unaligned_width(width),
      unaligned_height(height) {
#ifndef __LIBGBM__
    version = (int) sizeof(native_handle);
    numInts = sNumInts();
    numFds = sNumFds;
#endif
  };

  ~private_handle_t() {
  };
};

/* APIs to get allocator dependent handles and devices - only if their use
   cannot be avoided (for interfacing outside QMMF)
 */

// Support for code with hard dependency to native handles.
#ifdef TARGET_USES_GRALLOC1
buffer_handle_t &GetAllocBufferHandle(const IBufferHandle &handle);
gralloc1_device_t *GetAllocDeviceHandle(const IAllocDevice &handle);
#elif TARGET_USES_GRALLOC2
buffer_handle_t &GetAllocBufferHandle(const IBufferHandle &handle);
gralloc2_device_t *GetAllocDeviceHandle(const IAllocDevice &handle);
#elif USE_LIBGBM
struct gbm_bo *GetAllocBufferHandle(const IBufferHandle &handle);
struct gbm_device *GetAllocDeviceHandle(const IAllocDevice &handle);
buffer_handle_t &GetGrallocBufferHandle(const IBufferHandle &handle);
#elif defined(HAVE_BINDER) && !defined(USE_LIBGBM)
buffer_handle_t &GetAllocBufferHandle(const IBufferHandle &handle);
alloc_device_t *GetAllocDeviceHandle(const IAllocDevice &handle);
#else
buffer_handle_t &GetAllocBufferHandle(const IBufferHandle &handle);
#endif
