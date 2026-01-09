/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include "common/utils/qmmf_log.h"
#include "qmmf_memory_interface.h"

#include <camx/camxformatutilexternal.h>

using FormatUtilGetBufferSizeFn =
      int (*)(CamxPixelFormat, int, int, unsigned int*);
using FormatUtilGetPlaneTypesFn =
      int (*)(CamxPixelFormat, CamxPlaneType[], int*);
using FormatUtilGetStrideInBytesFn =
     int (*)(CamxPixelFormat, CamxPlaneType, int, int*);
using FormatUtilGetScanlineFn =
      int (*)(CamxPixelFormat, CamxPlaneType, int, int*);

class DMABufUsage : public IMemAllocUsage {
 public:
  int32_t ToLocal(int32_t common) const;
  int32_t ToLocal(MemAllocFlags common) const;
  int32_t ToGralloc(MemAllocFlags common) const;
  MemAllocFlags ToCommon(int32_t local) const;
 private:
  static const std::unordered_map<int32_t, int32_t> usage_flag_map_;
};

class DMABuffer : public IBufferInterface {
 public:
  DMABuffer() : generic_handle_(nullptr),
                fd_(-1) {};
  ~DMABuffer();
  buffer_handle_t& GetNativeHandle();
  void SetNativeHandle(int fd, uint32_t size,
                       uint32_t width, uint32_t height,
                       int format, int32_t usage,
                       int stride, int scanline);
  int GetFD() override;
  int GetFormat() override;
  uint32_t GetSize() override;
  uint32_t GetWidth() override;
  uint32_t GetHeight() override;
  uint32_t GetStride();
  uint32_t GetScanline();
 private:
  buffer_handle_t generic_handle_;
  int fd_;
};

class DMABufDevice : public IAllocDevice {
 public:
  DMABufDevice();

  ~DMABufDevice();

  MemAllocError AllocBuffer(IBufferHandle& handle, int32_t width,
                            int32_t height, int32_t format,
                            int32_t override_format,
                            MemAllocFlags usage, uint32_t* stride,
                            uint32_t colorimetry) override;

  MemAllocError ImportBuffer(IBufferHandle& handle,
                             void* buffer_handle, int fd) override
                                { return MemAllocError::kAllocOk; }

  MemAllocError FreeBuffer(IBufferHandle handle) override;

  MemAllocError Perform(const IBufferHandle& handle, AllocDeviceAction action,
                        void* result) override;

  MemAllocError MapBuffer(const IBufferHandle& handle, int32_t start_x,
                          int32_t start_y, int32_t width, int32_t height,
                          MemAllocFlags usage,void** vaddr) override
                              { return MemAllocError::kAllocOk; }

  MemAllocError UnmapBuffer(const IBufferHandle& handle) override
                              { return MemAllocError::kAllocOk; }
 private:
  bool LoadFormatUtil();
  void UnloadFormatUtil();

  int dma_dev_fd_;

  void * format_util_handle_;
  FormatUtilGetBufferSizeFn get_buffer_size_fn_;
  FormatUtilGetPlaneTypesFn get_plane_types_fn_;
  FormatUtilGetStrideInBytesFn get_stride_in_bytes_fn_;
  FormatUtilGetScanlineFn  get_scanline_fn_;
};
