/*
 * Copyright (c) 2018, The Linux Foundation. All rights reserved.
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

#include "qmmf_gralloc_interface.h"
#include <dlfcn.h>
#include "common/cameraadaptor/qmmf_camera3_utils.h"

using namespace qmmf;

const std::unordered_map<int32_t, int32_t> GrallocUsage::usage_flag_map_ = {
  {IMemAllocUsage::kHwCameraZsl,          GRALLOC_USAGE_HW_CAMERA_ZSL},
  {IMemAllocUsage::kPrivateAllocUbwc,     GRALLOC_USAGE_PRIVATE_ALLOC_UBWC},
  {IMemAllocUsage::kPrivateAllocP010,     GRALLOC_USAGE_PRIVATE_ALLOC_10BIT},
  {IMemAllocUsage::kPrivateAllocTP10,     GRALLOC_USAGE_PRIVATE_ALLOC_10BIT},
  {IMemAllocUsage::kPrivateIommUHeap,     GRALLOC_USAGE_PRIVATE_IOMMU_HEAP},
  {IMemAllocUsage::kPrivateMmHeap,        GRALLOC_USAGE_PRIVATE_MM_HEAP},
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
  {IMemAllocUsage::kHwCameraWrite,        GRALLOC_USAGE_HW_CAMERA_WRITE}};

int32_t GrallocUsage::ToLocal(int32_t common) const {
  int32_t local_usage = 0;
  for (auto &it : usage_flag_map_) {
    if (it.first & common) {
      local_usage |= it.second;
    }
  }
  return local_usage;
}

int32_t GrallocUsage::ToLocal(MemAllocFlags common) const {
  int32_t local_usage = 0;
  for (auto &it : usage_flag_map_) {
    if (it.first & common.flags) {
      local_usage |= it.second;
    }
  }
  return local_usage;
}

int32_t GrallocUsage::ToGralloc(MemAllocFlags common) const {
  return ToLocal(common);
}

MemAllocFlags GrallocUsage::ToCommon(int32_t local) const {
  MemAllocFlags common;
  common.flags = 0;
  for (auto &it : usage_flag_map_) {
    if (it.second & local) {
      common.flags |= it.first;
    }
  }
  return common;
}

buffer_handle_t &GrallocBuffer::GetNativeHandle() { return generic_handle_; }

int GrallocBuffer::GetFD() {
  return (static_cast<const private_handle_t *>(generic_handle_))->fd;
}

int GrallocBuffer::GetFormat() {
  return (static_cast<const private_handle_t *>(generic_handle_))->format;
}
uint32_t GrallocBuffer::GetSize() {
  return (static_cast<const private_handle_t *>(generic_handle_))->size;
}
uint32_t GrallocBuffer::GetWidth() {
  return (static_cast<const private_handle_t *>(generic_handle_))
      ->unaligned_width;
}
uint32_t GrallocBuffer::GetHeight() {
  return (static_cast<const private_handle_t *>(generic_handle_))
      ->unaligned_height;
}

MemAllocError GrallocDevice::AllocBuffer(IBufferHandle& handle, int32_t width,
                                         int32_t height, int32_t format,
                                         MemAllocFlags usage,
                                         uint32_t *stride, uint32_t colorimetry) {
  assert(width && height);
  int32_t local_usage = GrallocUsage().ToLocal(usage);
  // Filter out any usage bits that should not be passed
  // to the Gralloc module.
  local_usage &= GRALLOC_USAGE_ALLOC_MASK;

  int32_t buf_stride;
  GrallocBuffer *b = new GrallocBuffer;
  handle = b;
  int32_t res =
      gralloc_device_->alloc(gralloc_device_, width, height, format,
                             local_usage, &b->GetNativeHandle(), &buf_stride);
  if (0 != res) {
    QMMF_ERROR("%s: Unable to allocate Gralloc buffer: %d\n", __func__, res);
    delete b;
    handle = nullptr;
    return MemAllocError::kAllocFail;
  }
  *stride = static_cast<uint32_t>(buf_stride);

  return MemAllocError::kAllocOk;
}

MemAllocError GrallocDevice::ImportBuffer(IBufferHandle& handle,
                                          void* buffer_handle, int fd) {
  QMMF_ERROR("%s: Not implemented", __func__);
  assert(0);
  return MemAllocError::kAllocOk;
}

MemAllocError GrallocDevice::FreeBuffer(IBufferHandle handle) {
  if (nullptr != handle) {
    GrallocBuffer *b = static_cast<GrallocBuffer *>(handle);
    assert(b != nullptr);
    int32_t res = gralloc_device_->free(gralloc_device_, b->GetNativeHandle());
    if (0 != res) {
      QMMF_ERROR("%s: Error in Free\n", __func__);
      return MemAllocError::kAllocFail;
    }
    delete handle;
    handle = nullptr;
  }
  return MemAllocError::kAllocOk;
}

MemAllocError GrallocDevice::MapBuffer(const IBufferHandle& handle,
                                       int32_t start_x, int32_t start_y,
                                       int32_t width, int32_t height,
                                       MemAllocFlags usage, void **vaddr) {
  GrallocBuffer *b = static_cast<GrallocBuffer *>(handle);
  assert(b != nullptr);
  int32_t local_usage = GrallocUsage().ToLocal(usage);

  const struct private_handle_t *priv_handle =
      static_cast<const private_handle_t *>(b->GetNativeHandle());
  gralloc_module_t const *mapper = reinterpret_cast<gralloc_module_t const *>(
      gralloc_device_->common.module);
  auto ret = mapper->lock(mapper, priv_handle, local_usage, start_x, start_y,
                          width, height, vaddr);
  if (0 != ret) return MemAllocError::kAllocFail;
  return MemAllocError::kAllocOk;
}

MemAllocError GrallocDevice::UnmapBuffer(const IBufferHandle& handle) {
  GrallocBuffer *b = static_cast<GrallocBuffer *>(handle);
  const struct private_handle_t *priv_handle =
      static_cast<const private_handle_t *>(b->GetNativeHandle());
  gralloc_module_t const *mapper = reinterpret_cast<gralloc_module_t const *>(
      gralloc_device_->common.module);
  auto ret = mapper->unlock(mapper, priv_handle);
  if (0 != ret) return MemAllocError::kAllocFail;
  return MemAllocError::kAllocOk;
}

MemAllocError GrallocDevice::Perform(const IBufferHandle& handle,
                                     AllocDeviceAction action, void* result) {
  GrallocBuffer *b = static_cast<GrallocBuffer *>(handle);
  if (nullptr == b) {
    return MemAllocError::kAllocFail;
  }
  int32_t stride, height;
  const struct private_handle_t *priv_handle =
          static_cast<const private_handle_t *>(b->GetNativeHandle());
  gralloc_module_t const *mapper = reinterpret_cast<gralloc_module_t const *>(
          gralloc_device_->common.module);
  int32_t res = mapper->perform(mapper,
      GRALLOC_MODULE_PERFORM_GET_CUSTOM_STRIDE_AND_HEIGHT_FROM_HANDLE,
      priv_handle, &stride, &height);
   if (0 != res) {
     QMMF_ERROR("%s: Error in querying stride & height: %d\n", __func__, res);
     return MemAllocError::kAllocFail;
   }
  switch (action) {
    case AllocDeviceAction::GetHeight:
      *static_cast<int32_t*>(result) = height;
      return MemAllocError::kAllocOk;
    case AllocDeviceAction::GetStride:
      *static_cast<int32_t*>(result) = stride;
      return MemAllocError::kAllocOk;
    case AllocDeviceAction::GetAlignedHeight:
      *static_cast<int32_t*>(result) = height;
      return MemAllocError::kAllocOk;
    case AllocDeviceAction::GetAlignedWidth:
      *static_cast<int32_t*>(result) = stride;
      return MemAllocError::kAllocOk;
    default:
      QMMF_ERROR("%s: Unrecognized action to perform.", __func__);
      return MemAllocError::kAllocFail;
  }

}

alloc_device_t *GrallocDevice::GetDevice() const {
  return gralloc_device_;
}

GrallocDevice::GrallocDevice() {
  status_t ret = 0;

  ret = hw_get_module(GRALLOC_HARDWARE_MODULE_ID, &hw_module_);
  assert((0 == ret) && (nullptr != hw_module_));

  hw_module_->methods->open(hw_module_, GRALLOC_HARDWARE_GPU0,
                            (struct hw_device_t **)&gralloc_device_);
  assert(nullptr != gralloc_device_);
}

GrallocDevice::~GrallocDevice() {
  gralloc_device_->common.close(&gralloc_device_->common);
  dlclose(hw_module_->dso);
}
