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
 * Changes from Qualcomm Innovation Center are provided under the following license:
 *
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the
 * disclaimer below) provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *     * Neither the name of Qualcomm Innovation Center, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
 * GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
 * HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "qmmf_gralloc1_interface.h"

using namespace qmmf;

const std::unordered_map<int32_t, int32_t> Gralloc1Usage::usage_flag_map_ = {
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
  {IMemAllocUsage::kHwCameraWrite,        GRALLOC_USAGE_HW_CAMERA_WRITE},
  {IMemAllocUsage::kPrivateAllocHEIF,     GRALLOC_USAGE_PRIVATE_HEIF}};

int32_t Gralloc1Usage::ToLocal(int32_t common) const {
  int32_t local_usage = 0;
  for (auto &it : usage_flag_map_) {
    if (it.first & common) {
      local_usage |= it.second;
    }
  }
  return local_usage;
}

int32_t Gralloc1Usage::ToLocal(MemAllocFlags common) const {
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

MemAllocFlags Gralloc1Usage::ToCommon(int32_t local) const {
  MemAllocFlags common;
  common.flags = 0;
  for (auto &it : usage_flag_map_) {
    if (it.second & local) {
      common.flags |= it.first;
    }
  }
  return common;
}

buffer_handle_t &Gralloc1Buffer::GetNativeHandle() { return generic_handle_; }

int Gralloc1Buffer::GetFD() {
  return (static_cast<const private_handle_t *>(generic_handle_))->fd;
}

int Gralloc1Buffer::GetFormat() {
  return (static_cast<const private_handle_t *>(generic_handle_))->format;
}
uint32_t Gralloc1Buffer::GetSize() {
  return (static_cast<const private_handle_t *>(generic_handle_))->size;
}
uint32_t Gralloc1Buffer::GetWidth() {
  return (static_cast<const private_handle_t *>(generic_handle_))
      ->unaligned_width;
}
uint32_t Gralloc1Buffer::GetHeight() {
  return (static_cast<const private_handle_t *>(generic_handle_))
      ->unaligned_height;
}

MemAllocError Gralloc1Device::AllocBuffer(IBufferHandle& handle,
                                          int32_t width, int32_t height,
                                          int32_t format, MemAllocFlags usage,
                                          uint32_t *stride) {
  Gralloc1Buffer *b = new Gralloc1Buffer;
  handle = b;
  assert(nullptr != gralloc1_device_);

  int32_t res = GRALLOC1_ERROR_NONE;
  gralloc1_buffer_descriptor_t buf_desc;
  uint64_t producer_flags = 0;
  uint64_t consumer_flags = 0;
  assert(width && height);

  int32_t local_usage = Gralloc1Usage().ToLocal(usage);

  android_convertGralloc0To1Usage(static_cast<int32_t>(local_usage),
                                  &producer_flags, &consumer_flags);

  if (local_usage & GRALLOC_USAGE_PRIVATE_ALLOC_UBWC) {
    QMMF_INFO("%s: Setting UBWC producer_flags", __func__);
    //UBWC being custom format, needs to be handled seperately
    producer_flags |= GRALLOC1_PRODUCER_USAGE_PRIVATE_ALLOC_UBWC;
  }

  if (local_usage & GRALLOC_USAGE_PRIVATE_UNCACHED) {
    QMMF_INFO("%s: Setting UNCACHED producer_flags", __func__);
    producer_flags |=  GRALLOC1_PRODUCER_USAGE_PRIVATE_UNCACHED;
   }

  res = CreateDescriptor(gralloc1_device_, &buf_desc);
  if (GRALLOC1_ERROR_NONE != res) {
    QMMF_ERROR("%s: Error in CreateDescriptor\n", __func__);
    return MemAllocError::kAllocFail;
  }

  res = SetDimensions(gralloc1_device_, buf_desc, width, height);
  if (GRALLOC1_ERROR_NONE != res) {
    QMMF_ERROR("%s: Error in SetDimensions\n", __func__);
    DestroyDescriptor(gralloc1_device_, buf_desc);
    return MemAllocError::kAllocFail;
  }

  // TODO: Find a better way to handle this format
  uint32_t stream_format;
  if (HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED == format) {
    stream_format = HAL_PIXEL_FORMAT_YCbCr_420_888;
  } else {
    stream_format = format;
  }
  res = SetFormat(gralloc1_device_, buf_desc, stream_format);
  if (GRALLOC1_ERROR_NONE != res) {
    QMMF_ERROR("%s: Error in SetFormat\n", __func__);
    DestroyDescriptor(gralloc1_device_, buf_desc);
    return MemAllocError::kAllocFail;
  }

  res = SetProducerUsage(gralloc1_device_, buf_desc, producer_flags);
  if (GRALLOC1_ERROR_NONE != res) {
    QMMF_ERROR("%s: Error in SetProducerUsage\n", __func__);
    DestroyDescriptor(gralloc1_device_, buf_desc);
    return MemAllocError::kAllocFail;
  }

  res = SetConsumerUsage(gralloc1_device_, buf_desc, consumer_flags);
  if (GRALLOC1_ERROR_NONE != res) {
    QMMF_ERROR("%s: Error in SetConsumerUsage\n", __func__);
    DestroyDescriptor(gralloc1_device_, buf_desc);
    return MemAllocError::kAllocFail;
  }

  res = Allocate(gralloc1_device_, 1, &buf_desc, &b->GetNativeHandle());
  if (GRALLOC1_ERROR_NONE != res) {
    QMMF_ERROR("%s: Error in Allocate\n", __func__);
    DestroyDescriptor(gralloc1_device_, buf_desc);
    return MemAllocError::kAllocFail;
  }
  res = GetStride(gralloc1_device_, b->GetNativeHandle(), stride);
  if (GRALLOC1_ERROR_NONE != res) {
    QMMF_ERROR("%s: Error in GetStride\n", __func__);
    DestroyDescriptor(gralloc1_device_, buf_desc);
    return MemAllocError::kAllocFail;
  }

  res = DestroyDescriptor(gralloc1_device_, buf_desc);
  if (GRALLOC1_ERROR_NONE != res) {
    QMMF_ERROR("%s: Error in DestroyDescriptor\n", __func__);
    return MemAllocError::kAllocFail;
  }

  return MemAllocError::kAllocOk;
}

MemAllocError Gralloc1Device::ImportBuffer(IBufferHandle& handle,
                                           void* buffer_handle, int fd) {
  QMMF_ERROR("%s: Not implemented", __func__);
  assert(0);
  return MemAllocError::kAllocOk;
}

MemAllocError Gralloc1Device::FreeBuffer(IBufferHandle handle) {
  if (nullptr != handle) {
    Gralloc1Buffer *b = static_cast<Gralloc1Buffer *>(handle);
    assert(b != nullptr);
    int32_t res = Release(gralloc1_device_, b->GetNativeHandle());
    if (GRALLOC1_ERROR_NONE != res) {
      QMMF_ERROR("%s: Error in Release\n", __func__);
      return MemAllocError::kAllocFail;
    }
    delete handle;
    handle = nullptr;
  }
  return MemAllocError::kAllocOk;
}

MemAllocError Gralloc1Device::Perform(const IBufferHandle& handle,
                                     AllocDeviceAction action, void* result) {
  Gralloc1Buffer *b = static_cast<Gralloc1Buffer *>(handle);
  int32_t height, stride;
  assert(b != nullptr);
  const struct private_handle_t *priv_handle =
      static_cast<const private_handle_t *>(b->GetNativeHandle());
  int32_t res = Gralloc_Perform(gralloc1_device_,
              GRALLOC_MODULE_PERFORM_GET_CUSTOM_STRIDE_AND_HEIGHT_FROM_HANDLE,
              priv_handle, &stride, &height);
  if (GRALLOC1_ERROR_NONE != res) {
    QMMF_ERROR("%s: Error in Gralloc_Perform\n", __func__);
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

MemAllocError Gralloc1Device::MapBuffer(const IBufferHandle& handle,
                                        int32_t start_x, int32_t start_y,
                                        int32_t width, int32_t height,
                                        MemAllocFlags usage, void **vaddr) {
  Gralloc1Buffer *b = static_cast<Gralloc1Buffer *>(handle);
  assert(b != nullptr);
  int32_t local_usage = Gralloc1Usage().ToLocal(usage);

  const struct private_handle_t *priv_handle =
      static_cast<const private_handle_t *>(b->GetNativeHandle());
  gralloc_module_t const *mapper = reinterpret_cast<gralloc_module_t const *>(
      gralloc1_device_->common.module);
  auto ret = mapper->lock(mapper, priv_handle, local_usage, start_x, start_y,
                          width, height, vaddr);
  if (0 != ret) return MemAllocError::kAllocFail;
  return MemAllocError::kAllocOk;
}

MemAllocError Gralloc1Device::UnmapBuffer(const IBufferHandle& handle) {
  Gralloc1Buffer *b = static_cast<Gralloc1Buffer *>(handle);
  const struct private_handle_t *priv_handle =
      static_cast<const private_handle_t *>(b->GetNativeHandle());
  gralloc_module_t const *mapper = reinterpret_cast<gralloc_module_t const *>(
      gralloc1_device_->common.module);
  auto ret = mapper->unlock(mapper, priv_handle);
  if (0 != ret) return MemAllocError::kAllocFail;
  return MemAllocError::kAllocOk;
}

Gralloc1Device::Gralloc1Device() {
  hw_module_t const *module;
  status_t ret = 0;

  ret = hw_get_module(GRALLOC_HARDWARE_MODULE_ID, &module);
  assert((0 == ret) && (nullptr != module));

  int32_t res = gralloc1_open(module, &gralloc1_device_);
  if ((0 != res) || (nullptr == gralloc1_device_)) {
    QMMF_ERROR("%s: Could not open Gralloc module: %s (%d) \n", __func__,
               strerror(-res), res);
  }

  CreateDescriptor = reinterpret_cast<GRALLOC1_PFN_CREATE_DESCRIPTOR>(
      gralloc1_device_->getFunction(gralloc1_device_,
                                    GRALLOC1_FUNCTION_CREATE_DESCRIPTOR));

  DestroyDescriptor = reinterpret_cast<GRALLOC1_PFN_DESTROY_DESCRIPTOR>(
      gralloc1_device_->getFunction(gralloc1_device_,
                                    GRALLOC1_FUNCTION_DESTROY_DESCRIPTOR));

  SetDimensions = reinterpret_cast<GRALLOC1_PFN_SET_DIMENSIONS>(
      gralloc1_device_->getFunction(gralloc1_device_,
                                    GRALLOC1_FUNCTION_SET_DIMENSIONS));

  SetFormat =
      reinterpret_cast<GRALLOC1_PFN_SET_FORMAT>(gralloc1_device_->getFunction(
          gralloc1_device_, GRALLOC1_FUNCTION_SET_FORMAT));

  SetProducerUsage = reinterpret_cast<GRALLOC1_PFN_SET_PRODUCER_USAGE>(
      gralloc1_device_->getFunction(gralloc1_device_,
                                    GRALLOC1_FUNCTION_SET_PRODUCER_USAGE));

  SetConsumerUsage = reinterpret_cast<GRALLOC1_PFN_SET_CONSUMER_USAGE>(
      gralloc1_device_->getFunction(gralloc1_device_,
                                    GRALLOC1_FUNCTION_SET_CONSUMER_USAGE));

  Allocate =
      reinterpret_cast<GRALLOC1_PFN_ALLOCATE>(gralloc1_device_->getFunction(
          gralloc1_device_, GRALLOC1_FUNCTION_ALLOCATE));

  GetStride =
      reinterpret_cast<GRALLOC1_PFN_GET_STRIDE>(gralloc1_device_->getFunction(
          gralloc1_device_, GRALLOC1_FUNCTION_GET_STRIDE));

  Release =
      reinterpret_cast<GRALLOC1_PFN_RELEASE>(gralloc1_device_->getFunction(
          gralloc1_device_, GRALLOC1_FUNCTION_RELEASE));

  Lock = reinterpret_cast<GRALLOC1_PFN_LOCK>(
      gralloc1_device_->getFunction(gralloc1_device_, GRALLOC1_FUNCTION_LOCK));

  UnLock = reinterpret_cast<GRALLOC1_PFN_UNLOCK>(gralloc1_device_->getFunction(
      gralloc1_device_, GRALLOC1_FUNCTION_UNLOCK));

  Gralloc_Perform =
      reinterpret_cast<GRALLOC1_PFN_PERFORM>(gralloc1_device_->getFunction(
          gralloc1_device_, GRALLOC1_FUNCTION_PERFORM));

  assert(!((nullptr == CreateDescriptor) || (nullptr == DestroyDescriptor) ||
           (nullptr == SetDimensions) || (nullptr == SetFormat) ||
           (nullptr == SetProducerUsage) || (nullptr == SetConsumerUsage) ||
           (nullptr == Allocate) || (nullptr == GetStride) ||
           (nullptr == Release) || (nullptr == Lock) || (nullptr == UnLock) ||
           (nullptr == Gralloc_Perform)));
}

gralloc1_device_t *Gralloc1Device::GetDevice() const {
  return (gralloc1_device_);
}

Gralloc1Device::~Gralloc1Device() { gralloc1_close(gralloc1_device_); }
