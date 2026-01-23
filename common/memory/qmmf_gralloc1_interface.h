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

#pragma once

#include <grallocusage/GrallocUsageConversion.h>
#include <libgralloc1/gralloc_priv.h>
#include "qmmf_memory_interface.h"
#include "common/cameraadaptor/qmmf_camera3_utils.h"

class Gralloc1Usage : public IMemAllocUsage {
 public:
  int32_t ToLocal(int32_t common) const;
  int32_t ToLocal(MemAllocFlags common) const;
  int32_t ToGralloc(MemAllocFlags common) const;
  MemAllocFlags ToCommon(int32_t local) const;

 private:
  static const std::unordered_map<int32_t, int32_t> usage_flag_map_;
};

class Gralloc1Buffer : public IBufferInterface {
 public:
  Gralloc1Buffer(){};
  ~Gralloc1Buffer(){};
  buffer_handle_t& GetNativeHandle();
  int GetFD() override;
  int GetFormat() override;
  uint32_t GetSize() override;
  uint32_t GetWidth() override;
  uint32_t GetHeight() override;

 private:
  buffer_handle_t generic_handle_;
};

class Gralloc1Device : public IAllocDevice {
 public:
  Gralloc1Device();

  ~Gralloc1Device();

  gralloc1_device_t* GetDevice() const;

  MemAllocError AllocBuffer(IBufferHandle& handle, int32_t width,
                            int32_t height, int32_t format,
                            MemAllocFlags usage, uint32_t* stride,
                            uint32_t colorimetry) override;

  MemAllocError ImportBuffer(IBufferHandle& handle,
                             void* buffer_handle, int fd) override;

  MemAllocError FreeBuffer(IBufferHandle handle) override;

  MemAllocError Perform(const IBufferHandle& handle, AllocDeviceAction action,
                        void* result) override;

  MemAllocError MapBuffer(const IBufferHandle& handle, int32_t start_x,
                            int32_t start_y, int32_t width, int32_t height,
                            MemAllocFlags usage, void** vaddr) override;

  MemAllocError UnmapBuffer(const IBufferHandle& handle) override;

 private:
  int32_t (*CreateDescriptor)(gralloc1_device_t* device,
                              gralloc1_buffer_descriptor_t* pCreatedDescriptor);

  int32_t (*DestroyDescriptor)(gralloc1_device_t* device,
                               gralloc1_buffer_descriptor_t descriptor);

  int32_t (*SetDimensions)(gralloc1_device_t* device,
                           gralloc1_buffer_descriptor_t descriptor,
                           uint32_t width, uint32_t height);

  int32_t (*SetFormat)(gralloc1_device_t* device,
                       gralloc1_buffer_descriptor_t descriptor, int32_t format);

  int32_t (*SetProducerUsage)(gralloc1_device_t* device,
                              gralloc1_buffer_descriptor_t descriptor,
                              uint64_t usage);

  int32_t (*SetConsumerUsage)(gralloc1_device_t* device,
                              gralloc1_buffer_descriptor_t descriptor,
                              uint64_t usage);

  int32_t (*Allocate)(gralloc1_device_t* device, uint32_t numDescriptors,
                      const gralloc1_buffer_descriptor_t* pDescriptors,
                      buffer_handle_t* pAllocatedBuffers);

  int32_t (*GetStride)(gralloc1_device_t* device, buffer_handle_t buffer,
                       uint32_t* pStride);

  int32_t (*Release)(gralloc1_device_t* device, buffer_handle_t buffer);

  int32_t (*Lock)(gralloc1_device_t* device, buffer_handle_t buffer,
                  uint64_t producerUsage, uint64_t consumerUsage,
                  const gralloc1_rect_t* accessRegion, void** outData,
                  int32_t acquireFence);

  int32_t (*UnLock)(gralloc1_device_t* device, buffer_handle_t buffer,
                    int32_t* outReleaseFence);

  gralloc1_error_t (*Gralloc_Perform)(gralloc1_device_t* device, int32_t operation,
                              ...);
  gralloc1_device_t* gralloc1_device_;
};
