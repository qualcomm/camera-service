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

#include <qcom/display/gralloc_priv.h>
#include "qmmf_memory_interface.h"

class GrallocUsage : public IMemAllocUsage {
 public:
  int32_t ToLocal(int32_t common) const;
  int32_t ToLocal(MemAllocFlags common) const;
  int32_t ToGralloc(MemAllocFlags common) const;
  MemAllocFlags ToCommon(int32_t local) const;

 private:
  static const std::unordered_map<int32_t, int32_t> usage_flag_map_;
};

class GrallocBuffer : public IBufferInterface {
 public:
  GrallocBuffer(){};
  ~GrallocBuffer(){};
  buffer_handle_t& GetNativeHandle();
  int GetFD() override;
  int GetFormat() override;
  uint32_t GetSize() override;
  uint32_t GetWidth() override;
  uint32_t GetHeight() override;

 private:
  buffer_handle_t generic_handle_;
};

class GrallocDevice : public IAllocDevice {
 public:
  GrallocDevice();

  ~GrallocDevice();

  MemAllocError AllocBuffer(IBufferHandle& handle, int32_t width,
                            int32_t height, int32_t format,
                            MemAllocFlags usage, uint32_t* stride) override;

  MemAllocError ImportBuffer(IBufferHandle& handle,
                             void* buffer_handle, int fd) override;

  MemAllocError FreeBuffer(IBufferHandle handle) override;

  MemAllocError Perform(const IBufferHandle& handle, AllocDeviceAction action,
                        void* result) override;

  MemAllocError MapBuffer(const IBufferHandle& handle, int32_t start_x,
                          int32_t start_y, int32_t width, int32_t height,
                          MemAllocFlags usage, void** vaddr) override;

  MemAllocError UnmapBuffer(const IBufferHandle& handle) override;

  alloc_device_t* GetDevice() const;

 private:
  hw_module_t const* hw_module_;
  alloc_device_t* gralloc_device_;
};
