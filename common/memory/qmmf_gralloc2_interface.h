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
#include "qmmf_memory_interface.h"

class Gralloc2Buffer : public IBufferInterface {
 public:
  Gralloc2Buffer(){};
  ~Gralloc2Buffer(){};
  generic_handle_t& GetNativeHandle() override;
  int GetFD() override;
  int GetFormat() override;
  uint32_t GetSize() override;
  uint32_t GetWidth() override;
  uint32_t GetHeight() override;

 private:
  buffer_handle_t generic_handle_;
};

class Gralloc2Device : public IAllocDevice {
 public:
  Gralloc2Device();

  ~Gralloc2Device();

  mem_alloc_device GetDevice() const;

  mem_alloc_error AllocBuffer(IBufferHandle& handle, int32_t width,
                              int32_t height, int32_t format,
                              MemAllocFlags usage, uint32_t* stride) override;

  MemAllocError ImportBuffer(IBufferHandle& handle,
                             void* buffer_handle, int fd) override;

  mem_alloc_error FreeBuffer(IBufferHandle handle) override;

  MemAllocError Perform(const IBufferHandle& handle, AllocDeviceAction action,
                        void* result) override;

 private:
  mem_alloc_device gralloc_device_;
};
