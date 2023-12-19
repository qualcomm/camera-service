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
 * Changes from Qualcomm Innovation Center are provided under the following license:
 *
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
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

#pragma once

#include <gbm_priv.h>
#include <mutex>
#include <map>
#include "qmmf_memory_interface.h"
#include "common/utils/qmmf_log.h"

class GBMUsage : public IMemAllocUsage {
 public:
  int32_t ToLocal(int32_t common) const;
  int32_t ToLocal(MemAllocFlags common) const;
  MemAllocFlags ToCommon(int32_t local) const;
  int32_t ToGralloc(MemAllocFlags common) const;
  MemAllocFlags GrallocToCommon(int32_t gralloc) const;
  int32_t LocalToGralloc(int32_t local) const;
  int32_t GrallocToLocal(int32_t gralloc) const;
 private:
  static const std::unordered_map<int32_t, int32_t> usage_flag_map_;
  static const std::unordered_map<int32_t, int32_t> gralloc_usage_flag_map_;
};

class GBMBuffer : public IBufferInterface {
 public:
  GBMBuffer() : generic_handle_(nullptr),
                gralloc_handle_(nullptr),
                usage_(),
                fd_(-1),
                imported_(false) {};
  ~GBMBuffer();
  struct gbm_bo *GetNativeHandle() const;
  int GetFD() override;
  int GetFormat() override;
  uint32_t GetSize() override;
  uint32_t GetWidth() override;
  uint32_t GetHeight() override;
  void SetNativeHandle(struct gbm_bo *bo);
  void ImportBuffer(struct gbm_bo *bo, int fd);
  int GetUsage ();
  uint32_t GetLocalFormat (int common);
  buffer_handle_t &RepackToGralloc ();

 private:
  struct gbm_bo* generic_handle_;
  buffer_handle_t gralloc_handle_;
  MemAllocFlags usage_;
  int fd_;
  bool imported_;
  static const std::unordered_map<int32_t, int32_t> from_gbm_;
  static const std::unordered_map<uint32_t, uint32_t> to_gbm_;
};

class GBMDevice : public IAllocDevice {
public:

  static std::mutex gbm_device_mutex_;
  static GBMDevice* CreateGBMDevice();
  static void DestroyGBMDevice();

  gbm_device* GetDevice() const;

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
                          MemAllocFlags usage,void** vaddr) override
                                      { return MemAllocError::kAllocOk; }

  MemAllocError UnmapBuffer(const IBufferHandle& handle) override
                                      { return MemAllocError::kAllocOk; }

private:

  GBMDevice();
  GBMDevice(GBMDevice const&);
  GBMDevice& operator=(GBMDevice const&);
  ~GBMDevice();

  gbm_device* gbm_device_;

  int gbm_fd_;

  static int32_t ref_count_;
  static GBMDevice* gbm_device_obj_;
  static const std::unordered_map<int32_t, int32_t> usage_flag_map_;
};
