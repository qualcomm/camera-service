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
 * Changes from Qualcomm Innovation Center, Inc. are provided under the following license:
 *
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
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

#ifdef USE_LIBGBM
#include "qmmf_gbm_interface.h"
#else
#include "qmmf_dmabuf_interface.h"
#endif // USE_LIBGBM
#include "qmmf_memory_interface.h"
#include "common/utils/qmmf_log.h"

const int IMemAllocUsage::kHwCameraZsl          = (1 << 0);
const int IMemAllocUsage::kPrivateAllocUbwc     = (1 << 1);
const int IMemAllocUsage::kPrivateAllocP010     = (1 << 2);
const int IMemAllocUsage::kPrivateAllocTP10     = (1 << 3);
const int IMemAllocUsage::kPrivateIommUHeap     = (1 << 4);
const int IMemAllocUsage::kPrivateMmHeap        = (1 << 5);
const int IMemAllocUsage::kPrivateUncached      = (1 << 6);
const int IMemAllocUsage::kProtected            = (1 << 7);
const int IMemAllocUsage::kSwReadOften          = (1 << 8);
const int IMemAllocUsage::kSwWriteOften         = (1 << 9);
const int IMemAllocUsage::kVideoEncoder         = (1 << 10);
const int IMemAllocUsage::kHwFb                 = (1 << 11);
const int IMemAllocUsage::kHwTexture            = (1 << 12);
const int IMemAllocUsage::kHwRender             = (1 << 13);
const int IMemAllocUsage::kHwComposer           = (1 << 14);
const int IMemAllocUsage::kHwCameraRead         = (1 << 15);
const int IMemAllocUsage::kHwCameraWrite        = (1 << 16);
const int IMemAllocUsage::kPrivateAllocHEIF     = (1 << 17);

IAllocDevice *AllocDeviceFactory::CreateAllocDevice() {
#ifdef USE_LIBGBM
  return GBMDevice::CreateGBMDevice();
#else
  return new DMABufDevice;
#endif // USE_LIBGBM
}

void AllocDeviceFactory::DestroyAllocDevice(IAllocDevice* alloc_device_interface) {
#ifdef USE_LIBGBM
  GBMDevice::DestroyGBMDevice();
#else
  delete alloc_device_interface;
#endif // USE_LIBGBM
}

const IMemAllocUsage &AllocUsageFactory::GetAllocUsage() {
#ifdef USE_LIBGBM
  static const GBMUsage x = GBMUsage();
#else
  static const DMABufUsage x = DMABufUsage();
#endif // USE_LIBGBM
  return x;
}

#ifdef USE_LIBGBM
struct gbm_bo *GetAllocBufferHandle(const IBufferHandle &handle) {
  GBMBuffer *b = static_cast<GBMBuffer *>(handle);
  assert(b != nullptr);
  return b->GetNativeHandle();
}

struct gbm_device *GetAllocDeviceHandle(const IAllocDevice &handle) {
  const GBMDevice &b = static_cast<const GBMDevice &>(handle);
  return b.GetDevice();
}

buffer_handle_t &GetGrallocBufferHandle(const IBufferHandle &handle) {
  GBMBuffer *b = static_cast<GBMBuffer *>(handle);
  assert(b != nullptr);
  return b->RepackToGralloc();
}
#else
buffer_handle_t &GetAllocBufferHandle(const IBufferHandle &handle) {
  DMABuffer *b = static_cast<DMABuffer *>(handle);
  assert(b != nullptr);
  return b->GetNativeHandle();
}
#endif // USE_LIBGBM
