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

#define LOG_TAG "MemoryInterface"

#ifdef TARGET_USES_GRALLOC1
#include "qmmf_gralloc1_interface.h"
#elif TARGET_USES_GRALLOC2
#include "qmmf_gralloc2_interface.h"
#elif USE_LIBGBM
#include "qmmf_gbm_interface.h"
#elif HAVE_BINDER
#include "qmmf_gralloc_interface.h"
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
const int IMemAllocUsage::kFlex2Batch           = (1 << 18);
const int IMemAllocUsage::kFlex4Batch           = (1 << 19);
const int IMemAllocUsage::kFlex8Batch           = (1 << 20);
const int IMemAllocUsage::kFlexBatch            = (1 << 21);
const int IMemAllocUsage::kPrivateSnapshot      = (1 << 22);

IAllocDevice *AllocDeviceFactory::CreateAllocDevice() {
#ifdef TARGET_USES_GRALLOC1
  return new Gralloc1Device;
#elif TARGET_USES_GRALLOC2
  return new Gralloc2Device;
#elif USE_LIBGBM
  return GBMDevice::CreateGBMDevice();
#elif HAVE_BINDER
  return new GrallocDevice;
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
#ifdef TARGET_USES_GRALLOC1
  static const Gralloc1Usage x = Gralloc1Usage();
#elif TARGET_USES_GRALLOC2
  static const Gralloc2Usage x = Gralloc2Usage();
#elif USE_LIBGBM
  static const GBMUsage x = GBMUsage();
#elif HAVE_BINDER
  static const GrallocUsage x = GrallocUsage();
#else
  static const DMABufUsage x = DMABufUsage();
#endif // USE_LIBGBM
  return x;
}

#ifdef TARGET_USES_GRALLOC1
buffer_handle_t &GetAllocBufferHandle(const IBufferHandle &handle) {
  Gralloc1Buffer *b = static_cast<Gralloc1Buffer *>(handle);
  assert(b != nullptr);
  return b->GetNativeHandle();
}

gralloc1_device_t *GetAllocDeviceHandle(const IAllocDevice &handle) {
  const Gralloc1Device &b = static_cast<const Gralloc1Device &>(handle);
  return b.GetDevice();
}

#elif TARGET_USES_GRALLOC2
buffer_handle_t &GetAllocBufferHandle(const IBufferHandle &handle) {
  const Gralloc2Buffer *b = static_cast<const Gralloc2Buffer *>(handle);
  assert(b != nullptr);
  return b->GetNativeHandle();
}

gralloc2_device_t *GetAllocDeviceHandle(const IAllocDevice &handle) {
  const Gralloc2Device &b = static_cast<const Gralloc2Device &>(handle);
  return b.GetDevice();
}

#elif USE_LIBGBM
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

#elif HAVE_BINDER
buffer_handle_t &GetAllocBufferHandle(const IBufferHandle &handle) {
  GrallocBuffer *b = static_cast<GrallocBuffer *>(handle);
  assert(b != nullptr);
  return b->GetNativeHandle();
}

alloc_device_t *GetAllocDeviceHandle(const IAllocDevice &handle) {
  const GrallocDevice &b = static_cast<const GrallocDevice &>(handle);
  return b.GetDevice();
}

#else
buffer_handle_t &GetAllocBufferHandle(const IBufferHandle &handle) {
  DMABuffer *b = static_cast<DMABuffer *>(handle);
  assert(b != nullptr);
  return b->GetNativeHandle();
}
#endif // USE_LIBGBM
