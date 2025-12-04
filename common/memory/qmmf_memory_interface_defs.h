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

#include <cstdint>

/** MemAllocError
* @Fail - error while memory allocator operation
* @Ok - memory allocator operation complete with success
*
* Enumeration type for memory allocator operations result.
*
**/
enum class MemAllocError { kAllocFail = -1, kAllocOk = 0 };

/** MemAllocFlags
* @flags - member to hold actual common flags
*
* Abstract interface for allocator usage flags. The main purpose of this class
* is to avoid confusion of native vs abstracted flags - as typically these are
* kept as bit fields in an integer variable so compiler cannot make difference.
*
**/
class MemAllocFlags {
 public:
  int flags;
  MemAllocFlags() { flags = 0; }
  MemAllocFlags(int32_t new_flags) { flags = new_flags; }

  /** MemAllocFlags::Equals
  *
  * Compares with another set of flags. Returns true if both are same.
  *
  **/
  bool Equals(const MemAllocFlags& to) const { return flags == to.flags; }


  bool Exists(const int flag) const { return flags & flag; }

  bool Matches(const int flag) const { return ((flags & flag) == flag); }
};

/** MemAllocFlags
* @kHwCameraZsl - stream will be used for ZSL capture
* @kPrivateAllocUbwc - buffer will contain UBWC formatted data
* @kPrivateIommUHeap - buffer will be mapped to IOMMU
* @kPrivateMmHeap -
* @kPrivateUncached - buffer will be uncached
* @kProtected - buffer will be protected
* @kSwReadOften - buffer will be used for SW read
* @kSwWriteOften - buffer will be used for SW write
* @kHwFb - buffer will be used for HW read/write
* @kVideoEncoder - buffer will be used by encoder
*
* Abstract class providing definitions and convertion methods for usage flags
* Does not contain any variable data.
*
**/
class IMemAllocUsage {
 public:
  static const int kHwCameraZsl;
  static const int kPrivateAllocUbwc;
  static const int kPrivateAllocP010;
  static const int kPrivateAllocTP10;
  static const int kPrivateIommUHeap;
  static const int kPrivateMmHeap;
  static const int kPrivateUncached;
  static const int kProtected;
  static const int kSwReadOften;
  static const int kSwWriteOften;
  static const int kVideoEncoder;
  static const int kHwFb;
  static const int kHwTexture;
  static const int kHwRender;
  static const int kHwComposer;
  static const int kHwCameraRead;
  static const int kHwCameraWrite;
  static const int kPrivateAllocHEIF;
  static const int kFlex2Batch;
  static const int kFlex4Batch;
  static const int kFlex8Batch;
  static const int kFlexBatch;
  static const int kPrivateSnapshot;

  /** IMemAllocUsage::ToLocal
  *
  * Converts common usage flags (as int) to native flags
  *
  **/
  virtual int32_t ToLocal(int32_t common) const = 0;

  /** IMemAllocUsage::ToLocal
  *
  * Converts common usage flags (as MemAllocFlags) to native flags
  *
  **/
  virtual int32_t ToLocal(MemAllocFlags common) const = 0;

  /** IMemAllocUsage::ToLocal
  *
  * Converts native usage flags to common flags
  *
  **/
  virtual MemAllocFlags ToCommon(int32_t local) const = 0;

  /** IMemAllocUsage::ToGralloc
  *
  * Converts native usage flags to Gralloc flags
  *
  **/
  virtual int32_t ToGralloc(MemAllocFlags common) const = 0;

  virtual ~IMemAllocUsage(){};
};

/** IBufferInterface
*
* Abstract interface for mem buffer objects acquired via some allocator
*
**/
class IBufferInterface {
 public:
  virtual ~IBufferInterface(){};

  /** GetFD
  *
  * Returns file descriptor of the buffer
  *
  **/
  virtual int GetFD() = 0;

  /** GetFormat
  *
  * Returns format of the buffer
  *
  **/
  virtual int GetFormat() = 0;

  /** GetSize
  *
  * Returns size of the buffer
  *
  **/
  virtual uint32_t GetSize() = 0;

  /** GetWidth
  *
  * Returns width of the buffer
  *
  **/
  virtual uint32_t GetWidth() = 0;

  /** GetHeight
  *
  * Returns height of the buffer
  *
  **/
  virtual uint32_t GetHeight() = 0;
};

/** IBufferHandle
*
* Type for buffer objects used by interface classes described here
*
**/
typedef IBufferInterface* IBufferHandle;

/** IAllocDevice
*
* Abstract interface to the memory allocator device.
*
**/
class IAllocDevice {
 public:
  /** AllocDeviceAction
  * @GetStride - reads stride from handle
  * @GetHeight - reads height from handle
  * @GetAlignedWidth - reads aligned width in pixels from handle
  * @GetAlignedHeight - reads aligned height in pixels from handle
  *
  * Enumeration type for performing action on BufferHandler.
  *
  **/
  enum class AllocDeviceAction {GetMetaFd, GetStride, GetHeight,
                                GetAlignedWidth, GetAlignedHeight};

  virtual ~IAllocDevice(){};

  /** IAllocDevice::AllocBuffer
  * @handle - handle to the allocated buffer
  * @width - width of the buffer
  * @height - height of the buffer
  * @format - HAL format of the buffer
  * @cam_format - CAM format of the buffer
  * @usage - usage flags of the buffer
  * @stride - returned: result stride for the allocated buffer according format
  *           width and usage
  * @colorimetry - color space of the buffer
  *
  * Allocates buffer with given dimensions, format and usage
  *
  * Returns MemAllocError::kAllocOk - buffer is allocated successfully
  *         MemAllocError::kAllocFail - otherwise
  *
  **/
  virtual MemAllocError AllocBuffer(IBufferHandle& handle, int32_t width,
                                    int32_t height, int32_t format,
#ifdef HAVE_BINDER
                                    MemAllocFlags usage,
#else
                                    int override_format, MemAllocFlags usage,
#endif
                                    uint32_t* stride, uint32_t colorimetry) = 0;

  virtual MemAllocError ImportBuffer(IBufferHandle& handle,
                                     void* buffer_handle, int fd) = 0;

  /** IAllocDevice::FreeBuffer
  * @handle - handle to the allocated buffer
  *
  * Frees buffer with given handle
  *
  * Returns MemAllocError::kAllocOk - buffer is freed successfully
  *         MemAllocError::kAllocFail - otherwise
  *
  **/
  virtual MemAllocError FreeBuffer(IBufferHandle handle) = 0;

  /** IAllocDevice::MapBuffer
  * @handle - handle to the allocated buffer
  * @sx - start horizontal offset of mapped area in the buffer
  * @sy - start vertical offset of mapped area in the buffer
  * @width - width of mapped area in the buffer
  * @height - height of mapped area in the buffer
  * @usage - usage flags of the buffer
  * @vaddr - Returned: virtual address to mapped area
  *
  * Maps area of the buffer for SW access
  *
  * Returns MemAllocError::kAllocOk - buffer is mapped successfully
  *         MemAllocError::kAllocFail - otherwise
  *
  **/
  virtual MemAllocError MapBuffer(const IBufferHandle& handle, int32_t start_x,
                                  int32_t start_y, int32_t width, int32_t height,
                                  MemAllocFlags usage, void** vaddr) = 0;

  /** IAllocDevice::UnmapBuffer
  * @handle - handle to the allocated buffer
  *
  * Unmaps area of the buffer mapped with IAllocDevice::MapBuffer
  *
  * Returns MemAllocError::kAllocOk - buffer is unmapped successfully
  *         MemAllocError::kAllocFail - otherwise
  *
  **/
  virtual MemAllocError UnmapBuffer(const IBufferHandle& handle) = 0;

  /** IAllocDevice::Perform
  * @handle - handle to the allocated buffer
  * @action - generic action to be performed
  * @result - returned: result for given action
  * Performs action and returns result
  *
  * Returns MemAllocError::kAllocOk - operation is successful
  *         MemAllocError::kAllocFail - otherwise
  *
  **/
  virtual MemAllocError Perform(const IBufferHandle& handle,
                                AllocDeviceAction action,
                                void *result) = 0;
};

/** AllocDeviceFactory
*
* Factory class producing new allocator device. Currently type of device is
* determined compile time, however interface can support runtime decision
*
**/
class AllocDeviceFactory {
 public:
  static IAllocDevice* CreateAllocDevice();
  static void DestroyAllocDevice(IAllocDevice* alloc_device_interface);
};

/** AllocUsageFactory
*
* Factory class producing new usage flag type converting class. Currently type
* is determined compile time, however interface can support runtime decision
**/
class AllocUsageFactory {
 public:
  static const IMemAllocUsage& GetAllocUsage();
};
