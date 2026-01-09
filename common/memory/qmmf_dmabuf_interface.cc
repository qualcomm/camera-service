/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define LOG_TAG "DmabufInterface"

#include "qmmf_dmabuf_interface.h"

#include <dlfcn.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <sys/ioctl.h>
#include "qmmf_common_utils.h"

using namespace qmmf;

const char* kFormatUtilLibName = "libcamxexternalformatutils";

const std::unordered_map<int32_t, int32_t> DMABufUsage::usage_flag_map_ = {
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

int32_t DMABufUsage::ToLocal(int32_t common) const {
  int32_t local_usage = 0;
  for (auto &it : usage_flag_map_) {
    if (it.first & common) {
      local_usage |= it.second;
    }
  }
  return local_usage;
}

int32_t DMABufUsage::ToLocal(MemAllocFlags common) const {
  int32_t local_usage = 0;
  for (auto &it : usage_flag_map_) {
    if (it.first & common.flags) {
      local_usage |= it.second;
    }
  }
  return local_usage;
}

int32_t DMABufUsage::ToGralloc(MemAllocFlags common) const {
  return ToLocal(common);
}

MemAllocFlags DMABufUsage::ToCommon(int32_t local) const {
  MemAllocFlags common;
  common.flags = 0;
  for (auto &it : usage_flag_map_) {
    if (it.second & local) {
      common.flags |= it.first;
    }
  }
  return common;
}

DMABuffer::~DMABuffer() {
  if (fd_ >= 0) {
    close(fd_);
  }

  delete generic_handle_;
}

buffer_handle_t &DMABuffer::GetNativeHandle() { return generic_handle_; }

void DMABuffer::SetNativeHandle(int fd, uint32_t size,
                                uint32_t width, uint32_t height,
                                int format, int32_t usage,
                                int stride, int scanline) {
  private_handle_t *priv_hnd = new private_handle_t(fd, size,
    (int)usage, 0, format, width, height);
  assert(priv_hnd != NULL);
  priv_hnd->height = scanline;
  priv_hnd->width = stride;
  generic_handle_ = static_cast<buffer_handle_t>(priv_hnd);
  fd_ = fd;
}

int DMABuffer::GetFD() {
  return (static_cast<const private_handle_t *>(generic_handle_))->fd;
}

int DMABuffer::GetFormat() {
  return (static_cast<const private_handle_t *>(generic_handle_))->format;
}

uint32_t DMABuffer::GetSize() {
  return (static_cast<const private_handle_t *>(generic_handle_))->size;
}

uint32_t DMABuffer::GetWidth() {
  return (static_cast<const private_handle_t *>(generic_handle_))
      ->unaligned_width;
}

uint32_t DMABuffer::GetHeight() {
  return (static_cast<const private_handle_t *>(generic_handle_))
      ->unaligned_height;
}

uint32_t DMABuffer::GetStride() {
  return (static_cast<const private_handle_t *>(generic_handle_))->width;
}

uint32_t DMABuffer::GetScanline() {
  return (static_cast<const private_handle_t *>(generic_handle_))->height;
}

MemAllocError DMABufDevice::AllocBuffer(IBufferHandle& handle, int32_t width,
                                         int32_t height, int32_t format,
                                         int32_t override_format,
                                         MemAllocFlags usage, uint32_t *stride,
                                         uint32_t colorimetry) {
  assert(width && height);
  int32_t local_usage = DMABufUsage().ToLocal(usage);
  int p_stride = width;
  int p_scanline = height;
  unsigned int size = 0;
  int res;

  DMABuffer *b = new DMABuffer;
  handle = b;

  CamxPixelFormat cam_format = static_cast<CamxPixelFormat>(override_format);
  res = get_buffer_size_fn_(cam_format, width, height, &size);
  if (res != 0) {
    QMMF_ERROR("%s: failed to get size for DMA buffer: %d\n", __func__, res);
    delete b;
    handle = nullptr;
    return MemAllocError::kAllocFail;
  }

  if (format != HAL_PIXEL_FORMAT_BLOB) {
    CamxPlaneType plane_types[CamxFormatUtilMaxNumPlanes] = {};
    int plane_count;
    res = get_plane_types_fn_(cam_format, plane_types, &plane_count);
    if (res != 0) {
      QMMF_ERROR("%s: failed to get plane types for format: %d\n", __func__, res);
      delete b;
      handle = nullptr;
      return MemAllocError::kAllocFail;
    }

    res = get_stride_in_bytes_fn_(cam_format, plane_types[0],
                                          width, &p_stride);
    if (res != 0) {
      QMMF_ERROR("%s: failed to get stride for format: %d\n", __func__, res);
      delete b;
      handle = nullptr;
      return MemAllocError::kAllocFail;
    }

    res = get_scanline_fn_(cam_format, plane_types[0],
                                    height, &p_scanline);
    if (res != 0) {
      QMMF_ERROR("%s: failed to get stride for format: %d\n", __func__, res);
      delete b;
      handle = nullptr;
      return MemAllocError::kAllocFail;
    }
  }

  struct dma_heap_allocation_data alloc_data;
  alloc_data.fd = 0;
  alloc_data.len = (unsigned long)size;
  alloc_data.fd_flags = O_RDWR | O_CLOEXEC;
  alloc_data.heap_flags = 0;

  res = ioctl (dma_dev_fd_, DMA_HEAP_IOCTL_ALLOC, &alloc_data);

  if (0 != res) {
    QMMF_ERROR("%s: Unable to allocate DMA buffer: %d\n", __func__, res);
    delete b;
    handle = nullptr;
    return MemAllocError::kAllocFail;
  }

  *stride = static_cast<uint32_t>(p_stride);

  b->SetNativeHandle(alloc_data.fd, size, width,
       height, override_format, local_usage,
       p_stride, p_scanline);

  QMMF_DEBUG ("%s: Allocated fd: %d format: 0x%x unaligned dim: %dx%d"
      " stride: %d scanline: %d size: %d", __func__,
      alloc_data.fd, override_format, width, height,
      b->GetStride(), b->GetScanline(), size);

  return MemAllocError::kAllocOk;
}

MemAllocError DMABufDevice::FreeBuffer(IBufferHandle handle) {
  if (nullptr != handle) {
    delete handle;
    handle = nullptr;
  }
  return MemAllocError::kAllocOk;
}

MemAllocError DMABufDevice::Perform(const IBufferHandle& handle,
                                     AllocDeviceAction action, void* result) {
  DMABuffer *b = static_cast<DMABuffer *>(handle);
  if (nullptr == b) {
    return MemAllocError::kAllocFail;
  }
  switch (action) {
    case AllocDeviceAction::GetHeight:
      *static_cast<int32_t*>(result) = b->GetHeight();
      return MemAllocError::kAllocOk;
    case AllocDeviceAction::GetStride:
      *static_cast<int32_t*>(result) = b->GetStride();
      return MemAllocError::kAllocOk;
    case AllocDeviceAction::GetAlignedHeight:
      *static_cast<int32_t*>(result) = b->GetScanline();
      return MemAllocError::kAllocOk;
    case AllocDeviceAction::GetAlignedWidth:
      *static_cast<int32_t*>(result) = b->GetStride();
      return MemAllocError::kAllocOk;
    case AllocDeviceAction::GetMetaFd: {
      int32_t metafd = -1;
      *static_cast<int32_t*>(result) = metafd;
      return MemAllocError::kAllocOk;
    }
    default:
      QMMF_ERROR("%s: Unrecognized action to perform.", __func__);
      return MemAllocError::kAllocFail;
  }

}

bool DMABufDevice::LoadFormatUtil() {
  std::string lib_name =
      Target::GetLibName(std::string(kFormatUtilLibName), "0");

  format_util_handle_ = dlopen(lib_name.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (!format_util_handle_) {
    QMMF_ERROR("%s: dlopen failed for format util lib: %s with error %s",
               __func__, lib_name, dlerror());
    return false;
  }

  dlerror();

  get_buffer_size_fn_ = reinterpret_cast<FormatUtilGetBufferSizeFn>(
      dlsym(format_util_handle_, "CamxFormatUtil_GetBufferSize"));
  get_plane_types_fn_ = reinterpret_cast<FormatUtilGetPlaneTypesFn>(
      dlsym(format_util_handle_, "CamxFormatUtil_GetPlaneTypes"));
  get_stride_in_bytes_fn_ = reinterpret_cast<FormatUtilGetStrideInBytesFn>(
      dlsym(format_util_handle_, "CamxFormatUtil_GetStrideInBytes"));
  get_scanline_fn_ = reinterpret_cast<FormatUtilGetScanlineFn>(
      dlsym(format_util_handle_, "CamxFormatUtil_GetScanline"));

  const char *err = dlerror();
  if (err != nullptr || !get_buffer_size_fn_ || !get_plane_types_fn_ ||
      !get_stride_in_bytes_fn_ || !get_scanline_fn_) {
    QMMF_ERROR("%s: dlsym() failed for format Util symbols: %s", __func__,
        err ? err : "missing symbol");
    dlclose(format_util_handle_);
    format_util_handle_ = nullptr;
    get_buffer_size_fn_ = nullptr;
    get_plane_types_fn_ = nullptr;
    get_stride_in_bytes_fn_ = nullptr;
    get_scanline_fn_ = nullptr;
    return false;
  }

  return true;
}

void DMABufDevice::UnloadFormatUtil() {
  if (format_util_handle_) {
    dlclose(format_util_handle_);
    format_util_handle_ = nullptr;
  }
  get_buffer_size_fn_ = nullptr;
  get_plane_types_fn_ = nullptr;
  get_stride_in_bytes_fn_ = nullptr;
  get_scanline_fn_ = nullptr;
}

DMABufDevice::DMABufDevice() {
  dma_dev_fd_ = open("/dev/dma_heap/qcom,system", O_RDONLY | O_CLOEXEC);

  if (dma_dev_fd_ < 0) {
    QMMF_WARN ("%s: Failed to open /dev/dma_heap/qcom,system, "
      "Falling back to /dev/dma_heap/system", __func__);
    dma_dev_fd_ = open ("/dev/dma_heap/system", O_RDONLY | O_CLOEXEC);
  }

  if (dma_dev_fd_ < 0) {
    QMMF_WARN ("%s: Failed to open /dev/dma_heap/system", __func__);
  }

  assert(dma_dev_fd_ >= 0);

  assert(LoadFormatUtil() && "LoadFormatUtil() failed");
}

DMABufDevice::~DMABufDevice() {
  close(dma_dev_fd_);
  UnloadFormatUtil();
}
