/*
* Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
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

#define LOG_TAG "RecorderRescaler"

#include <chrono>
#include <map>
#include <sys/prctl.h>

#ifndef DISABLE_RESCALER_COLORSPACE
#include "qdMetaData.h"
#endif

#include "recorder/src/service/qmmf_camera_rescaler.h"
#include "recorder/src/service/qmmf_recorder_utils.h"

#ifndef HAVE_BINDER
#include "common/propertyvault/qmmf_propertyvault.h"
#endif
#include "common/resizer-neon/qmmf_resizer_neon.h"
#include "common/resizer-c2d/qmmf_resizer_c2d.h"
#ifndef CAMERA_HAL1_SUPPORT
#ifdef ENABLE_RESCALER_FASTCV
#include "common/resizer-fastCV/qmmf_resizer_fastCV.h"
#endif
#endif

namespace qmmf {

namespace recorder {

using ::std::chrono::high_resolution_clock;
using ::std::chrono::microseconds;
using ::std::chrono::nanoseconds;
using ::std::chrono::time_point;
using ::std::chrono::duration_cast;

CameraRescalerBase::CameraRescalerBase()
    :CameraRescalerThread() {
  QMMF_INFO("%s: Enter", __func__);
  char prop[PROP_VALUE_MAX];
  memset(prop, 0, sizeof(prop));
#ifdef HAVE_BINDER
#ifdef ENABLE_RESCALER_NEON
  property_get("persist.qmmf.rescaler.type", prop, "Neon");
#elif ENABLE_RESCALER_C2D
  property_get("persist.qmmf.rescaler.type", prop, "C2D");
#elif ENABLE_RESCALER_FASTCV
  property_get("persist.qmmf.rescaler.type", prop, "FastCV");
#endif
#else
#ifdef ENABLE_RESCALER_NEON
  qmmf_property_get("persist.qmmf.rescaler.type", prop, "Neon");
#elif ENABLE_RESCALER_C2D
  qmmf_property_get("persist.qmmf.rescaler.type", prop, "C2D");
#elif ENABLE_RESCALER_FASTCV
  qmmf_property_get("persist.qmmf.rescaler.type", prop, "FastCV");
#endif
#endif
  std::string name = prop;
#ifndef CAMERA_HAL1_SUPPORT
  if (name == "Neon") {
#ifdef ENABLE_RESCALER_NEON
      rescaler_ = new NEONResizer();
#endif
  } else if (name == "FastCV") {
#ifdef ENABLE_RESCALER_FASTCV
    rescaler_ = new FastCVResizer();
#endif
  } else {
#ifdef ENABLE_RESCALER_C2D
    rescaler_ = new C2DResizer();
#endif
  }
#else
  if (name == "Neon") {
    rescaler_ = new NEONResizer();
  } else {
    rescaler_ = new C2DResizer();
  }
#endif

  memset(prop, 0, sizeof(prop));
#ifdef HAVE_BINDER
  property_get("persist.qipcam.rescaler.perf", prop, "0");
#else
  qmmf_property_get("persist.qipcam.rescaler.perf", prop, "0");
#endif
  uint32_t value = (uint32_t) atoi(prop);
  print_process_time_ = (value == 1) ? true : false;

#ifdef HAVE_BINDER
  property_get(PRESERVE_ASPECT_RATIO, prop, "1");
#else
  qmmf_property_get(PRESERVE_ASPECT_RATIO, prop, "1");
#endif
  value = (uint32_t) atoi(prop);
  rescaler_->aspect_ratio_preserve_ = (value == 1) ? true : false;

  rescaler_->Init();
  QMMF_INFO("%s: Exit (%p)", __func__, this);
}

CameraRescalerBase::~CameraRescalerBase() {
  QMMF_INFO("%s: Enter", __func__);
  rescaler_->DeInit();
  delete rescaler_;
  QMMF_INFO("%s: Exit (%p)", __func__, this);
}

status_t CameraRescalerBase::Validate(const uint32_t& width,
                                      const uint32_t& height,
                                      const BufferFormat& fmt) {
  if (rescaler_ == nullptr) {
    QMMF_ERROR("%s: Missing Rescaler engine!!!", __func__);
    return -EINVAL;
  }

  if (rescaler_->ValidateOutput(width, height, fmt) != RESIZER_STATUS_OK) {
    QMMF_ERROR("%s: Validation Error!!!", __func__);
    return -EINVAL;
  }

  return 0;
}

status_t CameraRescalerBase::ReturnBufferToBufferPool(
    const StreamBuffer &buffer) {
  status_t ret = ReturnBufferLocked(buffer);
  if (0 != ret) {
    QMMF_ERROR("%s: Failed to return buffer to memory pool", __func__);
  }
  return ret;
}

void CameraRescalerBase::FlushBufs() {
  std::unique_lock<std::mutex> lock(wait_lock_);

  StreamBuffer buffer;
  auto iter = bufs_list_.begin();
  while (iter != bufs_list_.end()) {
    iter = bufs_list_.begin();
    buffer = *iter;
    bufs_list_.erase(iter);
    QMMF_INFO("%s: back to client node: FD: %d", __func__, buffer.fd);
    ReturnBufferToProducer(buffer);
  }
}

void CameraRescalerBase::AddBuf(StreamBuffer& buffer) {
  QMMF_DEBUG("%s: Enter", __func__);
  std::unique_lock<std::mutex> lock(wait_lock_);
  bufs_list_.push_back(buffer);
  wait_.Signal();
}

bool CameraRescalerBase::ThreadLoop() {
  bool status = true;

  StreamBuffer in_buffer;
  {
    std::unique_lock<std::mutex> lock(wait_lock_);
    std::chrono::nanoseconds wait_time(kFrameTimeout);
    while (bufs_list_.empty()) {
      auto ret = wait_.WaitFor(lock, wait_time);
      if (ret != 0) {
        QMMF_DEBUG("%s: Wait for frame available timed out", __func__);
        // timeout loop again
        return true;
      }
    }
    auto iter = bufs_list_.begin();
    in_buffer = *iter;
    bufs_list_.erase(iter);
  }

  StreamBuffer out_buffer{};
  GetFreeOutputBuffer(&out_buffer);

  out_buffer.stream_id    = 0x55aa;
  out_buffer.timestamp    = in_buffer.timestamp;
  out_buffer.frame_number = in_buffer.frame_number;
  out_buffer.camera_id    = in_buffer.camera_id;
  out_buffer.flags        = in_buffer.flags;

  QMMF_DEBUG("%s: Thread map", __func__);
  auto ret = MapBuf(out_buffer);
  if (ret != 0) {
    QMMF_ERROR("%s: fail to map in_buffer", __func__);
    ReturnBufferToBufferPool(out_buffer);
    ReturnBufferToProducer(in_buffer);
    return true;
  }

  void *vaaddr = nullptr;
  bool in_buff_map = false;
  if (in_buffer.data == nullptr) {
    vaaddr = mmap(nullptr, in_buffer.size, PROT_READ  | PROT_WRITE,
        MAP_SHARED, in_buffer.fd, 0);
    QMMF_DEBUG("%s: Thread map in buff done", __func__);
    in_buff_map = true;
    in_buffer.data = vaaddr;
  }

  time_point<high_resolution_clock>   start_time;
  if (print_process_time_) {
    start_time = high_resolution_clock::now();
  }

  if (rescaler_) {
    rescaler_->Draw(in_buffer, out_buffer);
  }

  if(print_process_time_) {
    time_point<high_resolution_clock> curr_time = high_resolution_clock::now();
    uint64_t time_diff = duration_cast<microseconds>
                             (curr_time - start_time).count();
    QMMF_INFO("%s: stream_id(%d) Full ProcessingTime=%lld",
        __func__, in_buffer.stream_id, time_diff);
  }

  if (in_buff_map) {
    munmap(in_buffer.data, in_buffer.size);
    in_buffer.data = nullptr;
  }

  ReturnBufferToProducer(in_buffer);
  NotifyBufferToClient(out_buffer);

  return status;
}

status_t CameraRescalerBase::MapBuf(StreamBuffer& buffer) {
  void *vaaddr = nullptr;

  if (buffer.fd == -1) {
    QMMF_ERROR("%s: Error Invalid FD", __func__);
    return -EINVAL;
  }

  QMMF_DEBUG("%s: buffer.fd=%d buffer.size=%d", __func__,
      buffer.fd, buffer.size);

  if (mapped_buffs_.count(buffer.fd) == 0) {
    vaaddr = mmap(nullptr, buffer.size, PROT_READ  | PROT_WRITE,
        MAP_SHARED, buffer.fd, 0);
    if (vaaddr == MAP_FAILED) {
        QMMF_ERROR("%s: ION mmap failed: error(%s):(%d)", __func__,
            strerror(errno), errno);
        return -EINVAL;
    }
    buffer.data = vaaddr;
    map_data_t map;
    map.addr = vaaddr;
    map.size = buffer.size;
    mapped_buffs_[buffer.fd] = map;
    buffer.data = vaaddr;
  } else {
    buffer.data = mapped_buffs_[buffer.fd].addr;
  }

  return 0;
}

void CameraRescalerBase::UnMapBufs() {
  for (auto iter : mapped_buffs_) {
    auto map = iter.second;
    if (map.addr) {
      QMMF_INFO("%s: Unmap addr(%p) size(%lu)", __func__,
          map.addr, map.size);
      munmap(map.addr, map.size);
    }
  }
  mapped_buffs_.clear();
}

status_t CameraRescalerBase::Configure(const ResizerCrop& config_data) {
  auto ret = rescaler_->Configure(config_data);
  if (ret != RESIZER_STATUS_OK) {
    return -EINVAL;
  }
  return 0;
}

int32_t CameraRescalerThread::Run(const std::string &name) {
  int32_t res = 0;

  std::lock_guard<std::mutex> lock(lock_);
  if (running_) {
    QMMF_ERROR("%s: Thread %s already started!\n", __func__, name_.c_str());
    res = -ENOSYS;
    goto exit;
  }

  abort_ = false;
  running_ = true;
  thread_ = new std::thread(MainLoop, this);
  if (thread_ == nullptr) {
    QMMF_ERROR("%s: Unable to create thread\n", __func__);
    running_ = false;
    goto exit;
  }

  if (name.empty()) {
    // use thread id as name
    std::stringstream ss;
    ss << thread_->get_id();
    name_ = ss.str();
  } else {
    name_ = name;
  }
  prctl(PR_SET_NAME, name_.c_str(), 0, 0, 0);
  QMMF_INFO("%s: Thread %s is running\n", __func__, name_.c_str());

exit:
  return res;
}

void CameraRescalerThread::RequestExit() {
  std::lock_guard<std::mutex> lock(lock_);
  if (thread_ == nullptr || running_ == false) {
    QMMF_ERROR("%s: Thread %s is not running\n", __func__, name_.c_str());
    return;
  }

  abort_ = true;
}

void CameraRescalerThread::RequestExitAndWait() {
  std::lock_guard<std::mutex> lock(lock_);
  if (thread_ == nullptr) {
    QMMF_ERROR("%s: Thread %s is stopped\n", __func__, name_.c_str());
    return;
  }

  abort_ = true;
  thread_->join();
  delete(thread_);
  thread_ = nullptr;
}

void *CameraRescalerThread::MainLoop(void *userdata) {

  CameraRescalerThread *pme = reinterpret_cast<CameraRescalerThread *>(userdata);
  if (nullptr == pme) {
    return nullptr;
  }

  bool run = true;
  while (pme->abort_ == false && run == true) {
    run = pme->ThreadLoop();
  }

  pme->running_ = false;
  return nullptr;
}

bool CameraRescalerThread::ExitPending() {
  std::lock_guard<std::mutex> lock(lock_);
  return (abort_ == true && running_ == true)  ? false : true;
}

#define RESCALER_BUFFERS_CNT (13)

CameraRescalerMemPool::CameraRescalerMemPool()
    : alloc_device_interface_(nullptr),
      mem_alloc_slots_(nullptr),
      buffers_allocated_(0),
      pending_buffer_count_(0),
      buffer_cnt_(RESCALER_BUFFERS_CNT),
      is_eis_on_(false),
      is_ldc_on_(false) {
  QMMF_INFO("%s: Enter", __func__);
  QMMF_INFO("%s: Exit (%p)", __func__, this);
}

CameraRescalerMemPool::~CameraRescalerMemPool() {
  QMMF_INFO("%s: Enter", __func__);

  if (!mem_alloc_buffers_.empty()) {
    for (auto const& it : mem_alloc_buffers_) {
      FreeHWMemBuffer(it.first);
    }
    mem_alloc_buffers_.clear();
  }
  if (nullptr != mem_alloc_slots_) {
    delete[] mem_alloc_slots_;
  }
  if (nullptr != alloc_device_interface_) {
    AllocDeviceFactory::DestroyAllocDevice(alloc_device_interface_);
    alloc_device_interface_ = nullptr;
  }
  QMMF_INFO("%s: Exit (%p)", __func__, this);
}

int32_t CameraRescalerMemPool::Initialize(uint32_t width,
                                          uint32_t height,
                                          int32_t  format,
                                          const CameraExtraParam& extra_param) {
  status_t ret = 0;

  init_params_.width = width;
  init_params_.height = height;
  init_params_.format = format;

  if (extra_param.Exists(QMMF_EIS)) {
    size_t entry_count = extra_param.EntryCount(QMMF_EIS);
    if (entry_count == 1) {
      EISSetup eis_mode;
      extra_param.Fetch(QMMF_EIS, eis_mode, 0);
      if (eis_mode.enable == true) {
        is_eis_on_ = true;
      }
    } else {
      QMMF_ERROR("%s: Invalid EIS mode received", __func__);
      return -EINVAL;
    }
  }

#ifdef EIS_MODES_ENABLE
  if (extra_param.Exists(QMMF_EIS_MODE)) {
    size_t entry_count = extra_param.EntryCount(QMMF_EIS_MODE);
    if (entry_count == 1) {
      EISModeSetup eis_mode;
      extra_param.Fetch(QMMF_EIS_MODE, eis_mode, 0);
      if (eis_mode.mode == EisMode::kEisSingleStream) {
        is_eis_on_ = true;
      }
    } else {
      QMMF_ERROR("%s: Invalid EIS mode received", __func__);
      return -EINVAL;
    }
  }
#endif // EIS_MODES_ENABLE

  if (extra_param.Exists(QMMF_LDC)) {
    size_t entry_count = extra_param.EntryCount(QMMF_LDC);
    if (entry_count == 1) {
      LDCMode ldc_mode;
      extra_param.Fetch(QMMF_LDC, ldc_mode, 0);
      if (ldc_mode.enable == true) {
        is_ldc_on_ = true;
      }
    } else {
      QMMF_ERROR("%s: Invalid LDC mode received", __func__);
      return -EINVAL;
    }
  }

  alloc_device_interface_ = AllocDeviceFactory::CreateAllocDevice();
  if (nullptr == alloc_device_interface_) {
    QMMF_ERROR("%s: Could not create alloc device", __func__);
    goto FAIL;
  }

  // Allocate mem alloc slots.
  if (buffer_cnt_ > 0) {
    mem_alloc_slots_ = new IBufferHandle[buffer_cnt_];
    if (mem_alloc_slots_ == nullptr) {
      QMMF_ERROR("%s: Unable to allocate buffer handles!", __func__);
      ret = -ENOMEM;
      goto FAIL;
    }
  } else {
    mem_alloc_slots_ = nullptr;
  }

  return 0;

FAIL:
  if (nullptr != alloc_device_interface_) {
    AllocDeviceFactory::DestroyAllocDevice(alloc_device_interface_);
    alloc_device_interface_ = nullptr;
  }
  return -1;
}

status_t CameraRescalerMemPool::ReturnBufferLocked(const StreamBuffer &buffer) {

  if (pending_buffer_count_ == 0) {
    QMMF_ERROR("%s: Not expecting any buffers!", __func__);
    return -ENOSYS;
  }

  std::lock_guard<std::mutex> lock(buffer_lock_);

  if (mem_alloc_buffers_.find(buffer.handle) == mem_alloc_buffers_.end()) {
    QMMF_ERROR("%s: Buffer %p returned that wasn't allocated by this node",
        __func__, buffer.handle);
    return -EINVAL;
  }

  mem_alloc_buffers_.at(buffer.handle) = true;
  --pending_buffer_count_;

  wait_for_buffer_.Signal();
  return 0;
}

status_t CameraRescalerMemPool::GetFreeOutputBuffer(StreamBuffer* buffer) {

  status_t ret = 0;
  std::unique_lock<std::mutex> lock(buffer_lock_);

  buffer->fd = -1;

  if (mem_alloc_slots_ == nullptr) {
    QMMF_ERROR("%s: Error mem alloc slots!", __func__);
    return 0;
  }

  while (pending_buffer_count_ == buffer_cnt_) {
    QMMF_VERBOSE("%s: Already retrieved maximum buffers (%d), waiting"
        " on a free one",  __func__, buffer_cnt_);

    std::chrono::nanoseconds wait_time(kBufferWaitTimeout);
    auto status = wait_for_buffer_.WaitFor(lock, wait_time);
    if (status != 0) {
      QMMF_ERROR("%s: Wait for output buffer return timed out", __func__);
    }
  }
  ret = GetBufferLocked(buffer);
  if (0 != ret) {
    QMMF_ERROR("%s: Failed to retrieve output buffer", __func__);
  }

  return ret;
}

status_t CameraRescalerMemPool::GetBufferLocked(StreamBuffer* buffer) {
  status_t ret = 0;
  int32_t idx = -1;
  IBufferHandle handle = nullptr;

  //Only pre-allocate buffers in case no valid stream Buffer
  //is passed as an argument.
  if (nullptr != buffer) {
    for (auto& it : mem_alloc_buffers_) {
      if(it.second == true) {
        handle = it.first;
        it.second = false;
        break;
      }
    }
  }
  // Find the slot of the available buffer.
  if (nullptr != handle) {
    for (uint32_t i = 0; i < buffers_allocated_; i++) {
      if (mem_alloc_slots_[i] == handle) {
        idx = i;
        break;
      }
    }
  } else if ((nullptr == handle) &&
             (buffers_allocated_ < buffer_cnt_)) {
    ret = AllocHWMemBuffer(handle);
    if (0 != ret) {
      return ret;
    }
    idx = buffers_allocated_;
    mem_alloc_slots_[idx] = handle;
    mem_alloc_buffers_.emplace(mem_alloc_slots_[idx], (nullptr == buffer));
    buffers_allocated_++;
  }

  if ((nullptr == handle) || (0 > idx)) {
    QMMF_ERROR("%s: Unable to allocate or find a free buffer!",
               __func__);
    return -ENOSYS;
  }

  if (nullptr != buffer) {
    buffer->handle = mem_alloc_slots_[idx];
    ret = PopulateBufferMeta(buffer->info, buffer->handle);
    if (0 != ret) {
      QMMF_ERROR("%s: Failed to populate buffer meta info", __func__);
      return ret;
    }
    buffer->fd = buffer->handle->GetFD();
    alloc_device_interface_->Perform(buffer->handle,
       IAllocDevice::AllocDeviceAction::GetMetaFd,
       static_cast<void*>(&buffer->metafd));
    buffer->size = buffer->handle->GetSize();
    ++pending_buffer_count_;
  }

  return ret;
}

status_t CameraRescalerMemPool::PopulateBufferMeta(BufferMeta &info,
                                                   IBufferHandle &handle) {

  int stride, scanline;
  auto ret = alloc_device_interface_->Perform(handle,
      IAllocDevice::AllocDeviceAction::GetAlignedHeight,
      static_cast<void*>(&scanline));
  if (MemAllocError::kAllocOk != ret) {
    QMMF_ERROR("%s: Unable to query stride&scanline: %d\n", __func__,
      (int32_t) ret);
    return -EINVAL;
  }

  ret = alloc_device_interface_->Perform(handle,
      IAllocDevice::AllocDeviceAction::GetStride,
      static_cast<void*>(&stride));
  if (MemAllocError::kAllocOk != ret) {
    QMMF_ERROR("%s: Unable to query stride&scanline: %d\n", __func__,
      (int32_t) ret);
    return -EINVAL;
  }

  switch (handle->GetFormat()) {
    case HAL_PIXEL_FORMAT_YCbCr_420_888:
    case HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED:
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS:
    case HAL_PIXEL_FORMAT_NV12_ENCODEABLE:
      info.format = BufferFormat::kNV12;
      info.n_planes = 2;
      info.planes[0].width = init_params_.width;
      info.planes[0].height = init_params_.height;
      info.planes[0].stride = stride;
      info.planes[0].scanline = scanline;
      info.planes[0].size = stride * scanline;
      info.planes[0].offset = 0;
      info.planes[1].width = init_params_.width;
      info.planes[1].height = init_params_.height/2;
      info.planes[1].stride = stride;
      info.planes[1].scanline = scanline / 2;
      info.planes[1].size = stride * (scanline / 2);
      info.planes[1].offset = stride * scanline;
      break;
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC:
      info.format = BufferFormat::kNV12UBWC;
      info.n_planes = 2;
      info.planes[0].width = init_params_.width;
      info.planes[0].height = init_params_.height;
      info.planes[0].stride = stride;
      info.planes[0].scanline = scanline;
      info.planes[0].size = stride * scanline;
      info.planes[0].offset = 0;
      info.planes[1].width = init_params_.width;
      info.planes[1].height = init_params_.height/2;
      info.planes[1].stride = stride;
      info.planes[1].scanline = scanline / 2;
      info.planes[1].size = stride * (scanline / 2);
      info.planes[1].offset = stride * scanline;
      break;
    case HAL_PIXEL_FORMAT_RGB_888:
      info.format = BufferFormat::kRGB;
      info.n_planes = 1;
      info.planes[0].width = init_params_.width;
      info.planes[0].height = init_params_.height;
      info.planes[0].stride = stride;
      info.planes[0].scanline = scanline;
      break;
    case HAL_PIXEL_FORMAT_NV21_ZSL:
      info.format = BufferFormat::kNV21;
      info.n_planes = 2;
      info.planes[0].width = init_params_.width;
      info.planes[0].height = init_params_.height;
      info.planes[0].stride = stride;
      info.planes[0].scanline = scanline;
      info.planes[0].size = stride * scanline;
      info.planes[0].offset = 0;
      info.planes[1].width = init_params_.width;
      info.planes[1].height = init_params_.height/2;
      info.planes[1].stride = stride;
      info.planes[1].scanline = scanline / 2;
      info.planes[1].size = stride * (scanline / 2);
      info.planes[1].offset = stride * scanline;
      break;
    case HAL_PIXEL_FORMAT_YCbCr_422_888:
      info.format = BufferFormat::kNV16;
      info.n_planes = 2;
      info.planes[0].width = init_params_.width;
      info.planes[0].height = init_params_.height;
      info.planes[0].stride = stride;
      info.planes[0].scanline = scanline;
      info.planes[0].size = stride * scanline;
      info.planes[0].offset = 0;
      info.planes[1].width = init_params_.width;
      info.planes[1].height = init_params_.height;
      info.planes[1].stride = stride;
      info.planes[1].scanline = scanline;
      info.planes[1].size = stride * scanline;
      info.planes[1].offset = stride * scanline;
      break;
    default:
      QMMF_ERROR("%s: Unsupported format: %d", __func__,
                 handle->GetFormat());
      return -ENOENT;
  }

  return 0;
}

status_t CameraRescalerMemPool::AllocHWMemBuffer(IBufferHandle &buf) {

  uint32_t width    = init_params_.width;
  uint32_t height   = init_params_.height;
  int32_t  format   = init_params_.format;
  MemAllocFlags usage(0);

  usage.flags |= IMemAllocUsage::kSwWriteOften | IMemAllocUsage::kSwReadOften;
  usage.flags |= IMemAllocUsage::kHwFb | IMemAllocUsage::kVideoEncoder;

  if (format == HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC) {
    usage.flags |= IMemAllocUsage::kPrivateAllocUbwc;
  }

  if (is_eis_on_ || is_ldc_on_) {
    usage.flags &= ~ (IMemAllocUsage::kPrivateAllocUbwc);
  }

  // Remove the CPU read/write flags since they are confusing GBM
  // when UBWC flag is set which causes the allocated buffer to be plain NV12
  if ((usage.flags | IMemAllocUsage::kPrivateAllocUbwc) != 0) {
    usage.flags &= ~(IMemAllocUsage::kSwWriteOften |
        IMemAllocUsage::kSwReadOften);
  }

  if (!width || !height) {
    width = height = 1;
  }

  uint32_t stride = 0;

  MemAllocError ret = alloc_device_interface_->AllocBuffer(buf,
#ifdef HAVE_BINDER
    static_cast<int>(width), static_cast<int>(height), format, usage, &stride);
#else
    static_cast<int>(width), static_cast<int>(height), format, 0, usage, &stride);
#endif
  if (MemAllocError::kAllocOk != ret) {
    QMMF_ERROR("%s: Failed to allocate alloc buffer", __func__);
    return -ENOMEM;
  }
#ifndef DISABLE_RESCALER_COLORSPACE
  int32_t color_space = ITU_R_601_FR;
  private_handle_t *priv_handle = const_cast<private_handle_t *>(
      static_cast<const private_handle_t *>(*buf));

  auto status = setMetaData(priv_handle, UPDATE_COLOR_SPACE,
                    static_cast<void *>(&color_space));

  if (0 != ret) {
    QMMF_ERROR("%s  setMetaData Failed: (%d)", __func__, status);
    return status;
  }
#endif
  return 0;
}

status_t CameraRescalerMemPool::FreeHWMemBuffer(IBufferHandle buf) {
  MemAllocError ret = alloc_device_interface_->FreeBuffer(buf);
  return ret == MemAllocError::kAllocOk ? 0 : -EINVAL;
}

CameraRescaler::CameraRescaler()
  : CameraRescalerBase(),
    is_stop_(false) {
  QMMF_INFO("%s: Enter", __func__);

  buffer_consumer_impl_ =
      std::make_shared<BufferConsumerImpl<CameraRescaler>>(this);

  buffer_producer_impl_ =
      std::make_shared<BufferProducerImpl<CameraRescaler>>(this);

  QMMF_INFO("%s: Exit (0x%p)", __func__, this);
}

CameraRescaler::~CameraRescaler() {
  QMMF_INFO("%s: Enter", __func__);
  buffer_producer_impl_.reset();
  buffer_consumer_impl_.reset();
  QMMF_INFO("%s: Exit (0x%p)", __func__, this);
}


status_t CameraRescaler::AddConsumer(const std::shared_ptr<IBufferConsumer>& consumer) {
  if (consumer.get() == nullptr) {
    QMMF_ERROR("%s: Input consumer is nullptr", __func__);
    return -EINVAL;
  }

  buffer_producer_impl_->AddConsumer(consumer);
  consumer->SetProducerHandle(buffer_producer_impl_);
  QMMF_VERBOSE("%s: Consumer(%p) has been added.", __func__,
      consumer.get());

  return 0;
}

uint32_t CameraRescaler::GetNumConsumer() {
  return buffer_producer_impl_->GetNumConsumer();
}

status_t CameraRescaler::RemoveConsumer(std::shared_ptr<IBufferConsumer>& consumer) {
  if(buffer_producer_impl_->GetNumConsumer() == 0) {
    QMMF_ERROR("%s: There are no connected consumers!", __func__);
    return -ENOSYS;
  }
  buffer_producer_impl_->RemoveConsumer(consumer);

  return 0;
}

std::shared_ptr<IBufferConsumer>& CameraRescaler::GetConsumer() {
  return buffer_consumer_impl_;
}

void CameraRescaler::OnFrameAvailable(StreamBuffer& buffer) {
  QMMF_DEBUG("%s: Camera %u: Frame %d is available",
      __func__, buffer.camera_id, buffer.frame_number);

  if (IsStop()) {
    QMMF_DEBUG("%s: IsStop", __func__);
    ReturnBufferToProducer(buffer);
    return;
  }

  AddBuf(buffer);
}

void CameraRescaler::NotifyBufferReturned(const StreamBuffer& buffer) {
  QMMF_DEBUG("%s: Stream buffer(handle %p) returned", __func__,
      buffer.handle);
  ReturnBufferToBufferPool(buffer);
}

status_t CameraRescaler::NotifyBufferToClient(StreamBuffer &buffer) {
  status_t ret = 0;
  std::lock_guard<std::mutex> lock(consumer_lock_);
  if(buffer_producer_impl_->GetNumConsumer() > 0) {
    buffer_producer_impl_->NotifyBuffer(buffer);
  } else {
    QMMF_DEBUG("%s: No consumer, simply return buffer back to"
        " memory pool!",  __func__);
    ret = ReturnBufferToBufferPool(buffer);
  }
  return ret;
}

status_t CameraRescaler::ReturnBufferToProducer(StreamBuffer &buffer) {
  QMMF_DEBUG("%s: Enter", __func__);
  const std::shared_ptr<IBufferConsumer> consumer = buffer_consumer_impl_;
  if (consumer.get() == nullptr) {
    QMMF_ERROR("%s: Failed to retrieve buffer consumer for camera(%d)!",
               __func__, buffer.camera_id);
    return -EINVAL;
  }
  consumer->GetProducerHandle()->NotifyBufferReturned(buffer);

  QMMF_DEBUG("%s: Exit", __func__);
  return 0;
}

status_t CameraRescaler::Start() {
  QMMF_INFO("%s: Enter", __func__);
  std::lock_guard<std::mutex> lock(stop_lock_);
  is_stop_ = false;
  QMMF_INFO("%s: Start thread", __func__);
  Run(LOG_TAG);
  QMMF_INFO("%s: Exit", __func__);
  return 0;
}

status_t CameraRescaler::Stop() {
  QMMF_INFO("%s: Enter", __func__);
  {
    std::lock_guard<std::mutex> lock(stop_lock_);
    is_stop_ = true;
  }
  QMMF_INFO("%s: Stop thread", __func__);
  RequestExitAndWait();
  QMMF_INFO("%s: Stop thread done", __func__);
  UnMapBufs();
  QMMF_INFO("%s: unmap done", __func__);
  QMMF_INFO("%s: Exit", __func__);
  return 0;
}

bool CameraRescaler::IsStop() {
  std::lock_guard<std::mutex> lock(stop_lock_);
  return is_stop_;
}

status_t CameraRescaler::Init(const uint32_t& width, const uint32_t& height,
                              const BufferFormat& fmt,
                              const float& in_fps, const float& out_fps,
                              const CameraExtraParam& extra_param) {

  if ((width == 0) || (height == 0)) {
    QMMF_ERROR("%s: Invalid dimensions: %ux%u!", __func__, width, height);
    return -EINVAL;
  }

  if (Validate(width, height, fmt) != 0) {
    QMMF_ERROR("%s: Error: Unsupported in params.!!!", __func__);
    return -EINVAL;
  }

  auto format = Common::FromQmmfToHalFormat(fmt);

  auto ret = Initialize(width, height, format, extra_param);
  return ret;
}

}; //namespace recorder

}; //namespace qmmf
