/*
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 * Not a Contribution.
 */

/*
 * Copyright (C) 2013 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Changes from Qualcomm Technologies, Inc. are provided under the following license:
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "qmmf_camera3_utils.h"
#include "qmmf_camera3_monitor.h"
#include "qmmf_camera3_stream.h"
#include "qmmf_memory_interface.h"
#include "recorder/src/service/qmmf_recorder_common.h"

#ifdef HAVE_MMM_COLOR_FMT_H
#include <display/media/mmm_color_fmt.h>
#else
#ifdef HAVE_BINDER
#include <media/msm_media_info.h>
#endif
#define MMM_COLOR_FMT_NV12_UBWC COLOR_FMT_NV12_UBWC
#define MMM_COLOR_FMT_NV12_BPP10_UBWC COLOR_FMT_NV12_BPP10_UBWC
#define MMM_COLOR_FMT_ALIGN MSM_MEDIA_ALIGN
#define MMM_COLOR_FMT_UV_SCANLINES VENUS_UV_SCANLINES
#define MMM_COLOR_FMT_Y_META_STRIDE VENUS_Y_META_STRIDE
#define MMM_COLOR_FMT_Y_META_SCANLINES VENUS_Y_META_SCANLINES
#define MMM_COLOR_FMT_UV_META_STRIDE VENUS_UV_META_STRIDE
#define MMM_COLOR_FMT_UV_META_SCANLINES VENUS_UV_META_SCANLINES
#endif

#ifdef HAVE_ANDROID_UTILS
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif // HAVE_ANDROID_UTILS

#define LOG_TAG "Camera3Stream"

namespace qmmf {

namespace cameraadaptor {

buffer_handle_t* Camera3Stream::DUMMY_BUFFER = (buffer_handle_t *)0xFFFFFFFF;

Camera3Stream::Camera3Stream(int id, size_t maxSize,
                             const CameraStreamParameters &outputConfiguration,
                             IAllocDevice *device,
                             Camera3Monitor &monitor)
    : camera3_stream(),
      mem_alloc_interface_(nullptr),
      current_buffer_stride_(0),
      id_(id),
      max_size_(maxSize),
      status_(STATUS_INTIALIZED),
      total_buffer_count_(0),
      pending_buffer_count_(0),
      hal_buffer_cnt_(0),
      client_buffer_cnt_(0),
      callbacks_(outputConfiguration.cb),
      old_usage_(),
      client_usage_(outputConfiguration.allocFlags),
      old_max_buffers_(0),
      client_max_buffers_(outputConfiguration.bufferCount),
      mem_alloc_slots_(NULL),
      hw_buffer_allocated_(0),
      monitor_(monitor),
      monitor_id_(Camera3Monitor::INVALID_ID),
      is_stream_active_(false),
      is_stream_idle_(true),
      prepared_buffers_count_(0),
      dummy_buffer_{},
      stream_camera_id() {
  camera3_stream::stream_type = CAMERA3_STREAM_OUTPUT;
  camera3_stream::width = outputConfiguration.width;
  camera3_stream::height = outputConfiguration.height;
  camera3_stream::format = outputConfiguration.format;
  camera3_stream::data_space = outputConfiguration.data_space;
  data_space_ = outputConfiguration.data_space;
#if defined(CAMX_ANDROID_API) && (CAMX_ANDROID_API >= 33)
  camera3_stream::color_space = outputConfiguration.color_space;
  color_space_ = outputConfiguration.color_space;
#endif
#if defined(CAMX_ANDROID_API) && (CAMX_ANDROID_API >= 31)
  camera3_stream::stream_use_case = outputConfiguration.usecase;
  camera3_stream::dynamic_range_profile = outputConfiguration.hdrmode;
  hdrmode_ = outputConfiguration.hdrmode;
#endif
  camera3_stream::rotation = outputConfiguration.rotation;
  camera3_stream::usage =
    AllocUsageFactory::GetAllocUsage().ToGralloc(outputConfiguration.allocFlags);
  camera3_stream::max_buffers = outputConfiguration.bufferCount;
#ifdef CAM_ARCH_V2
  if (HAL_PIXEL_FORMAT_BLOB == outputConfiguration.format) {
    camera3_stream::data_space = HAL_DATASPACE_V0_JFIF;
  }
  camera3_stream::priv = nullptr;
#endif
#if defined(CAMERA_HAL_API_VERSION) && (CAMERA_HAL_API_VERSION >= 0x0307)
  camera3_stream::group_id  = -1;
#endif
  stream_camera_id = outputConfiguration.stream_camera_id;
  if (stream_camera_id.empty())
    camera3_stream::physical_camera_id = nullptr;
  else
    camera3_stream::physical_camera_id = stream_camera_id.c_str();

  if ((HAL_PIXEL_FORMAT_BLOB == format) && (0 == maxSize)) {
    QMMF_ERROR("%s: blob with zero size\n", __func__);
    status_ = STATUS_ERROR;
  }

  if (NULL == device) {
    QMMF_ERROR("%s:Memory allocator device is invalid!\n", __func__);
    status_ = STATUS_ERROR;
  }

  mem_alloc_interface_ = device;

  dummy_buffer_.stream = this;
  dummy_buffer_.buffer = DUMMY_BUFFER;
  dummy_buffer_.acquire_fence = -1;
  dummy_buffer_.release_fence = -1;
  dummy_buffer_.status = CAMERA3_BUFFER_STATUS_OK;

  pthread_mutex_init(&lock_, NULL);
  cond_init(&output_buffer_returned_signal_);
  cond_init(&idle_signal_);
}

Camera3Stream::~Camera3Stream() {
  if (0 <= monitor_id_) {
    monitor_.ReleaseMonitor(monitor_id_);
    monitor_id_ = Camera3Monitor::INVALID_ID;
  }

  CloseLocked();

  pthread_mutex_destroy(&lock_);
  pthread_cond_destroy(&output_buffer_returned_signal_);
  pthread_cond_destroy(&idle_signal_);
  if (NULL != mem_alloc_slots_) {
    delete[] mem_alloc_slots_;
  }
}

camera3_stream *Camera3Stream::BeginConfigure() {
  pthread_mutex_lock(&lock_);

  switch (status_) {
    case STATUS_ERROR:
      QMMF_ERROR("%s: Error status\n", __func__);
      goto exit;
    case STATUS_INTIALIZED:
      break;
    case STATUS_CONFIG_ACTIVE:
    case STATUS_RECONFIG_ACTIVE:
      goto done;
    case STATUS_CONFIGURED:
      break;
    default:
      QMMF_ERROR("%s: Unknown status %d", __func__, status_);
      goto exit;
  }

  camera3_stream::usage =
    AllocUsageFactory::GetAllocUsage().ToGralloc(client_usage_);
  camera3_stream::max_buffers = client_max_buffers_;

  if (monitor_id_ != Camera3Monitor::INVALID_ID) {
    monitor_.ReleaseMonitor(monitor_id_);
    monitor_id_ = Camera3Monitor::INVALID_ID;
  }

  if (status_ == STATUS_INTIALIZED) {
    status_ = STATUS_CONFIG_ACTIVE;
  } else {
    if (status_ != STATUS_CONFIGURED) {
      QMMF_ERROR("%s: Invalid state: 0x%x \n", __func__, status_);
      goto exit;
    }
    status_ = STATUS_RECONFIG_ACTIVE;
  }

done:
  pthread_mutex_unlock(&lock_);

  return this;

exit:
  pthread_mutex_unlock(&lock_);
  return NULL;
}

bool Camera3Stream::IsConfigureActive() {
  pthread_mutex_lock(&lock_);
  bool ret =
      (status_ == STATUS_CONFIG_ACTIVE) || (status_ == STATUS_RECONFIG_ACTIVE);
  pthread_mutex_unlock(&lock_);
  return ret;
}

int32_t Camera3Stream::EndConfigure() {
  int32_t res;
  pthread_mutex_lock(&lock_);
  switch (status_) {
    case STATUS_ERROR:
      QMMF_ERROR("%s: Error status\n", __func__);
      res = -ENOSYS;
      goto exit;
    case STATUS_CONFIG_ACTIVE:
    case STATUS_RECONFIG_ACTIVE:
      break;
    case STATUS_INTIALIZED:
    case STATUS_CONFIGURED:
      QMMF_ERROR("%s: Configuration didn't start before!\n", __func__);
      res = -ENOSYS;
      goto exit;
    default:
      QMMF_ERROR("%s: Unknown status", __func__);
      res = -ENOSYS;
      goto exit;
  }

  monitor_id_ = monitor_.AcquireMonitor();
  if (0 > monitor_id_) {
    QMMF_ERROR("%s: Unable to acquire monitor: %d\n", __func__, monitor_id_);
    res = monitor_id_;
    goto exit;
  }

  if (status_ == STATUS_RECONFIG_ACTIVE &&
      client_usage_.Equals(old_usage_) &&
      old_max_buffers_ == camera3_stream::max_buffers) {
    status_ = STATUS_CONFIGURED;
    res = 0;
    goto exit;
  }

  res = ConfigureLocked();
  if (0 != res) {
    QMMF_ERROR("%s: Unable to configure stream %d\n", __func__, id_);
    status_ = STATUS_ERROR;
    goto exit;
  }

  status_ = STATUS_CONFIGURED;
  old_usage_ = client_usage_;
  old_max_buffers_ = camera3_stream::max_buffers;

exit:
  pthread_mutex_unlock(&lock_);

  return res;
}

int32_t Camera3Stream::AbortConfigure() {
  int32_t res;
  pthread_mutex_lock(&lock_);
  switch (status_) {
    case STATUS_ERROR:
      QMMF_ERROR("%s: Error status\n", __func__);
      res = -ENOSYS;
      goto exit;
    case STATUS_CONFIG_ACTIVE:
    case STATUS_RECONFIG_ACTIVE:
      break;
    case STATUS_INTIALIZED:
    case STATUS_CONFIGURED:
      QMMF_ERROR("%s: Cannot abort configure that is not started\n", __func__);
      res = -ENOSYS;
      goto exit;
    default:
      QMMF_ERROR("%s: Unknown status\n", __func__);
      res = -ENOSYS;
      goto exit;
  }

  client_usage_ = old_usage_;
  camera3_stream::usage =
    AllocUsageFactory::GetAllocUsage().ToGralloc(old_usage_);
  camera3_stream::max_buffers = old_max_buffers_;

  status_ = (status_ == STATUS_RECONFIG_ACTIVE) ? STATUS_CONFIGURED
                                                : STATUS_INTIALIZED;
  pthread_mutex_unlock(&lock_);
  return 0;

exit:

  pthread_mutex_unlock(&lock_);
  return res;
}

int32_t Camera3Stream::BeginPrepare() {
  int32_t res = 0;

  pthread_mutex_lock(&lock_);

  if (STATUS_CONFIGURED != status_) {
    QMMF_ERROR("%s: Stream %d: Cannot prepare unconfigured stream with "
        "status: %d\n", __func__, id_, status_);
      res = -ENOSYS;
      goto exit;
  }

  if (is_stream_active_) {
    QMMF_ERROR("%s: Stream %d: Cannot prepare already active stream\n",
               __func__, id_);
    res = -ENOSYS;
    goto exit;
  }

  if (0 < GetPendingBufferCountLocked()) {
    QMMF_ERROR("%s: Stream %d: Cannot prepare stream with pending buffers\n",
               __func__, id_);
    res = -ENOSYS;
    goto exit;
  }

  prepared_buffers_count_ = 0;
  status_ = STATUS_PREPARE_ACTIVE;
  res = -ENODATA;

exit:

  pthread_mutex_unlock(&lock_);

  return res;
}

int32_t Camera3Stream::PrepareBuffer() {
  int32_t res = 0;

  pthread_mutex_lock(&lock_);

  if (STATUS_PREPARE_ACTIVE != status_) {
    QMMF_ERROR("%s: Stream %d: Invalid status: %d\n", __func__, id_, status_);
    res = -ENOSYS;
    goto exit;
  }

  res = GetBufferLocked();
  if (0 != res) {
    QMMF_ERROR("%s: Stream %d: Failed to pre-allocate buffer %d", __func__,
               id_, prepared_buffers_count_);
    res = -ENODEV;
    goto exit;
  }

  prepared_buffers_count_++;

  if (prepared_buffers_count_ < total_buffer_count_) {
    res = -ENODATA;
    goto exit;
  }

  res = EndPrepareLocked();

exit:

  pthread_mutex_unlock(&lock_);

  return res;
}

int32_t Camera3Stream::EndPrepare() {
  pthread_mutex_lock(&lock_);

  int32_t res = EndPrepareLocked();

  pthread_mutex_unlock(&lock_);

  return res;
}

int32_t Camera3Stream::EndPrepareLocked() {
  if (STATUS_PREPARE_ACTIVE != status_) {
    QMMF_ERROR("%s: Stream %d: Cannot abort stream prepare with wrong"
        "status: %d\n", __func__, id_, status_);
    return -ENOSYS;
  }

  prepared_buffers_count_ = 0;
  status_ = STATUS_CONFIGURED;

  return 0;
}

bool Camera3Stream::IsPrepareActive() {
  pthread_mutex_lock(&lock_);

  bool res = (STATUS_PREPARE_ACTIVE == status_);

  pthread_mutex_unlock(&lock_);

  return res;
}

bool Camera3Stream::IsStreamActive() {
  pthread_mutex_lock(&lock_);

  bool res = is_stream_active_;

  pthread_mutex_unlock(&lock_);

  return res;
}

int32_t Camera3Stream::TearDown() {
  int32_t res = 0;

  pthread_mutex_lock(&lock_);

  if (status_ != STATUS_CONFIGURED) {
    QMMF_ERROR(
        "%s: Stream %d: Cannot be torn down when stream"
        "is still un-configured: %d\n",
        __func__, id_, status_);
    res = -ENOSYS;
    goto exit;
  }

  if (0 < GetPendingBufferCountLocked()) {
    QMMF_ERROR(
        "%s: Stream %d: Cannot be torn down while buffers are still pending\n",
        __func__, id_);
    res = -ENOSYS;
    goto exit;
  }

  if (0 < hw_buffer_allocated_) {
    assert(nullptr != mem_alloc_interface_);
    for (auto it = mem_alloc_buffers_.begin(); it != mem_alloc_buffers_.end(); ++it) {
      mem_alloc_interface_->FreeBuffer(it->first);
    }
    mem_alloc_buffers_.clear();
    hw_buffer_allocated_ = 0;
  }

  for (uint32_t i = 0; i < total_buffer_count_; i++) {
    mem_alloc_slots_[i] = NULL;
  }

  is_stream_active_ = false;

exit:

  pthread_mutex_unlock(&lock_);

  return res;
}

int32_t Camera3Stream::GetDummyBuffer(camera3_stream_buffer *buffer) {
  int32_t res = 0;

  pthread_mutex_lock(&lock_);

  if (status_ != STATUS_CONFIGURED) {
    QMMF_ERROR(
        "%s: Stream %d: Can't retrieve buffer when stream"
        "is not configured%d\n",
        __func__, id_, status_);
    res = -ENOSYS;
    goto exit;
  }

  *buffer = dummy_buffer_;

  if (is_stream_idle_ && status_ != STATUS_CONFIG_ACTIVE &&
      status_ != STATUS_RECONFIG_ACTIVE) {
    monitor_.ChangeStateToActive(monitor_id_);
    is_stream_idle_ = false;
  }

exit:

  pthread_mutex_unlock(&lock_);
  return res;
}

int32_t Camera3Stream::GetBuffer(camera3_stream_buffer *buffer) {
  int32_t res = 0;

  pthread_mutex_lock(&lock_);

  if (status_ != STATUS_CONFIGURED) {
    QMMF_ERROR(
        "%s: Stream %d: Can't retrieve buffer when stream"
        "is not configured%d\n",
        __func__, id_, status_);
    res = -ENOSYS;
    goto exit;
  }

  if (GetPendingBufferCountLocked() == total_buffer_count_) {
    QMMF_DEBUG(
        "%s: Already retrieved maximum buffers (%d), waiting on a"
        "free one\n",
        __func__, total_buffer_count_);
    res = cond_wait_relative(&output_buffer_returned_signal_, &lock_,
                             BUFFER_WAIT_TIMEOUT);
    if (res != 0) {
      if (-ETIMEDOUT == res) {
        QMMF_ERROR("%s: wait for output buffer return timed out\n", __func__);
        PrintBuffersInfoLocked();
      }
      goto exit;
    }
  }

  res = GetBufferLocked(buffer);

exit:

  pthread_mutex_unlock(&lock_);
  return res;
}

int32_t Camera3Stream::PopulateBufferMeta(BufferMeta &info,
                                          IBufferHandle &handle) {
  int stride, scanline;
  auto ret = mem_alloc_interface_->Perform(handle,
                                      IAllocDevice::AllocDeviceAction::GetStride,
                                      static_cast<void*>(&stride));

  if (MemAllocError::kAllocOk != ret) {
    QMMF_ERROR("%s: Error in GetStrideAndHeightFromHandle() : %ld\n", __func__,
               static_cast<int64_t>(ret));
    return -EINVAL;
  }
  ret = mem_alloc_interface_->Perform(handle,
                            IAllocDevice::AllocDeviceAction::GetAlignedHeight,
                            static_cast<void*>(&scanline));
  if (MemAllocError::kAllocOk != ret) {
    QMMF_ERROR("%s: Error in GetStrideAndHeightFromHandle() : %ld\n", __func__,
               static_cast<int64_t>(ret));
    return -EINVAL;
  }

  QMMF_DEBUG("%s: format(0x%x)", __func__, handle->GetFormat());

  switch (handle->GetFormat()) {
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_2_BATCH:
    case HAL_PIXEL_FORMAT_NV12_FLEX_2_BATCH:
    case HAL_PIXEL_FORMAT_P010_FLEX_2_BATCH:
    case HAL_PIXEL_FORMAT_TP10_UBWC_FLEX_2_BATCH:
      info.n_frames = 2;
      break;
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_4_BATCH:
    case HAL_PIXEL_FORMAT_NV12_FLEX_4_BATCH:
    case HAL_PIXEL_FORMAT_P010_FLEX_4_BATCH:
    case HAL_PIXEL_FORMAT_TP10_UBWC_FLEX_4_BATCH:
      info.n_frames = 4;
      break;
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_8_BATCH:
    case HAL_PIXEL_FORMAT_NV12_FLEX_8_BATCH:
    case HAL_PIXEL_FORMAT_P010_FLEX_8_BATCH:
    case HAL_PIXEL_FORMAT_TP10_UBWC_FLEX_8_BATCH:
      info.n_frames = 8;
      break;
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX:
    case HAL_PIXEL_FORMAT_NV12_FLEX:
    case HAL_PIXEL_FORMAT_P010_FLEX:
    case HAL_PIXEL_FORMAT_TP10_UBWC_FLEX:
      info.n_frames = 16;
      break;
  }

  switch (handle->GetFormat()) {
    case HAL_PIXEL_FORMAT_BLOB:
      info.format = BufferFormat::kBLOB;
      info.n_planes = 1;
      info.planes[0].width = width;
      info.planes[0].height = height;
      info.planes[0].stride = stride;
      info.planes[0].scanline = scanline;
      info.planes[0].size = max_size_;
      info.planes[0].offset = 0;
      break;
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS:
    case HAL_PIXEL_FORMAT_NV12_ENCODEABLE:
    case HAL_PIXEL_FORMAT_YCbCr_420_888:
    case HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED:
      info.format = BufferFormat::kNV12;
      info.n_planes = 2;
      info.planes[0].width = width;
      info.planes[0].height = height;
      info.planes[0].stride = stride;
      info.planes[0].scanline = scanline;
      info.planes[0].size = stride * scanline;
      info.planes[0].offset = 0;
      info.planes[1].width = width;
      info.planes[1].height = height / 2;
      info.planes[1].stride = stride;
      info.planes[1].scanline = scanline / 2;
      info.planes[1].size = stride * (scanline / 2);
      info.planes[1].offset = stride * scanline;
      break;
    case HAL_PIXEL_FORMAT_NV12_FLEX_2_BATCH:
    case HAL_PIXEL_FORMAT_NV12_FLEX_4_BATCH:
    case HAL_PIXEL_FORMAT_NV12_FLEX_8_BATCH:
    case HAL_PIXEL_FORMAT_NV12_FLEX:
      info.format = BufferFormat::kNV12FLEX;
      info.n_planes = 2;
      info.planes[0].width = width;
      info.planes[0].height = height;
      info.planes[0].stride = stride;
      info.planes[0].scanline = scanline;
      info.planes[0].size = stride * scanline;
      info.planes[0].offset = 0;
      info.planes[1].width = width;
      info.planes[1].height = height / 2;
      info.planes[1].stride = stride;
      info.planes[1].scanline = scanline / 2;
      info.planes[1].size = stride * (scanline / 2);
      info.planes[1].offset = stride * scanline;
      break;
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC:
      info.format = BufferFormat::kNV12UBWC;
      info.n_planes = 2;
      info.planes[0].width = width;
      info.planes[0].height = height;
      info.planes[0].stride = stride;
      info.planes[0].scanline = scanline;
      info.planes[0].size = MMM_COLOR_FMT_ALIGN((stride * scanline), 4096) +
          MMM_COLOR_FMT_ALIGN((MMM_COLOR_FMT_Y_META_STRIDE(MMM_COLOR_FMT_NV12_UBWC, width) *
          MMM_COLOR_FMT_Y_META_SCANLINES(MMM_COLOR_FMT_NV12_UBWC, height)), 4096);
      info.planes[0].offset = 0;
      info.planes[1].width = width;
      info.planes[1].height = height / 2;
      info.planes[1].stride = stride;
      info.planes[1].scanline = scanline / 2;
      info.planes[1].size = MMM_COLOR_FMT_ALIGN((stride * scanline / 2), 4096) +
          MMM_COLOR_FMT_ALIGN((MMM_COLOR_FMT_UV_META_STRIDE(MMM_COLOR_FMT_NV12_UBWC, width) *
          MMM_COLOR_FMT_UV_META_SCANLINES(MMM_COLOR_FMT_NV12_UBWC, height)), 4096);
      info.planes[1].offset = info.planes[0].offset + info.planes[0].size;
      break;
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_2_BATCH:
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_4_BATCH:
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_8_BATCH:
    case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX:
      info.format = BufferFormat::kNV12UBWCFLEX;
      info.n_planes = 2;
      info.planes[0].width = width;
      info.planes[0].height = height;
      info.planes[0].stride = stride;
      info.planes[0].scanline = scanline;
      info.planes[0].size = MMM_COLOR_FMT_ALIGN((stride * scanline), 4096) +
          MMM_COLOR_FMT_ALIGN((MMM_COLOR_FMT_Y_META_STRIDE(MMM_COLOR_FMT_NV12_UBWC, width) *
          MMM_COLOR_FMT_Y_META_SCANLINES(MMM_COLOR_FMT_NV12_UBWC, height)), 4096);
      info.planes[0].offset = 0;
      info.planes[1].width = width;
      info.planes[1].height = height / 2;
      info.planes[1].stride = stride;
      info.planes[1].scanline = MMM_COLOR_FMT_UV_SCANLINES(MMM_COLOR_FMT_NV12_UBWC, height);;
      info.planes[1].size = MMM_COLOR_FMT_ALIGN((stride * info.planes[1].scanline), 4096) +
          MMM_COLOR_FMT_ALIGN((MMM_COLOR_FMT_UV_META_STRIDE(MMM_COLOR_FMT_NV12_UBWC, width) *
          MMM_COLOR_FMT_UV_META_SCANLINES(MMM_COLOR_FMT_NV12_UBWC, height)), 4096);
      info.planes[1].offset = info.planes[0].offset + info.planes[0].size;
      break;
    case HAL_PIXEL_FORMAT_YCbCr_422_I_10BIT:
      info.format = BufferFormat::kP010;
      info.n_planes = 2;
      info.planes[0].width = width;
      info.planes[0].height = height;
      info.planes[0].stride = stride;
      info.planes[0].scanline = scanline;
      info.planes[0].size = MMM_COLOR_FMT_ALIGN((stride * scanline), 4096);
      info.planes[0].offset = 0;
      info.planes[1].width = width;
      info.planes[1].height = height / 2;
      info.planes[1].stride = stride;
      info.planes[1].scanline = scanline / 2;
      info.planes[1].size = MMM_COLOR_FMT_ALIGN((stride * scanline / 2), 4096);
      info.planes[1].offset = info.planes[0].offset + info.planes[0].size;
      break;
    case HAL_PIXEL_FORMAT_P010_FLEX_2_BATCH:
    case HAL_PIXEL_FORMAT_P010_FLEX_4_BATCH:
    case HAL_PIXEL_FORMAT_P010_FLEX_8_BATCH:
    case HAL_PIXEL_FORMAT_P010_FLEX:
      info.format = BufferFormat::kP010FLEX;
      info.n_planes = 2;
      info.planes[0].width = width;
      info.planes[0].height = height;
      info.planes[0].stride = stride;
      info.planes[0].scanline = scanline;
      info.planes[0].size = MMM_COLOR_FMT_ALIGN((stride * scanline), 4096);
      info.planes[0].offset = 0;
      info.planes[1].width = width;
      info.planes[1].height = height / 2;
      info.planes[1].stride = stride;
      info.planes[1].scanline = scanline / 2;
      info.planes[1].size = MMM_COLOR_FMT_ALIGN((stride * scanline / 2), 4096);
      info.planes[1].offset = info.planes[0].offset + info.planes[0].size;
      break;
    case HAL_PIXEL_FORMAT_YCbCr_420_TP10_UBWC:
      info.format = BufferFormat::kTP10UBWC;
      info.n_planes = 2;
      info.planes[0].width = width;
      info.planes[0].height = height;
      info.planes[0].stride = stride;
      info.planes[0].scanline = scanline;
      info.planes[0].size = MMM_COLOR_FMT_ALIGN((stride * scanline), 4096) +
          MMM_COLOR_FMT_ALIGN((MMM_COLOR_FMT_Y_META_STRIDE(MMM_COLOR_FMT_NV12_BPP10_UBWC, width) *
          MMM_COLOR_FMT_Y_META_SCANLINES(MMM_COLOR_FMT_NV12_BPP10_UBWC, height)), 4096);
      info.planes[0].offset = 0;
      info.planes[1].width = width;
      info.planes[1].height = height / 2;
      info.planes[1].stride = stride;
      info.planes[1].scanline = scanline / 2;
      info.planes[1].size = MMM_COLOR_FMT_ALIGN((stride * scanline / 2), 4096) +
          MMM_COLOR_FMT_ALIGN((MMM_COLOR_FMT_UV_META_STRIDE(MMM_COLOR_FMT_NV12_BPP10_UBWC, width) *
          MMM_COLOR_FMT_UV_META_SCANLINES(MMM_COLOR_FMT_NV12_BPP10_UBWC, height)), 4096);
      info.planes[1].offset = info.planes[0].offset + info.planes[0].size;
      break;
    case HAL_PIXEL_FORMAT_TP10_UBWC_FLEX_2_BATCH:
    case HAL_PIXEL_FORMAT_TP10_UBWC_FLEX_4_BATCH:
    case HAL_PIXEL_FORMAT_TP10_UBWC_FLEX_8_BATCH:
    case HAL_PIXEL_FORMAT_TP10_UBWC_FLEX:
      info.format = BufferFormat::kTP10UBWCFLEX;
      info.n_planes = 2;
      info.planes[0].width = width;
      info.planes[0].height = height;
      info.planes[0].stride = stride;
      info.planes[0].scanline = scanline;
      info.planes[0].size = MMM_COLOR_FMT_ALIGN((stride * scanline), 4096) +
          MMM_COLOR_FMT_ALIGN((MMM_COLOR_FMT_Y_META_STRIDE(MMM_COLOR_FMT_NV12_BPP10_UBWC, width) *
          MMM_COLOR_FMT_Y_META_SCANLINES(MMM_COLOR_FMT_NV12_BPP10_UBWC, height)), 4096);
      info.planes[0].offset = 0;
      info.planes[1].width = width;
      info.planes[1].height = height / 2;
      info.planes[1].stride = stride;
      info.planes[1].scanline = scanline / 2;
      info.planes[1].size = MMM_COLOR_FMT_ALIGN((stride * scanline / 2), 4096) +
          MMM_COLOR_FMT_ALIGN((MMM_COLOR_FMT_UV_META_STRIDE(MMM_COLOR_FMT_NV12_BPP10_UBWC, width) *
          MMM_COLOR_FMT_UV_META_SCANLINES(MMM_COLOR_FMT_NV12_BPP10_UBWC, height)), 4096);
      info.planes[1].offset = info.planes[0].offset + info.planes[0].size;
      break;
    case HAL_PIXEL_FORMAT_YCbCr_422_888:
    case HAL_PIXEL_FORMAT_YCbCr_422_SP:
      info.format = BufferFormat::kNV16;
      info.n_planes = 2;
      info.planes[0].width = width;
      info.planes[0].height = height;
      info.planes[0].stride = stride;
      info.planes[0].scanline = scanline;
      info.planes[0].size = stride * scanline;
      info.planes[0].offset = 0;
      info.planes[1].width = width;
      info.planes[1].height = height;
      info.planes[1].stride = stride;
      info.planes[1].scanline = scanline;
      info.planes[1].size = stride * scanline;
      info.planes[1].offset = stride * scanline;
      break;
    case HAL_PIXEL_FORMAT_NV21_ZSL:
      info.format = BufferFormat::kNV21;
      info.n_planes = 2;
      info.planes[0].width = width;
      info.planes[0].height = height;
      info.planes[0].stride = stride;
      info.planes[0].scanline = scanline;
      info.planes[0].size = stride * scanline;
      info.planes[0].offset = 0;
      info.planes[1].width = width;
      info.planes[1].height = height / 2;
      info.planes[1].stride = stride;
      info.planes[1].scanline = scanline / 2;
      info.planes[1].size = stride * (scanline / 2);
      info.planes[1].offset = stride * scanline;
      break;
    case HAL_PIXEL_FORMAT_RAW8:
      info.format = BufferFormat::kRAW8;
      info.n_planes = 1;
      info.planes[0].width = width;
      info.planes[0].height = height;
      info.planes[0].stride = stride;
      info.planes[0].scanline = scanline;
      info.planes[0].size = stride * scanline;
      info.planes[0].offset = 0;
      break;
    case HAL_PIXEL_FORMAT_RAW10:
      info.format = BufferFormat::kRAW10;
      info.n_planes = 1;
      info.planes[0].width = width;
      info.planes[0].height = height;
      info.planes[0].stride = stride;
      info.planes[0].scanline = scanline;
      info.planes[0].size = stride * scanline;
      info.planes[0].offset = 0;
      break;
    case HAL_PIXEL_FORMAT_RAW12:
      info.format = BufferFormat::kRAW12;
      info.n_planes = 1;
      info.planes[0].width = width;
      info.planes[0].height = height;
      info.planes[0].stride = stride;
      info.planes[0].scanline = scanline;
      info.planes[0].size = stride * scanline;
      info.planes[0].offset = 0;
      break;
    case HAL_PIXEL_FORMAT_RAW16:
      info.format = BufferFormat::kRAW16;
      info.n_planes = 1;
      info.planes[0].width = width;
      info.planes[0].height = height;
      info.planes[0].stride = stride;
      info.planes[0].scanline = scanline;
      info.planes[0].size = stride * scanline;
      info.planes[0].offset = 0;
      break;
    case HAL_PIXEL_FORMAT_YCbCr_422_I:
      info.format = BufferFormat::kYUY2;
      info.n_planes = 1;
      info.planes[0].width = width;
      info.planes[0].height = height;
      info.planes[0].stride = stride;
      info.planes[0].scanline = scanline;
      info.planes[0].size = stride * scanline;
      info.planes[0].offset = 0;
      break;
    case HAL_PIXEL_FORMAT_CbYCrY_422_I:
      info.format = BufferFormat::kUYVY;
      info.n_planes = 1;
      info.planes[0].width = width;
      info.planes[0].height = height;
      info.planes[0].stride = stride;
      info.planes[0].scanline = scanline;
      info.planes[0].size = stride * scanline;
      info.planes[0].offset = 0;
      break;
    case HAL_PIXEL_FORMAT_NV12_HEIF:
      info.format = BufferFormat::kNV12HEIF;
      info.n_planes = 2;
      info.planes[0].width = width;
      info.planes[0].height = height;
      info.planes[0].stride = stride;
      info.planes[0].scanline = scanline;
      info.planes[0].size = stride * scanline;
      info.planes[0].offset = 0;
      info.planes[1].width = width;
      info.planes[1].height = height / 2;
      info.planes[1].stride = stride;
      info.planes[1].scanline = scanline / 2;
      info.planes[1].size = stride * (scanline / 2);
      info.planes[1].offset = info.planes[0].size;
      break;
    case HAL_PIXEL_FORMAT_RGB_888:
      info.format = BufferFormat::kRGB;
      info.n_planes = 1;
      info.planes[0].width = width;
      info.planes[0].height = height;
      info.planes[0].stride = stride;
      info.planes[0].scanline = scanline;
      info.planes[0].size = stride * scanline;
      info.planes[0].offset = 0;
      break;
    default:
      QMMF_ERROR("%s: Unsupported format: %d\n", __func__,
                 handle->GetFormat());
      return -ENOENT;
  }

  return 0;
}

void Camera3Stream::ReturnBufferToClient(const camera3_stream_buffer &buffer,
                                         int64_t timestamp,
                                         int64_t frame_number) {
  assert(nullptr != callbacks_);

  pthread_mutex_lock(&lock_);

  if (buffer.buffer == DUMMY_BUFFER) {
    QMMF_INFO("%s: Camera HAL return dummy buffer", __func__);
    pthread_mutex_unlock(&lock_);
    return;
  }

  hal_buffer_cnt_--;
  client_buffer_cnt_++;

  if (status_ != STATUS_CONFIG_ACTIVE && status_ != STATUS_RECONFIG_ACTIVE) {
    if (hal_buffer_cnt_ == 0) {
      // notify hal is idle for this stream i.e. buffers are returned by hal
      QMMF_DEBUG("%s: Stream(%d): Changing state to idle", __func__, id_);
      monitor_.ChangeStateToIdle(monitor_id_);
    }
  }

  StreamBuffer b;
  memset(&b, 0, sizeof(b));
  b.timestamp = timestamp;
  b.frame_number = frame_number;
  b.stream_id = id_;
  b.data_space = data_space;
  b.handle = buffers_map[*buffer.buffer];
  assert(b.handle != nullptr);
  b.fd = b.handle->GetFD();
  b.size = b.handle->GetSize();
  b.in_use_client = false;
  b.in_use_camera = false;
  PopulateBufferMeta(b.info, b.handle);
  is_stream_active_ = true;

  mem_alloc_interface_->Perform(
      b.handle, IAllocDevice::AllocDeviceAction::GetMetaFd,
      static_cast<void*>(&b.metafd)
  );

  pthread_mutex_unlock(&lock_);

  if (CAMERA3_BUFFER_STATUS_OK == buffer.status) {
    callbacks_(b);
  } else {
    QMMF_WARN("%s: Got buffer(%p) from stream(%d), frame_number(%u) and "
        " ts(%ld) with error status!", __func__, b.handle, b.stream_id,
        b.frame_number, b.timestamp);
    ReturnBuffer(b);
  }
}

void Camera3Stream::ReturnBuffer(const buffer_handle_t &buffer) {

  pthread_mutex_lock(&lock_);

  hal_buffer_cnt_--;
  client_buffer_cnt_++;

  if (status_ != STATUS_CONFIG_ACTIVE && status_ != STATUS_RECONFIG_ACTIVE) {
    if (hal_buffer_cnt_ == 0) {
      // notify hal is idle for this stream i.e. buffers are returned by hal
      QMMF_DEBUG("%s: Stream(%d): Changing state to idle", __func__, id_);
      monitor_.ChangeStateToIdle(monitor_id_);
    }
  }

  StreamBuffer b;
  memset(&b, 0, sizeof(b));
  b.handle = buffers_map[buffer];
  assert(b.handle != nullptr);

  pthread_mutex_unlock(&lock_);

  ReturnBuffer(b);
}

int32_t Camera3Stream::ReturnBuffer(const StreamBuffer &buffer) {
  pthread_mutex_lock(&lock_);

  int32_t res = ReturnBufferLocked(buffer);
  if (res == 0) {
    pthread_cond_signal(&output_buffer_returned_signal_);
  }

  pthread_mutex_unlock(&lock_);
  return res;
}

int32_t Camera3Stream::Close() {
  pthread_mutex_lock(&lock_);
  int32_t res = CloseLocked();

  if (res == -ENOTCONN) {
    res = 0;
  }

  pthread_mutex_unlock(&lock_);
  return res;
}

int32_t Camera3Stream::ReturnBufferLocked(const StreamBuffer &buffer) {
  if (status_ == STATUS_INTIALIZED) {
    QMMF_ERROR(
        "%s: Stream %d: Can't return buffer when we only "
        "got initialized %d\n",
        __func__, id_, status_);
    return -ENOSYS;
  }

  if (pending_buffer_count_ == 0) {
    QMMF_ERROR("%s: Stream %d: Not expecting any buffers!\n", __func__, id_);
    return -ENOSYS;
  }

  if (mem_alloc_buffers_.count(buffer.handle) == 0) {
    QMMF_ERROR(
        "%s: Buffer %p returned that wasn't allocated by this"
        " stream!\n",
        __func__, buffer.handle);
    return -EINVAL;
  } else {
    mem_alloc_buffers_[buffer.handle] = true;
  }

  pending_buffer_count_--;
  client_buffer_cnt_--;
  QMMF_DEBUG("%s: Stream(%d): pending_buffer_count_(%u)", __func__, id_,
      pending_buffer_count_);

  if (status_ != STATUS_CONFIG_ACTIVE && status_ != STATUS_RECONFIG_ACTIVE) {
    if (pending_buffer_count_ == 0) {
      // notify stream is idle i.e. all buffers are returned
      QMMF_DEBUG("%s: Stream(%d): Stream is idle", __func__, id_);
      pthread_cond_signal(&idle_signal_);
      is_stream_idle_ = true;
    }
  }

  return 0;
}

void Camera3Stream::WaitForIdle() {
  int32_t res = 0;

  pthread_mutex_lock(&lock_);
  while (pending_buffer_count_) {
    auto res = cond_wait_relative(&idle_signal_, &lock_, BUFFER_WAIT_TIMEOUT);
    if (0 != res) {
      if (-ETIMEDOUT != res) {
        QMMF_ERROR("%s: wait for output buffer return timed out \n", __func__);
      } else {
        QMMF_ERROR("%s: Error during state change wait: %s (%d)\n", __func__,
                   strerror(-res), res);
      }
      PrintBuffersInfoLocked();
    }
  }
  pthread_mutex_unlock(&lock_);
}

int32_t Camera3Stream::GetBufferLocked(camera3_stream_buffer *streamBuffer) {
  int32_t idx = -1;
  if ((status_ != STATUS_CONFIGURED) && (status_ != STATUS_CONFIG_ACTIVE) &&
      (status_ != STATUS_RECONFIG_ACTIVE) &&
      (status_ != STATUS_PREPARE_ACTIVE)) {
    QMMF_ERROR(
        "%s: Stream %d: Can't get buffers before being"
        " configured  or preparing %d\n",
        __func__, id_, status_);
    return -ENOSYS;
  }

  IBufferHandle handle = NULL;
  //Only pre-allocate buffers in case no valid streamBuffer
  //is passed as an argument.
  if (NULL != streamBuffer) {
    for (auto it = mem_alloc_buffers_.begin(); it != mem_alloc_buffers_.end(); ++it) {
      if (it->second) {
        handle = it->first;
        it->second = false;
        break;
      }
    }
  }

  if (NULL != handle) {
    for (uint32_t i = 0; i < hw_buffer_allocated_; i++) {
      if (mem_alloc_slots_[i] == handle) {
        idx = i;
        break;
      }
    }
  } else if ((NULL == handle) &&
             (hw_buffer_allocated_ < total_buffer_count_)) {
    assert(nullptr != mem_alloc_interface_);
    // Blob buffers are expected to get allocated with width equal to blob
    // max size and height equal to 1.
    int32_t buf_width, buf_height;
    if (HAL_PIXEL_FORMAT_BLOB == camera3_stream::format) {
      buf_width = max_size_;
      buf_height = 1;
    } else {
      buf_width = camera3_stream::width;
      buf_height = camera3_stream::height;
    }
    MemAllocFlags memusage = client_usage_;

    // Remove the CPU read/write flags set by CamX since they are confusing GBM
    // when UBWC flag is set which causes the allocated buffer to be plain NV12.
    if (memusage.flags & IMemAllocUsage::kPrivateAllocUbwc) {
      memusage.flags &= ~(IMemAllocUsage::kHwCameraRead |
          IMemAllocUsage::kHwCameraWrite);
    }

    Colorimetry colorimetry = Colorimetry::kBT601;

#if defined(CAMX_ANDROID_API) && (CAMX_ANDROID_API >= 31)
    if (hdrmode_ == 0) {
      colorimetry = Colorimetry::kBT601;
    } else if (hdrmode_ == ANDROID_REQUEST_AVAILABLE_DYNAMIC_RANGE_PROFILES_MAP_HLG10) {
      colorimetry = Colorimetry::kBT2100HLGFULL;
    } else if (hdrmode_ == ANDROID_REQUEST_AVAILABLE_DYNAMIC_RANGE_PROFILES_MAP_HDR10) {
      colorimetry = Colorimetry::kBT2100PQFULL;
    } else if (hdrmode_ ==  ANDROID_REQUEST_AVAILABLE_DYNAMIC_RANGE_PROFILES_MAP_STANDARD) {
      if (data_space_ == HAL_DATASPACE_BT601_525
#if defined(ENABLE_IMAGE_NV12)
        || ((data_space_ == HAL_DATASPACE_UNKNOWN ||
          data_space_ == HAL_DATASPACE_HEIF) && color_space_ == 0)
#endif
        ) {
        colorimetry = Colorimetry::kBT601FULL;
      } else if (data_space_ == HAL_DATASPACE_BT709 || color_space_ == 4) {
        colorimetry = Colorimetry::kBT709FULL;
      } else {
        QMMF_ERROR("%s: Data space is not found in MAP_STANDARD.\n", __func__);
        return -ENOSYS;
      }
    } else {
      QMMF_ERROR("%s: HDR mode is not found. Still using BT601.\n", __func__);
    }
#endif

    QMMF_INFO("%s: Select Colorimetry = %ld", __func__, static_cast<int64_t>(colorimetry));

    MemAllocError ret = mem_alloc_interface_->AllocBuffer(
        handle,
        buf_width,
        buf_height,
        camera3_stream::format,
#ifndef HAVE_BINDER
        camera3_stream::override_format,
#endif
        memusage,
        &current_buffer_stride_,
        static_cast<uint32_t>(colorimetry));

    if (MemAllocError::kAllocOk != ret) {
      return -ENOMEM;
    }
    idx = hw_buffer_allocated_;
    mem_alloc_slots_[idx] = handle;
    mem_alloc_buffers_.emplace(mem_alloc_slots_[idx], (NULL == streamBuffer));
    hw_buffer_allocated_++;
    QMMF_INFO("%s: Allocated new buffer, total buffers allocated = %d",
        __func__, hw_buffer_allocated_);
  }

  if ((NULL == handle) || (0 > idx)) {
    QMMF_ERROR("%s: Unable to allocate or find a free buffer!\n", __func__);
    return -ENOSYS;
  }

  if (NULL != streamBuffer) {
    streamBuffer->stream = this;
    streamBuffer->acquire_fence = -1;
    streamBuffer->release_fence = -1;
    streamBuffer->status = CAMERA3_BUFFER_STATUS_OK;
#ifdef USE_LIBGBM
    streamBuffer->buffer = &GetGrallocBufferHandle(mem_alloc_slots_[idx]);
#else
    streamBuffer->buffer = &GetAllocBufferHandle(mem_alloc_slots_[idx]);
#endif // USE_LIBGBM

   buffers_map.emplace(*streamBuffer->buffer, mem_alloc_slots_[idx]);

    if (is_stream_idle_ && hal_buffer_cnt_ == 0 &&
        status_ != STATUS_CONFIG_ACTIVE &&
        status_ != STATUS_RECONFIG_ACTIVE) {
      monitor_.ChangeStateToActive(monitor_id_);
      is_stream_idle_ = false;
    }

    hal_buffer_cnt_++;
    pending_buffer_count_++;

    QMMF_DEBUG("%s: Stream(%d): pending_buffer_count_(%u)", __func__, id_,
        pending_buffer_count_);
  }

  return 0;
}

int32_t Camera3Stream::ConfigureLocked() {
  int32_t res;

  switch (status_) {
    case STATUS_RECONFIG_ACTIVE:
      res = CloseLocked();
      if (0 != res) {
        return res;
      }
      break;
    case STATUS_CONFIG_ACTIVE:
      break;
    default:
      QMMF_ERROR("%s: Bad status: %d\n", __func__, status_);
      return -ENOSYS;
  }

  total_buffer_count_ = MAX(client_max_buffers_, camera3_stream::max_buffers);
  pending_buffer_count_ = 0;
  hw_buffer_allocated_ = 0;
  is_stream_active_ = false;
  if (NULL != mem_alloc_slots_) {
    delete[] mem_alloc_slots_;
  }

  if (!mem_alloc_buffers_.empty()) {
    assert(nullptr != mem_alloc_interface_);
    for (auto it = mem_alloc_buffers_.begin(); it != mem_alloc_buffers_.end(); ++it) {
      mem_alloc_interface_->FreeBuffer(it->first);
    }
    mem_alloc_buffers_.clear();
  }

  mem_alloc_slots_ = new IBufferHandle[total_buffer_count_];
  if (NULL == mem_alloc_slots_) {
    QMMF_ERROR("%s: Unable to allocate buffer handles!\n", __func__);
    status_ = STATUS_ERROR;
    return -ENOMEM;
  }

  return 0;
}

int32_t Camera3Stream::CloseLocked() {
  switch (status_) {
    case STATUS_RECONFIG_ACTIVE:
    case STATUS_CONFIGURED:
      break;
    default:
      QMMF_ERROR("%s: Stream %d is already closed!\n", __func__, id_);
      return -ENOTCONN;
  }

  if (pending_buffer_count_ > 0) {
    QMMF_ERROR("%s: Can't disconnect with %u buffers still dequeued!\n",
               __func__, pending_buffer_count_);
    auto it = mem_alloc_buffers_.begin();
    int32_t i = 0;
    while (it != mem_alloc_buffers_.end()) {
      QMMF_ERROR("%s: buffer[%d] = %p status: %d\n", __func__, i,
                 it->first, it->second);
      ++it;
      ++i;
    }
    PrintBuffersInfoLocked();
    return -ENOSYS;
  }

  assert(nullptr != mem_alloc_interface_);
  for (auto it = mem_alloc_buffers_.begin(); it != mem_alloc_buffers_.end(); ++it) {
    mem_alloc_interface_->FreeBuffer(it->first);
  }
  mem_alloc_buffers_.clear();

  status_ = (status_ == STATUS_RECONFIG_ACTIVE) ? STATUS_CONFIG_ACTIVE
                                                : STATUS_INTIALIZED;
  return 0;
}

void Camera3Stream::PrintBuffersInfo() {
  pthread_mutex_lock(&lock_);
  PrintBuffersInfoLocked();
  pthread_mutex_unlock(&lock_);
}

void Camera3Stream::PrintBuffersInfoLocked() {
  QMMF_ERROR("%s: Stream id: %d dim: %ux%u, fmt: %d "
      "Buffers: HAL(%u) Client(%u) Pending(%d) Total(%d)", __func__, id_,
      width, height, format, hal_buffer_cnt_, client_buffer_cnt_,
      pending_buffer_count_, total_buffer_count_);
}

}  // namespace cameraadaptor ends here

}  // namespace qmmf ends here
