/*
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
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
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
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
 */

#define LOG_TAG "CameraAdaptor"
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <dlfcn.h>
#include <string.h>
#include <algorithm>
#include <memory>
#include <string>


#include "recorder/src/service/qmmf_recorder_common.h"
#include "qmmf_camera3_utils.h"
#include "qmmf_camera3_device_client.h"
#ifdef QCAMERA3_TAG_LOCAL_COPY
#include "common/utils/qmmf_common_utils.h"
#else
#include <QCamera3VendorTags.h>
#endif
#ifdef TARGET_USES_GBM
#include "common/memory/qmmf_gbm_interface.h"
#endif

#ifdef DISABLE_OP_MODES
#define QCAMERA3_SENSORMODE_ZZHDR_OPMODE      (0xF002)
#define QCAMERA3_SENSORMODE_FPS_DEFAULT_INDEX (0x0)
#define FORCE_SENSORMODE_ENABLE               (1 << 24)
#define EIS_ENABLE                            (0xF200)
#define LDC_ENABLE                            (0xF800)
#define LCAC_ENABLE                           (0x100000)
#define IFE_DIRECT_STREAM                     (1 << 25)
#define CAM_OPMODE_FRAME_SELECTION            (0xF400)
#define CAM_OPMODE_FAST_SWITCH                (0xF900)
#endif

// Convenience macros for transitioning to the error state
#define SET_ERR(fmt, ...) \
  SetErrorState("%s: " fmt, __FUNCTION__, ##__VA_ARGS__)
#define SET_ERR_L(fmt, ...) \
  SetErrorStateLocked("%s: " fmt, __FUNCTION__, ##__VA_ARGS__)
using namespace qcamera;

uint32_t qmmf_log_level;

namespace qmmf {

namespace cameraadaptor {

std::mutex Camera3DeviceClient::vendor_tag_mutex_;
std::shared_ptr<VendorTagDescriptor> Camera3DeviceClient::vendor_tag_desc_;
uint32_t Camera3DeviceClient::client_count_ = 0;

Camera3DeviceClient::Camera3DeviceClient(CameraClientCallbacks clientCb)
    : client_cb_(clientCb),
      id_(0),
      state_(STATE_NOT_INITIALIZED),
      flush_on_going_(false),
      next_stream_id_(0),
      reconfig_(false),
      camera_module_(NULL),
      device_(NULL),
      number_of_cameras_(0),
      alloc_device_interface_(NULL),
      next_request_id_(0),
      frame_number_(0),
      next_shutter_frame_number_(0),
      next_shutter_input_frame_number_(0),
      partial_result_count_(0),
      is_partial_result_supported_(false),
      next_result_frame_number_(0),
      next_result_input_frame_number_(0),
      monitor_(),
      request_handler_(monitor_),
      pause_state_notify_(false),
      state_listeners_(0),
      is_hfr_supported_(false),
      is_raw_only_(false),
      hfr_mode_enabled_(false),
      cam_feature_flags_(static_cast<uint32_t>(CamFeatureFlag::kNone)),
      fps_sensormode_index_(0),
      prepare_handler_(),
      input_stream_{},
      is_camera_device_available_ (true),
      cam_opmode_ (CamOperationMode::kCamOperationModeNone),
      session_metadata_ (CameraMetadata(128, 128)) {
  QMMF_GET_LOG_LEVEL();
  camera3_callback_ops::notify = &notifyFromHal;
  camera3_callback_ops::process_capture_result = &processCaptureResult;
#if defined(CAMERA_HAL_API_VERSION) && (CAMERA_HAL_API_VERSION >= 0x0307)
  camera3_callback_ops_t::request_stream_buffers = &requestStreamBuffers;
  camera3_callback_ops_t::return_stream_buffers = &returnStreamBuffers;
#endif
  camera_module_callbacks_t::camera_device_status_change = &deviceStatusChange;
  camera_module_callbacks_t::torch_mode_status_change = &torchModeStatusChange;
  pthread_mutex_init(&lock_, NULL);
  pthread_mutex_init(&pending_requests_lock_, NULL);
  cond_init(&state_updated_);
  input_stream_.stream_id = -1;
  prepare_handler_.SetPrepareCb(clientCb.peparedCb);
}

Camera3DeviceClient::~Camera3DeviceClient() {
  request_handler_.RequestExit();

  if (NULL != device_) {
    device_->common.close(&device_->common);
  }

  prepare_handler_.Clear();
  prepare_handler_.RequestExit();

  for (uint32_t i = 0; i < streams_.size(); i++) {
    Camera3Stream *stream = streams_[i];
    delete stream;
  }
  streams_.clear();

  auto it = deleted_streams_.begin();
  while (it != deleted_streams_.end()) {
    Camera3Stream *stream = *it;
    it = deleted_streams_.erase(it);
    delete stream;

  }
  deleted_streams_.clear();

  monitor_.RequestExitAndWait();

  if (nullptr != alloc_device_interface_) {
    AllocDeviceFactory::DestroyAllocDevice(alloc_device_interface_);
    alloc_device_interface_ = nullptr;
  }

  {
    std::lock_guard<std::mutex> lk(vendor_tag_mutex_);
    if (--client_count_ == 0) {
      VendorTagDescriptor::clearGlobalVendorTagDescriptor();
      if (vendor_tag_desc_.get() != nullptr)
        vendor_tag_desc_.reset();
    }
  }

  pending_error_requests_vector_.clear();

  pthread_cond_destroy(&state_updated_);
  pthread_mutex_destroy(&pending_requests_lock_);
  pthread_mutex_destroy(&lock_);
}

int32_t Camera3DeviceClient::Initialize() {
  int32_t res = 0;

  pthread_mutex_lock(&lock_);

  if (state_ != STATE_NOT_INITIALIZED) {
    QMMF_ERROR("%s: Already initialized! \n", __func__);
    res = -ENOSYS;
    goto exit;
  }

  res = LoadHWModule(CAMERA_HARDWARE_MODULE_ID,
                     (const hw_module_t **)&camera_module_);

  if ((0 != res) || (NULL == camera_module_)) {
    QMMF_ERROR("%s: Unable to load Hal module: %d\n", __func__, res);
    goto exit;
  }

  QMMF_INFO("%s: Camera Module author: %s, version: %d name: %s\n", __func__,
            camera_module_->common.author, camera_module_->common.hal_api_version,
            camera_module_->common.name);

  number_of_cameras_ = camera_module_->get_number_of_cameras();
  QMMF_INFO("%s: Number of cameras: %d\n", __func__, number_of_cameras_);

  if (NULL != camera_module_->init) {
    res = camera_module_->init();
    if (0 != res) {
      QMMF_ERROR("%s: Failed to initialize Camera Hal module!", __func__);
      goto exit;
    }
  }

  if (camera_module_->get_vendor_tag_ops) {
    std::lock_guard<std::mutex> lk(vendor_tag_mutex_);
    if (client_count_ == 0) {
      vendor_tag_ops_ = vendor_tag_ops_t();
      camera_module_->get_vendor_tag_ops(&vendor_tag_ops_);

      res = VendorTagDescriptor::createDescriptorFromOps(&vendor_tag_ops_,
                                                         vendor_tag_desc_);

      if (0 != res) {
        QMMF_ERROR("%s: Could not generate descriptor from vendor tag operations,"
            "received error %s (%d). Camera clients will not be able to use"
            "vendor tags", __FUNCTION__, strerror(res), res);
        goto exit;
      }

      // Set the global descriptor to use with camera metadata
      res = VendorTagDescriptor::setAsGlobalVendorTagDescriptor(vendor_tag_desc_);

      if (0 != res) {
        QMMF_ERROR(
            "%s: Could not set vendor tag descriptor, "
            "received error %s (%d). \n",
            __func__, strerror(-res), res);
        goto exit;
      }
    }
    ++client_count_;
  }

  camera_module_->set_callbacks(this);

  alloc_device_interface_ = AllocDeviceFactory::CreateAllocDevice();
  state_ = STATE_CLOSED;
  next_stream_id_ = 0;
  reconfig_ = true;

  pthread_mutex_unlock(&lock_);

  return res;

exit:

  if (nullptr != alloc_device_interface_) {
    AllocDeviceFactory::DestroyAllocDevice(alloc_device_interface_);
    alloc_device_interface_ = nullptr;
  }

  {
    std::lock_guard<std::mutex> lk(vendor_tag_mutex_);
    if (client_count_ == 0) {
      VendorTagDescriptor::clearGlobalVendorTagDescriptor();
      if (vendor_tag_desc_.get() != nullptr)
        vendor_tag_desc_.reset();
    }
  }

  if (NULL != camera_module_) {
    dlclose(camera_module_->common.dso);
  }
  device_ = NULL;
  camera_module_ = NULL;

  pthread_mutex_unlock(&lock_);

  return res;
}

int32_t Camera3DeviceClient::OpenCamera(uint32_t idx) {
  int32_t res = 0;
  std::string name;
  std::string id;
  camera_metadata_entry_t capsEntry;
  MarkRequest mark_cb = [&] (uint32_t frameNumber, int32_t numBuffers,
                                 CaptureResultExtras resultExtras) {
    return MarkPendingRequest(frameNumber,numBuffers, resultExtras); };
  SetError set_error = [&] (const char *fmt, va_list args) {
    SetErrorStateV(fmt, args);
  };
  if (idx >= number_of_cameras_) {
    QMMF_ERROR("%s: Invalid camera idx: %d\n", __func__, idx);
    return -EINVAL;
  }

  if (NULL == camera_module_) {
    QMMF_ERROR("%s: Hal module not initialized yet!\n", __func__);
    return -ENODEV;
  }

  if (NULL != device_) {
    QMMF_ERROR("%s: Camera device is already open!\n", __func__);
    return -EINVAL;
  }

  pthread_mutex_lock(&lock_);

  if (state_ != STATE_CLOSED) {
    QMMF_ERROR("%s: Invalid state: %d! \n", __func__, state_);
    res = -ENOSYS;
    goto exit;
  }

  res = GetCameraInfo(idx, &device_info_);
  if (0 != res) {
    QMMF_ERROR("%s: Error during camera static info query: %s!\n", __func__,
               strerror(res));
    goto exit;
  }

  id = std::to_string(idx);
  res = camera_module_->common.methods->open(&camera_module_->common, id.c_str(),
                                            (hw_device_t **)(&device_));
  if (0 != res) {
    QMMF_ERROR("Could not open camera: %s (%d) \n", strerror(-res), res);
    goto exit;
  }

  if (device_->common.version < CAMERA_DEVICE_API_VERSION_3_2) {
    QMMF_ERROR(
        "Could not open camera: "
        "Camera device should be at least %x, reports %x instead",
        CAMERA_DEVICE_API_VERSION_3_2, device_->common.version);
    res = -EINVAL;
    goto exit;
  }

  res = device_->ops->initialize(device_, this);
  if (0 != res) {
    QMMF_ERROR("Could not initialize camera: %s (%d) \n", strerror(-res), res);
    goto exit;
  }

  {
    camera_metadata_entry partialResultsCount =
        device_info_.find(ANDROID_REQUEST_PARTIAL_RESULT_COUNT);
    if (partialResultsCount.count > 0) {
      partial_result_count_ = partialResultsCount.data.i32[0];
      is_partial_result_supported_ = (partial_result_count_ > 1);
    }
  }

  capsEntry = device_info_.find(ANDROID_REQUEST_AVAILABLE_CAPABILITIES);
  for (uint32_t i = 0; i < capsEntry.count; ++i) {
    uint8_t caps = capsEntry.data.u8[i];
    if (ANDROID_REQUEST_AVAILABLE_CAPABILITIES_CONSTRAINED_HIGH_SPEED_VIDEO ==
        caps) {
      is_hfr_supported_ = true;
      break;
    }
  }

  id_ = idx;
  state_ = STATE_NOT_CONFIGURED;

  name = "C3-" + id + "-Monitor";

  monitor_.SetIdleNotifyCb([&] (bool idle) {NotifyStatus(idle);});
  monitor_.Run(name);
  if (0 != res) {
    SET_ERR_L("Unable to start monitor: %s (%d)", strerror(-res), res);
    goto exit;
  }

  name = "C3-" + id + "-Handler";

  request_handler_.Initialize(device_, client_cb_.errorCb, mark_cb, set_error);
  res = request_handler_.Run(name);
  if (0 > res) {
    SET_ERR_L("Unable to start request handler: %s (%d)", strerror(-res), res);
    goto exit;
  }

  pthread_mutex_unlock(&lock_);

  return res;

exit:

  if (device_) {
    device_->common.close(&device_->common);
  }
  device_ = NULL;
  pthread_mutex_unlock(&lock_);

  return res;
}

int32_t Camera3DeviceClient::EndConfigure(const CameraParameters& stream_config) {

  if (NULL == camera_module_) {
    return -ENODEV;
  }

  if (stream_config.is_constrained_high_speed && !is_hfr_supported_) {
    QMMF_ERROR("%s: HFR mode is not supported by this camera!\n", __func__);
    return -EINVAL;
  }

  return ConfigureStreams(stream_config);
}

int32_t Camera3DeviceClient::UpdateCameraParams(
    const CameraParameters& stream_config) {

  if (NULL == camera_module_) {
    return -ENODEV;
  }

  if (stream_config.is_constrained_high_speed && !is_hfr_supported_) {
    QMMF_ERROR("%s: HFR mode is not supported by this camera!\n", __func__);
    return -EINVAL;
  }

  return ConfigureStreams(stream_config, true);
}

int32_t Camera3DeviceClient::ConfigureStreams(
    const CameraParameters& stream_config, bool force_reconfiguration) {

  pthread_mutex_lock(&lock_);

  hfr_mode_enabled_ = stream_config.is_constrained_high_speed;
  is_raw_only_ = stream_config.is_raw_only;
  batch_size_ = stream_config.batch_size;
  frame_rate_range_[0] = stream_config.frame_rate_range[0];
  frame_rate_range_[1] = stream_config.frame_rate_range[1];
  cam_opmode_ = stream_config.cam_opmode;
  request_handler_.SetRequestMode(cam_opmode_);

  if (force_reconfiguration) {
    cam_feature_flags_ = stream_config.cam_feature_flags;
  } else {
    cam_feature_flags_ |= stream_config.cam_feature_flags;
  }

#ifdef USE_FPS_IDX
  fps_sensormode_index_ = stream_config.fps_sensormode_index;
#endif

  bool res = ConfigureStreamsLocked(force_reconfiguration);

  pthread_mutex_unlock(&lock_);

  return res;
}

int32_t Camera3DeviceClient::ConfigureStreamsLocked(
    bool force_reconfiguration) {

  status_t res;

  if (state_ != STATE_NOT_CONFIGURED && state_ != STATE_CONFIGURED &&
      !force_reconfiguration) {
    QMMF_ERROR("%s: Not idle\n", __func__);
    return -ENOSYS;
  }

  if (!reconfig_ && !force_reconfiguration) {
    QMMF_ERROR("%s: Skipping config, no stream changes\n", __func__);
    return 0;
  }

  camera3_stream_configuration config;
  memset(&config, 0, sizeof(config));

  config.operation_mode = GetOpMode();

  QMMF_INFO("%s: operation_mode: 0x%x \n", __func__, config.operation_mode);

#if defined(CAMERA_HAL_API_VERSION) && (CAMERA_HAL_API_VERSION >= 0x0305)
  session_metadata_.update(ANDROID_CONTROL_AE_TARGET_FPS_RANGE,
                            frame_rate_range_, 2);

  if (IsInputROIMode()) {
    uint32_t tag_id = 0;
    uint8_t roienable = true;
    const std::shared_ptr<VendorTagDescriptor> vtags =
        VendorTagDescriptor::getGlobalVendorTagDescriptor();
    if (vtags.get() == NULL) {
      QMMF_ERROR ("Failed to retrieve Global Vendor Tag Descriptor!");
      return -1;
    }

    session_metadata_.getTagFromName(
        "org.codeaurora.qcamera3.sessionParameters.MultiRoIEnable",
        vtags.get(), &tag_id);

    session_metadata_.update(tag_id, &roienable, 1);
  }

  config.session_parameters = session_metadata_.getAndLock();
#endif

  std::vector<camera3_stream_t *> streams;

  if (0 <= input_stream_.stream_id) {
    input_stream_.usage = 0; // Reset any previously set usage flags from Hal
    streams.push_back(&input_stream_);
  }

  for (size_t i = 0; i < streams_.size(); i++) {
    camera3_stream_t *outputStream;
    outputStream = streams_[i]->BeginConfigure();
    if (outputStream == NULL) {
      QMMF_ERROR("%s: Can't start stream configuration\n", __func__);
      return -ENOSYS;
    }
    streams.push_back(outputStream);
  }

  config.streams = streams.data();
  config.num_streams = streams.size();

  res = device_->ops->configure_streams(device_, &config);
#if defined(CAMERA_HAL_API_VERSION) && (CAMERA_HAL_API_VERSION >= 0x0305)
  if (config.session_parameters != NULL) {
    session_metadata_.unlock(config.session_parameters);
  }
#endif
  if (res == -EINVAL) {
    for (uint32_t i = 0; i < streams_.size(); i++) {
      Camera3Stream *stream = streams_[i];
      if (stream->IsConfigureActive()) {
        res = stream->AbortConfigure();
        if (0 != res) {
          QMMF_ERROR("Can't abort stream %d configure: %s (%d)\n",
                     stream->GetId(), strerror(-res), res);
          return res;
        }
      }
    }

    InternalUpdateStatusLocked(STATE_NOT_CONFIGURED);
    reconfig_ = true;

    return -EINVAL;
  } else if (0 != res) {
    QMMF_ERROR("%s: Unable to configure streams with HAL: %s (%d)\n", __func__,
               strerror(-res), res);
    return res;
  }

  for (uint32_t i = 0; i < streams_.size(); i++) {
    Camera3Stream *outputStream = streams_[i];
    if (outputStream->IsConfigureActive()) {
      res = outputStream->EndConfigure();
      if (0 != res) {
        QMMF_ERROR(
            "%s: Unable to complete stream configuration"
            "%d: %s (%d)\n",
            __func__, outputStream->GetId(), strerror(-res), res);
        return res;
      }
    }
  }

  request_handler_.FinishConfiguration(batch_size_);
  reconfig_ = false;
  frame_number_ = 0;
  InternalUpdateStatusLocked(STATE_CONFIGURED);

  auto it = deleted_streams_.begin();
  while (it != deleted_streams_.end()) {
    Camera3Stream *stream = *it;
    it = deleted_streams_.erase(it);
    delete stream;

  }
  deleted_streams_.clear();

  return 0;
}

int32_t Camera3DeviceClient::DeleteStream(int streamId, bool cache) {
  int32_t res = 0;
  Camera3Stream *stream;
  int32_t outputStreamIdx;
  pthread_mutex_lock(&lock_);

  switch (state_) {
    case STATE_ERROR:
      QMMF_ERROR("%s: Device has encountered a serious error\n", __func__);
      res = -ENOSYS;
      goto exit;
    case STATE_NOT_INITIALIZED:
    case STATE_CLOSED:
      QMMF_ERROR("%s: Device not initialized\n", __func__);
      res = -ENOSYS;
      goto exit;
    case STATE_NOT_CONFIGURED:
    case STATE_CONFIGURED:
    case STATE_RUNNING:
      if (!cache) {
        QMMF_INFO("%s: Stream is not cached, Issue internal reconfig!",
            __func__);
        res = InternalPauseAndWaitLocked();
        if (0 != res) {
          SET_ERR_L("Can't pause captures to reconfigure streams!");
          goto exit;
        }
      }
      break;
    default:
      QMMF_ERROR("%s: Unknown state: %d\n", __func__, state_);
      res = -ENOSYS;
      goto exit;
  }

  if (streamId == input_stream_.stream_id) {
    input_stream_.stream_id = -1;
    // todo: wait for stream idle
  } else {
    if (streams_.count(streamId) == 0) {
      QMMF_ERROR("%s: Stream %d does not exist\n", __func__, streamId);
      res = -EINVAL;
      goto exit;
    }

    stream = streams_[streamId];
    if (request_handler_.IsStreamActive(*stream)) {
      QMMF_ERROR("%s: Stream %d still has pending requests\n", __func__,
                 streamId);
      res = -ENOSYS;
      goto exit;
    }

    // If this point is reached then state is Idle. Idle means that HAL idle i.e.
    // HAL is returned all requests/buffers. But we still must wait client
    // to return buffer before we can delete stream.
    pthread_mutex_unlock(&lock_);
    stream->WaitForIdle();
    pthread_mutex_lock(&lock_);

    // Remove stream after buffer returned from client
    streams_.erase(streamId);

    if (streams_.empty()) {
      cam_feature_flags_ = static_cast<uint32_t>(CamFeatureFlag::kNone);
    }

    res = stream->Close();
    if (0 != res) {
      QMMF_ERROR("%s: Can't close deleted stream %d\n", __func__, streamId);
    }
    if (!cache && !streams_.empty()) {
      reconfig_ = true;
      res = ConfigureStreamsLocked();
      if (0 != res) {
        QMMF_ERROR("%s: Can't reconfigure device for new stream %d: %s (%d)",
                 __func__, next_stream_id_, strerror(-res), res);
        goto exit;
      }
      InternalResumeLocked();
    } else {
      // In this scenario stream will be cached and will not trigger stream
      // reconfiguration. reconfiguration will be triggered in next round of
      // updating streaming capture request - creating a brand new stream or
      // deleting an existing stream without caching.
      if (state_ != STATE_RUNNING) {
        // Avoid reconfiguration if any existing stream is running, otherwise
        // updating capture request for setting parameters will try to
        // reconfigure it.
        reconfig_ = true;
      }
      deleted_streams_.push_back(stream);
    }
  }

exit:

  pthread_mutex_unlock(&lock_);

  return res;
}

int32_t Camera3DeviceClient::CreateInputStream(
    const CameraInputStreamParameters &inputConfiguration) {
  int32_t res = 0;
  bool wasActive = false;
  pthread_mutex_lock(&lock_);

  if ((nullptr == inputConfiguration.get_input_buffer) ||
      (nullptr == inputConfiguration.return_input_buffer)) {
    QMMF_ERROR("%s: Input stream callbacks are invalid!\n", __func__);
    res = -EINVAL;
    goto exit;
  }

  if (0 <= input_stream_.stream_id) {
    QMMF_ERROR("%s: Only one input stream can be created at any time!\n",
               __func__);
    res = -ENOSYS;
    goto exit;
  }

  switch (state_) {
    case STATE_ERROR:
      QMMF_ERROR("%s: Device has encountered a serious error\n", __func__);
      res = -ENOSYS;
      goto exit;
    case STATE_NOT_INITIALIZED:
    case STATE_CLOSED:
      QMMF_ERROR("%s: Device not initialized\n", __func__);
      res = -ENOSYS;
      goto exit;
    case STATE_NOT_CONFIGURED:
    case STATE_CONFIGURED:
      break;
    case STATE_RUNNING:
      res = InternalPauseAndWaitLocked();
      if (0 != res) {
        SET_ERR_L("Can't pause captures to reconfigure streams!");
        goto exit;
      }
      wasActive = true;
      break;
    default:
      QMMF_ERROR("%s: Unknown state: %d\n", __func__, state_);
      res = -ENOSYS;
      goto exit;
  }
  assert(state_ != STATE_RUNNING);

  reconfig_ = true;

  input_stream_ = {};
  input_stream_.width = inputConfiguration.width;
  input_stream_.height = inputConfiguration.height;
  input_stream_.format = inputConfiguration.format;
  input_stream_.get_input_buffer = inputConfiguration.get_input_buffer;
  input_stream_.return_input_buffer = inputConfiguration.return_input_buffer;
  input_stream_.stream_id = next_stream_id_++;
  input_stream_.stream_type = CAMERA3_STREAM_INPUT;

  // Continue captures if active at start
  if (wasActive) {
    res = ConfigureStreamsLocked();
    if (0 != res) {
      QMMF_ERROR("%s: Can't reconfigure device for new stream %d: %s (%d)",
                 __func__, next_stream_id_, strerror(-res), res);
      goto exit;
    }
    InternalResumeLocked();
  }

  res = input_stream_.stream_id;

exit:

    pthread_mutex_unlock(&lock_);

    return res;
}

int32_t Camera3DeviceClient::CreateStream(
    const CameraStreamParameters &outputConfiguration) {

  int32_t res = 0;
  Camera3Stream *newStream = NULL;
  int32_t blobBufferSize = 0;
  bool wasActive = false;
  pthread_mutex_lock(&lock_);

  if (nullptr == outputConfiguration.cb) {
    QMMF_ERROR("%s: Stream callback invalid!\n", __func__);
    res = -EINVAL;
    goto exit;
  }

  switch (state_) {
    case STATE_ERROR:
      QMMF_ERROR("%s: Device has encountered a serious error\n", __func__);
      res = -ENOSYS;
      goto exit;
    case STATE_NOT_INITIALIZED:
    case STATE_CLOSED:
      QMMF_ERROR("%s: Device not initialized\n", __func__);
      res = -ENOSYS;
      goto exit;
    case STATE_NOT_CONFIGURED:
    case STATE_CONFIGURED:
      break;
    case STATE_RUNNING:
      res = InternalPauseAndWaitLocked();
      if (0 != res) {
        SET_ERR_L("Can't pause captures to reconfigure streams!");
        goto exit;
      }
      wasActive = true;
      break;
    default:
      QMMF_ERROR("%s: Unknown state: %d\n", __func__, state_);
      res = -ENOSYS;
      goto exit;
  }
  assert(state_ != STATE_RUNNING);

  if (outputConfiguration.format == HAL_PIXEL_FORMAT_BLOB) {
    blobBufferSize = CalculateBlobSize(outputConfiguration.width,
                                       outputConfiguration.height);
    if (blobBufferSize <= 0) {
      QMMF_ERROR("%s: Invalid jpeg buffer size %zd\n", __func__,
                 blobBufferSize);
      res = -EINVAL;
      goto exit;
    }
  }

  newStream = new Camera3Stream(next_stream_id_, blobBufferSize,
                                outputConfiguration, alloc_device_interface_, monitor_);
  if (NULL == newStream) {
    res = -ENOMEM;
    goto exit;
  }

  streams_.emplace(next_stream_id_, newStream);
  reconfig_ = true;

  // Continue captures if active at start
  if (wasActive) {
    res = ConfigureStreamsLocked();
    if (0 != res) {
      QMMF_ERROR("%s: Can't reconfigure device for new stream %d: %s (%d)",
                 __func__, next_stream_id_, strerror(-res), res);
      goto exit;
    }
    InternalResumeLocked();
  }

  res = next_stream_id_++;

exit:

  pthread_mutex_unlock(&lock_);
  return res;
}

int32_t Camera3DeviceClient::CalculateBlobSize(int32_t width, int32_t height) {
  int32_t maxJpegBufferSize, maxJpegSizeWidth, maxJpegSizeHeight, res, jpegDebugDataSize;
  int32_t maxWidth, maxHeight;
  int32_t maxUHRWidth, maxUHRHeight;
  int32_t ret;
  camera_metadata_entry entry;

  maxWidth = maxHeight = maxUHRWidth = maxUHRHeight = res = jpegDebugDataSize = 0;

  entry = device_info_.find(ANDROID_JPEG_MAX_SIZE);
  if (entry.count == 0) {
    QMMF_ERROR(
        "%s: Camera %d: Can't find maximum JPEG size in static"
        " metadata!\n",
        __func__, id_);
    return -EINVAL;
  }
  maxJpegBufferSize = entry.data.i32[0];
  assert(JPEG_BUFFER_SIZE_MIN < maxJpegBufferSize);

  QMMF_INFO("%s: default maxJpegBufferSize=%d", __func__, maxJpegBufferSize);


#if defined(CAMERA_HAL_API_VERSION) && (CAMERA_HAL_API_VERSION >= 0x0307)
  if (device_info_.exists(
      ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_MAXIMUM_RESOLUTION)) {
    auto entry = device_info_.find(
      ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_MAXIMUM_RESOLUTION);
    for (uint32_t i = 0; i < entry.count; i += 4) {
      if (HAL_PIXEL_FORMAT_BLOB == entry.data.i32[i] &&
          ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT ==
            entry.data.i32[i+3]) {
        int32_t w = entry.data.i32[i + 1];
        int32_t h = entry.data.i32[i + 2];

        if (w * h > maxUHRWidth * maxUHRHeight) {
          maxUHRWidth = w;
          maxUHRHeight = h;
        }
      }
    }
  }

  //Calculate debuging buffer size of jpeg.
  uint32_t tag = 0;

  std::shared_ptr<VendorTagDescriptor> vTags =
      VendorTagDescriptor::getGlobalVendorTagDescriptor();

  CameraMetadata::getTagFromName(
      "org.quic.camera.jpegdebugdata.size",vTags.get(), &tag);

  if (device_info_.exists(tag)) {
    auto entry  = device_info_.find(tag);
    jpegDebugDataSize = entry.data.i32[0];
  }

  QMMF_INFO("%s: jpegDebugDataSize=%d",
      __func__, jpegDebugDataSize);
#endif

  if (device_info_.exists(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS)) {
    auto entry = device_info_.find(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS);
    for (uint32_t i = 0; i < entry.count; i += 4) {
      if (HAL_PIXEL_FORMAT_BLOB == entry.data.i32[i] &&
          ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT ==
            entry.data.i32[i+3]) {
        int32_t w = entry.data.i32[i + 1];
        int32_t h = entry.data.i32[i + 2];

        if (w * h > maxWidth * maxHeight) {
          maxWidth = w;
          maxHeight = h;
        }
      }
    }
  } else {
    QMMF_ERROR(
        "%s: Camera %d: Can't find available stream configs", __func__, id_);
    return -EINVAL;
  }

  QMMF_INFO("%s: maxUHRWidth=%d maxUHRHeight=%d maxWidth=%d maxHeight=%d",
      __func__, maxUHRWidth, maxUHRHeight, maxWidth, maxHeight);

  // if input width * height is larger than default max width * height,
  // it means ultra hight resolution has been selected, to make scaleFactor
  // calculation work correctly, we need to update buffersize, max width and
  // max height accordingly
  if ((maxUHRWidth != 0) && ((width * height) > (maxWidth * maxHeight))) {
    maxJpegSizeWidth = maxUHRWidth;
    maxJpegSizeHeight = maxUHRHeight;
    maxJpegBufferSize =
      ((maxUHRWidth * 1.0f * maxUHRHeight) / (maxWidth * maxHeight)) *
        maxJpegBufferSize;
  } else {
    maxJpegSizeWidth = maxWidth;
    maxJpegSizeHeight = maxHeight;
  }

  QMMF_INFO("%s: input width=%d height=%d"
      " maxJpegBufferSize=%d maxJpegSizeWidth=%d maxJpegSizeHeight=%d",
      __func__, width, height,
      maxJpegBufferSize, maxJpegSizeWidth, maxJpegSizeHeight);

  assert(JPEG_BUFFER_SIZE_MIN < maxJpegBufferSize);

  // Calculate final jpeg buffer size for the given resolution.
  float scaleFactor =
      ((float)(width * height)) / (maxJpegSizeWidth * maxJpegSizeHeight);
  ssize_t jpegBufferSize =
      scaleFactor * (maxJpegBufferSize - JPEG_BUFFER_SIZE_MIN
      - jpegDebugDataSize) + JPEG_BUFFER_SIZE_MIN + jpegDebugDataSize;

  if (jpegBufferSize > maxJpegBufferSize) {
    jpegBufferSize = maxJpegBufferSize;
  }

  QMMF_INFO("%s: scaleFactor=%f jpegBufferSize=%d",
      __func__, scaleFactor, jpegBufferSize);

  return jpegBufferSize;
}

int32_t Camera3DeviceClient::CreateDefaultRequest(int templateId,
                                                  CameraMetadata *request) {
  int32_t res = 0;
  pthread_mutex_lock(&lock_);

  switch (state_) {
    case STATE_ERROR:
      QMMF_ERROR("%s: Device has encountered a serious error\n", __func__);
      res = -ENOSYS;
      goto exit;
    case STATE_NOT_INITIALIZED:
    case STATE_CLOSED:
      QMMF_ERROR("%s: Device is not initialized!\n", __func__);
      res = -ENOSYS;
      goto exit;
    case STATE_NOT_CONFIGURED:
    case STATE_CONFIGURED:
    case STATE_RUNNING:
      break;
    default:
      QMMF_ERROR("%s: Unexpected status: %d", __func__, state_);
      res = -ENOSYS;
      goto exit;
  }

  if (!request_templates_[templateId].isEmpty()) {
    *request = request_templates_[templateId];
    goto exit;
  }

  const camera_metadata_t *rawRequest;
  rawRequest =
      device_->ops->construct_default_request_settings(device_, templateId);
  if (rawRequest == NULL) {
    QMMF_ERROR("%s: template %d is not supported on this camera device\n",
               __func__, templateId);
    res = -EINVAL;
    goto exit;
  }
  *request = rawRequest;
  request_templates_[templateId] = rawRequest;

exit:

  pthread_mutex_unlock(&lock_);
  return res;
}

int32_t Camera3DeviceClient::MarkPendingRequest(
    uint32_t frameNumber, int32_t numBuffers,
    CaptureResultExtras resultExtras) {
  pthread_mutex_lock(&pending_requests_lock_);

  pending_requests_vector_.emplace(frameNumber,
                                   PendingRequest(numBuffers, resultExtras));

  pthread_mutex_unlock(&pending_requests_lock_);

  return 0;
}

bool Camera3DeviceClient::HandlePartialResult(
    uint32_t frameNumber, const CameraMetadata &partial,
    const CaptureResultExtras &resultExtras) {

  if (nullptr != client_cb_.resultCb) {
    CaptureResult captureResult;
    captureResult.resultExtras = resultExtras;
    captureResult.metadata = partial;

    if (!UpdatePartialTag(captureResult.metadata, ANDROID_REQUEST_FRAME_COUNT,
                          reinterpret_cast<int32_t *>(&frameNumber),
                          frameNumber)) {
      return false;
    }

    int32_t requestId = resultExtras.requestId;
    if (!UpdatePartialTag(captureResult.metadata, ANDROID_REQUEST_ID,
                          &requestId, frameNumber)) {
      return false;
    }

    if (device_->common.version < CAMERA_DEVICE_API_VERSION_3_2) {
      static const uint8_t partialResult =
          ANDROID_QUIRKS_PARTIAL_RESULT_PARTIAL;
      if (!UpdatePartialTag(captureResult.metadata,
                            ANDROID_QUIRKS_PARTIAL_RESULT, &partialResult,
                            frameNumber)) {
        return false;
      }
    }

    client_cb_.resultCb(captureResult);
  }

  return true;
}

template <typename T>
bool Camera3DeviceClient::QueryPartialTag(const CameraMetadata &result,
                                          int32_t tag, T *value,
                                          uint32_t frameNumber) {
  (void)frameNumber;

  camera_metadata_ro_entry_t entry;

  entry = result.find(tag);
  if (entry.count == 0) {
    return false;
  }

  if (sizeof(T) == sizeof(uint8_t)) {
    *value = entry.data.u8[0];
  } else if (sizeof(T) == sizeof(int32_t)) {
    *value = entry.data.i32[0];
  } else {
    return false;
  }
  return true;
}

template <typename T>
bool Camera3DeviceClient::UpdatePartialTag(CameraMetadata &result, int32_t tag,
                                           const T *value,
                                           uint32_t frameNumber) {
  if (0 != result.update(tag, value, 1)) {
    return false;
  }
  return true;
}

int32_t Camera3DeviceClient::CancelRequest(int requestId,
                                           int64_t *lastFrameNumber) {
  std::vector<int32_t>::iterator it, end;
  int32_t res = 0;

  pthread_mutex_lock(&lock_);

  switch (state_) {
    case STATE_ERROR:
      QMMF_ERROR("%s: Device has encountered a serious error\n", __func__);
      res = -ENOSYS;
      goto exit;
    case STATE_NOT_INITIALIZED:
    case STATE_CLOSED:
      QMMF_ERROR("%s: Device not initialized\n", __func__);
      res = -ENOSYS;
      goto exit;
    case STATE_NOT_CONFIGURED:
    case STATE_CONFIGURED:
    case STATE_RUNNING:
      break;
    default:
      SET_ERR_L("Unknown state: %d", state_);
      res = -ENOSYS;
      goto exit;
  }

  for (it = repeating_requests_.begin(), end = repeating_requests_.end();
       it != end; ++it) {
    if (*it == requestId) {
      break;
    }
  }

  if (it == end) {
    QMMF_ERROR(
        "%s: Camera%d: Did not find request id %d in list of"
        " streaming requests",
        __FUNCTION__, id_, requestId);
    res = -EINVAL;
    goto exit;
  }

  res = request_handler_.ClearRepeatingRequests(lastFrameNumber);
  if (0 == res) {
    repeating_requests_.clear();
  }

exit:

  pthread_mutex_unlock(&lock_);
  return res;
}

void Camera3DeviceClient::HandleCaptureResult(
    const camera3_capture_result *result) {
  int32_t res;

  uint32_t frameNumber = result->frame_number;
  if (result->result == NULL && result->num_output_buffers == 0 &&
      result->input_buffer == NULL) {
    SET_ERR("No result data provided by HAL for frame %d", frameNumber);
    return;
  }

  if (!is_partial_result_supported_ && result->result != NULL &&
      result->partial_result != 1) {
    SET_ERR(
        "Result is invalid for frame %d: partial_result %u should be 1"
        " when partial result are not supported\n",
        frameNumber, result->partial_result);
    return;
  }

  bool isPartialResult = false;
  CameraMetadata collectedPartialResult;
  camera_metadata_ro_entry_t entry;
  uint32_t numBuffersReturned;

  int64_t shutterTimestamp = 0;

  pthread_mutex_lock(&pending_requests_lock_);
  if (!pending_requests_vector_.count(frameNumber)) {
    if (pending_error_requests_vector_.count(frameNumber)) {
      QMMF_INFO("%s: Found a request with error status. "
          "Ignore the capture result.\n", __func__);
      pending_error_requests_vector_.erase(frameNumber);
      pthread_mutex_unlock(&pending_requests_lock_);
      return;
    }
    SET_ERR("Invalid frame number in capture result: %d", frameNumber);
    pthread_mutex_unlock(&pending_requests_lock_);
    return;
  }
  PendingRequest &request = pending_requests_vector_.at(frameNumber);
  QMMF_DEBUG(
      "%s: Received PendingRequest requestId = %d, frameNumber = %d,"
      "burstId = %d, partialResultCount = %d\n",
      __func__, request.resultExtras.requestId,
      request.resultExtras.frameNumber, request.resultExtras.burstId,
      result->partial_result);
  if (result->partial_result != 0)
    request.resultExtras.partialResultCount = result->partial_result;

  if (is_partial_result_supported_ && result->result != NULL) {
    if (result->partial_result > partial_result_count_ ||
        result->partial_result < 1) {
      SET_ERR(
          "Result is invalid for frame %d: partial_result %u"
          "should be in the range of [1, %d] when meta gets included"
          "in the result",
          frameNumber, result->partial_result, partial_result_count_);
      goto exit;
    }
    isPartialResult = (result->partial_result < partial_result_count_);
    if (isPartialResult) {
      request.partialResult.composedResult.append(result->result);
    }

    if (isPartialResult) {
      request.partialResult.partial3AReceived = HandlePartialResult(
          frameNumber, request.partialResult.composedResult,
          request.resultExtras);

    }

    if (result->partial_result == 1 &&
        CAM_OPMODE_IS_FRAMESELECTION(cam_opmode_)) {
      uint32_t tag = 0;
      camera_metadata_ro_entry entry;
      int32_t res;

      std::shared_ptr<VendorTagDescriptor> vTags =
          VendorTagDescriptor::getGlobalVendorTagDescriptor();

      CameraMetadata::getTagFromName(
          "org.quic.camera.frameselection.updatedPickedFrames",
          vTags.get(), &tag);

      if (tag > 0) {
        res = find_camera_metadata_ro_entry(result->result, tag, &entry);
        if (res == 0 && entry.count > 0) {
          CamReqModeInputParams params;

          params.frame_selection.total_selected_frames = entry.data.i32[0];
          request_handler_.UpdateRequestedStreams(params);
        }
      }
    }
  }

  shutterTimestamp = request.shutterTS;

  if (result->result != NULL && !isPartialResult) {
    if (request.isMetaPresent) {
      SET_ERR("Called several times with meta for frame %d", frameNumber);
      goto exit;
    }
    if (is_partial_result_supported_ &&
        !request.partialResult.composedResult.isEmpty()) {
      collectedPartialResult.acquire(request.partialResult.composedResult);
    }
    request.isMetaPresent = true;
  }

  numBuffersReturned = result->num_output_buffers;
  request.buffersRemaining -= numBuffersReturned;
  if (NULL != result->input_buffer) {
    request.buffersRemaining--;
  }
  if (request.buffersRemaining < 0) {
    SET_ERR("Too many buffers returned for frame %d", frameNumber);
    goto exit;
  }

  res = find_camera_metadata_ro_entry(result->result, ANDROID_SENSOR_TIMESTAMP,
                                      &entry);
  if ((0 == res) && (entry.count == 1)) {
    request.sensorTS = entry.data.i64[0];
  }

  if ((shutterTimestamp == 0) && (NULL != result->output_buffers) &&
      (0 < result->num_output_buffers)) {
    for (uint32_t i = 0; i < result->num_output_buffers; ++i) {
      request.pendingBuffers.push_back(result->output_buffers[i]);
    }
  }

  if (result->result != NULL && !isPartialResult) {
    if (shutterTimestamp == 0) {
      request.pendingMetadata = result->result;
      request.partialResult.composedResult = collectedPartialResult;
    } else {
      CameraMetadata metadata;
      metadata = result->result;
      SendCaptureResult(metadata, request.resultExtras, collectedPartialResult,
                        frameNumber);
    }
  }

  if (0 < shutterTimestamp) {
    ReturnOutputBuffers(result->output_buffers, result->num_output_buffers,
                        shutterTimestamp, result->frame_number);
  }

  RemovePendingRequestLocked(frameNumber);
  pthread_mutex_unlock(&pending_requests_lock_);

  if (NULL != result->input_buffer) {
    StreamBuffer input_buffer;
    memset(&input_buffer, 0, sizeof(input_buffer));
    Camera3InputStream *input_stream =
        static_cast<Camera3InputStream *>(result->input_buffer->stream);
    input_buffer.stream_id = input_stream->stream_id;
    input_buffer.data_space = input_stream->data_space;
    input_buffer.handle =
      input_stream->buffers_map[*result->input_buffer->buffer];
    input_stream->buffers_map.erase(*result->input_buffer->buffer);
    input_stream->return_input_buffer(input_buffer);
    input_stream->input_buffer_cnt--;
  }

  return;

exit:
  pthread_mutex_unlock(&pending_requests_lock_);
}

void Camera3DeviceClient::Notify(const camera3_notify_msg *msg) {
  if (msg == NULL) {
    SET_ERR("HAL sent NULL notify message!");
    return;
  }

  switch (msg->type) {
    case CAMERA3_MSG_SHUTTER: {
      NotifyShutter(msg->message.shutter);
      break;
    }
    case CAMERA3_MSG_ERROR: {
      NotifyError(msg->message.error);
      break;
    }
    default:
      SET_ERR("Unknown notify message from HAL: %d", msg->type);
  }
}

void Camera3DeviceClient::NotifyError(const camera3_error_msg_t &msg) {

  static const CameraErrorCode halErrorMap[CAMERA3_MSG_NUM_ERRORS] = {
      ERROR_CAMERA_INVALID_ERROR, ERROR_CAMERA_DEVICE, ERROR_CAMERA_REQUEST,
      ERROR_CAMERA_RESULT,        ERROR_CAMERA_BUFFER};

  CameraErrorCode errorCode =
      ((msg.error_code >= 0) && (msg.error_code < CAMERA3_MSG_NUM_ERRORS))
          ? halErrorMap[msg.error_code]
          : ERROR_CAMERA_INVALID_ERROR;

  CaptureResultExtras resultExtras;
  switch (errorCode) {
    case ERROR_CAMERA_DEVICE:
      SET_ERR("Camera HAL reported serious device error");
      break;
    case ERROR_CAMERA_REQUEST:
      if (state_ == STATE_ERROR) {
        // Here we are removing request when the camera is error state
        // to handle camera unplug scenario, in other scenarios this
        // error will be processed as usual.
        QMMF_ERROR("%s: Error request for camera id:%d, frame_number:%u\n",
            __func__, id_, msg.frame_number);
        pthread_mutex_lock(&pending_requests_lock_);
        if (pending_requests_vector_.count(msg.frame_number)) {
          pending_requests_vector_.erase(msg.frame_number);
        }
        pthread_mutex_unlock(&pending_requests_lock_);
        break;
      }
    case ERROR_CAMERA_RESULT:
    case ERROR_CAMERA_BUFFER:
      pthread_mutex_lock(&pending_requests_lock_);
      if (pending_requests_vector_.count(msg.frame_number)) {
        PendingRequest &r = pending_requests_vector_.at(msg.frame_number);
        r.status = msg.error_code;
        resultExtras = r.resultExtras;
      } else {
        resultExtras.frameNumber = msg.frame_number;
        QMMF_ERROR(
            "%s: Camera %d: cannot find pending request for "
            "frame %u error\n",
            __func__, id_, resultExtras.frameNumber);
      }
      pthread_mutex_unlock(&pending_requests_lock_);
      if (flush_on_going_ == false) {
        if (nullptr != client_cb_.errorCb) {
          client_cb_.errorCb(errorCode, resultExtras);
        } else {
          QMMF_ERROR("%s: Camera %d: no listener available\n", __func__, id_);
        }
      }
      break;
    default:
      SET_ERR("Unknown error message from HAL: %d", msg.error_code);
      break;
  }
}

void Camera3DeviceClient::NotifyShutter(const camera3_shutter_msg_t &msg) {

  pthread_mutex_lock(&pending_requests_lock_);
  bool pending_request_found = false;
  if (pending_requests_vector_.count(msg.frame_number)) {
    pending_request_found = true;
    PendingRequest &r = pending_requests_vector_.at(msg.frame_number);

    if (nullptr != client_cb_.shutterCb) {
      client_cb_.shutterCb(r.resultExtras, msg.timestamp);
    }

    if (r.resultExtras.input) {
      if (msg.frame_number < next_shutter_input_frame_number_) {
        SET_ERR(
            "Shutter notification out-of-order. Expected "
            "notification for frame %d, got frame %d",
            next_shutter_input_frame_number_, msg.frame_number);
        pthread_mutex_unlock(&pending_requests_lock_);
        return;
      }
      next_shutter_input_frame_number_ = msg.frame_number + 1;
    } else {
      if (msg.frame_number < next_shutter_frame_number_) {
        SET_ERR(
            "Shutter notification out-of-order. Expected "
            "notification for frame %d, got frame %d",
            next_shutter_frame_number_, msg.frame_number);
        pthread_mutex_unlock(&pending_requests_lock_);
        return;
      }
      next_shutter_frame_number_ = msg.frame_number + 1;
    }

    r.shutterTS = msg.timestamp;

    SendCaptureResult(r.pendingMetadata, r.resultExtras,
                      r.partialResult.composedResult, msg.frame_number);
    ReturnOutputBuffers(r.pendingBuffers.data(), r.pendingBuffers.size(),
                        r.shutterTS, msg.frame_number);
    r.pendingBuffers.clear();

    RemovePendingRequestLocked(msg.frame_number);
  }
  pthread_mutex_unlock(&pending_requests_lock_);

  if (!pending_request_found) {
    SET_ERR("Shutter notification with invalid frame number %d",
            msg.frame_number);
  }
}

#if defined(CAMERA_HAL_API_VERSION) && (CAMERA_HAL_API_VERSION >= 0x0307)
void Camera3DeviceClient::ReturnStreamBuffers(uint32_t num_buffers, const camera3_stream_buffer_t* const* buffers) {
  QMMF_ERROR("%s: return buffer %d: not supported", __func__, num_buffers);
}

camera3_buffer_request_status_t Camera3DeviceClient::RequestStreamBuffers(uint32_t num_buffer_reqs,
    const camera3_buffer_request_t *buffer_reqs, uint32_t *num_returned_buf_reqs,
    camera3_stream_buffer_ret_t *returned_buf_reqs) {
  QMMF_ERROR("%s: request buffer %d: not supported", __func__, num_buffer_reqs);

  *num_returned_buf_reqs = 0;
  return CAMERA3_BUF_REQ_FAILED_UNKNOWN;
}
#endif

void Camera3DeviceClient::UpdateCameraStatus(bool status) {
  is_camera_device_available_ = status;
}

void Camera3DeviceClient::SendCaptureResult(
    CameraMetadata &pendingMetadata, CaptureResultExtras &resultExtras,
    CameraMetadata &collectedPartialResult, uint32_t frameNumber) {
  if (pendingMetadata.isEmpty()) return;

  if (nullptr == client_cb_.resultCb) {
    return;
  }

  // when camera operation mode is frame selection, video packets from
  // pickframe node will be held on EISv3 module for several seconds at most
  // so frame sequence passed by camx will be out of order,

  if (!CAM_OPMODE_IS_FRAMESELECTION(cam_opmode_)) {
    if (resultExtras.input) {
      if (frameNumber < next_result_input_frame_number_) {
        SET_ERR(
            "Out-of-order result received! "
            "(arriving frame number %d, expecting %d)",
            frameNumber, next_result_input_frame_number_);
        return;
      }
      next_result_input_frame_number_ = frameNumber + 1;
    } else {
      if (frameNumber < next_result_frame_number_) {
        SET_ERR(
            "Out-of-order result received! "
            "(arriving frame number %d, expecting %d)",
            frameNumber, next_result_frame_number_);
        return;
      }
      next_result_frame_number_ = frameNumber + 1;
    }
  }

  CaptureResult captureResult;
  captureResult.resultExtras = resultExtras;
  captureResult.metadata = pendingMetadata;

  if (captureResult.metadata.update(ANDROID_REQUEST_FRAME_COUNT,
                                    (int32_t *)&frameNumber, 1) != 0) {
    SET_ERR("Unable to update frame number (%d)", frameNumber);
    return;
  }

  if (is_partial_result_supported_ && !collectedPartialResult.isEmpty()) {
    captureResult.metadata.append(collectedPartialResult);
  }

  captureResult.metadata.sort();

  camera_metadata_entry entry =
      captureResult.metadata.find(ANDROID_SENSOR_TIMESTAMP);
  if (entry.count == 0) {
    SET_ERR("No timestamp from Hal for frame %d!", frameNumber);
    return;
  }

  client_cb_.resultCb(captureResult);
}

void Camera3DeviceClient::ReturnOutputBuffers(
    const camera3_stream_buffer_t *outputBuffers, size_t numBuffers,
    int64_t timestamp, int64_t frame_number) {
  for (size_t i = 0; i < numBuffers; i++) {
    Camera3Stream *stream = Camera3Stream::CastTo(outputBuffers[i].stream);
    stream->ReturnBufferToClient(outputBuffers[i], timestamp, frame_number);

    if (CAMERA3_BUFFER_STATUS_ERROR == outputBuffers[i].status &&
        flush_on_going_ == false) {
      CaptureResultExtras resultExtras;
      if (pending_requests_vector_.count(frame_number)) {
        PendingRequest &r = pending_requests_vector_.at(frame_number);
        r.status = CAMERA3_MSG_ERROR_BUFFER;
        resultExtras = r.resultExtras;
      } else {
        resultExtras.frameNumber = frame_number;
        QMMF_ERROR("%s: Camera %d: cannot find pending request for "
            "frame %u\n", __func__, id_, resultExtras.frameNumber);
      }
      client_cb_.errorCb(ERROR_CAMERA_BUFFER, resultExtras);
    }
  }
}

int32_t Camera3DeviceClient::ReturnStreamBuffer(StreamBuffer buffer) {
  Camera3Stream *stream;
  bool isStreamValid;
  int32_t res = 0;
  pthread_mutex_lock(&lock_);

  isStreamValid = (streams_.count(buffer.stream_id) != 0);

  switch (state_) {
    case STATE_ERROR:
      if (!isStreamValid) {
        QMMF_ERROR("%s: Device has encountered a serious error\n", __func__);
        res = -ENOSYS;
        goto exit;
      } else {
        //Successful buffer received after unplug, so even though the Camera in
        //error state we need to handle this buffer.
        break;
      }
    case STATE_NOT_INITIALIZED:
    case STATE_CLOSED:
    case STATE_NOT_CONFIGURED:
      QMMF_ERROR("%s: Device is not initialized/configured!\n", __func__);
      res = -ENOSYS;
      goto exit;
    case STATE_CONFIGURED:
    case STATE_RUNNING:
      break;
    default:
      QMMF_ERROR("%s: Unknown state: %d", __func__, state_);
      res = -ENOSYS;
      goto exit;
  }

  if (!isStreamValid) {
    QMMF_ERROR("%s: Stream %d does not exist\n", __func__, buffer.stream_id);
    res = -EINVAL;
    goto exit;
  }

  stream = streams_[buffer.stream_id];
  if (0 != res) {
    QMMF_ERROR("%s: Can't return buffer to its stream: %s (%d)\n", __func__,
               strerror(-res), res);
  }

  res = stream->ReturnBuffer(buffer);
exit:

  pthread_mutex_unlock(&lock_);
  return res;
}

void Camera3DeviceClient::RemovePendingRequestLocked(uint32_t frameNumber) {
  PendingRequest &request = pending_requests_vector_.at(frameNumber);

  int64_t sensorTS = request.sensorTS;
  int64_t shutterTS = request.shutterTS;

  if (request.buffersRemaining == 0 &&
      (0 != request.status || (request.isMetaPresent && shutterTS != 0))) {

    if (0 == request.status && sensorTS != shutterTS) {
      SET_ERR(
          "sensor timestamp (%ld) for frame %d doesn't match shutter"
          " timestamp (%ld)\n",
          sensorTS, frameNumber, shutterTS);
    }

    ReturnOutputBuffers(request.pendingBuffers.data(),
                        request.pendingBuffers.size(), 0,
                        frameNumber);

    if (0 != request.status && (!request.isMetaPresent || shutterTS == 0)) {
      QMMF_INFO("%s: Received error in the capture request. Added to the error"
          " requests vector.\n", __func__);
      pending_error_requests_vector_.emplace(frameNumber, request);
    }

    pending_requests_vector_.erase(frameNumber);
  }
}

int32_t Camera3DeviceClient::LoadHWModule(const char *moduleId,
                                          const struct hw_module_t **pHmi) {

  int32_t status;

  if (NULL == moduleId) {
    QMMF_ERROR("%s: Invalid module id! \n", __func__);
    return -EINVAL;
  }

  status = hw_get_module(moduleId, pHmi);

  return status;
}

int32_t Camera3DeviceClient::GetCameraInfo(uint32_t idx, CameraMetadata *info) {
  if (NULL == info) {
    return -EINVAL;
  }

  if (idx >= number_of_cameras_) {
    return -EINVAL;
  }

  if (NULL == camera_module_) {
    return -ENODEV;
  }

  camera_info cam_info;
  int32_t res = camera_module_->get_camera_info(idx, &cam_info);
  if (0 != res) {
    QMMF_ERROR("%s: Error during camera static info query: %s!\n", __func__,
               strerror(res));
    return res;
  }

  if (!is_camera_device_available_) {
    QMMF_ERROR("%s: Camera device is not available: %s!\n", __func__);
    return -ENODEV;
  }

  *info = cam_info.static_camera_characteristics;

  return res;
}

int32_t Camera3DeviceClient::SubmitRequest(Camera3Request request,
                                           bool streaming,
                                           int64_t *lastFrameNumber) {
  std::vector<Camera3Request> requestList;
  requestList.push_back(request);
  return SubmitRequestList(requestList, streaming, lastFrameNumber);
}

int32_t Camera3DeviceClient::SubmitRequestList(std::vector<Camera3Request> requests,
                                               bool streaming,
                                               int64_t *lastFrameNumber) {
  int32_t res = 0;
  if (requests.empty()) {
    QMMF_ERROR("%s: Camera %d: Received empty!\n", __func__, id_);
    return -EINVAL;
  }

  std::vector<CameraMetadata> metadataRequestList;
  int32_t requestId = next_request_id_;
  int32_t temp_request_id = requestId;

  pthread_mutex_lock(&lock_);
  current_request_ids_.clear();

  switch (state_) {
    case STATE_ERROR:
      QMMF_ERROR("%s: Device has encountered a serious error\n", __func__);
      res = -ENOSYS;
      goto exit;
    case STATE_NOT_INITIALIZED:
    case STATE_CLOSED:
      QMMF_ERROR("%s: Device not initialized\n", __func__);
      res = -ENOSYS;
      goto exit;
    case STATE_NOT_CONFIGURED:
    case STATE_CONFIGURED:
    case STATE_RUNNING:
      break;
    default:
      QMMF_ERROR("%s: Unknown state: %d", __func__, state_);
      res = -ENOSYS;
      goto exit;
  }

  for (std::vector<Camera3Request>::iterator it = requests.begin();
       it != requests.end(); ++it) {
    Camera3Request request = *it;
    CameraMetadata metadata(request.metadata);
    if (metadata.isEmpty()) {
      QMMF_ERROR("%s: Camera %d: Received invalid meta.\n", __func__, id_);
      res = -EINVAL;
      goto exit;
    } else if (request.streamIds.empty()) {
      QMMF_ERROR(
          "%s: Camera %d: Requests must have at least one"
          " stream.\n",
          __func__, id_);
      res = -EINVAL;
      goto exit;
    }

    std::vector<int32_t> request_stream_id {request.streamIds.begin(), request.streamIds.end()};
    std::sort(request_stream_id.begin(), request_stream_id.end());
    int32_t prev_id = -1;
    int32_t input_stream_idx = -1;
    for (uint32_t i = 0; i < request_stream_id.size(); ++i) {
      if (input_stream_.stream_id == request_stream_id[i]) {
        metadata.update(ANDROID_REQUEST_INPUT_STREAMS,
                        &input_stream_.stream_id, 1);
        input_stream_idx = i;
        continue;
      }
      Camera3Stream *stream = streams_[request_stream_id[i]];

      if (NULL == stream) {
        QMMF_ERROR("%s: Camera %d: Request contains invalid stream!\n",
                   __func__, id_);
        res = -EINVAL;
        goto exit;
      }

      if (prev_id == request_stream_id[i]) {
        QMMF_ERROR("%s: Camera %d: Stream with id: %d appears several times in "
            "request!\n", __func__, id_, prev_id);
        res = -EINVAL;
        goto exit;
      } else {
        prev_id = request_stream_id[i];
      }
    }

    if (0 <= input_stream_idx) {
      auto it = request_stream_id.begin();
      request_stream_id.erase(it + input_stream_idx);
    }
    metadata.update(ANDROID_REQUEST_OUTPUT_STREAMS, &request_stream_id[0],
                    request_stream_id.size());

    metadata.update(ANDROID_REQUEST_ID, &temp_request_id, 1);
    metadataRequestList.push_back(metadata);
    current_request_ids_.push_back(temp_request_id);
    temp_request_id++;
  }
  next_request_id_ = temp_request_id;

  res = AddRequestListLocked(metadataRequestList, streaming, lastFrameNumber);
  if (0 != res) {
    QMMF_ERROR("%s: Camera %d: Got error %d after trying to set capture\n",
               __func__, id_, res);
  }

  if (0 == res) {
    res = requestId;

    if (streaming) {
      repeating_requests_.push_back(requestId);
    }
  }

exit:
  pthread_mutex_unlock(&lock_);

  return res;
}

int32_t Camera3DeviceClient::AddRequestListLocked(
    const std::vector<CameraMetadata> &requests, bool streaming,
    int64_t *lastFrameNumber) {
  RequestList requestList;
  RequestList requestListReproc;

  int32_t res = GetRequestListLocked(requests, &requestList, &requestListReproc);
  if (0 != res) {
    return res;
  }

  if (requestList.empty() == requestListReproc.empty()) {
    QMMF_ERROR("%s: Invalid request list. requests: %d reproc: %d\n", __func__,
      requestList.empty(), requestListReproc.empty());
    return -EINVAL;
  }

  if (!requestListReproc.empty()) {
    res = request_handler_.QueueReprocRequestList(requestListReproc,
        lastFrameNumber);
  } else if (!requestList.empty()) {
    if (streaming) {
      res = request_handler_.SetRepeatingRequests(requestList, lastFrameNumber);
    } else {
      res = request_handler_.QueueRequestList(requestList, lastFrameNumber);
    }
  }
  if (0 != res) {
    QMMF_ERROR("%s: Request queue failed: %d reproc: %d\n", __func__, res,
        !requestListReproc.empty());
    return res;
  }

  WaitUntilStateThenRelock(true, WAIT_FOR_RUNNING);
  if (0 != res) {
    SET_ERR_L("Unable to change to running in %f seconds!",
              WAIT_FOR_RUNNING / 1e9);
  }

  return res;
}

int32_t Camera3DeviceClient::GetRequestListLocked(
    const std::vector<CameraMetadata> &metadataList,
    RequestList *requestList,
    RequestList *requestListReproc) {
  if (requestList == NULL) {
    QMMF_ERROR("%s: Invalid requestList\n", __func__);
    return -EINVAL;
  }

  int32_t burstId = 0;
  for (std::vector<CameraMetadata>::const_iterator it = metadataList.begin();
       it != metadataList.end(); ++it) {
    CaptureRequest newRequest;
    int32_t res = GenerateCaptureRequestLocked(*it, newRequest);
    if (0 != res) {
      QMMF_ERROR("%s: Can't create capture request\n", __func__);
      return -EINVAL;
    }

    // Setup burst Id and request Id
    newRequest.resultExtras.burstId = burstId++;
    if (it->exists(ANDROID_REQUEST_ID)) {
      if (it->find(ANDROID_REQUEST_ID).count == 0) {
        QMMF_ERROR("%s: Empty RequestID\n", __func__);
        return -EINVAL;
      }
      newRequest.resultExtras.requestId =
          it->find(ANDROID_REQUEST_ID).data.i32[0];
    } else {
      QMMF_ERROR("%s: RequestID missing\n", __func__);
      return -EINVAL;
    }

    requestList->push_back(newRequest);

  }

  return 0;
}

int32_t Camera3DeviceClient::GenerateCaptureRequestLocked(
    const CameraMetadata &request, CaptureRequest &captureRequest) {
  int32_t res;

  if (state_ == STATE_NOT_CONFIGURED || reconfig_) {
    res = ConfigureStreamsLocked();
    if (res == -EINVAL && state_ == STATE_NOT_CONFIGURED) {
      QMMF_ERROR("%s: No streams configured\n", __func__);
      return -EINVAL;
    }
    if (0 != res) {
      QMMF_ERROR("%s: Can't set up streams: %s (%d)\n", __func__,
                 strerror(-res), res);
      return res;
    }
    if (state_ == STATE_NOT_CONFIGURED) {
      QMMF_ERROR("%s: No streams configured\n", __func__);
      return -ENODEV;
    }
  }

  captureRequest.metadata = request;

  camera_metadata_entry_t streams =
      captureRequest.metadata.find(ANDROID_REQUEST_OUTPUT_STREAMS);
  if (streams.count == 0) {
    QMMF_ERROR("%s: Zero output streams specified!\n", __func__);
    return -EINVAL;
  }

  for (uint32_t i = 0; i < streams.count; i++) {
    int idx = streams.data.i32[i];
    if (streams_.count(idx) == 0) {
      QMMF_ERROR("%s: Request references unknown stream %d\n", __func__,
                 streams.data.u8[i]);
      return -EINVAL;
    }
    Camera3Stream *stream = streams_[idx];

    if (stream->IsConfigureActive()) {
      res = stream->EndConfigure();
      if (0 != res) {
        QMMF_ERROR("%s: Stream configuration failed %d: %s (%d)\n", __func__,
                   stream->GetId(), strerror(-res), res);
        return -ENODEV;
      }
    }

    if (stream->IsPrepareActive()) {
      QMMF_ERROR("%s: Request contains a stream that is currently being"
          "prepared!\n", __func__);
        return -ENOSYS;
    }

    captureRequest.streams.push_back(stream);
  }
  captureRequest.metadata.erase(ANDROID_REQUEST_OUTPUT_STREAMS);

  captureRequest.input = NULL;
  captureRequest.resultExtras.input = false;
  if (captureRequest.metadata.exists(ANDROID_REQUEST_INPUT_STREAMS)) {
    streams =
          captureRequest.metadata.find(ANDROID_REQUEST_INPUT_STREAMS);
    if (1 == streams.count) {
      if (input_stream_.stream_id == streams.data.i32[0]) {
        captureRequest.input = &input_stream_;
        captureRequest.resultExtras.input = true;
      } else {
        QMMF_ERROR("%s: Request contains input stream with id: %d that"
            "doesn't match the registered one: %d\n", __func__,
            streams.data.i32[0], input_stream_.stream_id);
          return -ENOSYS;
      }
    } else {
      QMMF_ERROR("%s: Request contains multiple input streams: %d\n", __func__,
                 streams.count);
        return -ENOSYS;
    }
    captureRequest.metadata.erase(ANDROID_REQUEST_INPUT_STREAMS);
  }

  return 0;
}

void Camera3DeviceClient::SetErrorState(const char *fmt, ...) {
  pthread_mutex_lock(&lock_);
  va_list args;
  va_start(args, fmt);

  SetErrorStateLockedV(fmt, args);

  va_end(args);
  pthread_mutex_unlock(&lock_);
}

void Camera3DeviceClient::SetErrorStateV(const char *fmt, va_list args) {
  pthread_mutex_lock(&lock_);
  SetErrorStateLockedV(fmt, args);
  pthread_mutex_unlock(&lock_);
}

void Camera3DeviceClient::SetErrorStateLocked(const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);

  SetErrorStateLockedV(fmt, args);

  va_end(args);
}

void Camera3DeviceClient::SetErrorStateLockedV(const char *fmt, va_list args) {
  va_list tmp_args;
  va_copy(tmp_args, args);
  const int size = std::vsnprintf(nullptr, 0, fmt, tmp_args);
  va_end(tmp_args);

  last_error_.clear();
  last_error_.resize(size + 1, '\0');
  std::vsnprintf(&last_error_[0], size + 1, fmt, args);

  QMMF_ERROR("%s: Camera %d: %s\n", __func__, id_, last_error_.c_str());

  if (state_ == STATE_ERROR || state_ == STATE_NOT_INITIALIZED ||
      state_ == STATE_CLOSED)
    return;

  request_handler_.RequestExit();
  InternalUpdateStatusLocked(STATE_ERROR);

  if (nullptr != client_cb_.errorCb) {
    client_cb_.errorCb(ERROR_CAMERA_DEVICE, CaptureResultExtras());
  }
}

void Camera3DeviceClient::NotifyStatus(bool idle) {
  pthread_mutex_lock(&lock_);
  if (state_ != STATE_RUNNING && state_ != STATE_CONFIGURED) {
    pthread_mutex_unlock(&lock_);
    return;
  }
  InternalUpdateStatusLocked(idle ? STATE_CONFIGURED : STATE_RUNNING);

  if (pause_state_notify_) {
    pthread_mutex_unlock(&lock_);
    return;
  }

  pthread_mutex_unlock(&lock_);

  if (idle && nullptr != client_cb_.idleCb) {
    client_cb_.idleCb();
  }
}

int32_t Camera3DeviceClient::Flush(int64_t *lastFrameNumber) {
  int32_t res;
  pthread_mutex_lock(&lock_);
  flush_on_going_ = true;
  pthread_mutex_unlock(&lock_);

  // We can't hold locks during RequestHandler call to Clear() or HAL call to
  // flush. Some implementations will return buffers to client from the same
  // context and this can cause deadlock if client tries to return them.
  res = request_handler_.Clear(lastFrameNumber);
  if (0 != res) {
    QMMF_ERROR("%s: Couldn't reset request handler, err: %d!", __func__, res);
    pthread_mutex_lock(&lock_);
    goto exit;
  }

  res = device_->ops->flush(device_);

  pthread_mutex_lock(&lock_);

  if (0 == res) {
    repeating_requests_.clear();
  }

exit:

  flush_on_going_ = false;
  pthread_mutex_unlock(&lock_);

  return res;
}

int32_t Camera3DeviceClient::WaitUntilIdle() {
  pthread_mutex_lock(&lock_);
  int32_t res = WaitUntilDrainedLocked();
  pthread_mutex_unlock(&lock_);

  return res;
}

int32_t Camera3DeviceClient::WaitUntilDrainedLocked() {
  switch (state_) {
    case STATE_NOT_INITIALIZED:
    case STATE_CLOSED:
    case STATE_NOT_CONFIGURED:
      return 0;
    case STATE_CONFIGURED:
    // To avoid race conditions, check with tracker to be sure
    case STATE_ERROR:
    case STATE_RUNNING:
      // Need to verify shut down
      break;
    default:
      SET_ERR_L("Unexpected status: %d", state_);
      return -ENOSYS;
  }

  int32_t res = WaitUntilStateThenRelock(false, WAIT_FOR_SHUTDOWN);
  if (0 != res) {
    SET_ERR_L("Error waiting for HAL to drain: %s (%d)", strerror(-res), res);
    for (uint32_t i = 0; i < streams_.size(); i++) {
      streams_[i]->PrintBuffersInfo();
    }
    if (input_stream_.stream_id != -1) {
      QMMF_ERROR("%s: Input Stream: dim: %ux%u, fmt: %d "
          "input_buffer_cnt(%u)", __func__,
          input_stream_.width, input_stream_.height,
          input_stream_.format, input_stream_.input_buffer_cnt);
    }
  }

  pthread_mutex_lock(&pending_requests_lock_);
  pending_error_requests_vector_.clear();
  pthread_mutex_unlock(&pending_requests_lock_);

  return res;
}

void Camera3DeviceClient::InternalUpdateStatusLocked(State state) {
  state_ = state;
  current_state_updates_.push_back(state_);
  pthread_cond_broadcast(&state_updated_);
}

int32_t Camera3DeviceClient::InternalPauseAndWaitLocked() {
  request_handler_.TogglePause(true);
  pause_state_notify_ = true;

  int32_t res = WaitUntilStateThenRelock(false, WAIT_FOR_SHUTDOWN);
  if (0 != res) {
    SET_ERR_L("Can't idle device in %f seconds!", WAIT_FOR_SHUTDOWN / 1e9);
  }

  return res;
}

int32_t Camera3DeviceClient::InternalResumeLocked() {
  int32_t res = 0;

  bool pending_request;
  request_handler_.TogglePause(false, pending_request);
  if (pending_request == false) {
    return res;
  }

  res = WaitUntilStateThenRelock(true, WAIT_FOR_RUNNING);
  if (0 != res) {
    SET_ERR_L("Can't transition to active in %f seconds!",
              WAIT_FOR_RUNNING / 1e9);
  }

  pause_state_notify_ = false;
  return res;
}

int32_t Camera3DeviceClient::WaitUntilStateThenRelock(bool active,
                                                      int64_t timeout) {
  int32_t res = 0;

  uint32_t startIndex = 0;
  if (state_listeners_ == 0) {
    current_state_updates_.clear();
  } else {
    startIndex = current_state_updates_.size();
  }

  state_listeners_++;

  bool stateSeen = false;
  do {
    if (active == (state_ == STATE_RUNNING)) {
      break;
    }

    res = cond_wait_relative(&state_updated_, &lock_, timeout);
    if (0 != res) {
      break;
    }

    for (uint32_t i = startIndex; i < current_state_updates_.size(); i++) {
      if (active == (current_state_updates_[i] == STATE_RUNNING)) {
        stateSeen = true;
        break;
      }
    }
  } while (!stateSeen);

  state_listeners_--;

  return res;
}

int32_t Camera3DeviceClient::Prepare(int streamId) {
  int32_t res = 0;
  pthread_mutex_lock(&lock_);

  Camera3Stream *stream;
  if (streams_.count(streamId) == 0) {
      QMMF_ERROR("%s: Stream %d is invalid!\n", __func__, streamId);
      res = -EINVAL;
  }

  stream = streams_[streamId];
  if (stream->IsStreamActive()) {
    QMMF_ERROR("%s: Stream %d has already received requests\n", __func__,
               streamId);
    res = -EINVAL;
    goto exit;
  }

  if (request_handler_.IsStreamActive(*stream)) {
    QMMF_ERROR("%s: Stream %d already has pending requests\n", __func__,
               streamId);
    res = -EINVAL;
    goto exit;
  }

  res = prepare_handler_.Prepare(stream);

exit:

  pthread_mutex_unlock(&lock_);

  return res;
}

int32_t Camera3DeviceClient::TearDown(int streamId) {
  int32_t res = 0;
  pthread_mutex_lock(&lock_);

  Camera3Stream *stream;
  if (streams_.count(streamId) == 0) {
      QMMF_ERROR("%s: Stream %d is invalid!\n", __func__, streamId);
      res = -EINVAL;
  }

  stream = streams_[streamId];
  if (request_handler_.IsStreamActive(*stream)) {
    QMMF_ERROR("%s: Stream %d already has pending requests\n", __func__,
               streamId);
    res = -EINVAL;
    goto exit;
  }

  res = stream->TearDown();

exit:

  pthread_mutex_unlock(&lock_);

  return res;
}

int32_t Camera3DeviceClient::SetCameraSessionParam(
    const CameraMetadata &meta) {
  int32_t res = 0;
  pthread_mutex_lock(&lock_);

  session_metadata_.clear();
  res = session_metadata_.append(meta);
  if (res != 0)
    QMMF_ERROR("%s Append cammera session metadata failed!\n", __func__);

  pthread_mutex_unlock(&lock_);
  return res;
}

void Camera3DeviceClient::processCaptureResult(
    const camera3_callback_ops *cb, const camera3_capture_result *result) {
  Camera3DeviceClient *ctx = const_cast<Camera3DeviceClient *>(
      static_cast<const Camera3DeviceClient *>(cb));
  ctx->HandleCaptureResult(result);
}

void Camera3DeviceClient::notifyFromHal(const camera3_callback_ops *cb,
                                        const camera3_notify_msg *msg) {
  Camera3DeviceClient *ctx = const_cast<Camera3DeviceClient *>(
      static_cast<const Camera3DeviceClient *>(cb));
  ctx->Notify(msg);
}

#if defined(CAMERA_HAL_API_VERSION) && (CAMERA_HAL_API_VERSION >= 0x0307)
camera3_buffer_request_status_t Camera3DeviceClient::requestStreamBuffers(
    const struct camera3_callback_ops *cb, uint32_t num_buffer_reqs,
    const camera3_buffer_request_t *buffer_reqs, uint32_t *num_returned_buf_reqs,
    camera3_stream_buffer_ret_t *returned_buf_reqs) {
  Camera3DeviceClient *ctx = const_cast<Camera3DeviceClient *>(
      static_cast<const Camera3DeviceClient *>(cb));
  if (num_buffer_reqs == 0 || buffer_reqs == nullptr || num_returned_buf_reqs == nullptr || returned_buf_reqs == nullptr)
  {
    return CAMERA3_BUF_REQ_FAILED_ILLEGAL_ARGUMENTS;
  }

  return ctx->RequestStreamBuffers(num_buffer_reqs, buffer_reqs, num_returned_buf_reqs, returned_buf_reqs);
}

void Camera3DeviceClient::returnStreamBuffers(
    const struct camera3_callback_ops *cb, uint32_t num_buffers,
    const camera3_stream_buffer_t* const* buffers) {
  Camera3DeviceClient *ctx = const_cast<Camera3DeviceClient *>(
      static_cast<const Camera3DeviceClient *>(cb));
  ctx->ReturnStreamBuffers(num_buffers, buffers);
}
#endif

void Camera3DeviceClient::deviceStatusChange(
    const struct camera_module_callbacks *cb, int camera_id, int new_status) {
  Camera3DeviceClient *ctx = const_cast<Camera3DeviceClient *>(
      static_cast<const Camera3DeviceClient *>(cb));
  if (new_status == CAMERA_DEVICE_STATUS_NOT_PRESENT) {
    ctx->UpdateCameraStatus(false);
    QMMF_WARN ("%s: Camera with id (%d) is not present", __func__, camera_id);
  } else if (new_status == CAMERA_DEVICE_STATUS_PRESENT) {
    ctx->UpdateCameraStatus(true);
    QMMF_DEBUG ("%s: Camera with id (%d) is present", __func__, camera_id);
  }
}

void Camera3DeviceClient::torchModeStatusChange(
    const struct camera_module_callbacks *, const char *camera_id,
    int new_status) {
  // TODO: No implementation yet
}

bool Camera3DeviceClient::IsInputROIMode() {
  return (cam_feature_flags_ &
          static_cast<uint32_t>(CamFeatureFlag::kInputROIEnable));
}

uint32_t Camera3DeviceClient::GetOpMode() {
  QMMF_DEBUG("%s: Enter: \n", __func__);

  uint32_t operation_mode = 0;

#ifndef DISABLE_OP_MODES
  if (is_raw_only_) {
    operation_mode = QCAMERA3_VENDOR_STREAM_CONFIGURATION_RAW_ONLY_MODE;
  } else {
    operation_mode = CAMERA3_STREAM_CONFIGURATION_NORMAL_MODE;
  }
#else
  operation_mode = CAMERA3_STREAM_CONFIGURATION_NORMAL_MODE;

  // Handle ZZHDR Mode
  if (cam_feature_flags_ & static_cast<uint32_t>(CamFeatureFlag::kHDR)) {
    operation_mode |= QCAMERA3_SENSORMODE_ZZHDR_OPMODE;
  }
  // Handle HFR Mode
  if (hfr_mode_enabled_) {
    operation_mode |= CAMERA3_STREAM_CONFIGURATION_CONSTRAINED_HIGH_SPEED_MODE;
  }
  // Handle EIS mode
  if (cam_feature_flags_ & static_cast<uint32_t>(CamFeatureFlag::kEIS)) {
    operation_mode |= EIS_ENABLE;
  }
  // Handle LDC mode
  if (cam_feature_flags_ & static_cast<uint32_t>(CamFeatureFlag::kLDC)) {
    operation_mode |= LDC_ENABLE;
  }
  // Handle LCAC mode
  if (cam_feature_flags_ & static_cast<uint32_t>(CamFeatureFlag::kLCAC)) {
    operation_mode |= LCAC_ENABLE;
  }
  /*
   * Below two features are mutually exclusive:
   * 1. Using force sensor mode
   * 2. Default 60 fps usecase, in which OpMode is index of 60fps
   *    in sensor mode table
   */
  if (cam_feature_flags_ &
      static_cast<uint32_t>(CamFeatureFlag::kForceSensorMode)) {
    operation_mode |= ((FORCE_SENSOR_MODE_MASK & cam_feature_flags_)
        | FORCE_SENSORMODE_ENABLE);
    QMMF_INFO("%s: Force_sensor_mode OpMode is set to 0x%x \n", __func__,
              operation_mode);

  } else if (fps_sensormode_index_ > QCAMERA3_SENSORMODE_FPS_DEFAULT_INDEX) {
    operation_mode |= (fps_sensormode_index_ << 16);
    QMMF_INFO("%s: 60+ FPS OpMode is Set 0x%x \n", __func__, operation_mode);
  }

  // Handle IFE Direct Stream
  if (cam_feature_flags_ &
      static_cast<uint32_t>(CamFeatureFlag::kIFEDirectStream)) {
    operation_mode |= IFE_DIRECT_STREAM;
    QMMF_INFO("%s: IFEDirectStream OpMode Set, operation_mode = 0x%x \n",
        __func__, operation_mode);
  }

  if (CAM_OPMODE_IS_FRAMESELECTION(cam_opmode_))
    operation_mode |= CAM_OPMODE_FRAME_SELECTION;

  if (CAM_OPMODE_IS_FASTSWTICH(cam_opmode_))
    operation_mode |= CAM_OPMODE_FAST_SWITCH;

#endif

  QMMF_DEBUG("%s: Exit: \n", __func__);

  return operation_mode;
}

}  // namespace cameraadaptor ends here

}  // namespace qmmf ends here
