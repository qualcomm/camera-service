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

#define LOG_TAG "Camera3RequestHandler"

#include <qmmf_camera3_utils.h>
#include <qmmf_camera3_device_client.h>
#include <qmmf_camera3_request_handler.h>
#include "recorder/src/service/qmmf_recorder_common.h"

#ifdef USE_LIBGBM
#include "common/memory/qmmf_gbm_interface.h"
#endif // USE_LIBGBM

#define SIG_ERROR(fmt, ...) \
  SignalError("%s: " fmt, __FUNCTION__, ##__VA_ARGS__)

namespace qmmf {

namespace cameraadaptor {

static const uint64_t kWaitWorkerTimeout = 50;

Camera3RequestHandler::Camera3RequestHandler(Camera3Monitor &monitor)
    : error_cb_(nullptr),
      mark_cb_(nullptr),
      set_error_(nullptr),
      hal3_device_(NULL),
      configuration_update_(false),
      toggle_pause_state_(false),
      paused_state_(true),
      current_frame_number_(0),
      current_input_frame_number_(0),
      streaming_last_frame_number_(NO_IN_FLIGHT_REPEATING_FRAMES),
      monitor_(monitor),
      monitor_id_(Camera3Monitor::INVALID_ID),
      batch_size_(1),
      run_worker_(false),
      cam_opmode_(0),
      request_mdata_(CameraMetadata(128,128)),
      system_event_occurred_(false) {
  pthread_mutex_init(&lock_, NULL);
  cond_init(&requests_signal_);
  cond_init(&current_request_signal_);
  pthread_mutex_init(&pause_lock_, NULL);
  cond_init(&toggle_pause_signal_);
  cond_init(&pause_state_signal_);
  ClearCaptureRequest(old_request_);
  ClearCaptureRequest(current_request_);
  memset(&cam_reqmode_params_, 0, sizeof(cam_reqmode_params_));
}

Camera3RequestHandler::~Camera3RequestHandler() {
  RequestExitAndWait();

  if (!input_buffer_map_.empty())
    input_buffer_map_.clear();

  if (0 <= monitor_id_) {
    monitor_.ReleaseMonitor(monitor_id_);
    monitor_id_ = Camera3Monitor::INVALID_ID;
  }
  {
    std::unique_lock<std::mutex> lock(worker_lock_);
    run_worker_ = false;
    worker_signal_.Signal();
  }
  if (worker_.joinable()) {
    worker_.join();
  }
  pthread_cond_destroy(&pause_state_signal_);
  pthread_cond_destroy(&toggle_pause_signal_);
  pthread_mutex_destroy(&pause_lock_);
  pthread_cond_destroy(&current_request_signal_);
  pthread_cond_destroy(&requests_signal_);
  pthread_mutex_destroy(&lock_);
}

int32_t Camera3RequestHandler::Initialize(camera3_device_t *device,
                                          uint8_t buffer_api_version,
                                          ErrorCallback error_cb,
                                          MarkRequest mark_cb,
                                          SetError set_error) {
  pthread_mutex_lock(&lock_);
  hal3_device_ = device;
  buffer_api_version_ = buffer_api_version;
  error_cb_ = error_cb;
  mark_cb_ = mark_cb;
  set_error_ = set_error;
  monitor_id_ = monitor_.AcquireMonitor();
  if (0 > monitor_id_) {
    QMMF_ERROR("%s: Unable to acquire monitor: %d\n", __func__, monitor_id_);
  }
  smooth_zoom_.Enable();
  pthread_mutex_unlock(&lock_);

  return monitor_id_;
}

void Camera3RequestHandler::FinishConfiguration(uint32_t batch_size) {
  pthread_mutex_lock(&lock_);
  configuration_update_ = true;
  batch_size_ = batch_size;
  pthread_mutex_unlock(&lock_);
}

int32_t Camera3RequestHandler::QueueRequestList(std::vector<CaptureRequest> &requests,
                                                int64_t *lastFrameNumber) {
  pthread_mutex_lock(&lock_);
  auto it = requests.begin();
  for (; it != requests.end(); ++it) {
    requests_.push_back(*it);
  }

  if (lastFrameNumber != NULL) {
    *lastFrameNumber = current_frame_number_ + requests_.size() - 1;
  }

  Resume();

  pthread_mutex_unlock(&lock_);
  return 0;
}

int32_t Camera3RequestHandler::QueueReprocRequestList(std::vector<CaptureRequest> &requests,
                                                int64_t *lastFrameNumber) {
  pthread_mutex_lock(&lock_);
  std::unique_lock<std::mutex> lock(worker_lock_);
  if (!run_worker_) {
    run_worker_ = true;
    try {
      worker_ = std::thread([this]() -> void {
          Camera3RequestHandler::ReprocLoop(this); });
    } catch (const std::exception &e) {
      QMMF_ERROR("%s: Unable to create worker thread exception: %s !",
                 __func__, e.what());
      run_worker_ = false;
      return -EINTR;
    }
  }
  auto it = requests.begin();
  for (; it != requests.end(); ++it) {
    reproc_requests_.push_back(*it);
  }

  if (lastFrameNumber != NULL) {
    *lastFrameNumber = current_frame_number_ + reproc_requests_.size() - 1;
  }

  Resume();

  worker_signal_.Signal();

  pthread_mutex_unlock(&lock_);
  return 0;
}

int32_t Camera3RequestHandler::SetRepeatingRequests(const RequestList &requests,
                                                    int64_t *lastFrameNumber) {
  pthread_mutex_lock(&lock_);
  if (lastFrameNumber != NULL) {
    *lastFrameNumber = streaming_last_frame_number_;
  }
  streaming_requests_.clear();
  streaming_requests_.insert(streaming_requests_.begin(), requests.begin(),
                            requests.end());

  Resume();

  streaming_last_frame_number_ = NO_IN_FLIGHT_REPEATING_FRAMES;
  pthread_mutex_unlock(&lock_);
  return 0;
}

int32_t Camera3RequestHandler::ClearRepeatingRequests(
    int64_t *lastFrameNumber) {
  pthread_mutex_lock(&lock_);
  streaming_requests_.clear();
  if (lastFrameNumber != NULL) {
    *lastFrameNumber = streaming_last_frame_number_;
  }
  streaming_last_frame_number_ = NO_IN_FLIGHT_REPEATING_FRAMES;
  pthread_mutex_unlock(&lock_);
  return 0;
}

int32_t Camera3RequestHandler::Clear(int64_t *lastFrameNumber) {
  pthread_mutex_lock(&lock_);
  streaming_requests_.clear();

  if (nullptr != error_cb_) {
    for (RequestList::iterator it = requests_.begin(); it != requests_.end();
         ++it) {
      (*it).resultExtras.frameNumber = current_frame_number_++;
      error_cb_(ERROR_CAMERA_REQUEST, (*it).resultExtras);
    }
  }
  requests_.clear();
  if (lastFrameNumber != NULL) {
    *lastFrameNumber = streaming_last_frame_number_;
  }
  streaming_last_frame_number_ = NO_IN_FLIGHT_REPEATING_FRAMES;

  int32_t ret = 0;
  // Skip waiting for current request if system event occurred
  if (!system_event_occurred_) {
    while (current_request_.resultExtras.requestId != -1) {
      // If there is a in-flight request, wait until it is submitted to HAL.
      ret = cond_wait_relative(&current_request_signal_, &lock_, CLEAR_TIMEOUT);
      if (-ETIMEDOUT == ret) {
        break;
      }
    }
  }

  RequestModeClear(CAM_OPMODE_FLAG_MASK);

  pthread_mutex_unlock(&lock_);
  return ret;
}

void Camera3RequestHandler::TogglePause(bool pause) {
  bool pending_request;
  TogglePause(pause, pending_request);
}

void Camera3RequestHandler::TogglePause(bool pause, bool &pending_request) {
  pthread_mutex_lock(&pause_lock_);
  pending_request = !(requests_.empty() && streaming_requests_.empty());
  toggle_pause_state_ = pause;
  pthread_cond_signal(&toggle_pause_signal_);
  pthread_mutex_unlock(&pause_lock_);
}

int32_t
Camera3RequestHandler::CreateInputBuffer(uint32_t frameNumber,
    camera3_stream_buffer_t **input_buffer) {
  camera3_stream_buffer_t *in_buffer = new camera3_stream_buffer_t();
  if (in_buffer == NULL)
    return -1;
  pthread_mutex_lock(&input_buffer_map_lock_);
  input_buffer_map_.emplace(frameNumber, in_buffer);
  *input_buffer = in_buffer;
  pthread_mutex_unlock(&input_buffer_map_lock_);
  return 0;
}

void Camera3RequestHandler::DeleteInputBuffer(uint32_t frameNumber) {
  pthread_mutex_lock(&input_buffer_map_lock_);
  camera3_stream_buffer_t *input_stream_buffer = input_buffer_map_[frameNumber];
  input_buffer_map_.erase(frameNumber);

  if (input_stream_buffer)
    delete (input_stream_buffer);

  pthread_mutex_unlock(&input_buffer_map_lock_);
}

void Camera3RequestHandler::RequestExit() {
  ThreadHelper::RequestExit();

  pthread_cond_signal(&toggle_pause_signal_);
  pthread_cond_signal(&requests_signal_);
}

void Camera3RequestHandler::RequestExitAndWait() {
  pthread_cond_signal(&toggle_pause_signal_);
  pthread_cond_signal(&requests_signal_);

  ThreadHelper::RequestExitAndWait();
}

bool Camera3RequestHandler::ThreadLoop() {
  int32_t res;

  if (WaitOnPause()) {
    return true;
  } else if (ExitPending()) {
    return false;
  }

  CaptureRequest nextRequest;
  res = GetRequest(nextRequest);
  if (0 != res) {
    return true;
  } else if (ExitPending()) {
    // Clear the request, as it will not be submitted to camera.
    pthread_mutex_lock(&lock_);
    ClearCaptureRequest(current_request_);
    pthread_cond_signal(&current_request_signal_);
    pthread_mutex_unlock(&lock_);
    return false;
  }

  res = SubmitRequest(nextRequest);
  if (0 != res) {
    if (res == -ENODEV) {
      // Stop submitting requests if the camera does not exist
      return false;
    } else {
      return true;
    }
  }

  return true;
}

void Camera3RequestHandler::ReprocLoop(Camera3RequestHandler *ctx) {
  prctl(PR_SET_NAME, "ReprocThread", 0, 0, 0);
  while(ctx->run_worker_) {
    std::unique_lock<std::mutex> lock(ctx->worker_lock_);
    while (ctx->reproc_requests_.empty()) {
      auto res = ctx->worker_signal_.WaitFor(lock,
          std::chrono::milliseconds(kWaitWorkerTimeout),
          [&] { return (ctx->run_worker_ == false); });
      if (!res) {
        QMMF_WARN("%s: Time out!", __func__);
      }
      if (!ctx->run_worker_) {
        QMMF_INFO("%s:%d: Exit", __func__, __LINE__);
        return;
      }
    }

    for (auto &nextRequest : ctx->reproc_requests_) {
      QMMF_INFO("%s: Submit reprocess request E", __func__);
      nextRequest.resultExtras.frameNumber = ctx->current_input_frame_number_;
      ctx->current_input_frame_number_++;
      ctx->current_request_ = nextRequest;

      pthread_mutex_lock(&ctx->pause_lock_);
      if (ctx->paused_state_) {
        ctx->monitor_.ChangeStateToActive(ctx->monitor_id_);
      }
      ctx->paused_state_ = false;
      pthread_mutex_unlock(&ctx->pause_lock_);

      if (ctx->configuration_update_) {
        ctx->ClearCaptureRequest(ctx->old_request_);
        ctx->configuration_update_ = false;
      }
      pthread_mutex_unlock(&ctx->lock_);

      StreamBuffer in_buf = {};
      buffer_handle_t in_buf_handle = nullptr;

      nextRequest.input->get_input_buffer(in_buf);
      nextRequest.input->input_buffer_cnt++;

      // TODO: To be removed when camera supports GBM
 #ifdef USE_LIBGBM
      in_buf_handle = GetGrallocBufferHandle(in_buf.handle);
 #else
      in_buf_handle = GetAllocBufferHandle(in_buf.handle);
 #endif // USE_LIBGBM

      nextRequest.input->buffers_map.insert(
          std::make_pair(in_buf_handle, in_buf.handle));
      camera3_stream_buffer_t camera3_in_buf = {};
      camera3_in_buf.buffer = &in_buf_handle;
      camera3_in_buf.acquire_fence = -1;
      camera3_in_buf.release_fence = -1;
      camera3_in_buf.status = CAMERA3_BUFFER_STATUS_OK;
      camera3_in_buf.stream = nextRequest.input;

      auto ret = ctx->SubmitRequest(nextRequest, &camera3_in_buf);
      if (ret == -ENODEV) {
        QMMF_ERROR("%s: Camera encountered serious error!", __func__);
        ctx->run_worker_ = false;
        break;
      }

      QMMF_INFO("%s: Submit reprocess request X", __func__);
    }
    ctx->reproc_requests_.clear();

  }
  QMMF_INFO("%s:%d: Exit", __func__, __LINE__);
}

int32_t Camera3RequestHandler::SubmitRequest(CaptureRequest &nextRequest,
                                             camera3_stream_buffer_t *in_buf) {

  int32_t res = 0;
  camera3_capture_request_t request = camera3_capture_request_t();
  request.frame_number = nextRequest.resultExtras.frameNumber;
  request.input_buffer = nullptr;
  std::vector<camera3_stream_buffer_t> outputBuffers;

  if ((old_request_.resultExtras.requestId !=
      nextRequest.resultExtras.requestId) ||
      smooth_zoom_.IsGoing())
  {
    nextRequest.metadata.sort();
    request.settings = nextRequest.metadata.getAndLock();
    old_request_ = nextRequest;
  }

  RequestStreamSubmitPreProcess(request, nextRequest);

  uint32_t totalNumBuffers = 0;

  // Handle output buffers
  for (auto i = 0; i < nextRequest.streams.size(); ++i) {
    outputBuffers.push_back(camera3_stream_buffer_t());
  }
  request.output_buffers = outputBuffers.data();
  for (size_t i = 0; i < nextRequest.streams.size(); i++) {
    if (buffer_api_version_ !=
        ANDROID_INFO_SUPPORTED_BUFFER_MANAGEMENT_VERSION_HIDL_DEVICE_3_5) {
      res = nextRequest.streams[i]->GetBuffer(&outputBuffers[i]);
    } else {
      res = nextRequest.streams[i]->GetDummyBuffer(&outputBuffers[i]);
    }
    if (0 != res) {
      QMMF_ERROR(
          "%s: Can't get stream buffer, skip this"
          " request: %s (%d)\n",
          __func__, strerror(-res), res);

      pthread_mutex_lock(&lock_);
      if (nullptr != error_cb_) {
        error_cb_(ERROR_CAMERA_REQUEST, nextRequest.resultExtras);
      }
      pthread_mutex_unlock(&lock_);
      HandleErrorRequest(request, nextRequest, outputBuffers);
      return res;
    }
    request.num_output_buffers++;
  }
  totalNumBuffers += request.num_output_buffers;

  if ((nullptr == mark_cb_) || (NULL == hal3_device_)) {
    HandleErrorRequest(request, nextRequest, outputBuffers);
    return -1;
  }

  if (nextRequest.input) {
    StreamBuffer in_buf = {};
    buffer_handle_t in_buf_handle = nullptr;

    nextRequest.input->get_input_buffer(in_buf);
    nextRequest.input->input_buffer_cnt++;

    // TODO: To be removed when camera supports GBM
#ifdef USE_LIBGBM
    in_buf_handle = GetGrallocBufferHandle(in_buf.handle);
#else
    in_buf_handle = GetAllocBufferHandle(in_buf.handle);
#endif // USE_LIBGBM
    nextRequest.input->buffers_map.insert(
    std::make_pair(in_buf_handle, in_buf.handle));
    res = CreateInputBuffer(request.frame_number, &request.input_buffer);
    if (0 != res) {
      SIG_ERROR("%s: Unable to create input buffer for request %d : %s (%d)",
              __func__, request.frame_number, strerror(-res), res);
      HandleErrorRequest(request, nextRequest, outputBuffers);
      return res;
    }

#ifdef USE_LIBGBM
    request.input_buffer->buffer = &GetGrallocBufferHandle(in_buf.handle);
#else
    request.input_buffer->buffer = &GetAllocBufferHandle(in_buf.handle);
#endif // USE_LIBGBM
    request.input_buffer->acquire_fence = -1;
    request.input_buffer->release_fence = -1;
    request.input_buffer->status = CAMERA3_BUFFER_STATUS_OK;
    request.input_buffer->stream = nextRequest.input;
    totalNumBuffers++;
  }

  // Register capture request
  res = mark_cb_(request.frame_number, totalNumBuffers,
                 nextRequest.resultExtras);
  if (0 > res) {
    SIG_ERROR("%s: Unable to register new request: %s (%d)", __func__,
              strerror(-res), res);
    HandleErrorRequest(request, nextRequest, outputBuffers);
    return res;
  }

  // Send capture request
  res = hal3_device_->ops->process_capture_request(hal3_device_, &request);
  if (0 != res) {
    SIG_ERROR("%s: Unable to submit request %d in CameraHal : %s (%d)",
              __func__, request.frame_number, strerror(-res), res);
    HandleErrorRequest(request, nextRequest, outputBuffers);
    return res;
  }

  if (RequestStreamSubmitPostProcess(request) == false) {
    if (request.settings != nullptr) {
      nextRequest.metadata.unlock(request.settings);
    }
  }

  pthread_mutex_lock(&lock_);
  ClearCaptureRequest(current_request_);
  pthread_cond_signal(&current_request_signal_);
  pthread_mutex_unlock(&lock_);

  return res;
}

bool Camera3RequestHandler::IsStreamActive(Camera3Stream &stream) {
  bool res = false;
  pthread_mutex_lock(&lock_);

  if (!current_request_.streams.empty()) {
    for (const auto &s : current_request_.streams) {
      if (stream.GetId() == s->GetId()) {
        res = true;
        goto exit;
      }
    }
  }

  for (const auto &request : requests_) {
    for (const auto &s : request.streams) {
      if (stream.GetId() == s->GetId()) {
        res = true;
        goto exit;
      }
    }
  }

  for (const auto &request : streaming_requests_) {
    for (const auto &s : request.streams) {
      if (stream.GetId() == s->GetId()) {
        res = true;
        goto exit;
      }
    }
  }

  res = false;

exit:
  pthread_mutex_unlock(&lock_);

  return res;
}

void Camera3RequestHandler::HandleErrorRequest(
    camera3_capture_request_t &request, CaptureRequest &nextRequest,
    std::vector<camera3_stream_buffer_t> &outputBuffers) {
  if (request.settings != NULL) {
    nextRequest.metadata.unlock(request.settings);
  }

  for (size_t i = 0; i < request.num_output_buffers; i++) {
    outputBuffers[i].status = CAMERA3_BUFFER_STATUS_ERROR;
    StreamBuffer b;
    memset(&b, 0, sizeof(b));
    b.handle =
      nextRequest.streams[i]->buffers_map[*outputBuffers[i].buffer];
    nextRequest.streams[i]->
      buffers_map.erase(*outputBuffers[i].buffer);
    nextRequest.streams[i]->ReturnBuffer(b);
  }

  pthread_mutex_lock(&lock_);
  ClearCaptureRequest(current_request_);
  pthread_cond_signal(&current_request_signal_);
  pthread_mutex_unlock(&lock_);
}

int32_t Camera3RequestHandler::GetRequest(CaptureRequest &request) {
  int32_t res = 0;
  CaptureRequest nextRequest;
  bool found = false;

  pthread_mutex_lock(&lock_);

  while (requests_.empty()) {
    if (!streaming_requests_.empty()) {
      RequestList request_list;
      RequestList::iterator it = streaming_requests_.begin();
      CaptureRequest realRequest;

      for (; it != streaming_requests_.end(); ++it) {
        if (RequestStreamGetProcess(it, realRequest,
            (it == streaming_requests_.begin())) == false) {
          realRequest = *it;
        }
        smooth_zoom_.Update(realRequest);
        request_list.push_back(realRequest);
      }

      const RequestList &requests = request_list;
      RequestList::const_iterator firstRequest = requests.begin();
      nextRequest = *firstRequest;
      requests_.insert(requests_.end(), ++firstRequest, requests.end());

      streaming_last_frame_number_ =
          current_frame_number_ + requests.size() - 1;

      nextRequest.resultExtras.frameNumber = current_frame_number_;
      current_frame_number_++;

      found = true;
      break;
    }

    cond_wait_relative(&requests_signal_, &lock_, WAIT_TIMEOUT);
    if ((requests_.empty() && streaming_requests_.empty()) || ExitPending()) {
      pthread_mutex_lock(&pause_lock_);
      if (paused_state_ == false) {
        paused_state_ = true;
        monitor_.ChangeStateToIdle(monitor_id_);
      }
      pthread_mutex_unlock(&pause_lock_);
      res = -ETIMEDOUT;
      goto exit;
    }
  }

  if (!found) {
    RequestList::iterator reproc_request = requests_.begin();
    for (; reproc_request != requests_.end(); reproc_request++) {
      if (reproc_request->input) {
        nextRequest = *reproc_request;
        requests_.erase(reproc_request);

        nextRequest.resultExtras.frameNumber = current_frame_number_;
        current_frame_number_++;

        found = true;
        break;
      }
    }
  }

  if (!found) {
    RequestList::iterator firstRequest = requests_.begin();
    nextRequest = *firstRequest;
    requests_.erase(firstRequest);

    nextRequest.resultExtras.frameNumber = current_frame_number_;
    current_frame_number_++;
  }

  pthread_mutex_lock(&pause_lock_);
  if (paused_state_) {
    monitor_.ChangeStateToActive(monitor_id_);
  }
  paused_state_ = false;
  pthread_mutex_unlock(&pause_lock_);

  if (configuration_update_) {
    ClearCaptureRequest(old_request_);
    configuration_update_ = false;
  }

  current_request_ = nextRequest;
  request = nextRequest;

exit:

  pthread_mutex_unlock(&lock_);

  return res;
}

void Camera3RequestHandler::ClearCaptureRequest(CaptureRequest &request) {
  request.streams.clear();
  request.metadata.clear();
  memset(&request.resultExtras, 0, sizeof(CaptureResultExtras));
  request.resultExtras.requestId = -1;
}

bool Camera3RequestHandler::WaitOnPause() {
  int32_t res;
  pthread_mutex_lock(&pause_lock_);
  /* the full batch request packet should be send before wait */
  if (current_frame_number_ % batch_size_) {
    res = false;
    goto exit;
  }
  while (toggle_pause_state_) {
    if (paused_state_ == false) {
      paused_state_ = true;
      monitor_.ChangeStateToIdle(monitor_id_);
    }

    int32_t ret =
        cond_wait_relative(&toggle_pause_signal_, &pause_lock_, WAIT_TIMEOUT);
    if ((-ETIMEDOUT == ret) || ExitPending()) {
      res = true;
      goto exit;
    }
  }

  res = false;

exit:

  pthread_mutex_unlock(&pause_lock_);

  return res;
}

void Camera3RequestHandler::Resume() {
  pthread_cond_signal(&requests_signal_);
  pthread_mutex_lock(&pause_lock_);
  if (!toggle_pause_state_) {
    monitor_.ChangeStateToActive(monitor_id_);
    paused_state_ = false;
  }
  pthread_mutex_unlock(&pause_lock_);
}

void Camera3RequestHandler::SignalError(const char *fmt, ...) {
  if (nullptr != set_error_) {
    va_list args;
    va_start(args, fmt);
    set_error_(fmt, args);
    va_end(args);
  }
}

void Camera3RequestHandler::SetRequestMode(CamOperationMode mode) {
  if (!CAM_OPMODE_FLAG_VALID(mode)) {
    QMMF_ERROR("%s: mode (0x%x) invalid", __func__, mode);
    return ;
  }

  if (mode != cam_opmode_) {
    cam_reqmode_lock_.lock();

    cam_opmode_ = mode;

    if (CAM_OPMODE_IS_FRAMESELECTION(cam_opmode_)) {
      cam_reqmode_params_.frame_selection.total_selected_frames = 0;
      cam_reqmode_params_.frame_selection.available_frames = 0;
      cam_reqmode_params_.frame_selection.cur_state = kFrameSelStateIdle;
    }

    cam_reqmode_lock_.unlock();

    QMMF_INFO("%s: Camera Operation Mode update (%d)", __func__, cam_opmode_);
  }
}

void Camera3RequestHandler::UpdateRequestedStreams(CamReqModeInputParams &params) {

  if (CAM_OPMODE_IS_FRAMESELECTION(cam_opmode_)) {
    int32_t prev_frms, cur_frms;
    uint32_t new_frms;
    FrameSelectionState cur_state;
    bool frame_update = false;

    QMMF_VERBOSE("%s:FrameSel: cur_state(%d) "
        " target frame num(%d) cap frame num(%d)", __func__,
        cam_reqmode_params_.frame_selection.cur_state,
        cam_reqmode_params_.frame_selection.target_frame_num,
        params.frame_selection.cap_frame_num);

    cam_reqmode_lock_.lock();

    // frame selection state machine
    // +----------------+
    // |      Idle      | <+
    // +----------------+  |
    //   |                 |
    //   | video buffer    |
    //   | added, frame    |
    //   | number as FN1   |
    //   v                 |
    // +----------------+  |
    // |  Idle2Active   |  |
    // +----------------+  |
    //   |                 |
    //   | HAL return      |
    //   | FN2 = FN1       | hal return
    //   v                 | FN4 = FN3
    // +----------------+  |
    // |     Active     |  |
    // +----------------+  |
    //   |                 |
    //   | video buffer    |
    //   | removed,        |
    //   | frame number    |
    //   | as FN3          |
    //   v                 |
    // +----------------+  |
    // |  Active2Idle   | -+
    // +----------------+

    if (CAM_OPMODE_IS_FASTSWTICH(cam_opmode_)) {
      cur_state = cam_reqmode_params_.frame_selection.cur_state;

      if (cur_state == kFrameSelStateActive) {
        frame_update = true;
      } else if (cur_state == kFrameSelStateIdle2Active) {
        if (params.frame_selection.cap_frame_num ==
              cam_reqmode_params_.frame_selection.target_frame_num) {
          cam_reqmode_params_.frame_selection.cur_state = kFrameSelStateActive;
          frame_update = true;
        }
      } else if (cur_state == kFrameSelStateActive2Idle) {
        if (params.frame_selection.cap_frame_num ==
            cam_reqmode_params_.frame_selection.target_frame_num) {
          cam_reqmode_params_.frame_selection.cur_state = kFrameSelStateIdle;
        } else {
          frame_update = true;
        }
      } else {
        // do nothing when FrameSelectionState is idle
      }
    } else {
      frame_update = true;
    }

    if (frame_update == true) {
      prev_frms = cam_reqmode_params_.frame_selection.total_selected_frames;
      cur_frms = params.frame_selection.total_selected_frames;

      if (prev_frms != cur_frms) {
        new_frms = (uint32_t)(cur_frms - prev_frms);
        cam_reqmode_params_.frame_selection.available_frames += new_frms;
        cam_reqmode_params_.frame_selection.total_selected_frames = cur_frms;
      }
    } else {
        cam_reqmode_params_.frame_selection.available_frames = 0;
        cam_reqmode_params_.frame_selection.total_selected_frames = 0;
    }

    cam_reqmode_lock_.unlock();

    QMMF_VERBOSE("%s:FrameSel:frames(prev=%d cur=%d) available=%u "
        "update(%d) cur_state(%d)",
        __func__, prev_frms, cur_frms,
        cam_reqmode_params_.frame_selection.available_frames,
        frame_update, cam_reqmode_params_.frame_selection.cur_state);
  }
}

void Camera3RequestHandler::RequestModeClear(CamOperationMode mode) {
  QMMF_VERBOSE("%s: clear mode (0x%x)", __func__, mode);

  cam_reqmode_lock_.lock();
  if (CAM_OPMODE_IS_FRAMESELECTION(mode)) {
    cam_reqmode_params_.frame_selection.available_frames = 0;
    cam_reqmode_params_.frame_selection.total_selected_frames = 0;
    cam_reqmode_params_.frame_selection.cur_state = kFrameSelStateIdle;
    cam_reqmode_params_.frame_selection.target_frame_num = 0;
  }

  if (CAM_OPMODE_IS_FASTSWTICH(mode)) {
    // nothing to do
  }

  cam_reqmode_lock_.unlock();
}

void Camera3RequestHandler::RequestStreamSubmitPreProcess(
    camera3_capture_request_t &request, CaptureRequest &nextRequest) {

  if (CAM_OPMODE_IS_FASTSWTICH(cam_opmode_)) {
    int32_t res = 0;
    uint32_t tag = 0;
    uint8_t val;
    FrameSelectionState cur_state;

    if (request.settings != nullptr) {
      nextRequest.metadata.unlock(request.settings);
      request_mdata_ = nextRequest.metadata;
    }

    const std::shared_ptr<VendorTagDescriptor> vTags =
        ::camera::VendorTagDescriptor::getGlobalVendorTagDescriptor();

    cam_reqmode_lock_.lock();

    // doSwitch tag is required only when frame_selection and hyperlapse is
    // configured. to help fastswitch in frameselection, we need to monitor
    // video streams in capture request, when video streams show up, doSwitch
    // is set, otherwise it's unset.
    if (CAM_OPMODE_IS_FRAMESELECTION(cam_opmode_)) {
      ::camera::CameraMetadata::getTagFromName(
          "com.qti.chi.fastswitchControl.doSwitch", vTags.get(), &tag);

      cur_state = cam_reqmode_params_.frame_selection.cur_state;

      if (tag > 0) {
        if ((cur_state == kFrameSelStateActive) ||
            (cur_state == kFrameSelStateIdle2Active)) {
          val = 1;
        } else {
          val = 0;
        }

        res = request_mdata_.update(tag, &val, 1);

        QMMF_VERBOSE("%s:cur_state(%d), val(%d)", __func__, cur_state, val);

        if (res != 0)
          QMMF_ERROR("%s: fast switch control tag update failed", __func__);
      } else {
        QMMF_ERROR("%s: fast switch control tag not found", __func__);
      }
    }

    cam_reqmode_lock_.unlock();
    request.settings = request_mdata_.getAndLock();
  }
}

bool Camera3RequestHandler::RequestStreamSubmitPostProcess(
    camera3_capture_request_t &request) {

  bool ret = false;

  if (CAM_OPMODE_IS_FASTSWTICH(cam_opmode_)) {
    request_mdata_.unlock(request.settings);
    ret = true;
  }

  return ret;
}

void Camera3RequestHandler::SetSystemEventOccurred(bool occurred) {
  system_event_occurred_ = occurred;
}

bool Camera3RequestHandler::RequestStreamGetProcess(
    RequestList::iterator it, CaptureRequest &realRequest, bool first) {

  bool ret = false;

  if (CAM_OPMODE_IS_FRAMESELECTION(cam_opmode_)) {
    int64_t frame_number = current_frame_number_;
    int32_t stream_num = it->streams.size();
    FrameSelectionState cur_state, next_state;
    bool found_video_stream = false;

    cam_reqmode_lock_.lock();

    cur_state = next_state = cam_reqmode_params_.frame_selection.cur_state;
    if (CAM_OPMODE_IS_FASTSWTICH(cam_opmode_) && (first == true)) {
      found_video_stream = false;
      for (int32_t i = 0; i < stream_num; i++) {
        if (it->streams[i]->IsPreviewStream() == false) {
          found_video_stream = true;
          break;
        }
      }

      switch (cur_state) {
      case kFrameSelStateIdle:
        if (found_video_stream)
          next_state = kFrameSelStateIdle2Active;
        break;
      case kFrameSelStateActive:
        if (!found_video_stream)
          next_state = kFrameSelStateActive2Idle;
        break;
      default:
        break;
      }
      if (next_state != cur_state) {
        cam_reqmode_params_.frame_selection.cur_state = next_state;
        cam_reqmode_params_.frame_selection.target_frame_num = frame_number;
      }

      QMMF_VERBOSE("%s:Framesel:stream_num (%d) found_video_stream (%d)"
          " cur_state (%d) next_state (%d) current frame (%ld)",
          __func__, stream_num, found_video_stream,
          cur_state, next_state, frame_number);
      cur_state = cam_reqmode_params_.frame_selection.cur_state;

    } else if (first == true) {
      cur_state = kFrameSelStateActive;
    } else {
      cur_state = kFrameSelStateIdle;
    }

    if (cur_state == kFrameSelStateActive ||
       cur_state == kFrameSelStateIdle2Active) {
      if (cam_reqmode_params_.frame_selection.available_frames <= 0) {
        realRequest.metadata = (*it).metadata;
        for (int32_t i = 0; i < stream_num; i++) {
          if (it->streams[i]->IsPreviewStream()) {
            realRequest.streams.push_back((*it).streams[i]);
          }
        }
        realRequest.resultExtras = (*it).resultExtras;
        realRequest.input = (*it).input;

        QMMF_VERBOSE("%s:FrameSel: 0 video frames, generate req for preview",
            __func__);
      } else {
        realRequest = *it;
        cam_reqmode_params_.frame_selection.available_frames--;

        QMMF_VERBOSE("%s:FrameSel: remain available video frames (%d),"
            "generate req for preview plus video",
            __func__, cam_reqmode_params_.frame_selection.available_frames);
      }
      ret = true;
    }

    cam_reqmode_lock_.unlock();
  }
  return ret;
}

}  // namespace cameraadaptor ends here

}  // namespace qmmf ends here
