/*
 * Copyright (c) 2016-2020 The Linux Foundation. All rights reserved.
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
#ifndef CAMERA3REQUESTHANDLER_H_
#define CAMERA3REQUESTHANDLER_H_

#include <pthread.h>
#ifdef HAVE_ANDROID_UTILS
#include <hardware/hardware.h>
#else
#include <hardware/camera_hardware.h>
#endif // HAVE_ANDROID_UTILS
#include <chrono>
#include <mutex>
#include <thread>

#include "qmmf-sdk/qmmf_camera_metadata.h"
#include "common/cameraadaptor/qmmf_camera3_types.h"
#include "common/cameraadaptor/qmmf_camera3_internal_types.h"
#include "common/utils/qmmf_thread.h"
#include "qmmf_camera3_smooth_zoom.h"

using namespace android;

namespace qmmf {

namespace cameraadaptor {

typedef enum EnumFrameSelectionState {
  kFrameSelStateIdle,
  kFrameSelStateIdle2Active,
  kFrameSelStateActive,
  kFrameSelStateActive2Idle,
} FrameSelectionState;

typedef struct {
  // frame selection
  struct frame_selection_params {
    int32_t total_selected_frames;
    uint32_t available_frames;
    FrameSelectionState cur_state;
    uint32_t target_frame_num;
  } frame_selection;
} CamReqModeParams;

typedef struct {
  struct frame_selection_input_params {
    int32_t total_selected_frames;
    uint32_t cap_frame_num;
  } frame_selection;
} CamReqModeInputParams;

typedef std::function<void(const char *fmt, va_list args)> SetError;
typedef std::function<int32_t(uint32_t frameNumber, int32_t numBuffers,
                              CaptureResultExtras resultExtras)> MarkRequest;

class Camera3Monitor;

class Camera3RequestHandler : public ThreadHelper {
 public:
  Camera3RequestHandler(Camera3Monitor &monitor);
  virtual ~Camera3RequestHandler();

  int32_t Initialize(camera3_device_t *device,
                     uint8_t buffer_api_version,
                     ErrorCallback error_cb,
                     MarkRequest mark_cb,
                     SetError set_error);

  int32_t SetRepeatingRequests(const RequestList &requests,
                               int64_t *lastFrameNumber = NULL);

  int32_t ClearRepeatingRequests(int64_t *lastFrameNumber = NULL);

  int32_t QueueRequestList(std::vector<CaptureRequest> &requests,
                           int64_t *lastFrameNumber = NULL);

  int32_t QueueReprocRequestList(std::vector<CaptureRequest> &requests,
                                 int64_t *lastFrameNumber = NULL);

  int32_t Clear(int64_t *lastFrameNumber = NULL);

  void TogglePause(bool pause);

  void TogglePause(bool pause, bool &pending_request);

  bool IsStreamActive(Camera3Stream &stream);
  void FinishConfiguration(uint32_t batch_size);

  void RequestExit() override;
  void RequestExitAndWait() override;

  void SetRequestMode(CamOperationMode mode);
  void UpdateRequestedStreams(CamReqModeInputParams &params);
  void RequestModeClear(CamOperationMode mode);
  void RequestStreamSubmitPreProcess(camera3_capture_request_t &request,
                                     CaptureRequest &nextRequest);
  bool RequestStreamSubmitPostProcess(camera3_capture_request_t &request);
  bool RequestStreamGetProcess(RequestList::iterator it,
                               CaptureRequest &realRequest, bool first);

  int32_t CreateInputBuffer(uint32_t frameNumber,
                            camera3_stream_buffer_t **input_buffer);

  void DeleteInputBuffer(uint32_t frameNumber);

  void SetSystemEventOccurred(bool occurred);

 protected:
  bool ThreadLoop() override;

 private:
  int32_t GetRequest(CaptureRequest &request);
  int32_t SubmitRequest(CaptureRequest &nextRequest,
                        camera3_stream_buffer_t *in_buf = nullptr);
  void ClearCaptureRequest(CaptureRequest &request);
  void HandleErrorRequest(camera3_capture_request_t &request,
                          CaptureRequest &nextRequest,
                          std::vector<camera3_stream_buffer_t> &outputBuffers);

  bool WaitOnPause();
  void Resume();

  void SignalError(const char *fmt, ...);

  /**Not allowed */
  Camera3RequestHandler(const Camera3RequestHandler &);
  Camera3RequestHandler &operator=(const Camera3RequestHandler &);

  static const int64_t WAIT_TIMEOUT  = 50e6;  // 50 ms
  static const int64_t CLEAR_TIMEOUT = 1000e6; // 1000 ms

  ErrorCallback error_cb_;
  MarkRequest mark_cb_;
  SetError set_error_;
  camera3_device_t *hal3_device_;

  pthread_mutex_t lock_;
  pthread_cond_t requests_signal_;
  pthread_cond_t current_request_signal_;
  RequestList requests_;
  RequestList streaming_requests_;

  CaptureRequest current_request_;
  CaptureRequest old_request_;

  bool configuration_update_;

  bool toggle_pause_state_;
  bool paused_state_;
  pthread_mutex_t pause_lock_;
  pthread_cond_t toggle_pause_signal_;
  pthread_cond_t pause_state_signal_;

  uint32_t current_frame_number_;
  uint32_t current_input_frame_number_;
  int64_t streaming_last_frame_number_;

  Camera3Monitor &monitor_;
  int32_t monitor_id_;
  uint32_t batch_size_;

  Camera3SmoothZoom smooth_zoom_;

  pthread_mutex_t input_buffer_map_lock_;

  // map of framenumber and input_buffer
  std::unordered_map <uint32_t, camera3_stream_buffer_t *> input_buffer_map_;

  static void ReprocLoop(Camera3RequestHandler *ctx);
  RequestList       reproc_requests_;
  std::thread       worker_;
  std::atomic<bool> run_worker_;
  std::mutex        worker_lock_;
  QCondition        worker_signal_;
  uint8_t           buffer_api_version_;

  // camera request mode params
  CamOperationMode  cam_opmode_;
  CamReqModeParams  cam_reqmode_params_;
  std::mutex        cam_reqmode_lock_;
  CameraMetadata    request_mdata_;

  // Flag to track system event occurrence
  std::atomic<bool> system_event_occurred_;
};

}  // namespace cameraadaptor ends here

}  // namespace qmmf ends here

#endif /* CAMERA3REQUESTHANDLER_H_ */
