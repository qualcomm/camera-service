/*
 * Copyright (c) 2016-2020, The Linux Foundation. All rights reserved.
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
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
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

#ifndef CAMERA3DEVICE_H_
#define CAMERA3DEVICE_H_

#include <pthread.h>
#ifdef HAVE_ANDROID_UTILS
#include <hardware/hardware.h>
#else
#include <hardware/camera_hardware.h>
#endif // HAVE_ANDROID_UTILS
#include <mutex>

#ifdef USE_LIBGBM
#include <gbm.h>
#include <gbm_priv.h>
#include <fcntl.h>
#endif // USE_LIBGBM

#include "qmmf-sdk/qmmf_vendor_tag_descriptor.h"
#include "qmmf-sdk/qmmf_camera_metadata.h"
#include "qmmf_camera3_types.h"
#include "qmmf_camera3_internal_types.h"
#include "qmmf_camera3_stream.h"
#include "qmmf_camera3_request_handler.h"
#include "qmmf_camera3_monitor.h"
#include "qmmf_camera3_prepare_handler.h"

extern "C" {
typedef void(callbacks_process_capture_result_t)(
    const struct camera3_callback_ops *, const camera3_capture_result_t *);

typedef void(callbacks_notify_t)(const struct camera3_callback_ops *,
                                 const camera3_notify_msg_t *);

typedef void(camera_device_status_change_t)(
    const struct camera_module_callbacks *, int camera_id, int new_status);

typedef void(torch_mode_status_change_t)(const struct camera_module_callbacks *,
                                         const char *camera_id, int new_status);
}

using namespace android;

namespace qmmf {

namespace cameraadaptor {

class Camera3DeviceClient : public camera3_callback_ops,
                            public camera_module_callbacks_t {
 public:
  Camera3DeviceClient(CameraClientCallbacks clientCb);
  virtual ~Camera3DeviceClient();

  int32_t Initialize();

  int32_t OpenCamera(uint32_t idx);
  int32_t BeginConfigure() { return 0; }

  int32_t EndConfigure(const CameraParameters& stream_config
                       = CameraParameters());

  int32_t UpdateCameraParams(const CameraParameters& camera_parameters
                       = CameraParameters());

  int32_t DeleteStream(int streamId, bool cache);
  int32_t CreateStream(const CameraStreamParameters &outputConfiguration);
  int32_t CreateInputStream(
      const CameraInputStreamParameters &inputConfiguration);

  int32_t CreateDefaultRequest(int templateId, CameraMetadata *request);
  int32_t SubmitRequest(Camera3Request request, bool streaming = false,
                        int64_t *lastFrameNumber = NULL);
  int32_t SubmitRequestList(std::vector<Camera3Request> requests,
                            bool streaming = false,
                            int64_t *lastFrameNumber = NULL);
  int32_t ReturnStreamBuffer(StreamBuffer buffer);
  int32_t CancelRequest(int requestId, int64_t *lastFrameNumber = NULL);

  int32_t GetCameraInfo(uint32_t idx, CameraMetadata *info);
  int32_t GetNumberOfCameras() { return number_of_cameras_; }
  const std::vector<int32_t> GetRequestIds(){ return current_request_ids_; }
  int32_t WaitUntilIdle();

  int32_t Flush(int64_t *lastFrameNumber = NULL);
  int32_t Prepare(int streamId);
  int32_t TearDown(int streamId);
  int32_t SetCameraSessionParam(const CameraMetadata &meta);

  static int32_t LoadHWModule(const char *moduleId,
                              const struct hw_module_t **pHmi);

 private:
  std::vector<int32_t> current_request_ids_;
  typedef enum State_t {
    STATE_ERROR,
    STATE_NOT_INITIALIZED,
    STATE_CLOSED,
    STATE_NOT_CONFIGURED,
    STATE_CONFIGURED,
    STATE_RUNNING
  } State;

  friend class Camera3PrepareHandler;
  friend class Camera3RequestHandler;
  friend class Camera3Monitor;
  friend class Camera3Gtest;
  friend class DualCamera3Gtest;

  int32_t AddRequestListLocked(const std::vector<CameraMetadata> &requests,
                               bool streaming, int64_t *lastFrameNumber = NULL);

  void HandleCaptureResult(const camera3_capture_result *result);
#if defined(CAMERA_HAL_API_VERSION) && (CAMERA_HAL_API_VERSION >= 0x0307)
  void ReturnStreamBuffers(uint32_t num_buffers, const camera3_stream_buffer_t* const* buffers);
  camera3_buffer_request_status_t RequestStreamBuffers(uint32_t num_buffer_reqs,
          const camera3_buffer_request_t *buffer_reqs, uint32_t *num_returned_buf_reqs,
          camera3_stream_buffer_ret_t *returned_buf_reqs);
#endif
  void UpdateCameraStatus(bool status);
  void Notify(const camera3_notify_msg *msg);
  void NotifyError(const camera3_error_msg_t &msg);
  void NotifyShutter(const camera3_shutter_msg_t &msg);
  void RemovePendingRequestLocked(uint32_t frameNumber);
  void ReturnOutputBuffers(const camera3_stream_buffer_t *outputBuffers,
                           size_t numBuffers, int64_t timestamp,
                           int64_t frame_number);
  void SendCaptureResult(CameraMetadata &pendingMetadata,
                         CaptureResultExtras &resultExtras,
                         CameraMetadata &collectedPartialResult,
                         uint32_t frameNumber);

  void NotifyStatus(bool idle);
  int32_t WaitUntilDrainedLocked();
  void InternalUpdateStatusLocked(State state);
  int32_t InternalPauseAndWaitLocked();
  int32_t InternalResumeLocked();
  int32_t WaitUntilStateThenRelock(bool active, int64_t timeout);

  int32_t CalculateBlobSize(int32_t width, int32_t height);

  int32_t ConfigureStreams(
        const CameraParameters& camera_parameters = CameraParameters(),
        bool force_reconfiguration = false);

  int32_t ConfigureStreamsLocked(bool force_reconfiguration = false);

  void SetErrorState(const char *fmt, ...);
  void SetErrorStateV(const char *fmt, va_list args);
  void SetErrorStateLocked(const char *fmt, ...);
  void SetErrorStateLockedV(const char *fmt, va_list args);

  static callbacks_process_capture_result_t processCaptureResult;
  static callbacks_notify_t notifyFromHal;
#if defined(CAMERA_HAL_API_VERSION) && (CAMERA_HAL_API_VERSION >= 0x0307)
  static camera3_buffer_request_status_t requestStreamBuffers(
            const struct camera3_callback_ops *cb, uint32_t num_buffer_reqs,
            const camera3_buffer_request_t *buffer_reqs, uint32_t *num_returned_buf_reqs,
            camera3_stream_buffer_ret_t *returned_buf_reqs);
  static void returnStreamBuffers(
            const struct camera3_callback_ops *cb, uint32_t num_buffers,
            const camera3_stream_buffer_t* const* buffers);
#endif
  static camera_device_status_change_t deviceStatusChange;
  static torch_mode_status_change_t torchModeStatusChange;

  int32_t MarkPendingRequest(uint32_t frameNumber, int32_t numBuffers,
                             CaptureResultExtras resultExtras);

  bool HandlePartialResult(uint32_t frameNumber, const CameraMetadata &partial,
                           const CaptureResultExtras &resultExtras);

  /**Not allowed */
  Camera3DeviceClient(const Camera3DeviceClient &);
  Camera3DeviceClient &operator=(const Camera3DeviceClient &);

  template <typename T>
  bool QueryPartialTag(const CameraMetadata &result, int32_t tag, T *value,
                       uint32_t frameNumber);
  template <typename T>
  bool UpdatePartialTag(CameraMetadata &result, int32_t tag, const T *value,
                        uint32_t frameNumber);

  int32_t GetRequestListLocked(const std::vector<CameraMetadata> &metadataList,
                               RequestList *requestList,
                               RequestList *requestListReproc);
  int32_t GenerateCaptureRequestLocked(const CameraMetadata &request,
                                       CaptureRequest &captureRequest);

  uint32_t GetOpMode();

  bool IsInputROIMode();

#ifdef VHDR_MODES_ENABLE
  int32_t GetSHDRMode();
#endif // VHDR_MODES_ENABLE

  pthread_mutex_t pending_requests_lock_;
  PendingRequestVector pending_requests_vector_;
  PendingRequestVector pending_error_requests_vector_;

  pthread_mutex_t lock_;
  CameraClientCallbacks client_cb_;

  std::string last_error_;
  uint32_t id_;

  State state_;
  bool flush_on_going_;

  std::map<int, Camera3Stream *> streams_;
  std::vector<Camera3Stream *> deleted_streams_;

  int next_stream_id_;
  bool reconfig_;

  CameraMetadata request_templates_[CAMERA3_TEMPLATE_COUNT];
  static const int32_t JPEG_BUFFER_SIZE_MIN =
      256 * 1024 + sizeof(camera3_jpeg_blob);

  camera_module_t *camera_module_;
  camera3_device_t *device_;
  uint32_t number_of_cameras_;
  CameraMetadata device_info_;
  IAllocDevice* alloc_device_interface_;

  std::vector<int32_t> repeating_requests_;
  int32_t next_request_id_;
  uint32_t frame_number_;
  uint32_t next_shutter_frame_number_;
  uint32_t next_shutter_input_frame_number_;
  uint32_t partial_result_count_;
  bool is_partial_result_supported_;
  uint32_t next_result_frame_number_;
  uint32_t next_result_input_frame_number_;
  vendor_tag_ops_t vendor_tag_ops_;
  Camera3Monitor monitor_;
  Camera3RequestHandler request_handler_;

  bool pause_state_notify_;
  std::vector<State> current_state_updates_;
  int state_listeners_;
  pthread_cond_t state_updated_;
  static const int64_t WAIT_FOR_SHUTDOWN = 10e9;  // 10 sec.
  static const int64_t WAIT_FOR_RUNNING = 1e9;    // 1 sec.

  bool is_hfr_supported_;
  bool is_raw_only_;
  bool hfr_mode_enabled_;
  uint32_t cam_feature_flags_;
  uint32_t fps_sensormode_index_;
  int32_t frame_rate_range_[2];
  Camera3PrepareHandler prepare_handler_;
  Camera3InputStream input_stream_;
  uint32_t batch_size_;
  static std::mutex vendor_tag_mutex_;
  static std::shared_ptr<VendorTagDescriptor> vendor_tag_desc_;
  static uint32_t client_count_;
  std::atomic<bool> is_camera_device_available_;

  CamOperationMode cam_opmode_;
  CameraMetadata session_metadata_;
};

}  // namespace cameraadaptor ends here

}  // namespace qmmf ends here

#endif /* CAMERA3DEVICE_H_ */
