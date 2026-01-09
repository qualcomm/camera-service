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
* Changes from Qualcomm Technologies, Inc. are provided under the following license:
* Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#pragma once

#include <list>
#include <mutex>

#include <qmmf-sdk/qmmf_recorder_params.h>
#include <qmmf-sdk/qmmf_recorder_extra_param_tags.h>

#include "common/utils/qmmf_condition.h"
#include "qmmf_camera3_device_intf.h"
#include "common/cameraadaptor/qmmf_camera3_device_client.h"
#include "recorder/src/service/qmmf_camera_interface.h"

namespace qmmf {

using namespace cameraadaptor;

#define MAX_SENSOR_FPS              480
#define STREAM_BUFFER_COUNT          12
#define REPROC_STREAM_BUFFER_COUNT    2
#define SNAPSHOT_STREAM_BUFFER_COUNT 30
#define EXTRA_DCVS_BUFFERS            2
#define EXTRA_HFR_BUFFERS             4
#define MAX_SNAPSHOT_BUFFER_COUNT    15

namespace recorder {

class CameraPort;
class IBufferConsumer;
class IBufferProducer;

struct AECData {
  uint8_t state;
  int64_t timestamp;

  AECData()
    : state(ANDROID_CONTROL_AE_STATE_INACTIVE),
      timestamp(-1) {}

  void Reset() {
    state = ANDROID_CONTROL_AE_STATE_INACTIVE;
    timestamp = -1;
  }
};

// This class deals with Camera3DeviceClient, and exposes simple Apis to create
// Different types of streams (preview, video, and snashot). this class has a
// Concept of ports, maintains vector of ports, each port is mapped one-to-one
// to camera device stream.
class CameraContext : public CameraInterface {
 public:
  CameraContext();

  ~CameraContext();

  status_t OpenCamera(const uint32_t camera_id, const float frame_rate,
                      const CameraExtraParam& extra_param,
                      const ResultCb &cb = nullptr,
                      const ErrorCb &errcb = nullptr,
                      const SystemCb &syscb = nullptr) override;

  status_t CloseCamera(const uint32_t camera_id) override;

  status_t WaitAecToConverge(const uint32_t timeout) override;

  status_t ConfigImageCapture(const uint32_t image_id,
                              const SnapshotParam& param,
                              const ImageExtraParam &xtraparam) override;

  status_t CaptureImage(const SnapshotType type, const uint32_t n_images,
                        const std::vector<CameraMetadata> &meta,
                        const StreamSnapshotCb& cb) override;

  status_t CancelCaptureImage(const uint32_t image_id,
                              const bool cache) override;

  status_t CreateStream(const StreamParam& param,
                        const VideoExtraParam& extra_param) override;

  status_t DeleteStream(const uint32_t track_id) override;

  status_t AddConsumer(const uint32_t& track_id,
                       std::shared_ptr<IBufferConsumer>& consumer) override;

  status_t RemoveConsumer(const uint32_t& track_id,
                          std::shared_ptr<IBufferConsumer>& consumer) override;

  status_t StartStream(const uint32_t track_id, bool cached) override;

  status_t StopStream(const uint32_t track_id, bool cached) override;

  status_t SetCameraParam(const CameraMetadata &meta) override;

  status_t GetCameraParam(CameraMetadata &meta) override;

  status_t SetCameraSessionParam(const CameraMetadata &meta) override;

  status_t GetDefaultCaptureParam(CameraMetadata &meta) override;

  status_t GetCamStaticInfo(std::vector<CameraMetadata> &meta) override;

  status_t GetCameraCharacteristics(CameraMetadata &meta) override;

  status_t ReturnAllImageCaptureBuffers() override;

  status_t ReturnImageCaptureBuffer(const uint32_t camera_id,
                                    const int32_t buffer_id) override;

  std::vector<int32_t>& GetSupportedFps() override;

#ifdef VHDR_MODES_ENABLE
  status_t SetVHDR(const int32_t mode) override;
#else
  status_t SetSHDR(const bool enable) override;
#endif // VHDR_MODES_ENABLE

  status_t ReturnStreamBuffer(StreamBuffer buffer);

  status_t CreateDeviceInputStream(CameraInputStreamParameters& params,
                                   int32_t* stream_id,
                                   bool cache = false);

  status_t CreateDeviceStream(CameraStreamParameters& params,
                              uint32_t frame_rate, int32_t* stream_id,
                              bool cache = false);

  int32_t SubmitRequest(Camera3Request request,
                        bool is_streaming,
                        int64_t *lastFrameNumber);

  status_t DeleteDeviceStream(int32_t stream_id, bool cache);

  void OnFrameAvailable(StreamBuffer& buffer);

  void NotifyBufferReturned(StreamBuffer& buffer);

  status_t CreateCaptureRequest(Camera3Request& request,
                        camera3_request_template_t template_type);

  CameraMetadata GetCameraStaticMeta();

  void SetReprocPortId( uint32_t port_id) { reproc_port_id_ = port_id; }

 private:

  struct HFRMode_t {
    uint32_t width;
    uint32_t height;
    uint32_t batch_size;
    uint32_t framerate;
  };

  friend class CameraPort;
  friend class ZslPort;

  void StoreBatchStreamId(std::shared_ptr<CameraPort>& port);

  void RestoreBatchStreamId(std::shared_ptr<CameraPort>& port);

  status_t GetBatchSize(const StreamParam& param, uint32_t& batch_size);

  void InitSupportedFPS();

  bool IsInputSupported();

  AECData GetAECData();

  status_t CreateSnapshotStream(uint32_t image_id,
                                CameraStreamParameters &stream_param,
                                bool cache = false);

  status_t DeleteSnapshotStream(uint32_t image_id, bool cache = false);

  status_t SetPerStreamFrameRate();

  status_t UpdateRequest(bool cached = false);

  status_t CancelRequest();

  status_t PauseActiveStreams(bool immedialtely = true);

  status_t ValidateResolution(const BufferFormat format, const uint32_t width,
                              const uint32_t height);

  status_t ResumeActiveStreams(bool state_only = false);

  status_t GetSnapshotStreamParams(const SnapshotParam &image_param,
                                   CameraStreamParameters &stream_param);

#ifdef USE_FPS_IDX
  uint32_t GetSensorModeIndex(uint32_t width, uint32_t height,
                              uint32_t framerate);
#endif

  void InitHFRModes();

  status_t StartZSL(const uint32_t image_id, const SnapshotParam& param,
                    const SnapshotZslSetup &zslparam);

  status_t StopZSL(const uint32_t image_id);

  status_t CaptureZSLImage(const SnapshotType type);

  void SendReprocRequest(StreamBuffer buffer);

#ifndef FLUSH_RESTART_NOTAVAILABLE
  status_t DisableFlushRestart(const bool& disable, CameraMetadata& meta);
#endif

  //Camera client callbacks.
  void SnapshotCaptureCallback(StreamBuffer &buffer);

  void CameraErrorCb(CameraErrorCode error_code, const CaptureResultExtras &);

  void CameraIdleCb();

  void CameraShutterCb(const CaptureResultExtras &, int64_t time_stamp);

  void CameraPreparedCb(int32_t stream_id);

  void CameraResultCb(const CaptureResult &result);

  void CameraSystemCb(uint32_t errcode);

  uint32_t GetROICountTag () { return multi_roi_count_tag_; }

  uint32_t GetROIInfoTag () { return multi_roi_info_tag_; }

  int32_t GetROICount () { return multi_roi_count_; }

  std::vector<int32_t> GetROIInfo () { return multi_roi_info_; }

  bool IsReproc () { return enable_reproc_; }

  std::function<void(StreamBuffer)> GetStreamCb(const SnapshotParam& param);

  std::shared_ptr<CameraPort> GetPort(const uint32_t& track_id);

  template <typename T>
  bool QueryPartialTag(const CameraMetadata &result, int32_t tag, T *value,
                       uint32_t frame_number);

  template <typename T>
  bool UpdatePartialTag(CameraMetadata &result, int32_t tag, const T *value,
                        uint32_t frame_number);

  void HandleFinalResult(const CaptureResult &result);

  bool IsRawOnly(const int32_t format);

  std::vector<int32_t> GetReprocOutputStreamIds() { return reproc_out_stream_ids_; };


  bool IsStreamParamsChanged(const CameraStreamParameters& stream_param);

  bool IsNeedReconfigSnapshotStream();

  std::shared_ptr<Camera3DeviceClient>  camera_device_;
  CameraClientCallbacks                 camera_callbacks_;
  uint32_t                              camera_id_;
  std::mutex                            device_access_lock_;
  CameraMetadata                        static_meta_;

  std::map<uint32_t, bool> stream_prepared_;
  QCondition               prepare_done_;
  std::mutex               prepare_lock_;

  // Global Capture request.
  int32_t                  streaming_request_id_;
  int32_t                  capture_request_id_;

  // Map of stream id and it's last request frame number submitted to HAL.
  std::map<int32_t, int64_t> last_frame_number_map_;

  // Stream ids that have been removed from capture requests.
  std::set<int32_t>        removed_stream_ids_;

  int64_t                  last_frame_number_;

  Camera3Request           reproc_request_;

  //Non zsl capture request.
  Camera3Request           snapshot_request_;
  StreamSnapshotCb         client_snapshot_cb_;
  uint32_t                 capture_cnt_;
  std::mutex               capture_lock_;
  bool                     cancel_capture_ = false;

  ResultCb                 result_cb_;
  ErrorCb                  error_cb_;
  SystemCb                 system_cb_;
  std::vector<int32_t>     supported_fps_;
  uint32_t                 zsl_port_id_;
  uint32_t                 reproc_port_id_;
  std::vector<int32_t>     reproc_out_stream_ids_;

  // Map of <consumer id and CameraPort>
  std::map<uint32_t, std::shared_ptr<CameraPort> > active_ports_;

  // Maps of buffer Id and Buffer.
  std::map<uint32_t, StreamBuffer> snapshot_buffer_list_;
  std::mutex                       snapshot_buffer_lock_;

  static float             kConstrainedModeThreshold;
  static float             kHFRBatchModeThreshold;
  bool                     hfr_supported_;
  std::vector<HFRMode_t>   hfr_batch_modes_list_;
  std::vector<Camera3Request> streaming_active_requests_;

  std::map<uint32_t, int32_t> snapshot_buffer_stream_list_;
  int32_t                  batch_stream_id_;

  std::mutex               pending_frames_lock_;
  QCondition               pending_frames_;

  AECData                  aec_;
  std::mutex               aec_lock_;
  QCondition               aec_state_updated_;

  static const uint32_t    kWaitAecTimeout;
  static const uint32_t    kWaitPendingFramesTimeout = 1500000000; // 1500 ms.

  int32_t                  partial_result_count_;
  std::mutex               partial_result_lock_;

  // snapshot configuration

  // <stream id, image id>
  std::map<uint32_t, uint32_t>  stream_image_map_;
  std::mutex                    stream_image_lock_;
  ImageMode                     snapshot_mode_;
  uint32_t                      snapshot_quality_;
  bool                          port_paused_;
  std::set<int32_t>             stopped_stream_ids_;
  CameraParameters              camera_parameters_;
  bool                          is_partial_metadata_enabled_;
  bool                          pcr_frc_enabled_;
  bool                          continuous_mode_is_on_;
  bool                          is_camera_dead_;
  bool                          pending_cached_stream_;
  bool                          enable_reproc_;
  int32_t                       multi_roi_count_ = 0;
  std::vector<int32_t>          multi_roi_info_ = {};
  uint32_t                      multi_roi_count_tag_ = 0;
  uint32_t                      multi_roi_info_tag_ = 0;

  std::vector<Camera3Request>   last_submitted_streaming_requests_;
  bool                          video_streams_active_;
};

enum class CameraPortType {
  kVideo,
  kZSL,
};

enum class PortState {
  PORT_CREATED,
  PORT_READYTOSTART,
  PORT_STARTED,
  PORT_READYTOSTOP,
  PORT_STOPPED,
  PORT_PAUSED,
};

struct ZSLEntry {
  StreamBuffer    buffer;
  CameraMetadata  result;
  int64_t         timestamp;
};

// CameraPort is one to one mapped to Camera device stream. It takes buffers
// from camera stream and passes to its consumers, for optimization reason
// single port can serve multiple consumers if their characterstics are exactly
// same.
class CameraPort {
 public:
  CameraPort(const StreamParam& param, const VideoExtraParam& extraparam,
             const CameraParameters camera_parameters, CameraPortType port_type,
             CameraContext *context);

  virtual ~CameraPort();

  virtual status_t Init();

  virtual status_t DeInit();

  status_t Start(bool cached = false);

  status_t Stop(bool cached = false);

  status_t Pause();

  status_t Resume();

  // Apis to Add/Remove consumer at run time.
  status_t AddConsumer(std::shared_ptr<IBufferConsumer>& consumer);

  status_t RemoveConsumer(std::shared_ptr<IBufferConsumer>& consumer);

  void NotifyBufferReturned(const StreamBuffer& buffer);

  int32_t GetNumConsumers();

  bool IsReadyToStart();

  PortState& getPortState();

  float GetPortFramerate() { return params_.framerate; }

  size_t GetPortBatchSize() { return camera_parameters_.batch_size; }

  int32_t GetCameraStreamId() { return camera_stream_id_; }

  uint32_t GetPortId() { return port_id_; }

  CameraPortType GetPortType() { return port_type_; }

  ReprocEntry& GetInputBuffer() { return reproc_input_buffer_; }

  StreamBuffer GetInputStreamBuffer () { return reproc_input_buffer_.buffer; }

  void HandleReprocCaptureResult(const CaptureResult *result, StreamBuffer *buffer);

  int32_t GetInputStreamId() { return reproc_input_stream_id_; }

  void ReturnReprocInputBuffer(StreamBuffer &buffer);

  bool IsPreviewStream() {
    return (cam_stream_params_.allocFlags.flags & IMemAllocUsage::kHwComposer);
  }

 protected:
  CameraPortType         port_type_;
  CameraContext*         context_;
  int32_t                camera_stream_id_;
  PortState              port_state_;
  StreamParam            params_;

 private:

  bool IsConsumerConnected(std::shared_ptr<IBufferConsumer>& consumer);

  void StreamCallback(StreamBuffer buffer);

  uint32_t GetExtraBufferCount();

  void ReprocCaptureCallback(StreamBuffer buffer);

  void GetReprocInputBuffer(StreamBuffer &buffer);

  std::shared_ptr<IBufferProducer>         buffer_producer_impl_;
  CameraStreamParameters      cam_stream_params_;
  CameraInputStreamParameters input_stream_params_;
  bool                        ready_to_start_;
  uint32_t                    port_id_;

  // Indicates whether and for which frame the AE has converged after start.
  bool                   aec_converged_;
  int64_t                aec_timestamp_;

  std::map<uintptr_t, std::shared_ptr<IBufferConsumer> >consumers_;

  std::shared_ptr<IBufferConsumer>    consumer_;

  std::mutex             consumer_lock_;
  std::mutex             stop_lock_;
  std::mutex             aec_lock_;

  CameraParameters       camera_parameters_;

  int32_t                reproc_input_stream_id_;
  std::mutex             reproc_queue_lock_;
  std::list<ReprocEntry> reproc_queue_;
  ReprocEntry            reproc_input_buffer_;
  bool                   cameraport_enable_reproc_;
};

class ZslPort : public CameraPort {

 public:
  ZslPort(const StreamParam& param, const CameraParameters camera_parameters,
          CameraPortType port_type, CameraContext *context,
          uint32_t zsl_queue_depth);

  ~ZslPort();

  status_t Init() override;

  status_t DeInit() override;

  status_t PauseAndFlushZSLQueue();

  void ResumeZSL();

  bool IsRunning();

  status_t PickZSLBuffer();

  ZSLEntry& GetInputBuffer() { return zsl_input_buffer_; }

  void HandleZSLCaptureResult(const CaptureResult &result);

  int32_t GetInputStreamId() { return input_stream_id_; }

  status_t ValidateCaptureParams(uint32_t width, uint32_t height,
                                 BufferFormat format);

  void ReturnZSLInputBuffer(StreamBuffer &buffer);

 private:

  status_t SetUpZSL();

  void ZSLCaptureCallback(StreamBuffer buffer);

  void GetZSLInputBuffer(StreamBuffer &buffer);

  int32_t         input_stream_id_ = -1;
  std::mutex      zsl_queue_lock_;
  std::vector<ZSLEntry>  zsl_queue_;
  ZSLEntry        zsl_input_buffer_ = {};
  bool            zsl_running_ = false;
  uint32_t        zsl_queue_depth_ = 0;
};

}; //namespace recorder

}; //namespace qmmf
