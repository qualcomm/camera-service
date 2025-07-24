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

/*! @file qmmf_camera_source.h
*/

#pragma once

#include <list>
#include <memory>
#include <mutex>

#include "qmmf-sdk/qmmf_camera_metadata.h"
#include <qmmf-sdk/qmmf_recorder_extra_param_tags.h>

#include "common/utils/qmmf_condition.h"
#include "recorder/src/service/qmmf_recorder_common.h"
#include "recorder/src/service/qmmf_camera_interface.h"
#ifndef CAMERA_HAL1_SUPPORT
#include "common/cameraadaptor/qmmf_camera3_device_client.h"
#include "recorder/src/service/qmmf_camera_context.h"
#else
#include "recorder/src/service/qmmf_camera_context_hal1.h"
#endif
#include "recorder/src/service/qmmf_camera_rescaler.h"
#include "recorder/src/service/qmmf_camera_frc.h"

namespace qmmf {

#ifndef CAMERA_HAL1_SUPPORT
using namespace cameraadaptor;
#endif
#ifdef HAVE_BINDER
using namespace android;
#endif

namespace recorder {

class TrackSource;


/// @brief CameraSource class
/// Operates the Camera
class CameraSource {
 public:

  /// Create CameraSource Instance
  static CameraSource* CreateCameraSource();

  /// CameraSource Destructor
  ~CameraSource();

  /// Open Camera.
  status_t StartCamera(const uint32_t camera_id,
                       const float framerate,
                       const CameraExtraParam& extra_param,
                       const ResultCb &cb = nullptr,
                       const ErrorCb &errcb = nullptr,
                       const SystemCb &syscb = nullptr);

  /// Close Camera.
  status_t StopCamera(const uint32_t camera_id);

  /// Image Capture
  status_t CaptureImage(const uint32_t camera_id,
                        const SnapshotType type,
                        const uint32_t n_images,
                        const std::vector<CameraMetadata> &meta,
                        const SnapshotCb &cb);

  /// Configure Image Capture
  status_t ConfigImageCapture(const uint32_t camera_id,
                              const uint32_t image_id,
                              const ImageParam &param,
                              const ImageExtraParam &xtraparam);

  /// Cancel Image Capture
  status_t CancelCaptureImage(const uint32_t camera_id,
                              const uint32_t image_id, const bool cache);

  /// Return All Image Capture buffers
  status_t ReturnAllImageCaptureBuffers(const uint32_t camera_id);

  /// Return Image Capture buffer
  status_t ReturnImageCaptureBuffer(const uint32_t camera_id,
                           const int32_t buffer_id);

  /// Create Track Source
  status_t CreateTrackSource(const uint32_t track_id,
                             const VideoTrackParam& params,
                             const VideoExtraParam& extraparams,
                             const BnBufferCallback &cb);
  /// Delete Track Source
  status_t DeleteTrackSource(const uint32_t track_id);

  /// Start Track Source
  status_t StartTrackSources(const std::unordered_set<uint32_t>& track_ids);

  /// Stop Track Source
  status_t StopTrackSources(const std::unordered_set<uint32_t>& track_ids);

  /// Force return all pending buffers to producer
  status_t FlushTrackSource(const uint32_t track_id);

  /// Return Track buffer
  status_t ReturnTrackBuffer(const uint32_t track_id,
                             std::vector<BnBuffer> &buffers);

  /// Set Camera configuration to Camera Interface
  status_t SetCameraParam(const uint32_t camera_id, const CameraMetadata &meta);

  /// Get Camera configuration to Camera Interface
  status_t GetCameraParam(const uint32_t camera_id, CameraMetadata &meta);

  /// Set Camera Session configuration to Camera Interface
  status_t SetCameraSessionParam(const uint32_t camera_id, const CameraMetadata &meta);

  /// Set Camera SHDR mode
#ifdef VHDR_MODES_ENABLE
  status_t SetVHDR(const uint32_t camera_id, const int32_t mode);
#else
  status_t SetSHDR(const uint32_t camera_id, const bool enable);
#endif // VHDR_MODES_ENABLE
  /// Return default settings for Image Capture
  status_t GetDefaultCaptureParam(const uint32_t camera_id,
                                  CameraMetadata &meta);

  /// Return static metadata
  status_t GetCameraCharacteristics(const uint32_t camera_id,
                                    CameraMetadata &meta);

  /// Return static metadata of all the camera's connected without opening the camera
  status_t GetCamStaticInfo(std::vector<CameraMetadata> &meta);

  /// UpdateTrackFrameRate
  status_t UpdateTrackFrameRate(const uint32_t track_id,
                                const float framerate);

  /// Enable repeating of frames to ensure target frame rate
  status_t EnableFrameRepeat(const uint32_t track_id,
                             const bool enable);

  /// Return instance for track source for given ID
  const ::std::shared_ptr<TrackSource>& GetTrackSource(uint32_t track_id);

  /// Get Rescaller configuration parameters
  ResizerCrop GetRescalerConfig(const VideoExtraParam& extraparams);

  /// Get calculated JPEG size
  static uint32_t GetJpegSize(uint8_t *blobBuffer, uint32_t width);

  /// @cond PRIVATE
 private:
  bool IsTrackIdValid(const uint32_t track_id);
  void SnapshotCallback(uint32_t image_id, uint32_t count,
                        StreamBuffer& buffer);

  bool ValidateSlaveTrackParam(
    const VideoTrackParam& slave_params,
    const VideoTrackParam& master_params);

  bool CheckLinkedStream(
    const VideoTrackParam& slave_params,
    const VideoTrackParam& master_params);

  int32_t GetSourceTrackId(const VideoExtraParam& extra_param);

  status_t ParseThumb(uint8_t* vaddr, uint32_t size, StreamBuffer& buffer);

  std::list<std::shared_ptr<CameraInterface>> preloaded_cameras_;

  // Map of camera id and CameraInterface.
  std::map<uint32_t, std::shared_ptr<CameraInterface>> active_cameras_;
  std::mutex active_cameras_lock_;

  // Map of track id and TrackSources.
  std::map<uint32_t, std::shared_ptr<TrackSource>> track_sources_;
  std::mutex    track_source_lock_;

  SnapshotCb client_snapshot_cb_;

  std::map<uint32_t, CameraExtraParam>  start_cam_param_;
  std::mutex                            start_cam_param_lock_;

  bool frame_rate_control_;

  // Not allowed
  CameraSource();
  CameraSource(const CameraSource&);
  CameraSource& operator=(const CameraSource&);
  static CameraSource* instance_;
  std::map<int32_t, std::shared_ptr<CameraRescaler> > rescalers_;
  /// @endcond

};

/// @brief This class behaves as producer and consumer both, at one end
/// it takes YUV buffers from camera stream and another end it provides buffers
/// and manages buffer circulation, frame skip etc.
class TrackSource {
 public:

  /// TrackSource Constructor
  TrackSource(const uint32_t id, const std::shared_ptr<CameraInterface>& camera,
              const VideoTrackParam& params, const VideoExtraParam& extraparams,
              const bool frame_rate_cotrol, const BnBufferCallback& cb);

  /// TrackSource Destructor
  ~TrackSource();

  /// Create stream and initialize additional processing
  status_t Init();

  /// Destroy stream and de-initialize additional processing
  status_t DeInit();

  /// Link track source with consumer and start additional processing
  status_t StartTrack(bool cached);

  /// Force return all pending buffers to producer
  status_t Flush();

  /// Unlink track source with consumer and stops additional processing
  status_t StopTrack(bool cached);

  // Global track specific params can be query from TrackSource during its life
  // cycle.
  /// Get Track parameters
  VideoTrackParam& GetParams() { return params_; }

  // This method to handle incoming buffers from producer, producer can be
  // anyone, Camera context's port or rescaler.
  /// Handle incoming buffers from producer
  void OnFrameAvailable(StreamBuffer& buffer);

  /// Return track buffers to producer
  status_t ReturnTrackBuffer(std::vector<BnBuffer>& buffers);

  /// Return true if current state is different then running
  bool IsStop();

  /// Change frame rate
  void UpdateFrameRate(const float framerate);

  /// Enable frame repeat
  void EnableFrameRepeat(const bool enable);

  /// Callback to handle returned buffers
  void NotifyBufferReturned(StreamBuffer& buffer);

  /// Sets source track and enable track duplication
  status_t InitCopy(std::shared_ptr<TrackSource> track_source,
                    const std::shared_ptr<CameraRescaler>& rescaler);

  /// Return if the source of this track is another track
  bool IsSlaveTrack() {return slave_track_source_; };

  /// Add track source Consumer
  status_t AddConsumer(const std::shared_ptr<IBufferConsumer>& consumer);

  /// Remove track source Consumer
  status_t RemoveConsumer(std::shared_ptr<IBufferConsumer>& consumer);

 private:

  // Method to provide consumer interface, it would be used by producer to
  // post buffers.
  std::shared_ptr<IBufferConsumer>& GetConsumer() { return buffer_consumer_; }

  void ReturnBufferToProducer(StreamBuffer& buffer);
  void CalculateFPS(StreamBuffer& buffer);

  uint32_t                 id_;
  VideoTrackParam          params_;
  VideoExtraParam          extraparams_;
  BnBufferCallback         buffer_cb_;

  bool                     is_stop_;
  std::mutex               stop_lock_;

  std::mutex               lock_;

  // will be used till we make stop api as async.
  bool                     is_idle_;
  std::mutex               idle_lock_;
  QCondition               wait_for_idle_;

  // Maps of Unique buffer Id and Buffer.
  std::map<uint32_t, StreamBuffer> buffer_list_;
  std::mutex buffer_list_lock_;

  std::map<IBufferHandle, uint32_t >  buffer_map_;
  std::mutex frame_lock_;

  std::shared_ptr<CameraInterface>     camera_;
  std::shared_ptr<FrameRateController> fsc_;
  std::shared_ptr<FrameRateController> frc_;
  std::shared_ptr<CameraRescaler>      rescaler_;

  std::shared_ptr<IBufferConsumer>    buffer_consumer_;
  std::shared_ptr<IBufferProducer>    buffer_producer_;
  std::mutex             consumer_lock_;
  uint32_t               num_consumers_;

  std::shared_ptr<TrackSource> master_track_;
  bool  slave_track_source_;

  //FPS calculation related variables
  int32_t                  input_frame_count_;
  int64_t                  measurement_interval_;
  int64_t                  previous_input_ts_;
  int64_t                  input_frame_interval_;
};

}; //namespace recorder

}; //namespace qmmf
