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
* Copyright (c) 2021-2024 Qualcomm Innovation Center, Inc. All rights reserved.
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

//! @file qmmf_recorder.h

#pragma once

#include <cstddef>
#include <cstdlib>
#include <vector>
#include <string>
#include <unordered_set>

#include "qmmf-sdk/qmmf_camera_metadata.h"
#include "qmmf-sdk/qmmf_recorder_params.h"
#include "qmmf-sdk/qmmf_recorder_extra_param.h"
#include "qmmf-sdk/qmmf_offline_jpeg_params.h"

namespace qmmf {
namespace recorder {

class RecorderClient;

/*! @brief Client interface for video recording and image capture.

    To start an video recording
    clients first should create a track.

    Multiple tracks can be created and run at the same time.
    Recorder provides elementary stream callback to clients and clients
    manage the muxer. A/V muxer is not part of the
    Recorder class and hence clients
    need to use an external muxer to save the elementary streams

    For image capture, Recorder API provides single, burst and
    timed image capture

    Some of the APIs in the class are sync and others async.
    The API documentation explicitly says whether the API is async.

    Callbacks used for async communication from recorder class are expected
    to be lambda functions. The client is expected to capture the context
    while setting the callback.
*/
class Recorder {
 public:
  Recorder();

  ~Recorder();

  /// Connect to recoder service and set callback.
  status_t Connect(const RecorderCb &cb);

  /// Disconnect from recorder service.
  status_t Disconnect();

  /// @brief Initializes camera and prepares camera for image capture and/or
  /// video record.
  ///
  /// This API must be called before calling create video track or
  /// calling CaptureImage API. The result callback is optional. In case
  /// it is set by client, notifications will get triggered on each
  /// incoming streaming request along with the camera results.
  status_t StartCamera(const uint32_t camera_id,
                       const float framerate,
                       const CameraExtraParam& extra_param = {},
                       const CameraResultCb &cb = nullptr);

  /// @brief Stops camera. This API should be called to free up all resources
  /// associated with camera.
  ///
  /// StopCamera cannot be called when there is an
  /// active video record track or image capture track.
  status_t StopCamera(const uint32_t camera_id);

  /// @brief Creates an video track with additional configurations set in
  /// extra_param container.
  /// User must specify the unique track_id.
  ///
  /// params must specify the video track characteristics such as codec,
  /// bitrate etc.
  /// extra_param may contain additional parameters for setting crop, source
  /// surface dimensions, multi-camera mode, etc.
  /// cb is used by the recorder to inform clients about track data
  /// availability and track specific async errors
  status_t CreateVideoTrack(const uint32_t track_id,
                            const VideoTrackParam &param,
                            const VideoExtraParam& xtraparam,
                            const TrackCb &cb);

  /// @brief Returns the track buffer back to recoder
  ///
  /// Track output buffers are passed to clietns through track callbacks.
  /// Clients returns these buffers through this API
  status_t ReturnTrackBuffer(const uint32_t track_id,
                             ::std::vector<BufferDescriptor> &buffers);

  /// @brief Changes runtime video track params such as video encoder
  /// bitrate, framerate, IDR insertion etc
  ///
  /// The type of *param* data depends on the enum value of *type*
  /// param_size should be set to sizeof the param data structure
  status_t SetVideoTrackParam(const uint32_t track_id,
                              VideoParam type, const void *param,
                              size_t size);

  /// @brief Deletes video track. The track can be deleted only when
  /// is in stopped state
  status_t DeleteVideoTrack(const uint32_t track_id);

  /// @brief Starts track corresponding to id.
  ///
  /// This is an async API. When start is completed, track specific event cb
  /// is called by recorder
  status_t StartVideoTracks(const std::unordered_set<uint32_t>& track_ids);

  /// @brief Stops track corresponding to id.
  ///
  /// This is an async API. When stop is completed, tracks specific
  /// event cb is called by recorder
  status_t StopVideoTracks(const std::unordered_set<uint32_t>& track_ids);

  /// @brief Capture burst or single images from a camera
  ///
  /// This is an async API. When the image is ready, data callback specified
  /// through cb is called which enables clients to process the image data.
  /// If multiple images are captured, data cb is called for every image.
  /// If CancelCaptureImage() is called before all images are returned to
  /// clients, the Recorder stops the burst or timed image capture and returns
  /// an event indicating CANCEL is complete
  ///
  /// @param camera_id: ID of camera
  /// @param type: The type of snapshot capture request
  /// @param n_images: Number of images to be captured
  /// @param meta: Optional camera meta parameter for each image to be captured
  ///        If this vector is empty default parameters are used for for image
  ///        capture.
  /// @param cb: Callbacks for data and error notifications
  status_t CaptureImage(const uint32_t camera_id,
                        const SnapshotType type,
                        const uint32_t n_images,
                        const std::vector<CameraMetadata> &meta,
                        const ImageCaptureCb &cb);

  /// @brief Enables clients to configure image capture parameters.
  ///
  /// This is an optional API to be used only for advanced post processing
  /// features.
  ///
  /// @param camera_id: ID of camera
  /// @param image_id: ID of snapshot stream
  /// @param param: Details dimensions, format of the image
  /// @param config: Additional parameters for setting crop, reprocess
  ///        plugins, multi-camera mode, etc.
  status_t ConfigImageCapture(const uint32_t camera_id,
                              const uint32_t image_id,
                              const ImageParam &param,
                              const ImageExtraParam &xtraparam);

  /// @brief Cancels an ongoing image capture
  ///
  /// CaptureImage is a async API. Clients can call CancelCaptureImage anytime
  /// after CaptureImage to cancel pending image captures
  status_t CancelCaptureImage(const uint32_t camera_id,
                              const uint32_t image_id,
                              const bool cache = false);

  /// @brief Returns image buffer back to recoder
  ///
  /// Image output buffers are passed to clients through a callback.
  /// Clients returns these buffers through this API
  status_t ReturnImageCaptureBuffer(const uint32_t camera_id,
                                    const BufferDescriptor &buffer);

  /// @brief Sets camera parameters
  ///
  /// Camera parameters are controlled through CameraMetadata class
  /// defined camera/CameraMetadata.h. Typical usage would be to get
  /// meta param through GetCameraParam, then update it using update
  /// function of CameraMetadata and then set this through SetCameraParam
  status_t SetCameraParam(const uint32_t camera_id,
                          const CameraMetadata &meta);

  /// Compelementary API of SetCameraParam. Clients generally calls
  /// GetCameraParam to get default camerametadata params
  status_t GetCameraParam(const uint32_t camera_id,
                          CameraMetadata &meta);

  /// @brief Sets camera session parameters
  ///
  /// Camera session parameters are controlled through CameraMetadata class
  /// defined camera/CameraMetadata.h. Typical usage would be to create
  /// meta param, then update it using update function of CameraMetadata and
  /// then set this through SetCameraSessionParam
  status_t SetCameraSessionParam(const uint32_t camera_id,
                                 const CameraMetadata &meta);

  /// Set Camera SHDR mode
#ifdef VHDR_MODES_ENABLE
  status_t SetVHDR(const uint32_t camera_id,
                   const int32_t mode);
#else
  status_t SetSHDR(const uint32_t camera_id,
                   const bool enable);
#endif // VHDR_MODES_ENABLE

  /// Complementary API of CaptureImage. Clients generally calls
  /// GetDefaultCaptureParam to get default capture params. this API is
  /// Different then GetCameraParam, it gives default params for image capture
  /// While GetCameraParam gives default params for video record. This API is
  /// applicable only for non-zsl capture.
  status_t GetDefaultCaptureParam(const uint32_t camera_id,
                                  CameraMetadata &meta);

  /// Clients generally calls GetCameraCharacteristics to get
  /// static camerametadata params.
  status_t GetCameraCharacteristics(const uint32_t camera_id,
                                    CameraMetadata &meta);

  /// @brief Create Offline JPEG
  ///
  /// When creation is ready, clients can submit requests
  /// to Offline JPEG via EncodeOfflineJPEG() API.
  ///
  /// @param params: Input and output buffers details: dimensions and format.
  /// @param cb: Client callback for data and error notifications
  status_t CreateOfflineJPEG(const OfflineJpegCreateParams &params,
                             const OfflineJpegCb &cb);

  /// @brief Submit request to Offline JPEG
  ///
  /// This is an async API. When encoding is ready, client callback specified
  /// through CreateOfflineJPEG() will be called.
  ///
  /// @param params: Input and output buffers details:
  /// dimensions, format and fds, metadata and request id.
  /// Client is the owner of the buffers.
  status_t EncodeOfflineJPEG(const OfflineJpegProcessParams &params);

  /// @brief Destroy Offline JPEG
  ///
  /// This API destroys offline JPEG, created with CreateOfflineJPEG()
  status_t DestroyOfflineJPEG();

 private:
  RecorderClient* recorder_client_;

};

};  // namespace recorder

};  // namespace qmmf
