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
 * Changes from Qualcomm Innovation Center are provided under the following license:
 *
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
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

/*! @file qmmf_recorder_impl.h
*/

#pragma once

#include <algorithm>
#include <atomic>
#include <vector>
#include <mutex>
#include <tuple>
#include <set>

#include <camera/CameraMetadata.h>

#include "recorder/src/client/qmmf_recorder_service_intf.h"
#include "recorder/src/service/qmmf_recorder_common.h"
#include "recorder/src/service/qmmf_camera_source.h"
#include "recorder/src/service/qmmf_remote_cb.h"

#ifdef ENABLE_OFFLINE_JPEG
#include "recorder/src/service/qmmf_offline_jpegenc_impl.h"
#endif

/// @namespace qmmf
namespace qmmf {

/// @namespace recorder
namespace recorder {


using namespace android;

/// @brief RecorderImpl interface
///
/// Handles all Server operations
class RecorderImpl {
 public:

  /// Create Recorder Instance
  static RecorderImpl* CreateRecorder();

  /// RecorderImpl Destructor
  ~RecorderImpl();

  /// Create CameraSource instances
  status_t Init(const RemoteCallbackHandle& remote_cb_handler);

  /// Destroys all instances created in Init
  status_t DeInit();

  /// Connect and register client to the session
  status_t RegisterClient(const uint32_t client_id);

  /// Cleans up and closes the cameras owned by previous "dead" client.
  /// Register new client.
  status_t DeRegisterClient(const uint32_t client_id,
                            bool force_cleanup = false);

  /// Start(open) the Camera
  status_t StartCamera(const uint32_t client_id, const uint32_t camera_id,
                       const float framerate,
                       const CameraExtraParam& extra_param,
                       bool enable_result_cb = false);

  /// Stop(close) the Camera
  status_t StopCamera(const uint32_t client_id, const uint32_t camera_id);

  /// Create session
  status_t CreateSession(const uint32_t client_id, uint32_t *session_id);

  /// Delete session
  status_t DeleteSession(const uint32_t client_id, const uint32_t session_id);

  /// Start the session
  status_t StartSession(const uint32_t client_id, const uint32_t session_id);

  /// Stop the session
  status_t StopSession(const uint32_t client_id, const uint32_t session_id,
                       bool do_flush, bool force_cleanup = false);

  /// Pause the session
  status_t PauseSession(const uint32_t client_id, const uint32_t session_id);

  /// Resume the session
  status_t ResumeSession(const uint32_t client_id, const uint32_t session_id);

  /// Create Video Track and associates it to the session with
  /// additional configure parameters
  status_t CreateVideoTrack(const uint32_t client_id,
                            const uint32_t session_id,
                            const uint32_t track_id,
                            const VideoTrackParam& param,
                            const VideoExtraParam& xtraparam);

  /// Delete Video Track from the session.
  status_t DeleteVideoTrack(const uint32_t client_id,
                            const uint32_t session_id,
                            const uint32_t track_id);

  /// Return Track buffers to Camera Source.
  status_t ReturnTrackBuffer(const uint32_t client_id,
                             const uint32_t session_id,
                             const uint32_t track_id,
                             std::vector<BnBuffer> &buffers);

  /// Set Video Track parameters
  status_t SetVideoTrackParam(const uint32_t client_id,
                              const uint32_t session_id,
                              const uint32_t track_id,
                              VideoParam type,
                              void *param,
                              size_t size);

  /// Image Capture
  status_t CaptureImage(const uint32_t client_id,
                        const uint32_t camera_id,
                        const SnapshotType type,
                        const uint32_t n_images,
                        const std::vector<::camera::CameraMetadata> &meta);

  /// Configuration for Image Capture
  status_t ConfigImageCapture(const uint32_t client_id,
                              const uint32_t camera_id,
                              const uint32_t image_id,
                              const ImageParam &param,
                              const ImageExtraParam &xtraparam);

  /// Cancel Image Capture
  status_t CancelCaptureImage(const uint32_t client_id,
                              const uint32_t camera_id,
                              const uint32_t image_id,
                              const bool cache);

  /// Return Image Capture buffer
  status_t ReturnImageCaptureBuffer(const uint32_t client_id,
                                    const uint32_t camera_id,
                                    const int32_t buffer_id);

  /// Set Camera parameters
  status_t SetCameraParam(const uint32_t client_id,
                          const uint32_t camera_id, const ::camera::CameraMetadata &meta);

  /// Get Camera parameters
  status_t GetCameraParam(const uint32_t client_id,
                          const uint32_t camera_id, ::camera::CameraMetadata &meta);

  /// Set Camera Session parameters
  status_t SetCameraSessionParam(const uint32_t client_id,
                                 const uint32_t camera_id,
                                 const ::camera::CameraMetadata &meta);

  /// Set Camera SHDR mode
  status_t SetSHDR(const uint32_t client_id,
                   const uint32_t camera_id,
                   const bool enable);

  /// Get default Capture parameters
  status_t GetDefaultCaptureParam(const uint32_t client_id,
                                  const uint32_t camera_id,
                                  ::camera::CameraMetadata &meta);

  /// Get static metadata
  status_t GetCameraCharacteristics(const uint32_t client_id,
                                    const uint32_t camera_id,
                                    ::camera::CameraMetadata &meta);

  status_t CreateOfflineJPEG(const uint32_t client_id,
                             const OfflineJpegCreateParams& params);

  status_t EncodeOfflineJPEG(const uint32_t client_id,
                             const BnBuffer& in_buf,
                             const BnBuffer& out_buf,
                             const OfflineJpegMeta& meta);

  status_t DestroyOfflineJPEG(const uint32_t client_id);


  // Data callback handlers.
  /// Video Track buffer callback handler
  void VideoTrackBufferCb(uint32_t client_id, uint32_t session_id,
                          uint32_t track_id, std::vector<BnBuffer>& buffers,
                          std::vector<BufferMeta>& metas);

  /// Camera Snapshot callback handler
  void CameraSnapshotCb(uint32_t client_id, uint32_t camera_id, uint32_t count,
                        BnBuffer& buffer, BufferMeta& meta);

  /// Camera Result callback handler
  void CameraResultCb(uint32_t camera_id, const ::camera::CameraMetadata &result);

  /// Camera Error callback handler
  void CameraErrorCb(uint32_t camera_id, uint32_t errcode);

  // Get suitable trackid for linked stream
  uint32_t FindSuitableIdForLinkedTrack(const VideoTrackParam& params);

/// @cond PRIVATE
 private:
  enum class ClientState {
    kAlive,
    kDead,
  };

  enum class SessionState {
    kActive,
    kPause,
    kIdle,
  };

  // <client track id, service track id>
  typedef std::map<uint32_t, uint32_t> TrackMap;
  // <session id, map <tracks> >
  typedef std::map<uint32_t, TrackMap> SessionTrackMap;
  // <client id, <session_id, vector<client track id, service track id> > >
  typedef std::map<uint32_t, SessionTrackMap> ClientSessionMap;

  // <client id, map<camera id, owned?> >
  typedef std::map<uint32_t, std::map<uint32_t, bool> > ClientCameraIdMap;

  // <client id, ClientState>
  typedef std::map<uint32_t, ClientState> ClientStateMap;

  // <session_id, SessionState>
  typedef std::map<uint32_t, SessionState> SessionStateMap;
  // <client_id, SessionStateMap>
  typedef std::map<uint32_t, SessionStateMap> ClientSessionStateMap;

  // <session_id, session_mutex>
  typedef std::map<uint32_t, std::mutex *> SessionMutexMap;
  // <client_id, SessionStateMap>
  typedef std::map<uint32_t, SessionMutexMap> ClientSessionMutexMap;


  bool IsClientValid(const uint32_t& client_id);
  bool IsClientAlive(const uint32_t& client_id);
  bool IsSessionValid(const uint32_t& client_id, const uint32_t& session_id);
  bool IsTrackValid(const uint32_t& client_id, const uint32_t& session_id,
                    const uint32_t& track_id);
  bool IsTrackValid(const uint32_t& client_id, const uint32_t& track_id);
  bool IsCameraValid(const uint32_t& client_id, const uint32_t& camera_id);
  bool IsCameraOwned(const uint32_t& client_id, const uint32_t& camera_id);

  bool IsSessionActive(const uint32_t& client_id, const uint32_t& session_id);
  bool IsSessionPaused(const uint32_t& client_id, const uint32_t& session_id);
  bool IsSessionIdle(const uint32_t& client_id, const uint32_t& session_id);
  void ChangeSessionState(const uint32_t& client_id,
                          const uint32_t& session_id,
                          const SessionState& state);

  uint32_t GetUniqueServiceTrackId(const uint32_t& client_id,
                                   const uint32_t& session_id,
                                   const uint32_t& track_id);

  uint32_t GetServiceTrackId(const uint32_t& client_id,
                             const uint32_t& session_id,
                             const uint32_t& track_id);

  uint32_t GetServiceTrackId(const uint32_t& client_id,
                             const uint32_t& track_id);

  std::vector<uint32_t> GetCameraClients(const uint32_t& camera_id);

  status_t ForceReturnBuffers(const uint32_t client_id);

  status_t GetUniqueSessionID(const uint32_t& client_id, uint32_t* session_id);

  CameraSource*                 camera_source_;

#ifdef ENABLE_OFFLINE_JPEG
  OfflineJpegEncoder*           offline_jpeg_encoder_;
#endif

  RemoteCallbackHandle          remote_cb_handle_;

  ClientSessionMap              client_session_map_;
  std::mutex                    client_session_lock_;

  ClientCameraIdMap             client_cameraid_map_;
  QCondition                    slave_camera_closed_;
  std::mutex                    camera_map_lock_;

  ClientStateMap                client_state_;
  std::mutex                    client_state_lock_;

  ClientSessionStateMap         client_sessions_state_;

  ClientSessionMutexMap         client_sessions_mutex_map_;

  std::mutex                    stop_camera_lock_;

  // Not allowed
  RecorderImpl();
  RecorderImpl(const RecorderImpl&);
  RecorderImpl& operator=(const RecorderImpl&);
  static RecorderImpl* instance_;
  /// @endcond

};

}; // namespace recorder

}; //namespace qmmf
