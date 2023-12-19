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


#define LOG_TAG "RecorderImpl"

#include "recorder/src/service/qmmf_recorder_impl.h"

#include <functional>
#include <future>

#ifdef LOG_LEVEL_KPI
volatile uint32_t kpi_debug_level = BASE_KPI_FLAG;
#endif

namespace qmmf {

namespace recorder {

RecorderImpl* RecorderImpl::instance_ = nullptr;

RecorderImpl* RecorderImpl::CreateRecorder() {

  if (!instance_) {
    instance_ = new RecorderImpl;
    if (!instance_) {
      QMMF_ERROR("%s: Can't Create Recorder Instance!", __func__);
      return NULL;
    }
  }
  QMMF_INFO("%s: Recorder Instance Created Successfully(0x%p)",
      __func__, instance_);
  return instance_;
}

RecorderImpl::RecorderImpl() : camera_source_(nullptr) {

  QMMF_GET_LOG_LEVEL();
  QMMF_KPI_GET_MASK();
  QMMF_KPI_DETAIL();
  QMMF_INFO("%s: Enter", __func__);
#ifdef ENABLE_OFFLINE_JPEG
  offline_jpeg_encoder_ = nullptr;
#endif

  QMMF_INFO("%s: Exit", __func__);
}

RecorderImpl::~RecorderImpl() {

  QMMF_KPI_DETAIL();
  QMMF_INFO("%s: Enter", __func__);

  if (camera_source_) {
    delete camera_source_;
    camera_source_ = nullptr;
  }

#ifdef ENABLE_OFFLINE_JPEG
  if (offline_jpeg_encoder_) {
    delete offline_jpeg_encoder_;
    offline_jpeg_encoder_ = nullptr;
  }
#endif

  instance_ = nullptr;
  QMMF_INFO("%s: Exit (0x%p)", __func__, this);
}

status_t RecorderImpl::Init(const RemoteCallbackHandle& remote_cb_handle) {

  QMMF_INFO("%s: Enter", __func__);
  QMMF_KPI_DETAIL();

  assert(remote_cb_handle != nullptr);
  remote_cb_handle_ = remote_cb_handle;

  camera_source_ = CameraSource::CreateCameraSource();
  if (!camera_source_) {
    QMMF_ERROR("%s: Can't Create CameraSource Instance!", __func__);
    return NO_MEMORY;
  }
  QMMF_INFO("%s: CameraSource Instance Created Successfully!",
      __func__);

#ifdef ENABLE_OFFLINE_JPEG
  offline_jpeg_encoder_ = new OfflineJpegEncoder;
  if (!offline_jpeg_encoder_) {
    QMMF_ERROR("%s: Can't Create OfflineJpegEncoder Instance!", __func__);
    return NO_MEMORY;
  }

  status_t ret = offline_jpeg_encoder_->Init(remote_cb_handle);
  if (NO_ERROR != ret) {
    QMMF_ERROR("%s: Offline JPEG lib initialization failed!", __func__);
    delete offline_jpeg_encoder_;
    offline_jpeg_encoder_ = nullptr;
    return ret;
  }
#endif

  QMMF_INFO("%s: Exit", __func__);
  return NO_ERROR;
}

status_t RecorderImpl::DeInit() {

  QMMF_INFO("%s: Enter", __func__);
  QMMF_KPI_DETAIL();

  if (camera_source_) {
    delete camera_source_;
    camera_source_ = nullptr;
  }

#ifdef ENABLE_OFFLINE_JPEG
  if (offline_jpeg_encoder_) {
    offline_jpeg_encoder_->DeInit();
    delete offline_jpeg_encoder_;
    offline_jpeg_encoder_ = nullptr;
  }
#endif

  QMMF_INFO("%s: Exit", __func__);
  return NO_ERROR;
}

status_t RecorderImpl::RegisterClient(const uint32_t client_id) {

  QMMF_INFO("%s: Enter client_id(%u)", __func__, client_id);

  std::lock_guard<std::mutex> session_lock(client_session_lock_);
  if (client_session_map_.count(client_id) != 0) {
    QMMF_WARN("%s: Client is already connected !!", __func__);
    return NO_ERROR;
  }
  client_session_map_.emplace(client_id, SessionTrackMap());
  QMMF_INFO("%s: client_session_map_.size(%lu)", __func__,
      client_session_map_.size());

  client_sessions_state_.emplace(client_id, SessionStateMap());
  QMMF_INFO("%s: client_sessions_state_.size(%lu)", __func__,
      client_sessions_state_.size());

  client_sessions_mutex_map_.emplace(client_id, SessionMutexMap());
  QMMF_INFO("%s: client_sessions_mutex_map_.size(%lu)", __func__,
      client_sessions_mutex_map_.size());

  auto const& session_track_map = client_session_map_[client_id];
  QMMF_INFO("%s: session_track_map.size(%lu)", __func__,
      session_track_map.size());

  auto const& sessions_state_map = client_sessions_state_[client_id];
  QMMF_INFO("%s: sessions_state_map.size(%lu)", __func__,
      sessions_state_map.size());

  std::lock_guard<std::mutex> status_lock(client_state_lock_);
  client_state_.emplace(client_id, ClientState::kAlive);
  QMMF_INFO("%s: client_status_map_.size(%lu)", __func__,
      client_cameraid_map_.size());

  std::lock_guard<std::mutex> camera_lock(camera_map_lock_);
  client_cameraid_map_.emplace(client_id, std::map<uint32_t, bool>());

#ifdef ENABLE_OFFLINE_JPEG
  if (offline_jpeg_encoder_) {
    offline_jpeg_encoder_->RegisterClient(client_id);
  }
#endif

  QMMF_INFO("%s: Exit client_id(%u)", __func__, client_id);
  return NO_ERROR;
}

status_t RecorderImpl::DeRegisterClient(const uint32_t client_id,
                                        bool force_cleanup) {

  QMMF_INFO("%s: Enter client_id(%u)", __func__, client_id);

  status_t ret = NO_ERROR;
  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s: Client(%u) is not connected!", __func__, client_id);
    return BAD_VALUE;
  }


  if (!force_cleanup) {
    QMMF_WARN("%s Resources belonging to client(%d) are not released!",
        __func__, client_id);
    std::unique_lock<std::mutex> lk(camera_map_lock_);
    if (client_cameraid_map_[client_id].empty()) {
      client_cameraid_map_.erase(client_id);
    }
    return NO_ERROR;
  }

#ifdef ENABLE_OFFLINE_JPEG
  if (offline_jpeg_encoder_) {
    offline_jpeg_encoder_->DeRegisterClient(client_id);
  }
#endif

  // This is the case when client is dead before releasing its acquired
  // resources, service is trying to free up his resources to avoid
  // affecting other connected clients, worst case if this doesn't help
  // then micro restart is the only option left.
  QMMF_INFO("%s: triggering force cleanup for dead client(%d)",
      __func__, client_id);
  {
    // Raise the client_died_ flag in order to signal the video
    // track callbacks to return the buffers from where they originated.
    std::lock_guard<std::mutex> lock(client_state_lock_);
    client_state_[client_id] = ClientState::kDead;
  }

  // Try to release buffer which are held by dead client.
  ret = ForceReturnBuffers(client_id);
  if (ret != NO_ERROR) {
    QMMF_WARN("%s: Client(%u): Buffers clean up failed!", __func__, client_id);
    // Carry-on even return buffers fails.
  }

  {
    // Cleanup client sessions.
    std::unique_lock<std::mutex> lk(client_session_lock_);
    auto session_track_map = client_session_map_[client_id];
    lk.unlock();

    for (auto session : session_track_map) {
      auto session_id = session.first;

      ret = StopSession(client_id, session_id, false, true);
      if (ret != NO_ERROR) {
        QMMF_WARN("%s: Client(%u): Session(%u) internal stop failed!,",
            __func__, client_id, session_id);
        // Carry-on even stop session fails.
      }
      auto track = session.second.rbegin();
      while (track != session.second.rend()) {
        uint32_t client_track_id  = track->first;
        uint32_t service_track_id = track->second;

        QMMF_INFO("%s: Track to Delete, client_id(%u):session_id(%u), "
            "client_track_id(%u):service_track_id(%x)", __func__,
            client_id, session_id, client_track_id, service_track_id);

        ret = DeleteVideoTrack(client_id, session_id, client_track_id);

        // Carry-on even delete track fails.
        ++track;
      }
      ret = DeleteSession(client_id, session_id);
      if (ret != NO_ERROR) {
        QMMF_WARN("%s: Internal delete session is failed! client_id(%u):"
            "session_id(%u)", __func__, client_id, session_id);
      }
    }
    QMMF_INFO("%s: Number of sessions(%lu) left after cleanup for"
        " client(%u)!", __func__, session_track_map.size(), client_id);

    lk.lock();
    client_session_map_.erase(client_id);
    client_sessions_state_.erase(client_id);
    client_sessions_mutex_map_.erase(client_id);
  }

  {
    // Close the cameras owned by the client.
    std::unique_lock<std::mutex> lk(camera_map_lock_);
    auto cameras = client_cameraid_map_[client_id];
    lk.unlock();

    QMMF_INFO("%s: Client(%u) Cameras %lu", __func__, client_id, cameras.size());
    for (auto camera : cameras) {
      auto camera_id = camera.first;
      ret = StopCamera(client_id, camera_id);
      if (ret != NO_ERROR) {
        QMMF_INFO("%s: Client(%u): Camera ID(%d) close failed!", __func__,
            client_id, camera_id);
        // Go ahead with removing camera id from map.
      }
    }

    lk.lock();
    client_cameraid_map_.erase(client_id);
  }

  std::lock_guard<std::mutex> lock(client_state_lock_);
  client_state_.erase(client_id);

  QMMF_INFO("%s: Exit client_id(%u)", __func__, client_id);
  return NO_ERROR;
}

status_t RecorderImpl::StartCamera(const uint32_t client_id,
                                   const uint32_t camera_id,
                                   const float framerate,
                                   const CameraExtraParam& extra_param,
                                   bool enable_result_cb) {

  QMMF_DEBUG("%s: Enter", __func__);
  QMMF_KPI_DETAIL();

  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s: Client(%u) is not connected!", __func__, client_id);
    return BAD_VALUE;
  }

  bool owned = IsCameraOwned(client_id, camera_id);

  CameraSlaveMode camera_slave_mode = {};
  if (extra_param.Exists(QMMF_CAMERA_SLAVE_MODE)) {
    size_t entry_count = extra_param.EntryCount(QMMF_CAMERA_SLAVE_MODE);
    if (entry_count == 1) {
      extra_param.Fetch(QMMF_CAMERA_SLAVE_MODE, camera_slave_mode, 0);
    }
  }

  if ((camera_slave_mode.mode == SlaveMode::kSlave) && !owned) {
    QMMF_WARN("%s Client(%u): Camera(%u) hasn't been opened yet,"
        " operation not allowed!", __func__, client_id, camera_id);
    return NAME_NOT_FOUND;
  } else if ((camera_slave_mode.mode == SlaveMode::kSlave) && owned) {
    QMMF_INFO("%s Client(%u): Camera(%u) is already owned by another client,"
        " using camera in slave mode!", __func__, client_id, camera_id);
    std::lock_guard<std::mutex> lock(camera_map_lock_);
    client_cameraid_map_[client_id].emplace(camera_id, false);
    return NO_ERROR;
  } else if (owned) {
    QMMF_WARN("%s Client(%u): Camera(%u) is already owned by another client,"
        " operation not allowed!", __func__, client_id, camera_id);
    return INVALID_OPERATION;
  }

  if (IsCameraValid(client_id, camera_id)) {
    QMMF_WARN("%s Client(%u): Camera(%u) has been already started,"
        " operation not allowed!", __func__, client_id, camera_id);
    return INVALID_OPERATION;
  }

  assert(camera_source_ != nullptr);
  ResultCb cb = [&] (uint32_t camera_id, const ::camera::CameraMetadata &result) {
    CameraResultCb(camera_id, result);
  };

  ErrorCb errcb = [&] (uint32_t camera_id, uint32_t errcode) {
      CameraErrorCb(camera_id, errcode); };

  auto ret = camera_source_->StartCamera(camera_id, framerate, extra_param,
                                         enable_result_cb ? cb : nullptr,
                                         errcb);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: StartCamera Failed!!", __func__);
    return BAD_VALUE;
  }

  std::lock_guard<std::mutex> lock(camera_map_lock_);

  // Notify all clients, except this one, that the camera has been opened.
  for (auto it : client_cameraid_map_) {
    auto& client = it.first;
    if (client != client_id) {
      remote_cb_handle_(client)->NotifyRecorderEvent(
          EventType::kCameraOpened,
          const_cast<void*>(reinterpret_cast<const void*>(&camera_id)),
          sizeof(uint32_t));
    }
  }

  client_cameraid_map_[client_id].emplace(camera_id, true);

  QMMF_INFO("%s: Number of clients connected(%lu)", __func__,
      client_cameraid_map_.size());

  for (auto iter : client_cameraid_map_) {
    auto cameras = iter.second;
    auto client = iter.first;

    QMMF_INFO("%s client_id(%u): number of cameras(%lu) owned!",
        __func__, client, cameras.size());
    for (auto camera : cameras) {
      auto& camid = camera.first;
      QMMF_INFO("%s: client_id(%u): camera_id(%d)", __func__, client, camid);
    }
  }
  QMMF_DEBUG("%s: Exit", __func__);
  return NO_ERROR;
}

status_t RecorderImpl::StopCamera(const uint32_t client_id,
                                  const uint32_t camera_id) {

  QMMF_DEBUG("%s: Enter", __func__);
  QMMF_KPI_DETAIL();

  std::unique_lock<std::mutex> lk(stop_camera_lock_);

  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s: Client(%u) is not connected!", __func__, client_id);
    return BAD_VALUE;
  }

  if (IsCameraOwned(client_id, camera_id)) {
    QMMF_WARN("%s Client(%u): Camera(%u) is not owned by this client,"
        " closing camera in slave mode!", __func__, client_id, camera_id);
    std::lock_guard<std::mutex> lock(camera_map_lock_);
    client_cameraid_map_[client_id].erase(camera_id);
    slave_camera_closed_.SignalAll();
    return NO_ERROR;
  }

  if (!IsCameraValid(client_id, camera_id)) {
    QMMF_ERROR("%s Client(%u): Camera(%u) is not owned by this client,"
        " operation not allowed!", __func__, client_id, camera_id);
    return INVALID_OPERATION;
  }
  assert(camera_source_ != nullptr);

  // Notify all clients, except this one, that the camera is about to be closed.
  for (auto it : client_cameraid_map_) {
    auto& client = it.first;
    if (client != client_id) {
      remote_cb_handle_(client)->NotifyRecorderEvent(
          EventType::kCameraClosing,
          const_cast<void*>(reinterpret_cast<const void*>(&camera_id)),
          sizeof(uint32_t));
    }
  }

  {
    std::unique_lock<std::mutex> lk(camera_map_lock_);
    std::chrono::milliseconds timeout(1000);

    // Wait until all slave camera clients have closed their connections.
    auto ret = slave_camera_closed_.WaitFor(lk, timeout, [&]() {
      for (auto const& it : client_cameraid_map_) {
        auto const& cameras = it.second;
        if ((cameras.count(camera_id) != 0) && !cameras.at(camera_id)) {
          return false;
        }
      }
      return true;
    });
    if (ret != 0) {
      QMMF_ERROR("%s: Failed, slave camera clients still active!!", __func__);
      return TIMED_OUT;
    }
  }

  auto ret = camera_source_->StopCamera(camera_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: StopCamera Failed!!", __func__);
    return BAD_VALUE;
  }

  std::lock_guard<std::mutex> lock(camera_map_lock_);
  client_cameraid_map_[client_id].erase(camera_id);

  // Notify all clients, except this one, that the camera has been closed.
  for (auto it : client_cameraid_map_) {
    auto& client = it.first;
    if (client_id != client) {
      remote_cb_handle_(client)->NotifyRecorderEvent(
          EventType::kCameraClosed,
          const_cast<void*>(reinterpret_cast<const void*>(&camera_id)),
          sizeof(uint32_t));
    }
  }

  QMMF_INFO("%s client_id(%u): number of cameras(%lu)", __func__,
      client_id, client_cameraid_map_[client_id].size());

  QMMF_DEBUG("%s: Exit", __func__);
  return NO_ERROR;
}

status_t RecorderImpl::CreateSession(const uint32_t client_id,
                                     uint32_t *session_id) {

  QMMF_DEBUG("%s: Enter", __func__);
  QMMF_KPI_DETAIL();

  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s: Client(%u) is not connected!", __func__, client_id);
    return BAD_VALUE;
  }

  if (!camera_source_) {
    QMMF_ERROR("%s: Can't Create Session! Connect Should be called before"
        " Calling CreateSession", __func__);
    return NO_INIT;
  }
  std::lock_guard<std::mutex> lock(client_session_lock_);

  if (GetUniqueSessionID(client_id, session_id) != NO_ERROR) {
    QMMF_ERROR("%s: Active sessions limit reached for Client(%u), 255!",
        __func__, client_id);
    return BAD_VALUE;
  }
  auto& session_track_map = client_session_map_[client_id];
  session_track_map.emplace(*session_id, TrackMap());

  auto& sessions_state_map = client_sessions_state_[client_id];
  sessions_state_map.emplace(*session_id, SessionState::kIdle);

  QMMF_INFO("%s: Client(%u): Session(%u) created successfully", __func__,
      client_id, *session_id);

  auto& sessions_mutex_map = client_sessions_mutex_map_[client_id];
  if (sessions_mutex_map.count(*session_id) == 0) {
    sessions_mutex_map.emplace(*session_id, new std::mutex());
  }


  QMMF_DEBUG("%s: Exit", __func__);
  return NO_ERROR;
}

status_t RecorderImpl::DeleteSession(const uint32_t client_id,
                                     const uint32_t session_id) {

  QMMF_DEBUG("%s: Enter", __func__);
  QMMF_KPI_DETAIL();

  int32_t ret = NO_ERROR;
  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s: Client(%u) is not connected!", __func__, client_id);
    return BAD_VALUE;
  }

  if (!IsSessionValid(client_id, session_id)) {
    QMMF_ERROR("%s: Client(%u): Session(%u) is not valid!", __func__,
        client_id, session_id);
    return BAD_VALUE;
  }

  std::lock_guard<std::mutex> lock(client_session_lock_);
  auto& session_track_map = client_session_map_[client_id];
  auto& sessions_state_map = client_sessions_state_[client_id];
  auto& tracks = session_track_map[session_id];
  auto& sessions_mutex_map = client_sessions_mutex_map_[client_id];

  if (!tracks.empty()) {
    QMMF_ERROR("%s: Client(%u): Session(%u) Can't be deleted until all"
        " tracks(%lu) within this session are stopped & deleted!", __func__,
        client_id, session_id, tracks.size());
    return INVALID_OPERATION;
  }

  session_track_map.erase(session_id);
  sessions_state_map.erase(session_id);
  if (sessions_mutex_map.count(session_id) != 0)
    delete sessions_mutex_map[session_id];
  sessions_mutex_map.erase(session_id);


  QMMF_INFO("%s: Number of sessions(%lu) left in client_id(%u)",
      __func__, session_track_map.size(), client_id);

  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}

status_t RecorderImpl::StartSession(const uint32_t client_id,
                                    const uint32_t session_id) {

  QMMF_DEBUG("%s: Enter client_id(%u):session_id(%u)", __func__,
      client_id, session_id);
  QMMF_KPI_DETAIL();

  client_session_lock_.lock();
  auto& session_track_map = client_session_map_[client_id];
  auto& tracks_in_session = session_track_map[session_id];
  auto& sessions_mutex_map = client_sessions_mutex_map_[client_id];
  auto& session_lock = sessions_mutex_map[session_id];

  client_session_lock_.unlock();
  std::lock_guard<std::mutex> lock(*session_lock);

  uint32_t ret = NO_ERROR;
  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s: Client(%u) is not connected!", __func__, client_id);
    return BAD_VALUE;
  }

  if (!IsSessionValid(client_id, session_id)) {
    QMMF_ERROR("%s: Client(%u): Session(%u) is not valid!", __func__,
        client_id, session_id);
    return BAD_VALUE;
  }

  if (IsSessionPaused(client_id, session_id)) {
    QMMF_WARN("%s: Client(%u): Session(%u) is paused, resuming!", __func__,
        client_id, session_id);
    ResumeSession(client_id, session_id);
    return NO_ERROR;
  } else if (IsSessionActive(client_id, session_id)) {
    QMMF_WARN("%s: Client(%u): Session(%u) is already started!", __func__,
        client_id, session_id);
    return NO_ERROR;
  } else if (!IsSessionIdle(client_id, session_id)) {
    QMMF_WARN("%s: Client(%u): Session(%u) hasn't been stopped!", __func__,
        client_id, session_id);
    return NO_ERROR;
  }

  // all of the video tracks associated to one session starts together
  for (auto const& track : tracks_in_session) {
    uint32_t client_track_id  = track.first;
    uint32_t service_track_id = track.second;

    QMMF_INFO("%s: Track to Start, client_id(%u):session_id(%u), "
        "client_track_id(%u):service_track_id(%x)", __func__,
        client_id, session_id, client_track_id, service_track_id);

    assert(camera_source_ != nullptr);
    ret = camera_source_->StartTrackSource(service_track_id);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s: client_id(%u):session_id(%u), StartTrackSource"
          " failed for client_track_id(%u):service_track_id(%x)",
          __func__, client_id, session_id, client_track_id, service_track_id);
      break;
    }

    QMMF_INFO("%s: client_id(%u):session_id(%u), "
        "client_track_id(%u):service_track_id(%x) Started Successfully!",
        __func__, client_id, session_id, client_track_id, service_track_id);
  }
  if (ret == NO_ERROR) {
    QMMF_INFO("%s: client_id(%u):session_id(%u) with num tracks(%lu) Started"
        " Successfully!", __func__, client_id, session_id,
        tracks_in_session.size());
    ChangeSessionState(client_id, session_id, SessionState::kActive);
  }

  QMMF_DEBUG("%s: Exit client_id(%u):session_id(%u)", __func__,
      client_id, session_id);
  return ret;
}

status_t RecorderImpl::StopSession(const uint32_t client_id,
                                   const uint32_t session_id, bool do_flush,
                                   bool is_force_cleanup) {

  QMMF_DEBUG("%s: Enter client_id(%u):session_id(%u)", __func__,
      client_id, session_id);
  QMMF_KPI_DETAIL();

  client_session_lock_.lock();
  auto& session_track_map = client_session_map_[client_id];
  auto& tracks_in_session = session_track_map[session_id];
  auto& sessions_mutex_map = client_sessions_mutex_map_[client_id];
  auto& session_lock = sessions_mutex_map[session_id];

  client_session_lock_.unlock();
  std::lock_guard<std::mutex> lock(*session_lock);

  uint32_t ret = NO_ERROR;
  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s: Client(%u) is not connected!", __func__, client_id);
    return BAD_VALUE;
  }

  if (!IsSessionValid(client_id, session_id)) {
    QMMF_ERROR("%s: Client(%u): Session(%u) is not valid!", __func__,
        client_id, session_id);
    return BAD_VALUE;
  }

  if (IsSessionIdle(client_id, session_id)) {
    QMMF_WARN("%s: Client(%u): Session(%u) not yet started!", __func__,
        client_id, session_id);
    return NO_ERROR;
  }

  QMMF_INFO("%s: client_id(%u):session_id(%u), number of tracks(%lu) to stop",
      __func__, client_id, session_id, tracks_in_session.size());

  // All the tracks associated to one session are stopped together.
  auto track = tracks_in_session.rbegin();
  while (track != tracks_in_session.rend()) {
    uint32_t client_track_id  = track->first;
    uint32_t service_track_id = track->second;

    QMMF_INFO("%s: Track to Stop, client_id(%u):session_id(%u), "
        "client_track_id(%u):service_track_id(%x)", __func__, client_id,
        session_id, client_track_id, service_track_id);

    // Stop TrackSource
    assert(camera_source_ != nullptr);
    ret = camera_source_->StopTrackSource(service_track_id);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s: client_id(%u):session_id(%u), StopTrackSource"
          " failed for client_track_id(%u):service_track_id(%x)",
          __func__, client_id, session_id, client_track_id,
          service_track_id);
      break;
    }

    QMMF_INFO("%s: client_id(%u):session_id(%u), "
        "client_track_id(%u):service_track_id(%x) Stoped Successfully!",
        __func__, client_id, session_id, client_track_id, service_track_id);
    ++track;
  } // tracks loop ends.

   if (ret == NO_ERROR) {
    QMMF_INFO("%s: client_id(%u):session_id(%u) with num tracks(%lu) Stoped"
        " Successfully!", __func__, client_id, session_id,
        tracks_in_session.size());

    ChangeSessionState(client_id, session_id, SessionState::kIdle);
  }
  QMMF_DEBUG("%s: Exit client_id(%u):session_id(%u)", __func__,
      client_id, session_id);
  return ret;
}

status_t RecorderImpl::PauseSession(const uint32_t client_id,
                                    const uint32_t session_id) {

  QMMF_DEBUG("%s: Enter client_id(%u):session_id(%u)", __func__,
      client_id, session_id);
  QMMF_KPI_DETAIL();

  uint32_t ret = NO_ERROR;
  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s: Client(%u) is not connected!", __func__, client_id);
    return BAD_VALUE;
  }

  if (!IsSessionValid(client_id, session_id)) {
    QMMF_ERROR("%s: Client(%u): Session(%u) is not valid!", __func__,
        client_id, session_id);
    return BAD_VALUE;
  }

  if (IsSessionPaused(client_id, session_id)) {
    QMMF_WARN("%s: Client(%u): Session(%u) already paused!", __func__,
        client_id, session_id);
    return NO_ERROR;
  } else if (IsSessionIdle(client_id, session_id)) {
    QMMF_WARN("%s: Client(%u): Session(%u) hasn't been started!", __func__,
        client_id, session_id);
    return NO_ERROR;
  }

  client_session_lock_.lock();
  auto session_track_map = client_session_map_[client_id];
  auto tracks_in_session = session_track_map[session_id];
  client_session_lock_.unlock();

  QMMF_INFO("%s: client_id(%u):session_id(%u),number of tracks(%lu) to Pause",
      __func__, client_id, session_id, tracks_in_session.size());

  // All the tracks associated to one session are paused together.
  auto track = tracks_in_session.rbegin();
  while (track != tracks_in_session.rend()) {
    uint32_t client_track_id  = track->first;
    uint32_t service_track_id = track->second;

    QMMF_INFO("%s: Track to Pause, client_id(%u):session_id(%u), "
        "client_track_id(%u):service_track_id(%x)", __func__, client_id,
        session_id, client_track_id, service_track_id);

    assert(camera_source_ != nullptr);
    ret = camera_source_->PauseTrackSource(service_track_id);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s: client_id(%u):session_id(%u), PauseTrackSource"
          " failed for client_track_id(%u):service_track_id(%x)",
          __func__, client_id, session_id, client_track_id,
          service_track_id);
      break;
    }

    ++track;
  }
  if (ret == NO_ERROR) {
    QMMF_INFO("%s: client_id(%u):session_id(%u) with num tracks(%lu) Paused"
        " Successfully!", __func__, client_id, session_id,
        tracks_in_session.size());

    ChangeSessionState(client_id, session_id, SessionState::kPause);
  }
  QMMF_DEBUG("%s: Exit client_id(%u):session_id(%u)", __func__,
      client_id, session_id);
  return ret;
}

status_t RecorderImpl::ResumeSession(const uint32_t client_id,
                                     const uint32_t session_id) {

  QMMF_DEBUG("%s: Enter client_id(%u):session_id(%u)", __func__,
      client_id, session_id);
  QMMF_KPI_DETAIL();

  uint32_t ret = NO_ERROR;
  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s: Client(%u) is not connected!", __func__, client_id);
    return BAD_VALUE;
  }

  if (!IsSessionValid(client_id, session_id)) {
    QMMF_ERROR("%s: Client(%u): Session(%u) is not valid!", __func__,
        client_id, session_id);
    return BAD_VALUE;
  }

  if (!IsSessionPaused(client_id, session_id)) {
    QMMF_WARN("%s: Client(%u): Session(%u) hasn't been paused!", __func__,
        client_id, session_id);
    return NO_ERROR;
  }

  client_session_lock_.lock();
  auto session_track_map = client_session_map_[client_id];
  auto tracks_in_session = session_track_map[session_id];
  client_session_lock_.unlock();

  QMMF_INFO("%s:client_id(%u):session_id(%u),number of tracks(%lu) to Resume",
      __func__, client_id, session_id, tracks_in_session.size());
  // All the tracks associated to one session starts together.
  for (auto const& track : tracks_in_session) {
    uint32_t client_track_id  = track.first;
    uint32_t service_track_id = track.second;

    QMMF_INFO("%s: Track to Resume, client_id(%u):session_id(%u), "
        "client_track_id(%u):service_track_id(%x)", __func__, client_id,
        session_id, client_track_id, service_track_id);

    assert(camera_source_ != nullptr);
    ret = camera_source_->ResumeTrackSource(service_track_id);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s: client_id(%u):session_id(%u), ResumeTrackSource"
          " failed for client_track_id(%u):service_track_id(%x)",
          __func__, client_id, session_id, client_track_id,
          service_track_id);
      break;
    }
  }
  if (ret == NO_ERROR) {
    QMMF_INFO("%s: client_id(%u):session_id(%u) with num tracks(%lu) Resumed"
        " Successfully!", __func__, client_id, session_id,
        tracks_in_session.size());

    ChangeSessionState(client_id, session_id, SessionState::kActive);
  }
  QMMF_DEBUG("%s: Exit client_id(%u):session_id(%u)", __func__,
      client_id, session_id);
  return ret;
}

uint32_t RecorderImpl::FindSuitableIdForLinkedTrack(
    const VideoTrackParam& params) {
  bool is_suitable_track_found = false;
  uint32_t selected_track_id = -1;
  auto client_ids = GetCameraClients(params.camera_id);
  for (auto const& id : client_ids) {
    if (IsCameraValid(id, params.camera_id)) {
      auto& main_session_track_map = client_session_map_[id];
      for (auto const& track_map : main_session_track_map) {
        // Try to find a track with same resolution
        for (auto const& track : track_map.second) {
          uint32_t service_track_id = track.second;
          std::shared_ptr<TrackSource> track_source =
              camera_source_->GetTrackSource(service_track_id);
          VideoTrackParam tr_params = track_source->GetParams();

          if (params.width == tr_params.width &&
              params.height == tr_params.height) {
            selected_track_id = service_track_id;
            is_suitable_track_found = true;
            break;
          }
        }
        // Try to find a track with bigger resolution
        if(!is_suitable_track_found) {
          uint32_t selected_width = 0;
          uint32_t selected_height = 0;
          for (auto const& track : track_map.second) {
            uint32_t service_track_id = track.second;
            std::shared_ptr<TrackSource> track_source =
                camera_source_->GetTrackSource(service_track_id);
            VideoTrackParam tr_params = track_source->GetParams();

            if (params.width <= tr_params.width &&
                params.height <= tr_params.height) {
              // Select the lowest possible resolution from the all running
              // tracks has resolution bigger than requested track.
              if ((selected_width == 0 || tr_params.width < selected_width) ||
                  (selected_height == 0 || tr_params.height < selected_height)) {
                selected_track_id = service_track_id;
                is_suitable_track_found = true;
                selected_width = tr_params.width;
                selected_height = tr_params.height;
              }
            }
          }
        }
      }
      break;
    }
  }
  return selected_track_id;
}

status_t RecorderImpl::CreateVideoTrack(const uint32_t client_id,
                                        const uint32_t session_id,
                                        const uint32_t track_id,
                                        const VideoTrackParam& params,
                                        const VideoExtraParam& xtraparam) {

  QMMF_DEBUG("%s: Enter client_id(%u):session_id(%u)", __func__,
      client_id, session_id);

  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s: Client(%u) is not connected!", __func__, client_id);
    return BAD_VALUE;
  }

  if (!IsSessionValid(client_id, session_id)) {
    QMMF_ERROR("%s: Client(%u): Session(%u) is not valid!", __func__,
        client_id, session_id);
    return BAD_VALUE;
  }

  if (IsTrackValid(client_id, session_id, track_id)) {
    QMMF_ERROR("%s: Client(%d):Session(%d): Track(%d) already exists!",
        __func__, client_id, session_id, track_id);
    return BAD_VALUE;
  }

  uint32_t service_track_id = GetUniqueServiceTrackId(client_id, session_id,
                                                      track_id);
  QMMF_INFO("%s: client_id(%u):session_id(%u) client_track_id(%u):"
      "service_track_id(%x)", __func__, client_id, session_id, track_id,
      service_track_id);

  BnBufferCallback cb = [this, client_id, session_id, track_id]
      (std::vector<BnBuffer>& buffers, std::vector<BufferMeta>& meta) {
          VideoTrackBufferCb(client_id, session_id, track_id,
                             buffers, meta);
      };

  // Local copy of extra params that is goign to be modified is necessary.
  VideoExtraParam extraparams = xtraparam;

  if (extraparams.Exists(QMMF_SOURCE_VIDEO_TRACK_ID)) {
    SourceVideoTrack source_track;
    extraparams.Fetch(QMMF_SOURCE_VIDEO_TRACK_ID, source_track);

    auto source_track_id =
        GetServiceTrackId(client_id, source_track.source_track_id);
    // Overwrite client source track id with service source track id.
    source_track.source_track_id = source_track_id;

    extraparams.Update(QMMF_SOURCE_VIDEO_TRACK_ID, source_track);
  } else if (extraparams.Exists(QMMF_USE_LINKED_TRACK_IN_SLAVE_MODE)) {
    LinkedTrackInSlaveMode linked_track_slave_mode;
    extraparams.Fetch(QMMF_USE_LINKED_TRACK_IN_SLAVE_MODE,
        linked_track_slave_mode);

    if (linked_track_slave_mode.enable) {
      uint32_t selected_track_id = FindSuitableIdForLinkedTrack(params);
      if (selected_track_id != -1) {
        SourceVideoTrack source_track;
        source_track.source_track_id = selected_track_id;
        extraparams.Update(QMMF_SOURCE_VIDEO_TRACK_ID, source_track);
      } else {
        QMMF_ERROR("%s: No suitable track found for linked stream!", __func__);
        return BAD_VALUE;
      }
    }
  }

  // Create Camera track first.
  assert(camera_source_ != nullptr);
  auto ret = camera_source_->CreateTrackSource(service_track_id, params,
                                               extraparams, cb);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: CreateTrackSource track_id(%u):service_track_id(%x) "
        " failed!", __func__, track_id, service_track_id);
    return BAD_VALUE;
  }
  QMMF_INFO("%s: client_id(%u):session_id(%u), TrackSource for "
      "client_track_id(%u):service_track_id(%x) Added Successfully in "
      "CameraSource!", __func__, client_id, session_id, track_id,
      service_track_id);

  // Assosiate track to session.
  std::lock_guard<std::mutex> lock(client_session_lock_);
  auto& session_track_map = client_session_map_[client_id];
  auto& tracks_in_session = session_track_map[session_id];
  tracks_in_session.emplace(track_id, service_track_id);

  QMMF_INFO("%s: client_id(%u), session_id(%u), num sessions=%lu",
      __func__, client_id, session_id, session_track_map.size());
  QMMF_INFO("%s: num of tracks=%lu", __func__, tracks_in_session.size());

  QMMF_INFO("%s: client_track_id(%u):service_track_id(%x)",
      __func__, track_id, service_track_id);

  QMMF_DEBUG("%s: Exit client_id(%u):session_id(%u)", __func__,
      client_id, session_id);
  return NO_ERROR;
}

status_t RecorderImpl::DeleteVideoTrack(const uint32_t client_id,
                                        const uint32_t session_id,
                                        const uint32_t track_id) {

  QMMF_DEBUG("%s: Enter client_id(%u):session_id(%u)", __func__,
      client_id, session_id);
  QMMF_KPI_DETAIL();

  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s: Client(%u) is not connected!", __func__, client_id);
    return BAD_VALUE;
  }

  if (!IsSessionValid(client_id, session_id)) {
    QMMF_ERROR("%s: Client(%u): Session(%u) is not valid!", __func__,
        client_id, session_id);
    return BAD_VALUE;
  }

  if (!IsTrackValid(client_id, session_id, track_id)) {
    QMMF_ERROR("%s: Client(%d):Session(%d): Track(%d) does not exist!",
        __func__, client_id, session_id, track_id);
    return BAD_VALUE;
  }

  client_session_lock_.lock();
  auto& session_track_map = client_session_map_[client_id];
  auto& tracks_in_session = session_track_map[session_id];

  uint32_t service_track_id = tracks_in_session[track_id];
  client_session_lock_.unlock();

  assert(camera_source_ != nullptr);
  assert(service_track_id > 0);
  auto ret = camera_source_->DeleteTrackSource(service_track_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: service_track_id(%x) DeleteTrackSource failed!",
        __func__, service_track_id);
    return ret;
  }

  {
    std::lock_guard<std::mutex> lock(client_session_lock_);
    tracks_in_session.erase(track_id);
  }

  QMMF_INFO("%s: client_track_id(%u):service_track_id(%x) Deleted "
      "Successfully", __func__, track_id, service_track_id);
  QMMF_INFO("%s: Number of tracks(%lu) left in session(%d)", __func__,
      tracks_in_session.size(), session_id);

  // This method doesn't go up to client as a callback, it is just to update
  // Internal data structure used for buffer mapping.
  remote_cb_handle_(client_id)->NotifyDeleteVideoTrack(track_id);

  QMMF_DEBUG("%s: Exit client_id(%u):session_id(%u)", __func__,
      client_id, session_id);
  return NO_ERROR;
}

status_t RecorderImpl::ReturnTrackBuffer(const uint32_t client_id,
                                         const uint32_t session_id,
                                         const uint32_t track_id,
                                         std::vector<BnBuffer> &buffers) {

  QMMF_VERBOSE("%s: Enter client_id(%u):session_id(%u)", __func__,
      client_id, session_id);
  for (const BnBuffer& buffer : buffers)
    QMMF_VERBOSE("%s INPARAM: buffers[%s]", __func__,
                 buffer.ToString().c_str());

  uint32_t ret = NO_ERROR;
  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s: Client(%u) is not connected!", __func__, client_id);
    return BAD_VALUE;
  }

  if (!IsSessionValid(client_id, session_id)) {
    QMMF_ERROR("%s: Client(%u): Session(%u) is not valid!", __func__,
        client_id, session_id);
    return BAD_VALUE;
  }

  if (!IsTrackValid(client_id, session_id, track_id)) {
    QMMF_ERROR("%s: Client(%d):Session(%d): Track(%d) does not exist!",
        __func__, client_id, session_id, track_id);
    return BAD_VALUE;
  }

  uint32_t service_track_id = GetServiceTrackId(client_id, session_id, track_id);

  assert(camera_source_ != nullptr);
  ret = camera_source_->ReturnTrackBuffer(service_track_id, buffers);
  assert(ret == NO_ERROR);

  QMMF_VERBOSE("%s: Exit client_id(%u):session_id(%u)", __func__,
      client_id, session_id);
  return NO_ERROR;
}

status_t RecorderImpl::SetVideoTrackParam(const uint32_t client_id,
                                          const uint32_t session_id,
                                          const uint32_t track_id,
                                          VideoParam type,
                                          void *param,
                                          size_t size) {
  QMMF_DEBUG("%s: Enter client_id(%u):session_id(%u)", __func__,
      client_id, session_id);

  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s: Client(%u) is not connected!", __func__, client_id);
    return BAD_VALUE;
  }

  if (!IsSessionValid(client_id, session_id)) {
    QMMF_ERROR("%s: Client(%u): Session(%u) is not valid!", __func__,
        client_id, session_id);
    return BAD_VALUE;
  }

  if (!IsTrackValid(client_id, session_id, track_id)) {
    QMMF_ERROR("%s: Client(%d):Session(%d): Track(%d) does not exist!",
        __func__, client_id, session_id, track_id);
    return BAD_VALUE;
  }

  uint32_t service_track_id = GetServiceTrackId(client_id, session_id, track_id);
  status_t ret = 0;

  if (type == VideoParam::kFrameRate) {
    float fps = *(static_cast<float*>(param));
    ret = camera_source_->UpdateTrackFrameRate(service_track_id, fps);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s: client_id(%u) Failed to set FrameRate to TrackSource",
          __func__, client_id);
      return ret;
    }
  }

  if (type == VideoParam::kEnableFrameRepeat) {
    bool enable = *(static_cast<bool*>(param));
    ret = camera_source_->EnableFrameRepeat(service_track_id, enable);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s: client_id(%u) Failed to set FrameRepeat to TrackSource!",
          __func__, client_id);
      return ret;
    }
  }
  QMMF_DEBUG("%s: Exit client_id(%u):session_id(%u)", __func__,
      client_id, session_id);
  return NO_ERROR;
}

status_t RecorderImpl::CaptureImage(const uint32_t client_id,
                                    const uint32_t camera_id,
                                    const SnapshotType type,
                                    const uint32_t n_images,
                                    const std::vector<::camera::CameraMetadata> &meta) {

  QMMF_DEBUG("%s: Enter client_id(%u):camera_id(%d)", __func__,
      client_id, camera_id);

  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s: Client(%u) is not connected!", __func__, client_id);
    return BAD_VALUE;
  }

  if (!IsCameraValid(client_id, camera_id)) {
    QMMF_ERROR("%s Client(%u): Camera(%u) is not owned by this client,"
        " operation not allowed!", __func__, client_id, camera_id);
    return INVALID_OPERATION;
  }

  assert(camera_source_ != nullptr);
  SnapshotCb cb = [ this, client_id ] (uint32_t camera_id,
      uint32_t count, BnBuffer& buf, BufferMeta& meta) {
          CameraSnapshotCb(client_id, camera_id, count, buf, meta);
      };

  auto ret = camera_source_->CaptureImage(camera_id, type, n_images, meta, cb);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: client_id(%u):camera_id(%d) CaptureImage failed!",
        __func__, client_id, camera_id);
    return ret;
  }
  QMMF_DEBUG("%s: Exit client_id(%u):camera_id(%d)", __func__,
      client_id, camera_id);;
  return NO_ERROR;
}

status_t RecorderImpl::ConfigImageCapture(const uint32_t client_id,
                                          const uint32_t camera_id,
                                          const uint32_t image_id,
                                          const ImageParam &param,
                                          const ImageExtraParam &xtraparam) {

  QMMF_DEBUG("%s: Enter client_id(%u):camera_id(%d):image_id(%d)", __func__,
      client_id, camera_id, image_id);

  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s: Client(%u) is not connected!", __func__, client_id);
    return BAD_VALUE;
  }

  if (!IsCameraValid(client_id, camera_id)) {
    QMMF_ERROR("%s Client(%u): Camera(%u) is not owned by this client,"
        " operation not allowed!", __func__, client_id, camera_id);
    return INVALID_OPERATION;
  }

  assert(camera_source_ != nullptr);
  auto ret = camera_source_->ConfigImageCapture(camera_id, image_id, param,
                                                xtraparam);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: client_id(%u):camera_id(%d) ConfigImageCapture failed!",
        __func__, client_id, camera_id);
    return ret;
  }
  QMMF_DEBUG("%s: Exit client_id(%u):camera_id(%d)", __func__,
      client_id, camera_id);;
  return NO_ERROR;
}


status_t RecorderImpl::CancelCaptureImage(const uint32_t client_id,
                                          const uint32_t camera_id,
                                          const uint32_t image_id,
                                          const bool cache) {

  QMMF_DEBUG("%s: Enter client_id(%u):camera_id(%d):image_id(%d)", __func__,
      client_id, camera_id, image_id);

  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s: Client(%u) is not connected!", __func__, client_id);
    return BAD_VALUE;
  }

  if (!IsCameraValid(client_id, camera_id)) {
    QMMF_ERROR("%s Client(%u): Camera(%u) is not owned by this client,"
        " operation not allowed!", __func__, client_id, camera_id);
    return INVALID_OPERATION;
  }

  assert(camera_source_ != nullptr);
  auto ret = camera_source_->CancelCaptureImage(camera_id, image_id, cache);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: CancelCaptureImage failed!", __func__);
    return ret;
  }
  QMMF_DEBUG("%s: Exit client_id(%u):camera_id(%d):image_id(%d)", __func__,
      client_id, camera_id, image_id);
  return NO_ERROR;
}

status_t RecorderImpl::ReturnImageCaptureBuffer(const uint32_t client_id,
                                                const uint32_t camera_id,
                                                const int32_t buffer_id) {

  QMMF_DEBUG("%s: Enter client_id(%u):camera_id(%d)", __func__,
      client_id, camera_id);
  assert(camera_source_ != nullptr);
  auto ret = camera_source_->ReturnImageCaptureBuffer(camera_id, buffer_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: ReturnImageCaptureBuffer failed!", __func__);
    return ret;
  }
  QMMF_DEBUG("%s: Exit client_id(%u):camera_id(%d)", __func__,
      client_id, camera_id);
  return NO_ERROR;
}


status_t RecorderImpl::SetCameraParam(const uint32_t client_id,
                                      const uint32_t camera_id,
                                      const ::camera::CameraMetadata &meta) {

  QMMF_DEBUG("%s: Enter client_id(%u):camera_id(%d)", __func__,
      client_id, camera_id);

  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s: Client(%u) is not connected!", __func__, client_id);
    return BAD_VALUE;
  }

  if (!IsCameraValid(client_id, camera_id)) {
    QMMF_ERROR("%s Client(%u): Camera(%u) is not owned by this client,"
        " operation not allowed!", __func__, client_id, camera_id);
    return INVALID_OPERATION;
  }

  assert(camera_source_ != nullptr);
  auto ret = camera_source_->SetCameraParam(camera_id, meta);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: SetCameraParam failed!", __func__);
    return ret;
  }
  QMMF_DEBUG("%s: Enter client_id(%u):camera_id(%d)", __func__,
      client_id, camera_id);
  return NO_ERROR;
}

status_t RecorderImpl::GetCameraParam(const uint32_t client_id,
                                      const uint32_t camera_id,
                                      ::camera::CameraMetadata &meta) {

  QMMF_DEBUG("%s: Enter client_id(%u):camera_id(%d)", __func__,
      client_id, camera_id);

  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s: Client(%u) is not connected!", __func__, client_id);
    return BAD_VALUE;
  }

  if (!IsCameraValid(client_id, camera_id)) {
    QMMF_ERROR("%s Client(%u): Camera(%u) is not owned by this client,"
        " operation not allowed!", __func__, client_id, camera_id);
    return INVALID_OPERATION;
  }

  assert(camera_source_ != nullptr);
  auto ret = camera_source_->GetCameraParam(camera_id, meta);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: GetCameraParam failed!", __func__);
    return ret;
  }
  QMMF_DEBUG("%s: Exit client_id(%u):camera_id(%d)", __func__,
      client_id, camera_id);
  return NO_ERROR;
}

status_t RecorderImpl::SetCameraSessionParam(const uint32_t client_id,
                                             const uint32_t camera_id,
                                             const ::camera::CameraMetadata &meta) {

  QMMF_DEBUG("%s: Enter client_id(%u):camera_id(%d)", __func__,
      client_id, camera_id);

  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s: Client(%u) is not connected!", __func__, client_id);
    return BAD_VALUE;
  }

  if (!IsCameraValid(client_id, camera_id)) {
    QMMF_ERROR("%s Client(%u): Camera(%u) is not owned by this client,"
        " operation not allowed!", __func__, client_id, camera_id);
    return INVALID_OPERATION;
  }

  assert(camera_source_ != nullptr);
  auto ret = camera_source_->SetCameraSessionParam(camera_id, meta);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: SetCameraSessionParam failed!", __func__);
    return ret;
  }
  QMMF_DEBUG("%s: Enter client_id(%u):camera_id(%d)", __func__,
      client_id, camera_id);
  return NO_ERROR;
}

status_t RecorderImpl::SetSHDR(const uint32_t client_id,
                               const uint32_t camera_id,
                               const bool enable) {
  QMMF_DEBUG("%s: Enter client_id(%u):camera_id(%d)", __func__,
      client_id, camera_id);

  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s: Client(%u) is not connected!", __func__, client_id);
    return BAD_VALUE;
  }

  if (!IsCameraValid(client_id, camera_id)) {
    QMMF_ERROR("%s Client(%u): Camera(%u) is not owned by this client,"
        " operation not allowed!", __func__, client_id, camera_id);
    return INVALID_OPERATION;
  }

  assert(camera_source_ != nullptr);
  auto ret = camera_source_->SetSHDR(camera_id, enable);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: client_id(%u) Failed to set SHDR to TrackSource!",
        __func__, client_id);
    return ret;
  }

  QMMF_DEBUG("%s: Exit client_id(%u):camera_id(%d)", __func__,
      client_id, camera_id);
  return NO_ERROR;
}

status_t RecorderImpl::GetDefaultCaptureParam(const uint32_t client_id,
                                              const uint32_t camera_id,
                                              ::camera::CameraMetadata &meta) {

  QMMF_DEBUG("%s: Enter client_id(%u):camera_id(%d)", __func__,
      client_id, camera_id);

  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s: Client(%u) is not connected!", __func__, client_id);
    return BAD_VALUE;
  }

  if (!IsCameraValid(client_id, camera_id)) {
    QMMF_ERROR("%s Client(%u): Camera(%u) is not owned by this client,"
        " operation not allowed!", __func__, client_id, camera_id);
    return INVALID_OPERATION;
  }

  assert(camera_source_ != nullptr);
  auto ret = camera_source_->GetDefaultCaptureParam(camera_id, meta);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: GetDefaultCaptureParam failed!", __func__);
    return ret;
  }
  QMMF_DEBUG("%s: Exit client_id(%u):camera_id(%d)", __func__,
      client_id, camera_id);
  return NO_ERROR;
}

status_t RecorderImpl::GetCameraCharacteristics(const uint32_t client_id,
                                                const uint32_t camera_id,
                                                ::camera::CameraMetadata &meta) {

  QMMF_DEBUG("%s: Enter client_id(%u):camera_id(%d)", __func__,
      client_id, camera_id);

  if (!IsClientValid(client_id)) {
    QMMF_ERROR("%s: Client(%u) is not connected!", __func__, client_id);
    return BAD_VALUE;
  }

  if (!IsCameraValid(client_id, camera_id)) {
    QMMF_ERROR("%s Client(%u): Camera(%u) is not owned by this client,"
        " operation not allowed!", __func__, client_id, camera_id);
    return INVALID_OPERATION;
  }

  assert(camera_source_ != nullptr);
  auto ret = camera_source_->GetCameraCharacteristics(camera_id, meta);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: GetCameraCharacteristics failed!", __func__);
    return ret;
  }
  QMMF_DEBUG("%s: Exit client_id(%u):camera_id(%d)", __func__,
      client_id, camera_id);
  return NO_ERROR;
}

status_t RecorderImpl::CreateOfflineJPEG(const uint32_t client_id,
                                      const OfflineJpegCreateParams& params) {

  QMMF_DEBUG("%s Enter client_id(%u)", __func__, client_id);

#ifdef ENABLE_OFFLINE_JPEG
  assert(offline_jpeg_encoder_ != nullptr);
  if (!offline_jpeg_encoder_->IsClientFound(client_id)) {
    QMMF_ERROR("%s: Client (%u) is not found", __func__, client_id);
    return BAD_VALUE;
  }
  auto ret = offline_jpeg_encoder_->Create(client_id, params);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: Offline JPEG encoder create failed!", __func__);
    return ret;
  }
#else
  QMMF_ERROR("Offline JPEG not supported on this platform");
  return INVALID_OPERATION;
#endif

  QMMF_DEBUG("%s Exit client_id(%u)", __func__, client_id);
  return NO_ERROR;
}

status_t RecorderImpl::EncodeOfflineJPEG(const uint32_t client_id,
                                         const BnBuffer& in_buf,
                                         const BnBuffer& out_buf,
                                         const OfflineJpegMeta& meta) {

  QMMF_DEBUG("%s Enter client_id(%u)", __func__, client_id);

#ifdef ENABLE_OFFLINE_JPEG
  assert(offline_jpeg_encoder_ != nullptr);
  if (!offline_jpeg_encoder_->IsClientFound(client_id)) {
    QMMF_ERROR("%s: Client (%u) is not found", __func__, client_id);
    return BAD_VALUE;
  }
  auto ret = offline_jpeg_encoder_->Process(client_id, in_buf, out_buf, meta);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: Offline JPEG encoder process failed!", __func__);
    return ret;
  }
#else
  QMMF_ERROR("Offline JPEG not supported on this platform");
  return INVALID_OPERATION;
#endif

  QMMF_DEBUG("%s Exit client_id(%u)", __func__, client_id);
  return NO_ERROR;
}

status_t RecorderImpl::DestroyOfflineJPEG(const uint32_t client_id) {

  QMMF_DEBUG("%s Enter client_id(%u)", __func__, client_id);

#ifdef ENABLE_OFFLINE_JPEG
  assert(offline_jpeg_encoder_ != nullptr);
  if (!offline_jpeg_encoder_->IsClientFound(client_id)) {
    QMMF_ERROR("%s: Client (%u) is not found", __func__, client_id);
    return BAD_VALUE;
  }
  auto ret = offline_jpeg_encoder_->Destroy(client_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: Offline JPEG encoder destroy failed!", __func__);
    return ret;
  }
#else
  QMMF_ERROR("Offline JPEG not supported on this platform");
  return INVALID_OPERATION;
#endif

  QMMF_DEBUG("%s Exit client_id(%u)", __func__, client_id);
  return NO_ERROR;
}

// Data callback handlers.
void RecorderImpl::VideoTrackBufferCb(uint32_t client_id, uint32_t session_id,
                                      uint32_t track_id,
                                      std::vector<BnBuffer>& buffers,
                                      std::vector<BufferMeta>& metas) {

  QMMF_DEBUG("%s Enter client_id(%u), session_id(%u), track_id(%u)",
      __func__, client_id, session_id, track_id);
  assert(remote_cb_handle_ != nullptr);
  assert(IsClientValid(client_id));

  if (!IsClientAlive(client_id)) {
    ReturnTrackBuffer(client_id, session_id, track_id, buffers);
  } else {
    remote_cb_handle_(client_id)->
        NotifyVideoTrackData(session_id, track_id, buffers, metas);
  }
  QMMF_DEBUG("%s Exit client_id(%u), session_id(%u), track_id(%u)",
      __func__, client_id, session_id, track_id);
}

void RecorderImpl::CameraSnapshotCb(uint32_t client_id, uint32_t camera_id,
                                    uint32_t imgcount, BnBuffer& buffer,
                                    BufferMeta& meta) {

  QMMF_DEBUG("%s Enter client_id(%u), camera_id(%u), count(%u)",
      __func__, client_id, camera_id, imgcount);
  assert(remote_cb_handle_ != nullptr);
  assert(IsClientValid(client_id));

  remote_cb_handle_(client_id)->NotifySnapshotData(camera_id, imgcount,
                                                   buffer, meta);
  QMMF_DEBUG("%s Exit client_id(%u), camera_id(%u), count(%u)",
      __func__, client_id, camera_id, imgcount);
}

void RecorderImpl::CameraResultCb(uint32_t camera_id,
                                  const ::camera::CameraMetadata &result) {

  QMMF_DEBUG("%s Enter camera_id(%u)", __func__, camera_id);
  assert(remote_cb_handle_ != nullptr);
  auto client_ids = GetCameraClients(camera_id);

  for (auto const& client_id : client_ids) {
    assert(IsClientValid(client_id));
    remote_cb_handle_(client_id)->NotifyCameraResult(camera_id, result);
  }
  QMMF_DEBUG("%s Exit camera_id(%u)", __func__, camera_id);
}

void RecorderImpl::CameraErrorCb(uint32_t camera_id, uint32_t errcode) {

  assert(remote_cb_handle_ != nullptr);
  EventType event = EventType::kUnknown;

  auto client_ids = GetCameraClients(camera_id);

  switch (errcode) {
    case ERROR_CAMERA_DEVICE:
    case ERROR_CAMERA_INVALID_ERROR:
      event = EventType::kCameraError;
      break;
    case ERROR_CAMERA_REQUEST:
    case ERROR_CAMERA_BUFFER:
      event = EventType::kFrameError;
      break;
    case ERROR_CAMERA_RESULT:
      event = EventType::kMetadataError;
      break;
    case REMAP_ALL_BUFFERS:
      event = static_cast<EventType>(REMAP_ALL_BUFFERS);
      break;
    default:
      event = EventType::kUnknown;
      break;
  }

  for (auto const& client_id : client_ids) {
    assert(IsClientValid(client_id));
    remote_cb_handle_(client_id)->NotifyRecorderEvent(
        event, &camera_id, sizeof(uint32_t));
  }
}

bool RecorderImpl::IsClientValid(const uint32_t& client_id) {

  std::lock_guard<std::mutex> lock(client_session_lock_);
  return (client_session_map_.count(client_id) != 0) ? true : false;
}

bool RecorderImpl::IsClientAlive(const uint32_t& client_id) {

  std::lock_guard<std::mutex> lock(client_state_lock_);
  return (client_state_[client_id] == ClientState::kAlive) ? true : false;
}

bool RecorderImpl::IsSessionValid(const uint32_t& client_id,
                                  const uint32_t& session_id) {

  std::lock_guard<std::mutex> lock(client_session_lock_);
  auto& session_track_map = client_session_map_[client_id];
  return (session_track_map.count(session_id) != 0) ? true : false;
}

bool RecorderImpl::IsTrackValid(const uint32_t& client_id,
                                const uint32_t& session_id,
                                const uint32_t& track_id) {

  std::lock_guard<std::mutex> lock(client_session_lock_);
  auto const& tracks_in_session = client_session_map_[client_id][session_id];
  return (tracks_in_session.count(track_id) != 0) ? true : false;
}

bool RecorderImpl::IsTrackValid(const uint32_t& client_id,
                                const uint32_t& track_id) {

  std::lock_guard<std::mutex> lock(client_session_lock_);
  auto const& session_track_map = client_session_map_[client_id];

  for (auto const& session : session_track_map) {
    auto const& tracks_in_session = session.second;
    if (tracks_in_session.count(track_id) != 0) {
      return true;
    }
  }
  return false;
}

bool RecorderImpl::IsCameraValid(const uint32_t& client_id,
                                 const uint32_t& camera_id) {

  std::lock_guard<std::mutex> lock(camera_map_lock_);
  bool valid = false;

  // Check if the camera id is registered for the client id and is owned by it.
  if (client_session_map_.count(client_id) != 0) {
    auto const& cameras = client_cameraid_map_[client_id];
    valid = (cameras.count(camera_id) != 0) ? cameras.at(camera_id) : false;
  }
  return valid;
}

bool RecorderImpl::IsCameraOwned(const uint32_t& client_id,
                                 const uint32_t& camera_id) {

  std::lock_guard<std::mutex> lock(camera_map_lock_);

  for (auto const& client_cameras : client_cameraid_map_) {
    auto const& cameras = client_cameras.second;
    auto const& camera_client_id = client_cameras.first;

    // Ignore check if current client_id.
    if ((client_id != camera_client_id) && (cameras.count(camera_id) != 0) &&
        cameras.at(camera_id)) {
      return true;
    }
  }
  return false;
}

bool RecorderImpl::IsSessionActive(const uint32_t& client_id,
                                   const uint32_t& session_id) {
  std::lock_guard<std::mutex> lock(client_session_lock_);
  auto& sessions_state_map = client_sessions_state_[client_id];
  return (sessions_state_map[session_id] == SessionState::kActive) ? true
                                                                   : false;
}

bool RecorderImpl::IsSessionPaused(const uint32_t& client_id,
                                   const uint32_t& session_id) {
  std::lock_guard<std::mutex> lock(client_session_lock_);
  auto& sessions_state_map = client_sessions_state_[client_id];
  return (sessions_state_map[session_id] == SessionState::kPause) ? true
                                                                  : false;
}

bool RecorderImpl::IsSessionIdle(const uint32_t& client_id,
                                 const uint32_t& session_id) {
  std::lock_guard<std::mutex> lock(client_session_lock_);
  auto& sessions_state_map = client_sessions_state_[client_id];
  return (sessions_state_map[session_id] == SessionState::kIdle) ? true : false;
}

void RecorderImpl::ChangeSessionState(const uint32_t& client_id,
                                      const uint32_t& session_id,
                                      const SessionState& state) {
  std::lock_guard<std::mutex> lock(client_session_lock_);
  auto& sessions_state_map = client_sessions_state_[client_id];
  QMMF_INFO("%s: Session(%u): state = %d", __func__, session_id,
    (int32_t) state);
  sessions_state_map[session_id] = state;
}

uint32_t RecorderImpl::GetUniqueServiceTrackId(const uint32_t& client_id,
                                               const uint32_t& session_id,
                                               const uint32_t& track_id) {
  uint32_t service_track_id = client_id << 24;
  service_track_id |= session_id << 16;
  service_track_id |= track_id;
  return service_track_id;
}

uint32_t RecorderImpl::GetServiceTrackId(const uint32_t& client_id,
                                         const uint32_t& session_id,
                                         const uint32_t& track_id) {

  std::lock_guard<std::mutex> lock(client_session_lock_);
  auto tracks_in_session = client_session_map_[client_id][session_id];
  return tracks_in_session[track_id];
}

uint32_t RecorderImpl::GetServiceTrackId(const uint32_t& client_id,
                                         const uint32_t& track_id) {

  std::lock_guard<std::mutex> lock(client_session_lock_);
  auto const& session_track_map = client_session_map_[client_id];

  for (auto const& session : session_track_map) {
    auto tracks_in_session = session.second;
    if (tracks_in_session.count(track_id) != 0) {
      return tracks_in_session[track_id];
    }
  }
  return NO_ERROR;
}

std::vector<uint32_t> RecorderImpl::GetCameraClients(const uint32_t& camera_id) {

  std::lock_guard<std::mutex> lock(camera_map_lock_);
  std::vector<uint32_t> client_ids;

  for (auto const& client_cameras : client_cameraid_map_) {
    auto const& cameras = client_cameras.second;
    auto const& client_id = client_cameras.first;

    if (cameras.count(camera_id) != 0) {
      client_ids.push_back(client_id);
    }
  }
  return client_ids;
}

status_t RecorderImpl::ForceReturnBuffers(const uint32_t client_id) {

  assert(camera_source_ != nullptr);

  uint32_t ret = NO_ERROR;

  // Return all image capture buffers
  auto const& cameras = client_cameraid_map_[client_id];
  for (auto camera : cameras) {
    auto camera_id = camera.first;
    ret = camera_source_->ReturnAllImageCaptureBuffers(camera_id);
    if (ret != NO_ERROR) {
      QMMF_WARN("%s: ReturnAllImageCaptureBuffers failed for camera_id %d",
          __func__, camera_id);
    }
  }

  // Return all track buffers
  client_session_lock_.lock();
  auto session_track_map = client_session_map_[client_id];
  client_session_lock_.unlock();

  // iterate all sessions for this client
  for (auto session : session_track_map) {
    auto session_id = session.first;

    client_session_lock_.lock();
    auto tracks_in_session = session_track_map[session_id];
    client_session_lock_.unlock();

    // iterate all tracks for this session
    auto track = tracks_in_session.rbegin();
    while (track != tracks_in_session.rend()) {
      uint32_t client_track_id  = track->first;
      uint32_t service_track_id = track->second;

      QMMF_INFO("%s: Return buffers to track, client_id(%u):session_id(%u), "
          "client_track_id(%u):service_track_id(%x)", __func__, client_id,
          session_id, client_track_id, service_track_id);

      ret = camera_source_->FlushTrackSource(service_track_id);
      if (ret != NO_ERROR) {
        QMMF_WARN("%s: FlushTrackSource failed for track_id %d", __func__,
            service_track_id);
      }
      ++track;
    }
  }
  return ret;
}

status_t RecorderImpl::GetUniqueSessionID(const uint32_t& client_id,
                                          uint32_t* session_id) {
  for (uint32_t id = 1; id <= 0xFF; id++) {
    if (client_sessions_state_[client_id].count(id) == 0) {
      *session_id = id;
      return NO_ERROR;
    }
  }
  return BAD_VALUE;
}

}; // namespace recorder

}; //namespace qmmf
