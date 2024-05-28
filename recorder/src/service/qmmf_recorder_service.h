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

#pragma once

#include <atomic>

#ifndef HAVE_BINDER
#include <future>
#include <queue>
#include <thread>
#include <unistd.h>
#include <vector>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <iostream>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#endif // !HAVE_BINDER

#include "qmmf-sdk/qmmf_camera_metadata.h"
#include "qmmf-sdk/qmmf_vendor_tag_descriptor.h"
#include "recorder/src/client/qmmf_recorder_service_intf.h"
#include "recorder/src/service/qmmf_recorder_impl.h"

namespace qmmf {

namespace recorder {

#ifdef HAVE_BINDER
using namespace android;
class RecorderService : public BnInterface<IRecorderService> {
 public:
  RecorderService();

  ~RecorderService();

 private:

  typedef std::function <void(void)> NotifyClientDeath;
  class DeathNotifier : public IBinder::DeathRecipient {
   public:
    DeathNotifier() {}

    void SetDeathNotifyCB(NotifyClientDeath& cb) {
      notify_client_death_ = cb;
    }
    void binderDied(const wp<IBinder>&) override {
      QMMF_WARN("RecorderSerive:%s: Client Exited or Died!", __func__);
      notify_client_death_();
    }
    NotifyClientDeath notify_client_death_;
  };

  // Method of BnInterface<IRecorderService>.
  // This method would get call to handle incoming messages from clients.
  status_t onTransact(uint32_t code, const Parcel& data,
                               Parcel* reply, uint32_t flags = 0) override;

  status_t Connect(const sp<IRecorderServiceCallback>& service_cb,
                   uint32_t* client_id) override;
#else

enum class TASK_PRIORITY { LOW, NORMAL, HIGH };
class ThreadPool {
 public:
  ThreadPool();
  ~ThreadPool() {
    QMMF_INFO("%s", __func__);
    stop_ = true;
    condition_.notify_all();
    for (std::thread &worker : workers_) {
      worker.join();
    }
  }

  template <typename Func>
  auto Enqueue(Func &&func, TASK_PRIORITY priority)
      -> std::future<decltype(func())>;
  void CancelAllTasks();

  std::pair<size_t, size_t> GetTaskStats() const {
    std::unique_lock<std::mutex> lock(executed_mutex_);
    return {total_tasks_executed_, total_tasks_cancelled_};
  }

  // Set the cancellation flag for the specified thread.
  void SetTaskCancelled(size_t thread_id);

  // Check if the task for the specified thread is canceled.
  bool IsTaskCancelled(size_t thread_id) const;

 private:
  struct TaskPriorityWrapper {
    TASK_PRIORITY priority;
    std::function<void()> task;

    TaskPriorityWrapper(TASK_PRIORITY p, std::function<void()> t)
        : priority(p), task(std::move(t)) {}

    bool operator<(const TaskPriorityWrapper &other) const {
      return priority < other.priority;
    }
  };

  std::vector<std::thread> workers_;
  std::vector<bool> worker_states_; // Per-thread cancellation state.
  std::priority_queue<TaskPriorityWrapper> tasks_;
  std::mutex queue_mutex_;
  mutable std::mutex executed_mutex_;
  mutable std::mutex thread_state_mutex_;
  std::condition_variable condition_;
  std::atomic<bool> stop_;
  std::atomic<size_t> total_tasks_executed_;
  std::atomic<size_t> total_tasks_cancelled_;

  bool HasTasks() const { return !tasks_.empty(); }

  std::function<void()> GetHighestPriorityTask() {
    auto task = tasks_.top();
    tasks_.pop();
    return task.task;
  }
};

class RecorderServiceCallbackProxy: public IRecorderServiceCallback {
 public:
  RecorderServiceCallbackProxy() {
    QMMF_INFO("%s: Enter ", __func__);
    QMMF_INFO("%s: Exit ", __func__);
  }

  ~RecorderServiceCallbackProxy() {
    QMMF_INFO("%s: Enter ", __func__);

    close(callback_socket_);

    track_buffers_map_.clear();

    QMMF_INFO("%s: Exit ", __func__);
  }

  status_t Init(uint32_t client_id, uint32_t server_pid) override;

  void NotifyRecorderEvent(EventType event_type, void *event_data,
                                   size_t event_data_size) override;

  void NotifySessionEvent(EventType event_type, void *event_data,
                                  size_t event_data_size) override;

  void NotifySnapshotData(uint32_t camera_id, uint32_t imgcount,
                                  BnBuffer& buffer, BufferMeta& meta) override;

  void NotifyOfflineJpegData(int32_t buf_fd,
                                     uint32_t encoded_size) override;

  void NotifyVideoTrackData(uint32_t session_id, uint32_t track_id,
                                    std::vector<BnBuffer>& buffers,
                                    std::vector<BufferMeta>& metas) override;

  void NotifyVideoTrackEvent(uint32_t session_id, uint32_t track_id,
                                     EventType event_type,
                                     void *event_data,
                                     size_t event_data_size) override;

  void NotifyCameraResult(uint32_t camera_id, const CameraMetadata &result) override;

  // This method is not exposed to client as a callback, it is just to update
  // Internal data structure, ServiceCallbackHandler is not forced to implement
  // this method.
  void NotifyDeleteVideoTrack(uint32_t track_id) override;

  void NotifyCancelCaptureImage() override;

 private:
  void SendCallbackData(RecorderClientCallbacksAsync& message);

  uint32_t      client_id_;
  int32_t       callback_socket_;

  std::set<uint32_t> snapshot_buffers_;
  std::mutex  snapshot_buffers_lock_;
  // map <track_id , set <buffer_id> >
  std::map<uint32_t,  std::set<uint32_t> > track_buffers_map_;
  // to protect track_buffers_map_
  std::mutex  track_buffers_lock_;
};

class RecorderService : public IRecorderService {
 public:
  RecorderService();

  ~RecorderService();

  void MainLoop();
 private:
  typedef std::function <void(void)> NotifyClientDeath;
  class DeathNotifier {
   public:
    DeathNotifier(NotifyClientDeath& cb): notify_client_death_(cb){}

    void ClientDied() {
      QMMF_WARN("RecorderSerive:%s: Client Exited or Died!", __func__);
      notify_client_death_();
    }
    NotifyClientDeath notify_client_death_;
  };

  void CheckClientDeath (const uint32_t client_id);

  status_t Connect (const std::shared_ptr<IRecorderServiceCallback>&
                    service_cb,
                    uint32_t* client_id) override;
#endif // HAVE_BINDER

  friend class DeathNotifier;

  status_t Disconnect(const uint32_t client_id) override;

  status_t StartCamera(const uint32_t client_id, const uint32_t camera_id,
                       const float framerate,
                       const CameraExtraParam& extra_param,
                       bool enable_result_cb = false) override;

  status_t StopCamera(const uint32_t client_id,
                      const uint32_t camera_id) override;

  status_t CreateSession(const uint32_t client_id,
                         uint32_t *session_id) override;

  status_t DeleteSession(const uint32_t client_id,
                         const uint32_t session_id) override;

  status_t StartSession(const uint32_t client_id,
                        const uint32_t session_id) override;

  status_t StopSession(const uint32_t client_id,
                       const uint32_t session_id, bool do_flush) override;

  status_t PauseSession(const uint32_t client_id,
                        const uint32_t session_id) override;

  status_t ResumeSession(const uint32_t client_id,
                         const uint32_t session_id) override;

  status_t CreateVideoTrack(const uint32_t client_id,
                            const uint32_t session_id,
                            const uint32_t track_id,
                            const VideoTrackParam& param,
                            const VideoExtraParam& xtraparam) override;

  status_t DeleteVideoTrack(const uint32_t client_id,
                            const uint32_t session_id,
                            const uint32_t track_id) override;

  status_t ReturnTrackBuffer(const uint32_t client_id,
                             const uint32_t session_id,
                             const uint32_t track_id,
                             std::vector<BnBuffer> &buffers) override;

  status_t SetVideoTrackParam(const uint32_t client_id,
                              const uint32_t session_id,
                              const uint32_t track_id,
                              VideoParam type,
                              void *param,
                              size_t size) override;

  status_t CaptureImage(const uint32_t client_id,
                        const uint32_t camera_id,
                        const SnapshotType type,
                        const uint32_t n_images,
                        const std::vector<CameraMetadata> &meta) override;

  status_t ConfigImageCapture(const uint32_t client_id,
                              const uint32_t camera_id,
                              const uint32_t image_id,
                              const ImageParam &param,
                              const ImageExtraParam &xtraparam) override;

  status_t CancelCaptureImage(const uint32_t client_id,
                              const uint32_t camera_id,
                              const uint32_t image_id,
                              const bool cache) override;

  status_t ReturnImageCaptureBuffer(const uint32_t client_id,
                                    const uint32_t camera_id,
                                    const int32_t  buffer_id) override;

  status_t SetCameraParam(const uint32_t client_id,
                          const uint32_t camera_id,
                          const CameraMetadata &meta) override;

  status_t GetCameraParam(const uint32_t client_id,
                          const uint32_t camera_id,
                          CameraMetadata &meta) override;

  status_t SetCameraSessionParam(const uint32_t client_id,
                                 const uint32_t camera_id,
                                 const CameraMetadata &meta) override;
#ifdef VHDR_MODES_ENABLE
  status_t SetVHDR(const uint32_t client_id,
                   const uint32_t camera_id,
                   const int32_t mode) override;
#else
  status_t SetSHDR(const uint32_t client_id,
                   const uint32_t camera_id,
                   const bool enable) override;
#endif // VHDR_MODES_ENABLE

  status_t GetDefaultCaptureParam(const uint32_t client_id,
                                  const uint32_t camera_id,
                                  CameraMetadata &meta) override;

  status_t GetCameraCharacteristics(const uint32_t client_id,
                                    const uint32_t camera_id,
                                    CameraMetadata &meta) override;

  status_t CreateOfflineJPEG(const uint32_t client_id,
                             const OfflineJpegCreateParams& params) override;

  status_t EncodeOfflineJPEG(const uint32_t client_id,
                             const BnBuffer& in_buf,
                             const BnBuffer& out_buf,
                             const OfflineJpegMeta& meta) override;

  status_t DestroyOfflineJPEG(const uint32_t client_id) override;

  void ClientDeathHandler(const uint32_t client_id);

  bool IsRecorderInitialized();

  status_t DisconnectInternal(const uint32_t client_id);

  status_t GetVendorTagDescriptor(std::shared_ptr<VendorTagDescriptor> &desc) override;

  status_t GetUniqueClientID(uint32_t *client_id);

  std::unique_ptr<RecorderImpl>           recorder_;

#ifdef HAVE_BINDER
  // Map of client ids and their death notifiers.
  std::map<uint32_t, sp<DeathNotifier> >  death_notifier_list_;
  // Map of client ids and their callback handlers.
  std::map<uint32_t, sp<RemoteCallBack> > remote_cb_list_;
#else
  status_t SetupSocket ();
  void ParseRequest(int client_socket, char *recv_buffer, size_t size);
  void ProcessRequest(int client_socket, RecorderClientReqMsg msg);
  status_t SetupRemoteCallback(const uint32_t client_id);
  status_t ReadRequest (int client_socket, void *buffer, size_t size);
  status_t SendResponse (int client_socket, void *buffer, size_t size);
  // TODO: Check if unique_ptr can be used instead
  // Map of client ids and their death notifiers.
  std::map<uint32_t, std::shared_ptr<DeathNotifier>> death_notifier_list_;
  // Map of client ids and their callback handlers.
  std::map<uint32_t, std::shared_ptr<RemoteCallBack>> remote_cb_list_;
  int socket_;
  std::string socket_path_;
  char* socket_recv_buf_;
  ThreadPool thread_pool_;
  // TODO: check how to stop server properly
  bool run_;
  // Map of client sockets and their client_ids .
  std::map<int, uint32_t> client_sockets_;
#endif // HAVE_BINDER

  std::mutex                   lock_;
};

}; //namespace qmmf

}; //namespace recorder
