/*
* Copyright (c) 2016, 2020-2021, The Linux Foundation. All rights reserved.
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
* Copyright (c) 2021-2023 Qualcomm Innovation Center, Inc. All rights reserved.
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

#include <vector>
#include <set>
#include <map>
#include <mutex>

#include "qmmf-sdk/qmmf_camera_metadata.h"
#include <qmmf-sdk/qmmf_recorder_params.h>
#include <qmmf-sdk/qmmf_recorder_extra_param.h>

#include <linux/dma-buf.h>
#ifdef TARGET_USES_GBM
#include <gbm.h>
#include <gbm_priv.h>
#endif

#include "common/utils/qmmf_log.h"
#include "recorder/src/client/qmmf_recorder_service_intf.h"

namespace qmmf {

namespace recorder {

using namespace android;

class RecorderClient {
 public:
  RecorderClient();

  ~RecorderClient();

  status_t Connect(const RecorderCb& cb);

  status_t Disconnect();

  status_t StartCamera(const uint32_t camera_id,
                       const float framerate,
                       const CameraExtraParam& extra_param,
                       const CameraResultCb &result_cb = nullptr);

  status_t StopCamera(const uint32_t camera_id);

  status_t CreateSession(const SessionCb& cb, uint32_t* session_id);

  status_t DeleteSession(const uint32_t session_id);

  status_t StartSession(const uint32_t session_id);

  status_t StopSession(const uint32_t session_id, bool do_flush);

  status_t PauseSession(const uint32_t session_id);

  status_t ResumeSession(const uint32_t session_id);

  status_t CreateVideoTrack(const uint32_t session_id, const uint32_t track_id,
                            const VideoTrackParam& param,
                            const VideoExtraParam& xtraparam,
                            const TrackCb& cb);

  status_t ReturnTrackBuffer(const uint32_t session_id,
                             const uint32_t track_id,
                             std::vector<BufferDescriptor> &buffers);

  status_t SetVideoTrackParam(const uint32_t session_id,
                              const uint32_t track_id,
                              VideoParam type, const void *param,
                              size_t size);

  status_t DeleteVideoTrack(const uint32_t session_id,
                            const uint32_t track_id);

  status_t CaptureImage(const uint32_t camera_id,
                        const SnapshotType type,
                        const uint32_t n_images,
                        const std::vector<CameraMetadata> &meta,
                        const ImageCaptureCb &cb);

  status_t ConfigImageCapture(const uint32_t camera_id,
                              const uint32_t image_id,
                              const ImageParam &param,
                              const ImageExtraParam &xtraparam);

  status_t CancelCaptureImage(const uint32_t camera_id,
                              const uint32_t image_id,
                              const bool cache);

  status_t ReturnImageCaptureBuffer(const uint32_t camera_id,
                                    const BufferDescriptor &buffer);

  status_t SetCameraParam(const uint32_t camera_id, const CameraMetadata &meta);

  status_t GetCameraParam(const uint32_t camera_id, CameraMetadata &meta);

  status_t SetCameraSessionParam(const uint32_t camera_id, const CameraMetadata &meta);

  status_t SetSHDR(const uint32_t camera_id, const bool enable);

  status_t GetDefaultCaptureParam(const uint32_t camera_id,
                                  CameraMetadata &meta);

  status_t GetCameraCharacteristics(const uint32_t camera_id,
                                    CameraMetadata &meta);

  status_t GetVendorTagDescriptor(std::shared_ptr<VendorTagDescriptor> &desc);

  status_t CreateOfflineJPEG(const OfflineJpegCreateParams &params,
                             const OfflineJpegCb &cb);

  status_t EncodeOfflineJPEG(const OfflineJpegProcessParams &params);

  status_t DestroyOfflineJPEG();

  // Callback handlers from service.ap
  void NotifyRecorderEvent(EventType event_type, void *event_data,
                           size_t event_data_size);

  void NotifySessionEvent(EventType event_type, void *event_data,
                          size_t event_data_size);

  void NotifySnapshotData(uint32_t camera_id, uint32_t imgcount,
                          BnBuffer& buffer, BufferMeta& meta);

  void NotifyOfflineJpegData(int32_t buf_fd, uint32_t encoded_size);

  void NotifyVideoTrackData(uint32_t session_id, uint32_t track_id,
                            std::vector<BnBuffer>& bn_buffers,
                            std::vector<BufferMeta>& metas);

  void NotifyVideoTrackEvent(uint32_t session_id, uint32_t track_id,
                             EventType event_type,
                             void *event_data,
                             size_t event_data_size);

  void NotifyCameraResult(uint32_t camera_id,
                          const CameraMetadata &result);

 private:
  typedef std::function <void(void)> NotifyServerDeathCB;
#ifdef HAVE_BINDER
  class DeathNotifier : public IBinder::DeathRecipient {
   public:
    DeathNotifier(NotifyServerDeathCB& cb) : notify_server_death_(cb) {}

    void binderDied(const wp<IBinder>&) override {
      QMMF_DEBUG("RecorderClient:%s: Recorder service died", __func__);
      notify_server_death_();
    }
    NotifyServerDeathCB notify_server_death_;
  };
#else
  class DeathNotifier {
   public:
    DeathNotifier(NotifyServerDeathCB& cb) : notify_server_death_(cb) {}

    void ServerDied() {
      QMMF_DEBUG("RecorderClient:%s: Recorder service died", __func__);
      notify_server_death_();
    }

   private:
    NotifyServerDeathCB notify_server_death_;
  };
#endif // HAVE_BINDER

  struct BufferInfo {
    int32_t ion_fd;      // Transferred ION Id.
    int32_t ion_meta_fd; // Transferred ION metadata Id.
    size_t  size;        // Buffer length/size.
    void*   vaddr;       // Memory mapped buffer.
  };

  // Map <buffer index, buffer info>
  typedef std::map<uint32_t, BufferInfo> BufferInfoMap;

#ifdef TARGET_USES_GBM
  void ImportBuffer(int32_t fd, int32_t metafd, const BufferMeta& meta);
  void ReleaseBuffer(int32_t& fd, int32_t& meta_fd);
#endif

  status_t MapBuffer(BufferInfo& info, const BufferMeta& meta);
  void UnmapBuffer(BufferInfo& info);

  void UpdateSessionTopology(const uint32_t& session_id,
                             const uint32_t& track_id,
                             bool /*Add or Delete*/);

  bool CheckServiceStatus();

  void ServiceDeathHandler();

  bool IsJpegBufPresent(const int32_t& buf_fd);

#ifdef HAVE_BINDER
  sp<IRecorderService>              recorder_service_;
  sp<DeathNotifier>                 death_notifier_;
#else
  std::unique_ptr<IRecorderService> recorder_service_;
  std::unique_ptr<DeathNotifier>    death_notifier_;
#endif // HAVE_BINDER
  int32_t                           ion_device_;
  uint32_t                          client_id_;

  // List track IDs in a session.
  std::map<uint32_t, std::set<uint32_t> > sessions_;

  // List of session callbacks.
  std::map<uint32_t, SessionCb >    session_cb_list_;

  // List of track callbacks.
  std::map<uint32_t, std::map<uint32_t, TrackCb> > track_cb_list_;
  std::mutex                        track_cb_lock_;

  RecorderCb                        recorder_cb_;
  ImageCaptureCb                    image_capture_cb_;
  CameraResultCb                    metadata_cb_;
  OfflineJpegCb                     offline_jpeg_cb_;

  std::vector<int32_t>              offline_jpeg_buffers_;

  // List of information regarding the buffers in a track.
  std::map<uint32_t, BufferInfoMap> track_buffers_map_;
  std::mutex                        track_buffers_lock_;

  // List of information regarding the buffers for image capture.
  BufferInfoMap                     snapshot_buffers_;
  std::mutex                        snapshot_buffers_lock_;

#ifdef TARGET_USES_GBM
  int32_t                           gbm_fd_;
  gbm_device*                       gbm_device_;

  std::map<int32_t, gbm_bo*>        gbm_buffers_map_;
  std::mutex                        gbm_lock_;
#endif

  // VendorTagDescriptor
  std::shared_ptr<VendorTagDescriptor>           vendor_tag_desc_;

  // Global mutex.
  std::mutex                        lock_;
};

class ServiceCallbackHandler : public RecorderServiceCallbackStub {
 public:

  ServiceCallbackHandler(RecorderClient* client);

  ~ServiceCallbackHandler();

 private:
  //Methods of BnRecorderServiceCallback.
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

  void NotifyCameraResult(uint32_t camera_id,
                          const CameraMetadata &result) override;

  RecorderClient *client_;
};


}; // namespace qmmf

}; // namespace recorder.
