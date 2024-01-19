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

#include <iomanip>
#include <sstream>
#include <string>

#include <unistd.h>

#include <binder/IBinder.h>
#include <binder/IServiceManager.h>
#include <binder/Parcel.h>

#include "qmmf-sdk/qmmf_camera_metadata.h"
#include "qmmf-sdk/qmmf_recorder_params.h"
#include "qmmf-sdk/qmmf_recorder_extra_param.h"
#include "qmmf-sdk/qmmf_offline_jpeg_params.h"

namespace qmmf {
namespace recorder {

using namespace android;
using ::std::setbase;
using ::std::string;
using ::std::stringstream;

#define QMMF_RECORDER_SERVICE_NAME "qmmf_recorder.service"

enum QMMF_RECORDER_SERVICE_CMDS {
  RECORDER_CONNECT = IBinder::FIRST_CALL_TRANSACTION,
  RECORDER_DISCONNECT,
  RECORDER_START_CAMERA,
  RECORDER_STOP_CAMERA,
  RECORDER_CREATE_SESSION,
  RECORDER_DELETE_SESSION,
  RECORDER_START_SESSION,
  RECORDER_STOP_SESSION,
  RECORDER_PAUSE_SESSION,
  RECORDER_RESUME_SESSION,
  RECORDER_GET_NUMBER_OF_CAMERAS,
  RECORDER_CREATE_VIDEOTRACK,
  RECORDER_DELETE_VIDEOTRACK,
  RECORDER_RETURN_TRACKBUFFER,
  RECORDER_SET_VIDEOTRACK_PARAMS,
  RECORDER_CAPTURE_IMAGE,
  RECORDER_CONFIG_IMAGECAPTURE,
  RECORDER_CANCEL_IMAGECAPTURE,
  RECORDER_RETURN_IMAGECAPTURE_BUFFER,
  RECORDER_SET_CAMERA_PARAMS,
  RECORDER_GET_CAMERA_PARAMS,
  RECORDER_SET_CAMERA_SESSION_PARAMS,
  RECORDER_SET_SHDR,
  RECORDER_GET_DEFAULT_CAPTURE_PARAMS,
  RECORDER_GET_CAMERA_CHARACTERISTICS,
  RECORDER_GET_VENDOR_TAG_DESCRIPTOR,
  RECORDER_CONFIGURE_OFFLINE_JPEG,
  RECORDER_ENCODE_OFFLINE_JPEG,
  RECORDER_DESTROY_OFFLINE_JPEG,
};

struct BnBuffer {
  int32_t   ion_fd;
  int32_t   ion_meta_fd;
  int32_t   img_id;
  uint32_t  size;
  uint64_t  timestamp;
  uint64_t  seqnum;
  uint32_t  buffer_id;
  uint64_t  flags;
  uint32_t  capacity;

  string ToString() const {
    stringstream stream;
    stream << "ion_fd[" << ion_fd << "] ";
    stream << "ion_meta_fd[" << ion_meta_fd << "] ";
    stream << "img_id[" << img_id << "] ";
    stream << "size[" << size << "] ";
    stream << "timestamp[" << timestamp << "] ";
    stream << "seqnum[" << seqnum << "] ";
    stream << "buffer_id[" << buffer_id << "] ";
    stream << "flag[" << setbase(16) << flags << setbase(10) << "] ";
    stream << "capacity[" << capacity << "]";
    return stream.str();
  }

  void ToParcel(Parcel* parcel, bool writeFileDescriptor) const {
    if (writeFileDescriptor) {
      if (ion_meta_fd == -1) {
        parcel->writeUint32(1);
        parcel->writeFileDescriptor(ion_fd);
      } else {
        parcel->writeUint32(2);
        parcel->writeFileDescriptor(ion_fd);
        parcel->writeFileDescriptor(ion_meta_fd);
      }
    } else {
      parcel->writeUint32(ion_fd);
      parcel->writeUint32(ion_meta_fd);
    }
    parcel->writeUint32(size);
    parcel->writeInt64(timestamp);
    parcel->writeInt64(seqnum);
    parcel->writeUint32(buffer_id);
    parcel->writeUint32(flags);
    parcel->writeUint32(capacity);
  }

  void FromParcel(const Parcel& parcel, bool readFileDescriptor) {
    if (readFileDescriptor) {
      uint32_t num_fds = parcel.readUint32();
      if (num_fds == 1) {
        ion_fd = dup(parcel.readFileDescriptor());
        ion_meta_fd = -1;
      } else {
        ion_fd = dup(parcel.readFileDescriptor());
        ion_meta_fd = dup(parcel.readFileDescriptor());
      }
    } else {
      ion_fd = parcel.readUint32();
      ion_meta_fd = parcel.readUint32();
    }
    size = parcel.readUint32();
    timestamp = parcel.readInt64();
    seqnum = parcel.readInt64();
    buffer_id = parcel.readUint32();
    flags = parcel.readUint32();
    capacity = parcel.readUint32();
  }
};

class IRecorderServiceCallback;
class IRecorderService : public IInterface {
 public:
  DECLARE_META_INTERFACE(RecorderService);

  virtual status_t Connect(const sp<IRecorderServiceCallback>& service_cb,
                           uint32_t* client_id) = 0;

  virtual status_t Disconnect(const uint32_t client_id) = 0;

  virtual status_t StartCamera(const uint32_t client_id,
                               const uint32_t camera_id,
                               const float framerate,
                               const CameraExtraParam& extra_param,
                               bool enable_result_cb = false) = 0;

  virtual status_t StopCamera(const uint32_t client_id,
                              const uint32_t camera_id) = 0;

  virtual status_t CreateSession(const uint32_t client_id,
                                 uint32_t *session_id) = 0;

  virtual status_t DeleteSession(const uint32_t client_id,
                                 const uint32_t session_id) = 0;

  virtual status_t StartSession(const uint32_t client_id,
                                const uint32_t session_id) = 0;

  virtual status_t StopSession(const uint32_t client_id,
                               const uint32_t session_id, bool do_flush) = 0;

  virtual status_t PauseSession(const uint32_t client_id,
                                const uint32_t session_id) = 0;

  virtual status_t ResumeSession(const uint32_t client_id,
                                 const uint32_t session_id) = 0;

  virtual status_t CreateVideoTrack(const uint32_t client_id,
                                    const uint32_t session_id,
                                    const uint32_t track_id,
                                    const VideoTrackParam& param,
                                    const VideoExtraParam& xtraparam) = 0;

  virtual status_t DeleteVideoTrack(const uint32_t client_id,
                                    const uint32_t session_id,
                                    const uint32_t track_id) = 0;

  virtual status_t ReturnTrackBuffer(const uint32_t client_id,
                                     const uint32_t session_id,
                                     const uint32_t track_id,
                                     std::vector<BnBuffer> &buffers) = 0;

  virtual status_t SetVideoTrackParam(const uint32_t client_id,
                                      const uint32_t session_id,
                                      const uint32_t track_id,
                                      VideoParam type,
                                      void *param,
                                      size_t size) = 0;

  virtual status_t CaptureImage(const uint32_t client_id,
                                const uint32_t camera_id,
                                const SnapshotType type,
                                const uint32_t n_images,
                                const std::vector<CameraMetadata> &meta) = 0;

  virtual status_t ConfigImageCapture(const uint32_t client_id,
                                      const uint32_t camera_id,
                                      const uint32_t image_id,
                                      const ImageParam &param,
                                      const ImageExtraParam &xtraparam) = 0;

  virtual status_t CancelCaptureImage(const uint32_t client_id,
                                      const uint32_t camera_id,
                                      const uint32_t image_id,
                                      const bool cache) = 0;

  virtual status_t ReturnImageCaptureBuffer(const uint32_t client_id,
                                            const uint32_t camera_id,
                                            const int32_t buffer_id) = 0;

  virtual status_t SetCameraParam(const uint32_t client_id,
                                  const uint32_t camera_id,
                                  const CameraMetadata &meta) = 0;

  virtual status_t GetCameraParam(const uint32_t client_id,
                                  const uint32_t camera_id,
                                  CameraMetadata &meta) = 0;

  virtual status_t SetCameraSessionParam(const uint32_t client_id,
                                         const uint32_t camera_id,
                                         const CameraMetadata &meta) = 0;

  virtual status_t SetSHDR(const uint32_t client_id,
                           const uint32_t camera_id,
                           const bool enable) = 0;

  virtual status_t GetDefaultCaptureParam(const uint32_t client_id,
                                          const uint32_t camera_id,
                                          CameraMetadata &meta) = 0;

  virtual status_t GetCameraCharacteristics(const uint32_t client_id,
                                            const uint32_t camera_id,
                                            CameraMetadata &meta) = 0;

  virtual status_t GetVendorTagDescriptor(std::shared_ptr<VendorTagDescriptor> &desc) = 0;

  virtual status_t CreateOfflineJPEG(
                                const uint32_t client_id,
                                const OfflineJpegCreateParams& params) = 0;

  virtual status_t EncodeOfflineJPEG(const uint32_t client_id,
                                     const BnBuffer& in_buf,
                                     const BnBuffer& out_buf,
                                     const OfflineJpegMeta& meta) = 0;

  virtual status_t DestroyOfflineJPEG(const uint32_t client_id) = 0;
};

enum RECORDER_SERVICE_CB_CMDS{
  RECORDER_NOTIFY_EVENT=IBinder::FIRST_CALL_TRANSACTION,
  RECORDER_NOTIFY_SESSION_EVENT,
  RECORDER_NOTIFY_SNAPSHOT_DATA,
  RECORDER_NOTIFY_OFFLINE_JPEG_DATA,
  RECORDER_NOTIFY_VIDEO_TRACK_DATA,
  RECORDER_NOTIFY_VIDEO_TRACK_EVENT,
  RECORDER_NOTIFY_CAMERA_RESULT,
};

//Binder interface for callbacks from RecorderService to RecorderClient.
class IRecorderServiceCallback : public IInterface {
 public:
  DECLARE_META_INTERFACE(RecorderServiceCallback);

  virtual void NotifyRecorderEvent(EventType event_type, void *event_data,
                                   size_t event_data_size) = 0;

  virtual void NotifySessionEvent(EventType event_type, void *event_data,
                                  size_t event_data_size) = 0;

  virtual void NotifySnapshotData(uint32_t camera_id, uint32_t imgcount,
                                  BnBuffer& buffer, BufferMeta& meta) = 0;

  virtual void NotifyOfflineJpegData(int32_t buf_fd,
                                     uint32_t encoded_size) = 0;

  virtual void NotifyVideoTrackData(uint32_t session_id, uint32_t track_id,
                                    std::vector<BnBuffer>& buffers,
                                    std::vector<BufferMeta>& metas) = 0;

  virtual void NotifyVideoTrackEvent(uint32_t session_id, uint32_t track_id,
                                     EventType event_type,
                                     void *event_data,
                                     size_t event_data_size) = 0;

  virtual void NotifyCameraResult(uint32_t camera_id,
                                  const CameraMetadata &result) = 0;

  // This method is not exposed to client as a callback, it is just to update
  // Internal data structure, ServiceCallbackHandler is not forced to implement
  // this method.
  virtual void NotifyDeleteVideoTrack(uint32_t track_id
      __attribute__((__unused__))) {}
};

//This class is responsible to provide callbacks from recoder service.
class BnRecorderServiceCallback : public BnInterface<IRecorderServiceCallback> {
 public:
  virtual status_t onTransact(uint32_t code, const Parcel& data,
                              Parcel* reply, uint32_t flags = 0) override;
};

}; //namespace recorder

}; //namespace qmmf
