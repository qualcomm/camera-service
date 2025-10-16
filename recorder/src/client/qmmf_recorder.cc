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

#define LOG_TAG "Recorder"

#ifdef HAVE_BINDER
#include <binder/IPCThreadState.h>
#endif

#include "qmmf-sdk/qmmf_recorder.h"
#include "qmmf-sdk/qmmf_recorder_params.h"
#include "recorder/src/client/qmmf_recorder_client.h"
#include "recorder/src/service/qmmf_recorder_common.h"

namespace qmmf {

namespace recorder {

Recorder::Recorder()
  : recorder_client_(nullptr) {

  recorder_client_ = new RecorderClient();
  assert( recorder_client_ != NULL);
}

Recorder::~Recorder() {

  if (recorder_client_) {
    delete recorder_client_;
    recorder_client_ = nullptr;
  }
}

status_t Recorder::Connect(const RecorderCb& callback) {

  auto ret = recorder_client_->Connect(callback);
  if (0 != ret) {
    QMMF_ERROR("%s: Init failed!", __func__);
  }
  return ret;
}

status_t Recorder::Disconnect() {

  QMMF_DEBUG("%s: Enter", __func__);
  assert(recorder_client_ != nullptr);

  auto ret = recorder_client_->Disconnect();
  if (0 != ret) {
    QMMF_ERROR("%s Disconnect failed!", __func__);
  }
  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}

status_t Recorder::StartCamera(const uint32_t camera_id,
                               const float framerate,
                               const CameraExtraParam& extra_param,
                               const CameraResultCb &cb) {

  assert(recorder_client_ != NULL);

  auto ret = recorder_client_->StartCamera(camera_id, framerate,
                                           extra_param, cb);
  if (0 != ret) {
    QMMF_ERROR("%s: StartCamera failed!", __func__);
  }

  return ret;
}

status_t Recorder::StopCamera(const uint32_t camera_id) {

  assert(recorder_client_ != NULL);

  auto ret = recorder_client_->StopCamera(camera_id);
  if (0 != ret) {
    QMMF_ERROR("%s: StopCamera failed!", __func__);
  }

  return ret;
}

status_t Recorder::CreateVideoTrack(const uint32_t track_id,
                                    const VideoTrackParam& param,
                                    const VideoExtraParam& xtraparam,
                                    const TrackCb& cb) {

  assert(recorder_client_ != NULL);
  auto ret = recorder_client_->CreateVideoTrack(track_id, param,
                                                xtraparam, cb);
  if (0 != ret) {
    QMMF_ERROR("%s: CreateVideoTrackWithExtraParam failed!", __func__);
  }
  return ret;
}

status_t Recorder::DeleteVideoTrack(const uint32_t track_id) {

  assert(recorder_client_ != NULL);
  auto ret = recorder_client_->DeleteVideoTrack(track_id);
  if (0 != ret) {
    QMMF_ERROR("%s: DeleteVideoTrack failed!", __func__);
  }

  return ret;
}

status_t Recorder::StartVideoTracks(
    const std::unordered_set<uint32_t>& track_ids) {

  assert(recorder_client_ != NULL);

  auto ret = recorder_client_->StartVideoTracks(track_ids);
  if (0 != ret) {
    QMMF_ERROR("%s: StartVideoTracks failed!", __func__);
  }

  return ret;
}

status_t Recorder::StopVideoTracks(
    const std::unordered_set<uint32_t>& track_ids) {

  assert(recorder_client_ != NULL);

  auto ret = recorder_client_->StopVideoTracks(track_ids);
  if (0 != ret) {
    QMMF_ERROR("%s: StopVideoTracks failed!", __func__);
  }

  return ret;
}

status_t Recorder::ReturnTrackBuffer(const uint32_t track_id,
                                     std::vector<BufferDescriptor> &buffers) {

  assert(recorder_client_ != NULL);
  auto ret = recorder_client_->ReturnTrackBuffer(track_id, buffers);
  if (0 != ret) {
    QMMF_ERROR("%s: ReturnTrackBuffer failed!", __func__);
  }
  return ret;
}

status_t Recorder::SetVideoTrackParam(const uint32_t track_id,
                                      VideoParam type,
                                      const void *params,
                                      size_t size) {

  assert(recorder_client_ != NULL);
  auto ret = recorder_client_->SetVideoTrackParam(track_id, type, params, size);
  if (0 != ret) {
    QMMF_ERROR("%s: SetVideoTrackParam failed!", __func__);
  }
  return ret;
}

status_t Recorder::CaptureImage(const uint32_t camera_id,
                                const SnapshotType type,
                                const uint32_t n_images,
                                const std::vector<CameraMetadata> &meta,
                                const ImageCaptureCb& cb) {

  assert(recorder_client_ != NULL);
  auto ret = recorder_client_->CaptureImage(camera_id, type, n_images, meta, cb);
  if (0 != ret) {
    QMMF_ERROR("%s: CaptureImage failed!", __func__);
  }
  return ret;
}

status_t Recorder::ConfigImageCapture(const uint32_t camera_id,
                                      const uint32_t image_id,
                                      const ImageParam &param,
                                      const ImageExtraParam &xtraparam) {

  assert(recorder_client_ != NULL);
  auto ret = recorder_client_->ConfigImageCapture(camera_id, image_id, param,
                                                  xtraparam);
  if (0 != ret) {
    QMMF_ERROR("%s: ConfigImageCapture failed!", __func__);
  }
  return ret;
}

status_t Recorder::CancelCaptureImage(const uint32_t camera_id,
                                      const uint32_t image_id, bool cache) {

  assert(recorder_client_ != NULL);
  auto ret = recorder_client_->CancelCaptureImage(camera_id, image_id, cache);
  if(0 != ret) {
      QMMF_ERROR("%s: CancelCaptureImage failed!", __func__);
  }
  return ret;
}

status_t Recorder::ReturnImageCaptureBuffer(const uint32_t camera_id,
                                            const BufferDescriptor &buffer) {

  QMMF_DEBUG("%s: Enter" ,__func__);
  assert(recorder_client_ != NULL);
  auto ret = recorder_client_->ReturnImageCaptureBuffer(camera_id, buffer);
  if (0 != ret) {
    QMMF_ERROR("%s: ReturnImageCaptureBuffer failed!", __func__);
}
  QMMF_DEBUG("%s: Exit" ,__func__);
  return ret;
}

status_t Recorder::SetCameraParam(const uint32_t camera_id,
                                  const CameraMetadata &meta) {

  QMMF_DEBUG("%s: Enter" ,__func__);
  assert(recorder_client_ != NULL);
  auto ret = recorder_client_->SetCameraParam(camera_id, meta);
  if (0 != ret) {
      QMMF_ERROR("%s: SetCameraParam failed!", __func__);
  }
  QMMF_DEBUG("%s: Exit" ,__func__);
  return ret;
}

status_t Recorder::GetCameraParam(const uint32_t camera_id,
                                  CameraMetadata &meta) {

  QMMF_DEBUG("%s: Enter" ,__func__);
  assert(recorder_client_ != NULL);
  auto ret = recorder_client_->GetCameraParam(camera_id, meta);
  if (0 != ret) {
      QMMF_ERROR("%s: GetCameraParam failed!", __func__);
  }

  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}

status_t Recorder::SetCameraSessionParam(const uint32_t camera_id,
                                         const CameraMetadata &meta) {

  QMMF_DEBUG("%s: Enter" ,__func__);
  assert(recorder_client_ != NULL);
  auto ret = recorder_client_->SetCameraSessionParam(camera_id, meta);
  if (0 != ret) {
      QMMF_ERROR("%s: SetCameraSessionParam failed!", __func__);
  }
  QMMF_DEBUG("%s: Exit" ,__func__);
  return ret;
}

#ifdef VHDR_MODES_ENABLE
status_t Recorder::SetVHDR(const uint32_t camera_id,
                           const int32_t mode) {

  QMMF_DEBUG("%s: Enter" ,__func__);
  assert(recorder_client_ != NULL);
  auto ret = recorder_client_->SetVHDR(camera_id, mode);
  if (0 != ret) {
      QMMF_ERROR("%s: SetVHDR failed!", __func__);
  }

  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}
#else
status_t Recorder::SetSHDR(const uint32_t camera_id,
                           const bool enable) {

  QMMF_DEBUG("%s: Enter" ,__func__);
  assert(recorder_client_ != NULL);
  auto ret = recorder_client_->SetSHDR(camera_id, enable);
  if (0 != ret) {
      QMMF_ERROR("%s: SetSHDR failed!", __func__);
  }

  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}
#endif // VHDR_MODES_ENABLE

status_t Recorder::GetDefaultCaptureParam(const uint32_t camera_id,
                                          CameraMetadata &meta) {

  QMMF_DEBUG("%s: Enter" ,__func__);
  assert(recorder_client_ != NULL);
  auto ret = recorder_client_->GetDefaultCaptureParam(camera_id, meta);
  if (0 != ret) {
      QMMF_ERROR("%s: GetDefaultCaptureParam failed!", __func__);
  }

  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}

status_t Recorder::GetCamStaticInfo(std::vector<CameraMetadata> &meta) {

  QMMF_DEBUG("%s: Enter" ,__func__);
  assert(recorder_client_ != NULL);
  auto ret = recorder_client_->GetCamStaticInfo(meta);
  if (0 != ret) {
      QMMF_ERROR("%s: GetCamStaticInfo failed!", __func__);
  }

  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}

status_t Recorder::GetCameraCharacteristics(const uint32_t camera_id,
                                            CameraMetadata &meta) {

  QMMF_DEBUG("%s: Enter" ,__func__);
  assert(recorder_client_ != NULL);
  auto ret = recorder_client_->GetCameraCharacteristics(camera_id, meta);
  if (0 != ret) {
      QMMF_ERROR("%s: GetCameraCharacteristics failed!", __func__);
  }

  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}

status_t Recorder::CreateOfflineJPEG(
                    const OfflineJpegCreateParams& params,
                    const OfflineJpegCb &cb) {

  QMMF_DEBUG("%s: Enter" ,__func__);
  assert(recorder_client_ != NULL);
  OfflineCameraCreateParams offline_params;
  offline_params.process_mode = params.process_mode;
  offline_params.camera_id[0] = params.camera_id;
  offline_params.in_buffer.width = params.in_buffer.width;
  offline_params.in_buffer.height = params.in_buffer.height;
  offline_params.in_buffer.format = params.in_buffer.format;

  offline_params.out_buffer.width = params.out_buffer.width;
  offline_params.out_buffer.height = params.out_buffer.height;
  offline_params.out_buffer.format = params.out_buffer.format;

  OfflineCameraCb offline_cb = cb;
  auto ret = recorder_client_->CreateOfflineProcess(offline_params, offline_cb);
  if (0 != ret) {
    QMMF_ERROR("%s: CreateOfflineProcess failed!", __func__);
  }
  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}

status_t Recorder::EncodeOfflineJPEG(const OfflineJpegProcessParams& params) {
  QMMF_DEBUG("%s: Enter" ,__func__);
  assert(recorder_client_ != NULL);
  OfflineCameraProcessParams proc_params;
  CameraMetadata meta(1, 128);
  meta.update(ANDROID_JPEG_QUALITY, (uint8_t *)(&params.metadata.quality), 1);
  proc_params.meta = meta;
  proc_params.in_buf_fd[0] = params.in_buf_fd;
  proc_params.in_buf_fd[1] = -1;
  proc_params.out_buf_fd = params.out_buf_fd;

  auto ret = recorder_client_->ProcOfflineProcess(proc_params);
  if (0 != ret) {
    QMMF_ERROR("%s: ProcOfflineProcess failed!", __func__);
  }

  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}

status_t Recorder::DestroyOfflineJPEG() {
  QMMF_DEBUG("%s: Enter" ,__func__);
  assert(recorder_client_ != NULL);
  auto ret = recorder_client_->DestroyOfflineProcess();
  if (0 != ret) {
    QMMF_ERROR("%s: DestroyOfflineProcess failed!", __func__);
  }
  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}

status_t Recorder::CreateOfflineCamera(const OfflineCameraCreateParams &params,
                              const OfflineCameraCb &cb) {
  QMMF_INFO("%s: Enter" ,__func__);
  assert(recorder_client_ != NULL);
  auto ret = recorder_client_->CreateOfflineProcess(params, cb);
  if (ret != 0) {
    QMMF_ERROR("%s: CreateOfflineProcess failed!", __func__);
  }
  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

status_t Recorder::ProcessOfflineCamera(const OfflineCameraProcessParams &params) {
  QMMF_INFO("%s: Enter" ,__func__);
  assert(recorder_client_ != NULL);
  auto ret = recorder_client_->ProcOfflineProcess(params);
  if (ret != 0) {
    QMMF_ERROR("%s: ProcOfflineProcess failed", __func__);
  }

  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

status_t Recorder::DestroyOfflineCamera() {
  QMMF_INFO("%s: Enter" ,__func__);
  assert(recorder_client_ != NULL);
  auto ret = recorder_client_->DestroyOfflineProcess();
  if (ret != 0) {
    QMMF_ERROR("%s: DestroyOfflineProcess failed!", __func__);
  }

  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

}; //namespace recoder.

}; //namespace qmmf.
