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

#define LOG_TAG "RecorderCameraSource"

#include <cmath>
#include <fcntl.h>
#include <dirent.h>
#include <sys/mman.h>
#include <sys/time.h>
#ifndef CAMERA_HAL1_SUPPORT
#include <hardware/camera3.h>
#endif

#include "recorder/src/service/qmmf_camera_source.h"
#include "recorder/src/service/qmmf_recorder_common.h"
#include "recorder/src/service/qmmf_recorder_utils.h"

#ifndef JPEG_BLOB_OFFSET
#define JPEG_BLOB_OFFSET (1)
#endif

namespace qmmf {

namespace recorder {

using ::std::make_shared;
using ::std::shared_ptr;

static const nsecs_t kWaitDuration = 5000000000; // 5 s.
static const uint64_t kTsFactor = 10000000; // 10 ms.

CameraSource* CameraSource::instance_ = nullptr;

CameraSource* CameraSource::CreateCameraSource() {

  if (!instance_) {
    instance_ = new CameraSource;
    if (!instance_) {
      QMMF_ERROR("%s: Can't Create CameraSource Instance", __func__);
      //return nullptr;
    }
  }
  QMMF_INFO("%s: CameraSource Instance Created Successfully(%p)",
      __func__, instance_);
  return instance_;
}

CameraSource::CameraSource()
    : frame_rate_control_(true) {

  QMMF_GET_LOG_LEVEL();
  QMMF_KPI_GET_MASK();
  QMMF_KPI_DETAIL();

  QMMF_INFO("%s: Enter", __func__);

  int32_t n_preload = Property::Get("persist.qmmf.preload.cameras", 0);

  // Preload camera interefaces.
  for (int32_t idx = 0; idx < n_preload; ++idx) {
    std::shared_ptr<CameraInterface> camera;

    if (!(camera = std::make_shared<CameraContext>())) {
      QMMF_WARN("%s: Can't Instantiate camera interface!", __func__);
      continue;
    }
    preloaded_cameras_.push_back(std::move(camera));
  }

  QMMF_INFO("%s: Exit", __func__);
}

CameraSource::~CameraSource() {

  QMMF_KPI_DETAIL();
  QMMF_INFO("%s: Enter", __func__);
  instance_ = nullptr;
  QMMF_INFO("%s: Exit (0x%p)", __func__, this);
}

status_t CameraSource::StartCamera(const uint32_t camera_id,
                                   const float framerate,
                                   const CameraExtraParam& extra_param,
                                   const ResultCb &cb,
                                   const ErrorCb &errcb) {

  QMMF_INFO("%s: Camera Id(%u) to open!", __func__, camera_id);
  QMMF_KPI_DETAIL();
  std::shared_ptr<CameraInterface> camera;

  if (!preloaded_cameras_.empty()) {
    camera = preloaded_cameras_.front();
    preloaded_cameras_.pop_front();
  } else {
    camera = std::make_shared<CameraContext>();
  }

  if (!camera) {
    QMMF_ERROR("%s: Can't Instantiate Camera(%d)!", __func__, camera_id);
    return NO_MEMORY;
  }

  // Add contexts to map when in regular camera case.
  active_cameras_lock_.lock();
  if (active_cameras_.count(camera_id) != 0) {
    active_cameras_lock_.unlock();
    QMMF_INFO("%s: Camera(%u) is already open!", __func__, camera_id);
    return NO_ERROR;
  }
  active_cameras_.emplace(camera_id, camera);
  active_cameras_lock_.unlock();

  // This is required to send it to rescaler to take decision on UBWC.
  start_cam_param_lock_.lock();
  start_cam_param_[camera_id] = extra_param;
  start_cam_param_lock_.unlock();

  auto ret = camera->OpenCamera(camera_id, framerate, extra_param, cb, errcb);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: OpenCamera(%d) Failed!", __func__, camera_id);
    active_cameras_lock_.lock();
    active_cameras_.erase(camera_id);
    active_cameras_lock_.unlock();
    return ret;
  }

  if (extra_param.Exists(QMMF_FRAME_RATE_CONTROL)) {
    size_t entry_count = extra_param.EntryCount(QMMF_FRAME_RATE_CONTROL);
    if (entry_count == 1) {
      FrameRateControl frc_mode;
      extra_param.Fetch(QMMF_FRAME_RATE_CONTROL, frc_mode, 0);
      if (frc_mode.mode == FrameRateControlMode::kCaptureRequest) {
        // Stream frame rate will be control by PCR
        QMMF_INFO("%s: PCR FRC enable", __func__);
        frame_rate_control_ = false;
      }
    } else {
      QMMF_ERROR("%s: Invalid FRC mode received", __func__);
      return BAD_VALUE;
    }
  }

  QMMF_INFO("%s: Camera(%d) opened successfully!", __func__, camera_id);
  return NO_ERROR;
}

status_t CameraSource::StopCamera(const uint32_t camera_id) {

  QMMF_INFO("%s: Camera(%u) to close!", __func__, camera_id);
  QMMF_KPI_DETAIL();

  //TODO: check if streams are still active, flush them before closing camera.

  active_cameras_lock_.lock();
  if (active_cameras_.count(camera_id) == 0) {
    active_cameras_lock_.unlock();
    QMMF_ERROR("%s: Invalid Camera Id(%d)", __func__, camera_id);
    return BAD_VALUE;
  }

  auto const& camera = active_cameras_[camera_id];
  active_cameras_lock_.unlock();

  auto ret = camera->CloseCamera(camera_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: Failed to close camera(%d)!", __func__, camera_id);
    return FAILED_TRANSACTION;
  }

  active_cameras_lock_.lock();
  active_cameras_.erase(camera_id);
  active_cameras_lock_.unlock();
  QMMF_INFO("%s: Camera(%d) successfully closed!", __func__, camera_id);

  return NO_ERROR;
}

status_t CameraSource::CaptureImage(const uint32_t camera_id,
                                    const SnapshotType type,
                                    const uint32_t n_images,
                                    const std::vector<::camera::CameraMetadata> &meta,
                                    const SnapshotCb& cb) {

  QMMF_DEBUG("%s: Enter", __func__);
  QMMF_KPI_DETAIL();

  active_cameras_lock_.lock();
  if (active_cameras_.count(camera_id) == 0) {
    active_cameras_lock_.unlock();
    QMMF_ERROR("%s: Invalid Camera Id(%d)", __func__, camera_id);
    return BAD_VALUE;
  }
  auto const& camera = active_cameras_[camera_id];
  active_cameras_lock_.unlock();

  client_snapshot_cb_ = cb;
  StreamSnapshotCb stream_cb = [&] (uint32_t image_id, uint32_t count,
      StreamBuffer& buf) {
    SnapshotCallback(image_id, count, buf);
  };
  auto ret = camera->CaptureImage(type, n_images, meta, stream_cb);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: CaptureImage Failed!", __func__);
    return ret;
  }

  QMMF_DEBUG("%s: Exit", __func__);
  return NO_ERROR;
}

status_t CameraSource::ConfigImageCapture(const uint32_t camera_id,
                                          const uint32_t image_id,
                                          const ImageParam &param,
                                          const ImageExtraParam &xtraparam) {

  QMMF_DEBUG("%s: Enter", __func__);

  active_cameras_lock_.lock();
  if (active_cameras_.count(camera_id) == 0) {
    active_cameras_lock_.unlock();
    QMMF_ERROR("%s: Invalid Camera Id(%d)", __func__, camera_id);
    return BAD_VALUE;
  }
  auto const& camera = active_cameras_[camera_id];
  active_cameras_lock_.unlock();

  SnapshotParam sparam {};
  sparam.mode    = param.mode;
  sparam.width   = param.width;
  sparam.height  = param.height;
  sparam.format  = Common::FromImageToQmmfFormat(param.format);
  sparam.quality = param.quality;
  sparam.rotation = param.rotation;

  auto ret = camera->ConfigImageCapture(image_id, sparam, xtraparam);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: ConfigImageCapture Failed!", __func__);
    return ret;
  }

  QMMF_DEBUG("%s: Exit", __func__);
  return NO_ERROR;
}

status_t CameraSource::CancelCaptureImage(const uint32_t camera_id,
                                          const uint32_t image_id,
                                          const bool cache) {

  QMMF_DEBUG("%s: Enter", __func__);
  QMMF_KPI_DETAIL();

  active_cameras_lock_.lock();
  if (active_cameras_.count(camera_id) == 0) {
    active_cameras_lock_.unlock();
    QMMF_ERROR("%s: Invalid Camera Id(%d)", __func__, camera_id);
    return BAD_VALUE;
  }
  auto const& camera = active_cameras_[camera_id];
  active_cameras_lock_.unlock();

  auto ret = camera->CancelCaptureImage(image_id, cache);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: CancelCaptureImage Failed!", __func__);
    return ret;
  }
  QMMF_DEBUG("%s: Exit", __func__);
  return NO_ERROR;
}

status_t CameraSource::ReturnAllImageCaptureBuffers(const uint32_t camera_id) {
  QMMF_DEBUG("%s: Enter", __func__);

  active_cameras_lock_.lock();
  if (active_cameras_.count(camera_id) == 0) {
    active_cameras_lock_.unlock();
    QMMF_ERROR("%s: Invalid Camera Id(%d)", __func__, camera_id);
    return BAD_VALUE;
  }
  auto const& camera = active_cameras_[camera_id];
  active_cameras_lock_.unlock();

  auto ret = camera->ReturnAllImageCaptureBuffers();
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: ReturnAllImageCaptureBuffers Failed!", __func__);
    return ret;
  }

  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}

status_t CameraSource::ReturnImageCaptureBuffer(const uint32_t camera_id,
                                                const int32_t buffer_id) {
  QMMF_DEBUG("%s: Enter", __func__);

  active_cameras_lock_.lock();
  if (active_cameras_.count(camera_id) == 0) {
    active_cameras_lock_.unlock();
    QMMF_ERROR("%s: Invalid Camera Id(%d)", __func__, camera_id);
    return BAD_VALUE;
  }
  auto const& camera = active_cameras_[camera_id];
  active_cameras_lock_.unlock();

  auto ret = camera->ReturnImageCaptureBuffer(camera_id, buffer_id);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: ReturnImageCaptureBuffer Failed!", __func__);
    return ret;
  }

  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}

int32_t CameraSource::GetSourceTrackId(const VideoExtraParam& extra_param) {

  if (extra_param.Exists(QMMF_SOURCE_VIDEO_TRACK_ID)) {
    SourceVideoTrack source_track;
    extra_param.Fetch(QMMF_SOURCE_VIDEO_TRACK_ID, source_track);
    return source_track.source_track_id;
  }
  return NAME_NOT_FOUND;
}

bool CameraSource::ValidateSlaveTrackParam(
    const VideoTrackParam& slave_params,
    const VideoTrackParam& master_params) {

  QMMF_DEBUG("%s %d x %d -> %d x %d fmt 0x%x -> 0x%x", __func__,
      master_params.width, master_params.height,
      slave_params.width, slave_params.height,
      master_params.format, slave_params.format);

  if ((slave_params.format != VideoFormat::kNV12) &&
      (slave_params.format != VideoFormat::kNV12UBWC) &&
      (slave_params.format != VideoFormat::kNV16) &&
      (slave_params.format != VideoFormat::kRGB) &&
      (master_params.format != VideoFormat::kNV12) &&
      (master_params.format != VideoFormat::kNV12UBWC) &&
      (master_params.format != VideoFormat::kNV16) &&
      (master_params.format != VideoFormat::kRGB)) {
    QMMF_ERROR("%s Invalid format !", __func__);
    return false;
  }

  if ((slave_params.width >  master_params.width) ||
      (slave_params.height > master_params.height)) {
    QMMF_ERROR("%s Invalid dimensions !", __func__);
    return false;
  }
  return true;
}

bool CameraSource::CheckLinkedStream(
    const VideoTrackParam& slave_params,
    const VideoTrackParam& master_params) {

  QMMF_DEBUG("%s %d x %d -> %d x %d fmt 0x%x -> 0x%x", __func__,
    master_params.width, master_params.height,
    slave_params.width, slave_params.height,
    master_params.format, slave_params.format);

  if ((slave_params.format != VideoFormat::kNV12) &&
      (slave_params.format != VideoFormat::kNV12UBWC) &&
      (slave_params.format != VideoFormat::kNV16) &&
      (slave_params.format != VideoFormat::kRGB) &&
      (master_params.format != VideoFormat::kNV12) &&
      (master_params.format != VideoFormat::kNV12UBWC) &&
      (master_params.format != VideoFormat::kNV16) &&
      (master_params.format != VideoFormat::kRGB)) {
    QMMF_ERROR("%s Invalid format !", __func__);
    return false;
  }

  if ((slave_params.width ==  master_params.width) &&
      (slave_params.height == master_params.height) &&
      (master_params.format == slave_params.format)) {
    QMMF_ERROR("%s Same size and format !", __func__);
    return true;
  }
  return false;
}

ResizerCrop CameraSource::GetRescalerConfig(const VideoExtraParam& extraparams) {

  ResizerCrop resizer_crop;
  if (extraparams.Exists(QMMF_TRACK_CROP)) {
    TrackCrop crop;
    extraparams.Fetch(QMMF_TRACK_CROP, crop);

    resizer_crop.x = crop.x;
    resizer_crop.y = crop.y;
    resizer_crop.width = crop.width;
    resizer_crop.height = crop.height;
    resizer_crop.valid = true;
    QMMF_INFO("%s Crop applied successfully!", __func__);
  } else {
    resizer_crop.valid = false;
    QMMF_INFO("%s Crop param doesn't exist so it's not applied!", __func__);
  }

  return resizer_crop;
}

status_t CameraSource::CreateTrackSource(const uint32_t track_id,
                                         const VideoTrackParam& params,
                                         const VideoExtraParam& xtraparam,
                                         const BnBufferCallback &cb) {

  QMMF_DEBUG("%s: Enter", __func__);
  QMMF_KPI_DETAIL();

  auto camera_id = params.camera_id;
  active_cameras_lock_.lock();
  if (active_cameras_.count(camera_id) == 0) {
    active_cameras_lock_.unlock();
    QMMF_ERROR("%s: Invalid Camera Id(%d)", __func__, camera_id);
    return BAD_VALUE;
  }
  auto const& camera = active_cameras_[camera_id];
  active_cameras_lock_.unlock();

  status_t ret = NO_ERROR;
  bool copy_stream_mode = false;
  bool linked_mode = false;
  VideoTrackParam source_params {};

  int32_t source_track_id = GetSourceTrackId(xtraparam);
  if (source_track_id != NAME_NOT_FOUND) {
    QMMF_INFO("%s: Master->slave 0x%x->0x%x", __func__, source_track_id,
        track_id);

    assert(track_sources_.count(source_track_id) != 0);
    auto track = track_sources_[source_track_id];

    if (ValidateSlaveTrackParam(params, track->GetParams())) {
      linked_mode = CheckLinkedStream(params, track->GetParams());
      copy_stream_mode = true;
      source_params = track->GetParams();
      QMMF_INFO("%s: Copy stream should be create.", __func__);
    } else {
      QMMF_ERROR("%s: Copy stream validation failed.", __func__);
    }
  } else {
    QMMF_INFO("%s: Normal stream should be create.", __func__);
  }

  // Enfroce frame rate cotrol in track for linked and copy streams
  bool fr_control = frame_rate_control_ | copy_stream_mode;

  QMMF_DEBUG ("%s: frame_rate_contol %d ", __func__, fr_control);

  // Create TrackSource and give it to CameraInterface, CameraConext in turn
  // would map it to its one of port.
  auto track_source = make_shared<TrackSource>(track_id, camera, params,
                                               xtraparam, fr_control, cb);
  if (!track_source.get()) {
    QMMF_ERROR("%s: Can't create TrackSource Instance", __func__);
    return NO_MEMORY;
  }

  shared_ptr<TrackSource> master_track;
  if (copy_stream_mode) {
    std::shared_ptr<CameraRescaler> rescaler;
    if (linked_mode == false) {
      rescaler = std::make_shared<CameraRescaler>();
      auto format = Common::FromVideoToQmmfFormat(params.format);

      start_cam_param_lock_.lock();
      auto extra_param = start_cam_param_[camera_id];
      start_cam_param_lock_.unlock();

      ret = rescaler->Init(params.width, params.height, format,
                           source_params.framerate, params.framerate,
                           extra_param);
      if (ret != NO_ERROR) {
        rescaler = nullptr;
        QMMF_ERROR("%s: Rescaler Init Failed", __func__);
        return BAD_VALUE;
      }
      rescaler->Configure(GetRescalerConfig(xtraparam));
      rescalers_.emplace(track_id, rescaler);
    }

    assert(track_sources_.count(source_track_id) != 0);
    master_track = track_sources_[source_track_id];
    assert(master_track.get() != nullptr);

    ret = track_source->InitCopy(master_track, rescaler);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s: Track(%x): TrackSource InitCopy failed!", __func__,
         track_id);
      rescalers_.erase(track_id);
      goto FAIL;
    }
  } else {
    if (params.format == VideoFormat::kRGB) {
      QMMF_ERROR("%s Unsupported format: RGB", __func__);
      goto FAIL;
    }
    ret = track_source->Init();
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s: Track(%x): TrackSource Init failed!", __func__,
          track_id);
      goto FAIL;
    }
  }

  track_sources_.emplace(track_id, track_source);

  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
FAIL:
  track_source = nullptr;
  return ret;
}

status_t CameraSource::DeleteTrackSource(const uint32_t track_id) {

  if (!IsTrackIdValid(track_id)) {
    QMMF_ERROR("%s: Track(%x): does not exist !!", __func__, track_id);
    return BAD_VALUE;
  }
  auto const& track = track_sources_[track_id];

  auto ret = track->DeInit();
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: Track(%x): DeInit failed !!", __func__, track_id);
    return ret;
  }

  track_sources_.erase(track_id);
  rescalers_.erase(track_id);

  QMMF_INFO("%s: Track(%x): Deleted Successfully!", __func__,
      track_id);
  return ret;
}

status_t CameraSource::StartTrackSource(const uint32_t track_id) {

  QMMF_KPI_DETAIL();
  if (!IsTrackIdValid(track_id)) {
    QMMF_ERROR("%s: Track(%x): does not exist !!", __func__, track_id);
    return BAD_VALUE;
  }
  auto const& track = track_sources_[track_id];

  auto ret = track->StartTrack();
  assert(ret == NO_ERROR);

  QMMF_VERBOSE("%s: TrackSource id(%x) Started Successfully!", __func__,
      track_id);
  return ret;
}

status_t CameraSource::FlushTrackSource(const uint32_t track_id) {

  QMMF_KPI_DETAIL();
  if (!IsTrackIdValid(track_id)) {
    QMMF_ERROR("%s: Track(%x): does not exist !!", __func__, track_id);
    return BAD_VALUE;
  }
  auto const& track = track_sources_[track_id];

  auto ret = track->Flush();
  assert(ret == NO_ERROR);

  QMMF_VERBOSE("%s: TrackSource id(%x) Flush Buffers Successfully!", __func__,
      track_id);
  return ret;
}

status_t CameraSource::StopTrackSource(const uint32_t track_id) {

  QMMF_KPI_DETAIL();
  if (!IsTrackIdValid(track_id)) {
    QMMF_ERROR("%s: Track(%x): does not exist !!", __func__, track_id);
    return BAD_VALUE;
  }
  auto const& track = track_sources_[track_id];

  auto ret = track->StopTrack();
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: Track(%x): Stop failed !!", __func__, track_id);
    return ret;
  }

  QMMF_VERBOSE("%s: TrackSource id(%x) Stopped Successfully!", __func__,
      track_id);
  return ret;
}

status_t CameraSource::PauseTrackSource(const uint32_t track_id) {

  QMMF_KPI_DETAIL();
  if (!IsTrackIdValid(track_id)) {
    QMMF_ERROR("%s: Track(%x): does not exist !!", __func__, track_id);
    return BAD_VALUE;
  }
  auto const& track = track_sources_[track_id];

  auto ret = track->PauseTrack();
  assert(ret == NO_ERROR);

  QMMF_VERBOSE("%s: TrackSource id(%x) Paused Successfully!", __func__,
      track_id);
  return ret;
}

status_t CameraSource::ResumeTrackSource(const uint32_t track_id) {

  QMMF_KPI_DETAIL();
  if (!IsTrackIdValid(track_id)) {
    QMMF_ERROR("%s: Track(%x): does not exist !!", __func__, track_id);
    return BAD_VALUE;
  }
  auto const& track = track_sources_[track_id];

  auto ret = track->ResumeTrack();
  assert(ret == NO_ERROR);

  QMMF_VERBOSE("%s: TrackSource id(%x) Resumed Successfully!", __func__,
      track_id);
  return ret;
}

status_t CameraSource::ReturnTrackBuffer(const uint32_t track_id,
                                         std::vector<BnBuffer> &buffers) {

  if (!IsTrackIdValid(track_id)) {
    QMMF_ERROR("%s: Track(%x): does not exist !!", __func__, track_id);
    return BAD_VALUE;
  }
  auto const& track = track_sources_[track_id];

  auto ret = track->ReturnTrackBuffer(buffers);
  assert(ret == NO_ERROR);
  return ret;
}

status_t CameraSource::SetCameraParam(const uint32_t camera_id,
                                      const ::camera::CameraMetadata &meta) {

  active_cameras_lock_.lock();
  if (active_cameras_.count(camera_id) == 0) {
    active_cameras_lock_.unlock();
    QMMF_ERROR("%s: Invalid Camera Id(%d)", __func__, camera_id);
    return BAD_VALUE;
  }

  auto const& camera = active_cameras_[camera_id];
  active_cameras_lock_.unlock();

  return camera->SetCameraParam(meta);
}

status_t CameraSource::GetCameraParam(const uint32_t camera_id,
                                      ::camera::CameraMetadata &meta) {

  active_cameras_lock_.lock();
  if (active_cameras_.count(camera_id) == 0) {
    active_cameras_lock_.unlock();
    QMMF_ERROR("%s: Invalid Camera Id(%d)", __func__, camera_id);
    return BAD_VALUE;
  }

  auto const& camera = active_cameras_[camera_id];
  active_cameras_lock_.unlock();

  return camera->GetCameraParam(meta);
}

status_t CameraSource::SetCameraSessionParam(const uint32_t camera_id,
                                             const ::camera::CameraMetadata &meta) {

  active_cameras_lock_.lock();
  if (active_cameras_.count(camera_id) == 0) {
    active_cameras_lock_.unlock();
    QMMF_ERROR("%s: Invalid Camera Id(%d)", __func__, camera_id);
    return BAD_VALUE;
  }

  auto const& camera = active_cameras_[camera_id];
  active_cameras_lock_.unlock();

  return camera->SetCameraSessionParam(meta);
}

status_t CameraSource::SetSHDR(const uint32_t camera_id,
                               const bool enable) {

  active_cameras_lock_.lock();
  if (active_cameras_.count(camera_id) == 0) {
    active_cameras_lock_.unlock();
    QMMF_ERROR("%s: Invalid Camera Id(%d)", __func__, camera_id);
    return BAD_VALUE;
  }

  auto const& camera = active_cameras_[camera_id];
  active_cameras_lock_.unlock();

  return camera->SetSHDR(enable);
}

status_t CameraSource::GetDefaultCaptureParam(const uint32_t camera_id,
                                              ::camera::CameraMetadata &meta) {

  active_cameras_lock_.lock();
  if (active_cameras_.count(camera_id) == 0) {
    active_cameras_lock_.unlock();
    QMMF_ERROR("%s: Invalid Camera Id(%d)", __func__, camera_id);
    return BAD_VALUE;
  }

  auto const& camera = active_cameras_[camera_id];
  active_cameras_lock_.unlock();

  return camera->GetDefaultCaptureParam(meta);
}

status_t CameraSource::GetCameraCharacteristics(const uint32_t camera_id,
                                                ::camera::CameraMetadata &meta) {

  active_cameras_lock_.lock();
  if (active_cameras_.count(camera_id) == 0) {
    active_cameras_lock_.unlock();
    QMMF_ERROR("%s: Invalid Camera Id(%d)", __func__, camera_id);
    return BAD_VALUE;
  }

  auto const& camera = active_cameras_[camera_id];
  active_cameras_lock_.unlock();

  return camera->GetCameraCharacteristics(meta);
}

status_t CameraSource::UpdateTrackFrameRate(const uint32_t track_id,
                                            const float framerate) {

  if (!IsTrackIdValid(track_id)) {
    QMMF_ERROR("%s: Track(%x): does not exist !!", __func__, track_id);
    return BAD_VALUE;
  }
  auto const& track = track_sources_[track_id];

  track->UpdateFrameRate(framerate);
  return NO_ERROR;
}

status_t CameraSource::EnableFrameRepeat(const uint32_t track_id,
                                         const bool enable) {

  if (!IsTrackIdValid(track_id)) {
    QMMF_ERROR("%s: Track(%x): does not exist !!", __func__, track_id);
    return BAD_VALUE;
  }
  auto const& track = track_sources_[track_id];

  track->EnableFrameRepeat(enable);
  return NO_ERROR;
}

const shared_ptr<TrackSource>& CameraSource::GetTrackSource(uint32_t track_id) {

  assert(track_sources_.count(track_id) != 0);
  return track_sources_[track_id];
}

bool CameraSource::IsTrackIdValid(const uint32_t track_id) {

  QMMF_DEBUG("%s: Number of Tracks exist: %d",__func__, track_sources_.size());
  return (track_sources_.count(track_id) != 0) ? true : false;
}

uint32_t CameraSource::GetJpegSize(uint8_t *blobBuffer, uint32_t size) {

  uint32_t ret = size;
#ifndef CAMERA_HAL1_SUPPORT
  uint32_t blob_size = sizeof(struct camera3_jpeg_blob);

  if (size > blob_size) {
    size_t offset = size - blob_size - JPEG_BLOB_OFFSET;
    uint8_t *footer = blobBuffer + offset;
    struct camera3_jpeg_blob *jpegBlob = (struct camera3_jpeg_blob *)footer;

    if (CAMERA3_JPEG_BLOB_ID == jpegBlob->jpeg_blob_id) {
      ret = jpegBlob->jpeg_size;
    } else {
      QMMF_ERROR("%s Jpeg Blob structure missing!\n", __func__);
    }
  } else {
    QMMF_ERROR("%s Buffer size: %u equal or smaller than Blob size: %u\n",
        __func__, size, blob_size);
  }
#endif
  return ret;
}

status_t CameraSource::ParseThumb(uint8_t* vaddr, uint32_t size,
                                  StreamBuffer& buffer) {
  enum Tags { TAG = 0xFF, SOI = 0xD8, EOI = 0xD9, APP1 = 0xE1, APP2 = 0xE2 };
  enum TagSizeByte { MARKER_TAG_SIZE = 2, MARKER_LENGTH_SIZE = 2 };

  uint8_t thumb_num = 0;
  uint8_t *in_img = vaddr;
  uint32_t block_size = 0;
  uint32_t block_start = 0;
  uint32_t block_end = 0;
  BufferMeta info = buffer.info;

  // reset planes num
  info.n_planes = 0;
  info.planes[info.n_planes].offset = 0;
  info.planes[info.n_planes].size = size;
  info.n_planes++;

  QMMF_INFO("%s: Parse Thumbnail", __func__);

  for (uint32_t i = 0; i < size - 1; i++) {
    // search for marker
    if (in_img[i] == TAG) {
      // search for App1 and App2 marker
      if ((in_img[i + 1] == APP1) || (in_img[i + 1] == APP2)) {
        if (i >= size - 4) { // prevent bad access
          break;
        }

        block_size  = (256UL * in_img[i + 2]) + in_img[i + 3];
        block_start = i + MARKER_TAG_SIZE; // AppN marker is not part of block
        block_end   = block_start + block_size;

        // Skip App marker and size
        i += (1 + MARKER_LENGTH_SIZE);

      // Search for start of thumbnail or continue with multy segment thumbnail
      } else if (in_img[i + 1] == SOI && block_size) {

        uint32_t w_size = block_end - i;
        if (i + w_size > size) {
          QMMF_ERROR("%s: Unable to write. Overflow thumb file. %d > %d",
              __func__, i + w_size, size);
          break;
        }

        for (;;) {

          if (info.n_planes == MAX_PLANE) {
            QMMF_ERROR("%s: Fail to parse thumbnail num_plane: %d!!!",
                __func__, info.n_planes);
            return BAD_VALUE;
          }

          info.planes[info.n_planes].offset = i;
          info.planes[info.n_planes].size = w_size;
          info.n_planes++;

          // Move to end of block
          i += w_size;

          // Check for end of thumbnail
          if (in_img[i - 2] == TAG && in_img[i - 1] == EOI) {
            break;
          } else if (i + 4 < size && // prevent bad access
                     in_img[i] == TAG && in_img[i + 1] == APP2) {
            block_size  = (256UL * in_img[i + 2]) + in_img[i + 3];
            block_start = i + MARKER_TAG_SIZE;//AppN marker is not part of block
            block_end   = block_start + block_size;

            i = block_start + MARKER_LENGTH_SIZE; // Skip length
            w_size = block_end - i;
          } else {
            return BAD_VALUE;
          }
        }

        i--; // because of increment in main loop
        block_size = 0;
        block_end = 0;
        thumb_num++;
        // max supported thumbnails is 2
        if (thumb_num > 1) {
          break;
        }
      }
    }
  }

  // main image
  if (info.n_planes > 1) {
    info.planes[0].offset =
        info.planes[info.n_planes - 1].offset +
        info.planes[info.n_planes - 1].size;
    info.planes[0].size = size - info.planes[0].offset;
  }

  // restore plane info
  buffer.info = info;
  return NO_ERROR;
}

void CameraSource::SnapshotCallback(uint32_t image_id, uint32_t count,
                                    StreamBuffer& buffer) {

  uint32_t content_size = 0;
  int32_t width = -1, height = -1;
  void* vaddr = nullptr;
  switch (buffer.info.format) {
    case BufferFormat::kNV12:
    case BufferFormat::kNV12HEIF:
    case BufferFormat::kNV21:
    case BufferFormat::kNV16:
    case BufferFormat::kRAW8:
    case BufferFormat::kRAW10:
    case BufferFormat::kRAW12:
    case BufferFormat::kRAW16:
      width  = buffer.info.planes[0].width;
      height = buffer.info.planes[0].height;
      content_size = buffer.size;
      break;
    case BufferFormat::kBLOB:
      vaddr = mmap(nullptr, buffer.size, PROT_READ | PROT_WRITE, MAP_SHARED,
          buffer.fd, 0);
      assert(vaddr != nullptr);
      assert(0 < buffer.info.n_planes);
      content_size = GetJpegSize((uint8_t*) vaddr,
                                 buffer.info.planes[0].size);
      QMMF_INFO("%s: jpeg buffer size(%d)", __func__, content_size);
      assert(0 < content_size);
      if (buffer.second_thumb) {
        auto ret = ParseThumb(static_cast<uint8_t*>(vaddr),
                              content_size, buffer);
        if (ret != NO_ERROR) {
          QMMF_ERROR("%s: Warning: ParseThumb failed!!", __func__);
        }
      }

      if (vaddr) {
        munmap(vaddr, buffer.size);
        vaddr = nullptr;
      }
      width  = -1;
      height = -1;
    break;
    default:
      QMMF_ERROR("%s format(%d) not supported", __func__,
        (int32_t) buffer.info.format);
      assert(0);
    break;
  }

  BnBuffer bn_buffer{};
  bn_buffer.img_id      = image_id;
  bn_buffer.ion_fd      = buffer.fd;
  bn_buffer.ion_meta_fd = buffer.metafd;
  bn_buffer.size        = content_size;
  bn_buffer.timestamp   = buffer.timestamp;
  bn_buffer.seqnum      = buffer.frame_number;
  bn_buffer.buffer_id   = buffer.fd;
  bn_buffer.capacity    = buffer.size;

  BufferMeta meta = buffer.info;
  client_snapshot_cb_(buffer.camera_id, count, bn_buffer, meta);
}

TrackSource::TrackSource(const uint32_t id,
                         const std::shared_ptr<CameraInterface>& camera,
                         const VideoTrackParam& params,
                         const VideoExtraParam& extraparams,
                         const bool frame_rate_cotrol,
                         const BnBufferCallback& cb)
    : id_(id),
      params_(params),
      extraparams_(extraparams),
      buffer_cb_(cb),
      is_stop_(false),
      is_paused_(false),
      is_idle_(true),
      fsc_(nullptr),
      frc_(nullptr),
      rescaler_(nullptr),
      num_consumers_(0),
      slave_track_source_(false) {

  QMMF_GET_LOG_LEVEL();

  BufferConsumerImpl<TrackSource> *consumer;
  consumer = new BufferConsumerImpl<TrackSource>(this);
  buffer_consumer_ = consumer;

  BufferProducerImpl<TrackSource> *producer;
  producer = new BufferProducerImpl<TrackSource>(this);
  buffer_producer_ = producer;

  assert(camera.get() != nullptr);
  camera_ = camera;

  std::stringstream name;
  name << "Track(" << std::hex << id_ << ")";

  if (frame_rate_cotrol) {
    fsc_ = std::make_shared<FrameRateController>("FrameSkip: " + name.str());
    assert(fsc_.get() != nullptr);
    fsc_->SetFrameRate(params_.framerate);

    frc_ = std::make_shared<FrameRateController>("FrameRepeat: " + name.str());
    assert(frc_.get() != nullptr);
    frc_->SetFrameRate(params_.framerate);
  }

  QMMF_INFO("%s: TrackSource (0x%p)", __func__, this);
}

TrackSource::~TrackSource() {

  QMMF_INFO("%s: Enter ", __func__);

  QMMF_INFO("%s: Exit(0x%p) ", __func__, this);
}

status_t TrackSource::AddConsumer(const sp<IBufferConsumer>& consumer) {

  std::lock_guard<std::mutex> lock(consumer_lock_);
  if (consumer.get() == nullptr) {
    QMMF_ERROR("%s: Input consumer is nullptr", __func__);
    return BAD_VALUE;
  }

  buffer_producer_->AddConsumer(consumer);
  consumer->SetProducerHandle(buffer_producer_);

  num_consumers_ = buffer_producer_->GetNumConsumer();

  QMMF_VERBOSE("%s: Consumer(%p) has been added.", __func__,
      consumer.get());
  return NO_ERROR;
}

status_t TrackSource::RemoveConsumer(sp<IBufferConsumer>& consumer) {

  std::lock_guard<std::mutex> lock(consumer_lock_);

  if (buffer_producer_->GetNumConsumer() == 0) {
    QMMF_ERROR("%s: There are no connected consumers!", __func__);
    return INVALID_OPERATION;
  }

  buffer_producer_->RemoveConsumer(consumer);

  num_consumers_ = buffer_producer_->GetNumConsumer();

  QMMF_VERBOSE("%s: Consumer(%p) has been removed.", __func__,
      consumer.get());
  return NO_ERROR;
}

status_t TrackSource::InitCopy(shared_ptr<TrackSource> master_track_source,
                               const std::shared_ptr<CameraRescaler>& rescaler) {

  QMMF_DEBUG("%s: Enter Track(%x)", __func__, id_);
  status_t result = 0;
  assert(master_track_source.get() != nullptr);
  master_track_ = master_track_source;

  slave_track_source_ = true;

  if (rescaler.get() == nullptr) {
    QMMF_INFO("%s Linked stream without down scale", __func__);
  } else {
    rescaler_ = rescaler;
  }

  sp<IBufferConsumer> consumer;
  consumer = GetConsumer();
  assert(consumer.get() != nullptr);

  if (frc_.get() != nullptr) {
    result = frc_->AddConsumer(consumer);
    assert(result == NO_ERROR);
    consumer = frc_->GetConsumer();
    assert(consumer.get() != nullptr);
  }

  if (rescaler_.get() != nullptr) {
    result = rescaler_->AddConsumer(consumer);
    assert(result == NO_ERROR);
    consumer = rescaler_->GetConsumer();
    assert(consumer.get() != nullptr);
  }

  if (fsc_.get() != nullptr) {
    result = fsc_->AddConsumer(consumer);
    assert(result == NO_ERROR);
    consumer = fsc_->GetConsumer();
    assert(consumer.get() != nullptr);
  }

  result = master_track_->AddConsumer(consumer);
  assert(result == NO_ERROR);

  QMMF_DEBUG("%s: Exit Track(%x)", __func__, id_);
  return result;
}

status_t TrackSource::Init() {

  QMMF_DEBUG("%s Enter Track(%x)", __func__, id_);

  slave_track_source_ = false;
  master_track_ = nullptr;

  StreamParam param{};
  param.id = id_;
  param.width = params_.width;
  param.height = params_.height;
  param.framerate = params_.framerate;
  param.rotation = params_.rotation;
  param.xtrabufs = params_.xtrabufs;
  param.flags = params_.flags;
  param.format = Common::FromVideoToQmmfFormat(params_.format);

  assert(camera_.get() != nullptr);
  auto ret = camera_->CreateStream(param, extraparams_);
  if (ret != NO_ERROR) {
    QMMF_ERROR("%s: CreateStream failed!!", __func__);
    return BAD_VALUE;
  }

  QMMF_INFO("%s: TrackSource(0x%p)(%dx%d) and Camera Device Stream "
      " Created Successfully for Track(%x)",  __func__, this,
      params_.width, params_.height, id_);

  sp<IBufferConsumer> consumer;
  consumer = GetConsumer();
  assert(consumer.get() != nullptr);

  if (frc_.get() != nullptr) {
    ret = frc_->AddConsumer(consumer);
    assert(ret == NO_ERROR);
    consumer = frc_->GetConsumer();
    assert(consumer.get() != nullptr);
  }

  if (fsc_.get() != nullptr) {
    ret = fsc_->AddConsumer(consumer);
    assert(ret == NO_ERROR);
    consumer = fsc_->GetConsumer();
    assert(consumer.get() != nullptr);
  }

  ret = camera_->AddConsumer(id_, consumer);
  assert(ret == NO_ERROR);

  QMMF_DEBUG("%s Exit Track(%x)", __func__, id_);
  return ret;
}

status_t TrackSource::DeInit() {

  QMMF_DEBUG("%s Enter Track(%x)", __func__, id_);
  assert(camera_.get() != nullptr);
  status_t ret = NO_ERROR;

  sp<IBufferConsumer> consumer = GetConsumer();
  assert(consumer.get() != nullptr);

  if (frc_.get() != nullptr) {
    ret = frc_->RemoveConsumer(consumer);
    assert(ret == NO_ERROR);
    consumer = frc_->GetConsumer();
    assert(consumer.get() != nullptr);
  }

  if (rescaler_.get() != nullptr) {
    ret = rescaler_->RemoveConsumer(consumer);
    assert(ret == NO_ERROR);
    consumer = rescaler_->GetConsumer();
    assert(consumer.get() != nullptr);
  }

  if (fsc_.get() != nullptr) {
    ret = fsc_->RemoveConsumer(consumer);
    assert(ret == NO_ERROR);
    consumer = fsc_->GetConsumer();
    assert(consumer.get() != nullptr);
  }

  if (slave_track_source_) {
    ret = master_track_->RemoveConsumer(consumer);
    assert(ret == NO_ERROR);
  } else {
    ret = camera_->RemoveConsumer(id_, consumer);
    assert(ret == NO_ERROR);
  }

  std::unique_lock<std::mutex> idle_lock(idle_lock_);
  std::chrono::nanoseconds wait_time(kWaitDuration);

  while (!is_idle_) {
    auto ret = wait_for_idle_.WaitFor(idle_lock, wait_time);
    if (ret != 0) {
      QMMF_ERROR("%s: Track(%x): StopTrack Timed out happened! Encoder"
          " failed to go in Idle state!",  __func__, id_);
      return TIMED_OUT;
    }
  }

  if (slave_track_source_ == false) {
    ret = camera_->DeleteStream(id_);
    if (ret != NO_ERROR) {
      QMMF_ERROR("%s: Track(%x): DeleteStream failed", __func__, id_);
      return ret;
    }
  }

  rescaler_ = nullptr;

  QMMF_DEBUG("%s Exit Track(%x)", __func__, id_);
  return ret;
}

status_t TrackSource::StartTrack() {

  QMMF_DEBUG("%s: Enter Track(%x)", __func__, id_);
  std::lock_guard<std::mutex> lock(lock_);

  assert(camera_.get() != nullptr);

  std::lock_guard<std::mutex> stop_lock(stop_lock_);
  is_stop_ = false;

  status_t ret = NO_ERROR;
  if (slave_track_source_ == false) {
    ret = camera_->StartStream(id_);
    assert(ret == NO_ERROR);
  }

  if (rescaler_.get() != nullptr) {
    ret = rescaler_->Start();
    assert(ret == NO_ERROR);
  }

  if (fsc_.get() != nullptr) {
    ret = fsc_->Start();
    assert(ret == NO_ERROR);
  }

  if (frc_.get() != nullptr) {
    ret = frc_->Start();
    assert(ret == NO_ERROR);
  }

  QMMF_DEBUG("%s: Exit Track(%x)", __func__, id_);
  return NO_ERROR;
}

status_t TrackSource::Flush() {
  status_t ret;

  QMMF_DEBUG("%s: Enter Track(%x)", __func__, id_);

  QMMF_INFO("%s: Track(%x): Force return buffers!", __func__, id_);
  std::lock_guard<std::mutex> lk(buffer_list_lock_);
  for (auto it = buffer_list_.begin(); it != buffer_list_.end(); it++) {
    StreamBuffer buffer = it->second;
    ReturnBufferToProducer(buffer);
  }
  buffer_list_.clear();

  std::lock_guard<std::mutex> idle_lock(idle_lock_);
  is_idle_ = true;

  QMMF_DEBUG("%s: Exit Track(%x)", __func__, id_);
  return NO_ERROR;
}

status_t TrackSource::StopTrack() {
  status_t ret;

  QMMF_DEBUG("%s: Enter Track(%x)", __func__, id_);
  is_paused_ = false;

  std::lock_guard<std::mutex> lock(lock_);
  {
    std::lock_guard<std::mutex> lock(stop_lock_);
    if (is_stop_ == true) {
      QMMF_WARN("%s: Track(%x): already stopped!", __func__, id_);
      return NO_ERROR;
    }
    is_stop_ = true;
  }

  assert(camera_.get() != nullptr);

  if (frc_.get() != nullptr) {
    ret = frc_->Stop();
    assert(ret == NO_ERROR);
  }

  if (fsc_.get() != nullptr) {
    ret = fsc_->Stop();
    assert(ret == NO_ERROR);
  }

  if (rescaler_.get() != nullptr) {
    ret = rescaler_->Stop();
    assert(ret == NO_ERROR);
  }

  if (slave_track_source_ == false) {
    ret = camera_->StopStream(id_);
    assert(ret == NO_ERROR);
  }

  QMMF_INFO("%s: Pipe stop done(%x)", __func__, id_);
  {
    std::lock_guard<std::mutex> lk(buffer_list_lock_);
    QMMF_DEBUG("%s: Track(%x): buffer_list_.size(%d)", __func__,
        id_, buffer_list_.size());
  }

  QMMF_DEBUG("%s: Exit Track(%x)", __func__, id_);
  return NO_ERROR;
}

status_t TrackSource::PauseTrack() {

  QMMF_DEBUG("%s: Enter Track(%x)", __func__, id_);

  std::lock_guard<std::mutex> lock(lock_);
  is_paused_ = true;

  assert(camera_.get() != nullptr);

  if (slave_track_source_ == false) {
    status_t ret = camera_->PauseStream(id_);
    assert(ret == NO_ERROR);
  }

  std::lock_guard<std::mutex> idle_lock(idle_lock_);
  is_idle_ = true;

  return NO_ERROR;
}

status_t TrackSource::ResumeTrack() {

  QMMF_DEBUG("%s: Enter Track(%x)", __func__, id_);

  std::lock_guard<std::mutex> lock(lock_);
  is_paused_ = false;

  assert(camera_.get() != nullptr);

  status_t ret = camera_->ResumeStream(id_);
  assert(ret == NO_ERROR);

  std::lock_guard<std::mutex> idle_lock(idle_lock_);
  is_idle_ = false;

  return NO_ERROR;
}

void TrackSource::OnFrameAvailable(StreamBuffer& buffer) {

  QMMF_VERBOSE("%s: Enter Track(%x)", __func__, id_);
  {
    std::unique_lock<std::mutex> lock(frame_lock_);
    buffer_map_.insert(std::make_pair(buffer.handle, 1));
  }

  {
    std::lock_guard<std::mutex> lk(consumer_lock_);
    if (num_consumers_ > 0) {
      {
        std::unique_lock<std::mutex> lock(frame_lock_);
        buffer_map_[buffer.handle] += 1;
      }
      buffer_producer_->NotifyBuffer(buffer);
    }
  }

  if (IsPaused()) {
    QMMF_DEBUG("%s: Track(%x): Pause is triggred, return buffer fd: %d ts: %lld",
        __func__, id_, buffer.fd, buffer.timestamp);

    std::lock_guard<std::mutex> lock(frame_lock_);
    ReturnBufferToProducer(buffer);
    return;
  }

  if (IsStop()) {
    QMMF_DEBUG("%s: Track(%x): Stop is triggred, return buffer fd: %d ts: %lld",
        __func__, id_, buffer.fd, buffer.timestamp);

    std::unique_lock<std::mutex> lock(frame_lock_);
    ReturnBufferToProducer(buffer);
    return;
  }

  BnBuffer bn_buffer{};
  bn_buffer.img_id            = 0;
  bn_buffer.ion_fd            = buffer.fd;
  bn_buffer.ion_meta_fd       = buffer.metafd;
  bn_buffer.size              = buffer.size;
  bn_buffer.timestamp         = buffer.timestamp;
  bn_buffer.seqnum            = buffer.frame_number;
  bn_buffer.buffer_id         = buffer.fd;
  bn_buffer.flags             = 0x10;
  bn_buffer.capacity          = buffer.size;

  if (params_.format == VideoFormat::kJPEG) {
    void* vaddr = mmap(nullptr, buffer.size, PROT_READ | PROT_WRITE,
                       MAP_SHARED, buffer.fd, 0);
    assert(vaddr != nullptr);

    bn_buffer.size = CameraSource::GetJpegSize(
        (uint8_t*) vaddr, buffer.info.planes[0].size);
    QMMF_INFO("%s: JPEG buffer size: %d", __func__, bn_buffer.size);

    assert(0 < bn_buffer.size);
    munmap(vaddr, buffer.size);
  }

  // Buffers from this list used for YUV callback.
  {
    std::lock_guard<std::mutex> autoLock(buffer_list_lock_);
    buffer_list_.insert(std::make_pair(buffer.fd, buffer));
  }
  {
    std::lock_guard<std::mutex> idle_lock(idle_lock_);
    is_idle_ = false;
  }
  std::vector<BnBuffer> bn_buffers;
  bn_buffers.push_back(bn_buffer);

  std::vector<BufferMeta> metas;
  metas.push_back(buffer.info);

  buffer_cb_(bn_buffers, metas);
}

status_t TrackSource::ReturnTrackBuffer(std::vector<BnBuffer>& bn_buffers) {

  QMMF_DEBUG("%s: Enter Track(%x)", __func__, id_);
  assert(bn_buffers.size() > 0);
  assert(buffer_consumer_ != nullptr);

  std::unique_lock<std::mutex> lock(frame_lock_);
  for (size_t i = 0; i < bn_buffers.size(); ++i) {
    QMMF_VERBOSE("%s: Track(%x): bn_buffers[%d].ion_fd=%d", __func__,
        id_, i, bn_buffers[i].ion_fd);

    std::lock_guard<std::mutex> autoLock(buffer_list_lock_);
    auto it = buffer_list_.find(bn_buffers[i].ion_fd);
    assert(it != buffer_list_.end());

    StreamBuffer buffer = it->second;

    ReturnBufferToProducer(buffer);
    buffer_list_.erase(it);
  }
  QMMF_DEBUG("%s: Track(%x): buffer count still with client = %d", __func__,
      id_, buffer_list_.size());

  if (buffer_list_.size() == 0) {
    std::lock_guard<std::mutex> lock(idle_lock_);
    // wait_for_idle_ will not be needed once we make stop api as async.
    QMMF_DEBUG("%s: Track(%x): All buffers have been returned from client!",
        __func__, id_);

    is_idle_ = true;
    wait_for_idle_.Signal();
  }

  QMMF_DEBUG("%s: Exit Track(%x)", __func__, id_);
  return NO_ERROR;
}

bool TrackSource::IsStop() {

  QMMF_VERBOSE("%s: Enter Track(%x)", __func__, id_);
  std::lock_guard<std::mutex> lock(stop_lock_);
  QMMF_VERBOSE("%s: Exit Track(%x)", __func__, id_);
  return is_stop_;
}

bool TrackSource::IsPaused() {

  return is_paused_;
}

void TrackSource::UpdateFrameRate(const float framerate) {

  if (fabs(params_.framerate - framerate) > 0.1f) {
    if (fsc_.get() != nullptr) {
      fsc_->SetFrameRate(framerate);
    }
    if (frc_.get() != nullptr) {
      frc_->SetFrameRate(framerate);
    }
    params_.framerate = framerate;
  }
}

void TrackSource::EnableFrameRepeat(const bool enable) {

  if (frc_.get() != nullptr) {
    frc_->EnableFrameRepeat(enable);
  }
}

void TrackSource::ReturnBufferToProducer(StreamBuffer& buffer) {
  QMMF_DEBUG("%s: Enter Track(%x): fd: %d ts: %lld", __func__,
      id_, buffer.fd, buffer.timestamp);

  if (buffer_map_.find(buffer.handle) == buffer_map_.end()) {
    QMMF_ERROR("%s: Track(%x): fd: %d ts: %lld", __func__,
        id_, buffer.fd, buffer.timestamp);
  } else {
    QMMF_DEBUG("%s: Buffer is back to Producer Intf,buffer(0x%p) RefCount=%d",
        __func__, buffer.handle, buffer_map_.at(buffer.handle));
    if (buffer_map_.at(buffer.handle) == 1) {
      buffer_map_.erase(buffer.handle);
      // Return buffer back to actual owner.
      buffer_consumer_->GetProducerHandle()->NotifyBufferReturned(buffer);
    } else {
      // Hold this buffer, do not return until its ref count is 1.
      buffer_map_[buffer.handle] -= 1;

      if (num_consumers_ == 0 && rescaler_) {
        buffer_consumer_->GetProducerHandle()->NotifyBufferReturned(buffer);
        buffer_map_.erase(buffer.handle);
      }
    }
  }
}

void TrackSource::NotifyBufferReturned(StreamBuffer& buffer) {
  QMMF_DEBUG("%s: Enter Track(%x): fd: %d ts: %lld", __func__,
      id_, buffer.fd, buffer.timestamp);
  std::unique_lock<std::mutex> lock(frame_lock_);
  ReturnBufferToProducer(buffer);
}

}; //namespace recorder

}; //namespace qmmf
