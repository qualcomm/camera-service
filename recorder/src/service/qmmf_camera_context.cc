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
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
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

#define LOG_TAG "RecorderCameraContext"

#include <future>
#include <thread>
#include <algorithm>
#include <chrono>
#include <fcntl.h>
#include <math.h>
#include <algorithm>
#include <sys/mman.h>
#ifdef QCAMERA3_TAG_LOCAL_COPY
#include "common/utils/qmmf_common_utils.h"
#else
#include <QCamera3VendorTags.h>
#endif

#include "recorder/src/service/qmmf_camera_context.h"
#include "recorder/src/service/qmmf_recorder_utils.h"

/* This map contains the vendor tags which needs to be set only once.
 * contains the map of vendor tags and its default value.
 * In the request meta if one of this vendor tag is present and
 * the value is not equal to default value, we will add the
 * meta to the request and then for subsequent request will update
 * value of that vendor tag to default.
 */
static const std::unordered_map<const char*, uint8_t> kSingleShotMeta {
  {"org.quic.camera.lensDriverManager.ResetIrisMotor",           0 },
  {"org.quic.camera.lensDriverManager.ResetFocusMotor",          0 },
  {"org.quic.camera.lensDriverManager.ResetZoomMotor",           0 },
  {"org.codeaurora.qcamera3.sensorwriteinput.SensorStandByFlag", 0 }
};

namespace qmmf {

namespace recorder {

// Framerate after which we need to run in constrained mode.
#ifndef HFR_THRESHOLD
float CameraContext::kConstrainedModeThreshold = 30.0f;
#else
float CameraContext::kConstrainedModeThreshold = HFR_THRESHOLD;
#endif

// Max time to wait for AEC convergence
const uint32_t CameraContext::kWaitAecTimeout = AEC_WAIT_TIMEOUT;

// Framerate at which batch requests are needed.
#ifdef _DRONE_
float CameraContext::kHFRBatchModeThreshold = 90.0f;
#else
float CameraContext::kHFRBatchModeThreshold = 120.0f;
#endif

CameraContext::CameraContext()
    : camera_id_(-1),
      streaming_request_id_(-1),
      capture_request_id_(-1),
      last_frame_number_(NO_IN_FLIGHT_REPEATING_FRAMES),
      capture_cnt_(0),
      result_cb_(nullptr),
      error_cb_(nullptr),
      zsl_port_id_(0x100),
      hfr_supported_(false),
      batch_stream_id_(-1),
      partial_result_count_(0),
      snapshot_mode_(ImageMode::kSnapshot),
      port_paused_(false),
      camera_parameters_{},
      is_partial_metadata_enabled_(false),
      pcr_frc_enabled_(false),
      continuous_mode_is_on_(false),
      is_camera_dead_(false),
      pending_cached_stream_(false),
      hfr_detected_(false),
      enable_reproc_(false),
      multi_roi_count_(0),
      multi_roi_count_tag_(0),
      multi_roi_info_tag_(0),
      multi_roi_info_{},
      hfr_wait_ports_ready_(false) {

  QMMF_INFO("%s: Enter", __func__);

  //Setup Camera3DeviceClient callbacks.
  camera_callbacks_.errorCb = [&] (CameraErrorCode errcode,
      const CaptureResultExtras &extras) { CameraErrorCb(errcode, extras);};

  camera_callbacks_.idleCb = [&] () { CameraIdleCb(); };

  camera_callbacks_.peparedCb = [&] (int32_t id) { CameraPreparedCb(id); };

  camera_callbacks_.shutterCb = [&] (const CaptureResultExtras &extras,
      int64_t ts) { CameraShutterCb(extras, ts); };

  camera_callbacks_.resultCb = [&] (const CaptureResult &result)
      { CameraResultCb(result); };

  camera_device_ = std::make_shared<Camera3DeviceClient>(camera_callbacks_);
  if (!camera_device_) {
    QMMF_ERROR("%s: Can't Instantiate Camera3DeviceClient", __func__);
  }

  if (camera_device_ && camera_device_->Initialize() != 0) {
    QMMF_ERROR("%s: Unable to Initialize Camera3DeviceClient", __func__);
    camera_device_.reset();
  }

  QMMF_INFO("%s: Exit", __func__);
}

CameraContext::~CameraContext() {

  QMMF_INFO("%s: Enter", __func__);
  //TODO: check all active ports
  is_camera_dead_ = false;
  QMMF_INFO("%s: Exit", __func__);
}

void CameraContext::InitSupportedFPS() {
  if (static_meta_.exists(ANDROID_CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES)) {
    camera_metadata_entry_t entry = static_meta_.find(
        ANDROID_CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES);
    for (size_t i = 0 ; i < entry.count; i += 2) {
      if (entry.data.i32[i] == entry.data.i32[i+1]) {
        supported_fps_.push_back(entry.data.i32[i]);
      }
    }
  } else {
    QMMF_INFO("%s: Tag ANDROID_CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES "
        " doesn't exist in static metadata",  __func__);
  }
}

bool CameraContext::IsInputSupported() {
  if (static_meta_.exists(ANDROID_REQUEST_MAX_NUM_INPUT_STREAMS)) {
    camera_metadata_entry entry = static_meta_.find(
        ANDROID_REQUEST_MAX_NUM_INPUT_STREAMS);
    if (0 < entry.data.i32[0]) {
      return true;
    }
  }

  return false;
}

status_t CameraContext::CreateSnapshotStream(uint32_t image_id,
      CameraStreamParameters &stream_param, bool cache) {

  QMMF_INFO("%s: Enter", __func__);
  int32_t stream_id = -1;
  status_t ret = 0;

  ret = CreateDeviceStream(stream_param,
                           camera_parameters_.frame_rate_range[1],
                           &stream_id, cache);
  if (ret != 0) {
    QMMF_ERROR("%s: Failed creating snapshot stream: %d!", __func__, ret);
    return ret;
  }

  QMMF_INFO("%s Snapshot stream_id(%d)", __func__, stream_id);
  snapshot_request_.streamIds.push_back(stream_id);
  std::lock_guard<std::mutex> lock(stream_image_lock_);
  stream_image_map_.emplace(stream_id, image_id);

  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

status_t CameraContext::DeleteSnapshotStream(uint32_t image_id, bool cache) {
  QMMF_INFO("%s: Enter", __func__);
  status_t ret = 0;
  int32_t stream_id = -1;

  cache |= streaming_request_id_ == -1;

  stream_image_lock_.lock();
  for (auto& pair : stream_image_map_) {
    if (pair.second == image_id) {
      stream_id = pair.first;
      break;
    }
  }
  stream_image_lock_.unlock();
  if (stream_id == -1) {
    QMMF_ERROR("%s: Failed to find stream_id according to img_id %d",
        __func__, image_id);
    return -EINVAL;
  }
  auto err = DeleteDeviceStream(stream_id, cache);
  if (0 != err) {
    QMMF_ERROR("%s: Failed to delete snapshot stream_id %d ret %d",
        __func__, stream_id, err);
    ret = err;
  }

  std::lock_guard<std::mutex> lock(stream_image_lock_);
  stream_image_map_.erase(stream_id);
  snapshot_request_.streamIds.erase(std::remove(
      snapshot_request_.streamIds.begin(),
      snapshot_request_.streamIds.end(), stream_id));

  if (snapshot_request_.streamIds.empty())
    capture_request_id_ = -1;

  QMMF_INFO("%s Exit", __func__);
  return ret;
}

status_t CameraContext::OpenCamera(const uint32_t camera_id,
                                   const float frame_rate,
                                   const CameraExtraParam& extra_param,
                                   const ResultCb &cb,
                                   const ErrorCb &errcb) {

  uint32_t ret = 0;
  bool match_camera_id = false;
  uint32_t num_camera = 0;

  camera_parameters_ = {};
  camera_parameters_.frame_rate_range[0] = frame_rate;
  camera_parameters_.frame_rate_range[1] = frame_rate;

  if (extra_param.Exists(QMMF_VIDEO_HDR_MODE)) {
    size_t entry_count = extra_param.EntryCount(QMMF_VIDEO_HDR_MODE);
    if (entry_count == 1) {
      VideoHDRMode vid_hdr_mode;
      extra_param.Fetch(QMMF_VIDEO_HDR_MODE, vid_hdr_mode, 0);
#ifdef VHDR_MODES_ENABLE
      switch (vid_hdr_mode.mode) {
        case VHDRMode::kVHDROff:
          camera_parameters_.cam_feature_flags &=
            ~(static_cast<uint32_t>(CamFeatureFlag::kSHDRRaw));
          camera_parameters_.cam_feature_flags &=
            ~(static_cast<uint32_t>(CamFeatureFlag::kSHDRYUV));
          camera_parameters_.cam_feature_flags &=
            ~(static_cast<uint32_t>(CamFeatureFlag::kSHDRRawSwitch));
          camera_parameters_.cam_feature_flags &=
            ~(static_cast<uint32_t>(CamFeatureFlag::kSHDRYUVSwitch));
          camera_parameters_.cam_feature_flags &=
            ~(static_cast<uint32_t>(CamFeatureFlag::kQBCHDRVideo));
          camera_parameters_.cam_feature_flags &=
            ~(static_cast<uint32_t>(CamFeatureFlag::kQBCHDRSnapshot));
          break;
        case VHDRMode::kSHDRRaw:
          camera_parameters_.cam_feature_flags |=
            static_cast<uint32_t>(CamFeatureFlag::kSHDRRaw);
          break;
        case VHDRMode::kSHDRYuv:
          camera_parameters_.cam_feature_flags |=
            static_cast<uint32_t>(CamFeatureFlag::kSHDRYUV);
          break;
        case VHDRMode::kSHDRRawSwitchEnable:
          camera_parameters_.cam_feature_flags |=
            static_cast<uint32_t>(CamFeatureFlag::kSHDRRawSwitch);
          break;
        case VHDRMode::kSHDRYUVSwitchEnable:
          camera_parameters_.cam_feature_flags |=
            static_cast<uint32_t>(CamFeatureFlag::kSHDRYUVSwitch);
          break;
        case VHDRMode::kQBCHDRVideo:
          camera_parameters_.cam_feature_flags |=
            static_cast<uint32_t>(CamFeatureFlag::kQBCHDRVideo);
          break;
        case VHDRMode::kQBCHDRSnapshot:
          camera_parameters_.cam_feature_flags |=
            static_cast<uint32_t>(CamFeatureFlag::kQBCHDRSnapshot);
          break;
      }
#else
      if (vid_hdr_mode.enable == true) {
        QMMF_INFO("%s: HDR is ON..", __func__);
        camera_parameters_.cam_feature_flags |=
            static_cast<uint32_t>(CamFeatureFlag::kHDR);
      }
#endif // VHDR_MODES_ENABLE
    } else {
      QMMF_ERROR("%s: Invalid hdr mode received", __func__);
      return -EINVAL;
    }
  }

  if (extra_param.Exists(QMMF_FORCE_SENSOR_MODE)) {
    size_t entry_count = extra_param.EntryCount(QMMF_FORCE_SENSOR_MODE);
    if (entry_count == 1) {
      ForceSensorMode force_sensor_mode;
      extra_param.Fetch(QMMF_FORCE_SENSOR_MODE, force_sensor_mode, 0);
      if (force_sensor_mode.mode >= 0) {
        camera_parameters_.cam_feature_flags |=
            (FORCE_SENSOR_MODE_DATA(force_sensor_mode.mode) |
            static_cast<uint32_t>(CamFeatureFlag::kForceSensorMode));
        QMMF_INFO("%s: Force sensor mode(%d) received",
                  __func__, force_sensor_mode.mode);
      } else {
        QMMF_WARN("%s: Invalid sensor mode(%i) received, "
                  "falling back to auto mode selection",
                  __func__, force_sensor_mode.mode);
      }
    } else {
      QMMF_ERROR("%s: Invalid sensor mode received", __func__);
      return -EINVAL;
    }
  }

  if (extra_param.Exists(QMMF_EIS)) {
    size_t entry_count = extra_param.EntryCount(QMMF_EIS);
    if (entry_count == 1) {
      EISSetup eis_mode;
      extra_param.Fetch(QMMF_EIS, eis_mode, 0);
      if (eis_mode.enable == true) {
        QMMF_INFO("%s: EIS is ON..", __func__);
        camera_parameters_.cam_feature_flags |=
            static_cast<uint32_t>(CamFeatureFlag::kEIS);
      }
    } else {
      QMMF_ERROR("%s: Invalid EIS mode received", __func__);
      return -EINVAL;
    }
  }

#ifdef EIS_MODES_ENABLE
  if (extra_param.Exists(QMMF_EIS_MODE)) {
    size_t entry_count = extra_param.EntryCount(QMMF_EIS_MODE);
    if (entry_count == 1) {
      EISModeSetup eis_mode;
      extra_param.Fetch(QMMF_EIS_MODE, eis_mode, 0);
      if (eis_mode.mode == EisMode::kEisOff) {
        QMMF_INFO("%s: EIS is disabled", __func__);
      } else if (eis_mode.mode == EisMode::kEisSingleStream) {
        QMMF_INFO("%s: EIS on single stream is ON..", __func__);
        camera_parameters_.cam_feature_flags |=
            static_cast<uint32_t>(CamFeatureFlag::kEISSingleStream);
      } else if (eis_mode.mode == EisMode::kEisDualStream) {
        QMMF_INFO("%s: EIS on dual stream is ON..", __func__);
        camera_parameters_.cam_feature_flags |=
            static_cast<uint32_t>(CamFeatureFlag::kEISDualStream);
      } else {
        QMMF_ERROR("%s: Invalid EIS mode %d",
              __func__, static_cast<int32_t>(eis_mode.mode));
      }
    } else {
      QMMF_ERROR("%s: Invalid EIS mode received", __func__);
      return -EINVAL;
    }
  }
#endif // EIS_MODES_ENABLE

  if (extra_param.Exists(QMMF_LDC)) {
    size_t entry_count = extra_param.EntryCount(QMMF_LDC);
    if (entry_count == 1) {
      LDCMode ldc_mode;
      extra_param.Fetch(QMMF_LDC, ldc_mode, 0);
      if (ldc_mode.enable == true) {
        QMMF_INFO("%s: LDC is ON..", __func__);
        camera_parameters_.cam_feature_flags |=
            static_cast<uint32_t>(CamFeatureFlag::kLDC);
      }
    } else {
      QMMF_ERROR("%s: Invalid LDC mode received", __func__);
      return -EINVAL;
    }
  }

  if (extra_param.Exists(QMMF_LCAC)) {
    size_t entry_count = extra_param.EntryCount(QMMF_LCAC);
    if (entry_count == 1) {
      LCACMode lcac_mode;
      extra_param.Fetch(QMMF_LCAC, lcac_mode, 0);
      if (lcac_mode.enable == true) {
        QMMF_INFO("%s: LCAC is ON..", __func__);
        camera_parameters_.cam_feature_flags |=
            static_cast<uint32_t>(CamFeatureFlag::kLCAC);
      }
    } else {
      QMMF_ERROR("%s: Invalid LCAC mode received", __func__);
      return -EINVAL;
    }
  }

  if (extra_param.Exists(QMMF_PARTIAL_METADATA)) {
    size_t entry_count = extra_param.EntryCount(QMMF_PARTIAL_METADATA);
    if (entry_count == 1) {
      PartialMetadata partial_metadata;
      extra_param.Fetch(QMMF_PARTIAL_METADATA, partial_metadata, 0);
      if (partial_metadata.enable == true) {
        QMMF_INFO("%s: PartialMetadata is ON..", __func__);
        is_partial_metadata_enabled_ = true;
      }
    } else {
      QMMF_ERROR("%s: Invalid partial metadata received", __func__);
      return -EINVAL;
    }
  }

  if (extra_param.Exists(QMMF_FRAME_RATE_CONTROL)) {
    size_t entry_count = extra_param.EntryCount(QMMF_FRAME_RATE_CONTROL);
    if (entry_count == 1) {
      FrameRateControl frc_mode;
      extra_param.Fetch(QMMF_FRAME_RATE_CONTROL, frc_mode, 0);
      if (frc_mode.mode == FrameRateControlMode::kCaptureRequest) {
        QMMF_INFO("%s: PCR FRC enable", __func__);
        pcr_frc_enabled_ = true;
      } else {
        QMMF_INFO("%s: PCR FRC disable", __func__);
        pcr_frc_enabled_ = false;
      }
    } else {
      QMMF_ERROR("%s: Invalid FRC mode received", __func__);
      return -EINVAL;
    }
  }

  if (extra_param.Exists(QMMF_IFE_DIRECT_STREAM)) {
    size_t entry_count = extra_param.EntryCount(QMMF_IFE_DIRECT_STREAM);
    if (entry_count == 1) {
      IFEDirectStream ife_direct_stream;
      extra_param.Fetch(QMMF_IFE_DIRECT_STREAM, ife_direct_stream, 0);
      if (ife_direct_stream.enable == true) {
        QMMF_INFO("%s: IFE Direct Stream is ON..", __func__);
        camera_parameters_.cam_feature_flags |=
            static_cast<uint32_t>(CamFeatureFlag::kIFEDirectStream);
      }
    } else {
      QMMF_ERROR("%s: Invalid IFE Direct Stream param received", __func__);
      return -EINVAL;
    }
  }

  if (extra_param.Exists(QMMF_CAM_OP_MODE_CONTROL)) {
    size_t entry_count = extra_param.EntryCount(QMMF_CAM_OP_MODE_CONTROL);
    if (entry_count == 1) {
      CamOpModeControl mode_control;

      extra_param.Fetch(QMMF_CAM_OP_MODE_CONTROL, mode_control, 0);
      switch (mode_control.mode) {
        case CamOpMode::kNone:
          camera_parameters_.cam_opmode =
            CamOperationMode::kCamOperationModeNone;
          break;
        case CamOpMode::kFrameSelection:
          camera_parameters_.cam_opmode =
            CamOperationMode::kCamOperationModeFrameSelection;
          break;
        case CamOpMode::kFastSwitch:
          camera_parameters_.cam_opmode =
            CamOperationMode::kCamOperationModeFastSwitch;
          break;
        default:
          QMMF_ERROR("%s: Invalid camera operation mode %d",
              __func__, mode_control.mode);
          break;
      }
    } else {
      QMMF_ERROR("%s: Invalid camera operation mode received", __func__);
      return -EINVAL;
    }
  }

  if (extra_param.Exists(QMMF_INPUT_ROI)) {
    size_t entry_count = extra_param.EntryCount(QMMF_INPUT_ROI);
    if (entry_count == 1) {
      InputROISetup input_roi;
      extra_param.Fetch(QMMF_INPUT_ROI, input_roi, 0);
      if (input_roi.enable == true) {
        QMMF_INFO("%s: Input ROI reprocess usecase is ON..", __func__);
        camera_parameters_.cam_feature_flags |=
            static_cast<uint32_t>(CamFeatureFlag::kInputROIEnable);
      }
    } else {
      QMMF_ERROR("%s: Invalid Input ROI param received", __func__);
      return -EINVAL;
    }
  }

  camera_parameters_.batch_size = 1;

  if (!camera_device_) {
    QMMF_ERROR("%s: Camera device was not created successfully!", __func__);
    return -ENODEV;
  }

  ret = camera_device_->OpenCamera(camera_id);
  if (ret !=  0) {
    QMMF_ERROR("%s: Failed to open camera!", __func__);
    return ret;
  }

  camera_id_ = camera_id;

  ret = camera_device_->GetCameraInfo(camera_id, &static_meta_);
  if (ret !=  0) {
    QMMF_ERROR("%s: Failed to Get Camera Info!", __func__);
    return ret;
  }

#ifndef FLUSH_RESTART_NOTAVAILABLE
  ret = DisableFlushRestart(true, static_meta_);
  assert(ret == 0);
#endif

  InitSupportedFPS();
  assert(!supported_fps_.empty());
  InitHFRModes();

  {
    camera_metadata_entry partial_result_count =
        static_meta_.find(ANDROID_REQUEST_PARTIAL_RESULT_COUNT);
    if (partial_result_count.count > 0) {
      partial_result_count_ = partial_result_count.data.i32[0];
    }
  }

  ret = CreateCaptureRequest(snapshot_request_,
                             CAMERA3_TEMPLATE_STILL_CAPTURE);
  assert(ret == 0);
  QMMF_INFO("%s: Non-zsl snapshot capture request created successfully!",
      __func__);

  result_cb_ = cb;
  error_cb_ = errcb;

  return ret;
}

void CameraContext::InitHFRModes() {
  uint32_t width_offset = 0;
  uint32_t height_offset = 1;
  uint32_t min_fps_offset = 2;
  uint32_t max_fps_offset = 3;
  uint32_t batch_size_offset = 4;
  uint32_t hfr_size = 5;

  camera_metadata_entry meta_entry =
      static_meta_.find(ANDROID_REQUEST_AVAILABLE_CAPABILITIES);
  for (uint32_t i = 0; i < meta_entry.count; ++i) {
    uint8_t caps = meta_entry.data.u8[i];
    if (ANDROID_REQUEST_AVAILABLE_CAPABILITIES_CONSTRAINED_HIGH_SPEED_VIDEO ==
        caps) {
      hfr_supported_ = true;
      break;
    }
  }
  if (!hfr_supported_) {
    return;
  }

  meta_entry = static_meta_.find(
      ANDROID_CONTROL_AVAILABLE_HIGH_SPEED_VIDEO_CONFIGURATIONS);
  for (uint32_t i = 0; i < meta_entry.count; i += hfr_size) {
    uint32_t width = meta_entry.data.i32[i + width_offset];
    uint32_t height = meta_entry.data.i32[i + height_offset];
    uint32_t min_fps = meta_entry.data.i32[i + min_fps_offset];
    uint32_t max_fps = meta_entry.data.i32[i + max_fps_offset];
    uint32_t batch = meta_entry.data.i32[i + batch_size_offset];
    if (min_fps == max_fps) { //Only constant framerates are supported
      HFRMode_t mode = {width, height, batch, min_fps};
      hfr_batch_modes_list_.push_back(mode);
    }
  }
}

status_t CameraContext::CloseCamera(const uint32_t camera_id) {

  QMMF_INFO("%s: Enter", __func__);
  int32_t ret = 0;
  assert(camera_id_ == camera_id);

  if (!camera_device_) {
    QMMF_ERROR("%s: Camera device was not created successfully!", __func__);
    return -ENODEV;
  }

  if (streaming_request_id_ > 0) {
    QMMF_ERROR("%s: Streaming Request still running! delete all tracks "
    "before closing camera",  __func__);
    return -ENOSYS;
  }

  ret = camera_device_->WaitUntilIdle();

  last_frame_number_ = NO_IN_FLIGHT_REPEATING_FRAMES;

  QMMF_INFO("%s: Camera Closed Succussfully!", __func__);
  return ret;
}

std::function<void(StreamBuffer buffer)>
    CameraContext::GetStreamCb(const SnapshotParam& param) {
  return [=](StreamBuffer buffer) { SnapshotCaptureCallback (buffer); };
}

status_t CameraContext::WaitAecToConverge(const uint32_t timeout) {

  QMMF_DEBUG("%s: Enter ", __func__);
  if (streaming_request_id_ == -1) {
    QMMF_INFO("%s: No active streams, skip wait!", __func__);
    return 0;
  }

  std::unique_lock<std::mutex> lock(aec_lock_);
  std::chrono::nanoseconds wait_time(timeout);

  while ((aec_.state != ANDROID_CONTROL_AE_STATE_LOCKED) &&
         (aec_.state != ANDROID_CONTROL_AE_STATE_CONVERGED)) {
    if (aec_state_updated_.WaitFor(lock, wait_time) != 0) {
      QMMF_ERROR("%s Timed out on AEC converge Wait", __func__);
      return -ETIMEDOUT;
    }
  }
  QMMF_DEBUG("%s: Exit ", __func__);
  return 0;
}

status_t CameraContext::ValidateResolution(const BufferFormat format,
    const uint32_t width, const uint32_t height) {

  auto ret = Common::ValidateResolution(static_meta_, format, width, height);
  if (ret == false) {
    QMMF_ERROR("%s Unsupported resolution %d x %d format %d!",
        __func__, width, height, format);
    return -EINVAL;
  }
  return 0;
}

status_t CameraContext::ConfigImageCapture(const uint32_t image_id,
                                           const SnapshotParam& param,
                                           const ImageExtraParam &xtraparam) {

  QMMF_DEBUG("%s Enter ", __func__);

  std::unique_lock<std::mutex> lock(capture_lock_);

  if (param.mode != ImageMode::kZsl) {
    auto ret = ValidateResolution(param.format, param.width, param.height);
    if (0 != ret) {
      QMMF_ERROR("%s Failed during snapshot validation", __func__);
      return ret;
    }

    CameraStreamParameters stream_param{};
    ret = GetSnapshotStreamParams(param, stream_param);
    assert(ret == 0);

#ifdef ENABLE_IMAGE_NV12
    if (param.format == BufferFormat::kNV12) {
      stream_param.allocFlags.flags |= IMemAllocUsage::kHwCameraWrite;
      // Not for HEIF, set HAL_DATASPACE_HEIF because of camera limitation.
      stream_param.data_space = static_cast<android_dataspace_t>
                                (HAL_DATASPACE_HEIF);
    } else if (param.format == BufferFormat::kNV12HEIF) {
      stream_param.allocFlags.flags = (IMemAllocUsage::kHwRender |
                                IMemAllocUsage::kPrivateAllocHEIF |
                                IMemAllocUsage::kHwTexture);
      stream_param.data_space = static_cast<android_dataspace_t>
                                (HAL_DATASPACE_HEIF);
    }
#endif

    ret = CreateSnapshotStream(image_id, stream_param, true);
    if (0 != ret) {
      QMMF_ERROR("%s Failed during snapshot re-configure", __func__);
      return ret;
    }

    snapshot_mode_ = param.mode;
    snapshot_quality_ = param.quality;

    // Wait AE to converge after reconfiguration if there are active streams.
    WaitAecToConverge(kWaitAecTimeout);
  } else {
    SnapshotZslSetup zslparam;

    if (xtraparam.Exists(QMMF_SNAPSHOT_ZSL_SETUP)) {
      xtraparam.Fetch(QMMF_SNAPSHOT_ZSL_SETUP, zslparam);
    }

    auto ret = StartZSL(image_id, param, zslparam);
    assert(ret == 0);

    auto zsl_port = std::static_pointer_cast<ZslPort>(GetPort(zsl_port_id_));
    assert(zsl_port.get() != nullptr);

    ret = zsl_port->ValidateCaptureParams(param.width, param.height, param.format);
    if (ret != 0) {
      QMMF_ERROR("%s ZSL validation fails! Stream dim: %dx%d format: %x",
          __func__, param.width, param.height, param.format);
      return ret;
    }
  }
  QMMF_DEBUG("%s Exit ", __func__);
  return 0;
}

status_t CameraContext::CaptureImage(const SnapshotType type,
                                     const uint32_t n_images,
                                     const std::vector<CameraMetadata> &meta,
                                     const StreamSnapshotCb& cb) {

  QMMF_INFO("%s: Enter", __func__);
  int32_t ret = 0;
  uint32_t imgcnt = 0;
  client_snapshot_cb_ = cb;
  capture_cnt_ = 0;

  if (snapshot_request_.streamIds.empty()) {
    QMMF_ERROR("%s: No snapshot stream available", __func__);
    return -EINVAL;
  }

  if (continuous_mode_is_on_) {
    QMMF_WARN("%s: CaptureImage() should be called only once "
        "in continuous capture mode", __func__);
    return 0;
  }

  continuous_mode_is_on_ = (n_images == 0) ? true : false;
  imgcnt = (n_images == 0) ? 1 : n_images;

  if (snapshot_mode_ != ImageMode::kZsl) {
    device_access_lock_.lock();
    int64_t last_frame_number;
    uint8_t jpeg_quality = snapshot_quality_;
    std::vector<Camera3Request> requests;
    std::vector<CameraMetadata>::const_iterator it = meta.begin();
    for (uint32_t i = 0; i < imgcnt; i++) {
      if (streaming_active_requests_.size() > 0 &&
          !streaming_active_requests_[0].metadata.isEmpty() &&
          (type == SnapshotType::kVideo || type == SnapshotType::kVideoPlusRaw)) {
        snapshot_request_.metadata.clear();
        snapshot_request_.metadata.append(streaming_active_requests_[0].metadata);
      } else if (it != meta.end()) {
        snapshot_request_.metadata.clear();
        snapshot_request_.metadata.append(*it++);
      }
      snapshot_request_.metadata.update(ANDROID_JPEG_QUALITY, &jpeg_quality, 1);
      uint32_t active_streamid_count = 0;

      if (continuous_mode_is_on_ || type == SnapshotType::kVideo ||
          type == SnapshotType::kVideoPlusRaw) {
        if (streaming_active_requests_.size() == 1) {
          if (port_paused_ == true) {
            QMMF_INFO("%s: streaming request is paused need to resume!",
                __func__);
            Camera3Request &req = streaming_active_requests_[0];
            // at this point stream id vector should be empty.
            assert(req.streamIds.size() == 0);
            for (auto stream_id : stopped_stream_ids_) {
              QMMF_INFO("%s: stream_id: %d to resume!", __func__, stream_id);
              // Add paused stream ids to snapshot request so they can be
              // resumed, and capture request for both video and snapshot can
              // go at same time.
              snapshot_request_.streamIds.push_back(stream_id);
              active_streamid_count++;
              // Also add puased stream ids back to streaming request so next
              // Update request will takecare them. eg cancel capture request
              // will call update request to remove snapshot stream id from
              // request and resume the streaming request.
              req.streamIds.push_back(stream_id);
              QMMF_INFO("%s: Added all Request to snapshot request! "
                "active_streamid_count=%d", __func__, active_streamid_count);
            }
            port_paused_ = false;
          } else {
            auto request = streaming_active_requests_[0];
            for (auto stream_id : request.streamIds) {
              snapshot_request_.streamIds.push_back(stream_id);
              active_streamid_count++;
              QMMF_INFO("%s: Added all Request to streaming request! "
                "active_streamid_count=%d", __func__, active_streamid_count);
            }
          }
        } else {
          QMMF_INFO("%s: No other active video streams!", __func__);
        }
      }
      requests.push_back(snapshot_request_);
      snapshot_request_.streamIds.
        resize(snapshot_request_.streamIds.size() - active_streamid_count);
    }

    bool streaming = continuous_mode_is_on_ ? true : false;

    {
      std::unique_lock<std::mutex> lock(capture_lock_);
      std::unique_lock<std::mutex> pending_frames_lock(pending_frames_lock_);
      cancel_capture_ = false;
      auto request_id = camera_device_->SubmitRequestList(requests, streaming,
                                                          &last_frame_number);

      QMMF_INFO("%s: last_frame_number: current=%lld previous=%lld", __func__,
          last_frame_number, last_frame_number_);

      // SubmitRequestList returns NO_IN_FLIGHT_REPEATING_FRAMES when there is
      // no previous request or when previous request is not submitted to HAL
      // yet.
      if (last_frame_number != NO_IN_FLIGHT_REPEATING_FRAMES) {
        last_frame_number_ = last_frame_number;
      }

      assert(request_id >= 0);
      if (streaming) {
        streaming_request_id_ = request_id;
      } else {
        capture_request_id_ = request_id;
      }
    }
    device_access_lock_.unlock();
    QMMF_INFO("%s: Request for non-zsl submitted successfully",
        __func__);

    ResumeActiveStreams(streaming);
  } else {
    ret = CaptureZSLImage(type);
    if (ret != 0) {
      QMMF_ERROR("%s: CaptureImage Failed in ZSL mode!", __func__);
      return ret;
    }
  }
  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

status_t CameraContext::CancelCaptureImage(const uint32_t image_id, 
                                           const bool cache) {

  QMMF_INFO("%s: Enter", __func__);

  if (snapshot_mode_ == ImageMode::kZsl) {
    auto ret = StopZSL(image_id);
    assert(ret == 0);

    // After cancel image capture snapshot mode is not ZSL anymore.
    // Switch mode to default.
    snapshot_mode_ = ImageMode::kSnapshot;
  } else if (!snapshot_request_.streamIds.empty()) {
    {
      std::unique_lock<std::mutex> lock(capture_lock_);
      cancel_capture_ = true;
    }

    continuous_mode_is_on_ = false;

    PauseActiveStreams(!cache);
    if (!cache) {
      DeleteSnapshotStream(image_id);
    }
    ResumeActiveStreams();
  }

  QMMF_INFO("%s: Exit", __func__);
  return 0;
}

void CameraContext::RestoreBatchStreamId(std::shared_ptr<CameraPort>& port) {
  if (!port) {
    QMMF_ERROR("%s: Invalid port", __func__);
    return;
  }

  if (batch_stream_id_ == port->GetCameraStreamId()) {
    batch_stream_id_ = -1;
    camera_parameters_.batch_size = 1;
  }
}

void CameraContext::StoreBatchStreamId(std::shared_ptr<CameraPort>& port) {
  assert(port.get() != nullptr);
  if (port->GetPortBatchSize() > 1) {
    if (batch_stream_id_ > -1) {
      QMMF_WARN("%s:The Batch stream is already configuried", __func__);
    } else {
      batch_stream_id_ = port->GetCameraStreamId();
    }
  }
}

status_t CameraContext::GetBatchSize(const StreamParam& param,
                                     uint32_t& batch_size) {

  /* only one batch stream is supported */
  if (camera_parameters_.batch_size > 1) {
    /* set batch size to default */
    batch_size = 1;
    return 0;
  }

  if ((kConstrainedModeThreshold <= param.framerate) && (!hfr_supported_)) {
    QMMF_ERROR("%s: Stream tries to enable HFR which is not supported!",
               __func__);
    return -EINVAL;
  }

  if ((kConstrainedModeThreshold <= param.framerate) &&
      (snapshot_mode_ == ImageMode::kZsl)) {
    QMMF_ERROR("%s: HFR and ZSL are mutually exclusive!",
               __func__);
    return -EINVAL;
  }

  size_t batch = 1;
  if (kHFRBatchModeThreshold <= param.framerate) {
    bool supported = false;
    for (size_t i = 0; i < hfr_batch_modes_list_.size(); i++) {
      if ((param.width == hfr_batch_modes_list_[i].width) &&
          (param.height == hfr_batch_modes_list_[i].height) &&
          fabs(param.framerate - hfr_batch_modes_list_[i].framerate) < 0.1f) {
        batch = hfr_batch_modes_list_[i].batch_size;
        supported = true;
        break;
      }
    }

    if (!supported) {
      QMMF_ERROR("%s: HFR stream with size %dx%d fps: %5.2f is not supported!",
          __func__, param.width, param.height, param.framerate);
      return -EINVAL;
    }
  }

  batch_size = batch;
  camera_parameters_.batch_size = batch_size;

  return 0;
}

status_t CameraContext::CreateStream(const StreamParam& param,
                                     const VideoExtraParam& extra_param) {

  int32_t stream_id = -1;
  QMMF_VERBOSE("%s: Enter", __func__);
  // 1. Check if streaming request already is going on, if yes then cancel it
  //    and reconfigure it with adding new request.
  // 2. Check for available port where consumer can be attached, if not then
  //    Create new one.
  // 3. Create camera adaptor stream.
  // 4. Create port and link it with adaptor stream.
  // 5. Create producer interface in port and link consumer.

  if (!camera_device_) {
    QMMF_ERROR("%s: Camera device was not created successfully!", __func__);
    return -ENODEV;
  }

  assert(param.id != 0);

  uint32_t batch;
  if (0 != GetBatchSize(param, batch)) {
    return -EINVAL;
  }

  camera_parameters_.batch_size = batch;

  // FIXME: HFR control and exception for fastswitch will be removed after
  // session clean-up merged
  if ((camera_parameters_.cam_opmode !=
        CamOperationMode::kCamOperationModeFastSwitch) &&
      (hfr_detected_ == false) && (batch > 1)) {
    QMMF_INFO("%s: HFR stream detected!"
        "track_id = %x", __func__, param.id);
    hfr_detected_ = true;
  }

  std::shared_ptr<CameraPort> port =
      std::make_shared<CameraPort>(param, camera_parameters_,
                                   CameraPortType::kVideo, this);
  assert(port.get() != nullptr);

  auto ret = port->Init();
  if (ret != 0) {
    QMMF_ERROR("%s: CameraPort Can't be Created!", __func__);
    return -EINVAL;
  } else {
    std::lock_guard<std::mutex> lk(prepare_lock_);
    char prop[PROP_VALUE_MAX];

    stream_id = port->GetCameraStreamId();
    property_get("persist.qmmf.static.mem.alloc", prop, "0");
    stream_prepared_[stream_id] = (std::stoi(prop) == 0) ? true : false;

    if (!stream_prepared_[stream_id]) {
      std::lock_guard<std::mutex> lk(device_access_lock_);
      ret = camera_device_->Prepare(stream_id);
      assert(ret == 0);
    }
  }

  StoreBatchStreamId(port);

  // Create global streaming capture request, this capture request would be
  // Common to all video/preview and zsl snapshot stream. non zsl snapshot
  // will have separate capture request.
  if (streaming_active_requests_.empty()) {
    streaming_active_requests_.emplace_back();
    ret = CreateCaptureRequest(streaming_active_requests_[0],
                               CAMERA3_TEMPLATE_VIDEO_SNAPSHOT);
    assert(ret == 0);
    QMMF_INFO("%s: Global Streaming request created successfully!",__func__);
  }

  if (enable_reproc_) {
    reproc_out_stream_ids_.push_back(stream_id);
  }

  if (static_cast<bool>(param.flags & VideoFlags::kReproc)) {
    enable_reproc_ = true;
  }

  // Add port to list of active ports.
  active_ports_.emplace(param.id, port);

  QMMF_INFO("%s: Number of Active ports=%d", __func__, active_ports_.size());

  QMMF_VERBOSE("%s: Exit", __func__);
  return ret;
}

status_t CameraContext::DeleteStream(const uint32_t track_id) {

  auto port = GetPort(track_id);
  if (!port) {
    QMMF_ERROR("%s: Invalid track_id(%x)", __func__, track_id);
    return -EINVAL;
  }

  if (port->GetNumConsumers() > 0) {
    // Port still being used by another consumer, eventually this port would be
    // deleted once consumers count would become zero.
    return 0;
  }

  auto ret = port->DeInit();
  if (ret != 0) {
    QMMF_ERROR("%s: Port DeInit failed!!", __func__);
    return ret;
  }

  RestoreBatchStreamId(port);
  active_ports_.erase(track_id);

  QMMF_INFO("%s: Camera Port for track_id(%x) deleted", __func__, track_id);
  return ret;
}

status_t CameraContext::AddConsumer(const uint32_t& track_id,
                                    std::shared_ptr<IBufferConsumer>& consumer) {

  auto port = GetPort(track_id);
  if (!port) {
    QMMF_ERROR("%s: Invalid track_id(%x)", __func__, track_id);
    return -EINVAL;
  }

  auto ret = port->AddConsumer(consumer);
  assert(ret == 0);
  QMMF_INFO("%s: Consumer(%p) added to track_id(%d)", __func__,
      consumer.get(), track_id);
  return 0;
}

status_t CameraContext::RemoveConsumer(const uint32_t& track_id,
                                       std::shared_ptr<IBufferConsumer>& consumer) {

  auto port = GetPort(track_id);
  if (!port) {
    QMMF_ERROR("%s: Invalid track_id(%x)", __func__, track_id);
    return -EINVAL;
  }

  auto ret = port->RemoveConsumer(consumer);
  assert(ret == 0);
  return 0;
}

status_t CameraContext::StartStream(const uint32_t track_id) {

  auto port = GetPort(track_id);
  if (!port) {
    QMMF_ERROR("%s: Invalid track_id(%x)", __func__, track_id);
    return -EINVAL;
  }

  auto ret = port->Start();
  assert(ret == 0);
  QMMF_INFO("%s: track_id(%d) started on port(0x%p)", __func__,
      track_id, port.get());
  return 0;
}

status_t CameraContext::StopStream(const uint32_t track_id) {

  QMMF_DEBUG("%s: Enter", __func__);
  auto port = GetPort(track_id);
  if (!port) {
    QMMF_ERROR("%s: Invalid track_id(%x)", __func__, track_id);
    return -EINVAL;
  }

  auto ret = port->Stop();
  if (ret != 0) {
    QMMF_ERROR("%s: Port Stop failed!!", __func__);
    return ret;
  }

  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}

status_t CameraContext::PauseStream(const uint32_t track_id) {

  QMMF_DEBUG("%s: Enter", __func__);
  auto port = GetPort(track_id);
  if (!port) {
    QMMF_ERROR("%s: Invalid track_id(%x)", __func__, track_id);
    return -EINVAL;
  }

  auto ret = port->Pause();
  assert(ret == 0);
  QMMF_DEBUG("%s: Exit", __func__);
  return 0;
}

status_t CameraContext::ResumeStream(const uint32_t track_id) {

  QMMF_DEBUG("%s: Enter", __func__);
  auto port = GetPort(track_id);
  if (!port) {
    QMMF_ERROR("%s: Invalid track_id(%x)", __func__, track_id);
    return -EINVAL;
  }

  auto ret = port->Resume();
  assert(ret == 0);
  QMMF_DEBUG("%s: Exit", __func__);
  return 0;
}

status_t CameraContext::SetCameraParam(const CameraMetadata &meta) {

  QMMF_DEBUG("%s: Enter", __func__);

  uint32_t tag_id = 0;
  bool     is_standby = false;
  const std::shared_ptr<VendorTagDescriptor> vtags =
      VendorTagDescriptor::getGlobalVendorTagDescriptor();

  if (vtags.get() == NULL) {
    QMMF_ERROR ("Failed to retrieve Global Vendor Tag Descriptor!");
    return -1;
  }

  // If standby metadata is present, then cancel requests
  if ((meta.getTagFromName(
      "org.codeaurora.qcamera3.sensorwriteinput.SensorStandByFlag",
      vtags.get(), &tag_id) == 0) &&  meta.exists(tag_id) &&
      meta.find(tag_id).data.u8[0] != 0) {
    CancelRequest();
    streaming_request_id_ = 0;
    is_standby = true;
  }

  // Check if Multi ROI info is present and store it in global variable.
  if ((meta.getTagFromName(
      "com.qti.camera.multiROIinfo.streamROICount",
      vtags.get(), &tag_id) == 0) &&  meta.exists(tag_id)) {
    multi_roi_count_ = meta.find(tag_id).data.i32[0];
    multi_roi_count_tag_ = tag_id;
  }

  if ((meta.getTagFromName(
      "com.qti.camera.multiROIinfo.streamROIInfo",
      vtags.get(), &tag_id) == 0) &&  meta.exists(tag_id)) {
    multi_roi_info_.clear();
    for (int i = 0; i < multi_roi_count_ * 4; i++){
      multi_roi_info_.push_back(meta.find(tag_id).data.i32[i]);
    }
    multi_roi_info_tag_ = tag_id;
  }


  std::lock_guard<std::mutex> lock(device_access_lock_);
  if ((!streaming_active_requests_.empty()) &&
      (!streaming_active_requests_[0].metadata.isEmpty())) {
    std::vector<Camera3Request> request_list;
    Camera3Request request;
    CameraMetadata metadata(meta);

    // Remove single shot meta entries and place them in separate request.
    for (auto& pair : kSingleShotMeta) {

      if ((meta.getTagFromName(pair.first, vtags.get(), &tag_id) != 0) ||
          !meta.exists(tag_id) || (meta.find(tag_id).data.u8[0] == pair.second)) {
        continue;
      }

      // Append single shot metadata.
      request = streaming_active_requests_[0];
      request.metadata.clear();
      request.metadata.append(meta);

      // Reset the single shot meta tag to the default value in local metadata.
      metadata.update(tag_id, &(pair.second), 1);
    }

    for (Camera3Request& req : streaming_active_requests_) {
      req.metadata.clear();
      req.metadata.append(metadata);
      request_list.push_back(req);
    }

    // when there're multiple streams with HFR stremas invovled
    // batch size will be more than one, in this case, camx requires
    // strict order of buffer numbers
    // SetcameraParam will be triggered after one stream is ready
    // and it will submit request to capture-request handler which will
    // provide buffers into camx, this violate rule for camx
    // adding hfr_detected_ to block requests untill all ports are ready
    if ((hfr_detected_ == false) ||
          ((hfr_detected_ == true) && (hfr_wait_ports_ready_ == true))) {

      // Submit request with updated camera meta data only if streaming is
      // started, if not then just update default meta data and leave it to
      // startSession -> startStream to submit request.
      std::unique_lock<std::mutex> pending_frames_lock(pending_frames_lock_);
      if (streaming_request_id_ >= 0 && !continuous_mode_is_on_) {
        int64_t last_frame_number;
        int32_t ret = 0;

        if (!request.metadata.isEmpty()) {
          // Submit one request containing single shot meta entries.
          ret = camera_device_->SubmitRequest(request, false,
                                              &last_frame_number);
          assert(ret >= 0);
        }

        if (!is_standby) {
          ret = camera_device_->SubmitRequestList(request_list, true,
                                                  &last_frame_number);
        }

        QMMF_INFO("%s: last_frame_number: current=%lld previous=%lld", __func__,
            last_frame_number, last_frame_number_);

        // SubmitRequestList returns NO_IN_FLIGHT_REPEATING_FRAMES when there is
        // no previous request or when previous request is not submitted to HAL
        // yet.
        if (last_frame_number != NO_IN_FLIGHT_REPEATING_FRAMES) {
          last_frame_number_ = last_frame_number;
        }

        assert(ret >= 0);
        streaming_request_id_ = ret;
      }
    }
  } else {
    QMMF_ERROR("%s: No active requests present!\n", __func__);
    return -ENODEV;
  }
  QMMF_DEBUG("%s: Exit", __func__);
  return 0;
}

status_t CameraContext::GetCameraParam(CameraMetadata &meta) {

  QMMF_DEBUG("%s: Enter", __func__);
  meta.clear();
  if ((!streaming_active_requests_.empty()) &&
      (!streaming_active_requests_[0].metadata.isEmpty())) {
    meta.append(streaming_active_requests_[0].metadata);
  } else {
    QMMF_INFO("%s No active request present. Return static meta!\n", __func__);
    meta.append(static_meta_);
  }
  QMMF_DEBUG("%s: Exit", __func__);
  return 0;
}

status_t CameraContext::SetCameraSessionParam(
    const CameraMetadata &meta) {
  int32_t ret = 0;
  QMMF_DEBUG("%s: Enter", __func__);

  ret = camera_device_->SetCameraSessionParam(meta);
  if (ret != 0)
     QMMF_ERROR("%s Set cammera session metadata failed!\n", __func__);

  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}

status_t CameraContext::GetDefaultCaptureParam(CameraMetadata &meta) {

  QMMF_DEBUG("%s: Enter", __func__);
  auto ret = 0;
  if (!snapshot_request_.metadata.isEmpty()) {
    meta.clear();
    // Append default snapshot meta data.
    meta.append(snapshot_request_.metadata);
  } else {
    QMMF_WARN("%s Camera is not started Or it is started in zsl mode!\n",
        __func__);
    ret = -ENODEV;
  }
  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}

status_t CameraContext::GetCameraCharacteristics(CameraMetadata &meta) {

  QMMF_DEBUG("%s: Enter", __func__);
  meta.clear();
  if (static_meta_.isEmpty()) {
    QMMF_ERROR("%s Static meta is empty!\n", __func__);
    return -ENODEV;
  }
  meta.append(static_meta_);
  QMMF_DEBUG("%s: Exit", __func__);
  return 0;
}

status_t CameraContext::ReturnAllImageCaptureBuffers() {

  QMMF_DEBUG("%s: Enter", __func__);
  status_t ret = 0;
  for (int i = 0; i < snapshot_buffer_list_.size(); i++) {
    auto entry = snapshot_buffer_list_.begin();
    ret = ReturnImageCaptureBuffer(0, entry->first);
    assert(ret == 0);
  }
  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}

status_t CameraContext::ReturnImageCaptureBuffer(const uint32_t camera_id,
                                                 const int32_t buffer_id) {

  std::lock_guard<std::mutex> lock(device_access_lock_);
  QMMF_DEBUG("%s: Enter", __func__);
  if (snapshot_buffer_list_.find(buffer_id) == snapshot_buffer_list_.end()) {
    QMMF_ERROR("%s: buffer_id(%u) is not valid!!", __func__, buffer_id);
    return -EINVAL;
  }

  StreamBuffer buffer = snapshot_buffer_list_.find(buffer_id)->second;
  assert(buffer.fd == buffer_id);


  if (snapshot_buffer_stream_list_.find(buffer_id) ==
      snapshot_buffer_stream_list_.end()) {
    QMMF_ERROR("%s: buffer_id(%u) is not valid!!", __func__, buffer_id);
    return -EINVAL;
  }
  int32_t stream_id = snapshot_buffer_stream_list_.find(buffer_id)->second;

  QMMF_DEBUG("%s: stream_id(%d):stream_buffer(0x%p):ion_fd(%d)"
      " returned back!",  __func__, stream_id, buffer.handle, buffer_id);

  status_t ret = 0;
  ret = camera_device_->ReturnStreamBuffer(buffer);

  QMMF_DEBUG("%s: ret %d", __func__, ret);
  assert(ret == 0);

  snapshot_buffer_list_.erase(buffer_id);
  snapshot_buffer_stream_list_.erase(buffer_id);

  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}

std::vector<int32_t>& CameraContext::GetSupportedFps() {

  return supported_fps_;
}

#ifdef VHDR_MODES_ENABLE
status_t CameraContext::SetVHDR(const int32_t mode) {

  QMMF_DEBUG("%s: Enter", __func__);
  int32_t is_raw = (camera_parameters_.cam_feature_flags &
                        static_cast<uint32_t>(CamFeatureFlag::kSHDRRaw));
  int32_t is_yuv = (camera_parameters_.cam_feature_flags &
                        static_cast<uint32_t>(CamFeatureFlag::kSHDRYUV));
  int32_t is_switch_shdrv2 = (camera_parameters_.cam_feature_flags &
                        static_cast<uint32_t>(CamFeatureFlag::kSHDRRawSwitch));
  int32_t is_switch_shdrv3 = (camera_parameters_.cam_feature_flags &
                        static_cast<uint32_t>(CamFeatureFlag::kSHDRYUVSwitch));
  int32_t is_qbc_vid = (camera_parameters_.cam_feature_flags &
                        static_cast<uint32_t>(CamFeatureFlag::kQBCHDRVideo));
  int32_t is_qbc_snap = (camera_parameters_.cam_feature_flags &
                        static_cast<uint32_t>(CamFeatureFlag::kQBCHDRSnapshot));

  if (mode == static_cast<int32_t>(VHDRMode::kVHDROff)) {
    if (!is_raw && !is_yuv && !is_switch_shdrv2 && !is_switch_shdrv3 &&
            !is_qbc_vid && !is_qbc_snap) {
      QMMF_DEBUG("%s: VHDR is already disabled", __func__);
      return 0;
    }
  }

  // Reset all modes
  camera_parameters_.cam_feature_flags &=
    ~(static_cast<uint32_t>(CamFeatureFlag::kSHDRRaw));
  camera_parameters_.cam_feature_flags &=
    ~(static_cast<uint32_t>(CamFeatureFlag::kSHDRYUV));
  camera_parameters_.cam_feature_flags &=
    ~(static_cast<uint32_t>(CamFeatureFlag::kSHDRRawSwitch));
  camera_parameters_.cam_feature_flags &=
    ~(static_cast<uint32_t>(CamFeatureFlag::kSHDRYUVSwitch));
  camera_parameters_.cam_feature_flags &=
    ~(static_cast<uint32_t>(CamFeatureFlag::kQBCHDRVideo));
  camera_parameters_.cam_feature_flags &=
    ~(static_cast<uint32_t>(CamFeatureFlag::kQBCHDRSnapshot));

  if (mode == static_cast<int32_t>(VHDRMode::kSHDRRaw)) {
    camera_parameters_.cam_feature_flags |=
      static_cast<uint32_t>(CamFeatureFlag::kSHDRRaw);
    if (is_raw) {
      QMMF_DEBUG("%s: SHDR mode is already RAW", __func__);
      return 0;
    }
  } else if (mode == static_cast<int32_t>(VHDRMode::kSHDRYuv)) {
    camera_parameters_.cam_feature_flags |=
      static_cast<uint32_t>(CamFeatureFlag::kSHDRYUV);
    if (is_yuv) {
      QMMF_DEBUG("%s: SHDR mode is already YUV", __func__);
      return 0;
    }
  } else if (mode == static_cast<int32_t>(VHDRMode::kSHDRRawSwitchEnable)) {
    camera_parameters_.cam_feature_flags |=
      static_cast<uint32_t>(CamFeatureFlag::kSHDRRawSwitch);
    if (is_switch_shdrv2) {
      QMMF_DEBUG("%s: Raw SHDR switch is already enabled", __func__);
      return 0;
    }
  } else if (mode == static_cast<int32_t>(VHDRMode::kSHDRYUVSwitchEnable)) {
    camera_parameters_.cam_feature_flags |=
      static_cast<uint32_t>(CamFeatureFlag::kSHDRYUVSwitch);
    if (is_switch_shdrv3) {
      QMMF_DEBUG("%s: YUV SHDR switch is already enabled", __func__);
      return 0;
    }
  } else if (mode == static_cast<int32_t>(VHDRMode::kQBCHDRVideo)) {
    camera_parameters_.cam_feature_flags |=
      static_cast<uint32_t>(CamFeatureFlag::kQBCHDRVideo);
    if (is_qbc_vid) {
      QMMF_DEBUG("%s: QBC HDR mode is already video", __func__);
      return 0;
    }
  } else if (mode == static_cast<int32_t>(VHDRMode::kQBCHDRSnapshot)) {
    camera_parameters_.cam_feature_flags |=
      static_cast<uint32_t>(CamFeatureFlag::kQBCHDRSnapshot);
    if (is_qbc_snap) {
      QMMF_DEBUG("%s: QBC HDR mode is already snapshot", __func__);
      return 0;
    }
  }

  if (!streaming_active_requests_[0].streamIds.size()) {
    QMMF_DEBUG("%s: No active streams. Update only cam params.", __func__);
    return 0;
  }

  PauseActiveStreams();

  {
    std::lock_guard<std::mutex> lock(device_access_lock_);
    camera_device_->UpdateCameraParams(camera_parameters_);

    QMMF_DEBUG("%s: SHDR enable: %d", __func__, mode);
  }

  ResumeActiveStreams();

  QMMF_DEBUG("%s: Exit", __func__);
  return 0;
}
#else
status_t CameraContext::SetSHDR(const bool enable) {

  QMMF_DEBUG("%s: Enter", __func__);

  bool is_shdr_enable = (camera_parameters_.cam_feature_flags &
                        static_cast<uint32_t>(CamFeatureFlag::kHDR));

  if (is_shdr_enable == enable) {
    QMMF_DEBUG("%s: SHDR is already %d", __func__, enable);
    return 0;
  }

  if (enable) {
    camera_parameters_.cam_feature_flags |=
        static_cast<uint32_t>(CamFeatureFlag::kHDR);
  } else  {
    camera_parameters_.cam_feature_flags &=
        ~(static_cast<uint32_t>(CamFeatureFlag::kHDR));
  }

  if (!streaming_active_requests_[0].streamIds.size()) {
    QMMF_DEBUG("%s: No active streams. Update only cam params.", __func__);
    return 0;
  }

  PauseActiveStreams();

  {
    std::lock_guard<std::mutex> lock(device_access_lock_);
    camera_device_->UpdateCameraParams(camera_parameters_);

    QMMF_DEBUG("%s: SHDR enable: %d", __func__, enable);
  }

  ResumeActiveStreams();

  QMMF_DEBUG("%s: Exit", __func__);
  return 0;
}
#endif // VHDR_MODES_ENABLE

bool CameraContext::IsRawOnly(const int32_t format) {
  switch(format) {
    case HAL_PIXEL_FORMAT_RAW8:
    case HAL_PIXEL_FORMAT_RAW10:
    case HAL_PIXEL_FORMAT_RAW12:
    case HAL_PIXEL_FORMAT_RAW16:
      return true;
  }
  return false;
}

status_t CameraContext::CreateDeviceStream(CameraStreamParameters& params,
                                           uint32_t frame_rate,
                                           int32_t* stream_id,
                                           bool cache) {

  std::lock_guard<std::mutex> lock(device_access_lock_);
  QMMF_VERBOSE("%s: Enter", __func__);

  int32_t ret = 0;

  if (!camera_device_) {
    QMMF_ERROR("%s: Camera device was not created successfully!", __func__);
    return -ENODEV;
  }

  if (snapshot_mode_ == ImageMode::kZsl
      && GetPort(zsl_port_id_).get() != nullptr) {
    auto zsl_port = std::static_pointer_cast<ZslPort>(GetPort(zsl_port_id_));
    if (zsl_port->IsRunning()) {
      QMMF_INFO("%s: ZSL is running, pause and flush queue!",
          __func__);
      ret = zsl_port->PauseAndFlushZSLQueue();
      if (ret != 0) {
        QMMF_ERROR("%s: zsl queue is not flashed!", __func__);
        return ret;
      }
    }
  }

  // Configure is required only once, if streaming request is already submitted
  // then BeginConfigure is not required to be called, stream can be created
  // without calling it.
  if (streaming_request_id_ < 0 && !cache) {
    ret = camera_device_->BeginConfigure();
    assert(ret == 0);
  }

  int32_t id;
  id = camera_device_->CreateStream(params);
  if (id < 0) {
    QMMF_INFO("%s: createStream failed!!", __func__);
    return -EINVAL;
  }
  *stream_id = id;

  // At this point stream is created but it is not added to request.
  // If streaming is active camera adapter trigger configure
  // stream automatically. Because of that we cannot call
  // EndConfigure in this situation.
  if (streaming_request_id_ < 0) {
    if (hfr_supported_) {
      if (kConstrainedModeThreshold <= frame_rate) {
        camera_parameters_.is_constrained_high_speed = true;
      } else {
        for (auto const& it : active_ports_) {
          auto& port = it.second;
          if (kConstrainedModeThreshold <= port->GetPortFramerate()) {
            camera_parameters_.is_constrained_high_speed = true;
            break;
          }
        }
      }
    }

    QMMF_VERBOSE("%s: is_constrained_high_speed(%d)", __func__,
        camera_parameters_.is_constrained_high_speed);

    uint32_t max_frame_rate = frame_rate;
#ifdef USE_FPS_IDX
    // 60-90 fps is consider HFR in some target whereas normal in other target
    // Hence for HFR mode we set OpMode 0x1 whereas in case of other targets
    // OpMode is set as index of given fps sensormode in the sensor mode table.

    for (auto const& it : active_ports_) {
      auto& port = it.second;
      uint32_t port_frm_rate = port->GetPortFramerate();
      if (port_frm_rate > max_frame_rate) {
        max_frame_rate = port_frm_rate;
      }
    }
    QMMF_DEBUG("%s: Max fps (%u)!!", __func__, max_frame_rate);

    if (max_frame_rate > 30 && max_frame_rate < kConstrainedModeThreshold) {
      camera_parameters_.fps_sensormode_index = GetSensorModeIndex(params.width,
          params.height, max_frame_rate);
      QMMF_DEBUG("%s: Sensor mode index (%u) for fps=%u!!", __func__,
          camera_parameters_.fps_sensormode_index, max_frame_rate);
    }
#endif
    camera_parameters_.frame_rate_range[0] = max_frame_rate;
    camera_parameters_.frame_rate_range[1] = max_frame_rate;
    camera_parameters_.is_raw_only = IsRawOnly(params.format);

    if (!cache) {
      pending_cached_stream_ = false;
      ret = camera_device_->EndConfigure(camera_parameters_);
      assert(ret == 0);

      // By default stream is prepared.
      stream_prepared_[id] = true;
    } else {
      pending_cached_stream_ = true;
    }
  }

  if (snapshot_mode_ == ImageMode::kZsl &&
      GetPort(zsl_port_id_).get() != nullptr && !cache) {
    auto zsl_port = std::static_pointer_cast<ZslPort>(GetPort(zsl_port_id_));
    QMMF_INFO("%s: Resume ZSL!", __func__);
    zsl_port->ResumeZSL();
  }

  QMMF_VERBOSE("%s: Exit", __func__);
  return ret;
}

#ifdef USE_FPS_IDX
uint32_t CameraContext::GetSensorModeIndex(uint32_t width, uint32_t height,
    uint32_t fps) {

  std::string tag_name("SensorModeTable");
  std::string section_name("org.quic.camera2.sensormode.info");
  uint32_t sensor_mode_table_tagid;
  std::shared_ptr<VendorTagDescriptor> vendor_tag_desc =
      VendorTagDescriptor::getGlobalVendorTagDescriptor();
  if (nullptr == vendor_tag_desc.get()) {
    QMMF_INFO("%s: no global vendor tag descriptor", __func__);
    return 0;
  }

  status_t result = vendor_tag_desc->lookupTag(tag_name, section_name,
                                               &sensor_mode_table_tagid);
  if (result != 0) {
    QMMF_INFO("%s: no sensor mode info", __func__);
    return 0;
  }

  camera_metadata_entry_t entry = static_meta_.find(sensor_mode_table_tagid);
  if (!entry.count) {
    QMMF_INFO("%s: no sensor mode count 0", __func__);
    return 0;
  }

  const int32_t *sensor_mode_table = entry.data.i32;
  int mode_count = sensor_mode_table[0];
  int mode_size = sensor_mode_table[1];
  int sensor_mode = -1;
  int s_width, s_height, s_fps, matched_fps;

  matched_fps = MAX_SENSOR_FPS;

  for (int i = 0; i < mode_count; i++) {
    s_width  =  sensor_mode_table[2 + i * mode_size];
    s_height =  sensor_mode_table[3 + i * mode_size];
    s_fps    =  sensor_mode_table[4 + i * mode_size];

    if ((s_width >= width) &&
        (s_height >= height) &&
        (s_fps >= fps) &&
        (s_fps <= matched_fps)) {
      matched_fps = s_fps;
      sensor_mode = i;
    }
  }

  if (sensor_mode > -1) {
    QMMF_INFO("%s: SELECTED SENSOR MODE WIDTH:%d HEIGHT:%d FPS:%d", __func__,
      sensor_mode_table[2 + sensor_mode * mode_size],
      sensor_mode_table[3 + sensor_mode * mode_size],
      sensor_mode_table[4 + sensor_mode * mode_size]);
  }

  // We have to incrase mode by 1 because sensor modes start from 1
  return sensor_mode + 1;
}
#endif

status_t CameraContext::CreateDeviceInputStream(
    CameraInputStreamParameters& params, int32_t* stream_id, bool cache) {
  std::lock_guard<std::mutex> lock(device_access_lock_);
  QMMF_INFO("%s: Enter", __func__);

  int32_t ret = 0;

  if (!camera_device_) {
    QMMF_ERROR("%s: Camera device was not created successfully!", __func__);
    return -ENODEV;
  }

  // Configure is required only once, if streaming request is already submitted
  // then BeginConfigure is not required to be called, stream can be created
  // without calling it.
  if (streaming_request_id_ < 0 && !cache) {
    ret = camera_device_->BeginConfigure();
    assert(ret == 0);
  }

  int32_t id;
  id = camera_device_->CreateInputStream(params);
  if (id < 0) {
    QMMF_INFO("%s: createStream failed!!", __func__);
    return -EINVAL;
  }
  *stream_id = id;

  // At this point stream is created but it is not added to request, it will be
  // added once corresponding port will get the start cmd from it's consumer.
  if (streaming_request_id_ < 0) {
    if (!cache) {
      pending_cached_stream_ = false;
      ret = camera_device_->EndConfigure(camera_parameters_);
      assert(ret == 0);
    } else {
      pending_cached_stream_ = true;
    }
  }

  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

status_t CameraContext::DeleteDeviceStream(int32_t stream_id, bool cache) {

  QMMF_VERBOSE("%s: Enter", __func__);
  status_t ret = 0;
  int64_t last_frame_mumber;

  if (!camera_device_) {
    QMMF_ERROR("%s: Camera device was not created successfully!", __func__);
    return -ENODEV;
  }

  bool resume_streaming = false;
  if (snapshot_mode_ == ImageMode::kZsl
      && GetPort(zsl_port_id_).get() != nullptr
      && (0 <= streaming_request_id_)) {

    auto zsl_port = std::static_pointer_cast<ZslPort>(GetPort(zsl_port_id_));
    if (zsl_port->IsRunning()) {
      QMMF_INFO("%s: ZSL is running, pause and flush queue!",
        __func__);
      auto ret = zsl_port->PauseAndFlushZSLQueue();
      if (ret != 0) {
        QMMF_ERROR("%s: zsl queue is not flashed!", __func__);
        return ret;
      }
      QMMF_INFO("%s: Cancelling Request!!", __func__);
      ret = CancelRequest();
      if (0 != ret) {
        QMMF_ERROR("%s Cancel request failed:%d", __func__, ret);
        return ret;
      }
      resume_streaming = true;
    }
  }

  std::lock_guard<std::mutex> lock(device_access_lock_);

  if (snapshot_mode_ == ImageMode::kZsl
      && GetPort(zsl_port_id_).get() != nullptr) {
    ret = camera_device_->BeginConfigure();
    assert(ret == 0);
  }

  if (!is_camera_dead_) {
    ret = camera_device_->DeleteStream(stream_id, cache);
    if (ret != 0) {
      QMMF_ERROR("%s: DeleteStream failed!", __func__);
      return ret;
    }
  }

  QMMF_INFO("%s: Camera Device Stream(%d) deleted successfully!",
      __func__, stream_id);

  if (snapshot_mode_ == ImageMode::kZsl
      && GetPort(zsl_port_id_).get() != nullptr
      && streaming_active_requests_.size() > 0) {
    ret = camera_device_->EndConfigure();

    auto zsl_port = std::static_pointer_cast<ZslPort>(GetPort(zsl_port_id_));
    zsl_port->ResumeZSL();

    if (resume_streaming) {
      ret = camera_device_->SubmitRequest(streaming_active_requests_[0], true,
                                          &last_frame_mumber);
      if (ret != 0) {
        QMMF_ERROR("%s: SubmitRequest failed!!", __func__);
        return ret;
      }

      streaming_request_id_ = ret;
    }
  }

  QMMF_VERBOSE("%s: Exit", __func__);
  return ret;
}

status_t CameraContext::CreateCaptureRequest(Camera3Request& request,
                                             camera3_request_template_t
                                             template_type) {

  std::lock_guard<std::mutex> lock(device_access_lock_);

  auto ret = camera_device_->CreateDefaultRequest(template_type,
      &request.metadata);
  assert(ret == 0);
  return ret;
}

CameraMetadata CameraContext::GetCameraStaticMeta() {
  return static_meta_;
}

status_t CameraContext::SetPerStreamFrameRate() {

  float max_fps = 0;

  if (active_ports_.size() <= 1) {
    // There is only one stream in total. Skip per stream control.
    return 0;
  }

  // We need to calculate frame rate even there is only one active stream
  // because first stream could have lower frame rate then max frame rate.

  std::vector<std::pair<uint32_t, uint32_t>> frame_rate_map;
  for (auto const& it : active_ports_) {
    auto& port = it.second;
    frame_rate_map.push_back(std::pair<uint32_t, uint32_t>(
        port->GetCameraStreamId(), ceil(port->GetPortFramerate() - 0.5)));

    if (max_fps < port->GetPortFramerate()) {
      max_fps = port->GetPortFramerate();
    }
  }

  auto gcd = std::__gcd (frame_rate_map[0].second, frame_rate_map[1].second);
  for (size_t i = 2; i < frame_rate_map.size(); i++) {
    gcd = std::__gcd (gcd, frame_rate_map[i].second);
  }
  uint32_t max_frame_rate = ceil(max_fps - 0.5);
  uint32_t request_count = max_frame_rate / gcd;

  std::vector<std::vector<uint32_t>> request_map;
  request_map.resize(request_count);
  for (auto const& it : frame_rate_map) {
    auto stream_id = it.first;
    auto stream_frame_rate = it.second;

    float frames = static_cast<float>(stream_frame_rate) / gcd;
    float gaps = request_count - frames;
    float ratio = frames / gaps;

    // All streams are part for first request. Start calulation
    // from the second request
    frames--;
    for (size_t i = 1; i < request_count; i++) {
      if (frames / gaps >= ratio) {
        request_map[i].push_back(stream_id);
        frames--;
      } else {
        gaps--;
      }
    }
  }

  streaming_active_requests_.resize(request_map.size());

  for (size_t i = 1; i < request_map.size(); i++) {
    if (streaming_active_requests_[i].metadata.isEmpty()) {
      assert(!streaming_active_requests_[0].metadata.isEmpty());

      streaming_active_requests_[i].metadata.append(
          streaming_active_requests_[0].metadata);

      for (auto stream_id : request_map[i]) {
        streaming_active_requests_[i].streamIds.push_back(stream_id);
      }
    }
  }

  for (size_t i = 0; i < streaming_active_requests_.size(); i++) {
    for (auto id : streaming_active_requests_[i].streamIds) {
      QMMF_INFO ("%s: request %d stream-id %d ", __func__, i, id);
    }
  }

  return 0;
}

status_t CameraContext::UpdateRequest(bool is_streaming) {

  QMMF_DEBUG("%s: Enter", __func__);
  float max_fps = 0;
  std::set<int32_t> stream_ids;
  std::set<int32_t> removed_streams;

  //Get all camera stream ids from all active ports which are ready to start.
  size_t size = active_ports_.size();
  size_t active_ports_number = size;

  QMMF_INFO("%s: Number of active_ports(%d)", __func__, size);

  for (auto const& it : active_ports_) {
    auto& port = it.second;

    int32_t cam_stream_id = port->GetCameraStreamId();
    size_t batch_size = port->GetPortBatchSize();
    QMMF_INFO("%s: cam_stream_id(%d)", __func__, cam_stream_id);

    if (port->getPortState() == PortState::PORT_READYTOSTART) {

      QMMF_INFO("%s: CameraPort(0x%p):camera_stream_id(%d) is ready to"
          " start!",  __func__, port.get(), cam_stream_id);
      if (max_fps < port->GetPortFramerate()) {
        max_fps = port->GetPortFramerate();
      }
      if (batch_size > streaming_active_requests_.size()) {
        streaming_active_requests_.resize(batch_size);
      }
      for (size_t i = 0; i < batch_size; i++) {
        if (std::find(streaming_active_requests_[i].streamIds.begin(),
                      streaming_active_requests_[i].streamIds.end(),
                      cam_stream_id) ==
                      streaming_active_requests_[i].streamIds.end() &&
                      !(std::count(reproc_out_stream_ids_.begin(),
                      reproc_out_stream_ids_.end(),
                      cam_stream_id) > 0)) {
          // Stream ID not found, so add now.
          streaming_active_requests_[i].streamIds.push_back(cam_stream_id);
          QMMF_DEBUG("%s: CameraPort(0x%p):camera_stream_id(%d) is adding to "
              "active stream !", __func__, port.get(), cam_stream_id);
        }
        if ((1 <= i) && (streaming_active_requests_[i].metadata.isEmpty())) {
          assert(!streaming_active_requests_[0].metadata.isEmpty());
          streaming_active_requests_[i].metadata.append(
              streaming_active_requests_[0].metadata);
        }
      }
      stream_ids.emplace(cam_stream_id);

      // Batchiing is use for HFR. HFR and per-stream frame rate control are
      // mutual exclusive.
      if (pcr_frc_enabled_ && batch_size <= 1) {
        SetPerStreamFrameRate();
      }
    } else if (port->getPortState() == PortState::PORT_READYTOSTOP) {

      QMMF_INFO("%s: CameraPort(0x%p):camera_stream_id(%d) is stopped ",
          __func__, port.get(), cam_stream_id);
      // Check if camera stream is already part of request, if yes then remove
      // it from request list. if not then it means stream is created but its
      // corresponding port is not started yet.
      QMMF_INFO("%s: streaming_active_requests_.size(%d)", __func__,
          streaming_active_requests_.size());
      for (size_t j = 0; j < streaming_active_requests_.size(); j++) {
        Camera3Request &req = streaming_active_requests_[j];
        bool match = false;
        size_t idx = -1;
        QMMF_INFO("%s: req.streamIds.size(%d)", __func__,
            req.streamIds.size());
        for (size_t i = 0; i < req.streamIds.size(); i++) {
          if (cam_stream_id == req.streamIds[i]) {
            match = true;
            idx = i;
            break;
          }
        }
        if(match == true) {
          auto itr = req.streamIds.begin();
          req.streamIds.erase(itr+idx);
          QMMF_INFO("%s: cam_stream_id(%d) removed from Request!",
                      __func__, cam_stream_id);
          removed_streams.emplace(cam_stream_id);
          stream_ids.emplace(cam_stream_id);
          QMMF_INFO("%s: removed_streams.size(%d)", __func__,
              removed_streams.size());
        }
      }
    } else if (port->getPortState() == PortState::PORT_STARTED) {
      if (max_fps < port->GetPortFramerate()) {
        max_fps = port->GetPortFramerate();
      }
      stream_ids.emplace(cam_stream_id);
    }
  }

  for (size_t i = 1; i < streaming_active_requests_.size(); ) {
    if (streaming_active_requests_[i].streamIds.empty()) {
        streaming_active_requests_.erase(streaming_active_requests_.begin() + i);
        continue;
    }
    i++;
  }

  size = streaming_active_requests_[0].streamIds.size();

  // when camera mode is fastswitch, multiple video streams should be added or
  // removed simultaneously, which is necessary for HFR case, since preview
  // stream always exists, we need to cache video streams update if active
  // request streams num is not equal to configured stream number.
  if ((camera_parameters_.cam_opmode ==
        CamOperationMode::kCamOperationModeFastSwitch) &&
        (size != 0 && size != 1 && size != active_ports_number)) {
      QMMF_INFO("%s: active_ports_number = %d, size =%d, caching this state",
          __func__, active_ports_number, size);
      return 0;
  }

  //TODO: this logic only works when static stream configurations are applied
  //in dynamic switch case, there will be extra streams created by application
  //so that all ports will not be ready forever
  hfr_wait_ports_ready_ = (active_ports_number == size) ? true : false;

  QMMF_INFO("%s: Number of streams(%d) to start", __func__, size);
  if (size == 0) {
    QMMF_INFO("%s:Cancelling the request, no pending stream!", __func__);
    return CancelRequest();
  } else if ((hfr_detected_ == true) && (hfr_wait_ports_ready_ == false)) {
    QMMF_INFO("%s: hfr enabled, cancelling the request", __func__);
    return CancelRequest();
  }

  {
    std::lock_guard<std::mutex> lock(device_access_lock_);
    if (0 < max_fps) {
      int32_t fpsRange[2];
      fpsRange[0] = ceil(max_fps - 0.5);
      fpsRange[1] = ceil(max_fps - 0.5);

      QMMF_INFO("%s: set frame rate to %d fps", __func__, fpsRange[0]);

      for (size_t i = 0; i < streaming_active_requests_.size(); ++i) {
        streaming_active_requests_[i].metadata.update(
            ANDROID_CONTROL_AE_TARGET_FPS_RANGE, fpsRange, 2);

        // This is required for streaming without AE.
        if (fpsRange[0] == fpsRange[1]) {
          int64_t frameDuration = 1e9 / fpsRange[0];
          streaming_active_requests_[i].metadata.update(
              ANDROID_SENSOR_FRAME_DURATION, &frameDuration, 1);
        }
      }

    }
    std::vector<Camera3Request> request_list;
    for (size_t i = 0; i < streaming_active_requests_.size(); i++) {
      request_list.push_back(streaming_active_requests_[i]);
      assert(!streaming_active_requests_[i].metadata.isEmpty());
    }

    // Configure streams for the first request in cached stream case
    if (streaming_request_id_ < 0 && pending_cached_stream_) {
      assert(0 == camera_device_->EndConfigure(camera_parameters_));
      pending_cached_stream_ = false;
    }

    for (auto const& stream_id : streaming_active_requests_[0].streamIds) {
      std::unique_lock<std::mutex> lk(prepare_lock_);
      prepare_done_.Wait(lk, [this, stream_id] () {
        return stream_prepared_[stream_id];
      });
    }

    int64_t last_frame_number = NO_IN_FLIGHT_REPEATING_FRAMES;
    std::unique_lock<std::mutex> pending_frames_lock(pending_frames_lock_);
    auto req_id = camera_device_->SubmitRequestList(request_list, is_streaming,
                                                    &last_frame_number);

    QMMF_INFO("%s: last_frame_number: current=%lld previous=%lld", __func__,
        last_frame_number, last_frame_number_);

    // SubmitRequestList returns NO_IN_FLIGHT_REPEATING_FRAMES when there is no
    // previous request or when previous request is not submitted to HAL yet.
    // Because of that we have to use previous last_frame_number of any in such
    // case. For example it could happen if SetCameraParam is called soon.
    if (last_frame_number == NO_IN_FLIGHT_REPEATING_FRAMES) {
      // use previous one
      last_frame_number = last_frame_number_;
    } else {
      // store the current one
      last_frame_number_ = last_frame_number;
    }

    assert(req_id >= 0);
    streaming_request_id_ = req_id;

    for (auto const& stream_id : stream_ids) {
      // Update the last submitted frame number for each stream id.
      if (last_frame_number_map_.count(stream_id) != 0 &&
          last_frame_number != NO_IN_FLIGHT_REPEATING_FRAMES) {
        // Request was submitted successfully since previous call, update.
        last_frame_number_map_[stream_id] = last_frame_number;

      } else if (last_frame_number_map_.count(stream_id) == 0) {
        // Newly initiated stream, request hasn't yet been submitted to HAL.
        last_frame_number_map_[stream_id] = NO_IN_FLIGHT_REPEATING_FRAMES;
      }
      QMMF_INFO("%s: last_frame_number_map_[%d]=%lld", __func__,
          stream_id, last_frame_number_map_[stream_id]);

      // Update the removed stream ids that need to wait for frames to return.
      if (removed_streams.count(stream_id) != 0 &&
          last_frame_number_map_[stream_id] != NO_IN_FLIGHT_REPEATING_FRAMES) {
        removed_stream_ids_.emplace(stream_id);
      }
    }

    std::chrono::nanoseconds wait_time(kWaitPendingFramesTimeout);
    while (!removed_stream_ids_.empty()) {
      auto ret = pending_frames_.WaitFor(pending_frames_lock, wait_time);
      if (ret != 0) {
        QMMF_WARN("%s: Waiting for submitted frames to return, timed out!",
            __func__);
        break;
      }
    }
  }
  QMMF_INFO("%s: SubmitRequest for Num streams(%d) is successfull"
      " request_id(%d) batches: %d",  __func__, size, streaming_request_id_,
      streaming_active_requests_.size());

  return 0;
}

int32_t CameraContext::SubmitRequest(Camera3Request request,
                                     bool is_streaming,
                                     int64_t *lastFrameNumber) {
  std::lock_guard<std::mutex> lock(device_access_lock_);

  int32_t ret = 0;
  ret = camera_device_->SubmitRequest(request, is_streaming,
                                      lastFrameNumber);
  assert(ret >= 0);
  return ret;
}

status_t CameraContext::CancelRequest() {

  std::lock_guard<std::mutex> lock(device_access_lock_);
  if (streaming_request_id_ < 0 && capture_request_id_ < 0) {
    QMMF_VERBOSE("%s: No active request\n", __func__);
    return 0;
  }

  int64_t last_frame_mumber;

  QMMF_INFO("%s: Issuing Flush!", __func__);
  auto ret = camera_device_->Flush(&last_frame_mumber);
  assert(ret == 0);
  QMMF_INFO("%s: last_frame_mumber(%lld) after Flush", __func__,
      last_frame_mumber);

  ret = camera_device_->WaitUntilIdle();
  assert(ret == 0);
  {
    std::lock_guard<std::mutex> aec_lock(aec_lock_);
    aec_.Reset();
  }
  streaming_request_id_ = -1;
  capture_request_id_ = -1;
  last_frame_number_ = NO_IN_FLIGHT_REPEATING_FRAMES;
  QMMF_INFO("%s: Request cancelled last frame number: %lld\n",
            __func__, last_frame_mumber);
  return ret;
}

status_t CameraContext::PauseActiveStreams(bool immedialtely) {
  QMMF_VERBOSE("%s Enter ", __func__);

  status_t ret = 0;

  if (streaming_request_id_ < 0 && capture_request_id_ < 0) {
    QMMF_INFO("%s no active streams ", __func__);
    return 0;
  }

  if (port_paused_) {
    QMMF_VERBOSE("%s Already paused ", __func__);
    return 0;
  }

  if (immedialtely) {
    std::lock_guard<std::mutex> lock(device_access_lock_);
    int64_t last_frame_mumber;
    ret = camera_device_->Flush(&last_frame_mumber);
    assert(ret == 0);

    ret = camera_device_->WaitUntilIdle();
    assert(ret == 0);

    last_frame_number_ = NO_IN_FLIGHT_REPEATING_FRAMES;

    // inform all active ports that streaming is interrupted
    for (auto const& it : active_ports_) {
      auto& port = it.second;
      ret = port->Pause();
      assert(ret == 0);
    }
  } else {
    for (auto const& it : active_ports_) {
      auto& port = it.second;
      ret = port->Pause();
      assert(ret == 0);
    }
  }

  assert(stopped_stream_ids_.empty());

  // move all active streams to stopped streams
  if (!streaming_active_requests_.empty()) {
    std::copy(streaming_active_requests_[0].streamIds.begin(),
              streaming_active_requests_[0].streamIds.end(),
              std::inserter(stopped_stream_ids_, stopped_stream_ids_.end()));
    streaming_active_requests_[0].streamIds.clear();
  }

  port_paused_ = true;
  streaming_request_id_ = -1;
  capture_request_id_ = -1;

  QMMF_VERBOSE("%s Exit ", __func__);

  return ret;
}

status_t CameraContext::ResumeActiveStreams(bool state_only) {
  QMMF_VERBOSE("%s Enter ", __func__);
  status_t ret = 0;

  if (stopped_stream_ids_.empty()) {
    QMMF_VERBOSE("%s Nothing to resume", __func__);
    port_paused_ = false;
    return 0;
  }

  QMMF_INFO("%s: Restart Ports! streaming %d", __func__, state_only);
  for (auto const& it : active_ports_) {
    auto& port = it.second;

    if (state_only) {
      // If snapshot capture request is streaming, than we should only
      // resume the port state. Other wise restarting of port will
      // overwrite snapshot capture request.
      ret = port->Resume();
    } else {
      // If snapshot capture request is not streaming, we have to restart port
      // to ensure video steaming after snapshot capture.
      ret = port->Start();
    }
    assert(ret == 0);
  }

  stopped_stream_ids_.clear();
  port_paused_ = false;

  QMMF_VERBOSE("%s Exit ", __func__);

  return ret;
}

status_t CameraContext::ReturnStreamBuffer(StreamBuffer buffer) {
  int32_t ret = 0;

  QMMF_DEBUG("%s: camera_id: %d, stream_id: %d, buffer: %p ts: %lld "
      "frame_number: %d", __func__, buffer.camera_id, buffer.stream_id,
      buffer.handle, buffer.timestamp, buffer.frame_number);

  if(!buffer.in_use_client && !buffer.in_use_camera) {
    ret = camera_device_->ReturnStreamBuffer(buffer);
    assert(ret == 0);

    std::lock_guard<std::mutex> lock(pending_frames_lock_);
    if (removed_stream_ids_.count(buffer.stream_id) != 0) {
      QMMF_DEBUG("%s: removed_stream_ids_.size(%d)", __func__,
          removed_stream_ids_.size());
      QMMF_DEBUG("%s: last_frame_number_map_[%d]=%lld, "
          "buffer.frame_number: %u", __func__,
          buffer.stream_id, last_frame_number_map_[buffer.stream_id],
          buffer.frame_number);
      if (last_frame_number_map_[buffer.stream_id] == buffer.frame_number) {
        removed_stream_ids_.erase(buffer.stream_id);
        last_frame_number_map_.erase(buffer.stream_id);
        pending_frames_.Signal();
      }
    }
  } else {
    QMMF_DEBUG ("%s: buffer.handle %p is in use by client %d and camera %d",
        __func__, buffer.handle, buffer.in_use_client, buffer.in_use_camera);
  }

  return ret;
}

void CameraContext::SnapshotCaptureCallback(StreamBuffer &buffer) {
  uint32_t image_id;

  QMMF_DEBUG("%s Enter ", __func__);

  QMMF_DEBUG("%s %s", __func__, buffer.ToString().c_str());
  QMMF_DEBUG("%s fd(0x%x):size(%d) ", __func__, buffer.fd, buffer.size);

  uint32_t frame_number = 0;
  {
    std::lock_guard<std::mutex> lock(capture_lock_);
    frame_number = capture_cnt_;
    capture_cnt_++;

    // return buffer if cancel capture
    if (cancel_capture_) {
      auto ret = camera_device_->ReturnStreamBuffer(buffer);
      assert(ret == 0);
      return;
    }
  }

  buffer.camera_id = camera_id_;
  snapshot_buffer_list_.insert(std::make_pair(buffer.fd, buffer));
  snapshot_buffer_stream_list_.insert(std::make_pair(buffer.fd, buffer.stream_id));

  assert(client_snapshot_cb_ != nullptr);

  stream_image_lock_.lock();
  if (stream_image_map_[buffer.stream_id] != 0)
      image_id = stream_image_map_[buffer.stream_id];
  else
      QMMF_ERROR("%s stream id(%u) has not image id!", __func__, buffer.stream_id);
  stream_image_lock_.unlock();
  client_snapshot_cb_(image_id, frame_number, buffer);

  QMMF_DEBUG("%s Exit ", __func__);
}

status_t CameraContext::GetSnapshotStreamParams(const SnapshotParam &param,
    CameraStreamParameters &stream_param) {

  QMMF_VERBOSE("%s Enter ", __func__);

  stream_param.format           = Common::FromQmmfToHalFormat(param.format);
  stream_param.width            = param.width;
  stream_param.height           = param.height;
  stream_param.rotation         =
      static_cast<camera3_stream_rotation_t> (param.rotation);
  stream_param.allocFlags.flags = IMemAllocUsage::kSwWriteOften |
                                    IMemAllocUsage::kSwReadOften;
  stream_param.cb               = GetStreamCb(param);

  // Reserve buffers for continuous capture in order to avoid camera and pipe
  // restart if snapshot mode is switched. Buffer are just reserved, not
  // allocated because buffer are allocated on demand in camera adapter.
  stream_param.bufferCount  = MAX_SNAPSHOT_BUFFER_COUNT;

  QMMF_VERBOSE("%s Exit ", __func__);
  return 0;
}

status_t CameraContext::StartZSL(const uint32_t image_id,
                                 const SnapshotParam& param,
                                 const SnapshotZslSetup &zslparam) {

  QMMF_VERBOSE("%s Enter ", __func__);

  if (!IsInputSupported()) {
    QMMF_ERROR("%s: Camera doesn't support input streams!",
               __func__);
    return -EINVAL;
  }

  snapshot_mode_ = param.mode;
  BufferFormat zslfmt = Common::FromImageToQmmfFormat(zslparam.format);

  QMMF_INFO("%s zsl_format %d img_format %d", __func__, zslfmt, param.format);

  CameraStreamParameters stream_param{};
  auto ret = GetSnapshotStreamParams(param, stream_param);
  if (0 != ret) {
    QMMF_ERROR("%s No able to get stream params for ZSL", __func__);
    return ret;
  }

  PauseActiveStreams();

  // The snapshot stream is fixed and matches the ZSL stream
  // size. We cannot re-configure streams dynamically during
  // re-processing as this could have impact on the already
  // cached ZSL buffers and they may fail re-process.
  ret = CreateSnapshotStream(image_id, stream_param, true);
  if (0 != ret) {
    QMMF_ERROR("%s Failed during snapshot stream setup", __func__);
    return ret;
  }

  // At this point of time stream_param contain ZSL staram dimension and format
  QMMF_INFO("%s: ZSL Stream dimension: %dx%d format %x ", __func__,
      stream_param.width, stream_param.height, stream_param.format);

  if (streaming_active_requests_.empty()) {
    streaming_active_requests_.emplace_back();
  }

  ret = CreateCaptureRequest(streaming_active_requests_[0],
                             CAMERA3_TEMPLATE_ZERO_SHUTTER_LAG);
  if (0 != ret) {
    QMMF_ERROR("%s: Capture request for ZSL failed!", __func__);
    return ret;
  }

  int32_t fps_range[2];
  fps_range[0] = camera_parameters_.frame_rate_range[0];
  fps_range[1] = camera_parameters_.frame_rate_range[1];

  streaming_active_requests_[0].metadata.update(
      ANDROID_CONTROL_AE_TARGET_FPS_RANGE, fps_range, 2);

  StreamParam zsl_param = {};
  zsl_param.width          = stream_param.width;
  zsl_param.height         = stream_param.height;
  zsl_param.format         = Common::FromHalToQmmfFormat(stream_param.format);
  zsl_param.framerate      = camera_parameters_.frame_rate_range[1];
  zsl_param.id = zsl_port_id_;

  auto zsl_port = std::make_shared<ZslPort>(zsl_param, camera_parameters_,
                                            CameraPortType::kZSL,
                                            this, zslparam.qdepth);
  assert(zsl_port.get() != nullptr);

  ret = zsl_port->Init();
  if(ret != 0) {
    QMMF_ERROR("%s: CameraPort is not initialized in ZSL mode!",
        __func__);
    zsl_port = nullptr;
    return -EINVAL;
  }

  active_ports_.emplace(zsl_param.id, zsl_port);

  ret = zsl_port->Start();
  if (ret != 0) {
    QMMF_ERROR("%s: zsl port start failed!", __func__);
    return ret;
  }

  ResumeActiveStreams();
  QMMF_INFO("%s: Number of Active ports=%d", __func__, active_ports_.size());

  QMMF_VERBOSE("%s Exit ", __func__);
  return 0;

}

status_t CameraContext::StopZSL(const uint32_t image_id) {
  QMMF_VERBOSE("%s Enter ", __func__);

  // GetPort(zsl_port_id_)
  auto port = std::static_pointer_cast<ZslPort>(GetPort(zsl_port_id_));
  assert(port.get() != nullptr);

  // This will return buffer to camera. Otherwise we cannot reconfigure camera.
  auto ret = port->PauseAndFlushZSLQueue();
  if (ret != 0) {
    QMMF_WARN("%s: ZSL queue is not flashed!", __func__);
    // Even it is not flushed still give a try to Stop it.
  }

  // Stop all on going stream, otherwise we cannot reconfigure camera.
  PauseActiveStreams();

  // Stop ZSL port. This will reconfigure the camera.
  ret = port->Stop();
  if (ret != 0) {
    QMMF_ERROR("%s ZSL port stop failed!", __func__);
    return ret;
  }

  DeleteSnapshotStream(image_id, true);

  ret = port->DeInit();
  if (ret != 0) {
    QMMF_ERROR("%s ZSL port DeInit failed!", __func__);
    return ret;
  }

  // Delete port before resuming streams
  active_ports_.erase(port->GetPortId());

  // Resume remind streams
  ResumeActiveStreams();

  QMMF_VERBOSE("%s Exit ", __func__);

  return ret;
}

status_t CameraContext::CaptureZSLImage(const SnapshotType type) {

  QMMF_INFO("%s: Enter", __func__);
  status_t ret = 0;

  bool regular_snapshot = false;

  auto zsl_port = std::static_pointer_cast<ZslPort>(GetPort(zsl_port_id_));
  assert(zsl_port.get() != nullptr);
  auto stat = zsl_port->PickZSLBuffer();
  if (0 != stat) {
    QMMF_ERROR("%s Failed to find a good ZSL input buffer: %d",
        __func__, stat);
    QMMF_ERROR("%s Switching to regular snapshot!", __func__);
    regular_snapshot = true;
  }

  assert(!snapshot_request_.streamIds.empty());

  std::lock_guard<std::mutex> lock(device_access_lock_);
  uint8_t jpeg_quality = snapshot_quality_;
  int64_t last_frame_mumber;

  if (!regular_snapshot) {
    Camera3Request reprocess_request;
    reprocess_request.streamIds.push_back(zsl_port->GetInputStreamId());
    reprocess_request.streamIds.push_back(snapshot_request_.streamIds[0]);
    reprocess_request.metadata = zsl_port->GetInputBuffer().result;
    reprocess_request.metadata.update(ANDROID_JPEG_QUALITY, &jpeg_quality,
                                      1);
    if (type == SnapshotType::kVideo) {
      if (streaming_active_requests_.size() == 1) {
        auto request = streaming_active_requests_[0];
        for (auto stream_id : request.streamIds) {
          reprocess_request.streamIds.push_back(stream_id);
        }
      } else {
        QMMF_INFO("%s: No other active video streams!", __func__);
      }
    }
    QMMF_INFO("%s: Submit ZSL reprocess request!!", __func__);
    ret = camera_device_->SubmitRequest(reprocess_request, false,
                                            &last_frame_mumber);
    if (ret != 0) {
      QMMF_ERROR("%s Failed to submit ZSL reprocess request: %d",
                 __func__, ret);
    }
  } else {
    QMMF_INFO("%s: Submit Reguar snapshot request!", __func__);

    uint32_t active_streamid_count = 0;
    if (type == SnapshotType::kVideo) {
      if (streaming_active_requests_.size() == 1) {
        auto request = streaming_active_requests_[0];
        for (auto stream_id : request.streamIds) {
          snapshot_request_.streamIds.push_back(stream_id);
          active_streamid_count++;
        }
      } else {
        QMMF_INFO("%s: No other active video streams!", __func__);
      }
    }
    snapshot_request_.metadata.update(ANDROID_JPEG_QUALITY, &jpeg_quality, 1);
    ret = camera_device_->SubmitRequest(snapshot_request_,
                                            false,
                                            &last_frame_mumber);
    if (ret != 0) {
      QMMF_ERROR("%s Failed to submit reguar snapshot request: %d",
                 __func__, ret);
    }

    snapshot_request_.streamIds.
      resize(snapshot_request_.streamIds.size() - active_streamid_count);
  }
  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

#ifndef FLUSH_RESTART_NOTAVAILABLE
status_t CameraContext::DisableFlushRestart(const bool& disable,
                                            CameraMetadata& meta) {

  // Disable restart of the streams on HAL flush in order to save power and
  // optimize the API execution. All streams will be in OFF state after this.
  uint8_t mode = (disable) ? 0 : 1;
  auto ret = meta.update(qcamera::QCAMERA3_HAL_FLUSH_RESTART_MODE, &mode, 1);
  if (ret != 0) {
    QMMF_ERROR("%s: Camera %d: Set flush mode failed!", __func__, camera_id_);
    return -ENOSYS;
  }

  QMMF_INFO("%s: Camera %d: Flush mode is set!", __func__, camera_id_);
  return 0;
}

#endif
//Camera device callbacks
void CameraContext::CameraErrorCb(CameraErrorCode errcode,
                                  const CaptureResultExtras &result) {

  QMMF_WARN("%s: Camera: %d, Error: %d, Request: %d, FrameNumber: %d",
      __func__, camera_id_, errcode, result.requestId, result.frameNumber);

  switch (errcode) {
    case ERROR_CAMERA_DEVICE:
      QMMF_ERROR("%s: Camera device faced an unrecoverable error!", __func__);
      // Clearing active requests to ensure the stop sequence calls goes through
      // without error and all necessary clean up of this and layers above is
      // done when the client calls subsequent APIs (StopSession, DeleteDeviceStream, etc)
      streaming_active_requests_.clear();
      is_camera_dead_ = true;
      break;
    case ERROR_CAMERA_REQUEST:
    case ERROR_CAMERA_BUFFER: {
      std::unique_lock<std::mutex> lock(pending_frames_lock_);
      auto stream_ids = removed_stream_ids_;

      for (auto& stream_id : stream_ids) {
        if (last_frame_number_map_[stream_id] == result.frameNumber) {
          last_frame_number_map_.erase(stream_id);
          removed_stream_ids_.erase(stream_id);

          QMMF_VERBOSE("%s: last_frame_number_map_.size(%d)", __func__,
              last_frame_number_map_.size());
          QMMF_VERBOSE("%s: removed_stream_ids_.size(%d)", __func__,
              removed_stream_ids_.size());
        }
      }
      QMMF_WARN("%s: Frame %u returned with error! Notify all threads waiting"
          " for pending frames!!", __func__, result.frameNumber);
      pending_frames_.Signal();
      break;
    }
    case ERROR_CAMERA_RESULT: {
      QMMF_WARN("%s: An error has occurred in producing an output result"
          " metadata buffer for frame: %u ", __func__, result.frameNumber);
      break;
    }
    default:
      QMMF_WARN("%s: Camera: %d, Error %d won't be handled by CameraContext!",
          __func__, camera_id_, errcode);
      break;
  }

  if (nullptr != error_cb_) {
    error_cb_(camera_id_, errcode);
  }
}

void CameraContext::CameraIdleCb() {
  QMMF_WARN("%s: Camera is in Idle State!!", __func__);
}

void CameraContext::CameraShutterCb(const CaptureResultExtras &result,
                                    int64_t time_stamp) {

}

void CameraContext::CameraPreparedCb(int32_t stream_id) {

  QMMF_INFO("%s: Stream(%d) has been prepared", __func__, stream_id);

  std::lock_guard<std::mutex> lk(prepare_lock_);
  stream_prepared_[stream_id] = true;
  prepare_done_.Signal();
}

template <typename T>
bool CameraContext::UpdatePartialTag(CameraMetadata &result, int32_t tag,
                                     const T *value,
                                     uint32_t frame_number) {
  if (0 != result.update(tag, value, 1)) {
    return false;
  }
  return true;
}

template <typename T>
bool CameraContext::QueryPartialTag(const CameraMetadata &result,
                                    int32_t tag, T *value,
                                    uint32_t frame_number) {
  (void)frame_number;

  camera_metadata_ro_entry_t entry;

  entry = result.find(tag);
  if (entry.count == 0) {
    return false;
  }

  if (sizeof(T) == sizeof(uint8_t)) {
    *value = entry.data.u8[0];
  } else if (sizeof(T) == sizeof(int32_t)) {
    *value = entry.data.i32[0];
  } else {
    return false;
  }
  return true;
}

void CameraContext::HandleFinalResult(const CaptureResult &result) {

  if (result.metadata.exists(ANDROID_CONTROL_AE_STATE) &&
      result.metadata.exists(ANDROID_SENSOR_TIMESTAMP)) {
    auto new_state = result.metadata.find(ANDROID_CONTROL_AE_STATE).data.u8[0];
    auto timestamp = result.metadata.find(ANDROID_SENSOR_TIMESTAMP).data.i64[0];

    std::lock_guard<std::mutex> lock(aec_lock_);
    if (aec_.state != new_state) {
      QMMF_VERBOSE("%s: Camera: %d, Frame: %d, Ts: %lld AE state changed:"
          " %d --> %d", __func__, camera_id_,
          result.resultExtras.frameNumber, timestamp, aec_.state, new_state);

      aec_.state     = new_state;
      aec_.timestamp = timestamp;
      aec_state_updated_.Signal();
    }
    if (enable_reproc_) {
      auto reproc_port = std::static_pointer_cast<CameraPort>(GetPort(reproc_port_id_));
      assert(reproc_port.get() != nullptr);
      reproc_port->HandleReprocCaptureResult(&result, nullptr);
    }
  }

  if (snapshot_mode_ == ImageMode::kZsl) {
    auto zsl_port = std::static_pointer_cast<ZslPort>(GetPort(zsl_port_id_));
    assert(zsl_port.get() != nullptr);
    zsl_port->HandleZSLCaptureResult(result);
  }

  if (nullptr != result_cb_) {
    result_cb_(camera_id_, result.metadata);
  }
  return;
}

void CameraContext::CameraResultCb(const CaptureResult &result) {

  std::lock_guard<std::mutex> lock(partial_result_lock_);
  if (result.resultExtras.partialResultCount < partial_result_count_) {

    bool complete_result = true;

    uint8_t afMode, afState, aeState, awbState, awbMode;
    auto frame_number  = result.resultExtras.frameNumber;

    complete_result &=
        QueryPartialTag(result.metadata, ANDROID_CONTROL_AWB_MODE,
                        &awbMode, frame_number);
    complete_result &=
        QueryPartialTag(result.metadata, ANDROID_CONTROL_AF_MODE,
                        &afMode, frame_number);
    complete_result &=
        QueryPartialTag(result.metadata, ANDROID_CONTROL_AE_STATE,
                        &aeState, frame_number);
    complete_result &= QueryPartialTag(result.metadata,
                                       ANDROID_CONTROL_AWB_STATE,
                                       &awbState, frame_number);
    complete_result &=
        QueryPartialTag(result.metadata, ANDROID_CONTROL_AF_STATE, &afState,
                        frame_number);

    if (!complete_result && is_partial_metadata_enabled_) {
      if (nullptr != result_cb_) {
        result_cb_(camera_id_, result.metadata);
      }
      return;
    }

    if (complete_result) {
      CaptureResult captureResult;
      captureResult.resultExtras = result.resultExtras;
      captureResult.metadata = CameraMetadata(10, 0);

      if (!UpdatePartialTag(captureResult.metadata, ANDROID_REQUEST_FRAME_COUNT,
                            reinterpret_cast<int32_t *>(&frame_number),
                            frame_number)) {
        return;
      }

      int32_t requestId = result.resultExtras.requestId;
      if (!UpdatePartialTag(captureResult.metadata, ANDROID_REQUEST_ID,
                            &requestId, frame_number)) {
        return;
      }

      if (!UpdatePartialTag(captureResult.metadata, ANDROID_CONTROL_AWB_STATE,
                            &awbState, frame_number)) {
        return;
      }
      if (!UpdatePartialTag(captureResult.metadata, ANDROID_CONTROL_AF_MODE,
                            &afMode, frame_number)) {
        return;
      }
      if (!UpdatePartialTag(captureResult.metadata, ANDROID_CONTROL_AWB_MODE,
                            &awbMode, frame_number)) {
        return;
      }
      if (!UpdatePartialTag(captureResult.metadata, ANDROID_CONTROL_AF_STATE,
                            &afState, frame_number)) {
        return;
      }
      if (!UpdatePartialTag(captureResult.metadata, ANDROID_CONTROL_AE_STATE,
                            &aeState, frame_number)) {
        return;
      }

      HandleFinalResult(captureResult);
    }
  } else {
    HandleFinalResult(result);
  }
}

std::shared_ptr<CameraPort> CameraContext::GetPort(const uint32_t& track_id) {

  if (active_ports_.count(track_id) == 0) {
    QMMF_ERROR("%s: No port is associated with id(%x)!", __func__, track_id);
    return nullptr;
  }
  auto port = active_ports_[track_id];

  QMMF_INFO("%s: Found port for track_id(%x)", __func__, track_id);
  return port;
}

void CameraContext::OnFrameAvailable(StreamBuffer& buffer) {

  QMMF_DEBUG("%s: StreamBuffer(0x%p) fd: %d stream_id: %d ts: %lld",
      __func__, buffer.handle, buffer.fd, buffer.stream_id,
      buffer.timestamp);
  SnapshotCaptureCallback(buffer);
}

void CameraContext::NotifyBufferReturned(StreamBuffer& buffer) {

  QMMF_DEBUG("%s: StreamBuffer(0x%p) fd: %d stream_id: %d ts: %lld",
      __func__, buffer.handle, buffer.fd, buffer.stream_id,
      buffer.timestamp);

  if (snapshot_mode_ == ImageMode::kZsl) {
    auto zsl_port = std::static_pointer_cast<ZslPort>(GetPort(zsl_port_id_));
    assert(zsl_port.get() != nullptr);
    zsl_port->ReturnZSLInputBuffer(buffer);
  } else {
    ReturnStreamBuffer(buffer);
  }

}

AECData CameraContext::GetAECData() {

  std::lock_guard<std::mutex> lock(aec_lock_);
  return aec_;
}

CameraPort::CameraPort(const StreamParam& param,
                       const CameraParameters camera_parameters,
                       CameraPortType port_type, CameraContext* context)
    : port_type_(port_type),
      context_(context),
      camera_stream_id_(-1),
      params_(param),
      ready_to_start_(false),
      port_id_(param.id),
      reproc_input_stream_id_(-1),
      reproc_queue_{},
      reproc_input_buffer_{},
      cameraport_enable_reproc_(false),
      camera_parameters_(camera_parameters) {

  QMMF_INFO("%s: Enter", __func__);

  buffer_producer_impl_ = std::make_shared<BufferProducerImpl<CameraPort>>(this);

  QMMF_INFO("%s: Exit (0x%p)", __func__, this);
}

CameraPort::~CameraPort() {

  QMMF_INFO("%s: Enter ", __func__);
  buffer_producer_impl_.reset();
  buffer_producer_impl_ = nullptr;
  QMMF_INFO("%s: Exit (0x%p)", __func__, this);
}

status_t CameraPort::Init() {

  QMMF_VERBOSE("%s port type %d id %d state %d ", __func__,
    GetPortType(), GetPortId(), port_state_);

  cam_stream_params_ = {};
  cam_stream_params_.width  = params_.width;
  cam_stream_params_.height = params_.height;
  if (context_->IsReproc() ||
      static_cast<bool>(params_.flags & VideoFlags::kReproc))
    cam_stream_params_.format = HAL_PIXEL_FORMAT_YCbCr_420_888;
  else
    cam_stream_params_.format = Common::FromQmmfToHalFormat(params_.format);
  cam_stream_params_.rotation =
      static_cast<camera3_stream_rotation_t> (params_.rotation);

  if (params_.format == BufferFormat::kBLOB) {
    cam_stream_params_.allocFlags.flags = IMemAllocUsage::kSwWriteOften |
                                    IMemAllocUsage::kSwReadOften;
    cam_stream_params_.bufferCount  = MAX_SNAPSHOT_BUFFER_COUNT;
  } else {
    // VideoFlags::kPreview is added, stream with this flag
    // will be treated as preview stream
    if (static_cast<bool>(params_.flags & VideoFlags::kPreview)) {

      QMMF_INFO("%s: port %d with preview flag",__func__, GetPortId());

      cam_stream_params_.allocFlags.flags = IMemAllocUsage::kHwComposer;
    } else {
      // This flag should be mandatory if preview flag is not set.
      // Stream is considered as preview stream without it.
      // Different tuning, setings and sensor mode is applied for preview and
      // video streams. This is why this flag is needed.
      cam_stream_params_.allocFlags.flags = IMemAllocUsage::kVideoEncoder;
#ifndef HAVE_BINDER
      cam_stream_params_.allocFlags.flags |= IMemAllocUsage::kHwRender;
#endif // !HAVE_BINDER
    }

    switch (params_.format) {
      case BufferFormat::kNV12UBWC:
        cam_stream_params_.allocFlags.flags |=
            IMemAllocUsage::kPrivateAllocUbwc;
        break;
      case BufferFormat::kP010:
        cam_stream_params_.data_space = HAL_DATASPACE_TRANSFER_GAMMA2_8;
        cam_stream_params_.allocFlags.flags |=
            IMemAllocUsage::kPrivateAllocP010;
        break;
      case BufferFormat::kTP10UBWC:
        cam_stream_params_.data_space = HAL_DATASPACE_TRANSFER_GAMMA2_8;
        cam_stream_params_.allocFlags.flags |=
            IMemAllocUsage::kPrivateAllocTP10 |
              IMemAllocUsage::kPrivateAllocUbwc;
        break;
      default:
#ifdef HAVE_BINDER
        cam_stream_params_.allocFlags.flags |=
            IMemAllocUsage::kSwReadOften | IMemAllocUsage::kSwWriteOften;
#else
        cam_stream_params_.allocFlags.flags |= IMemAllocUsage::kHwTexture;
#endif // HAVE_BINDER
        break;
    }

    //TODO: This needs to be rework and provide proper solution to
    //      set UBWC per stream basis.
    if ((camera_parameters_.cam_feature_flags &
         static_cast<uint32_t>(CamFeatureFlag::kLDC)) ||
        (camera_parameters_.cam_feature_flags &
         static_cast<uint32_t>(CamFeatureFlag::kEIS))) {
      cam_stream_params_.allocFlags.flags &=
          ~(IMemAllocUsage::kPrivateAllocUbwc);
    }

    cam_stream_params_.allocFlags.flags |=
        static_cast<bool>(params_.flags & VideoFlags::kUncashed) ?
            IMemAllocUsage::kPrivateUncached : 0;

    // round extra buffer count to batch size
    params_.xtrabufs =
        ((params_.xtrabufs + camera_parameters_.batch_size - 1) /
          camera_parameters_.batch_size) * camera_parameters_.batch_size;

    cam_stream_params_.bufferCount = STREAM_BUFFER_COUNT +
        GetExtraBufferCount() + params_.xtrabufs;

    QMMF_INFO ("%s: track_id(0%x) total buffer count(%d)", __func__,
        params_.id, cam_stream_params_.bufferCount);
  }

#if defined(CAMX_ANDROID_API) && (CAMX_ANDROID_API >= 31)
  if (params_.colorimetry == VideoColorimetry::kBT2100HLG) {
    cam_stream_params_.hdrmode = ANDROID_REQUEST_AVAILABLE_DYNAMIC_RANGE_PROFILES_MAP_HLG10;
  }
#endif

  cam_stream_params_.cb = [&] (StreamBuffer buffer) { StreamCallback(buffer); };

  assert(context_ != nullptr);

  int32_t stream_id;

  if (static_cast<bool>(params_.flags & VideoFlags::kReproc)) {
    // Create Input stream for reprocess.
    cameraport_enable_reproc_ = true;
    context_->SetReprocPortId(GetPortId());
    input_stream_params_ = {};
    input_stream_params_.format = HAL_PIXEL_FORMAT_YCbCr_420_888;
    input_stream_params_.width  = params_.width;
    input_stream_params_.height = params_.height;
    input_stream_params_.get_input_buffer = [&] (StreamBuffer& buffer)
        { GetReprocInputBuffer(buffer); };
    input_stream_params_.return_input_buffer  = [&] (StreamBuffer& buffer)
        { ReturnReprocInputBuffer(buffer); };

    auto ret = context_->CreateDeviceInputStream(input_stream_params_, &stream_id, true);
    if (0 != ret) {
      QMMF_ERROR("%s Failed to create input reprocess stream: %d",
                 __func__, ret);
      return ret;
    }
    assert(stream_id >= 0);
    reproc_input_stream_id_ = stream_id;
    QMMF_INFO("%s: reproc input_stream_id_(%d)", __func__, reproc_input_stream_id_);
  }

  auto ret = context_->CreateDeviceStream(cam_stream_params_,
                                          params_.framerate, &stream_id, true);
  if (ret != 0 || stream_id < 0) {
    QMMF_ERROR("%s: CreateDeviceStream failed!!", __func__);
    return -EINVAL;
  }
  camera_stream_id_ = stream_id;
  port_state_ = PortState::PORT_CREATED;

  QMMF_INFO("%s: Camera Device Stream(%d) is created Succussfully with"
            "flag(0x%x) and format(0x%x)!", __func__, camera_stream_id_,
            cam_stream_params_.allocFlags.flags, cam_stream_params_.format);
  QMMF_INFO("%s: track_id(0%x) is mapped to camera stream_id(%d)",
      __func__, params_.id, camera_stream_id_);
  return 0;
}

status_t CameraPort::DeInit() {

  QMMF_VERBOSE("%s port type %d id %d state %d ", __func__,
    GetPortType(), GetPortId(), port_state_);

  assert(ready_to_start_ == false);
  assert(context_ != nullptr);

  auto ret = context_->DeleteDeviceStream(camera_stream_id_, true);
  if(ret != 0) {
    QMMF_ERROR("%s: DeleteDeviceStream failed!!", __func__);
    return -EINVAL;
  }
  consumers_.clear();
  QMMF_DEBUG("%s: CameraPort(0x%p) deinitialized successfully! ",
      __func__, this);
  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

status_t CameraPort::Start() {

  QMMF_VERBOSE("%s port type %d id %d state %d ", __func__,
    GetPortType(), GetPortId(), port_state_);

  if (port_state_ == PortState::PORT_STARTED){
    // Port is already in started state.
    return 0;
  }

  //TODO: protect it with lock.
  ready_to_start_ = true;
  aec_converged_  = false;
  port_state_ = PortState::PORT_READYTOSTART;

  QMMF_INFO("%s: track_id(%x):camera stream(%d) to start!", __func__,
      port_id_, camera_stream_id_);

  auto ret = context_->UpdateRequest(true);
  if (ret != 0) {
    QMMF_ERROR("%s: UpdateRequest failed! for track_id = %d",
        __func__, port_id_);
    return ret;
  }
  QMMF_INFO("%s: track_id(%x):Port(%p) Started Succussfully!",
      __func__, port_id_, this);

  port_state_ = PortState::PORT_STARTED;
  return 0;
}

status_t CameraPort::Stop() {

  QMMF_VERBOSE("%s port type %d id %d state %d ", __func__,
    GetPortType(), GetPortId(), port_state_);

    if (port_state_ == PortState::PORT_CREATED /*||
      port_state_ == PortState::PORT_STOPPED*/){
    //TODO: this is hack at this point, we need to find proper solution when
    // port is started outside the updateRequest.
    // Port is already in stopped state.
    return 0;
  }

  //TODO: protect it with lock.
  ready_to_start_ = false;
  aec_converged_  = false;
  port_state_ = PortState::PORT_READYTOSTOP;

  if (cameraport_enable_reproc_) {
    QMMF_VERBOSE ("%s: Freeing the Reproc Requests not submitted before stopping %d",
      __func__, reproc_queue_.size());
    for (ReprocEntry entry : reproc_queue_) {
      if(entry.buffer.handle != NULL){
        entry.buffer.in_use_camera = false;
        entry.buffer.in_use_client = false;
        context_->ReturnStreamBuffer(entry.buffer);
      }
    }
  }
  // Stop basically removes the stream from current running capture request,
  // it doen't delete the stream.
  auto ret = context_->UpdateRequest(true);
  if (ret != 0) {
    QMMF_ERROR("%s: CameraPort:Start:UpdateRequest failed! for track_id = %d"
        ,  __func__, port_id_);
    return ret;
  }

  QMMF_INFO("%s: track_id(%x):Port(%p) Stopped Succussfully!",
      __func__, port_id_, this);

  port_state_ = PortState::PORT_STOPPED;
  return ret;
}

status_t CameraPort::Pause() {

  QMMF_VERBOSE("%s port type %d id %d state %d ", __func__,
    GetPortType(), GetPortId(), port_state_);
  assert(port_state_ == PortState::PORT_STARTED);
  port_state_ = PortState::PORT_PAUSED;
  return 0;
}

status_t CameraPort::Resume() {

  QMMF_VERBOSE("%s port type %d id %d state %d ", __func__,
    GetPortType(), GetPortId(), port_state_);
  assert(port_state_ == PortState::PORT_PAUSED);
  port_state_ = PortState::PORT_STARTED;
  return 0;
}

status_t CameraPort::AddConsumer(std::shared_ptr<IBufferConsumer>& consumer) {

  std::lock_guard<std::mutex> lock(consumer_lock_);
  assert(consumer.get() != nullptr);

  if (IsConsumerConnected(consumer)) {
    QMMF_ERROR("%s: consumer(%p) already added to the producer!",
        __func__, consumer.get());
    return -EEXIST;
  }

  // Add consumer to port's producer interface.
  assert(buffer_producer_impl_.get() != nullptr);
  buffer_producer_impl_->AddConsumer(consumer);
  consumer->SetProducerHandle(buffer_producer_impl_);
  QMMF_DEBUG("%s: Consumer(%p) has been added to CameraPort(%p)."
      "Total number of consumer = %d",  __func__, consumer.get()
      , this, buffer_producer_impl_->GetNumConsumer());
  consumers_.emplace(reinterpret_cast<uintptr_t>(consumer.get()), consumer);
  return 0;
}

status_t CameraPort::RemoveConsumer(std::shared_ptr<IBufferConsumer>& consumer) {
  std::lock_guard<std::mutex> lock(consumer_lock_);
  assert(consumer.get() != nullptr);
  if (!IsConsumerConnected(consumer)) {
    QMMF_ERROR("%s: consumer(%p) is not connected to this port(%p)!",
        __func__, consumer.get(), this);
    return -EINVAL;
  }

  // Remove consumer from port's producer interface.
  assert(buffer_producer_impl_.get() != nullptr);
  buffer_producer_impl_->RemoveConsumer(consumer);
  QMMF_DEBUG("%s: Consumer(%p) has been removed from CameraPort(%p)."
      "Total number of consumer = %d",  __func__, consumer.get()
      , this, buffer_producer_impl_->GetNumConsumer());
  consumers_.erase(reinterpret_cast<uintptr_t>(consumer.get()));
  return 0;
}

void CameraPort::NotifyBufferReturned(const StreamBuffer& buffer) {

  QMMF_VERBOSE("%s: StreamBuffer(0x%p) Cameback to CameraPort",
       __func__, buffer.handle);
  //TODO: protect this with lock, would be required once multiple camera ports
  // are enabled.
  context_->ReturnStreamBuffer(buffer);
}

int32_t CameraPort::GetNumConsumers() {

  std::lock_guard<std::mutex> lock(consumer_lock_);
  return consumers_.size();
}

bool CameraPort::IsReadyToStart() {
  //TODO: protect it with lock.
  return ready_to_start_;
}

PortState& CameraPort::getPortState() {
  return port_state_;
}

bool CameraPort::IsConsumerConnected(std::shared_ptr<IBufferConsumer>& consumer) {

  uintptr_t key = reinterpret_cast<uintptr_t>(consumer.get());
  return (consumers_.count(key) != 0) ? true : false;
}

void CameraPort::HandleReprocCaptureResult(const CaptureResult *result, StreamBuffer *buffer) {

  QMMF_VERBOSE("%s Enter ", __func__);

  if (0 <= camera_stream_id_) {
    int64_t timestamp;
    if (result != nullptr && result->metadata.exists(ANDROID_SENSOR_TIMESTAMP)) {
      timestamp = result->metadata.find(ANDROID_SENSOR_TIMESTAMP).data.i64[0];
    } else if (buffer == nullptr) {
      QMMF_ERROR("%s Sensor timestamp tag missing in result!\n",
        __func__);
      return;
    }

    {
      std::lock_guard<std::mutex> l(reproc_queue_lock_);
      bool append = true;
      if (!reproc_queue_.empty()) {
        std::list<ReprocEntry>::iterator it = reproc_queue_.begin();
        std::list<ReprocEntry>::iterator end = reproc_queue_.end();
        while (it != end) {
          if ( result != nullptr && it->timestamp == timestamp) {
            it->result.append(result->metadata);
            append = false;
            break;
          } else if (buffer != nullptr && it->timestamp == buffer->timestamp) {
              it->buffer = *buffer;
              append = false;
              break;
            }
          it++;
        }
      }

      if (append) {
        //Buffer is missing append to queue directly
        ReprocEntry new_entry{};
        if (result != nullptr) {
          new_entry.result.append(result->metadata);
          new_entry.timestamp = timestamp;
          reproc_queue_.push_back(new_entry);
        } else if (buffer != nullptr) {
          new_entry.buffer = *buffer;
          new_entry.timestamp = buffer->timestamp;
          new_entry.result.clear();
          reproc_queue_.push_back(new_entry);
        }
      } else {
        Camera3Request reprocess_request;
        int64_t last_frame_number;
        ReprocEntry input_entry{};
        input_entry.timestamp = -1;
        if (reproc_queue_.size()) {
          reproc_queue_.remove_if([](ReprocEntry entry) {
            return entry.buffer.handle == NULL;
          });

          input_entry = reproc_queue_.back();
        }
        if (input_entry.buffer.handle != NULL && !input_entry.result.isEmpty()) {
          int32_t roi_count = context_->GetROICount();
          std::vector<int32_t> roi_info = context_->GetROIInfo();
          int32_t roi_info_arr[roi_count * 4];
          uint32_t roi_count_tag = context_->GetROICountTag();
          uint32_t roi_info_tag = context_->GetROIInfoTag();
          for (int i = 0; i < roi_info.size(); i++)
            roi_info_arr[i] = roi_info[i];

          reprocess_request.streamIds.push_back(reproc_input_stream_id_);
          std::vector<int32_t> reproc_stream_ids = context_->GetReprocOutputStreamIds();

          for (auto id: reproc_stream_ids)
            reprocess_request.streamIds.push_back(id);

          reprocess_request.metadata = input_entry.result;
          reprocess_request.metadata.update(roi_count_tag, &roi_count, 1);
          reprocess_request.metadata.update(roi_info_tag, roi_info_arr, roi_count*4);

          // Submit one request containing single shot meta entries.
          auto ret = context_->SubmitRequest(reprocess_request, false,
                                                   &last_frame_number);
          assert(ret >= 0);
        } else {
          QMMF_ERROR ("Reproc buffer handle found to be NULL skipping");
        }
      }
    }
  }
  QMMF_VERBOSE("%s Exit ", __func__);
}

void CameraPort::StreamCallback(StreamBuffer buffer) {

  QMMF_VERBOSE("%s: Enter stream_id(%d)", __func__, buffer.stream_id);
  assert(buffer.stream_id == camera_stream_id_);
  assert(buffer_producer_impl_.get() != nullptr);

  // Assign camera id to the stream buffer.
  buffer.camera_id = context_->camera_id_;

  QMMF_DEBUG("%s: camera_id: %d, stream_id: %d, buffer: %p ts: %lld "
      "frame_number: %d", __func__, buffer.camera_id, buffer.stream_id,
      buffer.handle, buffer.timestamp, buffer.frame_number);

  bool skip_frame = false;

  if (static_cast<bool>(params_.flags & VideoFlags::kIAEC)) {
    // Get auto exposure data and check if initial AE has converged.
    std::lock_guard<std::mutex> lock(aec_lock_);
    if (!aec_converged_) {
      auto aec = context_->GetAECData();
      aec_converged_ = (aec.state == ANDROID_CONTROL_AE_STATE_LOCKED) ||
                       (aec.state == ANDROID_CONTROL_AE_STATE_CONVERGED);
      aec_timestamp_ = aec.timestamp;
    }
    // Raise the skip flag if AE did not converged for this buffer.
    skip_frame = !(aec_converged_ && (buffer.timestamp >= aec_timestamp_));
  }

  if (cameraport_enable_reproc_ && !(getPortState() == PortState::PORT_READYTOSTOP)) {
    buffer.in_use_camera = true;
    HandleReprocCaptureResult(nullptr, &buffer);
  }
  buffer.in_use_client = true;
  std::lock_guard<std::mutex> lock(consumer_lock_);
  if (buffer_producer_impl_->GetNumConsumer() > 0 && !skip_frame) {
    buffer_producer_impl_->NotifyBuffer(buffer);
  } else {
    buffer.in_use_client = false;
    buffer.in_use_camera = false;
    QMMF_VERBOSE("%s: Return buffer back to camera!", __func__);
    context_->ReturnStreamBuffer(buffer);
  }

  QMMF_VERBOSE("%s: Exit ", __func__);
}

uint32_t CameraPort::GetExtraBufferCount() {
  uint32_t extra_buffer_count = 0;

  if (params_.framerate < 24.0) {
    extra_buffer_count = 0;
  } else if (params_.framerate < 60.0) {
    extra_buffer_count = EXTRA_DCVS_BUFFERS;
  } else if (params_.framerate < 120.0) {
    extra_buffer_count = EXTRA_HFR_BUFFERS;
  } else if (params_.framerate < 240.0) {
    extra_buffer_count = 2 * EXTRA_HFR_BUFFERS;
  } else {
    extra_buffer_count = 3 * EXTRA_HFR_BUFFERS;
  }

  QMMF_DEBUG("%s: Number of extra buffers added: %u", __func__,
             extra_buffer_count);
  return extra_buffer_count;
}

void CameraPort::GetReprocInputBuffer(StreamBuffer& buffer) {

  std::lock_guard<std::mutex> l(reproc_queue_lock_);
  if(reproc_queue_.size())
    reproc_input_buffer_ = *reproc_queue_.begin();
  buffer = reproc_input_buffer_.buffer;
  QMMF_INFO("%s buffer(%d) submitted for reprocess with handle %p!", __func__,
    buffer.fd, buffer.handle);
  reproc_queue_.erase(reproc_queue_.begin());
}

void CameraPort::ReturnReprocInputBuffer(StreamBuffer& buffer) {

  QMMF_DEBUG("%s: Enter ", __func__);
  if (buffer.handle != NULL) {
    buffer.in_use_camera = false;
    buffer.stream_id = 1;
    assert (context_ != nullptr);
    auto ret = context_->ReturnStreamBuffer(buffer);
    if (0 != ret) {
      QMMF_ERROR("%s Failed to return input buffer: %d\n", __func__,
          ret);
    }
  } else {
    QMMF_ERROR ("%s: Returned Buffer handle is NULL", __func__);
  }

  QMMF_DEBUG("%s: Exit ", __func__);
}

ZslPort::ZslPort(const StreamParam& param,
                 const CameraParameters camera_port_parameters,
                 CameraPortType port_type, CameraContext *context,
                 uint32_t zsl_queue_depth)
    : CameraPort(param, camera_port_parameters, port_type, context),
      zsl_queue_depth_(zsl_queue_depth) {
  QMMF_INFO("%s: Enter", __func__);
  zsl_input_buffer_.timestamp = -1;
  QMMF_INFO("%s: Exit (0x%p)", __func__, this);
}

ZslPort::~ZslPort() {

  QMMF_INFO("%s: Enter ", __func__);
  if (!zsl_queue_.empty()) {
    PauseAndFlushZSLQueue();
  }
  QMMF_INFO("%s: Exit (0x%p)", __func__, this);
}

status_t ZslPort::Init() {

  assert(port_type_ == CameraPortType::kZSL);
  auto ret = SetUpZSL();
  if (ret != 0) {
    QMMF_INFO("%s SetUpZSL failed!", __func__);
    return ret;
  }

  port_state_ = PortState::PORT_CREATED;
  QMMF_INFO("%s: ZslPort Initialized!", __func__);
  return ret;
}

status_t ZslPort::DeInit() {

  QMMF_VERBOSE("%s port type %d id %d state %d ", __func__,
    GetPortType(), GetPortId(), port_state_);

  status_t ret = 0;

  assert(context_ != nullptr);

  if (GetInputStreamId() != -1) {
    ret = context_->DeleteDeviceStream(GetInputStreamId(), true);
    if (ret != 0) {
      QMMF_ERROR("%s: DeleteDeviceStream failed!!", __func__);
      return -EINVAL;
    }
  }

  ret = CameraPort::DeInit();

  QMMF_INFO("%s: Exit", __func__);
  return ret;
}

status_t ZslPort::PauseAndFlushZSLQueue() {

  QMMF_DEBUG("%s: Enter", __func__);
  int32_t ret = 0;
  std::lock_guard<std::mutex> l(zsl_queue_lock_);
  zsl_running_ = false;

  if (!zsl_queue_.empty()) {
    std::vector<ZSLEntry>::iterator it = zsl_queue_.begin();
    std::vector<ZSLEntry>::iterator end = zsl_queue_.end();
    while (it != end) {
      if (it->timestamp == it->buffer.timestamp) {
        assert(context_ != nullptr);
        auto stat = context_->ReturnStreamBuffer(it->buffer);
        if (0 != stat) {
          QMMF_ERROR("%s Failed to flush ZSL buffer: %d",
                     __func__, stat);
          ret = stat;
        }
      }
      it++;
    }
    zsl_queue_.clear();
  }
  QMMF_INFO("%s: Zsl queue flush is: %s", __func__,
      ret == 0 ? "Successful!" : "Failed!");

  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}

void ZslPort::ResumeZSL() {
  std::lock_guard<std::mutex> l(zsl_queue_lock_);
  zsl_running_ = true;
}

bool ZslPort::IsRunning() {
  std::lock_guard<std::mutex> l(zsl_queue_lock_);
  return zsl_running_ && (camera_stream_id_ >= 0);
}

status_t ZslPort::PickZSLBuffer() {

  QMMF_DEBUG("%s Enter ", __func__);
  std::lock_guard<std::mutex> l(zsl_queue_lock_);
  auto ret = 0;

  if (zsl_queue_.empty()) {
    QMMF_ERROR("%s ZSL queue is empty!\n", __func__);
    return -ENODEV;
  }

  if (0 <= zsl_input_buffer_.timestamp) {
    QMMF_ERROR("%s Previous ZSL input still processing!", __func__);
    return -EBUSY;
  }

  // search for frame with good exposure
  bool found = false;
  std::vector<ZSLEntry>::reverse_iterator it = zsl_queue_.rbegin();
  for (; it != zsl_queue_.rend(); it++) {
    if ((it->timestamp == it->buffer.timestamp) && (!it->result.isEmpty())) {
      camera_metadata_entry_t entry;
      entry = it->result.find(ANDROID_CONTROL_AE_STATE);
      if (0 < entry.count) {
        if ((entry.data.u8[0] == ANDROID_CONTROL_AE_STATE_CONVERGED) ||
            (entry.data.u8[0] == ANDROID_CONTROL_AE_STATE_LOCKED)) {
          found = true;
          break;
        }
      }
    }
  }

  if (found) {
    zsl_input_buffer_ = *it;
    zsl_queue_.erase(--(it.base())); // convert from riter to iter
    QMMF_INFO("%s: Found Good ZSL buffer!!", __func__);
  } else {
    QMMF_ERROR("%s: No appropriate ZSL buffer found!", __func__);
    ret = -ENOENT;
  }
  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t ZslPort::ValidateCaptureParams(uint32_t width, uint32_t height,
    BufferFormat format) {

  if (BufferFormat::kBLOB != format) {
    QMMF_ERROR("%s ZSL capture supports only Jpeg as output!", __func__);
    return -EINVAL;
  }

  // if post process isn't enabled image size should match ZSL size
  if ((width != params_.width || height != params_.height)) {
    QMMF_ERROR("%s ZSL stream size %dx%d doesn't match image size %dx%d!",
        __func__, params_.width, params_.height, width, height);
    return -EINVAL;
  }

  // if post process is enabled image size should be less or equeal to ZSL size
  if ((width > params_.width || height > params_.height)) {
    QMMF_ERROR("%s ZSL stream size %dx%d smaller than image size %dx%d!",
        __func__, params_.width, params_.height, width, height);
    return -EINVAL;
  }

  return 0;
}

void ZslPort::HandleZSLCaptureResult(const CaptureResult &result) {

  QMMF_VERBOSE("%s Enter ", __func__);

  if (0 <= camera_stream_id_) {
    int64_t timestamp;
    if (result.metadata.exists(ANDROID_SENSOR_TIMESTAMP)) {
      timestamp = result.metadata.find(ANDROID_SENSOR_TIMESTAMP).data.i64[0];
    } else {
      QMMF_ERROR("%s Sensor timestamp tag missing in result!\n",
        __func__);
      return;
    }

    {
      ZSLEntry entry{};
      entry.timestamp = -1;

      std::lock_guard<std::mutex> l(zsl_queue_lock_);
      if (zsl_running_) {
        bool append = true;
        if (!zsl_queue_.empty()) {
          std::vector<ZSLEntry>::iterator it = zsl_queue_.begin();
          std::vector<ZSLEntry>::iterator end = zsl_queue_.end();
          while (it != end) {
            if (it->timestamp == timestamp) {
              it->result.append(result.metadata);
              append = false;
              break;
            }
            it++;
          }
        }

        if (append) {
          //Buffer is missing append to queue directly
          ZSLEntry new_entry{};
          new_entry.result.append(result.metadata);
          new_entry.timestamp = timestamp;
          zsl_queue_.push_back(new_entry);
        }

        if (zsl_queue_.size() >= (zsl_queue_depth_ + 1)) {
          entry = *zsl_queue_.begin(); //return oldest buffer
          zsl_queue_.erase(zsl_queue_.begin());
        }
      }

      if (entry.timestamp == entry.buffer.timestamp) {
        QMMF_DEBUG("%s Return Buffer from ZSl Queue back to camera!",
            __func__);
        assert (context_ != nullptr);
        auto ret = context_->ReturnStreamBuffer(entry.buffer);
        if (0 != ret) {
          QMMF_ERROR("%s Failed to return ZSL buffer to camera: %d",
                     __func__, ret);
        }
      }
    }
  }
  QMMF_VERBOSE("%s Exit ", __func__);
}

status_t ZslPort::SetUpZSL() {

  QMMF_DEBUG("%s: Enter", __func__);
  bool is_fps_supported = false;

  if (0 == zsl_queue_depth_) {
    QMMF_ERROR("%s: Invalid ZSL queue depth size!", __func__);
    return -EINVAL;
  }

  for (auto &iter : context_->GetSupportedFps()) {
    if (iter == static_cast<int32_t>(params_.framerate)) {
      is_fps_supported = true;
      break;
    }
  }
  if (!is_fps_supported) {
    QMMF_ERROR("%s: Framerate: %f not supported by camera!", __func__,
        params_.framerate);
    return -EINVAL;
  }

  QMMF_INFO("%s zsl width(%d):height(%d), format=%x queue_depth=%d",
      __func__, params_.width, params_.height, params_.format,
      zsl_queue_depth_);

  auto ret = context_->ValidateResolution(params_.format, params_.width,
      params_.height);
  if (ret != 0) {
    QMMF_ERROR("%s: ZSL width(%d):height(%d) Not supported!",
        __func__, params_.width, params_.height);
    return ret;
  }

  // Create ZSL stream
  CameraStreamParameters zsl_stream_params{};
  zsl_stream_params.bufferCount = zsl_queue_depth_ + STREAM_BUFFER_COUNT;
  zsl_stream_params.format = Common::FromQmmfToHalFormat(params_.format);
  zsl_stream_params.width  = params_.width;
  zsl_stream_params.height = params_.height;
  zsl_stream_params.allocFlags.flags = IMemAllocUsage::kHwFb |
                                         IMemAllocUsage::kHwCameraZsl;
  zsl_stream_params.cb = [&](StreamBuffer buffer)
      { ZSLCaptureCallback(buffer); };

  ret = context_->CreateDeviceStream(zsl_stream_params,
                                     params_.framerate,
                                     &camera_stream_id_);
  if (0 != ret || camera_stream_id_ < 0) {
    QMMF_ERROR("%s: CreateDeviceStream failed!", __func__);
    return ret;
  }

  // Create Input stream for reprocess.
  CameraInputStreamParameters input_stream_params{};
  input_stream_params.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  input_stream_params.width  = params_.width;
  input_stream_params.height = params_.height;
  input_stream_params.get_input_buffer = [&] (StreamBuffer& buffer)
      { GetZSLInputBuffer(buffer); };
  input_stream_params.return_input_buffer  = [&] (StreamBuffer& buffer)
      { ReturnZSLInputBuffer(buffer); };

  int32_t stream_id;
  ret = context_->CreateDeviceInputStream(input_stream_params, &stream_id, true);
  if (0 != ret) {
    QMMF_ERROR("%s Failed to create input reprocess stream: %d",
               __func__, ret);
    return ret;
  }
  assert(stream_id >= 0);
  input_stream_id_ = stream_id;
  QMMF_INFO("%s: zsl input_stream_id_(%d)", __func__, input_stream_id_);

  zsl_running_ = true;
  QMMF_INFO("%s: zsl port configured with stream id(%d)", __func__,
      camera_stream_id_);

  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}

void ZslPort::ZSLCaptureCallback(StreamBuffer buffer) {

  QMMF_DEBUG("%s Enter ", __func__);
  ZSLEntry entry;
  entry.timestamp = -1;
  {
    std::lock_guard<std::mutex> l(zsl_queue_lock_);
    if (zsl_running_) {
      bool append = true;
      if (!zsl_queue_.empty()) {
        std::vector<ZSLEntry>::iterator it = zsl_queue_.begin();
        std::vector<ZSLEntry>::iterator end = zsl_queue_.end();
        while (it != end) {
          if (it->timestamp == buffer.timestamp) {
            it->buffer = buffer;
            append = false;
            break;
          }
          it++;
        }
      }

      if (append) {
        //Result is missing append to queue directly
        ZSLEntry new_entry;
        new_entry.buffer = buffer;
        new_entry.timestamp = buffer.timestamp;
        new_entry.result.clear();
        zsl_queue_.push_back(new_entry);
      }

      if (zsl_queue_.size() >= (zsl_queue_depth_ + 1)) {
        entry = *zsl_queue_.begin(); //return oldest buffer
        zsl_queue_.erase(zsl_queue_.begin());
      }
    } else {
      entry.buffer = buffer;
      entry.timestamp = buffer.timestamp;
    }
  }

  if (entry.timestamp == entry.buffer.timestamp) {
    QMMF_DEBUG("%s Return Buffer from ZSl Queue back to camera!",
        __func__);
    assert (context_ != nullptr);
    auto ret = context_->ReturnStreamBuffer(entry.buffer);
    if (0 != ret) {
      QMMF_ERROR("%s Failed to return ZSL buffer to camera: %d",
                 __func__, ret);
    }
  }
  QMMF_DEBUG("%s Exit ", __func__);
}

void ZslPort::GetZSLInputBuffer(StreamBuffer& buffer) {
  buffer = zsl_input_buffer_.buffer;
  QMMF_INFO("%s buffer(%d) submitted for reprocess!", __func__,
    buffer.fd);
}

void ZslPort::ReturnZSLInputBuffer(StreamBuffer& buffer) {

  QMMF_DEBUG("%s Enter ", __func__);
  if (buffer.handle == zsl_input_buffer_.buffer.handle) {
    assert (context_ != nullptr);
    QMMF_INFO("%s buffer(%d) returned from reprocess!", __func__,
        buffer.fd);
    auto ret = context_->ReturnStreamBuffer(zsl_input_buffer_.buffer);
    if (0 == ret) {
      zsl_input_buffer_.timestamp = -1;
    } else {
      QMMF_ERROR("%s Failed to return input buffer: %d\n", __func__,
          ret);
    }
  } else {
    QMMF_ERROR("%s: Buffer handle of returned buffer: %p doesn't match with"
        "expected handle: %p\n",  __func__, buffer.handle,
        zsl_input_buffer_.buffer.handle);
  }
  QMMF_DEBUG("%s Exit ", __func__);
}

}; // namespace recoder

}; // namespace qmmf
