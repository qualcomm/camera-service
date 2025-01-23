/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
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
 * Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
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

/*! @file qmmf_recorder_extra_param_tags.h
*/

#pragma once

#include "qmmf_recorder_params.h"
#include "qmmf_recorder_extra_param.h"

namespace qmmf {

namespace recorder {

enum ParamTag {
  QMMF_SOURCE_VIDEO_TRACK_ID = (1 << 16),
  QMMF_SNAPSHOT_RAW_SETUP,
  QMMF_SNAPSHOT_ZSL_SETUP,
  QMMF_VIDEO_HDR_MODE,
  QMMF_TRACK_CROP,
  QMMF_FORCE_SENSOR_MODE,
  QMMF_EIS,
#ifdef EIS_MODES_ENABLE
  QMMF_EIS_MODE,
#endif // EIS_MODES_ENABLE
  QMMF_PARTIAL_METADATA,
  QMMF_CAMERA_SLAVE_MODE,
  QMMF_USE_LINKED_TRACK_IN_SLAVE_MODE,
  QMMF_LDC,
  QMMF_LCAC,
  QMMF_FRAME_RATE_CONTROL,
  QMMF_IFE_DIRECT_STREAM,
  QMMF_CAM_OP_MODE_CONTROL,
  QMMF_INPUT_ROI,
  QMMF_OFFLINE_IFE,
};

enum class SlaveMode {
  /**< this is not valid mode */
  kNone,
  /**< Camera Master mode */
  kMaster,
  /**< Camera Slave mode */
  kSlave,
};


enum class FrameRateControlMode {
  /**< control stream frame rate by frame skip */
  kFrameSkip,
  /**< control stream frame rate by HAL3 capture requests */
  kCaptureRequest
};

#ifdef EIS_MODES_ENABLE
enum class EisMode {
  /**< Eis is off */
  kEisOff,
  /**< EIS on first stream */
  kEisSingleStream,
  /**< EIS on dual stream */
  kEisDualStream,
};
#endif // EIS_MODES_ENABLE

enum class CamOpMode {
  /**< camera operation mode is normal */
  kNone,
  /**< camera use frame selection node to filter frames */
  kFrameSelection,
  /**< camera will switch between preview and preview plus video */
  kFastSwitch,
};

#ifdef VHDR_MODES_ENABLE
enum class VHDRMode {
  /**< VDHR is disabled */
  kVHDROff,
  /**< Raw SHDR line interleaved mode with 2 frame */
  kSHDRRaw,
  /**< YUV SHDR virtual channel mode with 2 frames */
  kSHDRYuv,
  /**< Raw SHDR mode switch enable */
  kSHDRRawSwitchEnable,
  /**< YUV SHDR mode switch enable */
  kSHDRYUVSwitchEnable,
  /**< QBC HDR video mode */
  kQBCHDRVideo,
  /**< QBC HDR snapshot mode */
  kQBCHDRSnapshot,
};
#endif // VHDR_MODES_ENABLE

struct SourceVideoTrack : DataTagBase {
  int32_t source_track_id;  // Default: -1
  SourceVideoTrack()
    : DataTagBase(QMMF_SOURCE_VIDEO_TRACK_ID),
      source_track_id(-1) {}
};

struct SnapshotRawSetup : DataTagBase {
  /**< RAW format takes place only if image mode type is kSnapshotPlusRaw. */
  /**< Default RAW format is kBayerRDI10BIT. */
  ImageFormat format;
  uint32_t    width;
  uint32_t    height;
  Rotation    rotation;

  SnapshotRawSetup()
    : DataTagBase(QMMF_SNAPSHOT_RAW_SETUP),
      format(ImageFormat::kBayerRDI10BIT), width(0), height(0),
      rotation(Rotation::kNone) {}
};

struct SnapshotZslSetup : DataTagBase {
  /**< This is output images width in kZsl image mode. */
  uint32_t    width;
  /**< This is output images height in kZsl image mode. */
  uint32_t    height;
  /**< This is output images format in kZsl image mode. */
  ImageFormat format;
  /**< This is depth if the image queue in kZsl image mode. */
  uint32_t    qdepth;

  SnapshotZslSetup()
    : DataTagBase(QMMF_SNAPSHOT_ZSL_SETUP),
      width(3840), height(2160), format(ImageFormat::kNV21), qdepth(4) {}
};

struct VideoHDRMode : DataTagBase {
#ifdef VHDR_MODES_ENABLE
  VHDRMode mode;  // Default: disable HDR
  VideoHDRMode()
    : DataTagBase(QMMF_VIDEO_HDR_MODE),
      mode(VHDRMode::kVHDROff) {}
#else
  bool enable;  // Default: false to disable HDR
  VideoHDRMode()
    : DataTagBase(QMMF_VIDEO_HDR_MODE),
      enable(false) {}
#endif // VHDR_MODES_ENABLE
};

struct TrackCrop : DataTagBase {
  // Y-axis coordinate of the crop rectangle top left starting point.
  // The coordinate system begins from the top left corner of the source.
  uint32_t x;         // Default: 0
  // X-axis coordinate of the crop rectangle top left starting point.
  // The coordinate system begins from the top left corner of the source.
  uint32_t y;         // Default: 0
  // Width in pixels of the crop rectangle.
  uint32_t width;     // Default: 0
  // Height in pixels of the crop rectangle.
  uint32_t height;    // Default: 0

  TrackCrop()
    : DataTagBase(QMMF_TRACK_CROP),
       x(0), y(0), width(0), height(0) {}
};

struct ForceSensorMode : DataTagBase {
  int32_t  mode;    // Default: -1 to disable ForceSensorMode
  ForceSensorMode()
    : DataTagBase(QMMF_FORCE_SENSOR_MODE),
      /**< Index of sensor mode to be passed by the application. */
      /**< Application needs to set the mode only once, attach this tag */
      /**< to only one of the tracks. Once all tracks are deleted, */
      /**< framework will return to auto mode selection. */
      /**< Force Sensor Mode index starts with 0. To disable this feature, */
      /**< set it to -1 in any one track, to allow auto mode selection. */
      mode(-1) {}
};

struct EISSetup : DataTagBase {
  bool enable; // Default: false to disable EIS
  EISSetup() :
    DataTagBase(QMMF_EIS),
    enable(false) {
  }
};

#ifdef EIS_MODES_ENABLE
struct EISModeSetup : DataTagBase {
  /**< Add support for electronic image stabilization mode  */
  EisMode mode;
  EISModeSetup() :
    DataTagBase(QMMF_EIS_MODE),
    mode(EisMode::kEisOff) {
  }
};
#endif // EIS_MODES_ENABLE

struct PartialMetadata : DataTagBase {
  /**< Client can configure whether it requires partial Metadata or not. */
  /**< Camera Adaptor will send the partial data to camera context */
  /**< irrespective of clients needs it or not.Its responsibility  */
  /**< of context to check whether to propagate the partial data to */
  /**< client or not. Default: false to disable PartialMetadata*/
  bool enable;
  PartialMetadata() :
    DataTagBase(QMMF_PARTIAL_METADATA),
    enable(false) {
  }
};

struct CameraSlaveMode : DataTagBase {
  /**< Add support for multi client support for same camera. */
  /**< Client can open a given camera in slave mode. */
  /**< The camera being opened as slave needs to be already opened by */
  /**< another client which uses it as master. If this requirement is */
  /**< not fulfilled then the slave client needs to wait until signaled */
  /**< on event from the service. Default: SlaveMode::kNone*/
  SlaveMode mode;
  CameraSlaveMode() :
    DataTagBase(QMMF_CAMERA_SLAVE_MODE),
    mode(SlaveMode::kNone) {
  }
};

struct LinkedTrackInSlaveMode : DataTagBase {
  /**< Add support for client to enable/disable linked track in slave mode. */
  /**< Default: True*/
  bool enable;
  LinkedTrackInSlaveMode() :
    DataTagBase(QMMF_USE_LINKED_TRACK_IN_SLAVE_MODE),
    enable(false) {
  }
};

struct LDCMode : DataTagBase {
  /**< Add support for client to enable/disable */
  /**< LDC (Lens Distortion Correction). */
  /**< Default: False */
  bool enable;
  LDCMode() :
    DataTagBase(QMMF_LDC), enable(false) {
  }
};

struct LCACMode : DataTagBase {
  /**< Add support for client to enable/disable */
  /**< LCAC (Lateral Chromatic Aberration Correction). */
  /**< Default: False */
  bool enable;
  LCACMode() :
    DataTagBase(QMMF_LCAC), enable(false) {
  }
};

struct FrameRateControl : DataTagBase {
  /**< Add support for stream frame rate control mode  */
  FrameRateControlMode mode;
  FrameRateControl() :
    DataTagBase(QMMF_FRAME_RATE_CONTROL),
    mode(FrameRateControlMode::kFrameSkip) {
  }
};

struct IFEDirectStream: DataTagBase {
  /**< Add support for client to enable/disable */
  /**< enable IFE direct stream */
  /**< Default: False */
  bool enable;
  IFEDirectStream() :
    DataTagBase(QMMF_IFE_DIRECT_STREAM), enable(false) {
  }
};

struct CamOpModeControl: DataTagBase {
  CamOpMode mode;    // Default: kNone
  CamOpModeControl()
    : DataTagBase(QMMF_CAM_OP_MODE_CONTROL),
      /**< add to support special camera pipelines */
      /**< by default, no special camera mode will be passed */
      mode(CamOpMode::kNone) {}
};

struct InputROISetup: DataTagBase {
  /**< Add support for client to enable/disable */
  /**< Input ROI reprocess usecase */
  /**< Default: False */
  bool enable;
  InputROISetup() :
    DataTagBase(QMMF_INPUT_ROI), enable(false) {
  }
};

  struct OfflineIFE: DataTagBase {
  /**< Add support for client to enable/disable */
  /**< Offline IFE usecase */
  /**< Default: False */
  bool enable;
  OfflineIFE() :
    DataTagBase(QMMF_OFFLINE_IFE), enable(false) {
  }
};

}; //namespace recorder.

}; //namespace qmmf.
