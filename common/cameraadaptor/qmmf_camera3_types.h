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

/*
 * Copyright (C) 2013 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CAMERA3TYPES_H_
#define CAMERA3TYPES_H_
#include <functional>
#include <hardware/camera_common.h>
#include <hardware/camera3.h>

#include "qmmf-sdk/qmmf_camera_metadata.h"
#include "common/utils/qmmf_common_utils.h"

using namespace android;

namespace qmmf {

namespace cameraadaptor {

// Please note that you can call all "Camera3DeviceClient" API methods
// from the same context of this callback.
typedef std::function<void(StreamBuffer buffer)> StreamCallback;

enum class CamFeatureFlag : uint32_t {
  kNone = 0,                   /// No Feature is on.
  kEIS = 1 << 0,               /// EIS is on.
  kHDR = 1 << 1,               /// HDR is on.
  kLDC = 1 << 2,               /// LDC is on.
  kLCAC = 1 << 3,              /// LCAC is on.
  kForceSensorMode = 1 << 4,   /// Force Sensor Mode is on.
  kIFEDirectStream = 1 << 5,   /// IFE Direct Stream is on.
  kInputROIEnable  = 1 << 6,   /// Input ROI reprocess is on.
#ifdef EIS_MODES_ENABLE
  kEISSingleStream = 1 << 7,   /// EIS Single Stream is on.
  kEISDualStream = 1 << 8,     /// EIS Dual Stream is on.
#endif // EIS_MODES_ENABLE
#ifdef VHDR_MODES_ENABLE
  kSHDRRaw = 1 << 9,           /// Raw SHDR line interleaved mode with 2 frame
  kSHDRYUV = 1 << 10,          /// YUV SHDR virtual channel mode with 2 frames
  kSHDRRawSwitch = 1 << 11,    /// Linear to Raw SHDR switch
  kSHDRYUVSwitch = 1 << 12,    /// Linear to YUV SHDR switch
  kQBCHDRVideo = 1 << 13,      /// QBC (in sensor) HDR on video stream is on
  kQBCHDRSnapshot = 1 << 14,   /// QBC (in sensor) HDR on snapshot is on
#endif // VHDR_MODES_ENABLE
  kOfflineIFEEnable = 1 <<15,  /// Offline IFE Enable
  kSWTNR = 1 <<16,             /// SW TNR
};

#define FORCE_SENSOR_MODE_MASK      (0x00F00000)
#define FORCE_SENSOR_MODE_DATA(idx) ((idx + 1) << 20)

typedef uint32_t CamOperationMode;

enum CamOperationModeEnum {
  kCamOpModeFrameSelectionEnum = 0,
  kCamOpModeFastSwitchEnum,
  kCamOpModeMaxEnum,
};

enum StreamUsecase {
  kStreamUsecaseNone        = 0x00000,
  kStreamUsecaseSideBySide  = 0x10002,
  kStreamUsecasePanorama    = 0x10003,
};

#define CAM_OPMODE_FLAG_FRAMESELECTION      (1 << kCamOpModeFrameSelectionEnum)
#define CAM_OPMODE_FLAG_FASTSWITCH          (1 << kCamOpModeFastSwitchEnum)
#define CAM_OPMODE_FLAG_MASK                ((1 << kCamOpModeMaxEnum) - 1)

#define CAM_OPMODE_FLAG_VALID(mode) \
  (!(mode & (~CAM_OPMODE_FLAG_MASK)))

#define CAM_OPMODE_SET_FRAMESELECTION(mode) \
  (mode = (mode | CAM_OPMODE_FLAG_FRAMESELECTION))

#define CAM_OPMODE_CLR_FRAMESELECTION(mode) \
  (mode = ((mode & (~CAM_OPMODE_FLAG_FRAMESELECTION)) & CAM_OPMODE_FLAG_MASK))

#define CAM_OPMODE_IS_FRAMESELECTION(mode) \
  (mode & CAM_OPMODE_FLAG_FRAMESELECTION)

#define CAM_OPMODE_SET_FASTSWITCH(mode) \
  (mode = (mode | CAM_OPMODE_FLAG_FASTSWITCH))

#define CAM_OPMODE_CLR_FASTSWITCH(mode) \
  (mode = ((mode & (~CAM_OPMODE_FLAG_FASTSWITCH)) & CAM_OPMODE_FLAG_MASK))

#define CAM_OPMODE_IS_FASTSWTICH(mode) \
  (mode & CAM_OPMODE_FLAG_FASTSWITCH)

struct CameraStreamParameters {
  uint32_t width;
  uint32_t height;
  int32_t format;
  android_dataspace data_space;
#if defined(CAMX_ANDROID_API) && (CAMX_ANDROID_API >= 31)
  uint64_t usecase;
  int hdrmode;
#endif
  camera3_stream_rotation_t rotation;
  MemAllocFlags allocFlags;
  uint32_t bufferCount;
  StreamCallback cb;
  ::std::string stream_camera_id;
  CameraStreamParameters() :
      width(0), height(0), format(-1), data_space(HAL_DATASPACE_UNKNOWN),
#if defined(CAMX_ANDROID_API) && (CAMX_ANDROID_API >= 31)
      usecase(0), hdrmode(0),
#endif
      rotation(CAMERA3_STREAM_ROTATION_0), allocFlags(), bufferCount(0),
      cb(nullptr), stream_camera_id() {}
};

struct CameraParameters {
  bool is_constrained_high_speed;
  bool is_raw_only;
  int8_t super_frames;
  uint32_t batch_size;
  uint32_t fps_sensormode_index;
  int32_t frame_rate_range[2];
  uint32_t cam_feature_flags;
  CamOperationMode cam_opmode;
  CameraParameters() :
      is_constrained_high_speed(false), is_raw_only(false), batch_size(1),
      super_frames(1), fps_sensormode_index(0), frame_rate_range{},
      cam_feature_flags(static_cast<uint32_t>(CamFeatureFlag::kNone)),
      cam_opmode(0) {}
};

typedef struct Camera3Request_t {
  CameraMetadata metadata;
  std::vector<int32_t> streamIds;
} Camera3Request;

typedef struct {
  int32_t  requestId;
  int32_t  burstId;
  uint32_t frameNumber;
  int32_t  partialResultCount;
  bool     input;
} CaptureResultExtras;

typedef struct {
  CameraMetadata metadata;
  CaptureResultExtras resultExtras;
} CaptureResult;

enum CameraErrorCode {
  ERROR_CAMERA_INVALID_ERROR = 0, // All other invalid error codes
  ERROR_CAMERA_DEVICE = 1,        // Un-recoverable camera error
  ERROR_CAMERA_REQUEST = 2,       // Error during request processing
  ERROR_CAMERA_RESULT = 3,        // Error when generating request result
  ERROR_CAMERA_BUFFER = 4,        // Error during buffer processing
};

// Notifies about all sorts of errors that can happen during camera operation
typedef std::function<
    void(CameraErrorCode errorCode, const CaptureResultExtras &resultExtras)>
    ErrorCallback;
// Notifies the client that camera is idle with no pending requests
typedef std::function<void()> IdleCallback;
// Notifies about a shutter event
typedef std::function<void(const CaptureResultExtras &resultExtras,
                           int64_t timestamp)> ShutterCallback;
// Notifies when stream buffers got allocated
typedef std::function<void(int streamId)> PreparedCallback;
// Notifies about a new capture result
typedef std::function<void(const CaptureResult &result)> ResultCallback;

// Please note that these callbacks shouldn't get blocked for long durations.
// Also very important is to not to try and call "Camera3DeviceClient" API
// methods
// from the same context of these callbacks. This can lead to deadlocks!
typedef struct {
  ErrorCallback errorCb;
  IdleCallback idleCb;
  ShutterCallback shutterCb;
  PreparedCallback peparedCb;
  ResultCallback resultCb;
} CameraClientCallbacks;

//Please note that this callbacks need to return as fast as possible
//otherwise the camera framerate can be affected.
typedef std::function<void(StreamBuffer &buffer)> GetInputBuffer;
typedef std::function<void(StreamBuffer &buffer)> ReturnInputBuffer;

typedef struct {
  uint32_t width;
  uint32_t height;
  int32_t format;
  GetInputBuffer get_input_buffer;
  ReturnInputBuffer return_input_buffer;
} CameraInputStreamParameters;

enum {
  NO_IN_FLIGHT_REPEATING_FRAMES = -1,
};

}  // namespace cameraadaptor ends here

}  // namespace qmmf ends here

#endif /* CAMERA3TYPES_H_ */
