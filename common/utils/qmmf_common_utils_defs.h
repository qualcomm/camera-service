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

#pragma once

#include <sstream>
#include <iomanip>

#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>

#include "qmmf_memory_interface_defs.h"
#include "qmmf-sdk/qmmf_recorder_params.h"

namespace qmmf {

const int64_t kWaitDelay = 2000000000;  // 2 sec
const uint32_t kMaxSocketBufSize = 300000;

inline const char* kCameraMetaDataLibName = "libcamera_metadata";

#define FORCE_SENSOR_MODE_MASK (0x00F00000)
#define FORCE_SENSOR_MODE_DATA(idx) ((idx + 1) << 20)

#define SOC_DEV_PATH_PRIMARY "/sys/devices/soc0/soc_id"
#define SOC_DEV_PATH_SECONDARY "/sys/devices/system/soc/soc0/id"
#define CHIPSET_BUFFER_SIZE 32

enum class SocId {
  kInvalid = 0,
  kKODIAK_SM = 475,
  kKODIAK_SC7280 = 487,
  kKODIAK_SC7295 = 488,
  kQCM6490_IOT = 497,
  kQCS6490 = 498,
  kKODIAK_APQ = 499,
  kKODIAK_LTE_ONLY = 515,
  kLEMANS_IVI = 532,
  kLEMANS_ADAS_H = 533,
  kLEMANS_IVI_ADAS = 534,
  kLEMANS_ADAS = 535,
  kKODIAK_SC7280P = 546,
  kKODIAK_SC8270 = 553,
  kKODIAK_SC8270P = 563,
  kKODIAK_SC7270P = 567,
  kQCS5430_LITE = 575,
  kQCM5430_LITE = 576,
  kMONACO_ADAS = 605,
  kMONACO_IVI = 606,
  kMONACO_SRV1L = 607,
  kLEMANS_IVI_ADAS_L = 619,
  kMONACO_SRV1L_FC = 620,
  KLEMANS_QRB = 656,
  kQCS9100_IOT = 667,
  kQCS8300 = 674,
  kQCS8275 = 675,
  kQCS9075 = 676,
  kMONACO_FLEX = 695,
};

struct StreamBuffer {
  BufferMeta info;
  int64_t timestamp;
  uint32_t frame_number;
  uint32_t camera_id;
  int32_t stream_id;
  uint64_t data_space;
  IBufferHandle handle;
  int32_t fd;
  uint32_t size;
  int32_t metafd;
  void *data;
  uint32_t flags;
  bool second_thumb;
  bool in_use_camera;
  bool in_use_client;

  ::std::string ToString() const {
    ::std::stringstream stream;
    stream << "camera[" << camera_id << "] ";
    stream << "stream[" << stream_id << "] ";
    stream << "data[" << data << "] ";
    stream << "fd[" << fd << "] ";
    stream << "size[" << size << "] ";
    stream << "timestamp[" << timestamp << "] ";
    stream << "flags[" << ::std::setbase(16) << flags << ::std::setbase(10)
           << "]";
    stream << "second_thumb[" << second_thumb << "] ";
    stream << "in_use_client[" << in_use_client << "] ";
    stream << "in_use_camera[" << in_use_camera << "] ";
    return stream.str();
  }
};

// Please note that you can call all "Camera3DeviceClient" API methods
// from the same context of this callback.
typedef std::function<void(StreamBuffer buffer)> StreamCallback;
typedef uint32_t CamOperationMode;

enum class CamFeatureFlag : uint32_t {
  kNone = 0,                    /// No Feature is on.
  kEIS = 1 << 0,                /// EIS is on.
  kHDR = 1 << 1,                /// HDR is on.
  kLDC = 1 << 2,                /// LDC is on.
  kLCAC = 1 << 3,               /// LCAC is on.
  kForceSensorMode = 1 << 4,    /// Force Sensor Mode is on.
  kIFEDirectStream = 1 << 5,    /// IFE Direct Stream is on.
  kInputROIEnable = 1 << 6,     /// Input ROI reprocess is on.
  kEISSingleStream = 1 << 7,    /// EIS Single Stream is on.
  kEISDualStream = 1 << 8,      /// EIS Dual Stream is on.
  kSHDRRaw = 1 << 9,            /// Raw SHDR line interleaved mode with 2 frame
  kSHDRYUV = 1 << 10,           /// YUV SHDR virtual channel mode with 2 frames
  kSHDRRawSwitch = 1 << 11,     /// Linear to Raw SHDR switch
  kSHDRYUVSwitch = 1 << 12,     /// Linear to YUV SHDR switch
  kQBCHDRVideo = 1 << 13,       /// QBC (in sensor) HDR on video stream is on
  kQBCHDRSnapshot = 1 << 14,    /// QBC (in sensor) HDR on snapshot is on
  kOfflineIFEEnable = 1 << 15,  /// Offline IFE Enable
  kSWTNR = 1 << 16,             /// SW TNR
};

enum CamOperationModeEnum {
  kCamOpModeFrameSelectionEnum = 0,
  kCamOpModeFastSwitchEnum,
  kCamOpModeMaxEnum,
};

enum StreamUsecase {
  kStreamUsecaseNone = 0x00000,
  kStreamUsecaseSideBySide = 0x10002,
  kStreamUsecasePanorama = 0x10003,
};

#define CAM_OPMODE_FLAG_FRAMESELECTION (1 << kCamOpModeFrameSelectionEnum)
#define CAM_OPMODE_FLAG_FASTSWITCH (1 << kCamOpModeFastSwitchEnum)
#define CAM_OPMODE_FLAG_MASK ((1 << kCamOpModeMaxEnum) - 1)

#define CAM_OPMODE_FLAG_VALID(mode) (!(mode & (~CAM_OPMODE_FLAG_MASK)))

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

#define CAM_OPMODE_IS_FASTSWTICH(mode) (mode & CAM_OPMODE_FLAG_FASTSWITCH)

struct CameraStreamParameters {
  uint32_t width;
  uint32_t height;
  int32_t format;
  uint32_t data_space;
  int32_t color_space;
  uint64_t usecase;
  int32_t hdrmode;
  int32_t rotation;
  MemAllocFlags allocFlags;
  uint32_t bufferCount;
  StreamCallback cb;
  ::std::string stream_camera_id;
  CameraStreamParameters()
      : width(0),
        height(0),
        format(-1),
        data_space(0x0),
        color_space(0),
        usecase(0),
        hdrmode(0),
        rotation(0),
        allocFlags(),
        bufferCount(0),
        cb(nullptr),
        stream_camera_id() {}
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
  CameraParameters()
      : is_constrained_high_speed(false),
        is_raw_only(false),
        batch_size(1),
        super_frames(1),
        fps_sensormode_index(0),
        frame_rate_range{},
        cam_feature_flags(static_cast<uint32_t>(CamFeatureFlag::kNone)),
        cam_opmode(0) {}
};

typedef struct Camera3Request_t {
  CameraMetadata metadata;
  std::vector<int32_t> streamIds;
} Camera3Request;

typedef struct {
  int32_t requestId;
  int32_t burstId;
  uint32_t frameNumber;
  int32_t partialResultCount;
  bool input;
} CaptureResultExtras;

typedef struct {
  CameraMetadata metadata;
  CaptureResultExtras resultExtras;
} CaptureResult;

enum CameraErrorCode {
  ERROR_CAMERA_INVALID_ERROR = 0,  // All other invalid error codes
  ERROR_CAMERA_DEVICE = 1,         // Un-recoverable camera error
  ERROR_CAMERA_REQUEST = 2,        // Error during request processing
  ERROR_CAMERA_RESULT = 3,         // Error when generating request result
  ERROR_CAMERA_BUFFER = 4,         // Error during buffer processing
};

enum SystemEventMessage {
  MSG_SYSTEM_SOFFREEZE = 1,
  MSG_SYSTEM_RECOVERYFAILURE = 2,
  MSG_SYSTEM_FATAL = 3,
  MSG_SYSTEM_RECOVERYSUCCESS = 4,
  MSG_SYSTEM_INTERNAL_RECOVERY = 5,
};

// Notifies about all sorts of errors that can happen during camera operation
typedef std::function<void(CameraErrorCode errorCode,
                           const CaptureResultExtras &resultExtras)>
    ErrorCallback;
// Notifies the client that camera is idle with no pending requests
typedef std::function<void()> IdleCallback;
// Notifies about a shutter event
typedef std::function<void(const CaptureResultExtras &resultExtras,
                           int64_t timestamp)>
    ShutterCallback;
// Notifies when stream buffers got allocated
typedef std::function<void(int streamId)> PreparedCallback;
// Notifies about a new capture result
typedef std::function<void(const CaptureResult &result)> ResultCallback;
// Notifies about all sorts of system messages that can happen during camera
// operation
typedef std::function<void(uint32_t errorCode)> SystemCallback;

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
  SystemCallback systemCb;
} CameraClientCallbacks;

// Please note that this callbacks need to return as fast as possible
// otherwise the camera framerate can be affected.
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

class Target {
 public:
  // GetSocId
  // @brief  Query for HW for the SoC Id
  // @return socId returned by the HW being probed; SocId::Invalid on failure
  static SocId GetSocId() {
    static SocId id = SocId::kInvalid;

    if (id != SocId::kInvalid) return id;

    int32_t soc_fd;
    char buf[CHIPSET_BUFFER_SIZE] = {0};

    if (0 == access(SOC_DEV_PATH_PRIMARY, F_OK)) {
      soc_fd = open(SOC_DEV_PATH_PRIMARY, O_RDONLY);
    } else {
      soc_fd = open(SOC_DEV_PATH_SECONDARY, O_RDONLY);
    }
    if (soc_fd >= 0) {
      auto ret = read(soc_fd, buf, sizeof(buf) - 1);
      if (-1 != ret) id = static_cast<SocId>(atoi(buf));
      close(soc_fd);
    }
    return id;
  }

  // FileExists
  // @param[in] name     Base name of the shared library
  // @param[in] version  Library version string
  // @brief  Return true/false if file exist
  // @return bool
  static bool FileExists(const std::string &name, const std::string &version) {
    std::string full_path = "/usr/lib/" + name + ".so." + version;
    return access(full_path.c_str(), F_OK) == 0;
  }

  // GetLibName
  // @param[in] name     Base name of the shared library
  // @param[in] version  Library version string
  // @brief
  //   If platform-specific lib exits  → return the lib name with suffix.
  //   else returns lib name without suffix.
  // @return string return for libname
  static std::string GetLibName(const std::string &name,
                                const std::string &version) {
    std::string lib_name;

    auto soc_id = Target::GetSocId();
    auto suffix = Target::GetName(soc_id);
    lib_name = name + "_" + std::string(suffix);

    auto file_exist = Target::FileExists(lib_name , version);

    if (file_exist) {
      return lib_name + ".so." + version;
    }

    return name + ".so." + version;
  }

  // GetName
  // @param[in] id Soc id of the target
  // @brief  Return the target name based on SOC ID
  // @return string return for soc ID
  static std::string GetName(SocId id) {
    switch (id) {
      case SocId::kQCM6490_IOT:
      case SocId::kQCS6490:
      case SocId::kQCS5430_LITE:
      case SocId::kQCM5430_LITE:
      case SocId::kKODIAK_SM:
      case SocId::kKODIAK_SC7280:
      case SocId::kKODIAK_SC7295:
      case SocId::kKODIAK_APQ:
      case SocId::kKODIAK_LTE_ONLY:
      case SocId::kKODIAK_SC7280P:
      case SocId::kKODIAK_SC8270:
      case SocId::kKODIAK_SC8270P:
      case SocId::kKODIAK_SC7270P:
        return "kodiak";

      case SocId::kQCS9100_IOT:
      case SocId::kQCS9075:
      case SocId::kLEMANS_IVI:
      case SocId::kLEMANS_ADAS_H:
      case SocId::kLEMANS_IVI_ADAS:
      case SocId::kLEMANS_ADAS:
      case SocId::kLEMANS_IVI_ADAS_L:
      case SocId::KLEMANS_QRB:
      case SocId::kQCS8300:
      case SocId::kQCS8275:
      case SocId::kMONACO_IVI:
      case SocId::kMONACO_ADAS:
      case SocId::kMONACO_SRV1L:
      case SocId::kMONACO_SRV1L_FC:
      case SocId::kMONACO_FLEX:
        return "lemans";

      default:
        return {};
    }
  }
};

};  // namespace qmmf.
