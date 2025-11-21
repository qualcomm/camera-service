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

/*! @file qmmf_camera_interface.h
*/

#pragma once

#include "recorder/src/service/qmmf_recorder_common.h"
#include "recorder/src/service/qmmf_recorder_utils.h"

namespace qmmf {

namespace recorder {

struct StreamParam {
  uint32_t          id;
  uint32_t          width;
  uint32_t          height;
  BufferFormat      format;
  VideoColorimetry  colorimetry;
  float             framerate;
  Rotation          rotation;
  uint32_t          xtrabufs;
  VideoFlags        flags;

  StreamParam()
      :  id(0), width(0), height(0), format(BufferFormat::kUnsupported),
         colorimetry(VideoColorimetry::kBT601), framerate(0.0),
         rotation(Rotation::kNone), xtrabufs(0), flags(VideoFlags::kNone) {}
};

struct SnapshotParam {
  ImageMode    mode;
  uint32_t     width;
  uint32_t     height;
  uint32_t     quality;
  BufferFormat format;
  Rotation     rotation;

  SnapshotParam(): mode(ImageMode::kSnapshot), width(0), height(0), quality(95),
      format(BufferFormat::kBLOB), rotation(Rotation::kNone) {}
};

class CameraInterface {
 public:

  /// CameraInterface Destructor
  virtual ~CameraInterface() {};

  /// Open the camera
  virtual status_t OpenCamera(const uint32_t camera_id,
                              const float framerate,
                              const CameraExtraParam& extra_param,
                              const ResultCb &cb = nullptr,
                              const ErrorCb &errcb = nullptr) = 0;

  /// Close the camera
  virtual status_t CloseCamera(const uint32_t camera_id) = 0;

  /// Wait AEC to converge
  virtual status_t WaitAecToConverge(const uint32_t timeout) = 0;

  /// Configure Image Capture.
  virtual status_t ConfigImageCapture(const uint32_t image_id,
                                      const SnapshotParam& param,
                                      const ImageExtraParam &xtraparam) = 0;

  /// Image Capture
  virtual status_t CaptureImage(const SnapshotType type, const uint32_t n_images,
                                const std::vector<CameraMetadata> &meta,
                                const StreamSnapshotCb& cb) = 0;

  /// Abort ongoing Image Capture. This blocking API and returns when
  /// image capture is stopped and all buffers are returned
  virtual status_t CancelCaptureImage(uint32_t image_id,
                                      const bool cache) = 0;

  /// Create stream
  virtual status_t CreateStream(const StreamParam& param,
                                const VideoExtraParam& extra_param) = 0;

  /// Delete stream
  virtual status_t DeleteStream(const uint32_t track_id) = 0;

  /// Add consumer
  virtual status_t AddConsumer(const uint32_t& track_id,
                               std::shared_ptr<IBufferConsumer>& consumer) = 0;

  /// Remove consumer
  virtual status_t RemoveConsumer(const uint32_t& track_id,
                                  std::shared_ptr<IBufferConsumer>& consumer) = 0;

  /// Start stream
  virtual status_t StartStream(const uint32_t track_id, bool cached) = 0;

  /// Stop stream
  virtual status_t StopStream(const uint32_t track_id, bool cached) = 0;

  /// Set camera parameters
  virtual status_t SetCameraParam(const CameraMetadata &meta) = 0;

  /// Return camera parameters
  virtual status_t GetCameraParam(CameraMetadata &meta) = 0;

  /// Set camera session parameters
  virtual status_t SetCameraSessionParam(const CameraMetadata &meta) = 0;

  /// Return default capture parameters
  virtual status_t GetDefaultCaptureParam(CameraMetadata &meta) = 0;

  /// Return static metadata of the camera
  virtual status_t GetCameraCharacteristics(CameraMetadata &meta) = 0;

  /// Return static metadata of all the camera's connected without opening the camera
  virtual status_t GetCamStaticInfo(std::vector<CameraMetadata> &meta) = 0;

  /// Return All Image Capture buffers
  virtual status_t ReturnAllImageCaptureBuffers() = 0;

  /// Return Image Capture buffer
  virtual status_t ReturnImageCaptureBuffer(const uint32_t camera_id,
                                            const int32_t buffer_id) = 0;

  /// Return supported fps
  virtual std::vector<int32_t>& GetSupportedFps() = 0;

  /// Set Camera SHDR mode
#ifdef VHDR_MODES_ENABLE
  virtual status_t SetVHDR(const int32_t mode) = 0;
#else
  virtual status_t SetSHDR(const bool enable) = 0;
#endif // VHDR_MODES_ENABLE
};

}; //namespace recorder.

}; //namespace qmmf.
