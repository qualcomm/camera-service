/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <functional>
#include <vector>

#include "common/utils/qmmf_common_utils_defs.h"
#include "qmmf-sdk/qmmf_camera_metadata.h"

namespace qmmf {
namespace cameraadaptor {

class ICameraDeviceClient {
 public:
  virtual ~ICameraDeviceClient() = default;

  virtual int32_t Initialize() = 0;
  virtual int32_t OpenCamera(uint32_t camera_id) = 0;
  virtual int32_t WaitUntilIdle() = 0;
  virtual int32_t Flush(int64_t *last_frame_number) = 0;
  virtual int32_t TearDown(int streamId) = 0;

  virtual int32_t BeginConfigure() = 0;
  virtual int32_t EndConfigure(
      const CameraParameters &stream_config = CameraParameters()) = 0;

  virtual int32_t CreateStream(
      const CameraStreamParameters &outputConfiguration) = 0;
  virtual int32_t CreateInputStream(
      const CameraInputStreamParameters &inputConfiguration) = 0;
  virtual int32_t DeleteStream(int streamId, bool cache) = 0;
  virtual int32_t Prepare(int streamId) = 0;

  virtual int32_t CreateDefaultRequest(int templateId,
                                       CameraMetadata *request) = 0;
  virtual int32_t SubmitRequest(Camera3Request request, bool streaming = false,
                                int64_t *last_frame_number = NULL) = 0;
  virtual int32_t SubmitRequestList(std::vector<Camera3Request> requests,
                                    bool streaming = false,
                                    int64_t *last_frame_number = NULL) = 0;
  virtual int32_t CancelRequest(int requestId,
                                int64_t *lastFrameNumber = NULL) = 0;

  virtual int32_t ReturnStreamBuffer(StreamBuffer buffer) = 0;

  virtual int32_t GetCameraInfo(uint32_t idx, CameraMetadata *info) = 0;
  virtual int32_t GetNumberOfCameras() = 0;

  virtual int32_t SetCameraSessionParam(const CameraMetadata &meta) = 0;
  virtual int32_t UpdateCameraParams(
      const CameraParameters &camera_parameters = CameraParameters()) = 0;
  virtual const std::vector<int32_t> GetRequestIds() = 0;
};
}  // namespace cameraadaptor
}  // namespace qmmf
