/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <sys/types.h>

#include "qmmf_camera_metadata.h"
#include <qmmf-sdk/qmmf_recorder_params.h>

namespace qmmf {

#define OFFLINE_CAMERA_REQ_METADATA_PATH_MAX 128

// Offline Processing mode
enum OfflinePostProcMode
{
  YUVToJPEG    = 0,
  YUVToYUV     = 1,
  RAWToYUV     = 2,
  RAWToJPEGSBS = 3,
};

struct OfflineCameraBufferParams {
  uint32_t                    width;
  uint32_t                    height;
  qmmf::recorder::VideoFormat format;
};

// Input parameters of offline camera create function
// Currently, a maximum of two inputs are supported. The format of the inputs
// must be the same, while other parameters can be different.
struct OfflineCameraCreateParams {
  // The camera id corresponding to the input data
  uint32_t camera_id[2];
  // The information of input data
  OfflineCameraBufferParams in_buffer;
  // The information of output data
  OfflineCameraBufferParams out_buffer;
  // The processing mode of offline camera function, such as: YUVToYUV
  uint32_t process_mode;
  // The step size for setting the frame metadata
  uint32_t metadata_step[2];
  // The file path of frame metadata, this file lists the names of all metadata binary
  // file, The Post-Proc library of CAMX will read the medadata bin one by one according to
  // the names listed in this file and configure the metadata for input frame.
  char request_metadata_path[2]
      [OFFLINE_CAMERA_REQ_METADATA_PATH_MAX];
  // The meta data for the session
  CameraMetadata session_meta[2];
};

// Input parameters of offline camera process function
struct OfflineCameraProcessParams {
  // The fd of input GBM buffer
  int32_t in_buf_fd[2];
  // The fd of output GBM buffer
  int32_t out_buf_fd;
  // The metadata in order to configure each frame data
  CameraMetadata meta;
};

};  // namespace qmmf
