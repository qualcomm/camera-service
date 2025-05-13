/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
*/

//! @file qmmf_offline_jpeg_params.h

#pragma once

namespace qmmf {

struct OfflineJpegBufferParams {
  uint32_t    width;
  uint32_t    height;
  uint32_t    format;
};

struct OfflineJpegCreateParams {
  uint32_t                process_mode;
  OfflineJpegBufferParams in_buffer;
  OfflineJpegBufferParams out_buffer;
};

struct OfflineJpegMeta {
  uint32_t quality;
  //TODO add more parameters
};

struct OfflineJpegProcessParams {
  int32_t                 in_buf_fd;
  int32_t                 out_buf_fd;
  OfflineJpegMeta         metadata;
};

}; // namespace qmmf