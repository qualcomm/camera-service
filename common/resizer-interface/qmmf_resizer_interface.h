/*
* Copyright (c) 2018, 2020-2021, The Linux Foundation. All rights reserved.
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

#include "common/cameraadaptor/qmmf_camera3_utils.h"

namespace qmmf {

#define PRESERVE_ASPECT_RATIO "persist.qmmf.rescaler.ar_pre"

typedef enum {
  RESIZER_STATUS_ERROR = -1,
  RESIZER_STATUS_OK = 0
} RESIZER_STATUS;

struct ResizerCrop {
  uint32_t x;
  uint32_t y;
  uint32_t width;
  uint32_t height;
  bool valid;
  ResizerCrop() : x(0), y(0), width(0), height(0), valid(false) {};
  bool ValidateCropData(const StreamBuffer& buffer) {
    if (valid && (width > 0) && (height > 0) &&
       (width <= buffer.info.planes[0].width) &&
       (height <= buffer.info.planes[0].height) &&
       (x < buffer.info.planes[0].width) &&
       (y < buffer.info.planes[0].height)) {
      return true;
    }
    return false;
  };
};

class ResizerInterface {

 public:

  virtual ~ResizerInterface() {};

  virtual RESIZER_STATUS Init() = 0;

  virtual void DeInit() = 0;

  virtual RESIZER_STATUS Configure(const ResizerCrop& config_data) = 0;

  virtual RESIZER_STATUS Draw(StreamBuffer& src_buffer,
                              StreamBuffer& dst_buffer) = 0;

  virtual RESIZER_STATUS ValidateOutput(const uint32_t width,
                                        const uint32_t height,
                                        const BufferFormat format) = 0;

  bool aspect_ratio_preserve_;
};

}; //namespace qmmf.
