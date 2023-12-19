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
*/

#define LOG_TAG "CommonFastCVResizer"

#include <cstdint>

#include "common/utils/qmmf_log.h"

#include "qmmf_resizer_fastCV.h"

uint32_t qmmf_log_level;

namespace qmmf {

FastCVResizer::FastCVResizer()
  : fastcv_level_(FASTCV_OP_CPU_PERFORMANCE),
    crop_() {
  QMMF_VERBOSE("%s: Enter", __func__);
  QMMF_VERBOSE("%s: Exit (0x%p)", __func__, this);
}

FastCVResizer::~FastCVResizer() {
  QMMF_VERBOSE("%s: Enter", __func__);
  DeInit();
  QMMF_VERBOSE("%s: Exit (0x%p)", __func__, this);
}

RESIZER_STATUS FastCVResizer::Init() {
  char prop[PROPERTY_VALUE_MAX];
  memset(prop, 0, sizeof(prop));
  property_get("persist.qmmf.fastcv.level", prop, "3");
  uint32_t level = (uint32_t) atoi(prop);

  if ((level == FASTCV_OP_LOW_POWER) ||
      (level == FASTCV_OP_PERFORMANCE) ||
      (level == FASTCV_OP_CPU_OFFLOAD) ||
      (level == FASTCV_OP_CPU_PERFORMANCE)) {
    fastcv_level_ = level;
  } else {
    fastcv_level_ = FASTCV_OP_CPU_PERFORMANCE;
  }

  int stat = fcvSetOperationMode(static_cast<fcvOperationMode>(fastcv_level_));
  QMMF_INFO("%s: set fcvSetOperationMode %d",__func__, fastcv_level_);

  if (0 != stat) {
    QMMF_ERROR("%s: Unable to set FastCV operation mode: %d", __func__, stat);
  }

  return RESIZER_STATUS_OK;
}

void FastCVResizer::DeInit() {
}

RESIZER_STATUS FastCVResizer::Draw(StreamBuffer& src_buffer,
                                   StreamBuffer& dst_buffer) {
  uint8_t *src_buffer_y, *dst_buffer_y;
  uint8_t *src_buffer_uv, *dst_buffer_uv;
  size_t src_stride_y, dst_stride_y;
  size_t src_plane_y_len, dst_plane_y_len;

  if ((src_buffer.info.format != BufferFormat::kNV21) &&
      (src_buffer.info.format != BufferFormat::kNV12) &&
      (src_buffer.info.format != BufferFormat::kNV16)) {
    QMMF_ERROR("%s: Unsupported input format: 0x%x!",__func__,
        src_buffer.info.format);
    QMMF_ERROR("%s: Only NV12/NV21 are supported currently!", __func__);
    return RESIZER_STATUS_ERROR;
  }

  src_buffer_y = reinterpret_cast<uint8_t*>(src_buffer.data);

  src_stride_y = src_buffer.info.planes[0].stride;
  src_plane_y_len = src_stride_y * src_buffer.info.planes[0].scanline;

  src_buffer_uv = src_buffer_y + src_plane_y_len;

  dst_buffer_y = reinterpret_cast<uint8_t*>(dst_buffer.data);
  dst_stride_y = dst_buffer.info.planes[0].stride;
  dst_plane_y_len = dst_stride_y * dst_buffer.info.planes[0].scanline;
  dst_buffer_uv = dst_buffer_y + dst_plane_y_len;

  //STEP2: Scale down the two planes
  fcvScaleu8_v2(src_buffer_y,
      src_buffer.info.planes[0].width,
      src_buffer.info.planes[0].height,
      src_stride_y,
      dst_buffer_y, dst_buffer.info.planes[0].width ,
      dst_buffer.info.planes[0].height,
      dst_stride_y
      ,FASTCV_INTERPOLATION_TYPE_NEAREST_NEIGHBOR ,
      FASTCV_BORDER_REPLICATE,
      0
      );

  fcvScaleDownMNInterleaveu8(src_buffer_uv,
      src_buffer.info.planes[0].width >> 1,
      src_buffer.info.planes[0].height >> 1,
      src_stride_y,
      dst_buffer_uv, dst_buffer.info.planes[0].width >> 1,
      dst_buffer.info.planes[0].height >> 1, dst_stride_y
      );

  return RESIZER_STATUS_OK;
}

RESIZER_STATUS FastCVResizer::ValidateOutput(const uint32_t width,
                                             const uint32_t height,
                                             const BufferFormat format) {
  if (format != BufferFormat::kNV21 &&
      format != BufferFormat::kNV12) {
    QMMF_ERROR("%s: Unsupported format: %d", __func__, format);
    return RESIZER_STATUS_ERROR;
  }
  return RESIZER_STATUS_OK;
}

RESIZER_STATUS FastCVResizer::Configure(const ResizerCrop& config_data) {
  if(config_data.valid) {
    crop_.width = config_data.width;
    crop_.height = config_data.height;
    crop_.x = config_data.x;
    crop_.y = config_data.y;
    crop_.valid = true;
  }

  return RESIZER_STATUS_OK;
}

} //namespace qmmf ends here
