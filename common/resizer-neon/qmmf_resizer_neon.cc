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
* Changes from Qualcomm Innovation Center, Inc. are provided under the following license:
* Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#define LOG_TAG "CommonNEONResizer"

#include <cstdint>
#include <media/msm_media_info.h>

#include "common/utils/qmmf_log.h"

#include "qmmf_resizer_neon.h"

uint32_t qmmf_log_level;

namespace qmmf {

NEONResizer::NEONResizer()
  : handle_(),
    method_(neonresizer::ResMethod::kRES_BILINEAR_V_SKIP),
    crop_() {
  QMMF_VERBOSE("%s: Enter", __func__);
  QMMF_VERBOSE("%s: Exit (0x%p)", __func__, this);
}

NEONResizer::~NEONResizer() {
  QMMF_VERBOSE("%s: Enter", __func__);
  QMMF_VERBOSE("%s: Exit (0x%p)", __func__, this);
}

RESIZER_STATUS NEONResizer::Configure(const ResizerCrop& config_data) {
  if(config_data.valid) {
    crop_.width = config_data.width;
    crop_.height = config_data.height;
    crop_.x = config_data.x;
    crop_.y = config_data.y;
    crop_.valid = true;
  }
  return RESIZER_STATUS_OK;
}

RESIZER_STATUS NEONResizer::Init() {

  char prop[PROP_VALUE_MAX];
  memset(prop, 0, sizeof(prop));
  qmmf_property_get("persist.qmmf.rescaler.method", prop, "0");
  auto value = static_cast<neonresizer::ResMethod>(atoi(prop));
  if (value < neonresizer::ResMethod::kRES_NUMBER) {
    method_ = value;
  }

  std::lock_guard<std::mutex> lock(lock_);
  auto ret = handle_.resn_init();
  if (ret != neonresizer::ResnStatus::kRESN_SUCCESS) {
    QMMF_ERROR("%s: Failed!", __func__);
    return RESIZER_STATUS_ERROR;
  }

  QMMF_INFO("%s: version: %s", __func__, handle_.resn_get_version());

  return RESIZER_STATUS_OK;
}

void NEONResizer::DeInit() {
  std::lock_guard<std::mutex> lock(lock_);
    handle_.resn_deinit();
}

RESIZER_STATUS NEONResizer::Draw(StreamBuffer& src_buffer,
                                 StreamBuffer& dst_buffer) {
  if (ValidateInParams(src_buffer, dst_buffer) != RESIZER_STATUS_OK) {
    QMMF_ERROR("%s Input validation error!!!", __func__);
    return RESIZER_STATUS_ERROR;
  }

  std::lock_guard<std::mutex> lock(lock_);
  neonresizer::Resn params;
  auto status = FillProcessParams(src_buffer, dst_buffer, params);
  assert(status == RESIZER_STATUS_OK);
  auto ret = handle_.resn_process(&params);
  if (neonresizer::ResnStatus::kRESN_SUCCESS != ret) {
    QMMF_ERROR("%s: Neon process error: %d", __func__, (int32_t) ret);
    return RESIZER_STATUS_ERROR;
  }

  return RESIZER_STATUS_OK;
}

RESIZER_STATUS NEONResizer::FillProcessParams(const StreamBuffer& src_buffer,
                                              const StreamBuffer& dst_buffer,
                                              neonresizer::Resn &params) {
  uint32_t x = 0;
  uint32_t y = 0 ;
  uint32_t width = src_buffer.info.planes[0].width;
  uint32_t height = src_buffer.info.planes[0].height;

  if (crop_.ValidateCropData(src_buffer)) {
    x = crop_.x;
    y = crop_.y ;
    width = crop_.width;
    height = crop_.height;
  } else if (aspect_ratio_preserve_) {

    double in_ar = static_cast<double>(width) / height;
    double out_ar = static_cast<double>(dst_buffer.info.planes[0].width) /
                                        dst_buffer.info.planes[0].height;
    /*save aspect ratio*/
    if (in_ar > out_ar) {
      width = out_ar * height;
      x = (src_buffer.info.planes[0].width - width) / 2;
    } else if (in_ar < out_ar) {
      height = width / out_ar;
      y = (src_buffer.info.planes[0].height - height) / 2;
    }
  }

  //default tuning should be generate internaly
  params.resn_tuning = nullptr;

  auto stride = VENUS_Y_STRIDE(COLOR_FMT_NV12, src_buffer.info.planes[0].width);
  auto scanline = VENUS_Y_SCANLINES(COLOR_FMT_NV12, src_buffer.info.planes[0].height);

  auto luma_len = stride * scanline;

  if (luma_len > src_buffer.size) {
    QMMF_ERROR("%s: Failed: Iinvalid luma length %d!", __func__, luma_len);
    return RESIZER_STATUS_ERROR;
  }

  auto src_luma_offset = y * stride + x;

  if (luma_len < src_luma_offset) {
    QMMF_ERROR("%s: Failed: Iinvalid luma offset %d!", __func__,
        src_luma_offset);
    return RESIZER_STATUS_ERROR;
  }

  params.src_luma = reinterpret_cast<unsigned char *>((intptr_t)src_buffer.data
      + src_luma_offset);

  auto src_chroma_offset = (y/2) * stride + x;
  src_chroma_offset += luma_len;

  if (src_chroma_offset > src_buffer.size) {
    QMMF_ERROR("%s: Failed: Iinvalid chroma offset %d!", __func__,
        src_chroma_offset);
    return RESIZER_STATUS_ERROR;
  }

  params.src_chroma =
      reinterpret_cast<unsigned char *>((intptr_t)src_buffer.data +
      src_chroma_offset);

  // Output data pointers
  params.dst_luma = reinterpret_cast<unsigned char *>(dst_buffer.data);

  auto chroma_len = dst_buffer.info.planes[0].stride *
                    dst_buffer.info.planes[0].scanline;
  if (chroma_len > dst_buffer.size) {
    QMMF_ERROR("%s: Failed: Iinvalid chroma len %d!", __func__, chroma_len);
    return RESIZER_STATUS_ERROR;
  }

  params.dst_chroma =
      reinterpret_cast<unsigned char *>((intptr_t)params.dst_luma + chroma_len);

  // Input buffer dimensions
  params.src_width = width;
  params.src_height = height;
  params.src_stride = stride;

  // Output buffer dimensions
  params.dst_width = dst_buffer.info.planes[0].width;
  params.dst_height = dst_buffer.info.planes[0].height;
  params.dst_stride = dst_buffer.info.planes[0].stride;

  params.res_method = method_;

  QMMF_DEBUG("%s: SRC: %s", __func__, src_buffer.info.ToString().c_str());
  QMMF_DEBUG("%s: DST: %s", __func__, dst_buffer.info.ToString().c_str());

  return RESIZER_STATUS_OK;
}

RESIZER_STATUS NEONResizer::ValidateInParams(const StreamBuffer& src_buffer,
                                             const StreamBuffer& dst_buffer) {
  if (src_buffer.data == nullptr || dst_buffer.data == nullptr) {
    QMMF_ERROR("%s Bad buffer address!!!", __func__);
    return RESIZER_STATUS_ERROR;
  }

  auto &src_info = src_buffer.info;
  auto &dst_info = dst_buffer.info;

  if (src_info.n_planes == 0 || dst_info.n_planes == 0) {
    QMMF_ERROR("%s Bad planes number!!!", __func__);
    return RESIZER_STATUS_ERROR;
  }

  auto &src_plane_info = src_info.planes[0];
  auto &dst_plane_info = dst_info.planes[0];

  if (src_plane_info.width == 0 || src_plane_info.height == 0 ||
      dst_plane_info.width == 0 || dst_plane_info.height == 0) {
    QMMF_ERROR("%s Bad img width or height size number!!!", __func__);
    return RESIZER_STATUS_ERROR;
  }

  if (dst_plane_info.width % 8) {
    QMMF_ERROR("%s Output width needs to be multiple of 8 (w: %d)!!!",
        __func__, dst_plane_info.width);
    return RESIZER_STATUS_ERROR;
  }

  return RESIZER_STATUS_OK;
}

RESIZER_STATUS NEONResizer::ValidateOutput(const uint32_t width,
                                           const uint32_t height,
                                           const BufferFormat format) {
  if (width % 8) {
    QMMF_ERROR("%s Output width needs to be multiple of 8 (w: %d)!!!",
        __func__, width);
    return RESIZER_STATUS_ERROR;
  }

  if (format != BufferFormat::kNV21 &&
      format != BufferFormat::kNV12) {
    QMMF_ERROR("%s: Unsupported format: %d", __func__, (int32_t) format);
    return RESIZER_STATUS_ERROR;
  }

  return RESIZER_STATUS_OK;
}

} //namespace qmmf ends here
