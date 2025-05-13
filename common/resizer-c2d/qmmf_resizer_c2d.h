/*
 * Copyright (c) 2018, 2020, The Linux Foundation. All rights reserved.
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

#include <map>
#include <mutex>

#include "common/resizer-interface/qmmf_resizer_interface.h"

namespace qmmf {

class C2DResizer : public ResizerInterface {

 public:

  C2DResizer();

  ~C2DResizer();

  RESIZER_STATUS Init() override;

  void DeInit() override;

  RESIZER_STATUS Configure(const ResizerCrop& config_data) override;

  RESIZER_STATUS Draw(StreamBuffer& src_buffer,
                      StreamBuffer& dst_buffer) override;

  RESIZER_STATUS ValidateOutput(const uint32_t width,
                                const uint32_t height,
                                const BufferFormat format) override;

  void* MapBuf(StreamBuffer& buffer);

  void UnMapBufs();

  bool BufferFormatToC2D(StreamBuffer &buffer, uint32_t &c2d_color_format);

  RESIZER_STATUS UpdateRGBSurface(StreamBuffer& dst_buffer,
                                  uint32_t &c2d_color_format,
                                  void *dst_buf_gpu_addr);

  RESIZER_STATUS UpdateYUVSurface(StreamBuffer& src_buffer,
                                  StreamBuffer& dst_buffer,
                                  uint32_t &c2d_color_format,
                                  void *dst_buf_gpu_addr);

 private:

  uint32_t src_surface_id_;
  uint32_t dst_surface_id_;
  uint32_t dst_surface_rgb_id_;

  std::map<uint32_t, void*> mapped_buffs_;

  std::mutex crop_lock_;
  ResizerCrop crop_;

};

}; //namespace qmmf ends here
