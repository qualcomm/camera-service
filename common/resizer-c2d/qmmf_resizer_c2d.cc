/*
* Copyright (c) 2018-2021, The Linux Foundation. All rights reserved.
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

#define LOG_TAG "CommonC2DResizer"

#include <cstdint>
#include <adreno/c2d2.h>
#include <linux/msm_kgsl.h>
#include <media/msm_media_info.h>

#include "common/utils/qmmf_log.h"

#include "qmmf_resizer_c2d.h"

#define TOQ16(x) (x << 16)

uint32_t qmmf_log_level;

namespace qmmf {

C2DResizer::C2DResizer()
    : src_surface_id_(0),
      dst_surface_id_(0),
      dst_surface_rgb_id_(0),
      mapped_buffs_({}),
      crop_() {
  QMMF_VERBOSE("%s: Enter", __func__);
  QMMF_VERBOSE("%s: Exit (0x%p)", __func__, this);
}

C2DResizer::~C2DResizer() {
  QMMF_VERBOSE("%s: Enter", __func__);
  DeInit();
  QMMF_VERBOSE("%s: Exit (0x%p)", __func__, this);
}

RESIZER_STATUS C2DResizer::Init() {

  C2D_YUV_SURFACE_DEF surface_def = {
    C2D_COLOR_FORMAT_420_NV12,
    1 * 4,
    1 * 4,
    (void*)0xaaaaaaaa,
    (void*)0xaaaaaaaa,
    1 * 4,
    (void*)0xaaaaaaaa,
    (void*)0xaaaaaaaa,
    1 * 4,
    (void*)0xaaaaaaaa,
    (void*)0xaaaaaaaa,
    1 * 4,
  };

  C2D_SURFACE_TYPE surface_type = static_cast<C2D_SURFACE_TYPE>(
                                  C2D_SURFACE_YUV_HOST  |
                                  C2D_SURFACE_WITH_PHYS |
                                  C2D_SURFACE_WITH_PHYS_DUMMY);

  //target C2dSurface for output buffers. at this point surface
  //is dummy, it is not mapped to GPU, it will be mapped
  //to GPU once input buffers (camera stream buffers) will be
  //available for copy.
  auto ret = c2dCreateSurface(&dst_surface_id_, C2D_TARGET, surface_type,
                              &surface_def);
  if(ret != C2D_STATUS_OK) {
    QMMF_ERROR("%s: c2dCreateSurface failed! %d", __func__, ret);
    return RESIZER_STATUS_ERROR;
  }

  //Dummy surface for camera stream buffers, this surface will be
  //updated by actual camera buffers.
  ret = c2dCreateSurface(&src_surface_id_, C2D_SOURCE, surface_type,
                         &surface_def);
  if(ret != C2D_STATUS_OK) {
    QMMF_ERROR("%s: c2dCreateSurface failed! %d", __func__, ret);
    return RESIZER_STATUS_ERROR;
  }

  C2D_RGB_SURFACE_DEF surface_def_rgb {};

  C2D_SURFACE_TYPE surface_type_rgb = static_cast<C2D_SURFACE_TYPE>(
                                  C2D_SURFACE_RGB_HOST  |
                                  C2D_SURFACE_WITH_PHYS |
                                  C2D_SURFACE_WITH_PHYS_DUMMY);

  ret = c2dCreateSurface(&dst_surface_rgb_id_, C2D_TARGET,
                         surface_type_rgb, &surface_def_rgb);
  if(ret != C2D_STATUS_OK) {
    QMMF_ERROR("%s: c2dCreateSurface failed! %d", __func__, ret);
    return RESIZER_STATUS_ERROR;
  }

  return RESIZER_STATUS_OK;
}

void C2DResizer::DeInit() {
  UnMapBufs();
  if(dst_surface_id_) {
    c2dDestroySurface(dst_surface_id_);
    dst_surface_id_ = 0;
  }
  if(src_surface_id_) {
    c2dDestroySurface(src_surface_id_);
    src_surface_id_ = 0;
  }
  if(dst_surface_rgb_id_) {
    c2dDestroySurface(dst_surface_rgb_id_);
    dst_surface_rgb_id_ = 0;
  }
}

RESIZER_STATUS C2DResizer::Configure(const ResizerCrop& config_data) {
  if(config_data.valid) {
    crop_.width = config_data.width;
    crop_.height = config_data.height;
    crop_.x = config_data.x;
    crop_.y = config_data.y;
    crop_.valid = true;
  }
  return RESIZER_STATUS_OK;
}

RESIZER_STATUS C2DResizer::Draw(StreamBuffer& src_buffer,
                                  StreamBuffer& dst_buffer) {
  C2D_STATUS ret = C2D_STATUS_OK;
  RESIZER_STATUS status = RESIZER_STATUS_OK;
  void* src_buf_gpu_addr = nullptr;
  void* dst_buf_gpu_addr = nullptr;
  int32_t plane_y_len = 0;
  uint32_t c2d_color_format = C2D_COLOR_FORMAT_420_NV12;
  C2D_SURFACE_TYPE type;

  uint32_t x = 0, y = 0;
  uint32_t w = src_buffer.info.planes[0].width;
  uint32_t h = src_buffer.info.planes[0].height;
  double in_ar = 0, out_ar = 0;

  QMMF_DEBUG("%s: src_buffer.fd = %d", __func__, src_buffer.fd);
  QMMF_DEBUG("%s: src_buffer.size = %d", __func__, src_buffer.size);

  if(src_buffer.data == nullptr) {
    QMMF_ERROR("%s: Invalid src_buf_vaddr!", __func__);
    status = RESIZER_STATUS_ERROR;
    goto EXIT;
  }

  //STEP2: Map Input Camera stream buffer to GPU.
  src_buf_gpu_addr = MapBuf(src_buffer);
  if(src_buf_gpu_addr == nullptr) {
    QMMF_ERROR("%s: Invalid src_buf_gpu_addr!", __func__);
    status = RESIZER_STATUS_ERROR;
    goto EXIT;
  }

  //STEP3: Map target ION buffer to GPU.
  dst_buf_gpu_addr = MapBuf(dst_buffer);
  if(dst_buf_gpu_addr == nullptr) {
    QMMF_ERROR("%s: Invalid dst_buf_gpu_addr!", __func__);
    status = RESIZER_STATUS_ERROR;
    goto EXIT;
  }

  //STEP4: Create source C2dSurface for input Camera stream buffer.
  if ((src_buffer.info.planes[0].width == 0) ||
      (src_buffer.info.planes[0].height == 0)) {
    QMMF_ERROR("%s: Invalid Src size!", __func__);
    status = RESIZER_STATUS_ERROR;
    goto EXIT;
  }

  if (!BufferFormatToC2D(src_buffer, c2d_color_format)) {
    status = RESIZER_STATUS_ERROR;
    goto EXIT;
  }

  C2D_YUV_SURFACE_DEF src_surface;
  //destination format.
  src_surface.format  = c2d_color_format;
  //destination width.
  src_surface.width   = src_buffer.info.planes[0].width;
  //destination height.
  src_surface.height  = src_buffer.info.planes[0].height;
  //Y plane stride.
  src_surface.stride0 = src_buffer.info.planes[0].stride;
  //UV plane stride.
  src_surface.stride1 = src_buffer.info.planes[0].stride;
  //UV plane hostptr.
  plane_y_len = src_surface.stride0 * src_buffer.info.planes[0].scanline;
  //Y plane hostptr.
  src_surface.plane0 = src_buffer.data;
  //Y plane Gpu address.
  src_surface.phys0   = src_buf_gpu_addr;

  if (src_buffer.info.format == BufferFormat::kNV12UBWC) {
    plane_y_len =
        MSM_MEDIA_ALIGN(VENUS_Y_META_STRIDE(COLOR_FMT_NV12_UBWC,
                                            src_surface.width) *
                        VENUS_Y_META_SCANLINES(COLOR_FMT_NV12_UBWC,
                                               src_surface.height), 4096) +
        MSM_MEDIA_ALIGN(VENUS_Y_STRIDE(COLOR_FMT_NV12_UBWC,
                                       src_surface.width) *
                        VENUS_Y_SCANLINES(COLOR_FMT_NV12_UBWC,
                                          src_surface.height), 4096);
  } else if (src_buffer.info.format == BufferFormat::kNV12 ||
             src_buffer.info.format == BufferFormat::kNV21) {
    plane_y_len =
         VENUS_Y_STRIDE(COLOR_FMT_NV12, src_surface.width) *
         VENUS_Y_SCANLINES(COLOR_FMT_NV12, src_surface.height);
  }

  //UV plane hostptr.
  src_surface.plane1  = (void*)((intptr_t)src_buffer.data + plane_y_len);
  //UV plane Gpu address.
  src_surface.phys1 = (void*)((intptr_t)src_buf_gpu_addr + plane_y_len);

  QMMF_DEBUG("%s: src_surface.width = %d ", __func__, src_surface.width);
  QMMF_DEBUG("%s: src_surface.height = %d ", __func__, src_surface.height);
  QMMF_DEBUG("%s: src_surface.stride0 = %d ", __func__, src_surface.stride0);
  QMMF_DEBUG("%s: src_surface.stride1 = %d ", __func__, src_surface.stride1);
  QMMF_DEBUG("%s: plane_y_len = %d", __func__, plane_y_len);

  type = static_cast<C2D_SURFACE_TYPE>(
      C2D_SURFACE_YUV_HOST | C2D_SURFACE_WITH_PHYS);
  ret = c2dUpdateSurface(src_surface_id_, C2D_SOURCE, type, &src_surface);

  if(ret != C2D_STATUS_OK) {
    QMMF_ERROR("%s: c2dUpdateSurface failed! %d", __func__, ret);
    status = RESIZER_STATUS_ERROR;
    goto EXIT;
  }
  QMMF_DEBUG("%s: src_surface_id_ = %d", __func__, src_surface_id_);

  if (!BufferFormatToC2D(dst_buffer, c2d_color_format)) {
    status = RESIZER_STATUS_ERROR;
    goto EXIT;
  }

  //STEP5: Update target C2dSurface.
  uint32_t dst_surface_id;
  if (c2d_color_format == C2D_COLOR_FORMAT_888_RGB) {
    status = UpdateRGBSurface(dst_buffer, c2d_color_format,
                              dst_buf_gpu_addr);
    dst_surface_id = dst_surface_rgb_id_;
  } else {
    status = UpdateYUVSurface(src_buffer, dst_buffer,
                              c2d_color_format, dst_buf_gpu_addr);
    dst_surface_id = dst_surface_id_;
  }

  if(status != RESIZER_STATUS_OK) {
    goto EXIT;
  }

  //STEP6: save aspect ratio
  if (crop_.ValidateCropData(src_buffer)) {
    x = crop_.x;
    y = crop_.y;
    w = crop_.width;
    h = crop_.height;
  } else if (aspect_ratio_preserve_) {
    in_ar = static_cast<double>(w) / h;
    out_ar = static_cast<double>(dst_buffer.info.planes[0].width) /
                                 dst_buffer.info.planes[0].height;

    if (in_ar > out_ar) {
      w = out_ar * h;
      x = (src_buffer.info.planes[0].width - w) / 2;
    } else if (in_ar < out_ar) {
      h = w / out_ar;
      y = (src_buffer.info.planes[0].height - h) / 2;
    }
  }

  //STEP7: Create C2dObject outof source surface and fill target rectangle
  //values.
  C2D_OBJECT draw_obj[1];
  draw_obj[0].surface_id  = src_surface_id_;
  draw_obj[0].config_mask = C2D_ALPHA_BLEND_NONE | C2D_TARGET_RECT_BIT;

  if ((0 < dst_buffer.info.planes[0].width) &&
      (0 < dst_buffer.info.planes[0].height)) {
    {
      std::lock_guard<std::mutex> l(crop_lock_);
      draw_obj[0].config_mask |= C2D_SOURCE_RECT_BIT;
      draw_obj[0].source_rect.x = TOQ16(x);
      draw_obj[0].source_rect.y = TOQ16(y);
      draw_obj[0].source_rect.width = TOQ16(w);
      draw_obj[0].source_rect.height = TOQ16(h);
    }
  }

  draw_obj[0].target_rect.width  = dst_buffer.info.planes[0].width << 16;
  draw_obj[0].target_rect.height = dst_buffer.info.planes[0].height << 16;
  draw_obj[0].target_rect.x      = 0;
  draw_obj[0].target_rect.y      = 0;

  //STEP7: Draw C2dObject on target surface.
  ret = c2dDraw(dst_surface_id, 0, 0, 0, 0, draw_obj, 1);
  if(ret != C2D_STATUS_OK) {
    QMMF_ERROR("%s: c2dDraw failed! %d", __func__, ret);
    status = RESIZER_STATUS_ERROR;
    goto EXIT;
  }

  ret = c2dFinish(dst_surface_id);
  if(ret != C2D_STATUS_OK) {
    QMMF_ERROR("%s: c2dFinish failed! %d", __func__, ret);
    status = RESIZER_STATUS_ERROR;
    goto EXIT;
  }

EXIT:
  QMMF_DEBUG("%s: Exit", __func__);
  return status;
}

RESIZER_STATUS C2DResizer::ValidateOutput(const uint32_t width,
                                          const uint32_t height,
                                          const BufferFormat format) {
  if (format != BufferFormat::kNV12 &&
      format != BufferFormat::kNV12UBWC &&
      format != BufferFormat::kNV21 &&
      format != BufferFormat::kNV16 &&
      format != BufferFormat::kRGB) {
    QMMF_ERROR("%s: Unsupported format: %d", __func__, (int32_t) format);
    return RESIZER_STATUS_ERROR;
  }
  return RESIZER_STATUS_OK;
}

void* C2DResizer::MapBuf(StreamBuffer& buffer) {
  if (buffer.fd == -1 || buffer.data == nullptr || buffer.size == 0) {
    QMMF_ERROR("%s: Error Invalid Stream buffer", __func__);
    return nullptr;
  }
  void* gpu_addr = nullptr;
  int32_t data_offset  = 0;
  if (mapped_buffs_.count(buffer.fd) == 0) {
    auto ret = c2dMapAddr(buffer.fd, buffer.data, buffer.size,
                          data_offset, KGSL_USER_MEM_TYPE_ION, &gpu_addr);
    if(ret != C2D_STATUS_OK) {
     QMMF_ERROR("%s: c2dMapAddr failed!", __func__);
     return nullptr;
    }
    if(gpu_addr == nullptr) {
     QMMF_ERROR("%s: Invalid gpu_addr!", __func__);
     return nullptr;
    }
    mapped_buffs_[buffer.fd] = gpu_addr;
  } else {
    gpu_addr = mapped_buffs_[buffer.fd];
  }
  return gpu_addr;
}

void C2DResizer::UnMapBufs() {
  for (auto iter : mapped_buffs_) {
    if (iter.second != nullptr) {
      QMMF_INFO("%s: Unmap addr(%p)", __func__, iter.second);
      auto ret = c2dUnMapAddr(iter.second);
      if(ret != C2D_STATUS_OK) {
       QMMF_ERROR("%s: c2dUnMapAddr failed!", __func__);
      }
    }
  }
  mapped_buffs_.clear();
}

bool C2DResizer::BufferFormatToC2D(StreamBuffer &buffer,
                                   uint32_t &c2d_color_format) {
  switch (buffer.info.format) {
    case BufferFormat::kNV21:
      c2d_color_format = C2D_COLOR_FORMAT_420_NV21;
      break;
    case BufferFormat::kNV12:
      c2d_color_format = C2D_COLOR_FORMAT_420_NV12;
      break;
    case BufferFormat::kNV12UBWC:
      c2d_color_format =
          C2D_COLOR_FORMAT_420_NV12 | C2D_FORMAT_UBWC_COMPRESSED;
      break;
    case BufferFormat::kNV16:
      c2d_color_format = C2D_COLOR_FORMAT_422_IUYV;
      break;
    case BufferFormat::kRGB:
      c2d_color_format = C2D_COLOR_FORMAT_888_RGB;
      break;
    default:
      QMMF_ERROR("%s: Unsupported format:%d", __func__,
        (int32_t) buffer.info.format);
      return false;
  }
  return true;
}

RESIZER_STATUS C2DResizer::UpdateRGBSurface(StreamBuffer& dst_buffer,
                                            uint32_t &c2d_color_format,
                                            void *dst_buf_gpu_addr) {
  //Bytes per pixel
  uint8_t bpp = 3;
  C2D_RGB_SURFACE_DEF dst_surface;
  //destination format.
  dst_surface.format  = c2d_color_format;
  //destination width.
  dst_surface.width   = dst_buffer.info.planes[0].width;
  //destination height.
  dst_surface.height  = dst_buffer.info.planes[0].height;
  dst_surface.stride = dst_buffer.info.planes[0].stride * bpp;
  dst_surface.buffer  = dst_buffer.data;
  dst_surface.phys  = dst_buf_gpu_addr;

  QMMF_DEBUG("%s: dst_surface.format = %d ", __func__, dst_surface.format);
  QMMF_DEBUG("%s: dst_surface.width = %d ", __func__, dst_surface.width);
  QMMF_DEBUG("%s: dst_surface.height = %d ", __func__, dst_surface.height);
  QMMF_DEBUG("%s: dst_surface.stride0 = %d ", __func__, dst_surface.stride);

  C2D_SURFACE_TYPE type = static_cast<C2D_SURFACE_TYPE>
      (C2D_SURFACE_RGB_HOST | C2D_SURFACE_WITH_PHYS);
  auto ret = c2dUpdateSurface(dst_surface_rgb_id_, C2D_SOURCE,
                              type, &dst_surface);
  if(ret != C2D_STATUS_OK) {
    QMMF_ERROR("%s: c2dUpdateSurface failed! %d", __func__, ret);
    return RESIZER_STATUS_ERROR;
  }
  return RESIZER_STATUS_OK;
}

RESIZER_STATUS C2DResizer::UpdateYUVSurface(StreamBuffer& src_buffer,
                                            StreamBuffer& dst_buffer,
                                            uint32_t &c2d_color_format,
                                            void *dst_buf_gpu_addr) {
  C2D_YUV_SURFACE_DEF dst_surface;
  //destination format.
  dst_surface.format  = c2d_color_format;
  //destination width.
  dst_surface.width   = dst_buffer.info.planes[0].width;
  //destination height.
  dst_surface.height  = dst_buffer.info.planes[0].height;
  //Y plane stride.
  dst_surface.stride0 = dst_buffer.info.planes[0].stride;
  //Y plane hostptr.
  dst_surface.plane0  = dst_buffer.data;
  //Y plane Gpu address.
  dst_surface.phys0   = dst_buf_gpu_addr;
  //UV plane stride.
  dst_surface.stride1 = dst_buffer.info.planes[0].stride;
  //UV plane hostptr.
  int32_t plane_y_len =
      dst_surface.stride0 * dst_buffer.info.planes[0].scanline;

  if (dst_surface.format & C2D_FORMAT_UBWC_COMPRESSED) {
    plane_y_len =
        MSM_MEDIA_ALIGN(VENUS_Y_META_STRIDE(COLOR_FMT_NV12_UBWC,
                                            dst_surface.width) *
                        VENUS_Y_META_SCANLINES(COLOR_FMT_NV12_UBWC,
                                               dst_surface.height), 4096) +
        MSM_MEDIA_ALIGN(VENUS_Y_STRIDE(COLOR_FMT_NV12_UBWC,
                                       dst_surface.width) *
                        VENUS_Y_SCANLINES(COLOR_FMT_NV12_UBWC,
                                          dst_surface.height), 4096);
  }

  //UV plane hostptr.
  dst_surface.plane1  = (void*)((intptr_t)dst_buffer.data + plane_y_len);
  //UV plane Gpu address.
  dst_surface.phys1 = (void*)((intptr_t)dst_buf_gpu_addr + plane_y_len);

  QMMF_DEBUG("%s: dst_surface.width = %d ", __func__, dst_surface.width);
  QMMF_DEBUG("%s: dst_surface.height = %d ", __func__, dst_surface.height);
  QMMF_DEBUG("%s: dst_surface.stride0 = %d ", __func__, dst_surface.stride0);
  QMMF_DEBUG("%s: dst_surface.stride1 = %d ", __func__, dst_surface.stride1);
  QMMF_DEBUG("%s: plane_y_len = %d", __func__, plane_y_len);

  C2D_SURFACE_TYPE type = static_cast<C2D_SURFACE_TYPE>
      (C2D_SURFACE_YUV_HOST | C2D_SURFACE_WITH_PHYS);
  auto ret = c2dUpdateSurface(dst_surface_id_, C2D_SOURCE, type, &dst_surface);
  if(ret != C2D_STATUS_OK) {
    QMMF_ERROR("%s: c2dUpdateSurface failed! %d", __func__, ret);
    return RESIZER_STATUS_ERROR;
  }
  return RESIZER_STATUS_OK;
}

} //namespace qmmf ends here
