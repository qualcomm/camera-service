/*
* Copyright (c) 2018, The Linux Foundation. All rights reserved.
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

#define LOG_TAG "res_neon"

#include <iostream>
#include <cstring>
#include <numeric>

#include "common/utils/qmmf_log.h"

#include "res_neon.h"
#include "res_neon_prv.h"

namespace qmmf {

namespace neonresizer {

const char NeonCore::version_[] = "1.0.0.0";

const std::map<ResMethod, pfunc> NeonThrdArgs::func_y_ = {

      {ResMethod::kRES_BILINEAR_V_SKIP, LumaProcessBilinearVSkipNEON},
      {ResMethod::kRES_BILINEAR,        LumaProcessBilinearNEON},
};

const std::map<ResMethod, pfunc> NeonThrdArgs::func_uv_ = {

      {ResMethod::kRES_BILINEAR_V_SKIP, ChromaProcessBilinearVSkipNEON},
      {ResMethod::kRES_BILINEAR,        ChromaProcessBilinearNEON},
};

NeonThrdArgs::NeonThrdArgs()
 : thread_(nullptr), img_(),
   thread_started_(false), thread_ready_(false), thread_active_(true) {

  QMMF_DEBUG("%s: Enter", __func__);
  QMMF_DEBUG("%s: Exit", __func__);
}

NeonThrdArgs::~NeonThrdArgs() {
  QMMF_DEBUG("%s: Enter", __func__);
  QMMF_DEBUG("%s: Exit", __func__);
}

ResnStatus NeonThrdArgs::CreateYPass() {
  thread_ = new std::thread(NeonThreadYPass, this);
  if (thread_ == nullptr) {
    return ResnStatus::kRESN_FAILED_TO_CREATE_WORK_THREADS;
  }
  return ResnStatus::kRESN_SUCCESS;
}

ResnStatus NeonThrdArgs::CreateUVPass() {
  thread_ = new std::thread(NeonThreadUVPass, this);
  if (thread_ == nullptr) {
    return ResnStatus::kRESN_FAILED_TO_CREATE_WORK_THREADS;
  }
  return ResnStatus::kRESN_SUCCESS;
}

void NeonThrdArgs::DeleteYUVPass() {
  {
    std::unique_lock<std::mutex> lk(lock_);
    thread_active_ = false;
    signal_thread_.Signal();
  }

  thread_->join();
  delete(thread_);
  thread_ = nullptr;
  InitThreadFlags();
}

void NeonThrdArgs::setImg(NeonImgArgs& img_in) {
  std::unique_lock<std::mutex> lk(lock_);
  img_ = img_in;
}

NeonImgArgs& NeonThrdArgs::getImg() {
  std::unique_lock<std::mutex> lk(lock_);
  return img_;
}

void NeonThrdArgs::Start() {
  std::unique_lock<std::mutex> lk(lock_);
  thread_started_ = true;
  signal_thread_.Signal();
}

void NeonThrdArgs::WaitReady() {
  std::chrono::nanoseconds wait_time(kFrameTimeout);
  std::unique_lock<std::mutex> lk(lock_);
  while (!thread_ready_) {
    auto ret = signal_base_.WaitFor(lk, wait_time);
    if (ret != 0) {
      QMMF_DEBUG("%s: Wait for ready timed out", __func__);
      return;
    }
  }
  thread_ready_ = false;
}

void NeonThrdArgs::Done() {
  std::unique_lock<std::mutex> lk(lock_);
  thread_ready_ = true;
  signal_base_.Signal();
}

void* NeonThrdArgs::NeonThreadYPass(void* arg) {
  NeonThrdArgs* n_thrd_arg = reinterpret_cast<NeonThrdArgs*>(arg);
  std::shared_ptr<NeonImgArgs> thrd_arg;
  bool thread_active = true;

  if (NULL == n_thrd_arg) {
    thread_active = false;
    return NULL;
  }

  std::chrono::nanoseconds wait_time(kFrameTimeout);
  while (thread_active) {
    {
      std::unique_lock<std::mutex> lk(n_thrd_arg->lock_);
      while (!n_thrd_arg->thread_started_ && n_thrd_arg->thread_active_) {
        auto ret = n_thrd_arg->signal_thread_.WaitFor(lk, wait_time);
        if (ret != 0) {
          QMMF_INFO("%s: Wait for YPass timed out", __func__);
          return NULL;
        }
      }
      n_thrd_arg->thread_started_ = false;
      thrd_arg = std::make_shared<NeonImgArgs>(n_thrd_arg->img_);
      if (!n_thrd_arg->thread_active_) {
        thread_active = false;
        break;
      }
    }

    func_y_.find(n_thrd_arg->getMethod())->second(thrd_arg);

    n_thrd_arg->Done();
  }

  return NULL;
}

void* NeonThrdArgs::NeonThreadUVPass(void* arg) {
  NeonThrdArgs* n_thrd_arg = reinterpret_cast<NeonThrdArgs*>(arg);
  std::shared_ptr<NeonImgArgs> thrd_arg;
  bool thread_active = true;

  if (NULL == n_thrd_arg) {
    thread_active = false;
    return NULL;
  }

  std::chrono::nanoseconds wait_time(kFrameTimeout);
  while (thread_active) {
    {
      std::unique_lock<std::mutex> lk(n_thrd_arg->lock_);
      while (!n_thrd_arg->thread_started_ && n_thrd_arg->thread_active_) {
        auto ret = n_thrd_arg->signal_thread_.WaitFor(lk, wait_time);
        if (ret != 0) {
          QMMF_INFO("%s: Wait for YPass timed out", __func__);
          return NULL;
        }
      }
      n_thrd_arg->thread_started_ = false;
      thrd_arg = std::make_shared<NeonImgArgs>(n_thrd_arg->img_);
      if (!n_thrd_arg->thread_active_) {
        thread_active = false;
        break;
      }
    }

    func_uv_.find(n_thrd_arg->getMethod())->second(thrd_arg);

    n_thrd_arg->Done();
  }

  return NULL;
}

/** LumaProcessBilinearVSkikNEON
 *
 * @n_thrd_arg: struct with input thread parameters
 *
 * Resize Luma Process function NEON
 *
 * return:
 **/
void NeonThrdArgs::LumaProcessBilinearVSkipNEON(
    std::shared_ptr<NeonImgArgs> n_thrd_arg) {

  uint32_t row, col;
  uint8_t *src, *src0, *src1, *src2, *src3, *src4, *src5, *src6, *src7;
  uint16_t* pcoef;
  uint32_t h_ind;
  uint16_t h_rem;
  uint16_t one_val = 256;

  uint8_t* src_luma = n_thrd_arg->src_luma;
  uint8_t* dst =
      n_thrd_arg->dst_luma + n_thrd_arg->line_start * n_thrd_arg->stride;
  uint16_t* offsets = n_thrd_arg->input_offsets;
  uint16_t* coefs = n_thrd_arg->y_coefs;
  uint32_t h_coef = n_thrd_arg->h_coef;
  uint32_t line_start = n_thrd_arg->line_start;
  uint32_t line_end = n_thrd_arg->line_end;
  uint32_t width = n_thrd_arg->width;
  uint32_t stride = n_thrd_arg->stride;
  uint32_t src_stride = n_thrd_arg->src_stride;

  for (row = line_start; row < line_end; row++) {
    h_ind = row * h_coef;
    h_rem = h_ind - ((h_ind >> 8) << 8);
    h_ind = ((h_ind - h_rem) >> 8);
    src = src_luma + h_ind * src_stride;
    for (col = 0; col < (width / 8); col++) {
      src0 = src + offsets[col * 8];
      src1 = src + offsets[col * 8 + 1];
      src2 = src + offsets[col * 8 + 2];
      src3 = src + offsets[col * 8 + 3];
      src4 = src + offsets[col * 8 + 4];
      src5 = src + offsets[col * 8 + 5];
      src6 = src + offsets[col * 8 + 6];
      src7 = src + offsets[col * 8 + 7];
      pcoef = coefs + col * 8;

      asm volatile(
          "vld1.8     {d0}, [%0]                     \n"  // load up 4
          "vld1.8     {d1}, [%1]                     \n"
          "vld1.8     {d2}, [%2]                     \n"
          "vld1.8     {d3}, [%3]                     \n"
          "vld1.8     {d4}, [%4]                     \n"
          "vld1.8     {d5}, [%5]                     \n"
          "vld1.8     {d6}, [%6]                     \n"
          "vld1.8     {d7}, [%7]                     \n"
          "vld1.16    {q6}, [%9]                     \n"
          "vext.8 d8, d0, d0, #2                     \n"
          "vext.8 d8, d8, d1, #2                     \n"
          "vext.8 d8, d8, d2, #2                     \n"
          "vext.8 d8, d8, d3, #2                     \n"
          "vext.8 d9, d4, d4, #2                     \n"
          "vext.8 d9, d9, d5, #2                     \n"
          "vext.8 d9, d9, d6, #2                     \n"
          "vext.8 d9, d9, d7, #2                     \n"
          "vuzp.u8 q4, q5                            \n"
          "vmovl.u8 q0, d8                           \n"
          "vmovl.u8 q1, d10                          \n"
          "vdup.16 q2, %10                           \n"
          "vsub.i16 q3, q2, q6                       \n"
          "vmull.u32 q4, d0, d6                      \n"
          "vmlal.u32 q4, d2, d12                     \n"
          "vmull.u32 q5, d1, d7                      \n"
          "vmlal.u32 q5, d3, d13                     \n"
          "vrshrn.u16 d0, q4 , #8                    \n"
          "vrshrn.u16 d1, q5 , #8                    \n"
          "vmovn.u16 d10, q0                         \n"
          "vst1.u8    {d10}, [%8]                    \n"
          : "+r"(src0),    // %0
            "+r"(src1),    // %1
            "+r"(src2),    // %2
            "+r"(src3),    // %3
            "+r"(src4),    // %4
            "+r"(src5),    // %5
            "+r"(src6),    // %6
            "+r"(src7),    // %7
            "+r"(dst),     // %8
            "+r"(pcoef),   // %9
            "+r"(one_val)  // %10
          :
          : "q0", "q1", "q2", "q3", "q4", "q5", "q6", "memory", "cc");
      dst += 8;
    }
    dst += (stride - width);
  }
}

/** LumaProcessBilinearNEON
 *
 * @n_thrd_arg: struct with input thread parameters
 *
 * Resize Luma Process function NEON
 *
 * return:
 **/
void NeonThrdArgs::LumaProcessBilinearNEON(
    std::shared_ptr<NeonImgArgs> n_thrd_arg) {

  uint32_t row, col;
  uint8_t *src, *src0, *src1, *src2, *src3, *src4, *src5, *src6, *src7;
  uint16_t* pcoef;
  uint16_t h_rem;
  uint16_t one_val = 256;

  uint8_t* src_luma = n_thrd_arg->src_luma;
  uint8_t* dst =
      n_thrd_arg->dst_luma + n_thrd_arg->line_start * n_thrd_arg->stride;
  uint16_t* offsets = n_thrd_arg->input_offsets;
  uint16_t* ver_offsets = n_thrd_arg->ver_offsets;
  uint16_t* ver_coefs = n_thrd_arg->ver_coefs;
  uint16_t* coefs = n_thrd_arg->y_coefs;
  uint32_t line_start = n_thrd_arg->line_start;
  uint32_t line_end = n_thrd_arg->line_end;
  uint32_t width = n_thrd_arg->width;
  uint32_t stride = n_thrd_arg->stride;
  uint32_t src_stride = n_thrd_arg->src_stride;

  for (row = line_start; row < line_end; row++) {
    h_rem = ver_coefs[row];
    src = src_luma + ver_offsets[row] * src_stride;
    for (col = 0; col < (width / 8); col++) {
      src0 = src + offsets[col * 8];
      src1 = src + offsets[col * 8 + 1];
      src2 = src + offsets[col * 8 + 2];
      src3 = src + offsets[col * 8 + 3];
      src4 = src + offsets[col * 8 + 4];
      src5 = src + offsets[col * 8 + 5];
      src6 = src + offsets[col * 8 + 6];
      src7 = src + offsets[col * 8 + 7];
      pcoef = coefs + col * 8;

      asm volatile(
          "vld1.8     {d0}, [%0]                     \n"  // load up 4
          "vld1.8     {d1}, [%1]                     \n"
          "vld1.8     {d2}, [%2]                     \n"
          "vld1.8     {d3}, [%3]                     \n"
          "vld1.8     {d4}, [%4]                     \n"
          "vld1.8     {d5}, [%5]                     \n"
          "vld1.8     {d6}, [%6]                     \n"
          "vld1.8     {d7}, [%7]                     \n"
          "vext.8 d8, d0, d0, #2                     \n"
          "vext.8 d8, d8, d1, #2                     \n"
          "vext.8 d8, d8, d2, #2                     \n"
          "vext.8 d8, d8, d3, #2                     \n"
          "vext.8 d9, d4, d4, #2                     \n"
          "vext.8 d9, d9, d5, #2                     \n"
          "vext.8 d9, d9, d6, #2                     \n"
          "vext.8 d9, d9, d7, #2                     \n"
          "vuzp.u8 q4, q5                            \n"
          "vmovl.u8 q0, d8                           \n"
          "vmovl.u8 q1, d10                          \n"
          : "+r"(src0),    // %0
            "+r"(src1),    // %1
            "+r"(src2),    // %2
            "+r"(src3),    // %3
            "+r"(src4),    // %4
            "+r"(src5),    // %5
            "+r"(src6),    // %6
            "+r"(src7),    // %7
            "+r"(dst),     // %8
            "+r"(pcoef),   // %9
            "+r"(h_rem),   // %10
            "+r"(one_val)  // %11
          :
          : "q0", "q1", "q2", "q3", "q4", "q5", "q6", "q8", "q9", "memory",
            "cc");

      src0 += src_stride;
      src1 += src_stride;
      src2 += src_stride;
      src3 += src_stride;
      src4 += src_stride;
      src5 += src_stride;
      src6 += src_stride;
      src7 += src_stride;

      asm volatile(
          "vld1.8     {d4}, [%0]                     \n"  // load up 4
          "vld1.8     {d5}, [%1]                     \n"
          "vld1.8     {d6}, [%2]                     \n"
          "vld1.8     {d7}, [%3]                     \n"
          "vld1.8     {d8}, [%4]                     \n"
          "vld1.8     {d9}, [%5]                     \n"
          "vld1.8     {d10}, [%6]                    \n"
          "vld1.8     {d11}, [%7]                    \n"
          "vld1.16    {q6}, [%9]                     \n"
          "vext.8 d16, d4, d4, #2                    \n"
          "vext.8 d16, d16, d5, #2                   \n"
          "vext.8 d16, d16, d6, #2                   \n"
          "vext.8 d16, d16, d7, #2                   \n"
          "vext.8 d17, d8, d8, #2                    \n"
          "vext.8 d17, d17, d9, #2                   \n"
          "vext.8 d17, d17, d10, #2                  \n"
          "vext.8 d17, d17, d11, #2                  \n"
          "vuzp.u8 q8, q9                            \n"
          "vmovl.u8 q2, d16                          \n"
          "vmovl.u8 q3, d18                          \n"
          "vdup.16 q4, %11                           \n"
          "vsub.i16 q5, q4, q6                       \n"
          "vmull.u32 q7, d0, d10                     \n"
          "vmlal.u32 q7, d2, d12                     \n"
          "vmull.u32 q8, d1, d11                     \n"
          "vmlal.u32 q8, d3, d13                     \n"
          "vrshrn.u16 d0, q7 , #8                    \n"
          "vrshrn.u16 d1, q8 , #8                    \n"
          "vmull.u32 q7, d4, d10                     \n"
          "vmlal.u32 q7, d6, d12                     \n"
          "vmull.u32 q8, d5, d11                     \n"
          "vmlal.u32 q8, d7, d13                     \n"
          "vrshrn.u16 d2, q7 , #8                    \n"
          "vrshrn.u16 d3, q8 , #8                    \n"
          "vdup.16 q3, %10                           \n"
          "vsub.i16 q5, q4, q3                       \n"
          "vmovn.u16 d18, q0                         \n"
          "vmovl.u8 q0, d18                          \n"
          "vmovn.u16 d19, q1                         \n"
          "vmovl.u8 q1, d19                          \n"
          "vmull.u32 q7, d0, d10                     \n"
          "vmlal.u32 q7, d2, d6                      \n"
          "vmull.u32 q8, d1, d11                     \n"
          "vmlal.u32 q8, d3, d7                      \n"
          "vrshrn.u16 d0, q7 , #8                    \n"
          "vrshrn.u16 d1, q8 , #8                    \n"
          "vmovn.u16 d10, q0                         \n"
          "vst1.u8    {d10}, [%8]                    \n"
          : "+r"(src0),    // %0
            "+r"(src1),    // %1
            "+r"(src2),    // %2
            "+r"(src3),    // %3
            "+r"(src4),    // %4
            "+r"(src5),    // %5
            "+r"(src6),    // %6
            "+r"(src7),    // %7
            "+r"(dst),     // %8
            "+r"(pcoef),   // %9
            "+r"(h_rem),   // %10
            "+r"(one_val)  // %11
          :
          : "q0", "q1", "q2", "q3", "q4", "q5", "q6", "q8", "q9", "memory",
            "cc");
      dst += 8;
    }
    dst += (stride - width);
  }
}

/** ChromaProcessBilinearVSkipNEON
 *
 * @n_thrd_arg: struct with input thread parameters
 *
 * Resize Luma Process function
 *
 * return:
 **/
void NeonThrdArgs::ChromaProcessBilinearVSkipNEON(
    std::shared_ptr<NeonImgArgs> n_thrd_arg) {

  uint32_t row, col;
  uint8_t *src, *src0, *src1, *src2, *src3;
  uint16_t* pcoef;
  uint32_t h_ind;
  uint16_t h_rem;
  uint16_t one_val = 256;

  uint8_t* src_chroma = n_thrd_arg->src_chroma;
  uint8_t* dst =
      n_thrd_arg->dst_chroma + n_thrd_arg->line_start * n_thrd_arg->stride / 2;
  uint16_t* offsets = n_thrd_arg->input_offsets;
  uint16_t* coefs = n_thrd_arg->uv_coefs;
  uint32_t h_coef = n_thrd_arg->h_coef;
  uint32_t line_start = n_thrd_arg->line_start;
  uint32_t line_end = n_thrd_arg->line_end;
  uint32_t width = n_thrd_arg->width;
  uint32_t stride = n_thrd_arg->stride;
  uint32_t src_stride = n_thrd_arg->src_stride;

  for (row = (line_start / 2); row < (line_end / 2); row++) {
    h_ind = row * h_coef;
    h_rem = h_ind - ((h_ind >> 8) << 8);
    h_ind = ((h_ind - h_rem) >> 8);
    src = src_chroma + h_ind * src_stride;
    for (col = 0; col < (width / 8); col++) {
      src0 = src + offsets[col * 4] * 2;
      src1 = src + offsets[col * 4 + 1] * 2;
      src2 = src + offsets[col * 4 + 2] * 2;
      src3 = src + offsets[col * 4 + 3] * 2;
      pcoef = coefs + col * 8;

      asm volatile(
          "vld1.8     {d0}, [%0]                     \n"  // load up 4
          "vld1.8     {d1}, [%1]                     \n"
          "vld1.8     {d2}, [%2]                     \n"
          "vld1.8     {d3}, [%3]                     \n"
          "vld1.16    {q6}, [%5]                     \n"
          "vext.8 d4, d0, d0, #4                     \n"
          "vext.8 d4, d4, d1, #4                     \n"
          "vext.8 d5, d2, d2, #4                     \n"
          "vext.8 d5, d5, d3, #4                     \n"
          "vuzp.u16 d4, d5                           \n"
          "vmovl.u8 q0, d4                           \n"
          "vmovl.u8 q1, d5                           \n"
          "vdup.16 q4, %6                            \n"
          "vsub.i16 q5, q4, q6                       \n"
          "vmull.u32 q2, d0, d10                     \n"
          "vmlal.u32 q2, d2, d12                     \n"
          "vmull.u32 q3, d1, d11                     \n"
          "vmlal.u32 q3, d3, d13                     \n"
          "vrshrn.u16 d0, q2 , #8                    \n"
          "vrshrn.u16 d1, q3 , #8                    \n"
          "vmovn.u16 d10, q0                         \n"
          "vst1.u8    {d10}, [%4]                    \n"
          : "+r"(src0),    // %0
            "+r"(src1),    // %1
            "+r"(src2),    // %2
            "+r"(src3),    // %3
            "+r"(dst),     // %4
            "+r"(pcoef),   // %5
            "+r"(one_val)  // %6
          :
          : "q0", "q1", "q2", "q3", "q4", "q5", "q6", "q7", "memory", "cc");
      dst += 8;
    }
    dst += (stride - width);
  }
}

/** ChromaProcessBilinearNEON
 *
 * @n_thrd_arg: struct with input thread parameters
 *
 * Resize Luma Process function
 *
 * return:
 **/
void NeonThrdArgs::ChromaProcessBilinearNEON(
    std::shared_ptr<NeonImgArgs> n_thrd_arg) {

  uint32_t row, col;
  uint8_t *src, *src0, *src1, *src2, *src3;
  uint16_t* pcoef;
  uint16_t h_rem;
  uint16_t one_val = 256;

  uint8_t* src_chroma = n_thrd_arg->src_chroma;
  uint8_t* dst =
      n_thrd_arg->dst_chroma + n_thrd_arg->line_start * n_thrd_arg->stride / 2;
  uint16_t* offsets = n_thrd_arg->input_offsets;
  uint16_t* ver_offsets = n_thrd_arg->ver_offsets;
  uint16_t* ver_coefs = n_thrd_arg->ver_coefs;
  uint16_t* coefs = n_thrd_arg->uv_coefs;
  uint32_t line_start = n_thrd_arg->line_start;
  uint32_t line_end = n_thrd_arg->line_end;
  uint32_t width = n_thrd_arg->width;
  uint32_t stride = n_thrd_arg->stride;
  uint32_t src_stride = n_thrd_arg->src_stride;

  for (row = (line_start / 2); row < (line_end / 2); row++) {
    h_rem = ver_coefs[row];
    src = src_chroma + ver_offsets[row] * src_stride;
    for (col = 0; col < (width / 8); col++) {
      src0 = src + offsets[col * 4] * 2;
      src1 = src + offsets[col * 4 + 1] * 2;
      src2 = src + offsets[col * 4 + 2] * 2;
      src3 = src + offsets[col * 4 + 3] * 2;
      pcoef = coefs + col * 8;

      asm volatile(
          "vld1.8     {d0}, [%0]                     \n"  // load up 4
          "vld1.8     {d1}, [%1]                     \n"
          "vld1.8     {d2}, [%2]                     \n"
          "vld1.8     {d3}, [%3]                     \n"
          "vext.8 d4, d0, d0, #4                     \n"
          "vext.8 d4, d4, d1, #4                     \n"
          "vext.8 d5, d2, d2, #4                     \n"
          "vext.8 d5, d5, d3, #4                     \n"
          "vuzp.u16 d4, d5                           \n"
          "vmovl.u8 q0, d4                           \n"
          "vmovl.u8 q1, d5                           \n"
          : "+r"(src0),    // %0
            "+r"(src1),    // %1
            "+r"(src2),    // %2
            "+r"(src3),    // %3
            "+r"(dst),     // %4
            "+r"(pcoef),   // %5
            "+r"(h_rem),   // %6
            "+r"(one_val)  // %7
          :
          : "q0", "q1", "q2", "q3", "q4", "q5", "q6", "q7", "q8", "q9",
            "memory", "cc");

      src0 += src_stride;
      src1 += src_stride;
      src2 += src_stride;
      src3 += src_stride;

      asm volatile(
          "vld1.8     {d4}, [%0]                     \n"  // load up 4
          "vld1.8     {d5}, [%1]                     \n"
          "vld1.8     {d6}, [%2]                     \n"
          "vld1.8     {d7}, [%3]                     \n"
          "vld1.16    {q6}, [%5]                     \n"
          "vext.8 d16, d4, d4, #4                    \n"
          "vext.8 d16, d16, d5, #4                   \n"
          "vext.8 d17, d6, d6, #4                    \n"
          "vext.8 d17, d17, d7, #4                   \n"
          "vuzp.u16 d16, d17                         \n"
          "vmovl.u8 q2, d16                          \n"
          "vmovl.u8 q3, d17                          \n"
          "vdup.16 q4, %7                            \n"
          "vsub.i16 q5, q4, q6                       \n"
          "vmull.u32 q7, d0, d10                     \n"
          "vmlal.u32 q7, d2, d12                     \n"
          "vmull.u32 q8, d1, d11                     \n"
          "vmlal.u32 q8, d3, d13                     \n"
          "vrshrn.u16 d0, q7 , #8                    \n"
          "vrshrn.u16 d1, q8 , #8                    \n"
          "vmull.u32 q7, d4, d10                     \n"
          "vmlal.u32 q7, d6, d12                     \n"
          "vmull.u32 q8, d5, d11                     \n"
          "vmlal.u32 q8, d7, d13                     \n"
          "vrshrn.u16 d2, q7 , #8                    \n"
          "vrshrn.u16 d3, q8 , #8                    \n"
          "vdup.16 q6, %6                           \n"
          "vsub.i16 q5, q4, q6                       \n"
          "vmovn.u16 d18, q0                         \n"
          "vmovl.u8 q0, d18                          \n"
          "vmovn.u16 d19, q1                         \n"
          "vmovl.u8 q1, d19                          \n"
          "vmull.u32 q7, d0, d10                     \n"
          "vmlal.u32 q7, d2, d12                      \n"
          "vmull.u32 q8, d1, d11                     \n"
          "vmlal.u32 q8, d3, d13                      \n"
          "vrshrn.u16 d0, q7 , #8                    \n"
          "vrshrn.u16 d1, q8 , #8                    \n"
          "vmovn.u16 d10, q0                         \n"
          "vst1.u8    {d10}, [%4]                    \n"
          : "+r"(src0),    // %0
            "+r"(src1),    // %1
            "+r"(src2),    // %2
            "+r"(src3),    // %3
            "+r"(dst),     // %4
            "+r"(pcoef),   // %5
            "+r"(h_rem),   // %6
            "+r"(one_val)  // %7
          :
          : "q0", "q1", "q2", "q3", "q4", "q5", "q6", "q7", "q8", "q9",
            "memory", "cc");
      dst += 8;
    }
    dst += (stride - width);
  }
}

NeonCore::NeonCore() : ctx_(nullptr), num_threads_(8) {
  QMMF_DEBUG("%s: Enter", __func__);
  QMMF_DEBUG("%s: Exit", __func__);
}

NeonCore::~NeonCore() {
  QMMF_DEBUG("%s: Enter", __func__);
  QMMF_DEBUG("%s: Exit", __func__);
}

/** resn_get_version
 *
 * Returns lib version
 *
 * return: lib version
 **/
const char* NeonCore::resn_get_version() {
  return version_;
}

/** resn_init
   *
   * Main Resizer Init function
   *
   * return: ResnStatus
   **/
ResnStatus NeonCore::resn_init() {

  uint32_t i;

  ctx_ = std::make_unique<ResnCtx>(num_threads_);
  if (ctx_.get() == nullptr) {
    QMMF_DEBUG("%s: Memory error", __func__);
    assert(0);
  }

  for (i = 0; i < num_threads_ / 2; i++) {
    if (ctx_->CreateYPass(i) != ResnStatus::kRESN_SUCCESS) {
      num_threads_ = i;
      resn_deinit();
      return ResnStatus::kRESN_FAILED_TO_CREATE_WORK_THREADS;
    }
  }

  for (i = num_threads_ / 2; i < num_threads_; i++) {
    if (ctx_->CreateUVPass(i) != ResnStatus::kRESN_SUCCESS) {
      num_threads_ = i;
      resn_deinit();
      return ResnStatus::kRESN_FAILED_TO_CREATE_WORK_THREADS;
    }
  }

  return ResnStatus::kRESN_SUCCESS;
}

/** resn_deinit
   *    @handle: resize internal data pointer
   *
   * Main Resizer deinit function
   *
   * return:
   **/
void NeonCore::resn_deinit() {
  uint32_t i;

  if (ctx_.get() == nullptr) {
    return;
  }

  for (i = 0; i < num_threads_; i++) {
    ctx_->DeleteYUVPass(i);
  }
}

/** resn_process
*
* @handle: internal data structure
* @resn: resize input data structure
*
* Main Resizer function
*
* return: ResnStatus
**/
ResnStatus NeonCore::resn_process(Resn* resn) {

  ResnStatus status = ResnStatus::kRESN_SUCCESS;
  uint32_t i;

  if (ctx_.get() == nullptr) {
    QMMF_DEBUG("%s: Bad access", __func__);
    assert(0);
  }

  if (!resn) {
    return (ResnStatus::kRESN_NOT_INITIALIZED);
  }

  if (resn->src_width > resn->src_stride) {
    return ResnStatus::kRESN_WRONG_WIDTH_INBUF;
  }

  if (resn->dst_width > resn->dst_stride) {
    return ResnStatus::kRESN_WRONG_WIDTH_OUTBUF;
  }

  ctx_->update_coefs(resn->src_width,resn->src_height,
                     resn->dst_width, resn->dst_height);

  uint32_t w_coef, h_coef;

  w_coef = (resn->src_width * 256) / resn->dst_width;
  h_coef = (resn->src_height * 256) / resn->dst_height;

  for (i = 0; i < (num_threads_ / 2); i++) {
    NeonImgArgs img;
    img.res_method = resn->res_method;
    img.width = resn->dst_width;
    img.height = resn->dst_height;
    img.src_stride = resn->src_stride;
    img.stride = resn->dst_stride;
    img.src_luma = resn->src_luma;
    img.src_chroma = resn->src_chroma;
    img.dst_luma = resn->dst_luma;
    img.dst_chroma = resn->dst_chroma;
    img.y_coefs = ctx_->getYCoefs();
    img.uv_coefs =  ctx_->getUVCoefs();
    img.input_offsets = ctx_->getInputOffsets();
    img.ver_offsets = ctx_->getVerOffsets();
    img.ver_coefs = ctx_->getVerCoefs();
    img.w_coef = w_coef;
    img.h_coef = h_coef;
    img.line_start = i * ((resn->dst_height * 2 / num_threads_) & (~1u));
    img.line_end =
        img.line_start + ((resn->dst_height * 2 / num_threads_) & (~1u));

    ctx_->setImg(i, img);
  }

  ctx_->getImg(i - 1).line_end = resn->dst_height;

  for (; i < num_threads_; i++) {
    NeonImgArgs img;
    img.res_method = resn->res_method;
    img.width = resn->dst_width;
    img.height = resn->dst_height;
    img.src_stride = resn->src_stride;
    img.stride = resn->dst_stride;
    img.src_luma = resn->src_luma;
    img.src_chroma = resn->src_chroma;
    img.dst_luma = resn->dst_luma;
    img.dst_chroma = resn->dst_chroma;
    img.y_coefs = ctx_->getYCoefs();
    img.uv_coefs = ctx_->getUVCoefs();
    img.input_offsets = ctx_->getInputOffsets();
    img.ver_offsets = ctx_->getVerOffsets();
    img.ver_coefs = ctx_->getVerCoefs();

    img.w_coef = w_coef;
    img.h_coef = h_coef;
    img.line_start = (i - num_threads_ / 2) *
                     ((resn->dst_height * 2 / num_threads_) & (~1u));
    img.line_end =
        img.line_start + ((resn->dst_height * 2 / num_threads_) & (~1u));

    ctx_->setImg(i, img);
  }

  ctx_->getImg(i - 1).line_end = resn->dst_height;

  for (i = 0; i < num_threads_ / 2; i++) {
    ctx_->Start(i);
  }

  for (i = 0; i < num_threads_ / 2; i++) {
    ctx_->WaitReady(i);
  }

  for (i = num_threads_ / 2; i < num_threads_; i++) {
    ctx_->Start(i);
  }

  for (i = num_threads_ / 2; i < num_threads_; i++) {
    ctx_->WaitReady(i);
  }

  return status;
}

}; // namespace neonresizer

}; // namespace qmmf
