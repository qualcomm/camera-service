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

#pragma once

#include <cstdint>
#include <condition_variable>
#include <thread>
#include <mutex>
#include <vector>
#include <map>

#include "common/utils/qmmf_condition.h"

#include "res_neon.h"

namespace qmmf {

namespace neonresizer {

typedef void (*pfunc)(std::shared_ptr<NeonImgArgs>);

class NeonThrdArgs {

 public:

  NeonThrdArgs();
  ~NeonThrdArgs();

  ResnStatus CreateYPass();
  ResnStatus CreateUVPass();
  void DeleteYUVPass();
  void setImg(NeonImgArgs& img_in);
  NeonImgArgs& getImg();
  void Start();
  void WaitReady();
  void Done();
  ResMethod getMethod() { return img_.res_method; }

 private:

  static void LumaProcessBilinearVSkipNEON(std::shared_ptr<NeonImgArgs> arg);
  static void LumaProcessBilinearNEON(std::shared_ptr<NeonImgArgs> arg);
  static void ChromaProcessBilinearVSkipNEON(std::shared_ptr<NeonImgArgs> arg);
  static void ChromaProcessBilinearNEON(std::shared_ptr<NeonImgArgs> arg);

  void InitThreadFlags() {
    thread_started_ = false;
    thread_ready_ = false;
    thread_active_ = true;
  }

  std::thread *thread_;

  NeonImgArgs img_;

  std::mutex lock_;
  QCondition signal_thread_;
  QCondition signal_base_;
  bool thread_started_;
  bool thread_ready_;
  bool thread_active_;

  static const std::map<ResMethod, pfunc> func_y_;
  static const std::map<ResMethod, pfunc> func_uv_;
  static const int64_t kFrameTimeout  = 5000000000; // 5s.

 protected:
  static void *NeonThreadYPass(void* arg);
  static void *NeonThreadUVPass(void* arg);
};

class ResnCtx : public IResnCtx {

 public:

  ResnCtx(const uint32_t num) :
   num_threads_(num),
   src_width_(0),
   src_height_(0),
   dst_width_(0),
   dst_height_(0),
   y_coefs_(nullptr),
   uv_coefs_(nullptr),
   ver_offsets_(nullptr),
   ver_coefs_(nullptr),
   input_offsets_(nullptr),
   neon_work_args_(nullptr) {

    neon_work_args_ = std::make_unique<std::vector<NeonThrdArgs>>(num);
    CheckCtx(neon_work_args_.get() != nullptr);
  }

  ~ResnCtx() {}

  ResnStatus CreateYPass(const uint32_t id) override {
    CheckCtx(id < num_threads_);
    CheckCtx(neon_work_args_.get() != nullptr);
    return neon_work_args_->at(id).CreateYPass();
  }

  ResnStatus CreateUVPass(const uint32_t id) override {
    CheckCtx(id < num_threads_);
    CheckCtx(neon_work_args_.get() != nullptr);
    return neon_work_args_->at(id).CreateUVPass();
  }

  void DeleteYUVPass(const uint32_t id) override {
    CheckCtx(id < num_threads_);
    CheckCtx(neon_work_args_.get() != nullptr);
    neon_work_args_->at(id).DeleteYUVPass();
  }

  void setImg(const uint32_t id, NeonImgArgs& img) override {
    CheckCtx(id < num_threads_);
    CheckCtx(neon_work_args_.get() != nullptr);
    neon_work_args_->at(id).setImg(img);
  }

  NeonImgArgs& getImg(const uint32_t id) override {
    CheckCtx(id < num_threads_);
    CheckCtx(neon_work_args_.get() != nullptr);
    return neon_work_args_->at(id).getImg();
  }

  void Start(const uint32_t id) override {
    CheckCtx(id < num_threads_);
    CheckCtx(neon_work_args_.get() != nullptr);
    neon_work_args_->at(id).Start();
  }

  void WaitReady(const uint32_t id) override {
    CheckCtx(id < num_threads_);
    CheckCtx(neon_work_args_.get() != nullptr);
    neon_work_args_->at(id).WaitReady();
  }

  uint16_t* getInputOffsets() override {
    CheckCtx(input_offsets_.get() != nullptr);
    return input_offsets_->data();
  }

  uint16_t* getYCoefs() override {
    CheckCtx(y_coefs_.get() != nullptr);
    return y_coefs_->data();
  }

  uint16_t* getUVCoefs() override {
    CheckCtx(uv_coefs_.get() != nullptr);
    return uv_coefs_->data();
  }

  uint16_t* getVerOffsets() override {
    CheckCtx(ver_offsets_.get() != nullptr);
    return ver_offsets_->data();
  }

  uint16_t* getVerCoefs() override {
    CheckCtx(ver_coefs_.get() != nullptr);
    return ver_coefs_->data();
  }

  /** update_coefs
  *
  * @ctx: internal data pointer
  * @resn: resize input data structure
  * Update internal buffers
  *
  **/
  void update_coefs(unsigned int src_width,
                    unsigned int src_height,
                    unsigned int dst_width,
                    unsigned int dst_height) override {

    bool update_coefs = false;

    if ((src_width != src_width_) ||
        (src_height != src_height_) ||
        (dst_width != dst_width_) ||
        (dst_height != dst_height_)) {
      src_width_ = src_width;
      src_height_ = src_height;
      dst_width_ = dst_width;
      dst_height_ = dst_height;
      update_coefs = true;
    }

    if (update_coefs) {
      try {
        y_coefs_ = std::make_unique<std::vector<uint16_t>>(dst_width, 0);
        uv_coefs_ = std::make_unique<std::vector<uint16_t>>(dst_width, 0);
        input_offsets_ =
            std::make_unique<std::vector<uint16_t>>(dst_width, 0);
        ver_offsets_ =
            std::make_unique<std::vector<uint16_t>>(dst_height, 0);
        ver_coefs_ =
            std::make_unique<std::vector<uint16_t>>(dst_height, 0);
      } catch (const std::exception &e) {
        QMMF_ERROR("%s: Memory error exception: %s", __func__, e.what());
        throw e;
      }

      uint32_t w_coef = (src_width * 256) / dst_width;
      uint32_t h_coef = (src_height * 256) / dst_height;
      uint32_t w_ind, h_ind;
      uint16_t w_rem, h_rem, col, row;

      for (col = 0; col < dst_width; col++) {
        w_ind = col * w_coef;
        w_rem = w_ind - ((w_ind >> 8) << 8);
        w_ind = ((w_ind - w_rem) >> 8);
        y_coefs_->at(col) = w_rem;
        input_offsets_->at(col) = (uint16_t)w_ind;
      }

      for (col = 0; col < (dst_width / 2); col++) {
        w_ind = col * w_coef;
        w_rem = w_ind - ((w_ind >> 8) << 8);
        w_ind = ((w_ind - w_rem) >> 8);
        uv_coefs_->at(col * 2) = w_rem;
        uv_coefs_->at(col * 2 + 1) = w_rem;
      }

      for (row = 0; row < dst_height; row++) {
        h_ind = row * h_coef;
        h_rem = h_ind - ((h_ind >> 8) << 8);
        h_ind = ((h_ind - h_rem) >> 8);
        ver_offsets_->at(row) = h_ind;
        ver_coefs_->at(row) = h_rem;
      }
    }
  }

 private:

  /* assert if result is false */
  inline void CheckCtx(const bool res) {
    if (!res) {
      QMMF_ERROR("%s: %d Failed", __func__, __LINE__);
      assert(0);
    }
  }

  uint32_t num_threads_;

  // Input buffer dimensions
  uint32_t src_width_;
  uint32_t src_height_;

  // Output buffer dimensions
  uint32_t dst_width_;
  uint32_t dst_height_;

  // Internal buffers
  // Y interpolation coefficients size of dst_width
  std::unique_ptr<std::vector<uint16_t>> y_coefs_;
  // UV interpolation coefficients size of dst_width
  std::unique_ptr<std::vector<uint16_t>> uv_coefs_;

  // Ver offsets size of dst_height
  std::unique_ptr<std::vector<uint16_t>> ver_offsets_;
  // Ver interpolation coefficients size of dst_height
  std::unique_ptr<std::vector<uint16_t>> ver_coefs_;
  // Horizontal input buffer offsets
  std::unique_ptr<std::vector<uint16_t>> input_offsets_;

  // Threads data
  std::unique_ptr<std::vector<NeonThrdArgs>> neon_work_args_;
};

} // namespace neonresizer
} // namespace qmmf
