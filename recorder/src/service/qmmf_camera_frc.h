/*
 * Copyright (c) 2019, 2021, The Linux Foundation. All rights reserved.
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

/*
* Changes from Qualcomm Innovation Center, Inc. are provided under the following license:
* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#pragma once

#include <cstdint>
#include <string>
#include <mutex>
#include <cerrno>

#include "recorder/src/service/qmmf_recorder_utils.h"

namespace qmmf {

namespace recorder {

class FrameRateController {
 public:
  FrameRateController(const std::string& name);
  ~FrameRateController();

  status_t SetFrameRate(const float& fps);
  void EnableFrameRepeat(const bool& enable);

  status_t Start();
  status_t Stop();

  // Buffer Producer/Consumer APIs
  status_t AddConsumer(std::shared_ptr<IBufferConsumer>& consumer);
  status_t RemoveConsumer(std::shared_ptr<IBufferConsumer>& consumer);

  void OnFrameAvailable(StreamBuffer& buffer);
  void NotifyBufferReturned(StreamBuffer& buffer);

  std::shared_ptr<IBufferConsumer>& GetConsumer() { return buffer_consumer_; }

 private:
  bool SkipFrame(const StreamBuffer& buffer);
  bool RepeatFrame(const StreamBuffer& buffer);

  std::string              name_;
  bool                     active_;
  bool                     frame_repeat_enabled_;

  // Used for dynamic measurement of the FPS.
  int32_t                  input_frame_count_;
  int32_t                  output_frame_count_;
  int64_t                  measurement_interval_;

  // Used to determine whether a frame should be repeated or skipped.
  int64_t                  output_frame_interval_;
  int64_t                  expected_output_ts_;
  int64_t                  input_frame_interval_;
  int64_t                  previous_input_ts_;

  // Contains the debug flags set with the property.
  uint32_t                 debug_flags_;

  std::shared_ptr<IBufferProducer>      buffer_producer_;
  std::shared_ptr<IBufferConsumer>      buffer_consumer_;

  std::mutex               lock_;
  std::mutex               frame_lock_;
  std::mutex               consumer_lock_;
};

}; //namespace recorder

};  // namespace qmmf.
