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
* Changes from Qualcomm Innovation Center are provided under the following license:
* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#define LOG_TAG "RecorderFRC"

#include "recorder/src/service/qmmf_camera_frc.h"

#include <cstdlib>
#include <cmath>

#ifdef HAVE_ANDROID_UTILS
#include <cutils/properties.h>
#else
#include "properties.h"
#endif

#include "common/utils/qmmf_log.h"

namespace qmmf {

namespace recorder {

#define FRC_FRAME_RATE_TIMEBASE      1000000000.0f // 1 second
#define FRC_FPS_MEASUREMENT_INTERVAL 3000000000    // 3 seconds
#define FRC_THRESHOLD                0.5f          // 50% skip/repeat threshold
#define FRC_TS_DELTA                 0.01f         // 1% Delta

// Debug flags.
#define FRC_DEBUG_INPUT_FPS      (1 << 0)
#define FRC_DEBUG_OUTPUT_FPS     (1 << 1)
#define FRC_DEBUG_FRAME_SKIP     (1 << 2)
#define FRC_DEBUG_FRAME_REPEAT   (1 << 3)

FrameRateController::FrameRateController(const std::string& name)
  : name_(name),
    active_(false),
    frame_repeat_enabled_(false),
    input_frame_count_(0),
    output_frame_count_(0),
    measurement_interval_(0),
    output_frame_interval_(0),
    expected_output_ts_(0),
    input_frame_interval_(0),
    previous_input_ts_(0) {

  QMMF_INFO("%s: %s: Enter", __func__, name_.c_str());

  buffer_producer_ = std::make_shared<BufferProducerImpl<FrameRateController>>(this);

  buffer_consumer_ = std::make_shared<BufferConsumerImpl<FrameRateController>>(this);

  debug_flags_ = Property::Get("persist.qmmf.rec.frc.debug", 0);

  QMMF_INFO("%s: %s: Exit(%p)", __func__, name_.c_str(), this);
}

FrameRateController::~FrameRateController() {

  QMMF_INFO("%s: %s: Enter", __func__, name_.c_str());
  QMMF_INFO("%s: %s: Exit(%p)", __func__, name_.c_str(), this);
}

status_t FrameRateController::SetFrameRate(const float& fps) {

  std::lock_guard<std::mutex> lock(frame_lock_);
  if (fps == 0.0f) {
    QMMF_ERROR("%s: %s: Output fps is 0!", __func__, name_.c_str());
    return -EINVAL;
  }

  // Calculate the current FPS value from the output frame interval.
  float current_fps = (output_frame_interval_ == 0) ? 0.0 :
      (FRC_FRAME_RATE_TIMEBASE / output_frame_interval_);

  // Calculate new output frame interval in nanoseconds.
  output_frame_interval_ = std::llround(FRC_FRAME_RATE_TIMEBASE / fps);

  QMMF_INFO("%s: %s: FPS changed from (%.2f) to (%.2f)", __func__,
      name_.c_str(), current_fps, fps);
  return 0;
}

void FrameRateController::EnableFrameRepeat(const bool& enable) {

  std::lock_guard<std::mutex> lock(frame_lock_);
  QMMF_INFO("%s: %s: Frame repeat has been %s!", __func__, name_.c_str(),
      (enable) ? "enabled" : "disabled");
  frame_repeat_enabled_ = enable;
};

status_t FrameRateController::Start() {
  std::lock_guard<std::mutex> lock(lock_);
  {
    std::lock_guard<std::mutex> lock(frame_lock_);

    if (active_) {
      QMMF_WARN("%s: %s: Already started!", __func__, name_.c_str());
      return -EALREADY;
    }

    active_ = true;
    input_frame_count_    = 0;
    output_frame_count_   = 0;
    measurement_interval_ = 0;
    expected_output_ts_   = 0;
    previous_input_ts_    = 0;
  }
  QMMF_INFO("%s: %s: Started successfully!", __func__, name_.c_str());
  return 0;
}

status_t FrameRateController::Stop() {

  std::lock_guard<std::mutex> lock(lock_);
  {
    std::lock_guard<std::mutex> lock(frame_lock_);

    if (!active_) {
      QMMF_WARN("%s: %s: Already stopped!", __func__, name_.c_str());
      return -EALREADY;
    }

    active_ = false;
    input_frame_count_    = 0;
    output_frame_count_   = 0;
    measurement_interval_ = 0;
    expected_output_ts_   = 0;
    previous_input_ts_    = 0;
  }

  QMMF_INFO("%s: %s: Stopped successfully!", __func__, name_.c_str());
  return 0;
}

status_t FrameRateController::AddConsumer(std::shared_ptr<IBufferConsumer>& consumer) {
  {
    std::lock_guard<std::mutex> lock(lock_);
    if (active_) {
      QMMF_ERROR("%s: %s: Cannot add consumer while active!", __func__,
          name_.c_str());
      return -EBUSY;
    }
  }

  std::lock_guard<std::mutex> lock(consumer_lock_);
  if (consumer.get() == nullptr) {
    QMMF_ERROR("%s: %s: Input consumer is nullptr!", __func__, name_.c_str());
    return -EINVAL;
  }

  if (buffer_producer_->IsConnected(consumer)) {
    QMMF_WARN("%s: %s: Consumer(%p) already added to this controller!",
        __func__, name_.c_str(), consumer.get());
    return -EEXIST;
  }
  buffer_producer_->AddConsumer(consumer);
  consumer->SetProducerHandle(buffer_producer_);

  QMMF_DEBUG("%s: %s: Consumer(%p) added. Number of consumers: %d", __func__,
      name_.c_str(), consumer.get(), buffer_producer_->GetNumConsumer());
  return 0;
}

status_t FrameRateController::RemoveConsumer(std::shared_ptr<IBufferConsumer>& consumer) {
  {
    std::lock_guard<std::mutex> lock(lock_);
    if (active_) {
      QMMF_ERROR("%s: %s: Cannot remove consumer while active!", __func__,
          name_.c_str());
      return -EBUSY;
    }
  }

  std::lock_guard<std::mutex> lock(consumer_lock_);

  if (consumer.get() == nullptr) {
    QMMF_ERROR("%s: %s: Input consumer is nullptr!", __func__, name_.c_str());
    return -EINVAL;
  }

  if (!buffer_producer_->IsConnected(consumer)) {
    QMMF_WARN("%s: %s: Consumer(%p) is not connected to this controller!",
        __func__, name_.c_str(), consumer.get());
    return -ENOENT;
  }

  auto ret = buffer_producer_->CheckAndWaitPendingBuffers();
  if (ret == -ETIMEDOUT) {
    QMMF_WARN("%s: %s: Waiting for submitted frames to return, timed out!",
        __func__, name_.c_str());
    return ret;
  }

  buffer_producer_->RemoveConsumer(consumer);
  consumer->ClearProducerHandle();

  QMMF_DEBUG("%s: %s: Consumer(%p) removed. Number of consumers: %d", __func__,
      name_.c_str(), consumer.get(), buffer_producer_->GetNumConsumer());
  return 0;
}

void FrameRateController::OnFrameAvailable(StreamBuffer& buffer) {

  QMMF_DEBUG("%s: %s: Enter", __func__, name_.c_str());
  {
    std::lock_guard<std::mutex> lock(frame_lock_);
    if (!active_) {
      QMMF_DEBUG("%s: %s: Controller is in stopped state, return buffer"
          " back to source", __func__, name_.c_str());
      buffer_consumer_->GetProducerHandle()->NotifyBufferReturned(buffer);
      return;
    }

    // Initialize frame interval of the input frame at beginning of stream
    // after that calculate the interval between the last two source frames.
    input_frame_interval_ =
        (previous_input_ts_ == 0) ? 0 : buffer.timestamp - previous_input_ts_;

    // Calculate how much time has passed since last input FPS measurement.
    measurement_interval_ += input_frame_interval_;

    // Increment source frame count, used for fps measurement.
    ++input_frame_count_;

    // Save the timestamp for calculating input_frame_interval_ on next call.
    previous_input_ts_ = buffer.timestamp;

    if (expected_output_ts_ == 0) {
      // Initialize expected output timestamp variable at beginning of stream.
      expected_output_ts_ = buffer.timestamp;
    }

    // Dynamic frame skip/repeat algorithms.
    if (frame_repeat_enabled_) {
      while (RepeatFrame(buffer)) {
        if (debug_flags_ & FRC_DEBUG_FRAME_REPEAT) {
          QMMF_INFO("%s: %s: Repeat source frame: %u, camera: %u, stream: %d,"
              " ts: %lld", __func__, name_.c_str(), buffer.frame_number,
              buffer.camera_id, buffer.stream_id, buffer.timestamp);
        }
        buffer_producer_->NotifyBuffer(buffer);
        expected_output_ts_ += output_frame_interval_;

        // Increment output frame count, used for fps measurement.
        ++output_frame_count_;
      }
    } else if (SkipFrame(buffer)) {
      if (debug_flags_ & FRC_DEBUG_FRAME_SKIP) {
        QMMF_INFO("%s: %s: Skip source frame: %u, camera: %u, stream: %d,"
            " ts: %lld", __func__, name_.c_str(), buffer.frame_number,
            buffer.camera_id, buffer.stream_id, buffer.timestamp);
      }
      buffer_consumer_->GetProducerHandle()->NotifyBufferReturned(buffer);
      return;
    }
    // Timestamp difference between the frames coming from camera are not
    // exactly equidistant. Actual time from camera is actually SOF minus
    // line delta. After a certain point the difference between expected
    // time stamp and frame time stamp will be more than the threshold.

    auto timestamp_delta = expected_output_ts_ - buffer.timestamp;

    // We are considering here only early frame arrival by 1% time
    // stamp delta.
    if (timestamp_delta > 0 &&
        (timestamp_delta <
         (std::llround(FRC_TS_DELTA * output_frame_interval_)))) {
      expected_output_ts_ -= timestamp_delta;
    }
    // Increment the expected frame timestamp value.
    expected_output_ts_ += output_frame_interval_;

    // Increment output frame count, used for fps measurement.
    ++output_frame_count_;

    if (measurement_interval_ >= FRC_FPS_MEASUREMENT_INTERVAL) {
      // Dynamic source FPS measurement.
      int64_t frame_interval = measurement_interval_ / input_frame_count_;
      float fps = FRC_FRAME_RATE_TIMEBASE / frame_interval;

      if (debug_flags_ & FRC_DEBUG_INPUT_FPS) {
        QMMF_INFO("%s: %s: Source FPS: %.2f", __func__, name_.c_str(), fps);
      }

      // Dynamic output FPS measurement.
      frame_interval = measurement_interval_ / output_frame_count_;
      fps = FRC_FRAME_RATE_TIMEBASE / frame_interval;

      if (debug_flags_ & FRC_DEBUG_OUTPUT_FPS) {
        QMMF_INFO("%s: %s: Output FPS: %.2f", __func__, name_.c_str(), fps);
      }

      input_frame_count_    = 0;
      output_frame_count_   = 0;
      measurement_interval_ = 0;
    }
  }

  std::lock_guard<std::mutex> consumer_lock(consumer_lock_);
  buffer_producer_->NotifyBuffer(buffer);

  QMMF_DEBUG("%s: %s: Exit", __func__, name_.c_str());
  return;
}

void FrameRateController::NotifyBufferReturned(StreamBuffer& buffer) {

  buffer_consumer_->GetProducerHandle()->NotifyBufferReturned(buffer);
}

bool FrameRateController::SkipFrame(const StreamBuffer& buffer) {

  int64_t timestamp_delta = expected_output_ts_ - buffer.timestamp;
  int64_t max_delta = std::llround(FRC_THRESHOLD * output_frame_interval_);

  return ((timestamp_delta > 0) && (timestamp_delta >= max_delta));
}

bool FrameRateController::RepeatFrame(const StreamBuffer& buffer) {

  int64_t timestamp_delta = std::llabs(expected_output_ts_ - buffer.timestamp);
  double tshold = static_cast<double>(timestamp_delta) / output_frame_interval_;

  return ((input_frame_interval_ != 0) && (tshold >= FRC_THRESHOLD));
}

}; //namespace recorder

}; // namespace qmmf.
