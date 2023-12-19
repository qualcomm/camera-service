/*
* Copyright (c) 2016-2018, 2021, The Linux Foundation. All rights reserved.
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
 * Changes from Qualcomm Innovation Center are provided under the following license:
 *
 * Copyright (c) 2021-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the
 * disclaimer below) provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *     * Neither the name of Qualcomm Innovation Center, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
 * GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
 * HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <utils/RefBase.h>

#include "common/utils/qmmf_common_utils.h"
#include "common/utils/qmmf_condition.h"
#include "common/utils/qmmf_log.h"

namespace qmmf {

namespace recorder {

using namespace android;

class IBufferConsumer;

// Buffer Producer interface.
class IBufferProducer : public RefBase {
 public:
  virtual ~IBufferProducer() {}

  // This method would provide the buffer to all connected consumers.
  virtual void NotifyBuffer(StreamBuffer& buffer) = 0;

  // By using this method consumers would return buffer back to producer once
  // they done with buffer.
  virtual void NotifyBufferReturned(StreamBuffer& buffer) = 0;

  // By using this method consumer can be added to producer's list of consumer.
  virtual void AddConsumer(const sp<IBufferConsumer>& consumer) = 0;

  // Consumer can be removed at any point of time.
  virtual void RemoveConsumer(sp<IBufferConsumer>& consumer) = 0;

  virtual status_t CheckAndWaitPendingBuffers() = 0;

  // Check if the buffer consumer is already connected to this producer.
  bool IsConnected(const sp<IBufferConsumer>& consumer) {
    std::lock_guard<std::mutex> lock(lock_);
    return IsConnectedLocked(consumer);
  }

  // To provide number of connected consumers.
  uint32_t GetNumConsumer() {
    std::lock_guard<std::mutex> lock(lock_);
    return buffer_consumers_.size();
  }

 protected:
  bool IsConnectedLocked(const sp<IBufferConsumer>& consumer) {
    uintptr_t key = reinterpret_cast<uintptr_t>(consumer.get());
    return (buffer_consumers_.count(key) != 0) ? true : false;
  }

  // Lock to protect list of consumer.
  std::mutex lock_;

  // Lock to protect buffer sequence, buffer would come back to produces
  // from different thread context
  std::mutex buffer_lock_;

  // Condition marking that a pending buffer has been received.
  QCondition buffer_received_;

  // Map of unique buffer handle and reference count.
  std::map<IBufferHandle, uint32_t>  buffers_;

  // List of consumers.
  std::map<uintptr_t, sp<IBufferConsumer>> buffer_consumers_;
};

class IBufferConsumer : public RefBase {
 public:
  virtual ~IBufferConsumer() {}

  // Consumer's method to handle incoming buffer.
  virtual void OnFrameAvailable(StreamBuffer& buffer) = 0;

  // Set handle of producer, would be used to return buffers back to producer.
  void SetProducerHandle(sp<IBufferProducer>& producer) {
    assert(producer.get() != nullptr);
    buffer_producer_ = producer;
  }

  // Reset producer handle to nullptr, used when the consumer is disconnected
  // from a producer.
  void ClearProducerHandle() { buffer_producer_.clear(); }

  // Get handle of producer, used to return buffers back to producer.
  sp<IBufferProducer>& GetProducerHandle() { return buffer_producer_; }

 protected:
  sp<IBufferProducer> buffer_producer_;

};

template <typename _type>
class BufferProducerImpl : public IBufferProducer {
 public:
  BufferProducerImpl(_type* source);

  ~BufferProducerImpl();

  void NotifyBuffer(StreamBuffer& buffer);

  void NotifyBufferReturned(StreamBuffer& Buffer);

  void AddConsumer(const sp<IBufferConsumer>& consumer);

  void RemoveConsumer(sp<IBufferConsumer>& consumer);

  status_t CheckAndWaitPendingBuffers();

 private:
  _type* source_;

};

template <typename _type>
class BufferConsumerImpl : public IBufferConsumer {

 public:
  BufferConsumerImpl(_type* source);

  ~BufferConsumerImpl();

  void OnFrameAvailable(StreamBuffer& buffer);

 private:
  _type * source_;

};

template <typename _type>
BufferProducerImpl<_type>::BufferProducerImpl(_type* source)
    : source_(source) {

  QMMF_VERBOSE("%s: Enter", __func__);
  QMMF_VERBOSE("%s: Exit (%p)", __func__, this);
}

template <typename _type>
BufferProducerImpl<_type>::~BufferProducerImpl() {

  QMMF_VERBOSE("%s: Enter", __func__);
  QMMF_VERBOSE("%s: Exit (%p)", __func__, this);
}

template <typename _type>
void BufferProducerImpl<_type>::NotifyBuffer(StreamBuffer& buffer) {

  std::lock_guard<std::mutex> lock(lock_);
  //Check for any consumer present. Notify them
  //about the new incoming buffer and keep reference count.
  if (buffer_consumers_.empty()) {
    QMMF_WARN("%s: No consumer connected to producer, return buffer", __func__);
    source_->NotifyBufferReturned(buffer);
    return;
  }

  {
    std::lock_guard<std::mutex> lock(buffer_lock_);
    buffers_[buffer.handle] += buffer_consumers_.size();

    QMMF_VERBOSE("%s: Buffer(%p) with reference count(%d)", __func__,
        buffer.handle, buffers_[buffer.handle]);

    QMMF_VERBOSE("%s: Notify buffer to %lu-consumers", __func__,
        buffer_consumers_.size());
  }

  for (auto& iter : buffer_consumers_) {
    (iter).second->OnFrameAvailable(buffer);
  }
}

template <typename _type>
void BufferProducerImpl<_type>::NotifyBufferReturned(StreamBuffer& buffer) {

  {
    std::lock_guard<std::mutex> lock(buffer_lock_);
    if (buffers_.count(buffer.handle) == 0) {
      QMMF_WARN("%s: Buffer(%p) has already been returned!", __func__,
          buffer.handle);
      return;
    }

    if (buffers_[buffer.handle] > 1) {
      // Hold this buffer, do not return until its reference count is 1.
      QMMF_VERBOSE("%s: Hold buffer(%p), reference count is not 1", __func__,
          buffer.handle);
      --buffers_[buffer.handle];
      return;
    }
    buffers_.erase(buffer.handle);
    buffer_received_.Signal();
  }

  QMMF_VERBOSE("%s: Return buffer(%p) to source", __func__, buffer.handle);
  buffer.in_use_client = false;
  source_->NotifyBufferReturned(buffer);
}

template <typename _type>
void BufferProducerImpl<_type>::AddConsumer(const sp<IBufferConsumer>&
                                            consumer) {

  assert(consumer.get() != nullptr);
  std::lock_guard<std::mutex> lock(lock_);

  if (IsConnectedLocked(consumer)) {
    QMMF_WARN("%s: Consumer(%p) already added to the producer!", __func__,
        consumer.get());
    return;
  }
  uintptr_t key = reinterpret_cast<uintptr_t>(consumer.get());
  buffer_consumers_.emplace(key, consumer);

  QMMF_VERBOSE("%s: Consumer(0x%lx) added successfully!", __func__, key);
}

template <typename _type>
void BufferProducerImpl<_type>::RemoveConsumer(sp<IBufferConsumer>& consumer) {

  assert(consumer.get() != nullptr);
  std::lock_guard<std::mutex> lock(lock_);

  if (!IsConnectedLocked(consumer)) {
    QMMF_WARN("%s: Consumer(%p) is not connected to this producer!",
        __func__, consumer.get());
    return;
  }
  uintptr_t key = reinterpret_cast<uintptr_t>(consumer.get());
  buffer_consumers_.erase(key);

  QMMF_VERBOSE("%s: Consumer(0x%lx) removed successfully!", __func__, key);
}

template <typename _type>
status_t BufferProducerImpl<_type>::CheckAndWaitPendingBuffers() {

  std::unique_lock<std::mutex> lock(buffer_lock_);
  std::chrono::nanoseconds wait_time(kWaitDelay);

  while (!buffers_.empty()) {
    QMMF_VERBOSE("%s: There are still %lu pending buffers, waiting for them"
        " to return!", __func__, buffers_.size());

    auto ret = buffer_received_.WaitFor(lock, wait_time);
    if (ret != 0) {
      QMMF_ERROR("%s: Wait pending buffers to return, timed out!", __func__);
      return -ETIMEDOUT;
    }
  }
  QMMF_VERBOSE("%s: All pending buffers returned!", __func__);
  return 0;
}

template <typename _type>
BufferConsumerImpl<_type>::BufferConsumerImpl(_type* source)
    : source_(source) {

  QMMF_VERBOSE("%s: Enter", __func__);
  QMMF_VERBOSE("%s: Exit (%p)", __func__, this);
}

template <typename _type>
BufferConsumerImpl<_type>::~BufferConsumerImpl() {

  QMMF_VERBOSE("%s: Enter", __func__);
  QMMF_VERBOSE("%s: Exit (%p)", __func__, this);
}

template <typename _type>
void BufferConsumerImpl<_type>::OnFrameAvailable(StreamBuffer& buffer) {

  source_->OnFrameAvailable(buffer);
}

};

};
