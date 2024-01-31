/*
 * Copyright (c) 2017-2018, 2020-2021, The Linux Foundation. All rights reserved.
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

#include <memory>
#include <thread>

#include <qmmf-sdk/qmmf_recorder_extra_param_tags.h>

#include "common/utils/qmmf_common_utils.h"
#include "common/utils/qmmf_condition.h"
#include "recorder/src/service/qmmf_camera_interface.h"
#include "recorder/src/service/qmmf_recorder_common.h"

#include "common/resizer-interface/qmmf_resizer_interface.h"

namespace qmmf {

namespace recorder {

class IBufferProducer;

class IBufferConsumer;

class CameraRescalerThread {

 public:

  CameraRescalerThread()
      : thread_(nullptr),
         abort_(false),
         running_(false),
         name_("") {};

  virtual ~CameraRescalerThread() { RequestExitAndWait(); };

  int32_t Run(const std::string &name = std::string());

  virtual void RequestExit();

  virtual void RequestExitAndWait();

  bool ExitPending();

 protected:

  virtual bool ThreadLoop() = 0;

 private:

  static void *MainLoop(void *userdata);
  std::thread                   *thread_;
  std::mutex                    lock_;
  bool                          abort_;
  bool                          running_;
  std::string                   name_;
};


struct RescalerMemPoolParams {
  uint32_t width;
  uint32_t height;
  int32_t  format;
};

class CameraRescalerMemPool {

 public:

   CameraRescalerMemPool();

   ~CameraRescalerMemPool();

   int32_t Initialize(uint32_t width, uint32_t height, int32_t  format,
                      const CameraExtraParam& extra_param);

   status_t ReturnBufferLocked(const StreamBuffer &buffer);

   status_t GetFreeOutputBuffer(StreamBuffer* buffer);

 private:

   status_t GetBufferLocked(StreamBuffer* buffer);

   status_t PopulateBufferMeta(BufferMeta &info, IBufferHandle &handle);

   status_t AllocHWMemBuffer(IBufferHandle &buf);

   status_t FreeHWMemBuffer(IBufferHandle buf);
   IAllocDevice                 *alloc_device_interface_;
   IBufferHandle                *mem_alloc_slots_;
   uint32_t                      buffers_allocated_;
   uint32_t                      pending_buffer_count_;
   std::map<IBufferHandle, bool> mem_alloc_buffers_;

   RescalerMemPoolParams         init_params_;
   std::mutex                    buffer_lock_;
   QCondition                    wait_for_buffer_;

   static const int64_t kBufferWaitTimeout = 1000000000;// 1 s.
   uint32_t                      buffer_cnt_;
   bool                          is_eis_on_;
   bool                          is_ldc_on_;
};


class CameraRescalerBase : public CameraRescalerThread,
                           public CameraRescalerMemPool {

 public:

  CameraRescalerBase();

  ~CameraRescalerBase();

  void AddBuf(StreamBuffer& buffer);

  status_t MapBuf(StreamBuffer& buffer);

  void FlushBufs();

  void UnMapBufs();

  status_t Configure(const ResizerCrop& config_data);

 protected:
  // Thread for preparing synced and output buffers for processing by the
  // stitch library and passing them to the same library for stitching.
  bool ThreadLoop() override;

  // Pure virtual methods for handling the return of stream buffers
  // to their corresponding point of origin.
  virtual status_t NotifyBufferToClient(StreamBuffer &buffer) = 0;
  virtual status_t ReturnBufferToProducer(StreamBuffer &buffer) = 0;

  virtual status_t Start() = 0;
  virtual status_t Stop() = 0;

  // Method for returning an output buffer back to the memory pool.
  status_t ReturnBufferToBufferPool(const StreamBuffer &buffer);

  status_t Validate(const uint32_t& width, const uint32_t& height,
                    const BufferFormat& fmt);

 private:

  struct map_data_t {
    void* addr;
    size_t size;
  };

  static const int64_t              kFrameTimeout  = 50000000;  // 50 ms.
  std::map<uint32_t, map_data_t>    mapped_buffs_;
  std::vector<StreamBuffer>           bufs_list_;
  std::mutex                        wait_lock_;
  QCondition                        wait_;
  ResizerInterface*                 rescaler_;
  bool                              print_process_time_;
};

class CameraRescaler: public CameraRescalerBase {
 public:

  CameraRescaler();

  ~CameraRescaler();

  // Methods for establishing buffer communication link between the
  // consumer of the client and buffer producer of the stitching pipeline.
  status_t AddConsumer(const std::shared_ptr<IBufferConsumer>& consumer);

  status_t RemoveConsumer(std::shared_ptr<IBufferConsumer>& consumer);

  // Method to provide consumer interface, it would be used by a CameraContext
  // port producer to post buffers.
  std::shared_ptr<IBufferConsumer>& GetConsumer();

  // Method for handling incoming buffers from CameraPort buffer producer.
  void OnFrameAvailable(StreamBuffer& buffer);

  // Method for handling a buffer returned back from the CameraSource.
  void NotifyBufferReturned(const StreamBuffer& buffer);

  status_t NotifyBufferToClient(StreamBuffer &buffer) override;

  status_t ReturnBufferToProducer(StreamBuffer &buffer) override;

  status_t Start() override;

  status_t Stop() override;

  status_t Init(const uint32_t& width, const uint32_t& height,
                const BufferFormat& fmt,
                const float& in_fps, const float& out_fps,
                const CameraExtraParam& extra_param);

  bool IsStop();

  uint32_t GetNumConsumer();

 private:

  std::mutex               consumer_lock_;
  std::shared_ptr<IBufferConsumer>      buffer_consumer_impl_;
  std::shared_ptr<IBufferProducer>      buffer_producer_impl_;
  std::shared_ptr<IBufferConsumer>      copy_consumer_impl_;
  bool                     is_stop_;
  std::mutex               stop_lock_;

};

}; //namespace recorder

}; //namespace qmmf
