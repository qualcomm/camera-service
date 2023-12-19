/*
* Copyright (c) 2016, 2018, The Linux Foundation. All rights reserved.
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
#include <thread>
#include <string>
#include <mutex>

#include <common/utils/qmmf_condition.h>
#include <sys/prctl.h>

namespace qmmf {

class ThreadHelper {
 private:
  enum class ThreadHelperState {
    kActive,   /// MainLoop thread has been created and is running.
    kToIdle,   /// Request ThreadLoop in MainLoop to stop doing work and exit.
    kIdle,     /// MainLoop thread has stopped but it is yet not joined.
    kInactive, /// MainLoop thread has been stopped and resources cleared.
  };

 public:
  ThreadHelper() : state_(ThreadHelperState::kInactive) {}

  virtual ~ThreadHelper() { RequestExitAndWait(); }

  int32_t Run(const std::string& name);

  virtual void RequestExit();
  virtual void RequestExitAndWait();

  bool ExitPending() { return (GetState() == ThreadHelperState::kToIdle); };

 protected:
  virtual bool ThreadLoop() = 0;

 private:
  void ChangeState(const ThreadHelperState& state);
  void WaitState(const ThreadHelperState& state);
  ThreadHelperState GetState();

  void MainLoop();

  std::string              name_;
  std::thread              thread_;

  ThreadHelperState        state_;
  std::mutex               state_lock_;
  QCondition               state_updated_;

  std::mutex               lock_;
};

};  // namespace qmmf.
