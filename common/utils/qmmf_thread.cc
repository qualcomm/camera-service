/*
* Copyright (c) 2018-2019, The Linux Foundation. All rights reserved.
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

#include "qmmf_thread.h"

#include <cerrno>
#include <exception>
#include <sstream>

#include "common/utils/qmmf_log.h"

namespace qmmf {

int32_t ThreadHelper::Run(const std::string& name) {

  std::lock_guard<std::mutex> l(lock_);
  switch (GetState()) {
    case ThreadHelperState::kActive:
      QMMF_WARN("%s: %s thread already started!", __func__, name_.c_str());
      return -EALREADY;
    case ThreadHelperState::kToIdle:
      QMMF_WARN("%s: %s thread is pending exit!", __func__, name_.c_str());
      WaitState(ThreadHelperState::kIdle);
      thread_.join();
      ChangeState(ThreadHelperState::kInactive);
      break;
    case ThreadHelperState::kIdle:
      thread_.join();
      ChangeState(ThreadHelperState::kInactive);
      break;
    case ThreadHelperState::kInactive:
      // Thread is inactive and ready to be started.
      break;
  }

  try {
    thread_ = std::thread([this]() -> void { MainLoop(); });
  } catch (const std::exception &e) {
    QMMF_ERROR("%s: Unable to create thread %s, exception: %s !", __func__,
        name_.c_str(), e.what());
    return -EINTR;
  }

  if (name.empty()) {
    std::stringstream ss;
    ss << thread_.get_id();
    name_ = ss.str();
  } else {
    name_ = name;
  }
  prctl(PR_SET_NAME, name.c_str(), 0, 0, 0);
  ChangeState(ThreadHelperState::kActive);
  QMMF_INFO("%s: %s thread is active!", __func__, name_.c_str());
  return 0;
}

void ThreadHelper::RequestExit() {

  std::lock_guard<std::mutex> l(lock_);
  switch (GetState()) {
    case ThreadHelperState::kActive:
      QMMF_DEBUG("%s: %s thread request exit!", __func__, name_.c_str());
      ChangeState(ThreadHelperState::kToIdle);
      break;
    case ThreadHelperState::kToIdle:
      QMMF_WARN("%s: %s thread is pending exit!", __func__, name_.c_str());
      break;
    case ThreadHelperState::kIdle:
      QMMF_DEBUG("%s: %s thread has exited!", __func__, name_.c_str());
      break;
    case ThreadHelperState::kInactive:
      QMMF_WARN("%s: %s thread hasn't been started!", __func__, name_.c_str());
      break;
  }
}

void ThreadHelper::RequestExitAndWait() {

  std::lock_guard<std::mutex> l(lock_);
  switch (GetState()) {
    case ThreadHelperState::kActive:
      QMMF_DEBUG("%s: %s thread request exit & wait!", __func__, name_.c_str());
      ChangeState(ThreadHelperState::kToIdle);
      break;
    case ThreadHelperState::kToIdle:
      QMMF_WARN("%s: %s thread is pending exit!", __func__, name_.c_str());
      WaitState(ThreadHelperState::kIdle);
      break;
    case ThreadHelperState::kIdle:
      QMMF_DEBUG("%s: %s thread has exited!", __func__, name_.c_str());
      break;
    case ThreadHelperState::kInactive:
      QMMF_WARN("%s: %s thread hasn't been started!", __func__, name_.c_str());
      return;
  }

  thread_.join();
  ChangeState(ThreadHelperState::kInactive);
  QMMF_INFO("%s: %s thread is inactive!", __func__, name_.c_str());
}

void ThreadHelper::ChangeState(const ThreadHelperState& state) {

  std::lock_guard<std::mutex> l(state_lock_);
  state_ = state;
  state_updated_.Signal();
}

void ThreadHelper::WaitState(const ThreadHelperState& state) {

  std::unique_lock<std::mutex> l(state_lock_);
  state_updated_.Wait(l, [&]() { return (state_ == state); });
}

ThreadHelper::ThreadHelperState ThreadHelper::GetState() {

  std::lock_guard<std::mutex> l(state_lock_);
  return state_;
}

void ThreadHelper::MainLoop() {

  bool active = true;
  while (active) {
    active = ThreadLoop();
  }
  ChangeState(ThreadHelperState::kIdle);
  return;
}

};  // namespace qmmf.
