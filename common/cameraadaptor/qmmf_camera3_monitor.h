/*
 * Copyright (c) 2016 The Linux Foundation. All rights reserved.
 * Not a Contribution.
 */

/*
 * Copyright (C) 2013 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef CAMERA3MONITOR_H_
#define CAMERA3MONITOR_H_

#include <map>
#include <vector>

#include <pthread.h>
#include <functional>

#include "common/utils/qmmf_thread.h"
#ifdef HAVE_ANDROID_UTILS
using namespace android;
#endif // HAVE_ANDROID_UTILS

namespace qmmf {

namespace cameraadaptor {

typedef std::function<void(bool idle)> IdleNotify;

class Camera3Monitor : public ThreadHelper {
 public:
  Camera3Monitor();
  virtual ~Camera3Monitor();

  void SetIdleNotifyCb(IdleNotify idle_cb) {idle_notify_ = idle_cb;}

  void ChangeStateToIdle(int32_t id);
  void ChangeStateToActive(int32_t id);

  static const int32_t INVALID_ID = -1;

  int32_t AcquireMonitor();
  void ReleaseMonitor(int32_t id);

  void RequestExit() override;
  void RequestExitAndWait() override;

 protected:
  bool ThreadLoop() override;

 private:
  enum MonitorState {
    IDLE,
    ACTIVE
  };

  void ChangeState(int32_t id, MonitorState state);
  MonitorState BuildCompositeState();

  struct StateTransition {
    int32_t id;
    MonitorState state;
  };
  pthread_mutex_t input_lock_;
  pthread_cond_t input_signal_;
  std::vector<StateTransition> input_queue_;
  bool monitor_updated_;
  bool exit_pending_;

  IdleNotify idle_notify_;

  pthread_mutex_t lock_;
  int32_t next_monitor_;
  std::map<int32_t, MonitorState> monitor_states_;
  MonitorState composite_state_;
  std::vector<MonitorState> updated_states_;

  static const int64_t WAIT_TIMEOUT = 250000000LL;  // 250 ms
};

}  // namespace cameraadaptor ends here

}  // namespace qmmf ends here

#endif /* CAMERA3MONITOR_H_ */
