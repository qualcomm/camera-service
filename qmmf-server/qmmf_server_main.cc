/*
* Copyright (c) 2016, 2020, The Linux Foundation. All rights reserved.
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

#include <atomic>

#ifdef HAVE_BINDER
#include <binder/IInterface.h>
#include <binder/IBinder.h>
#include <binder/ProcessState.h>
#include <binder/IServiceManager.h>
#include <binder/IPCThreadState.h>
#endif // HAVE_BINDER

#ifdef HAVE_ANDROID_UTILS
#include <cutils/properties.h>
#else
#include "properties.h"
#endif

#include "common/utils/qmmf_log.h"
#include "recorder/src/service/qmmf_recorder_service.h"

/**
 * Property to indicate completion of QMMF services initialization.
 * When completed, value is set to 1.
 */
#define QMMF_BOOT_COMPLETE "vendor.qmmf.boot.complete"

using namespace qmmf;
using namespace recorder;

#define INFO(...) \
  do { \
    printf(__VA_ARGS__); \
    printf("\n"); \
    QMMF_DEBUG(__VA_ARGS__); \
} while(0)

uint32_t qmmf_log_level;

int32_t main(int32_t argc, char **argv) {
  QMMF_GET_LOG_LEVEL();

#ifdef HAVE_BINDER
  //Add Recorder service.
  defaultServiceManager()->addService(String16(QMMF_RECORDER_SERVICE_NAME),
                  new qmmf::recorder::RecorderService(), false);
  INFO("Service(%s) Added successfully!", QMMF_RECORDER_SERVICE_NAME);

  android::ProcessState::self()->startThreadPool();
  android::ProcessState::self()->giveThreadPoolName();
  property_set(QMMF_BOOT_COMPLETE, "1");
  IPCThreadState::self()->joinThreadPool();
#else
  try {
    RecorderService server;
    server.MainLoop();
  } catch (int err) {
    return err;
  }
#endif // HAVE_BINDER
  return 0;
}
