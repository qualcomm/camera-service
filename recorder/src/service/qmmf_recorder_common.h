/*
 * Copyright (c) 2016-2019, 2021, The Linux Foundation. All rights reserved.
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

#pragma once


#include "qmmf-sdk/qmmf_recorder_params.h"
#include "common/utils/qmmf_common_utils_defs.h"
#include "common/utils/qmmf_log.h"

#include "qmmf_recorder_service_intf.h"
#include "recorder/src/service/qmmf_remote_cb.h"

#ifdef HAVE_BINDER
#define FRAME_DUMP_PATH        "/data/misc/qmmf"
#else
#define FRAME_DUMP_PATH        "/var/tmp/qmmf"
#endif // HAVE_BINDER

#define FPS_TIME_INTERVAL 3000000

#define REMAP_ALL_BUFFERS 0x55555555

// Enable DUMP_BITSTREAM to enable encoded data at TrackEncoder layer.
//#define DUMP_BITSTREAM

// Prop to enable debugging FPS
#define PROP_DEBUG_FPS        "persist.qmmf.rec.debug.fps"

namespace qmmf {

namespace recorder {

typedef std::function<void(std::vector<BnBuffer>& buffers,
    std::vector<BufferMeta>& metas)> BnBufferCallback;

typedef std::function<void(uint32_t camera_id, uint32_t imgcount,
    BnBuffer& buffer, BufferMeta& meta)>  SnapshotCb;

typedef std::function<void(uint32_t image_id, uint32_t imgcount,
    StreamBuffer& buffer)> StreamSnapshotCb;

typedef std::function<void(uint32_t camera_id,
    const CameraMetadata &result)> ResultCb;

#ifdef HAVE_ANDROID_UTILS
typedef std::function< const sp<RemoteCallBack>& (uint32_t client_id)>
    RemoteCallbackHandle;
#else
typedef std::function< const std::shared_ptr<RemoteCallBack>& (uint32_t client_id)>
    RemoteCallbackHandle;
#endif // HAVE_ANDROID_UTILS

typedef std::function<void(uint32_t camera_id, int32_t errcode)> ErrorCb;

typedef std::function<void(uint32_t camera_id, int32_t errcode)> SystemCb;

}; //namespace recorder.

}; //namespace qmmf.
