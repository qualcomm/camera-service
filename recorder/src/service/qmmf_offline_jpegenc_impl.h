/*
* Copyright (c) 2021-2022 Qualcomm Innovation Center, Inc. All rights reserved.
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

#include <chicdk/chiofflinepostprocintf.h>

#include "common/utils/qmmf_common_utils.h"
#include "common/utils/qmmf_thread.h"
#include "qmmf_memory_interface.h"
#include "qmmf-sdk/qmmf_recorder_params.h"
#include "recorder/src/service/qmmf_recorder_common.h"
#include "qmmf-sdk/qmmf_offline_jpeg_params.h"

namespace qmmf {

struct JpegCbData;

struct JpegCreateParams {
  void* pproc_instance;
  JpegCbData* cb_data;
  PostProcCreateParams config;
};

struct JpegRequests {
  uint32_t request_id;
  uint32_t npr;
  bool destroy_pending;
};

class OfflineJpegEncoder {
 public:
  OfflineJpegEncoder();
  ~OfflineJpegEncoder(){}
  status_t Init(const recorder::RemoteCallbackHandle& remote_cb_handle);
  status_t DeInit();
  status_t Create(const uint32_t client_id,
                  const OfflineJpegCreateParams& params);
  status_t Process(const uint32_t client_id,
                   const BnBuffer& in_buf,
                   const BnBuffer& out_buf,
                   const OfflineJpegMeta& meta);
  status_t Destroy(const uint32_t client_id);
  status_t RegisterClient(const uint32_t client_id);
  status_t DeRegisterClient(const uint32_t client_id);
  bool IsClientFound(const uint32_t& client_id);
  void NotifyJpeg(const uint32_t& client_id,
                  const int32_t& buf_fd,
                  const uint32_t& encoded_size,
                  PostProcSessionParams* pproc_params);

 private:
  // Map between buffer id and fd
  typedef std::map<int32_t, int32_t> FdMap;

  int32_t GetBufferId(const uint32_t& client_id, const int32_t& buffer_fd);
  int32_t GetBufferFd(const uint32_t& client_id, const int32_t& buffer_id);

  void ReleaseRequestData(PostProcSessionParams* params);

  camera_module_t*                        camera_module_;
  int32_t                                 nubmer_of_cameras_;
  void*                                   jpeg_lib_;
  PFN_CameraPostProc_Create               pCameraPostProcCreate;
  PFN_CameraPostProc_Process              pCameraPostProcProcess;
  PFN_CameraPostProc_Destroy              pCameraPostProcDestroy;

  // <client id, FdMap>
  std::map<uint32_t, FdMap>               client_fd_map_;
  std::mutex                              client_fd_lock_;

  // <client id, JpegCreateParams>
  std::map<uint32_t, JpegCreateParams>    client_pproc_map_;
  std::mutex                              client_pproc_lock_;

  std::vector<uint32_t>                   clients_list_;

  std::mutex                              requests_lock_;
  QCondition                              requests_signal_;

  // <client id, JpegRequests>
  std::map<uint32_t, JpegRequests>        client_requests_map_;

  recorder::RemoteCallbackHandle          remote_cb_handle_;

};

struct JpegCbData {
  OfflineJpegEncoder* encoder;
  uint32_t client_id;
};

int32_t JpegCb(PostProcSessionParams* pproc_params,
               uint32_t encoded_size,
               void* user_data);

};  // namespace qmmf.
