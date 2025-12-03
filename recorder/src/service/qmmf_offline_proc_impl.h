/*
* Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#pragma once

#include <chicdk/chiofflinepostprocintf.h>

#include "common/utils/qmmf_common_utils.h"
#include "common/utils/qmmf_thread.h"
#include "qmmf_memory_interface.h"
#include "qmmf-sdk/qmmf_camera_metadata.h"
#include "qmmf-sdk/qmmf_recorder_params.h"
#include "qmmf-sdk/qmmf_vendor_tag_descriptor.h"
#include "qmmf-sdk/qmmf_offline_jpeg_params.h"
#include "qmmf-sdk/qmmf_offline_camera_params.h"
#include "recorder/src/service/qmmf_recorder_common.h"

namespace qmmf {

struct OfflineCbData;

struct OfflineCreateParams {
  void* pproc_instance;
  OfflineCbData* cb_data;
  PostProcCreateParams config;
};

struct OfflineRequests {
  uint32_t request_id;
  uint32_t npr;
  bool destroy_pending;
};

class OfflineProcess {
 public:
  OfflineProcess();
  ~OfflineProcess(){}
  status_t Init(const recorder::RemoteCallbackHandle& remote_cb_handle);
  status_t DeInit();
  status_t GetParams(const uint32_t client_id,
                     const OfflineCameraInputParams &in_params,
                     OfflineCameraOutputParams &out_params);
  status_t Create(const uint32_t client_id,
                  const OfflineCameraCreateParams& params);
  status_t Process(const uint32_t client_id,
                   const BnBuffer& in_buf0,
                   const BnBuffer& in_buf1,
                   const BnBuffer& out_buf,
                   const CameraMetadata& meta);
  status_t Destroy(const uint32_t client_id);
  status_t RegisterClient(const uint32_t client_id);
  status_t DeRegisterClient(const uint32_t client_id);
  bool IsClientFound(const uint32_t& client_id);
  void NotifyOfflineProc(const uint32_t& client_id,
                        const int32_t& buf_fd,
                        const uint32_t& out_size,
                        PostProcSessionParams* pproc_params);

 private:
  // Map between buffer id and fd
  typedef std::map<int32_t, int32_t> FdMap;

  int32_t GetBufferId(const uint32_t& client_id, const int32_t& buffer_fd);
  int32_t GetBufferFd(const uint32_t& client_id, const int32_t& buffer_id);
  uint32_t GetUsageFromFormat(BufferFormat format);

  void ReleaseRequestData(PostProcSessionParams* params);

  bool                                    offlineipe_enable;
  camera_module_t*                        camera_module_;
  int32_t                                 nubmer_of_cameras_;
  CameraMetadata                          device_info_;
  void*                                   offline_proc_lib_;
  PFN_CameraPostProc_Create               pCameraPostProcCreate;
  PFN_CameraPostProc_Process              pCameraPostProcProcess;
  PFN_CameraPostProc_Destroy              pCameraPostProcDestroy;
  PFN_CameraPostProc_ReleaseResources     pCameraPostProcRelease;

  // <client id, FdMap>
  std::map<uint32_t, FdMap>               client_fd_map_;
  std::mutex                              client_fd_lock_;

  // <client id, OfflineCreateParams>
  std::map<uint32_t, OfflineCreateParams> client_pproc_map_;
  std::mutex                              client_pproc_lock_;

  std::vector<uint32_t>                   clients_list_;

  std::mutex                              requests_lock_;
  QCondition                              requests_signal_;

  // <client id, OfflineRequests>
  std::map<uint32_t, OfflineRequests>     client_requests_map_;

  recorder::RemoteCallbackHandle          remote_cb_handle_;
};

struct OfflineCbData {
  OfflineProcess* offline_proc;
  uint32_t client_id;
};

int32_t OfflineCb(PostProcSessionParams* pproc_params,
                  uint32_t out_size,
                  void* user_data);

};  // namespace qmmf.
