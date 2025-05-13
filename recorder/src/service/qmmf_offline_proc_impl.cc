/*
* Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#define LOG_TAG "OfflinePROC"

#include "qmmf_offline_proc_impl.h"

#include <dlfcn.h>
#include <cutils/native_handle.h>

#include "common/utils/qmmf_log.h"
#ifdef QCAMERA3_TAG_LOCAL_COPY
#include "common/utils/qmmf_common_utils.h"
#else
#include <QCamera3VendorTags.h>
#endif

namespace qmmf {

std::mutex CameraModule::lock_;
CameraModule* CameraModule::instance_ = nullptr;

static const uint64_t kWaitDuration = 1000000000; // 1 s.

OfflineProcess::OfflineProcess() :
                    offlineipe_enable(false),
                    camera_module_(nullptr),
                    nubmer_of_cameras_(-1),
                    offline_proc_lib_(nullptr),
                    pCameraPostProcCreate(nullptr),
                    pCameraPostProcProcess(nullptr),
                    pCameraPostProcDestroy(nullptr) {
  QMMF_INFO("%s: Enter ", __func__);
  QMMF_INFO("%s: Exit ", __func__);
}

status_t OfflineProcess::Init(
                  const recorder::RemoteCallbackHandle& remote_cb_handle) {
  QMMF_INFO("%s: Enter ", __func__);

  std::lock_guard<std::mutex> client_lock(client_pproc_lock_);
  assert(remote_cb_handle != nullptr);
  remote_cb_handle_ = remote_cb_handle;

  int32_t ret = NO_ERROR;

  // This is required for proper working of the jpeg lib
  ret = CameraModule::getInstance(&camera_module_);
  if (0 != ret || nullptr == camera_module_) {
    QMMF_ERROR("%s: Unable to load Hal module: %d\n", __func__, ret);
    return ret;
  }

  offline_proc_lib_ = dlopen(JPEG_POSTPROC_LIB, RTLD_NOW | RTLD_LOCAL);
  if (!offline_proc_lib_) {
    QMMF_ERROR("%s: No postproc lib, dlopen failed with: %s.",
            __func__, dlerror());
    return BAD_VALUE;
  }

  pCameraPostProcCreate   = (PFN_CameraPostProc_Create)
                              dlsym(offline_proc_lib_, "CameraPostProc_Create");
  pCameraPostProcProcess  = (PFN_CameraPostProc_Process)
                              dlsym(offline_proc_lib_, "CameraPostProc_Process");
  pCameraPostProcDestroy  = (PFN_CameraPostProc_Destroy)
                              dlsym(offline_proc_lib_, "CameraPostProc_Destroy");

  if ((nullptr == pCameraPostProcCreate)       ||
      (nullptr == pCameraPostProcDestroy)      ||
      (nullptr == pCameraPostProcProcess)) {
    QMMF_ERROR("%s: dlsym failed, %p, %p, %p", __func__,
            pCameraPostProcCreate,
            pCameraPostProcProcess,
            pCameraPostProcDestroy);
    return BAD_VALUE;
  }

  QMMF_INFO("%s: Exit ", __func__);
  return ret;
}

status_t OfflineProcess::DeInit() {
  QMMF_INFO("%s: Enter ", __func__);

  pCameraPostProcCreate = nullptr;
  pCameraPostProcProcess = nullptr;
  pCameraPostProcDestroy = nullptr;

  if (offline_proc_lib_) {
    dlclose(offline_proc_lib_);
    offline_proc_lib_ = nullptr;
  }

  {
    std::lock_guard<std::mutex> lock(client_pproc_lock_);
    if (!clients_list_.empty()) {
      clients_list_.clear();
    }
    if (!client_pproc_map_.empty()) {
      client_pproc_map_.clear();
    }
  }

  {
    std::lock_guard<std::mutex> lock(requests_lock_);
    if (!client_requests_map_.empty()) {
      client_requests_map_.clear();
    }
  }

  QMMF_INFO("%s: Exit ", __func__);
  return NO_ERROR;
}

status_t OfflineProcess::RegisterClient(const uint32_t client_id) {
  QMMF_INFO("%s: Enter client_id %d", __func__, client_id);

  std::lock_guard<std::mutex> client_lock(client_pproc_lock_);
  if (IsClientFound(client_id)) {
    QMMF_INFO("%s: Client %d already registered.", __func__, client_id);
    return ALREADY_EXISTS;
  }

  clients_list_.push_back(client_id);
  QMMF_INFO("%s: Client %d registered successfully", __func__, client_id);

  QMMF_INFO("%s: Exit client_id %d", __func__, client_id);

  return NO_ERROR;
}

status_t OfflineProcess::DeRegisterClient(const uint32_t client_id) {
  QMMF_INFO("%s: Enter client_id %d", __func__, client_id);

  std::lock_guard<std::mutex> client_lock(client_pproc_lock_);
  if (!IsClientFound(client_id)) {
    QMMF_ERROR("%s: Client %d not found.", __func__, client_id);
    return BAD_VALUE;
  }

  for (uint32_t i = 0; i < clients_list_.size(); i++) {
    if (client_id == clients_list_[i]) {
      clients_list_.erase(clients_list_.begin() + i);
      QMMF_INFO("%s: Client %d removed successfully.", __func__, client_id);
      break;
    }
  }

  QMMF_INFO("%s: Exit client_id %d", __func__, client_id);

  return NO_ERROR;
}

bool OfflineProcess::IsClientFound(const uint32_t& client_id) {
  bool found = false;
  for (uint32_t i = 0; i < clients_list_.size(); i++) {
    if (client_id == clients_list_[i]) {
      found = true;
      break;
    }
  }

  return found;
}

int32_t OfflineProcess::GetBufferId(const uint32_t& client_id,
                                        const int32_t& buffer_fd) {
  std::lock_guard<std::mutex> l(client_fd_lock_);
  int32_t buffer_id = -1;
  for (auto i : client_fd_map_[client_id]) {
    if (buffer_fd == i.second) {
      buffer_id = i.first;
      break;
    }
  }
  return buffer_id;
}

int32_t OfflineProcess::GetBufferFd(const uint32_t& client_id,
                                        const int32_t& buffer_id) {
  std::lock_guard<std::mutex> l(client_fd_lock_);
  return client_fd_map_[client_id][buffer_id];
}

status_t OfflineProcess::Create(const uint32_t client_id,
                                    const OfflineCameraCreateParams& params) {
  QMMF_INFO("%s: Enter client_id %d", __func__, client_id);

  std::lock_guard<std::mutex> client_lock(client_pproc_lock_);

  // get_number_of_cameras() must be called once prior using jpeg lib
  if (-1 == nubmer_of_cameras_) {
    nubmer_of_cameras_ = camera_module_->get_number_of_cameras();
    QMMF_DEBUG("%s: number of cameras %d", __func__, nubmer_of_cameras_);
  }

  if(!IsClientFound(client_id)) {
    QMMF_ERROR("%s Error: Client %d not found.", __func__, client_id);
    return BAD_VALUE;
  }

  OfflineCreateParams create_params;

  create_params.config.streamId = client_id;
  create_params.config.processMode = (PostProcMode)params.process_mode;

  create_params.config.inBuffer.width = params.in_buffer.width;
  create_params.config.inBuffer.height = params.in_buffer.height;
  create_params.config.inBuffer.format = params.in_buffer.format;

  create_params.config.outBuffer.width = params.out_buffer.width;
  create_params.config.outBuffer.height = params.out_buffer.height;
  create_params.config.outBuffer.format = params.out_buffer.format;

  create_params.config.clientCb = OfflineCb;
  create_params.cb_data = new OfflineCbData;
  create_params.cb_data->offline_proc = this;
  create_params.cb_data->client_id = client_id;

  create_params.config.clientData =
      reinterpret_cast<void*>(create_params.cb_data);

#ifdef FEATURE_OFFLINE_IPE_ENABLE
  create_params.config.cameraId = params.camera_id;
  create_params.config.pMedatadaDirPath = params.request_metadata_path;
  create_params.config.metadataSteps = params.metadata_step;
  CameraMetadata session_meta(params.session_meta);
  create_params.config.pMetadata = session_meta.getbuffer();
  offlineipe_enable = true;
#endif

  if ((create_params.config.processMode == YUVToYUV)
    && (!offlineipe_enable)) {
    QMMF_INFO("%s offline IPE module not support.", __func__);
    return BAD_VALUE;
  }

  create_params.pproc_instance = pCameraPostProcCreate(&create_params.config);
  if (!create_params.pproc_instance) {
    QMMF_ERROR("%s pproc_instance creation failed.", __func__);
    return BAD_VALUE;
  }

  {
    std::lock_guard<std::mutex> l(client_fd_lock_);
    client_fd_map_.emplace(client_id, FdMap());
  }

  client_pproc_map_.emplace(client_id, create_params);
  QMMF_INFO("%s pproc_instance %p for client %d created successfully",
            __func__, create_params.pproc_instance, client_id);

  std::lock_guard<std::mutex> request_lock(requests_lock_);
  client_requests_map_.emplace(client_id, OfflineRequests());

  QMMF_INFO("%s: Exit client_id %d", __func__, client_id);
  return NO_ERROR;
}

status_t OfflineProcess::Process(const uint32_t client_id,
                                     const BnBuffer& in_buf,
                                     const BnBuffer& out_buf,
                                     const CameraMetadata& meta) {
  QMMF_INFO("%s: Enter client_id %d", __func__, client_id);

  std::unique_lock<std::mutex> client_lock(client_pproc_lock_);
  if(!IsClientFound(client_id)) {
    QMMF_ERROR("%s Error: Client %d not found.", __func__, client_id);
    return BAD_VALUE;
  }

  native_handle_t *input_nh;
  native_handle_t *output_nh;

  //native_handle_create(int numFds, int numInts)
  //TODO: check if the below creation could be optimized
  input_nh = native_handle_create(2, 8);
  output_nh = native_handle_create(2, 8);

  //check if buf fd is present
  {
    std::lock_guard<std::mutex> l(client_fd_lock_);
    if (-1 != in_buf.ion_fd) {
      if (0 == client_fd_map_[client_id].count(in_buf.buffer_id)) {
        client_fd_map_[client_id].emplace(in_buf.buffer_id, in_buf.ion_fd);
      } else {
        QMMF_ERROR("%s: Error: Expected buf fd %d, but got %d for buf id (%d)",
                  __func__,
                  client_fd_map_[client_id].at(in_buf.buffer_id),
                  in_buf.ion_fd,
                  in_buf.buffer_id);
        native_handle_delete(input_nh);
        native_handle_delete(output_nh);
        return BAD_VALUE;
      }
    }
    if (-1 != out_buf.ion_fd) {
      if (0 == client_fd_map_[client_id].count(out_buf.buffer_id)) {
        client_fd_map_[client_id].emplace(out_buf.buffer_id, out_buf.ion_fd);
      } else {
        QMMF_ERROR("%s: Error: Expected buf fd %d, but got %d for buf id (%d)",
                  __func__,
                  client_fd_map_[client_id].at(out_buf.buffer_id),
                  out_buf.ion_fd,
                  out_buf.buffer_id);
        native_handle_delete(input_nh);
        native_handle_delete(output_nh);
        return BAD_VALUE;
      }
    }
  }

  PostProcHandleParams in_handle_params, out_handle_params;

  input_nh->data[0] = GetBufferFd(client_id, in_buf.buffer_id);
  in_handle_params.phHandle = input_nh;

  output_nh->data[0] = GetBufferFd(client_id, out_buf.buffer_id);
  out_handle_params.phHandle = output_nh;

  PostProcSessionParams* pproc_params = new PostProcSessionParams;
  if (!pproc_params) {
    QMMF_ERROR("%s: PosptProc param allocation failed", __func__);
    native_handle_delete(input_nh);
    native_handle_delete(output_nh);
    delete pproc_params;
    return NO_MEMORY;
  }

  pproc_params->streamId = client_id;
  pproc_params->valid = true;

  CameraMetadata proc_meta(meta);
  pproc_params->pMetadata = proc_meta.getbuffer();

  auto pproc_instance = client_pproc_map_.at(client_id).pproc_instance;
  if (!pproc_instance) {
    QMMF_ERROR("%s: No jpeg encoder instance for client %d",
              __func__, client_id);
    ReleaseRequestData(pproc_params);
    return BAD_VALUE;
  }
  QMMF_INFO("pproc instance: %p", pproc_instance);

  in_handle_params.format =
      client_pproc_map_.at(client_id).config.inBuffer.format;
  in_handle_params.width =
      client_pproc_map_.at(client_id).config.inBuffer.width;
  in_handle_params.height =
      client_pproc_map_.at(client_id).config.inBuffer.height;

  out_handle_params.format =
      client_pproc_map_.at(client_id).config.outBuffer.format;
  out_handle_params.width =
      client_pproc_map_.at(client_id).config.outBuffer.width;
  out_handle_params.height =
      client_pproc_map_.at(client_id).config.outBuffer.height;

  pproc_params->inHandle.push_back(in_handle_params);
  pproc_params->outHandle.push_back(out_handle_params);

  requests_lock_.lock();
  pproc_params->frameNum = client_requests_map_[client_id].request_id++;
  requests_lock_.unlock();

  QMMF_INFO("%s: Submitting postproc request %d for client %d. Buf fd %d inbuf_fd = %d",
            __func__, pproc_params->frameNum,
            client_id, pproc_params->outHandle[0].phHandle->data[0],
            pproc_params->inHandle[0].phHandle->data[0]);

  PostProcResultInfo res =
      pCameraPostProcProcess(pproc_instance, pproc_params);
  if (POSTPROCSUCCESS != res.result) {
    QMMF_ERROR("%s: postproc request %d for client %d failed with %d",
                  __func__,
                  pproc_params->frameNum,
                  client_id,
                  res);

    // In case of failure notify client with encoded size 0
    remote_cb_handle_(client_id)->NotifyOfflineProcData(out_buf.buffer_id, 0);

    ReleaseRequestData(pproc_params);
    return NO_ERROR;
  }

  requests_lock_.lock();
  client_requests_map_[client_id].npr++;
  requests_lock_.unlock();

  QMMF_INFO("%s: Exit client_id %d", __func__, client_id);
  return NO_ERROR;
}

status_t OfflineProcess::Destroy(const uint32_t client_id) {
  QMMF_INFO("%s: Enter client_id %d", __func__, client_id);

  std::unique_lock<std::mutex> client_lock(client_pproc_lock_);
  if(!IsClientFound(client_id)) {
    QMMF_ERROR("%s Error: Client %d not found.", __func__, client_id);
    return BAD_VALUE;
  }

  {
    std::unique_lock<std::mutex> lock(requests_lock_);
    client_requests_map_[client_id].destroy_pending = true;
    std::chrono::nanoseconds wait_time(kWaitDuration);

    while (0 != client_requests_map_[client_id].npr) {
      QMMF_INFO("%s: Waiting for submitted requests to finish", __func__);
      auto ret = requests_signal_.WaitFor(lock, wait_time);
      if (0 != ret) {
        QMMF_ERROR("%s: Waiting for frames timed out", __func__);
        break;
      }
      QMMF_INFO("%s: Waiting finished", __func__);
    }
    client_requests_map_.erase(client_id);
  }

  {
    std::lock_guard<std::mutex> l(client_fd_lock_);
    for (auto it : client_fd_map_[client_id]) {
      close(it.second);
    }
    client_fd_map_[client_id].clear();
    client_fd_map_.erase(client_id);
  }

  auto pproc_instance = client_pproc_map_.at(client_id).pproc_instance;
  if (!pproc_instance) {
    QMMF_ERROR("%s: No jpeg encoder instance for client %d!",
               client_id, __func__);
    return BAD_VALUE;
  }
  QMMF_INFO("%s: pproc instance: %p", __func__, pproc_instance);

  auto cb_data = client_pproc_map_.at(client_id).cb_data;
  if (cb_data) {
    delete cb_data;
    cb_data = nullptr;
  }

  client_pproc_map_.erase(client_id);

  pCameraPostProcDestroy(pproc_instance);
  pproc_instance = nullptr;

  QMMF_INFO("%s: Exit client_id %d", __func__, client_id);
  return NO_ERROR;
}

void OfflineProcess::ReleaseRequestData(PostProcSessionParams* params) {
  if (params) {
    native_handle_delete(
        const_cast<native_handle_t*>(params->inHandle[0].phHandle));
    native_handle_delete(
        const_cast<native_handle_t*>(params->outHandle[0].phHandle));
    delete params;
    params = nullptr;
  }
}

void OfflineProcess::NotifyOfflineProc(const uint32_t& client_id,
                                    const int32_t& buf_fd,
                                    const uint32_t& out_size,
                                    PostProcSessionParams* pproc_params) {

  QMMF_INFO("%s: Notifying client %d for buf_fd %d out_size %d",
          __func__,
          client_id,
          buf_fd,
          out_size);
  if(nullptr != remote_cb_handle_) {
    remote_cb_handle_(client_id)->NotifyOfflineProcData(
                                          GetBufferId(client_id, buf_fd),
                                          out_size);
  }

  ReleaseRequestData(pproc_params);

  {
    std::lock_guard<std::mutex> lock(requests_lock_);
    client_requests_map_[client_id].npr--;
    if (0 == client_requests_map_[client_id].npr &&
        client_requests_map_[client_id].destroy_pending) {
      requests_signal_.SignalAll();
    }
  }
}

int32_t OfflineCb(PostProcSessionParams* pproc_params,
               uint32_t out_size,
               void* user_data) {
  if (!pproc_params) {
    QMMF_ERROR("%s: pproc_params is null", __func__);
    return BAD_VALUE;
  }
  if (!user_data) {
    QMMF_ERROR("%s: user_data is null", __func__);
    return BAD_VALUE;
  }

  OfflineCbData* cb_data = reinterpret_cast<OfflineCbData*>(user_data);
  OfflineProcess* enc = cb_data->offline_proc;
  uint32_t client = cb_data->client_id;
  int32_t out_buf_fd = pproc_params->outHandle[0].phHandle->data[0];
  enc->NotifyOfflineProc(client, out_buf_fd, out_size, pproc_params);

  return NO_ERROR;
}

};  // namespace qmmf.
