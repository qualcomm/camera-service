/*
* Copyright (c) 2016, 2020-2021, The Linux Foundation. All rights reserved.
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
* Changes from Qualcomm Innovation Center, Inc. are provided under the following license:
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

#define LOG_TAG "RecorderClient"

#include <type_traits>
#include <map>
#include <fcntl.h>
#include <dirent.h>
#include <dlfcn.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#ifdef HAVE_BINDER
#include <binder/Parcel.h>
#include <binder/ProcessState.h>
#include <binder/IPCThreadState.h>
#else
#include <cstring>
#include <functional>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <sys/un.h>
#include <thread>
#include <unistd.h>
#endif // HAVE_BINDER

#if TARGET_ION_ABI_VERSION >= 2
#include <linux/dma-buf.h>
#endif

#include "qmmf-sdk/qmmf_vendor_tag_descriptor.h"
#include "recorder/src/client/qmmf_recorder_client.h"
#include "recorder/src/service/qmmf_recorder_common.h"

#ifdef LOG_LEVEL_KPI
volatile uint32_t kpi_debug_level = BASE_KPI_FLAG;
#endif

uint32_t qmmf_log_level;

namespace qmmf {

namespace recorder {

//
// This file has implementation of following classes:
//
// - RecorderClient    : Delegation to binder proxy <IRecorderService>
//                       and implementation of binder CB.
// - RecorderServiceProxy : Binder proxy implementation.
// - RecorderServiceCallbackProxy : Binder CB proxy implementation.
// - RecorderServiceCallbackStub : Binder CB stub implementation.
//

#ifdef HAVE_BINDER
using namespace android;
using ::std::underlying_type;
#else
class RecorderServiceProxy: public IRecorderService {
 public:
  RecorderServiceProxy() {}
  ~RecorderServiceProxy() {
    QMMF_DEBUG("%s: Enter", __func__);
    if (socket_ != -1) {
      close(socket_);
    }
    service_cb_handler_.reset();
    QMMF_DEBUG("%s: Exit", __func__);
  }

  status_t Connect (const std::shared_ptr<IRecorderServiceCallback>& service_cb,
                    uint32_t* client_id) {
    // Connect to Server socket
    socket_ = socket(AF_UNIX, SOCK_STREAM, 0);
    if (socket_ == -1) {
      QMMF_ERROR("%s: sock failure %s", __func__, strerror(errno));
      return -errno;
    }

    // Set up server address
    std::string path{"/tmp/socket/cam_server/le_cam_socket"};
    sockaddr_un addr;
    addr.sun_family = AF_UNIX;
    auto size = path.size();
    snprintf(addr.sun_path, size+1, "%s", path.c_str());
    addr.sun_path[size+1] = '\0';
    if (connect(socket_, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
      close(socket_);
      QMMF_ERROR("%s: connect failure %s", __func__, strerror(errno));
      return -errno;
    }

    RecorderClientReqMsg cmd;
    status_t ret;
    uint32_t server_pid;
    cmd.set_command(RECORDER_SERVICE_CMDS::RECORDER_CONNECT);
    ret = SendRequest(cmd);
    if (ret != 0)
      return ret;

    RecorderClientRespMsg resp;
    ret = RecvResponse(resp);
    if (ret != 0)
      return ret;

    ret = resp.status();
    if (ret != 0)
      return ret;

    *client_id = resp.connect_resp().client_id();
    server_pid = resp.connect_resp().server_pid();

    // Callback handler
    ret = service_cb->Init(*client_id, server_pid);
    if(ret != 0)
      return ret;
    // CallbackSocketReady
    cmd.set_command(RECORDER_SERVICE_CMDS::RECORDER_CALLBACK_SOCKET_READY);
    cmd.mutable_callback_socket_ready()->set_client_id(*client_id);
    ret = SendRequest(cmd);
    if (ret != 0)
      return ret;

    ret = RecvResponse(resp);
    if (ret != 0)
      return ret;

    service_cb_handler_ = service_cb;
    ret = resp.status();
    return ret;
  }

  status_t Disconnect(const uint32_t client_id) {
    RecorderClientReqMsg cmd;
    cmd.set_command(RECORDER_SERVICE_CMDS::RECORDER_DISCONNECT);
    cmd.mutable_disconnect()->set_client_id(client_id);

    status_t ret;
    ret = SendRequest(cmd);
    if (ret < 0)
      return ret;

    RecorderClientRespMsg resp;
    ret = RecvResponse(resp);
    if (ret != 0)
      return ret;

    ret = resp.status();
    return ret;
  }

  status_t StartCamera(const uint32_t client_id, const uint32_t camera_id,
                       const float framerate,
                       const CameraExtraParam& extra_param,
                       bool enable_result_cb) {
    RecorderClientReqMsg cmd;
    cmd.set_command(RECORDER_SERVICE_CMDS::RECORDER_START_CAMERA);
    cmd.mutable_start_camera()->set_client_id(client_id);
    cmd.mutable_start_camera()->set_camera_id(camera_id);
    cmd.mutable_start_camera()->set_framerate(framerate);
    cmd.mutable_start_camera()->set_enable_result_cb(enable_result_cb);
    const void *extra_data = extra_param.GetAndLock();
    size_t extra_param_size = extra_param.Size();
    std::string *data = new std::string(
        reinterpret_cast<const char*>(extra_data), extra_param_size);
    cmd.mutable_start_camera()->set_allocated_extra_params(data);
    extra_param.ReturnAndUnlock(extra_data);

    status_t ret;
    ret = SendRequest(cmd);
    if (ret < 0)
      return ret;

    RecorderClientRespMsg resp;
    ret = RecvResponse(resp);
    if (ret < 0)
      return ret;

    ret = resp.status();
    return ret;
  }

  status_t StopCamera(const uint32_t client_id, const uint32_t camera_id) {
    RecorderClientReqMsg cmd;
    cmd.set_command(RECORDER_SERVICE_CMDS::RECORDER_STOP_CAMERA);
    cmd.mutable_stop_camera()->set_client_id(client_id);
    cmd.mutable_stop_camera()->set_camera_id(camera_id);

    status_t ret;
    ret = SendRequest(cmd);
    if (ret < 0)
      return ret;

    RecorderClientRespMsg resp;
    ret = RecvResponse(resp);
    if (ret < 0)
      return ret;

    ret = resp.status();
    return ret;
  }

  status_t CreateSession(const uint32_t client_id, uint32_t *session_id) {
    RecorderClientReqMsg cmd;
    cmd.set_command(RECORDER_SERVICE_CMDS::RECORDER_CREATE_SESSION);
    cmd.mutable_create_session()->set_client_id(client_id);

    status_t ret;
    ret = SendRequest(cmd);
    if (ret < 0)
      return ret;

    RecorderClientRespMsg resp;
    ret = RecvResponse(resp);
    if (ret < 0)
      return ret;

    *session_id = resp.create_session_resp().session_id();
    ret = resp.status();
    return ret;
  }

  status_t DeleteSession(const uint32_t client_id, const uint32_t session_id) {
    RecorderClientReqMsg cmd;
    cmd.set_command(RECORDER_SERVICE_CMDS::RECORDER_DELETE_SESSION);
    cmd.mutable_delete_session()->set_client_id(client_id);
    cmd.mutable_delete_session()->set_session_id(session_id);

    status_t ret;
    ret = SendRequest(cmd);
    if (ret < 0)
      return ret;

    RecorderClientRespMsg resp;
    ret = RecvResponse(resp);
    if (ret < 0)
      return ret;

    ret = resp.status();
    return ret;
  }

  status_t StartSession(const uint32_t client_id, const uint32_t session_id) {
    RecorderClientReqMsg cmd;
    cmd.set_command(RECORDER_SERVICE_CMDS::RECORDER_START_SESSION);
    cmd.mutable_start_session()->set_client_id(client_id);
    cmd.mutable_start_session()->set_session_id(session_id);

    status_t ret;
    ret = SendRequest(cmd);
    if (ret < 0)
      return ret;

    RecorderClientRespMsg resp;
    ret = RecvResponse(resp);
    if (ret < 0)
      return ret;

    ret = resp.status();
    return ret;
  }

  status_t StopSession(const uint32_t client_id, const uint32_t session_id,
                       bool do_flush) {
    RecorderClientReqMsg cmd;
    cmd.set_command(RECORDER_SERVICE_CMDS::RECORDER_STOP_SESSION);
    cmd.mutable_stop_session()->set_client_id(client_id);
    cmd.mutable_stop_session()->set_session_id(session_id);
    cmd.mutable_stop_session()->set_do_flush(do_flush);

    status_t ret;
    ret = SendRequest(cmd);
    if (ret < 0)
      return ret;

    RecorderClientRespMsg resp;
    ret = RecvResponse(resp);
    if (ret < 0)
      return ret;

    ret = resp.status();
    return ret;
  }

  status_t PauseSession(const uint32_t client_id, const uint32_t session_id) {
    RecorderClientReqMsg cmd;
    cmd.set_command(RECORDER_SERVICE_CMDS::RECORDER_PAUSE_SESSION);
    cmd.mutable_pause_session()->set_client_id(client_id);
    cmd.mutable_pause_session()->set_session_id(session_id);

    status_t ret;
    ret = SendRequest(cmd);
    if (ret < 0)
      return ret;

    RecorderClientRespMsg resp;
    ret = RecvResponse(resp);
    if (ret < 0)
      return ret;

    ret = resp.status();
    return ret;
  }

  status_t ResumeSession(const uint32_t client_id, const uint32_t session_id) {
    RecorderClientReqMsg cmd;
    cmd.set_command(RECORDER_SERVICE_CMDS::RECORDER_RESUME_SESSION);
    cmd.mutable_resume_session()->set_client_id(client_id);
    cmd.mutable_resume_session()->set_session_id(session_id);

    status_t ret;
    ret = SendRequest(cmd);
    if (ret < 0)
      return ret;

    RecorderClientRespMsg resp;
    ret = RecvResponse(resp);
    if (ret < 0)
      return ret;

    ret = resp.status();
    return ret;
  }

  status_t CreateVideoTrack(const uint32_t client_id,
                            const uint32_t session_id,
                            const uint32_t track_id,
                            const VideoTrackParam& params,
                            const VideoExtraParam& xtraparam) {
    RecorderClientReqMsg cmd;
    cmd.set_command(RECORDER_SERVICE_CMDS::RECORDER_CREATE_VIDEOTRACK);
    cmd.mutable_create_video_track()->set_client_id(client_id);
    cmd.mutable_create_video_track()->set_session_id(session_id);
    cmd.mutable_create_video_track()->set_track_id(track_id);

    VideoTrackParamMsg *vparam = cmd.mutable_create_video_track()->mutable_video_params();
    vparam->set_camera_id(params.camera_id);
    vparam->set_width(params.width);
    vparam->set_height(params.height);
    vparam->set_framerate(params.framerate);
    vparam->set_format(static_cast<VideoFormatMsg>(params.format));
    vparam->set_rotation(static_cast<RotationMsg>(params.rotation));
    vparam->set_xtrabufs(params.xtrabufs);
    vparam->set_flags(static_cast<VideoFlagsMsg>(params.flags));

    const void *extra_data = xtraparam.GetAndLock();
    size_t extra_param_size = xtraparam.Size();
    std::string *data = new std::string(
        reinterpret_cast<const char*>(extra_data), extra_param_size);
    cmd.mutable_create_video_track()->set_allocated_extra_params(data);
    xtraparam.ReturnAndUnlock(extra_data);

    status_t ret;
    ret = SendRequest(cmd);
    if (ret < 0)
      return ret;

    RecorderClientRespMsg resp;
    ret = RecvResponse(resp);
    if (ret < 0)
      return ret;

    ret = resp.status();
    return ret;
  }

  status_t DeleteVideoTrack(const uint32_t client_id,
                          const uint32_t session_id,
                          const uint32_t track_id) {
    RecorderClientReqMsg cmd;
    cmd.set_command(RECORDER_SERVICE_CMDS::RECORDER_DELETE_VIDEOTRACK);
    cmd.mutable_delete_video_track()->set_client_id(client_id);
    cmd.mutable_delete_video_track()->set_session_id(session_id);
    cmd.mutable_delete_video_track()->set_track_id(track_id);

    status_t ret;
    ret = SendRequest(cmd);
    if (ret < 0)
      return ret;

    RecorderClientRespMsg resp;
    ret = RecvResponse(resp);
    if (ret < 0)
      return ret;


    ret = resp.status();
    return ret;
  }

  status_t ReturnTrackBuffer(const uint32_t client_id,
                             const uint32_t session_id,
                             const uint32_t track_id,
                             std::vector<BnBuffer> &buffers) {

    QMMF_DEBUG("%s Enter", __func__);
    QMMF_VERBOSE("%s INPARAM: session_id[%u]", __func__, session_id);
    QMMF_VERBOSE("%s INPARAM: track_id[%u]", __func__, track_id);
    for (const BnBuffer& buffer : buffers) {
      QMMF_VERBOSE("%s INPARAM: buffers[%s]", __func__,
          buffer.ToString().c_str());
    }
    RecorderClientReqMsg cmd;
    cmd.set_command(RECORDER_SERVICE_CMDS::RECORDER_RETURN_TRACKBUFFER);
    cmd.mutable_return_track_buffer()->set_client_id(client_id);
    cmd.mutable_return_track_buffer()->set_session_id(session_id);
    cmd.mutable_return_track_buffer()->set_track_id(track_id);

    uint32_t size = buffers.size();
    for (uint32_t i = 0; i < size; i++) {
      BufferInfoMsg* buffer = cmd.mutable_return_track_buffer()->add_buffers();
      buffer->set_ion_fd(buffers[i].ion_fd);
      buffer->set_ion_meta_fd(buffers[i].ion_meta_fd);
      buffer->set_img_id(buffers[i].img_id);
      buffer->set_size(buffers[i].size);
      buffer->set_timestamp(buffers[i].timestamp);
      buffer->set_seqnum(buffers[i].seqnum);
      buffer->set_buffer_id(buffers[i].buffer_id);
      buffer->set_flags(buffers[i].flags);
      buffer->set_capacity(buffers[i].capacity);
    }

    status_t ret;
    RecorderClientRespMsg resp;
    {
      std::lock_guard<std::mutex> lock(socket_lock_);
      ret = SendRequest(cmd);
      if (ret < 0)
        return ret;
    }
    return ret;
  }

  status_t SetVideoTrackParam(const uint32_t client_id,
                              const uint32_t session_id,
                              const uint32_t track_id,
                              VideoParam type,
                              void *param, size_t size) {
    RecorderClientReqMsg cmd;
    cmd.set_command(RECORDER_SERVICE_CMDS::RECORDER_SET_VIDEOTRACK_PARAMS);
    cmd.mutable_set_video_track_param()->set_client_id(client_id);
    cmd.mutable_set_video_track_param()->set_session_id(session_id);
    cmd.mutable_set_video_track_param()->set_track_id(track_id);
    cmd.mutable_set_video_track_param()->set_type(static_cast<VIDEO_PARAM>(type));

    std::string *data = new std::string(
        reinterpret_cast<const char*>(param), size);
    cmd.mutable_set_video_track_param()->set_allocated_param(data);

    status_t ret;
    ret = SendRequest(cmd);
    if (ret < 0)
      return ret;

    RecorderClientRespMsg resp;
    ret = RecvResponse(resp);
    if (ret < 0)
      return ret;

    ret = resp.status();
    return ret;
  }

  status_t CaptureImage(const uint32_t client_id, const uint32_t camera_id,
                        const SnapshotType type, const uint32_t n_images,
                        const std::vector<CameraMetadata> &meta) {
    QMMF_DEBUG("%s Enter ", __func__);
    RecorderClientReqMsg cmd;
    cmd.set_command(RECORDER_SERVICE_CMDS::RECORDER_CAPTURE_IMAGE);
    cmd.mutable_capture_image()->set_client_id(client_id);
    cmd.mutable_capture_image()->set_camera_id(camera_id);
    cmd.mutable_capture_image()->set_type(static_cast<SNAPSHOT_TYPE>(type));
    cmd.mutable_capture_image()->set_n_images(n_images);

    uint32_t size = meta.size();
    for (uint32_t i = 0; i < size; i++) {
      const camera_metadata_t *meta_buffer = meta[i].getAndLock();
      uint32_t size = get_camera_metadata_compact_size(meta_buffer);
      std::string *data = cmd.mutable_capture_image()->add_meta();
      data->resize(size);
      copy_camera_metadata (&data->at(0), data->size(), meta_buffer);
      const_cast<CameraMetadata&>(meta[i]).unlock(meta_buffer);
    }

    status_t ret;
    ret = SendRequest(cmd);
    if (ret < 0)
      return ret;

    RecorderClientRespMsg resp;
    ret = RecvResponse(resp);
    if (ret < 0)
      return ret;

    ret = resp.status();
    QMMF_DEBUG("%s Exit ", __func__);
    return ret;
  }

  status_t ConfigImageCapture(const uint32_t client_id,
                              const uint32_t camera_id,
                              const uint32_t image_id,
                              const ImageParam &param,
                              const ImageExtraParam &xtraparam) {
    RecorderClientReqMsg cmd;
    cmd.set_command(RECORDER_SERVICE_CMDS::RECORDER_CONFIG_IMAGECAPTURE);
    cmd.mutable_config_image_capture()->set_client_id(client_id);
    cmd.mutable_config_image_capture()->set_camera_id(camera_id);
    cmd.mutable_config_image_capture()->set_image_id(image_id);

    ImageParamMsg *iparam = cmd.mutable_config_image_capture()->mutable_image_param();
    iparam->set_mode(static_cast<ImageModeMsg>(param.mode));
    iparam->set_width(param.width);
    iparam->set_height(param.height);
    iparam->set_format(static_cast<ImageFormatMsg>(param.format));
    iparam->set_quality(param.quality);
    iparam->set_rotation(static_cast<RotationMsg>(param.rotation));

    const void *extra_data = xtraparam.GetAndLock();
    size_t extra_param_size = xtraparam.Size();
    std::string *data = new std::string(
        reinterpret_cast<const char*>(extra_data), extra_param_size);
    cmd.mutable_config_image_capture()->set_allocated_extra_param(data);
    xtraparam.ReturnAndUnlock(extra_data);

    status_t ret;
    ret = SendRequest(cmd);
    if (ret < 0)
      return ret;

    RecorderClientRespMsg resp;
    ret = RecvResponse(resp);
    if (ret < 0)
      return ret;

    ret = resp.status();
    return ret;
  }

  status_t CancelCaptureImage(const uint32_t client_id,
                              const uint32_t camera_id,
                              const uint32_t image_id,
                              const bool cache) {
    RecorderClientReqMsg cmd;
    cmd.set_command(RECORDER_SERVICE_CMDS::RECORDER_CANCEL_IMAGECAPTURE);
    cmd.mutable_cancel_image_capture()->set_client_id(client_id);
    cmd.mutable_cancel_image_capture()->set_camera_id(camera_id);
    cmd.mutable_cancel_image_capture()->set_image_id(image_id);
    cmd.mutable_cancel_image_capture()->set_cache(cache);

    status_t ret;
    ret = SendRequest(cmd);
    if (ret < 0)
      return ret;

    RecorderClientRespMsg resp;
    ret = RecvResponse(resp);
    if (ret < 0)
      return ret;

    ret = resp.status();
    return ret;
  }

  status_t ReturnImageCaptureBuffer(const uint32_t client_id,
                                    const uint32_t camera_id,
                                    const int32_t buffer_id) {
    RecorderClientReqMsg cmd;
    cmd.set_command(RECORDER_SERVICE_CMDS::RECORDER_RETURN_IMAGECAPTURE_BUFFER);
    cmd.mutable_return_image_capture_buffer()->set_client_id(client_id);
    cmd.mutable_return_image_capture_buffer()->set_camera_id(camera_id);
    cmd.mutable_return_image_capture_buffer()->set_buffer_id(buffer_id);

    status_t ret;
    ret = SendRequest(cmd);
    if (ret < 0)
      return ret;

    RecorderClientRespMsg resp;
    ret = RecvResponse(resp);
    if (ret < 0)
      return ret;

    ret = resp.status();
    return ret;
  }

  status_t SetCameraParam(const uint32_t client_id,
                          const uint32_t camera_id,
                          const CameraMetadata &meta) {
    RecorderClientReqMsg cmd;
    cmd.set_command(RECORDER_SERVICE_CMDS::RECORDER_SET_CAMERA_PARAMS);
    cmd.mutable_set_camera_param()->set_client_id(client_id);
    cmd.mutable_set_camera_param()->set_camera_id(camera_id);

    const camera_metadata_t *meta_buffer = meta.getAndLock();
    uint32_t size = get_camera_metadata_compact_size(meta_buffer);
    std::string *data = new std::string;
    data->resize(size);
    auto copy_ptr = copy_camera_metadata (&data->at(0), data->size(), meta_buffer);
    if (!copy_ptr) {
      QMMF_ERROR ("%s: Failed to copy metadata", __func__);
      return -1;
    }

    cmd.mutable_set_camera_param()->set_allocated_meta(data);
    const_cast<CameraMetadata&>(meta).unlock(meta_buffer);

    status_t ret;
    ret = SendRequest(cmd);
    if (ret < 0)
      return ret;

    RecorderClientRespMsg resp;
    ret = RecvResponse(resp);
    if (ret < 0)
      return ret;

    ret = resp.status();
    return ret;
  }

  status_t GetCameraParam(const uint32_t client_id,
                          const uint32_t camera_id,
                          CameraMetadata &meta) {
    RecorderClientReqMsg cmd;
    cmd.set_command(RECORDER_SERVICE_CMDS::RECORDER_GET_CAMERA_PARAMS);
    cmd.mutable_get_camera_param()->set_client_id(client_id);
    cmd.mutable_get_camera_param()->set_camera_id(camera_id);

    status_t ret;
    ret = SendRequest(cmd);
    if (ret < 0)
      return false;

    RecorderClientRespMsg resp;
    ret = RecvResponse(resp);
    if (ret < 0)
      return ret;

    const std::string& data = resp.get_camera_param_resp().meta();
    uint8_t *raw_buf = new uint8_t[data.size()];
    camera_metadata_t *meta_buffer =
        copy_camera_metadata (raw_buf, data.size(), reinterpret_cast<const camera_metadata_t *>(data.data()));
    if (!meta_buffer) {
      QMMF_ERROR ("%s: Failed to copy metadata", __func__);
      return -1;
    }
    meta.clear();
    meta.acquire(meta_buffer);

    ret = resp.status();
    return ret;
  }

  status_t SetCameraSessionParam(const uint32_t client_id,
                          const uint32_t camera_id,
                          const CameraMetadata &meta) {
    RecorderClientReqMsg cmd;
    cmd.set_command(RECORDER_SERVICE_CMDS::RECORDER_SET_CAMERA_SESSION_PARAMS);
    cmd.mutable_set_camera_session_param()->set_client_id(client_id);
    cmd.mutable_set_camera_session_param()->set_camera_id(camera_id);

    const camera_metadata_t *meta_buffer = meta.getAndLock();
    uint32_t size = get_camera_metadata_compact_size(meta_buffer);

    std::string *data = new std::string;
    data->resize(size);
    auto copy_ptr = copy_camera_metadata (&data->at(0), data->size(), meta_buffer);
    if (!copy_ptr) {
      QMMF_ERROR ("%s: Failed to copy metadata", __func__);
      return -1;
    }
    cmd.mutable_set_camera_session_param()->set_allocated_meta(data);
    const_cast<CameraMetadata&>(meta).unlock(meta_buffer);

    status_t ret;
    ret = SendRequest(cmd);
    if (ret < 0)
      return ret;

    RecorderClientRespMsg resp;
    ret = RecvResponse(resp);
    if (ret < 0)
      return ret;

    ret = resp.status();
    return ret;
  }

  status_t SetSHDR(const uint32_t client_id,
                   const uint32_t camera_id,
                   const bool enable) {
  }

  status_t GetDefaultCaptureParam(const uint32_t client_id,
                                  const uint32_t camera_id,
                                  CameraMetadata &meta) {
    RecorderClientReqMsg cmd;
    cmd.set_command(RECORDER_SERVICE_CMDS::RECORDER_GET_DEFAULT_CAPTURE_PARAMS);
    cmd.mutable_get_default_capture_param()->set_client_id(client_id);
    cmd.mutable_get_default_capture_param()->set_camera_id(camera_id);

    status_t ret;
    ret = SendRequest(cmd);
    if (ret < 0)
      return false;

    RecorderClientRespMsg resp;
    ret = RecvResponse(resp);
    if (ret < 0)
      return ret;

    const std::string& data = resp.get_default_capture_param_resp().meta();
    uint8_t *raw_buf = new uint8_t[data.size()];
    camera_metadata_t *meta_buffer =
        copy_camera_metadata (raw_buf, data.size(), reinterpret_cast<const camera_metadata_t *>(data.data()));
    if (!meta_buffer) {
      QMMF_ERROR ("%s: Failed to copy metadata", __func__);
      return -1;
    }

    meta.clear();
    meta.acquire(meta_buffer);

    ret = resp.status();
    return ret;
  }

  status_t GetCameraCharacteristics(const uint32_t client_id,
                                    const uint32_t camera_id,
                                    CameraMetadata &meta) {
    RecorderClientReqMsg cmd;
    cmd.set_command(RECORDER_SERVICE_CMDS::RECORDER_GET_CAMERA_CHARACTERISTICS);
    cmd.mutable_get_camera_characteristics()->set_client_id(client_id);
    cmd.mutable_get_camera_characteristics()->set_camera_id(camera_id);

    status_t ret;
    ret = SendRequest(cmd);
    if (ret < 0)
      return ret;

    RecorderClientRespMsg resp;
    ret = RecvResponse(resp);
    if (ret < 0)
      return ret;

    const std::string& data = resp.get_camera_characteristics_resp().meta();
    uint8_t *raw_buf = new uint8_t[data.size()];
    camera_metadata_t *meta_buffer =
        copy_camera_metadata (raw_buf, data.size(), reinterpret_cast<const camera_metadata_t *>(data.data()));
    if (!meta_buffer) {
      QMMF_ERROR ("%s: Failed to copy metadata", __func__);
      return -1;
    }

    meta.clear();
    meta.acquire(meta_buffer);

    return resp.status();
  }

  status_t GetVendorTagDescriptor(std::shared_ptr<VendorTagDescriptor> &desc) {
    RecorderClientReqMsg cmd;
    cmd.set_command(RECORDER_SERVICE_CMDS::RECORDER_GET_VENDOR_TAG_DESCRIPTOR);

    status_t ret;
    ret = SendRequest(cmd);
    if (ret < 0)
      return ret;

    RecorderClientRespMsg resp;
    ret = RecvResponse(resp);
    if (ret < 0)
      return ret;

    const std::string& data = resp.get_vendor_tag_descriptor_resp().descs();
    desc->readFromBuffer(reinterpret_cast<const uint8_t *>(data.data()));

    return resp.status();
  }

  status_t CreateOfflineJPEG(const uint32_t client_id,
                             const OfflineJpegCreateParams &params) {

  }

  status_t EncodeOfflineJPEG(const uint32_t client_id,
                             const BnBuffer& in_buf,
                             const BnBuffer& out_buf,
                             const OfflineJpegMeta& meta) {
  }

  status_t DestroyOfflineJPEG(const uint32_t client_id) {
  }
 private:
  status_t SendRequest(RecorderClientReqMsg &cmd) {
    uint8_t offset = 4;
    auto msg_size = cmd.ByteSizeLong();
    auto buf_size = msg_size + offset;
    void *buffer = malloc(buf_size);
    *(static_cast<uint32_t *>(buffer)) = msg_size;
    cmd.SerializeToArray(buffer+offset, msg_size);

    ssize_t bytes_sent = send(socket_, buffer, buf_size, 0);
    free (buffer);
    if (bytes_sent == -1) {
      return -errno;
    }

    QMMF_DEBUG("%s: Sent to server cmd: %u, buf_size:%ld, bytes:%ld",
        __func__, cmd.command(), buf_size, bytes_sent);
    return 0;
  }

  status_t RecvResponse(RecorderClientRespMsg &resp) {
    char buffer[kMaxSocketBufSize] = {0};

    ssize_t bytes_read = recv(socket_, buffer, sizeof(buffer), 0);
    if (bytes_read == -1) {
      return -errno;
    } else  if (bytes_read == 0) {
      return -1;
    }

    resp.ParseFromArray(buffer, bytes_read);
    QMMF_DEBUG("%s: Received from server: %u bytes:%ld",
        __func__, resp.command(), bytes_read);

    return 0;
  }

  std::shared_ptr<IRecorderServiceCallback> service_cb_handler_;
  int socket_;
  std::mutex socket_lock_;
};
#endif // HAVE_BINDER

RecorderClient::RecorderClient()
    : recorder_service_(nullptr),
      death_notifier_(nullptr),
      ion_device_(-1),
      client_id_(0),
      metadata_cb_(nullptr),
      vendor_tag_desc_(nullptr) {

  QMMF_GET_LOG_LEVEL();
  QMMF_KPI_GET_MASK();
  QMMF_KPI_DETAIL();
  QMMF_INFO("%s Enter ", __func__);

#ifdef TARGET_USES_GBM
  gbm_fd_ = open("/dev/dma_heap/qcom,system", O_RDWR);
  if (gbm_fd_ < 0) {
    QMMF_WARN("%s: Falling back to /dev/ion \n", __func__);
    gbm_fd_ = open("/dev/ion", O_RDWR);
  }
  assert(gbm_fd_ >= 0);

  gbm_device_ = gbm_create_device(gbm_fd_);
  assert(gbm_device_ != nullptr);
#endif

#ifdef HAVE_BINDER
  sp<ProcessState> proc(ProcessState::self());
  proc->startThreadPool();
#endif // HAVE_BINDER
  QMMF_INFO("%s Exit (0x%p)", __func__, this);
}

RecorderClient::~RecorderClient() {

  QMMF_INFO("%s Enter ", __func__);
  QMMF_KPI_DETAIL();

  recorder_service_.reset();

#ifdef TARGET_USES_GBM
  gbm_device_destroy(gbm_device_);
  close(gbm_fd_);
#endif

  QMMF_INFO("%s Exit 0x%p", __func__, this);
}

status_t RecorderClient::Connect(const RecorderCb& cb) {

  QMMF_DEBUG("%s Enter ", __func__);
  QMMF_KPI_DETAIL();
  std::lock_guard<std::mutex> lock(lock_);

  if (CheckServiceStatus()) {
    QMMF_WARN("%s Client is already connected to service!", __func__);
    return 0;
  }

  ion_device_ = open("/dev/dma_heap/qcom,system", O_RDONLY | O_CLOEXEC);
  if (ion_device_ < 0) {
    QMMF_WARN("%s: Falling back to /dev/ion \n", __func__);
    ion_device_ = open("/dev/ion", O_RDONLY | O_CLOEXEC);
  }
  if (ion_device_ < 0) {
    QMMF_ERROR("%s: Can't open Ion device!", __func__);
    return -ENODEV;
  }

  recorder_cb_ = cb;

  NotifyServerDeathCB death_cb = [&] { ServiceDeathHandler(); };
  death_notifier_ = std::make_unique<DeathNotifier>(death_cb);
  if (nullptr == death_notifier_.get()) {
    QMMF_ERROR("%s Unable to allocate death notifier!", __func__);
    return -ENOMEM;
  }

  uint32_t client_id;
#ifdef HAVE_BINDER
  sp<IBinder> service_handle;
  sp<IServiceManager> service_manager = defaultServiceManager();

  service_handle = service_manager->
      getService(String16(QMMF_RECORDER_SERVICE_NAME));
  if (service_handle.get() == nullptr) {
    QMMF_ERROR("%s Can't get (%s) service", __func__,
        QMMF_RECORDER_SERVICE_NAME);
    return -ENODEV;
  }

  recorder_service_ = interface_cast<IRecorderService>(service_handle);
  IInterface::asBinder(recorder_service_)->linkToDeath(death_notifier_);

  sp<ServiceCallbackHandler> handler = new ServiceCallbackHandler(this);
  auto ret = recorder_service_->Connect(handler, &client_id);
  if (0 != ret) {
    QMMF_ERROR("%s Can't connect to (%s) service", __func__,
        QMMF_RECORDER_SERVICE_NAME);
  }
#else

  recorder_service_ = std::make_unique<RecorderServiceProxy>();

  std::shared_ptr<ServiceCallbackHandler> handler =
      std::make_shared<ServiceCallbackHandler>(this);
  auto ret = recorder_service_->Connect(handler, &client_id);
  if (0 != ret) {
    QMMF_ERROR("%s Can't connect to (%s) service", __func__,
        QMMF_RECORDER_SERVICE_NAME);
  }

#endif // HAVE_BINDER
  client_id_ = client_id;
  QMMF_INFO("%s: client_id(%d)", __func__, client_id);

  session_cb_list_.clear();
  track_cb_list_.clear();

  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t RecorderClient::Disconnect() {

  QMMF_DEBUG("%s Enter ", __func__);
  QMMF_KPI_DETAIL();
  std::lock_guard<std::mutex> lock(lock_);

  if (!CheckServiceStatus()) {
    return -ENODEV;
  }

  auto ret = recorder_service_->Disconnect(client_id_);
  if (0 != ret) {
    QMMF_ERROR("%s Disconnect failed!", __func__);
  }

#ifdef HAVE_BINDER
  recorder_service_->asBinder(recorder_service_)->
      unlinkToDeath(death_notifier_);
#else
  {
    std::unique_lock<std::mutex> lock(disconnect_lock_);
    wait_for_disconnect_.Signal();
  }
#endif // HAVE_BINDER

  recorder_service_.reset();
  death_notifier_.reset();

  sessions_.clear();
  session_cb_list_.clear();
  track_cb_list_.clear();

  if (ion_device_ > 0) {
    close(ion_device_);
    ion_device_ = -1;
  }
  client_id_ = 0;

  // Clear global tag descriptor for the process
  VendorTagDescriptor::clearGlobalVendorTagDescriptor();
  vendor_tag_desc_ = nullptr;

  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t RecorderClient::StartCamera(const uint32_t camera_id,
                                     const float framerate,
                                     const CameraExtraParam& extra_param,
                                     const CameraResultCb &result_cb) {
  bool enable_result_cb = false;
  QMMF_DEBUG("%s Enter ", __func__);
  QMMF_KPI_DETAIL();
  std::lock_guard<std::mutex> lock(lock_);

  if (!CheckServiceStatus()) {
    return -ENODEV;
  }

  if (nullptr != result_cb) {
    metadata_cb_ = result_cb;
    enable_result_cb = true;
  }

  assert(client_id_ > 0);
  auto ret = recorder_service_->StartCamera(client_id_, camera_id, framerate,
                                            extra_param,
                                            enable_result_cb);
  if (0 != ret) {
    QMMF_ERROR("%s StartCamera failed!", __func__);
    return ret;
  }

  if (vendor_tag_desc_ == nullptr) {
    vendor_tag_desc_ = std::make_shared<VendorTagDescriptor>();
    ret = GetVendorTagDescriptor(vendor_tag_desc_);
    if (0 != ret) {
      QMMF_ERROR("%s: Unable to GetVendorTagDescriptor : %d\n", __func__, ret);
      return ret;
    }

    // Set the global descriptor to use with camera metadata
    ret =
        VendorTagDescriptor::setAsGlobalVendorTagDescriptor(vendor_tag_desc_);
    if (0 != ret) {
      QMMF_ERROR("%s: Unable to setAsGlobalVendorTagDescriptor : %d", __func__,
                 ret);
      return ret;
    }
  }

  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t RecorderClient::StopCamera(const uint32_t camera_id) {

  QMMF_DEBUG("%s Enter ", __func__);
  QMMF_KPI_DETAIL();

  std::lock_guard<std::mutex> lock(lock_);

  if (!CheckServiceStatus()) {
    return -ENODEV;
  }

  assert(client_id_ > 0);
  auto ret = recorder_service_->StopCamera(client_id_, camera_id);
  if (0 != ret) {
    QMMF_ERROR("%s StopCamera failed!", __func__);
  }
  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t RecorderClient::CreateSession(const SessionCb& cb,
                                       uint32_t* session_id) {
  QMMF_DEBUG("%s Enter ", __func__);
  QMMF_KPI_DETAIL();
  std::lock_guard<std::mutex> lock(lock_);

  if (!CheckServiceStatus()) {
    return -ENODEV;
  }

  assert(client_id_ > 0);
  auto ret = recorder_service_->CreateSession(client_id_, session_id);
  if (0 != ret) {
    QMMF_ERROR("%s CreateSession failed!", __func__);
  } else {
    sessions_.emplace(*session_id, std::set<uint32_t>());
    session_cb_list_.emplace(*session_id, cb);
  }

  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t RecorderClient::DeleteSession(const uint32_t session_id) {

  QMMF_DEBUG("%s Enter ", __func__);
  QMMF_KPI_DETAIL();
  std::lock_guard<std::mutex> lock(lock_);

  if (!CheckServiceStatus()) {
    return -ENODEV;
  }

  if (sessions_.count(session_id) == 0) {
    QMMF_ERROR("%s Invalid session_id(%d)!", __func__, session_id);
    return -EINVAL;
  }
  auto& tracks = sessions_[session_id];

  if (!tracks.empty()) {
    QMMF_ERROR("%s: Delete tracks first before deleting Session(%d)",
        __func__, session_id);
    return -ENOSYS;
  }
  assert(client_id_ > 0);
  auto ret = recorder_service_->DeleteSession(client_id_, session_id);
  if (0 != ret) {
    QMMF_ERROR("%s DeleteSession failed!", __func__);
  }

  sessions_.erase(session_id);
  session_cb_list_.erase(session_id);
  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t RecorderClient::StartSession(const uint32_t session_id) {

  QMMF_DEBUG("%s Enter ", __func__);
  QMMF_KPI_BASE();
  std::lock_guard<std::mutex> lock(lock_);

  if (!CheckServiceStatus()) {
    return -ENODEV;
  }

  if (sessions_.count(session_id) == 0) {
    QMMF_ERROR("%s: Invalid session_id(%d)!", __func__, session_id);
    return -EINVAL;
  }
  auto& tracks = sessions_[session_id];

  assert(client_id_ > 0);
  auto ret = recorder_service_->StartSession(client_id_, session_id);
  if (0 != ret) {
    QMMF_ERROR("%s StartSession failed!", __func__);
  } else {
    for (auto const& track : tracks) {
      QMMF_KPI_ASYNC_BEGIN("FirstVidFrame", track);
    }
  }
  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t RecorderClient::StopSession(const uint32_t session_id,
                                     bool do_flush) {
  QMMF_DEBUG("%s Enter ", __func__);
  QMMF_KPI_BASE();
  std::lock_guard<std::mutex> lock(lock_);

  if (!CheckServiceStatus()) {
    return -ENODEV;
  }

  if (sessions_.count(session_id) == 0) {
    QMMF_ERROR("%s Invalid session_id(%d)!", __func__, session_id);
    return -EINVAL;
  }

  assert(client_id_ > 0);
  auto ret = recorder_service_->StopSession(client_id_, session_id, do_flush);
  if (0 != ret) {
    QMMF_ERROR("%s StopSession failed!", __func__);
  } else {
    auto& tracks = sessions_[session_id];
    for (auto const& track : tracks) {
      QMMF_KPI_ASYNC_BEGIN("LastVidFrame", track);
    }
  }
  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t RecorderClient::PauseSession(const uint32_t session_id) {

  QMMF_DEBUG("%s Enter ", __func__);
  QMMF_KPI_DETAIL();
  std::lock_guard<std::mutex> lock(lock_);

  if (!CheckServiceStatus()) {
    return -ENODEV;
  }

  if (sessions_.count(session_id) == 0) {
    QMMF_ERROR("%s Invalid session_id(%d)!", __func__, session_id);
    return -EINVAL;
  }

  assert(client_id_ > 0);
  auto ret = recorder_service_->PauseSession(client_id_, session_id);
  if (0 != ret) {
    QMMF_ERROR("%s PauseSession failed!", __func__);
  }
  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t RecorderClient::ResumeSession(const uint32_t session_id)
{
  QMMF_DEBUG("%s Enter ", __func__);
  QMMF_KPI_DETAIL();
  std::lock_guard<std::mutex> lock(lock_);

  if (!CheckServiceStatus()) {
    return -ENODEV;
  }

  if (sessions_.count(session_id) == 0) {
    QMMF_ERROR("%s Invalid session_id(%d)!", __func__, session_id);
    return -EINVAL;
  }

  assert(client_id_ > 0);
  auto ret = recorder_service_->ResumeSession(client_id_, session_id);
  if (0 != ret) {
    QMMF_ERROR("%s ResumeSession failed!", __func__);
  }
  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t RecorderClient::CreateVideoTrack(const uint32_t session_id,
                                          const uint32_t track_id,
                                          const VideoTrackParam& param,
                                          const VideoExtraParam& xtraparam,
                                          const TrackCb& cb) {

  QMMF_DEBUG("%s Enter ", __func__);
  QMMF_KPI_DETAIL();

  std::lock_guard<std::mutex> lock(lock_);
  if (!CheckServiceStatus()) {
    return -ENODEV;
  }
  assert(track_id != 0);

  if (sessions_.count(session_id) == 0) {
    QMMF_ERROR("%s: Invalid session_id(%d)!", __func__, session_id);
    return -EINVAL;
  }

  if (sessions_[session_id].count(track_id) != 0) {
    QMMF_ERROR("%s track_id(%d) already exists!", __func__, track_id);
    return -EINVAL;
  }

  assert(client_id_ > 0);
  auto ret = recorder_service_->CreateVideoTrack(client_id_, session_id,
                                                 track_id, param, xtraparam);
  if (0 != ret) {
    QMMF_ERROR("%s CreateVideoTrackWithExtraParam failed!", __func__);
  } else {
    UpdateSessionTopology(session_id, track_id, true /*add*/);
    std::lock_guard<std::mutex> l(track_cb_lock_);
    track_cb_list_.emplace(session_id, std::map<uint32_t, TrackCb>());
    track_cb_list_[session_id].emplace(track_id, cb);
  }

  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t RecorderClient::ReturnTrackBuffer(const uint32_t session_id,
                                           const uint32_t track_id,
                                           std::vector<BufferDescriptor>
                                           &buffers) {

  QMMF_DEBUG("%s Enter ", __func__);
  QMMF_VERBOSE("%s INPARAM: session_id[%u]", __func__, session_id);
  QMMF_VERBOSE("%s INPARAM: track_id[%u]", __func__, track_id);
  for (const BufferDescriptor& buffer : buffers)
    QMMF_VERBOSE("%s INPARAM: buffer[%s]", __func__,
                 buffer.ToString().c_str());
  if (!CheckServiceStatus()) {
    return -ENODEV;
  }

  QMMF_KPI_ASYNC_END("VideoAppCB", track_id);
  uint32_t ret = 0;
  std::vector<BnBuffer> bn_buffers;

  for (auto const& buffer : buffers) {
    BnBuffer bn_buffer = {
      static_cast<int32_t>(buffer.buf_id), // ion_fd
      -1,                                  // ion_meta_fd
      -1,                                  // img_id
      buffer.size,                         // size
      buffer.timestamp,                    // timestamp
      buffer.seqnum,                       // seqnum
      buffer.buf_id,                       // buffer_id
      buffer.flags,                        // flags
      buffer.capacity                      // capacity
    };
    bn_buffers.push_back(bn_buffer);
  }
  assert(client_id_ > 0);
  ret = recorder_service_->ReturnTrackBuffer(client_id_, session_id, track_id,
                                             bn_buffers);
  if (ret != 0) {
    QMMF_ERROR("%s ReturnTrackBuffer failed: %d", __func__, ret);
  }
  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t RecorderClient::SetVideoTrackParam(const uint32_t session_id,
                                            const uint32_t track_id,
                                            VideoParam type,
                                            const void *param,
                                            size_t size) {

  QMMF_DEBUG("%s Enter ", __func__);
  std::lock_guard<std::mutex> lock(lock_);
  if (!CheckServiceStatus()) {
    return -ENODEV;
  }

  if (sessions_.count(session_id) == 0) {
    QMMF_ERROR("%s: Invalid session_id(%d)!", __func__, session_id);
    return -EINVAL;
  }

  if (sessions_[session_id].count(track_id) == 0) {
    QMMF_ERROR("%s Invalid track_id(%d)!", __func__, track_id);
    return -EINVAL;
  }

  assert(client_id_ > 0);
  auto ret = recorder_service_->SetVideoTrackParam(client_id_, session_id,
      track_id, type, const_cast<void*>(param), size);
  if (0 != ret) {
    QMMF_ERROR("%s SetVideoTrackParam failed!", __func__);
  }
  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t RecorderClient::DeleteVideoTrack(const uint32_t session_id,
                                          const uint32_t track_id) {

  QMMF_DEBUG("%s Enter track_id(%d)", __func__, track_id);
  QMMF_KPI_DETAIL();

  std::lock_guard<std::mutex> lock(lock_);
  if (!CheckServiceStatus()) {
    return -ENODEV;
  }

  if (sessions_.count(session_id) == 0) {
    QMMF_ERROR("%s: Invalid session_id(%d)!", __func__, session_id);
    return -EINVAL;
  }

  if (sessions_[session_id].count(track_id) == 0) {
    QMMF_ERROR("%s Invalid track_id(%d)!", __func__, track_id);
    return -EINVAL;
  }

  {
    std::lock_guard<std::mutex> l(track_buffers_lock_);
    if (track_buffers_map_.count(track_id) != 0) {
      for (auto& pair : track_buffers_map_[track_id]) {
        auto& buffer_info = pair.second;

        QMMF_INFO("%s track_id(%d): BufInfo: ion_fd(%d), vaddr(%p), size(%lu)",
                  __func__, track_id, buffer_info.ion_fd, buffer_info.vaddr,
                  buffer_info.size);

        UnmapBuffer(buffer_info);
      }
      track_buffers_map_.erase(track_id);
    }
  }

  assert(client_id_ > 0);
  auto ret = recorder_service_->DeleteVideoTrack(client_id_, session_id,
      track_id);
  if (0 != ret) {
    QMMF_ERROR("%s track_id(%d) DeleteVideoTrack failed!", __func__, track_id);
  } else {
    UpdateSessionTopology(session_id, track_id, false /*remove*/);
    std::lock_guard<std::mutex> l(track_cb_lock_);
    track_cb_list_[session_id].erase(track_id);
  }

  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t RecorderClient::CaptureImage(const uint32_t camera_id,
                                      const SnapshotType type,
                                      const uint32_t n_images,
                                      const std::vector<CameraMetadata> &meta,
                                      const ImageCaptureCb &cb) {

  QMMF_DEBUG("%s Enter ", __func__);
  QMMF_KPI_ASYNC_BEGIN("FirstCapImg", camera_id);

  std::lock_guard<std::mutex> lock(lock_);
  if (!CheckServiceStatus()) {
    return -ENODEV;
  }
  assert(client_id_ > 0);
  auto ret = recorder_service_->CaptureImage(client_id_, camera_id, type,
                                             n_images, meta);
  if (0 != ret) {
    QMMF_ERROR("%s CaptureImage failed!", __func__);
  }
  image_capture_cb_ = cb;
  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t RecorderClient::ConfigImageCapture(const uint32_t camera_id,
                                            const uint32_t image_id,
                                            const ImageParam &param,
                                            const ImageExtraParam &xtraparam) {

  QMMF_DEBUG("%s Enter ", __func__);
  std::lock_guard<std::mutex> lock(lock_);
  if (!CheckServiceStatus()) {
    return -ENODEV;
  }
  assert(client_id_ > 0);
  auto ret = recorder_service_->ConfigImageCapture(client_id_, camera_id,
                                                   image_id, param, xtraparam);
  if (0 != ret) {
    QMMF_ERROR("%s ConfigImageCapture failed!", __func__);
  }
  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t RecorderClient::CancelCaptureImage(const uint32_t camera_id,
                                            const uint32_t image_id,
                                            const bool cache) {

  QMMF_DEBUG("%s Enter ", __func__);
  QMMF_KPI_DETAIL();
  std::lock_guard<std::mutex> lock(lock_);
  if (!CheckServiceStatus()) {
    return -ENODEV;
  }
  assert(client_id_ > 0);
  auto ret = recorder_service_->CancelCaptureImage(client_id_, camera_id,
                                                   image_id, cache);
  if(0 != ret) {
    QMMF_ERROR("%s CancelCaptureImage failed!", __func__);
  }
  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t RecorderClient::ReturnImageCaptureBuffer(const uint32_t camera_id,
                                                  const BufferDescriptor
                                                      &buffer) {

  QMMF_DEBUG("%s Enter ", __func__);
  if (!CheckServiceStatus()) {
    return -ENODEV;
  }

  {
    // Unmap buffer from client process.
    std::lock_guard<std::mutex> lock(snapshot_buffers_lock_);
    if (snapshot_buffers_.count(buffer.fd) == 0) {
      QMMF_ERROR("%s Invalid buffer fd(%d)!", __func__, buffer.fd);
      return -EINVAL;
    }
    auto buffer_info = snapshot_buffers_[buffer.fd];

    QMMF_INFO("%s Snapshot BufInfo: ion_fd(%d), vaddr(%p), size(%lu)", __func__,
              buffer_info.ion_fd, buffer_info.vaddr, buffer_info.size);

    UnmapBuffer(buffer_info);
    snapshot_buffers_.erase(buffer.fd);
  }

  QMMF_DEBUG("%s Returning buf_id(%d) back to service!", __func__,
      buffer.buf_id);
  assert(client_id_ > 0);
  auto ret = recorder_service_->ReturnImageCaptureBuffer(client_id_, camera_id,
                                                         buffer.buf_id);
  if (0 != ret) {
      QMMF_ERROR("%s ReturnImageCaptureBuffer failed!", __func__);
  }
  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t RecorderClient::SetCameraParam(const uint32_t camera_id,
                                        const CameraMetadata &meta) {

  QMMF_DEBUG("%s Enter ", __func__);
  std::lock_guard<std::mutex> lock(lock_);
  if (!CheckServiceStatus()) {
    return -ENODEV;
  }
  assert(client_id_ > 0);
  auto ret = recorder_service_->SetCameraParam(client_id_, camera_id, meta);
  if (0 != ret) {
    QMMF_ERROR("%s SetCameraParam failed!", __func__);
  }
  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t RecorderClient::GetCameraParam(const uint32_t camera_id,
                                        CameraMetadata &meta) {

  QMMF_DEBUG("%s Enter ", __func__);

  std::lock_guard<std::mutex> lock(lock_);
  if (!CheckServiceStatus()) {
    return -ENODEV;
  }
  assert(client_id_ > 0);
  auto ret = recorder_service_->GetCameraParam(client_id_, camera_id, meta);
  if (0 != ret) {
    QMMF_ERROR("%s GetCameraParam failed!", __func__);
  }
  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t RecorderClient::SetCameraSessionParam(const uint32_t camera_id,
                                               const CameraMetadata &meta) {

  QMMF_DEBUG("%s Enter ", __func__);
  std::lock_guard<std::mutex> lock(lock_);
  if (!CheckServiceStatus()) {
    return -ENODEV;
  }
  assert(client_id_ > 0);
  auto ret = recorder_service_->SetCameraSessionParam(client_id_, camera_id,
                                                      meta);
  if (0 != ret) {
    QMMF_ERROR("%s SetCameraSessionParam failed!", __func__);
  }
  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t RecorderClient::SetSHDR(const uint32_t camera_id,
                                 const bool enable) {

  QMMF_DEBUG("%s Enter ", __func__);

  std::lock_guard<std::mutex> lock(lock_);
  if (!CheckServiceStatus()) {
    return -ENODEV;
  }
  assert(client_id_ > 0);
  auto ret = recorder_service_->SetSHDR(client_id_, camera_id, enable);
  if (0 != ret) {
    QMMF_ERROR("%s SetSHDR failed!", __func__);
  }
  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t RecorderClient::GetDefaultCaptureParam(const uint32_t camera_id,
                                                CameraMetadata &meta) {

  QMMF_DEBUG("%s Enter ", __func__);
  std::lock_guard<std::mutex> lock(lock_);
  if (!CheckServiceStatus()) {
    return -ENODEV;
  }
  assert(client_id_ > 0);
  auto ret = recorder_service_->GetDefaultCaptureParam(client_id_,  camera_id,
                                                       meta);
  if (0 != ret) {
    QMMF_ERROR("%s GetDefaultCaptureParam failed!", __func__);
  }
  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t RecorderClient::GetCameraCharacteristics(const uint32_t camera_id,
                                                  CameraMetadata &meta) {

  QMMF_DEBUG("%s Enter ", __func__);
  std::lock_guard<std::mutex> lock(lock_);
  if (!CheckServiceStatus()) {
    return -ENODEV;
  }
  assert(client_id_ > 0);
  auto ret = recorder_service_->GetCameraCharacteristics(client_id_,  camera_id,
                                                         meta);
  if (0 != ret) {
    QMMF_ERROR("%s GetCameraCharacteristics failed!", __func__);
  }
  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t RecorderClient::GetVendorTagDescriptor(std::shared_ptr<VendorTagDescriptor> &desc) {

  QMMF_DEBUG("%s Enter ", __func__);

  if (!CheckServiceStatus()) {
    return -ENODEV;
  }
  assert(client_id_ > 0);
  auto ret = recorder_service_->GetVendorTagDescriptor(desc);
  if (0 != ret) {
    QMMF_ERROR("%s GetVendorTagDescriptor failed!", __func__);
  }
  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t RecorderClient::CreateOfflineJPEG(
                          const OfflineJpegCreateParams &params,
                          const OfflineJpegCb &cb) {

  QMMF_DEBUG("%s Enter ", __func__);
  std::lock_guard<std::mutex> lock(lock_);
  if (!CheckServiceStatus()) {
    return -ENODEV;
  }
  assert(client_id_ > 0);

  if (nullptr == cb) {
    QMMF_ERROR("%s: Error. Client callback is null.", __func__);
    return -EINVAL;
  }
  auto ret = recorder_service_->CreateOfflineJPEG(client_id_, params);
  if (0 != ret) {
    QMMF_ERROR("%s CreateOfflineJPEG failed!", __func__);
  }
  offline_jpeg_cb_ = cb;
  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t RecorderClient::EncodeOfflineJPEG(
                            const OfflineJpegProcessParams &params) {

  QMMF_DEBUG("%s Enter ", __func__);
  std::lock_guard<std::mutex> lock(lock_);
  if (!CheckServiceStatus()) {
    return -ENODEV;
  }
  assert(client_id_ > 0);

  BnBuffer in_buf = {};
  BnBuffer out_buf = {};
  in_buf.ion_fd = out_buf.ion_fd = -1;

  if (!IsJpegBufPresent(params.in_buf_fd)) {
    in_buf.ion_fd = params.in_buf_fd;
    offline_jpeg_buffers_.push_back(params.in_buf_fd);
  }
  in_buf.buffer_id = params.in_buf_fd;

  if (!IsJpegBufPresent(params.out_buf_fd)) {
    out_buf.ion_fd = params.out_buf_fd;
    offline_jpeg_buffers_.push_back(params.out_buf_fd);
  }
  out_buf.buffer_id = params.out_buf_fd;

  auto ret = recorder_service_->EncodeOfflineJPEG(client_id_,
                                                  in_buf,
                                                  out_buf,
                                                  params.metadata);
  if (0 != ret) {
    QMMF_ERROR("%s EncodeOfflineJPEG failed!", __func__);
  }
  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

status_t RecorderClient::DestroyOfflineJPEG() {

  QMMF_DEBUG("%s Enter ", __func__);
  std::lock_guard<std::mutex> lock(lock_);
  if (!CheckServiceStatus()) {
    return -ENODEV;
  }
  assert(client_id_ > 0);
  offline_jpeg_buffers_.clear();
  auto ret = recorder_service_->DestroyOfflineJPEG(client_id_);
  if (0 != ret) {
    QMMF_ERROR("%s DestroyOfflineJPEG failed!", __func__);
  }
  QMMF_DEBUG("%s Exit ", __func__);
  return ret;
}

bool RecorderClient::IsJpegBufPresent(const int32_t& buf_fd) {
  bool found = false;
  for ( auto fd : offline_jpeg_buffers_) {
    if (buf_fd == fd) {
      found = true;
      break;
    }
  }

  return found;
}

#ifdef TARGET_USES_GBM
void RecorderClient::ImportBuffer(int32_t fd, int32_t metafd,
                                  const BufferMeta& meta) {

  std::lock_guard<std::mutex> lock(gbm_lock_);
  if (gbm_buffers_map_.count(fd) != 0) {
    // Already imported fd and metafd.
    return;
  } else if (metafd == -1) {
    // The metadata FD is missing, do not import.
    return;
  }

  uint32_t width = meta.planes[0].width;
  uint32_t height = meta.planes[0].height;
  uint32_t format = 0;

  switch (meta.format) {
    case BufferFormat::kRAW10:
      format = GBM_FORMAT_RAW10;
      break;
    case BufferFormat::kRAW12:
      format = GBM_FORMAT_RAW12;
      break;
    case BufferFormat::kRAW16:
      format = GBM_FORMAT_RAW16;
      break;
    case BufferFormat::kNV12:
      format = GBM_FORMAT_NV12;
      break;
    case BufferFormat::kNV21:
      format = GBM_FORMAT_NV21_ZSL;
      break;
    case BufferFormat::kNV16:
      format = GBM_FORMAT_NV16;
      break;
    case BufferFormat::kBLOB:
      format = GBM_FORMAT_BLOB;
      width  = meta.planes[0].size;
      height = 1;
      break;
    case BufferFormat::kNV12UBWC:
      format = GBM_FORMAT_YCbCr_420_SP_VENUS_UBWC;
      break;
    case BufferFormat::kP010:
      format = GBM_FORMAT_YCbCr_420_P010_VENUS;
      break;
    case BufferFormat::kTP10UBWC:
      format = GBM_FORMAT_YCbCr_420_TP10_UBWC;
      break;
    case BufferFormat::kYUY2:
      format = GBM_FORMAT_YCrCb_422_I;
      break;
    case BufferFormat::kUYVY:
      format = GBM_FORMAT_UYVY;
      break;
    case BufferFormat::kNV12HEIF:
      format = GBM_FORMAT_NV12_HEIF;
      break;
    default:
      format = 0;
  }

  gbm_buf_info bufinfo = { fd, metafd, width , height, format };

  auto bo = gbm_bo_import(gbm_device_, GBM_BO_IMPORT_GBM_BUF_TYPE, &bufinfo, 0);
  if (bo == nullptr) {
    QMMF_WARN("%s: gbm bo import failed", __func__);
    return;
  }

  gbm_buffers_map_.emplace(fd, bo);
}

void RecorderClient::ReleaseBuffer(int32_t& fd, int32_t& meta_fd) {

  std::lock_guard<std::mutex> lock(gbm_lock_);
  if (gbm_buffers_map_.count(fd) == 0) {
    QMMF_WARN("%s Buffer is already released or never imported", __func__);
    return;
  }

  gbm_bo_destroy(gbm_buffers_map_[fd]);
  gbm_buffers_map_.erase(fd);

  uint32_t duplicated = 0;

#ifdef GBM_PERFORM_GET_FD_WITH_NEW
  gbm_perform(GBM_PERFORM_GET_FD_WITH_NEW, &duplicated);
#endif // GBM_PERFORM_GET_FD_WITH_NEW

  if (duplicated) {
    // FD and meta FD are opened by Binder and we have to closed them ideally.
    // But previous implementation of gbm_bo_destroy close even if FD is open by
    // someone else and FD is imported in GBM. This issud is fixed in newer
    // implementation of GBM and we have to close both FD and Meta FD to avoid
    // leak. Duplicated Flag show which version of GBM is used. If flag is not
    // set GBM exposes original FD and gbm_bo_destroy free original and
    // imported FD automatically.
    close(fd);
    close(meta_fd);
  }

  fd = -1;
  meta_fd = -1;
}
#endif // TARGET_USES_GBM

status_t RecorderClient::MapBuffer(BufferInfo& info, const BufferMeta& meta) {

  QMMF_DEBUG("%s Enter ", __func__);
  void* vaddr = nullptr;
  assert(ion_device_ > 0);

  vaddr = mmap(NULL, info.size, PROT_READ | PROT_WRITE, MAP_SHARED,
               info.ion_fd, 0);
  if (nullptr == vaddr) {
    QMMF_ERROR("%s Failed to map ion_fd %d : %d[%s]!", __func__, info.ion_fd,
               -errno, strerror(errno));
    return -ENOMEM;
  }

#if TARGET_ION_ABI_VERSION >= 2
  struct dma_buf_sync sync;
  sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_RW;

  if (ioctl(info.ion_fd, DMA_BUF_IOCTL_SYNC, &sync) != 0) {
    ALOGE("%s: DMA SYNC START failed!", __func__);
  }
#endif
  info.vaddr = vaddr;

#ifdef TARGET_USES_GBM
  ImportBuffer(info.ion_fd, info.ion_meta_fd, meta);
#endif

  QMMF_DEBUG("%s Exit ", __func__);
  return 0;
}

void RecorderClient::UnmapBuffer(BufferInfo& info) {

  QMMF_DEBUG("%s Enter ", __func__);
  assert(ion_device_ > 0);

  if (info.vaddr == nullptr) {
    QMMF_WARN("%s Buffer is not mapped", __func__);
    return;
  }

#if TARGET_ION_ABI_VERSION >= 2
  struct dma_buf_sync sync;
  sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_RW;

  if (ioctl(info.ion_fd, DMA_BUF_IOCTL_SYNC, &sync) != 0) {
    ALOGE("%s: DMA SYNC END failed!", __func__);
  }
#endif

  if (munmap(info.vaddr, info.size) < 0) {
    QMMF_ERROR("%s() unable to unmap buffer[%d]: %d[%s]", __func__,
                info.ion_fd, errno, strerror(errno));
    return;
  }
  info.vaddr = nullptr;

#ifdef TARGET_USES_GBM
  ReleaseBuffer(info.ion_fd, info.ion_meta_fd);
#endif

  QMMF_DEBUG("%s Exit ", __func__);
  return;
}

bool RecorderClient::CheckServiceStatus() {

  if (nullptr == recorder_service_.get()) {
    QMMF_WARN("%s Not connected to Recorder service!", __func__);
    return false;
  }
  return true;
}

void RecorderClient::UpdateSessionTopology(const uint32_t& session_id,
                                           const uint32_t& track_id, bool add) {
  QMMF_DEBUG("%s Enter ", __func__);

  auto& tracks = sessions_[session_id];
  if (!add) {
    tracks.erase(track_id);
  } else {
    tracks.emplace(track_id);
  }

  for (auto const& track : tracks) {
    QMMF_INFO("%s session_id(%d): track_id(%d)", __func__, session_id, track);
  }
  QMMF_DEBUG("%s Exit ", __func__);
}

void RecorderClient::ServiceDeathHandler() {
  QMMF_INFO("%s Enter ", __func__);

  std::lock_guard<std::mutex> lock(lock_);
  //Clear all pending buffers.
  {
    std::lock_guard<std::mutex> l(track_buffers_lock_);
    for (auto& iter : track_buffers_map_) {
      uint32_t track_id = iter.first;
      BufferInfoMap& info_map = iter.second;

      for (auto& it : info_map) {
        BufferInfo& buffer_info = it.second;

        QMMF_INFO("%s track_id(%d): BufInfo: ion_fd(%d), vaddr(%p), size(%lu)",
                  __func__, track_id, buffer_info.ion_fd,
                  buffer_info.vaddr, buffer_info.size);

        UnmapBuffer(buffer_info);
      }
    }
    track_buffers_map_.clear();
  }

  {
    std::lock_guard<std::mutex> l(snapshot_buffers_lock_);
    for (auto& it : snapshot_buffers_) {
      auto& buffer_info = it.second;

      QMMF_INFO("%s Snapshot BufInfo: ion_fd(%d), vaddr(%p), size(%lu)",
                __func__, buffer_info.ion_fd,
                buffer_info.vaddr, buffer_info.size);

      UnmapBuffer(buffer_info);
    }
    snapshot_buffers_.clear();
  }

#if HAVE_BINDER
  recorder_service_->asBinder(recorder_service_)->
      unlinkToDeath(death_notifier_);
#endif // HAVE_BINDER

  recorder_service_.reset();

  death_notifier_.reset();

  sessions_.clear();
  session_cb_list_.clear();
  track_cb_list_.clear();

  image_capture_cb_ = nullptr;
  metadata_cb_ = nullptr;
  offline_jpeg_cb_ = nullptr;

  if (ion_device_ > 0) {
    close(ion_device_);
    ion_device_ = -1;
  }
  client_id_ = 0;

  if (recorder_cb_.event_cb != nullptr) {
    recorder_cb_.event_cb(EventType::kServerDied, nullptr, 0);
  }
  QMMF_INFO("%s Exit ", __func__);
}

#ifndef HAVE_BINDER
void RecorderClient::NotifyServerDeath() {
  std::unique_lock<std::mutex> lock(disconnect_lock_);
  std::chrono::nanoseconds wait_time(kWaitDelay);
  // Waiting for 2 secs to see if the socket gracefully closed during Disconnect
  auto ret = wait_for_disconnect_.WaitFor(lock, wait_time);
  if (ret != 0) {
    QMMF_ERROR("%s: Wait for disconnect timed out! Server has died!", __func__);
    if (nullptr == death_notifier_.get()) {
      death_notifier_->ServerDied();
    }
  } else {
    QMMF_INFO("%s: Server is alive, nop!", __func__);
  }
}
#endif // !HAVE_BINDER

void RecorderClient::NotifyRecorderEvent(EventType event, void *payload,
                                         size_t size) {

  QMMF_DEBUG("%s Enter ", __func__);
  bool notify = true;

  if (REMAP_ALL_BUFFERS == static_cast<uint32_t>(event)) {
    for (auto& it : track_buffers_map_) {
      auto& track_id = it.first;
      BufferInfoMap& info_map = it.second;

      for (auto& pair : info_map) {
        UnmapBuffer(pair.second);
      }

      track_buffers_map_.erase(track_id);
    }
    // This is a internal event, won't be transmitted to client.
    notify = false;
  }

  if (notify && (recorder_cb_.event_cb != nullptr)) {
    recorder_cb_.event_cb(event, payload, size);
  }
  QMMF_DEBUG("%s Exit ", __func__);
}

void RecorderClient::NotifySessionEvent(EventType event_type, void *event_data,
                                        size_t event_data_size) {
    QMMF_DEBUG("%s Enter ", __func__);
    QMMF_DEBUG("%s Exit ", __func__);
}

void RecorderClient::NotifySnapshotData(uint32_t camera_id, uint32_t imgcount,
                                        BnBuffer& bn_buffer, BufferMeta& meta) {

  QMMF_DEBUG("%s Enter ", __func__);

  assert(image_capture_cb_ != nullptr);
  assert(bn_buffer.ion_fd > 0);
  assert(bn_buffer.buffer_id > 0);

  BufferInfo buffer_info {};
  buffer_info.ion_fd      = bn_buffer.ion_fd;
  buffer_info.ion_meta_fd = bn_buffer.ion_meta_fd;
  buffer_info.size        = bn_buffer.capacity;

  auto ret = MapBuffer(buffer_info, meta);
  if (0 != ret) {
    QMMF_ERROR("%s Failed to map buffer!", __func__);
    return;
  }
  {
    std::lock_guard<std::mutex> lock(snapshot_buffers_lock_);
    snapshot_buffers_.emplace(bn_buffer.ion_fd, buffer_info);
  }

  BufferDescriptor buffer {};
  buffer.img_id    = bn_buffer.img_id;
  buffer.data      = buffer_info.vaddr;
  buffer.size      = bn_buffer.size;
  buffer.timestamp = bn_buffer.timestamp;
  buffer.seqnum    = bn_buffer.seqnum;
  buffer.flags     = bn_buffer.flags;
  buffer.capacity  = bn_buffer.capacity;
  buffer.buf_id    = bn_buffer.buffer_id;
  buffer.fd        = bn_buffer.ion_fd;

  if (imgcount == 0) {
    QMMF_KPI_ASYNC_END("FirstCapImg", camera_id);
  } else {
    QMMF_KPI_ASYNC_END("SnapShot-Shot", camera_id);
  }

  QMMF_KPI_ASYNC_BEGIN("SnapShot-Shot", camera_id);

  image_capture_cb_(camera_id, imgcount, buffer, meta);
  QMMF_DEBUG("%s Exit ", __func__);
}

void RecorderClient::NotifyOfflineJpegData(int32_t buf_fd,
                                           uint32_t encoded_size) {
  QMMF_DEBUG("%s Enter ", __func__);
  assert(offline_jpeg_cb_ != nullptr);
  offline_jpeg_cb_(buf_fd, encoded_size);
  QMMF_DEBUG("%s Exit ", __func__);
}

void RecorderClient::NotifyVideoTrackData(uint32_t session_id,
                                          uint32_t track_id,
                                          std::vector<BnBuffer> &bn_buffers,
                                          std::vector<BufferMeta> &metas) {

  QMMF_DEBUG("%s Enter track_id=%d", __func__, track_id);

  std::vector<BufferDescriptor> track_buffers;
  for (uint32_t idx = 0; idx < bn_buffers.size(); ++idx) {
    BnBuffer& bn_buffer = bn_buffers[idx];

    bool is_mapped = false;
    BufferInfo buffer_info {};

    // Check if ION buffer is already imported and mapped, if it is then get
    // buffer info from map.
    {
      std::lock_guard<std::mutex> l(track_buffers_lock_);
      if (track_buffers_map_.count(track_id) != 0) {
        auto& info_map = track_buffers_map_[track_id];

        if (info_map.count(bn_buffer.buffer_id) != 0) {
          buffer_info = info_map[bn_buffer.buffer_id];

          bn_buffer.ion_fd = buffer_info.ion_fd;
          bn_buffer.ion_meta_fd = buffer_info.ion_meta_fd;
          is_mapped = true;

          QMMF_VERBOSE("%s Buffer is already mapped! buffer_id(%d):ion_fd(%d):"
              "vaddr(%p)",  __func__, bn_buffer.buffer_id,
              buffer_info.ion_fd, buffer_info.vaddr);
        }
      } else {
        QMMF_KPI_ASYNC_END("FirstVidFrame", track_id);
      }
    }
    if (!is_mapped) {
      buffer_info.ion_fd      = bn_buffer.ion_fd;
      buffer_info.ion_meta_fd = bn_buffer.ion_meta_fd;
      buffer_info.size        = bn_buffer.capacity;

      auto ret = MapBuffer(buffer_info, metas[idx]);
      if (0 != ret) {
        QMMF_ERROR("%s Failed to map buffer!", __func__);
        return;
      }

      QMMF_VERBOSE("%s track_id(%d): BufInfo: ion_fd(%d), "
          "vaddr(%p), size(%lu)", __func__, track_id, buffer_info.ion_fd,
           buffer_info.vaddr, buffer_info.size);

      // Update existing entry or add new one.
      std::lock_guard<std::mutex> l(track_buffers_lock_);
      BufferInfoMap& buffer_info_map = track_buffers_map_[track_id];
      buffer_info_map.emplace(bn_buffer.buffer_id, buffer_info);

      QMMF_VERBOSE("%s track_buffers_map_.size = %d", __func__,
          track_buffers_map_.size());

      for (auto const& iter : track_buffers_map_) {
        QMMF_VERBOSE("%s track_id(%d): BufInfoMap size = %d", __func__,
            iter.first, iter.second.size());

        for (auto const& it : iter.second) {
          QMMF_VERBOSE("%s BufInfo: key(%d), ion_fd(%d), vaddr(%p)",
              __func__, it.first, it.second.ion_fd, it.second.vaddr);
        }
      }
    }

    BufferDescriptor buffer {};
    buffer.data      = buffer_info.vaddr;
    buffer.size      = bn_buffer.size;
    buffer.timestamp = bn_buffer.timestamp;
    buffer.seqnum    = bn_buffer.seqnum;
    buffer.flags     = bn_buffer.flags;
    buffer.buf_id    = bn_buffer.buffer_id;
    buffer.capacity  = bn_buffer.capacity;
    buffer.fd        = buffer_info.ion_fd;
    track_buffers.push_back(buffer);
  }
  QMMF_DEBUG("%s Buffer Prepared for Callback track_id=%d", __func__, track_id);

  // Get the handle to track callbacks.
  std::unique_lock<std::mutex> l(track_cb_lock_);
  if (track_cb_list_.count(session_id) != 0 &&
      track_cb_list_[session_id].count(track_id) != 0) {
    TrackCb callbacks = track_cb_list_[session_id][track_id];
    l.unlock();

    QMMF_KPI_ASYNC_BEGIN("VideoAppCB", track_id);
    callbacks.data_cb(track_id, track_buffers, metas);
  } else {
    QMMF_ERROR("%s Track(%u) has not callback!", __func__, track_id);
  }
  QMMF_DEBUG("%s Exit ", __func__);
}

void RecorderClient::NotifyVideoTrackEvent(uint32_t session_id,
                                           uint32_t track_id,
                                           EventType event_type,
                                           void *event_data,
                                           size_t event_data_size) {
  QMMF_DEBUG("%s Enter ", __func__);
  QMMF_VERBOSE("%s Track(%u): Received event type = %d", __func__, track_id,
    (int32_t) event_type);

  // Get the handle to track callbacks.
  std::unique_lock<std::mutex> l(track_cb_lock_);
  if (track_cb_list_.count(session_id) != 0 &&
      track_cb_list_[session_id].count(track_id) != 0) {
    TrackCb callbacks = track_cb_list_[session_id][track_id];
    l.unlock();

    callbacks.event_cb(track_id, event_type, event_data, event_data_size);
  } else {
    QMMF_ERROR("%s Track(%u) has not callback!", __func__, track_id);
  }
  QMMF_DEBUG("%s Exit ", __func__);
}

void RecorderClient::NotifyCameraResult(uint32_t camera_id,
                                        const CameraMetadata &result) {
  if (nullptr != metadata_cb_) {
    metadata_cb_(camera_id, result);
  } else {
    QMMF_ERROR("%s No client registered result callback!\n", __func__);
  }
}

#ifdef HAVE_BINDER
//Binder Proxy implementation of IRecoderService.
class RecorderServiceProxy: public BpInterface<IRecorderService> {
 public:
  RecorderServiceProxy(const sp<IBinder>& impl)
  : BpInterface<IRecorderService>(impl) {}

  status_t Connect(const sp<IRecorderServiceCallback>& service_cb,
                   uint32_t* client_id) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    //Register service callback to get callbacks from recorder service.
    //eg : JPEG buffer, Tracks elementry buffers, Recorder/Session status
    //callbacks etc.
    data.writeStrongBinder(IInterface::asBinder(service_cb));
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                            RECORDER_CONNECT), data, &reply);
    *client_id = reply.readUint32();
    return reply.readInt32();
  }

  status_t Disconnect(const uint32_t client_id) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(client_id);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                            RECORDER_DISCONNECT), data, &reply);
    return reply.readInt32();
  }

  status_t StartCamera(const uint32_t client_id, const uint32_t camera_id,
                       const float framerate,
                       const CameraExtraParam& extra_param,
                       bool enable_result_cb) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(client_id);
    data.writeUint32(camera_id);
    data.writeFloat(framerate);
    data.writeUint32(enable_result_cb ? 1 : 0);
    uint32_t extra_param_size = extra_param.Size();
    data.writeUint32(extra_param_size);
    const void *extra_data = extra_param.GetAndLock();
    android::Parcel::WritableBlob extra_blob;
    data.writeBlob(extra_param_size, false, &extra_blob);
    memset(extra_blob.data(), 0x0, extra_param_size);
    memcpy(extra_blob.data(), extra_data, extra_param_size);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                            RECORDER_START_CAMERA), data, &reply);
    extra_param.ReturnAndUnlock(extra_data);
    extra_blob.release();
    return reply.readInt32();
  }

  status_t StopCamera(const uint32_t client_id, const uint32_t camera_id) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(client_id);
    data.writeUint32(camera_id);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                            RECORDER_STOP_CAMERA), data, &reply);
    return reply.readInt32();
  }

  status_t CreateSession(const uint32_t client_id, uint32_t *session_id) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(client_id);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                            RECORDER_CREATE_SESSION), data, &reply);
    uint32_t id;
    reply.readUint32(&id);
    *session_id = id;
    return reply.readInt32();
  }

  status_t DeleteSession(const uint32_t client_id, const uint32_t session_id) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(client_id);
    assert(session_id != 0);
    data.writeUint32(session_id);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                            RECORDER_DELETE_SESSION), data, &reply);
    return reply.readInt32();
  }

  status_t StartSession(const uint32_t client_id, const uint32_t session_id) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(client_id);
    assert(session_id != 0);
    data.writeUint32(session_id);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
        RECORDER_START_SESSION), data, &reply);
    return reply.readInt32();
  }

  status_t StopSession(const uint32_t client_id, const uint32_t session_id,
                       bool do_flush) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(client_id);
    assert(session_id != 0);
    data.writeUint32(session_id);
    data.writeInt32(do_flush);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
        RECORDER_STOP_SESSION), data, &reply);
    return reply.readInt32();
  }

  status_t PauseSession(const uint32_t client_id, const uint32_t session_id) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(client_id);
    assert(session_id != 0);
    data.writeUint32(session_id);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                            RECORDER_PAUSE_SESSION), data, &reply);
    return reply.readInt32();
  }

  status_t ResumeSession(const uint32_t client_id, const uint32_t session_id) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(client_id);
    assert(session_id != 0);
    data.writeUint32(session_id);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                            RECORDER_RESUME_SESSION), data, &reply);
    return reply.readInt32();
  }

  status_t CreateVideoTrack(const uint32_t client_id,
                            const uint32_t session_id,
                            const uint32_t track_id,
                            const VideoTrackParam& params,
                            const VideoExtraParam& xtraparam) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(client_id);
    data.writeUint32(session_id);
    data.writeUint32(track_id);
    uint32_t param_size = sizeof params;
    data.writeUint32(param_size);
    android::Parcel::WritableBlob blob;
    data.writeBlob(param_size, false, &blob);
    memset(blob.data(), 0x0, param_size);
    memcpy(blob.data(), &params, param_size);
    uint32_t xtraparam_size = xtraparam.Size();
    data.writeUint32(xtraparam_size);
    const void *extra_data = xtraparam.GetAndLock();
    android::Parcel::WritableBlob extra_blob;
    data.writeBlob(xtraparam_size, false, &extra_blob);
    memset(extra_blob.data(), 0x0, xtraparam_size);
    memcpy(extra_blob.data(), extra_data, xtraparam_size);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
        RECORDER_CREATE_VIDEOTRACK), data, &reply);
    xtraparam.ReturnAndUnlock(extra_data);
    extra_blob.release();
    blob.release();
    return reply.readInt32();
  }

status_t DeleteVideoTrack(const uint32_t client_id,
                          const uint32_t session_id,
                          const uint32_t track_id) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(client_id);
    data.writeUint32(session_id);
    data.writeUint32(track_id);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                      RECORDER_DELETE_VIDEOTRACK), data, &reply);
    return reply.readInt32();
  }

  status_t ReturnTrackBuffer(const uint32_t client_id,
                             const uint32_t session_id,
                             const uint32_t track_id,
                             std::vector<BnBuffer> &buffers) {

    QMMF_DEBUG("%s Enter", __func__);
    QMMF_VERBOSE("%s INPARAM: session_id[%u]", __func__, session_id);
    QMMF_VERBOSE("%s INPARAM: track_id[%u]", __func__, track_id);
    for (const BnBuffer& buffer : buffers) {
      QMMF_VERBOSE("%s INPARAM: buffers[%s]", __func__,
          buffer.ToString().c_str());
    }

    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(client_id);
    data.writeUint32(session_id);
    data.writeUint32(track_id);
    if (track_id < 100) {
      uint32_t size = buffers.size();
      assert(size > 0);
      data.writeUint32(size);
      // TODO: combine all BnBuffers together in single blob
      for (uint32_t i = 0; i < size; i++) {
        uint32_t param_size = sizeof (BnBuffer);
        data.writeUint32(param_size);
        android::Parcel::WritableBlob blob;
        data.writeBlob(param_size, false, &blob);
        memset(blob.data(), 0x0, param_size);
        buffers[i].ion_fd = buffers[i].ion_fd;
        buffers[i].ion_meta_fd = buffers[i].ion_meta_fd;
        memcpy(blob.data(), reinterpret_cast<void*>(&buffers[i]), param_size);
      }
    } else {
      data.writeInt32(static_cast<int32_t>(buffers.size()));
      for (const BnBuffer& buffer : buffers) {
        buffer.ToParcel(&data, false);
      }
    }

    remote()->transact(
        uint32_t(QMMF_RECORDER_SERVICE_CMDS::RECORDER_RETURN_TRACKBUFFER),
        data, &reply, IBinder::FLAG_ONEWAY);

    return 0;
  }

  status_t SetVideoTrackParam(const uint32_t client_id,
                              const uint32_t session_id,
                              const uint32_t track_id,
                              VideoParam type,
                              void *param, size_t size) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(client_id);
    data.writeUint32(session_id);
    data.writeUint32(track_id);
    data.writeUint32(static_cast<uint32_t>(type));
    data.writeUint32(size);
    android::Parcel::WritableBlob blob;
    data.writeBlob(size, false, &blob);
    memcpy(blob.data(), reinterpret_cast<void*>(param), size);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                            RECORDER_SET_VIDEOTRACK_PARAMS), data, &reply);
    blob.release();
    return reply.readInt32();
  }

  status_t CaptureImage(const uint32_t client_id, const uint32_t camera_id,
                        const SnapshotType type, const uint32_t n_images,
                        const std::vector<CameraMetadata> &meta) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(client_id);
    data.writeUint32(camera_id);
    data.writeUint32(static_cast<uint32_t>(type));
    data.writeUint32(n_images);
    data.writeUint32(meta.size());
    for (uint8_t i = 0; i < meta.size(); ++i) {
      meta[i].writeToParcel(&data);
    }
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
        RECORDER_CAPTURE_IMAGE), data, &reply);
    return reply.readInt32();
  }

  status_t ConfigImageCapture(const uint32_t client_id,
                              const uint32_t camera_id,
                              const uint32_t image_id,
                              const ImageParam &param,
                              const ImageExtraParam &xtraparam) {

    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(client_id);
    data.writeUint32(camera_id);
    data.writeUint32(image_id);
    uint32_t param_size = sizeof param;
    data.writeUint32(param_size);
    android::Parcel::WritableBlob blob;
    data.writeBlob(param_size, false, &blob);
    memcpy(blob.data(), &param, param_size);
    param_size = xtraparam.Size();
    data.writeUint32(param_size);
    const void *config_data = xtraparam.GetAndLock();
    android::Parcel::WritableBlob config_blob;
    data.writeBlob(param_size, false, &config_blob);
    memcpy(config_blob.data(), config_data, param_size);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
        RECORDER_CONFIG_IMAGECAPTURE), data, &reply);
    xtraparam.ReturnAndUnlock(config_data);
    blob.release();
    config_blob.release();
    return reply.readInt32();
  }

  status_t CancelCaptureImage(const uint32_t client_id,
                              const uint32_t camera_id,
                              const uint32_t image_id,
                              const bool cache) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(client_id);
    data.writeUint32(camera_id);
    data.writeUint32(image_id);
    data.writeUint32(cache);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                       RECORDER_CANCEL_IMAGECAPTURE), data, &reply);
    return reply.readInt32();
  }

  status_t ReturnImageCaptureBuffer(const uint32_t client_id,
                                    const uint32_t camera_id,
                                    const int32_t buffer_id) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(client_id);
    data.writeUint32(camera_id);
    data.writeUint32(buffer_id);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                       RECORDER_RETURN_IMAGECAPTURE_BUFFER), data, &reply);
    return reply.readInt32();
  }

  status_t SetCameraParam(const uint32_t client_id,
                          const uint32_t camera_id,
                          const CameraMetadata &meta) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(client_id);
    data.writeUint32(camera_id);
    meta.writeToParcel(&data);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                       RECORDER_SET_CAMERA_PARAMS), data, &reply);
    return reply.readInt32();
  }

  status_t GetCameraParam(const uint32_t client_id,
                          const uint32_t camera_id,
                          CameraMetadata &meta) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(client_id);
    data.writeUint32(camera_id);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                                RECORDER_GET_CAMERA_PARAMS), data, &reply);
    auto ret = reply.readInt32();
    if (0 == ret) {
      ret = meta.readFromParcel(&reply);
    }
    return ret;
  }

  status_t SetCameraSessionParam(const uint32_t client_id,
                                 const uint32_t camera_id,
                                 const CameraMetadata &meta) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(client_id);
    data.writeUint32(camera_id);
    meta.writeToParcel(&data);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                       RECORDER_SET_CAMERA_SESSION_PARAMS), data, &reply);
    return reply.readInt32();
  }

  status_t SetSHDR(const uint32_t client_id,
                   const uint32_t camera_id,
                   const bool enable) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(client_id);
    data.writeUint32(camera_id);
    data.writeInt32(enable);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
        RECORDER_SET_SHDR), data, &reply);
    return reply.readInt32();
  }

  status_t GetDefaultCaptureParam(const uint32_t client_id,
                                  const uint32_t camera_id,
                                  CameraMetadata &meta) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(client_id);
    data.writeUint32(camera_id);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                                RECORDER_GET_DEFAULT_CAPTURE_PARAMS), data,
                                &reply);
    auto ret = reply.readInt32();
    if (0 == ret) {
      ret = meta.readFromParcel(&reply);
    }
    return ret;
  }

  status_t GetCameraCharacteristics(const uint32_t client_id,
                                    const uint32_t camera_id,
                                    CameraMetadata &meta) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(client_id);
    data.writeUint32(camera_id);
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                                RECORDER_GET_CAMERA_CHARACTERISTICS), data,
                                &reply);
    auto ret = reply.readInt32();
    if (0 == ret) {
      ret = meta.readFromParcel(&reply);
    }
    return ret;
  }

  status_t GetVendorTagDescriptor(std::shared_ptr<VendorTagDescriptor> &desc) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
                                RECORDER_GET_VENDOR_TAG_DESCRIPTOR), data, &reply);
    auto ret = reply.readInt32();
    if (0 == ret) {
      ret = desc->readFromParcel(&reply);
    }
    return ret;
  }

  status_t CreateOfflineJPEG(const uint32_t client_id,
                             const OfflineJpegCreateParams &params) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(client_id);

    uint32_t param_size = sizeof (params);
    data.writeUint32(param_size);
    android::Parcel::WritableBlob blob;
    data.writeBlob(param_size, false, &blob);
    memcpy(blob.data(), &params, param_size);

    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
        RECORDER_CONFIGURE_OFFLINE_JPEG), data, &reply);
    return reply.readInt32();
  }

  status_t EncodeOfflineJPEG(const uint32_t client_id,
                             const BnBuffer& in_buf,
                             const BnBuffer& out_buf,
                             const OfflineJpegMeta& meta) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(client_id);

    // Input buffer
    bool present = (-1 == in_buf.ion_fd) ? true : false;
    data.writeInt32(present);
    if (!present) {
      data.writeFileDescriptor(in_buf.ion_fd);
    }
    data.writeInt32(in_buf.buffer_id);

    // Output buffer
    present = (-1 == out_buf.ion_fd) ? true : false;
    data.writeInt32(present);
    if (!present) {
      data.writeFileDescriptor(out_buf.ion_fd);
    }
    data.writeInt32(out_buf.buffer_id);

    uint32_t meta_size = sizeof (meta);
    data.writeUint32(meta_size);
    android::Parcel::WritableBlob blob;
    data.writeBlob(meta_size, false, &blob);
    memcpy(blob.data(), &meta, meta_size);

    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
        RECORDER_ENCODE_OFFLINE_JPEG), data, &reply);
    return reply.readInt32();
  }

  status_t DestroyOfflineJPEG(const uint32_t client_id) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderService::getInterfaceDescriptor());
    data.writeUint32(client_id);

    remote()->transact(uint32_t(QMMF_RECORDER_SERVICE_CMDS::
        RECORDER_DESTROY_OFFLINE_JPEG), data, &reply);
    return reply.readInt32();
  }
};

IMPLEMENT_META_INTERFACE(RecorderService, QMMF_RECORDER_SERVICE_NAME);
#endif // HAVE_BINDER

ServiceCallbackHandler::ServiceCallbackHandler(RecorderClient* client)
    : client_(client) {
    QMMF_GET_LOG_LEVEL();
    QMMF_DEBUG("%s Enter ", __func__);
    QMMF_DEBUG("%s Exit ", __func__);
}

ServiceCallbackHandler::~ServiceCallbackHandler() {
    QMMF_DEBUG("%s Enter ", __func__);
    QMMF_DEBUG("%s Exit ", __func__);
}

#ifndef HAVE_BINDER
void ServiceCallbackHandler::NotifyServerDeath() {
  QMMF_DEBUG("%s Enter ", __func__);
  assert(client_ != nullptr);
  client_->NotifyServerDeath();
  QMMF_DEBUG("%s Exit ", __func__);
}
#endif // !HAVE_BINDER

void ServiceCallbackHandler::NotifyRecorderEvent(EventType event, void *payload,
                                                 size_t size) {
  QMMF_DEBUG("%s Enter ", __func__);
  assert(client_ != nullptr);
  client_->NotifyRecorderEvent(event, payload, size);
  QMMF_DEBUG("%s Exit ", __func__);
}

void ServiceCallbackHandler::NotifySessionEvent(EventType event_type,
                                                void *event_data,
                                                size_t event_data_size) {
    QMMF_DEBUG("%s Enter ", __func__);
    QMMF_DEBUG("%s Exit ", __func__);
}

void ServiceCallbackHandler::NotifySnapshotData(uint32_t camera_id,
                                                uint32_t imgcount,
                                                BnBuffer& buffer,
                                                BufferMeta& meta) {
  assert(client_ != nullptr);
  client_->NotifySnapshotData(camera_id, imgcount, buffer, meta);
}

void ServiceCallbackHandler::NotifyOfflineJpegData(int32_t buf_fd,
                                                   uint32_t encoded_size) {
  assert(client_ != nullptr);
  client_->NotifyOfflineJpegData(buf_fd, encoded_size);
}


void ServiceCallbackHandler::NotifyVideoTrackData(uint32_t session_id,
                                                  uint32_t track_id,
                                                  std::vector<BnBuffer>&
                                                  bn_buffers,
                                                  std::vector<BufferMeta>&
                                                  metas) {

  QMMF_VERBOSE("%s Enter ", __func__);
  assert(client_ != nullptr);
  client_->NotifyVideoTrackData(session_id, track_id, bn_buffers, metas);
  QMMF_VERBOSE("%s Exit ", __func__);
}

void ServiceCallbackHandler::NotifyVideoTrackEvent(uint32_t session_id,
                                                   uint32_t track_id,
                                                   EventType event_type,
                                                   void *event_data,
                                                   size_t event_data_size) {
  QMMF_DEBUG("%s Enter ", __func__);
  QMMF_DEBUG("%s Exit ", __func__);
}

void ServiceCallbackHandler::NotifyCameraResult(uint32_t camera_id,
                                                const CameraMetadata &result) {
  assert(client_ != nullptr);
  client_->NotifyCameraResult(camera_id, result);
}

#ifdef HAVE_BINDER
class RecorderServiceCallbackProxy: public BpInterface<IRecorderServiceCallback> {
 public:
  RecorderServiceCallbackProxy(const sp<IBinder>& impl)
     : BpInterface<IRecorderServiceCallback>(impl) {}

  ~RecorderServiceCallbackProxy() {
    track_buffers_map_.clear();
    //TODO: Expose DeleteTrack Api from Binder proxy and call it from service.
  }

  void NotifyRecorderEvent(EventType event, void *payload, size_t size) {

    QMMF_DEBUG("%s Enter ", __func__);
    Parcel data, reply;

    data.writeInterfaceToken(
        IRecorderServiceCallback::getInterfaceDescriptor());
    data.writeUint32(static_cast<underlying_type<EventType>::type>(event));
    data.writeUint32(size);

    android::Parcel::WritableBlob blob;
    if (size) {
      data.writeBlob(size, false, &blob);
      memset(blob.data(), 0x0, size);
      memcpy(blob.data(), payload, size);
    }

    if (REMAP_ALL_BUFFERS == static_cast<uint32_t>(event)) {
      for (auto& iter : track_buffers_map_) {
        uint32_t track_id = iter.first;
        track_buffers_map_.erase(track_id);
      }
    }

    remote()->transact(
        uint32_t(RECORDER_SERVICE_CB_CMDS::RECORDER_NOTIFY_EVENT),
        data, &reply, IBinder::FLAG_ONEWAY);

    if (size) {
      blob.release();
    }
    QMMF_DEBUG("%s Exit ", __func__);
  }

  void NotifySessionEvent(EventType event_type, void *event_data,
                          size_t event_data_size) {

  }

  void NotifySnapshotData(uint32_t camera_id, uint32_t imgcount,
                          BnBuffer& buffer, BufferMeta& meta) {

    Parcel data, reply;
    data.writeInterfaceToken(IRecorderServiceCallback::
        getInterfaceDescriptor());
    data.writeUint32(camera_id);
    data.writeUint32(imgcount);
    data.writeFileDescriptor(buffer.ion_fd);
    data.writeFileDescriptor(buffer.ion_meta_fd);
    uint32_t size = sizeof buffer;
    data.writeUint32(size);
    android::Parcel::WritableBlob blob;
    data.writeBlob(size, false, &blob);
    memset(blob.data(), 0x0, size);
    memcpy(blob.data(), reinterpret_cast<void*>(&buffer), size);
    // Pack meta
    size = sizeof (meta);
    data.writeUint32(size);
    android::Parcel::WritableBlob meta_blob;
    data.writeBlob(size, false, &meta_blob);
    memset(meta_blob.data(), 0x0, size);
    memcpy(meta_blob.data(), reinterpret_cast<void*>(&meta), size);

    remote()->transact(uint32_t(RECORDER_SERVICE_CB_CMDS::
        RECORDER_NOTIFY_SNAPSHOT_DATA), data, &reply, IBinder::FLAG_ONEWAY);

    blob.release();
    meta_blob.release();
  }

  void NotifyOfflineJpegData(int32_t buf_fd, uint32_t encoded_size) {

    Parcel data, reply;
    data.writeInterfaceToken(IRecorderServiceCallback::
        getInterfaceDescriptor());
    // This is the client fd and thus passing it as int
    data.writeInt32(buf_fd);
    data.writeUint32(encoded_size);

    remote()->transact(uint32_t(RECORDER_SERVICE_CB_CMDS::
        RECORDER_NOTIFY_OFFLINE_JPEG_DATA), data, &reply, IBinder::FLAG_ONEWAY);
  }

  void NotifyVideoTrackData(uint32_t session_id, uint32_t track_id,
                            std::vector<BnBuffer>& buffers,
                            std::vector<BufferMeta>& metas) {

    QMMF_VERBOSE("Bp%s: Enter", __func__);

    Parcel data, reply;
    data.writeInterfaceToken(IRecorderServiceCallback::
        getInterfaceDescriptor());

    data.writeUint32(session_id);
    data.writeUint32(track_id);
    data.writeUint32(buffers.size());

    for(uint32_t i = 0; i < buffers.size(); i++) {
      bool ismapped = false;
      {
        std::lock_guard<std::mutex> l(track_buffers_lock_);
        auto& buffer_ids = track_buffers_map_[track_id];

        // If ION fd has already been sent to client, no binder packing is
        // required, only index would be sufficient for client to get mapped
        // buffer from his own map.
        ismapped = (buffer_ids.count(buffers[i].buffer_id) != 0);

        QMMF_VERBOSE("Bp%s: buffers[%d].ion_fd=%d ismapped:%d",
            __func__, i, buffers[i].ion_fd, ismapped);
      }
      // If buffer has not been sent to client then pack the file descriptor
      // and provide hint about incoming fd.
      data.writeInt32(ismapped);

      if (!ismapped) {
        // Pack file descriptor.
        data.writeFileDescriptor(buffers[i].ion_fd);
        bool hasmetafd = (buffers[i].ion_meta_fd > 0);
        data.writeUint32(hasmetafd);
        if (hasmetafd) {
          data.writeFileDescriptor(buffers[i].ion_meta_fd);
        }
        {
          std::lock_guard<std::mutex> l(track_buffers_lock_);
          auto& buffer_ids = track_buffers_map_[track_id];
          buffer_ids.emplace(buffers[i].buffer_id);
        }
        QMMF_VERBOSE("%s: Bp: track_id=%d", __func__, track_id);
        QMMF_VERBOSE("%s: Bp: buffers[%d].ion_fd=%d mapping:%d", __func__,
            i, buffers[i].ion_fd, true);
      }
      uint32_t size = sizeof (BnBuffer);
      data.writeUint32(size);
      android::Parcel::WritableBlob blob;
      data.writeBlob(size, false, &blob);
      memset(blob.data(), 0x0, size);
      memcpy(blob.data(), reinterpret_cast<void*>(&buffers[i]), size);
    }
    // Pack meta
    data.writeUint32(metas.size());
    for(uint32_t i = 0; i < metas.size(); ++i) {
      uint32_t size = sizeof (BufferMeta);
      data.writeUint32(size);
      android::Parcel::WritableBlob meta_blob;
      data.writeBlob(size, false, &meta_blob);
      memset(meta_blob.data(), 0x0, size);
      memcpy(meta_blob.data(), reinterpret_cast<void*>(&metas[i]), size);
    }

    remote()->transact(
        uint32_t(RECORDER_SERVICE_CB_CMDS::RECORDER_NOTIFY_VIDEO_TRACK_DATA),
        data, &reply, IBinder::FLAG_ONEWAY);

    QMMF_VERBOSE("%s: Exit - Sent Message One Way!!", __func__);
  }

  void NotifyVideoTrackEvent(uint32_t session_id, uint32_t track_id,
                             EventType event_type,
                             void *event_data, size_t event_data_size) {

  }

  void NotifyCameraResult(uint32_t camera_id, const CameraMetadata &result) {
    Parcel data, reply;
    data.writeInterfaceToken(IRecorderServiceCallback::getInterfaceDescriptor());
    data.writeUint32(camera_id);
    result.writeToParcel(&data);
    remote()->transact(uint32_t(RECORDER_SERVICE_CB_CMDS::
                                RECORDER_NOTIFY_CAMERA_RESULT), data, &reply,
                                IBinder::FLAG_ONEWAY);
  }

  void NotifyDeleteVideoTrack(uint32_t track_id) {
    QMMF_VERBOSE("Bp%s: Enter", __func__);
    std::lock_guard<std::mutex> l(track_buffers_lock_);
    track_buffers_map_.erase(track_id);
    QMMF_VERBOSE("Bp%s: Exit", __func__);
  }

 private:
  // map <track_id , set <buffer_id> >
  std::map<uint32_t,  std::set<uint32_t> > track_buffers_map_;
  // to protect track_buffers_map_
  std::mutex  track_buffers_lock_;
};

IMPLEMENT_META_INTERFACE(RecorderServiceCallback,
                            "recorder.service.IRecorderServiceCallback");

status_t BnRecorderServiceCallback::onTransact(uint32_t code,
                                               const Parcel& data,
                                               Parcel* reply,
                                               uint32_t flags) {
  QMMF_DEBUG("%s: Enter:(BnRecorderServiceCallback::onTransact)",
      __func__);
  CHECK_INTERFACE(IRecorderServiceCallback, data, reply);

  switch(code) {
    case RECORDER_SERVICE_CB_CMDS::RECORDER_NOTIFY_EVENT: {
      uint32_t event, size;

      data.readUint32(&event);
      data.readUint32(&size);

      android::Parcel::ReadableBlob blob;
      void *payload = nullptr;
      if (size) {
        data.readBlob(size, &blob);
        payload = const_cast<void*>(blob.data());
      }
      NotifyRecorderEvent(static_cast<EventType>(event), payload, size);
      if (size) {
        blob.release();
      }
      return 0;
    }
    break;
    case RECORDER_SERVICE_CB_CMDS::RECORDER_NOTIFY_SESSION_EVENT: {
      //TODO:
      return 0;
    }
    break;
    case RECORDER_SERVICE_CB_CMDS::RECORDER_NOTIFY_SNAPSHOT_DATA: {
      uint32_t camera_id, count, size;
      data.readUint32(&camera_id);
      data.readUint32(&count);
      uint32_t ion_fd = dup(data.readFileDescriptor());
      uint32_t ion_meta_fd = dup(data.readFileDescriptor());
      data.readUint32(&size);
      android::Parcel::ReadableBlob blob;
      data.readBlob(size, &blob);
      void* buf = const_cast<void*>(blob.data());
      BnBuffer bn_buffer{};
      memcpy(&bn_buffer, buf, size);
      bn_buffer.ion_fd = ion_fd;
      bn_buffer.ion_meta_fd = ion_meta_fd;
      uint32_t meta_size;
      BufferMeta meta{};
      android::Parcel::ReadableBlob meta_blob;
      data.readUint32(&meta_size);
      if (meta_size > 0) {
        data.readBlob(meta_size, &meta_blob);
        void* data = const_cast<void*>(meta_blob.data());
        memcpy(&meta, data, meta_size);
      }
      NotifySnapshotData(camera_id, count, bn_buffer, meta);
      blob.release();
      if (meta_size > 0) {
        meta_blob.release();
      }
      return 0;
    }
    break;
    case RECORDER_SERVICE_CB_CMDS::RECORDER_NOTIFY_OFFLINE_JPEG_DATA: {
      uint32_t encoded_size;
      int32_t buf_fd;
      // This is the client fd
      data.readInt32(&buf_fd);
      data.readUint32(&encoded_size);
      NotifyOfflineJpegData(buf_fd, encoded_size);
      return 0;
    }
    break;
    case RECORDER_SERVICE_CB_CMDS::RECORDER_NOTIFY_VIDEO_TRACK_DATA: {

      uint32_t session_id, track_id, vector_size;
      std::vector<BnBuffer> buffers;
      data.readUint32(&session_id);
      data.readUint32(&track_id);
      data.readUint32(&vector_size);
      QMMF_VERBOSE("Bn%s: vector_size=%d", __func__, vector_size);
      uint32_t size = 0;
      for (uint32_t i = 0; i < vector_size; i++)  {
        int32_t ismapped = 0, hasmetafd = 0;
        int32_t ion_fd = -1, ion_meta_fd = -1;
        data.readInt32(&ismapped);
        if (ismapped == 0) {
          ion_fd = dup(data.readFileDescriptor());
          data.readInt32(&hasmetafd);
          if (hasmetafd == 1) {
            ion_meta_fd = dup(data.readFileDescriptor());
          }
        }
        data.readUint32(&size);
        android::Parcel::ReadableBlob blob;
        data.readBlob(size, &blob);
        void* buffer = const_cast<void*>(blob.data());
        BnBuffer track_buffer;
        memcpy(&track_buffer, buffer, size);
        track_buffer.ion_fd = ion_fd;
        track_buffer.ion_meta_fd = ion_meta_fd;
        buffers.push_back(track_buffer);
        blob.release();
      }
      uint32_t meta_vector_size = 0;
      std::vector<BufferMeta> metas;
      data.readUint32(&meta_vector_size);
      QMMF_VERBOSE("%s: Bn: meta_vector_size=%d", __func__,
          meta_vector_size);
      android::Parcel::ReadableBlob meta_blob;
      for (uint32_t i = 0; i < meta_vector_size; i++)  {
        data.readUint32(&size);
        data.readBlob(size, &meta_blob);
        void* data = const_cast<void*>(meta_blob.data());
        BufferMeta meta;
        memcpy(&meta, data, size);
        metas.push_back(meta);
        meta_blob.release();
      }
      NotifyVideoTrackData(session_id, track_id, buffers, metas);
      return 0;
    }
    break;
    case RECORDER_SERVICE_CB_CMDS::RECORDER_NOTIFY_VIDEO_TRACK_EVENT: {
      //TODO:
      return 0;
    }
    break;
    case RECORDER_SERVICE_CB_CMDS::RECORDER_NOTIFY_CAMERA_RESULT: {
      uint32_t camera_id = data.readUint32();
      CameraMetadata meta;
      auto ret = meta.readFromParcel(&data);
      if (NO_ERROR == ret) {
        NotifyCameraResult(camera_id, meta);
      } else {
        QMMF_ERROR("%s Failed to read camera result from parcel: %d\n",
                     __func__, ret);
      }
      return 0;
    }
    break;
    default: {
      QMMF_ERROR("%s Method not supported ", __func__);
    }
    break;
  }
  return 0;
}
#else
RecorderServiceCallbackStub::RecorderServiceCallbackStub() {
  QMMF_INFO("%s: Enter", __func__);
  cb_socket_ = -1;
  client_socket_ = -1;
  socket_recv_buf_ = new char[kMaxSocketBufSize];
  QMMF_INFO("%s: Exit", __func__);
}

RecorderServiceCallbackStub::~RecorderServiceCallbackStub() {
  QMMF_INFO("%s: Enter", __func__);

  if (!ion_fd_map_.empty()) {
    for (const auto& [fd, dup_fd] : ion_fd_map_ ) {
      close(dup_fd);
    }
    ion_fd_map_.clear();
  }
  if (!meta_fd_map_.empty()) {
    for (const auto& [fd, dup_fd] : meta_fd_map_ ) {
      close(dup_fd);
    }
    meta_fd_map_.clear();
  }

  run_thread_ = false;
  if (callback_thread_.joinable()) {
    callback_thread_.join();
  }

  if (cb_socket_ != -1) {
    close(cb_socket_);
    cb_socket_ = -1;
  }

  if (client_socket_ != -1 ) {
    shutdown(client_socket_, SHUT_RDWR);
    close(client_socket_);
    client_socket_ = -1;
  }

  delete[] socket_recv_buf_;

  QMMF_INFO("%s: Exit", __func__);
}

status_t RecorderServiceCallbackStub::Init(uint32_t client_id, uint32_t server_pid) {
  QMMF_INFO("%s: Enter", __func__);

  std:;stringstream path;
  path << "/tmp/socket/cam_server/le_cam_client." << client_id << ".sock";
  socket_path_ = path.str();
  ::unlink(socket_path_.c_str());

  // Create a socket
  cb_socket_ = socket(AF_UNIX, SOCK_STREAM, 0);
  if (cb_socket_ == -1) {
    QMMF_ERROR("Server: sock failure - %s", strerror(errno));
    return -errno;
  }

  sockaddr_un addr;
  addr.sun_family = AF_UNIX;
  auto size = socket_path_.size();
  snprintf(addr.sun_path, size+1, "%s", socket_path_.c_str());
  addr.sun_path[size+1] = '\0';
  if (bind(cb_socket_, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
    QMMF_ERROR("Server: bind failure - %s", strerror(errno));
    return -errno;
  }

  // TODO: Check the max queue limit for incoming request
  if (listen(cb_socket_, 5) == -1) {
    QMMF_ERROR("%s: listen failure - %s", __func__, strerror(errno));
    return -errno;
  }

  server_pid_ = server_pid;

  run_thread_ = true;
  try {
    callback_thread_ = std::thread(&RecorderServiceCallbackStub::ThreadLoop, this);
    QMMF_VERBOSE("%s: Callback thread spawned!", __func__);
  } catch(const std::system_error& e) {
    QMMF_ERROR("%s: error creating callback thread - %s", __func__, e.what());
    return e.code().value();
  }

  QMMF_INFO("%s: Exit: Server is listening on %s", __func__, socket_path_.c_str());
  return 0;
}

void RecorderServiceCallbackStub::ThreadLoop () {
  QMMF_INFO("%s: waiting for connection on %d", __func__, cb_socket_);

  client_socket_ = accept(cb_socket_, nullptr, nullptr);
  if (client_socket_ == -1) {
    QMMF_ERROR("%s: accept failure %s", __func__, strerror(errno));
    close(cb_socket_);
    unlink(socket_path_.c_str());
    return;
  }

  QMMF_INFO("%s: connection accepted new fd %d", __func__, client_socket_);
  while (run_thread_) {
    ssize_t bytes_read = recv(client_socket_, socket_recv_buf_, kMaxSocketBufSize, 0);
    QMMF_VERBOSE("%s: recv %d bytes", __func__, bytes_read);

    if (bytes_read < 0) {
      QMMF_ERROR("%s: recv failure %s", __func__, strerror(errno));
      break;
    } else if (bytes_read == 0) {
      QMMF_ERROR("%s: connection closed", __func__);
      NotifyServerDeath();
      break;
    }

    ssize_t buf_size = bytes_read;
    auto buf_ptr = socket_recv_buf_;
    while (buf_size > 0) {
      QMMF_VERBOSE("%s: buf_size: %d", __func__, buf_size);
      uint32_t msg_size = *(reinterpret_cast<uint32_t *>(buf_ptr));
      // Moving past the size
      buf_ptr += 4;
      QMMF_VERBOSE("%s: msg_size: %d", __func__, msg_size);
      RecorderClientCallbacksAsync msg;
      msg.ParseFromArray(buf_ptr, msg_size);
      auto ret = ProcessCallbackMsg(msg);
      QMMF_VERBOSE("%s: Processed cmd: %d", __func__, msg.cmd());
      if (ret != 0) {
        QMMF_ERROR("%s: ProcessCallbackMsg failed %s", __func__, strerror(errno));
        break;
      } else {
        buf_ptr += msg_size;
        buf_size -= (msg_size + 4);
      }
    }
    memset(socket_recv_buf_, 0, bytes_read);
  }
  QMMF_INFO("%s: Exit %d", __func__, cb_socket_);
}

status_t RecorderServiceCallbackStub::ProcessCallbackMsg(
    RecorderClientCallbacksAsync &msg) {

  switch(msg.cmd()) {
    case RECORDER_SERVICE_CB_CMDS::RECORDER_NOTIFY_EVENT: {
      NotifyRecorderEventMsg data = msg.recorder_event();
      EventType event = static_cast<EventType>(data.type());
      const void *payload = static_cast<const void*>(data.event_msg().data());
      uint32_t size = data.event_msg().size();
      NotifyRecorderEvent(event, const_cast<void *>(payload), size);
      return 0;
    }
    break;
    case RECORDER_SERVICE_CB_CMDS::RECORDER_NOTIFY_SESSION_EVENT: {
      //TODO:
      return 0;
    }
    break;
    case RECORDER_SERVICE_CB_CMDS::RECORDER_NOTIFY_SNAPSHOT_DATA: {
      NotifySnapshotDataMsg data = msg.snapshot_data();
      uint32_t camera_id = data.camera_id();
      uint32_t count = data.img_count();
      BnBuffer bn_buffer;
      int32_t dup_buf_fd, dup_meta_fd;
      dup_buf_fd = syscall(SYS_pidfd_getfd,
                syscall(SYS_pidfd_open, server_pid_, 0),
                data.buffer().ion_fd(),
                0);

      dup_meta_fd = syscall(SYS_pidfd_getfd,
                syscall(SYS_pidfd_open, server_pid_, 0),
                data.buffer().ion_meta_fd(),
                0);
      bn_buffer.ion_fd = dup_buf_fd;
      bn_buffer.ion_meta_fd = dup_meta_fd;
      bn_buffer.img_id = data.buffer().img_id();
      bn_buffer.size = data.buffer().size();
      bn_buffer.timestamp = data.buffer().timestamp();
      bn_buffer.seqnum = data.buffer().seqnum();
      bn_buffer.buffer_id = data.buffer().buffer_id();
      bn_buffer.flags = data.buffer().flags();
      bn_buffer.capacity = data.buffer().capacity();
      QMMF_VERBOSE("%s : INPARAM: buffers: %s", __func__,
          bn_buffer.ToString().c_str());

      BufferMeta meta;
      meta.format = static_cast<BufferFormat>(data.meta().format());
      meta.n_planes = data.meta().n_planes();
      for (auto i = 0; i < meta.n_planes; i++) {
        auto p_info = data.meta().info(i);
        meta.planes[i].stride = p_info.stride();
        meta.planes[i].scanline = p_info.scanline();
        meta.planes[i].width = p_info.width();
        meta.planes[i].height = p_info.height();
        meta.planes[i].offset = p_info.offset();
        meta.planes[i].size = p_info.size();
      }

      NotifySnapshotData(camera_id, count, bn_buffer, meta);
      return 0;
    }
    break;
    case RECORDER_SERVICE_CB_CMDS::RECORDER_NOTIFY_OFFLINE_JPEG_DATA: {
      return 0;
    }
    break;
    case RECORDER_SERVICE_CB_CMDS::RECORDER_NOTIFY_VIDEO_TRACK_DATA: {
      NotifyVideoTrackDataMsg data = msg.video_track_data();
      uint32_t session_id = data.session_id();
      uint32_t track_id = data.track_id();
      std::vector<BnBuffer> buffers;
      int32_t ion_fd, meta_fd, dup_buf_fd, dup_meta_fd;
      for (auto &&b_data : data.buffers()) {
        BnBuffer buffer;
        ion_fd = b_data.ion_fd();
        meta_fd = b_data.ion_meta_fd();
        { 
          std::lock_guard<std::mutex> l(fd_map_lock_);
          if (ion_fd > 0) {
            if (ion_fd_map_.count(ion_fd) != 0) {
              // Buffer is already mapped to the client
              dup_buf_fd = ion_fd_map_.at(ion_fd);
            } else {
              // New buffer, map it to the client
              dup_buf_fd = syscall(SYS_pidfd_getfd,
                        syscall(SYS_pidfd_open, server_pid_, 0),
                        ion_fd,
                        0);
              ion_fd_map_[ion_fd] = dup_buf_fd;
            }
          }

          if (meta_fd > 0) {
            if (meta_fd_map_.count(meta_fd) != 0) {
              // Meta buffer is already mapped to the client
              dup_meta_fd = meta_fd_map_.at(meta_fd);
            } else {
              // New meta buffer, map it to the client
              dup_meta_fd = syscall(SYS_pidfd_getfd,
                        syscall(SYS_pidfd_open, server_pid_, 0),
                        meta_fd,
                        0);
              meta_fd_map_[meta_fd] = dup_meta_fd;
            }
          }
        }
        buffer.ion_fd = dup_buf_fd;
        buffer.ion_meta_fd = dup_meta_fd;
        buffer.img_id = b_data.img_id();
        buffer.size = b_data.size();
        buffer.timestamp = b_data.timestamp();
        buffer.seqnum = b_data.seqnum();
        buffer.buffer_id = b_data.buffer_id();
        buffer.flags = b_data.flags();
        buffer.capacity = b_data.capacity();
        QMMF_VERBOSE("%s : INPARAM: buffers: %s", __func__,
            buffer.ToString().c_str());
        buffers.push_back(buffer);
      }

      std::vector<BufferMeta> metas;
      for (auto &&m_data : data.metas()) {
        BufferMeta meta;
        meta.format = static_cast<BufferFormat>(m_data.format());
        meta.n_planes = m_data.n_planes();
        for (auto i = 0; i < meta.n_planes; i++) {
          auto p_info = m_data.info(i);
          meta.planes[i].stride = p_info.stride();
          meta.planes[i].scanline = p_info.scanline();
          meta.planes[i].width = p_info.width();
          meta.planes[i].height = p_info.height();
          meta.planes[i].offset = p_info.offset();
          meta.planes[i].size = p_info.size();
        }
        metas.push_back(meta);
      }
      NotifyVideoTrackData(session_id, track_id, buffers, metas);
      return 0;
    }
    break;
    case RECORDER_SERVICE_CB_CMDS::RECORDER_NOTIFY_VIDEO_TRACK_EVENT: {
      //TODO:
      return 0;
    }
    break;
    case RECORDER_SERVICE_CB_CMDS::RECORDER_NOTIFY_CAMERA_RESULT: {
      return 0;
    }
    break;
    default: {
      QMMF_ERROR("%s Method not supported ", __func__);
    }
    break;
  }

  return -1;
}

void RecorderServiceCallbackStub::NotifyServerDeath() {
  QMMF_INFO("%s Enter", __func__);
  QMMF_INFO("%s Exit", __func__);
}

#endif // HAVE_BINDER
}; //namespace qmmf

}; //namespace recorder
