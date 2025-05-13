/*
* Copyright (c) 2019-2021, The Linux Foundation. All rights reserved.
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

#include <mutex>
#include <atomic>
#include <map>

#include <qmmf-sdk/qmmf_recorder_params.h>
#include <qmmf-sdk/qmmf_recorder_extra_param_tags.h>

#include "common/utils/qmmf_condition.h"
#include <system/camera_metadata.h>
#include <camera/CameraParameters.h>
#include "recorder/src/service/qmmf_camera_interface.h"

namespace qmmf {

namespace recorder {

#define MAX_SNAPSHOT_BUFFER_COUNT        15

class CameraPort;
class PreviewPort;
class VideoPort;
enum class CameraPortType;

class CameraContext : public CameraInterface {
 public:
  CameraContext();
  ~CameraContext();

  status_t HAL_load();
  status_t HAL_unload();
  status_t camera_device_open(uint8_t id);
  status_t close_camera_device();
  status_t set_callbacks();
  char* get_parameters();
  void put_parameters(char *parms);

  status_t OpenCamera(const uint32_t camera_id, const float frame_rate,
    const CameraExtraParam& extra_param, const ResultCb &cb = nullptr,
    const ErrorCb &errcb = nullptr) override;

  status_t CloseCamera(const uint32_t camera_id) override;

  status_t WaitAecToConverge(const uint32_t timeout) override;

  status_t ConfigImageCapture(const SnapshotParam& param,
                              const ImageExtraParam &xtraparam) override;

  status_t CaptureImage(const SnapshotType type, const uint32_t n_images,
                        const std::vector<CameraMetadata> &meta,
                        const StreamSnapshotCb& cb) override;

  status_t CancelCaptureImage(const bool cache) override;

  status_t CreateStream(const StreamParam& param,
                        const VideoExtraParam& extra_param) override;

  status_t DeleteStream(const uint32_t track_id) override;

  status_t AddConsumer(const uint32_t& track_id,
                       sp<IBufferConsumer>& consumer) override;

  status_t RemoveConsumer(const uint32_t& track_id,
                          sp<IBufferConsumer>& consumer) override;

  status_t StartStream(const uint32_t track_id, bool cached) override;

  status_t StopStream(const uint32_t track_id, bool cached) override;

  status_t SetCameraParam(const CameraMetadata &meta) override;

  status_t GetCameraParam(CameraMetadata &meta) override;

  status_t GetDefaultCaptureParam(CameraMetadata &meta) override;

  status_t GetCameraCharacteristics(CameraMetadata &meta) override;

  status_t ReturnAllImageCaptureBuffers() override;

  status_t ReturnImageCaptureBuffer(const uint32_t camera_id,
                                    const int32_t buffer_id) override;

  std::vector<int32_t>& GetSupportedFps() override;

  status_t PopulateBufferMeta(BufferMeta &info, IBufferHandle &handle,
                              uint32_t width, uint32_t height);

  status_t SnapshotCallback(const camera_memory_t *data, int64_t timestamp = 0);

  status_t ReturnStreamBuffer(StreamBuffer buffer);

  const char *FromQmmfToHalFormat_hal1(const BufferFormat &format);

  std::shared_ptr<CameraPort> GetPortByType(const CameraPortType port_type);
  std::shared_ptr<CameraPort> GetPortById(const uint32_t& track_id);
  std::shared_ptr<CameraPort> GetFreePort();

  status_t ApplyParameters();

  status_t SetFps(float fps);

  sp<IBufferProducer>      buffer_producer_impl_;
  std::mutex               buffer_lock_;
  void*                    camera_device_;

  IAllocDevice*            alloc_device_interface_;

  uint32_t                 camera_id_;
  ResultCb                 result_cb_;
  ErrorCb                  error_cb_;

private:

  friend class PreviewPort;
  friend class VideoPort;

  CameraMetadata           metadata_;

  QCondition               wait_for_buffer_;

  std::map<uint32_t, std::shared_ptr<CameraPort> > ports_;
  std::map<uint32_t, StreamParam> streams_params_;
  std::map<uint32_t, sp<IBufferConsumer> > consumers_;

  void *libptr_;
  android::CameraParameters mParameters_;

  template <class mapType> uint32_t lookupAttr(const mapType *arr, size_t len,
    const char *name);
  template <class mapType> const char *lookupNameByValue(const mapType *arr,
    size_t len, int32_t value);

  template <typename valueType> struct QmmfCameraMap {
      const char *const desc;
      valueType val;
  };

  static const char KEY_QTI_RAW_PICUTRE_SIZE[];
  static const char WHITE_BALANCE_MANUAL[];
  static const char QTI_PIXEL_FORMAT_NV12_VENUS[];
  static const char QTI_PIXEL_FORMAT_NV21_VENUS[];

  static const QmmfCameraMap<camera_metadata_enum_android_control_awb_mode_t>
    WHITE_BALANCE_MODES_MAP[];
  static const QmmfCameraMap<camera_metadata_enum_android_control_effect_mode_t>
    EFFECT_MODES_MAP[];
  static const QmmfCameraMap<camera_metadata_enum_android_control_ae_antibanding_mode_t>
    ANTIBANDING_MODES_MAP[];
  static const QmmfCameraMap<camera_metadata_enum_android_control_scene_mode_t>
    SCENE_MODES_MAP[];
  static const QmmfCameraMap<uint8_t> TRUE_FALSE_MAP[];

  status_t ParsePair(const char *str, uint32_t *first, uint32_t *second,
                     char delim, char **endptr = NULL);
  status_t ParseList(const char *list, std::vector<std::string> &sizes);

  // Maps of buffer Id and Buffer.
  std::map<uint32_t, StreamBuffer> snapshot_buffer_list_;
  std::map<uint32_t, const camera_memory_t *> snapshot_hal_buff_list_;
  uint32_t                         snapshot_frame_id_;

  std::mutex                    capture_lock_;
  SnapshotParam                 snapshot_param_;
  StreamSnapshotCb              client_snapshot_cb_;
  uint32_t                      capture_cnt_;
};

enum class CameraPortType {
  kPreview,
  kVideo,
};

enum class PortState {
  PORT_CREATED,
  PORT_INITIALIZED,
  PORT_STARTED,
};

class CameraPort {
public:
  CameraPort(CameraPortType port_type, CameraContext *context);
  ~CameraPort();
  virtual status_t Init(const StreamParam& param) = 0;
  status_t DeInit();
  virtual status_t Start(bool cached = false) = 0;
  virtual status_t Stop(bool cached = false) = 0;

  status_t AddConsumer(sp<IBufferConsumer>& consumer);
  status_t RemoveConsumer(sp<IBufferConsumer>& consumer);
  int32_t GetNumConsumers();

  PortState& getPortState();
  float GetPortFramerate() { return port_frame_rate_; }
  uint32_t GetPortId() { return track_id_; }
  CameraPortType GetPortType() { return port_type_; }

  void NotifyBufferReturned(const StreamBuffer& buffer);
  void StreamCallback(const void *data, int64_t timestamp);

  status_t release_frame(const void *opaque);

  QCondition             wait_for_buffer_;
  int32_t                buffer_count_;

protected:
  CameraContext*         context_;
  sp<IBufferProducer>    buffer_producer_impl_;
  PortState              port_state_;
  uint32_t               track_id_;
  int32_t                frame_number_;
  std::mutex             state_lock_;
  float                  port_frame_rate_;
  BufferFormat           port_format_;
  uint32_t               width_;
  uint32_t               height_;
  std::map <int32_t, const void *> buffer_map_;

private:
  bool IsConsumerConnected(sp<IBufferConsumer>& consumer);
  std::map<uintptr_t, sp<IBufferConsumer> > consumers_;
  std::mutex consumer_lock_;
  CameraPortType port_type_;
};

class PreviewPort : public CameraPort {
public:
  PreviewPort(CameraPortType port_type, CameraContext *context) :
    CameraPort(port_type, context) {};
  status_t Init(const StreamParam& param) override;

  status_t Start(bool cached = false) override;
  status_t Stop(bool cached = false) override;
};

class VideoPort : public CameraPort {
public:
  VideoPort(CameraPortType port_type, CameraContext *context) :
    CameraPort(port_type, context) {};
  status_t Init(const StreamParam& param) override;

  status_t Start(bool cached = false) override;
  status_t Stop(bool cached = false) override;
};

}; //namespace recorder

}; //namespace qmmf
