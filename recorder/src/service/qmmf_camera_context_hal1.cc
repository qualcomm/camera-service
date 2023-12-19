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
 * Changes from Qualcomm Innovation Center are provided under the following license:
 *
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
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

#define LOG_TAG "RecorderCameraContextHal1"

#include <thread>
#include <algorithm>
#include <chrono>
#include <fcntl.h>
#include <math.h>
#include <sys/mman.h>
#include <sys/mman.h>
#include <fstream>

#include <dlfcn.h>

#include <sys/stat.h>
#include <binder/IMemory.h>
#include <binder/MemoryBase.h>
#include <binder/MemoryHeapBase.h>
#include <utils/RefBase.h>
#include <cutils/properties.h>
#include <hardware/hardware.h>
#include <utils/Timers.h>
#include <hardware/camera.h>

#include "qmmf_camera_context_hal1.h"
#include "recorder/src/service/qmmf_recorder_utils.h"
#include <gbm_priv.h>
#include <gbm.h>
#include <libstagefrighthw/QComOMXMetadata.h>

namespace qmmf {

namespace recorder {

#define PARAM_MAP_SIZE(MAP) (sizeof(MAP)/sizeof(MAP[0]))

template<class mapType> uint32_t CameraContext::lookupAttr(const mapType *arr,
  size_t len, const char *name) {
  if (name) {
    for (size_t i = 0; i < len; i++) {
      if (!strcmp(arr[i].desc, name))
        return arr[i].val;
    }
  }
  return NAME_NOT_FOUND;
}

template<class mapType> const char *CameraContext::lookupNameByValue(
  const mapType *arr, size_t len, int32_t value) {
  for (size_t i = 0; i < len; i++) {
    if (arr[i].val == value) {
      return arr[i].desc;
    }
  }
  return NULL;
}

const char CameraContext::KEY_QTI_RAW_PICUTRE_SIZE[] = "raw-size";
const char CameraContext::WHITE_BALANCE_MANUAL[] = "manual";
const char CameraContext::QTI_PIXEL_FORMAT_NV12_VENUS[] = "nv12-venus";
const char CameraContext::QTI_PIXEL_FORMAT_NV21_VENUS[] = "nv21-venus";

const CameraContext::QmmfCameraMap<camera_metadata_enum_android_control_awb_mode_t>
        CameraContext::WHITE_BALANCE_MODES_MAP[] = {
    { CameraParameters::WHITE_BALANCE_AUTO,            ANDROID_CONTROL_AWB_MODE_AUTO },
    { CameraParameters::WHITE_BALANCE_INCANDESCENT,    ANDROID_CONTROL_AWB_MODE_INCANDESCENT },
    { CameraParameters::WHITE_BALANCE_FLUORESCENT,     ANDROID_CONTROL_AWB_MODE_FLUORESCENT },
    { CameraParameters::WHITE_BALANCE_WARM_FLUORESCENT,ANDROID_CONTROL_AWB_MODE_WARM_FLUORESCENT},
    { CameraParameters::WHITE_BALANCE_DAYLIGHT,        ANDROID_CONTROL_AWB_MODE_DAYLIGHT },
    { CameraParameters::WHITE_BALANCE_CLOUDY_DAYLIGHT, ANDROID_CONTROL_AWB_MODE_CLOUDY_DAYLIGHT },
    { CameraParameters::WHITE_BALANCE_TWILIGHT,        ANDROID_CONTROL_AWB_MODE_TWILIGHT },
    { CameraParameters::WHITE_BALANCE_SHADE,           ANDROID_CONTROL_AWB_MODE_SHADE },
    { CameraContext::WHITE_BALANCE_MANUAL,             ANDROID_CONTROL_AWB_MODE_OFF },
};

const CameraContext::QmmfCameraMap<camera_metadata_enum_android_control_effect_mode_t>
        CameraContext::EFFECT_MODES_MAP[] = {
    { CameraParameters::EFFECT_NONE,       ANDROID_CONTROL_EFFECT_MODE_OFF },
    { CameraParameters::EFFECT_MONO,       ANDROID_CONTROL_EFFECT_MODE_MONO },
    { CameraParameters::EFFECT_NEGATIVE,   ANDROID_CONTROL_EFFECT_MODE_NEGATIVE },
    { CameraParameters::EFFECT_SOLARIZE,   ANDROID_CONTROL_EFFECT_MODE_SOLARIZE },
    { CameraParameters::EFFECT_SEPIA,      ANDROID_CONTROL_EFFECT_MODE_SEPIA },
    { CameraParameters::EFFECT_POSTERIZE,  ANDROID_CONTROL_EFFECT_MODE_POSTERIZE },
    { CameraParameters::EFFECT_WHITEBOARD, ANDROID_CONTROL_EFFECT_MODE_WHITEBOARD },
    { CameraParameters::EFFECT_BLACKBOARD, ANDROID_CONTROL_EFFECT_MODE_BLACKBOARD },
    { CameraParameters::EFFECT_AQUA,       ANDROID_CONTROL_EFFECT_MODE_AQUA }
};

const CameraContext::QmmfCameraMap<camera_metadata_enum_android_control_ae_antibanding_mode_t>
        CameraContext::ANTIBANDING_MODES_MAP[] = {
    { CameraParameters::ANTIBANDING_OFF,  ANDROID_CONTROL_AE_ANTIBANDING_MODE_OFF },
    { CameraParameters::ANTIBANDING_50HZ, ANDROID_CONTROL_AE_ANTIBANDING_MODE_50HZ },
    { CameraParameters::ANTIBANDING_60HZ, ANDROID_CONTROL_AE_ANTIBANDING_MODE_60HZ },
    { CameraParameters::ANTIBANDING_AUTO, ANDROID_CONTROL_AE_ANTIBANDING_MODE_AUTO }
};

const CameraContext::QmmfCameraMap<camera_metadata_enum_android_control_scene_mode_t>
        CameraContext::SCENE_MODES_MAP[] = {
    { CameraParameters::SCENE_MODE_AUTO,           ANDROID_CONTROL_SCENE_MODE_DISABLED },
    { CameraParameters::SCENE_MODE_ACTION,         ANDROID_CONTROL_SCENE_MODE_ACTION },
    { CameraParameters::SCENE_MODE_PORTRAIT,       ANDROID_CONTROL_SCENE_MODE_PORTRAIT },
    { CameraParameters::SCENE_MODE_LANDSCAPE,      ANDROID_CONTROL_SCENE_MODE_LANDSCAPE },
    { CameraParameters::SCENE_MODE_NIGHT,          ANDROID_CONTROL_SCENE_MODE_NIGHT },
    { CameraParameters::SCENE_MODE_NIGHT_PORTRAIT, ANDROID_CONTROL_SCENE_MODE_NIGHT_PORTRAIT },
    { CameraParameters::SCENE_MODE_THEATRE,        ANDROID_CONTROL_SCENE_MODE_THEATRE },
    { CameraParameters::SCENE_MODE_BEACH,          ANDROID_CONTROL_SCENE_MODE_BEACH },
    { CameraParameters::SCENE_MODE_SNOW,           ANDROID_CONTROL_SCENE_MODE_SNOW },
    { CameraParameters::SCENE_MODE_SUNSET,         ANDROID_CONTROL_SCENE_MODE_SUNSET },
    { CameraParameters::SCENE_MODE_STEADYPHOTO,    ANDROID_CONTROL_SCENE_MODE_SUNSET },
    { CameraParameters::SCENE_MODE_FIREWORKS ,     ANDROID_CONTROL_SCENE_MODE_FIREWORKS },
    { CameraParameters::SCENE_MODE_SPORTS ,        ANDROID_CONTROL_SCENE_MODE_SPORTS },
    { CameraParameters::SCENE_MODE_PARTY,          ANDROID_CONTROL_SCENE_MODE_PARTY },
    { CameraParameters::SCENE_MODE_CANDLELIGHT,    ANDROID_CONTROL_SCENE_MODE_CANDLELIGHT },
    { CameraParameters::SCENE_MODE_HDR,            ANDROID_CONTROL_SCENE_MODE_HDR },
};

const CameraContext::QmmfCameraMap<uint8_t> CameraContext::TRUE_FALSE_MAP[] = {
    { CameraParameters::TRUE,  1 },
    { CameraParameters::FALSE, 0 }
};


class FDdata {
public:
  camera_memory_t *camera_memory;
  CameraContext *context;
  bool isInUse;

  FDdata(camera_memory_t *cm, CameraContext *ctx, bool use) {
    camera_memory = cm;
    context = ctx;
    isInUse = use;
  }
};
std::map<uint32_t, FDdata*> g_allocated_fd_buffer_list_;
std::mutex                  g_allocated_fd_lock_;

CameraContext::CameraContext() :
  camera_id_(-1),
  result_cb_(nullptr),
  error_cb_(nullptr),
  libptr_(nullptr),
  camera_device_(nullptr),
  snapshot_frame_id_(0) {
  alloc_device_interface_ = AllocDeviceFactory::CreateAllocDevice();
}

CameraContext::~CameraContext() {
  if (nullptr != alloc_device_interface_) {
    AllocDeviceFactory::DestroyAllocDevice(alloc_device_interface_);
    alloc_device_interface_ = nullptr;
  }
  {
    std::unique_lock < std::mutex > lock(g_allocated_fd_lock_);
    for (auto iter : g_allocated_fd_buffer_list_) {
      int32_t fd = iter.first;
      FDdata* fd_data = iter.second;
      if (fd_data->context == this) {
        QMMF_ERROR("%s: HAL1 buffers are not freed", __func__);
        delete fd_data;
        g_allocated_fd_buffer_list_.erase(fd);
      }
    }
  }
  if(camera_id_ != -1) {
    CloseCamera(camera_id_);
  }
}

/* Internal Helper Functions*/
static void __notify_cb(int32_t msg_type, int32_t ext1, int32_t ext2,
  void *user) {
  QMMF_VERBOSE("%s\n", __FUNCTION__);
  QMMF_VERBOSE("msg_type %d, ext1 %d ext2 %d \n", msg_type, ext1, ext2);
}

static void __data_cb(int32_t msg_type, const camera_memory_t *data,
  uint32_t index, camera_frame_metadata_t *metadata, void *user) {
  CameraContext *camera_context = (CameraContext *)user;
  QMMF_VERBOSE("%s\n", __FUNCTION__);
  QMMF_VERBOSE("msg_type 0x%x, data %p, metadata %p index %d \n", msg_type,
    (uint32_t * )data, (uint32_t * )metadata, index);

  if (msg_type & CAMERA_MSG_PREVIEW_FRAME) {
    QMMF_VERBOSE("%s:%d: CAMERA_MSG_PREVIEW_FRAME data size - %d", __func__,
      __LINE__, data->size);
    if (camera_context != nullptr) {
      auto port = camera_context->GetPortByType(CameraPortType::kPreview);
      if (!port) {
        QMMF_ERROR("%s: Invalid port(%x)", __func__,
          (int32_t) CameraPortType::kPreview);
        return;
      }
      port->StreamCallback(data, 0);
    }
  }

  if (msg_type & CAMERA_MSG_RAW_IMAGE) {
    QMMF_VERBOSE("%s:%d: CAMERA_MSG_RAW_IMAGE", __func__, __LINE__);
  }

  if (msg_type & CAMERA_MSG_POSTVIEW_FRAME) {
    QMMF_VERBOSE("%s:%d: CAMERA_MSG_POSTVIEW_FRAME", __func__, __LINE__);
  }

  if (msg_type & CAMERA_MSG_COMPRESSED_IMAGE) {
    QMMF_VERBOSE("%s:%d: CAMERA_MSG_COMPRESSED_IMAGE", __func__, __LINE__);
    if (camera_context != nullptr) {
      camera_context->SnapshotCallback(data);
    }
  }

  if (msg_type & CAMERA_MSG_VIDEO_FRAME) {
    QMMF_VERBOSE("%s:%d: CAMERA_MSG_VIDEO_FRAME", __func__, __LINE__);
    if (camera_context != nullptr) {
      auto port = camera_context->GetPortByType(CameraPortType::kVideo);
      if (!port) {
        QMMF_ERROR("%s: Invalid port(%x)", __func__,
          (int32_t) CameraPortType::kVideo);
        return;
      }
      port->StreamCallback(data, 0);
    }
  }
}

static void __data_cb_timestamp(nsecs_t timestamp, int32_t msg_type,
  const camera_memory_t *data, uint32_t index, void *user) {
  CameraContext *camera_context = (CameraContext *)user;
  QMMF_VERBOSE("%s\n", __FUNCTION__);
  QMMF_VERBOSE("timestamp %ld msg_type 0x%x, data %p, index %d \n",
    (uint32_t )timestamp, msg_type, (uint32_t * )data, index);

  if (msg_type & CAMERA_MSG_PREVIEW_FRAME) {
    QMMF_VERBOSE("%s:%d: CAMERA_MSG_PREVIEW_FRAME data size - %d", __func__,
      __LINE__, data->size);
    if (camera_context != nullptr) {
      auto port = camera_context->GetPortByType(CameraPortType::kPreview);
      if (!port) {
        QMMF_ERROR("%s: Invalid port(%x)", __func__,
          (int32_t) CameraPortType::kPreview);
        return;
      }
      port->StreamCallback(data, (int64_t)timestamp);
    }
  }

  if (msg_type & CAMERA_MSG_RAW_IMAGE) {
    QMMF_VERBOSE("%s:%d: CAMERA_MSG_RAW_IMAGE", __func__, __LINE__);
  }

  if (msg_type & CAMERA_MSG_POSTVIEW_FRAME) {
    QMMF_VERBOSE("%s:%d: CAMERA_MSG_POSTVIEW_FRAME", __func__, __LINE__);
  }

  if (msg_type & CAMERA_MSG_COMPRESSED_IMAGE) {
    QMMF_VERBOSE("%s:%d: CAMERA_MSG_COMPRESSED_IMAGE", __func__, __LINE__);
    if (camera_context != nullptr) {
      camera_context->SnapshotCallback(data, (int64_t)timestamp);
    }
  }

  if (msg_type & CAMERA_MSG_VIDEO_FRAME) {
    QMMF_VERBOSE("%s:%d: CAMERA_MSG_VIDEO_FRAME", __func__, __LINE__);
    if (camera_context != nullptr) {
      auto port = camera_context->GetPortByType(CameraPortType::kVideo);
      if (!port) {
        QMMF_ERROR("%s: Invalid port(%x)", __func__,
          (int32_t) CameraPortType::kVideo);
        return;
      }
      port->StreamCallback(data, (int64_t)timestamp);
    }
  }
}

static void __put_memory(camera_memory_t *data) {
  QMMF_VERBOSE("E %s data :%p \n", __FUNCTION__, (uint32_t * )data);
  if (!data)
    return;

  {
    std::unique_lock < std::mutex > lock(g_allocated_fd_lock_);
    for (auto iter : g_allocated_fd_buffer_list_) {
      int32_t fd = iter.first;
      FDdata* fd_data = iter.second;
      if (fd_data->isInUse) {
        if (nullptr != fd_data->context->error_cb_) {
          fd_data->context->error_cb_(fd_data->context->camera_id_,
              REMAP_ALL_BUFFERS);
        }
      }
      if (fd_data->camera_memory == data) {
        delete fd_data;
        g_allocated_fd_buffer_list_.erase(fd);
      }
    }
  }

  if (data->data && data->size)
    munmap(data->data, data->size);
  free(data);
  QMMF_VERBOSE("X %s\n", __FUNCTION__);
}

static void __put_memory_heap(camera_memory_t *data) {
  QMMF_VERBOSE("E %s data :%p \n", __FUNCTION__, (uint32_t * )data);
  if (!data)
    return;
  if (data->data)
    free(data->data);
  free(data);
  QMMF_VERBOSE("X %s\n", __FUNCTION__);
}

static void * mapfd(int32_t fd, size_t size) {
  QMMF_VERBOSE("E %s fd %d size %d\n", __FUNCTION__, fd, (uint32_t )size);
  uint32_t offset = 0;
  void* base = NULL;
  if (size == 0) {
    // try to figure out the size automatically
#ifdef HAVE_ANDROID_OS
    // first try the PMEM ioctl
    QMMF_INFO("first try the PMEM ioctl\n");
    pmem_region reg;
    int32_t err = ioctl(fd, PMEM_GET_TOTAL_SIZE, &reg);
    if (err == 0)
    size = reg.len;
#endif
    if (size == 0) { // try fstat
      struct stat sb;
      if (fstat(fd, &sb) == 0)
        size = sb.st_size;
    }
    // if it didn't work, let mmap() fail.
  }
  QMMF_VERBOSE("calling mmap\n");
  base = (uint8_t*)mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
    offset);
  if (base == MAP_FAILED) {
    QMMF_INFO("mmap(fd=%d, size=%u) failed", fd, uint32_t(size));
    close(fd);
    return NULL;
  }
  QMMF_VERBOSE("mmap success base %p\n", (int * )base);
  return base;
}

static camera_memory_t* __get_memory(int fd, size_t buf_size, uint32_t num_bufs,
  void *user) {
  camera_memory_t * handle = NULL;
  QMMF_VERBOSE("%s fd:%d buffsize: %d num_bufs %d\n", __FUNCTION__, fd,
    (int )buf_size, num_bufs);

  CameraContext *camera_context = (CameraContext *)user;
  handle = (camera_memory_t *)malloc(sizeof(camera_memory_t));

  if (fd == -1) {
    QMMF_VERBOSE("buffer size: %d\n", (int )(buf_size * num_bufs));
    handle->data = (void *)malloc(buf_size * num_bufs);
    handle->release = __put_memory_heap;
  } else {
    const size_t pagesize = getpagesize();
    QMMF_VERBOSE("pagesize: %d\n", (int )pagesize);
    buf_size = ((buf_size + pagesize - 1) & ~(pagesize - 1));
    QMMF_VERBOSE("new buf_size: %d caling mapfd\n", buf_size);
    handle->data = mapfd(fd, buf_size);
    QMMF_VERBOSE("after mapfd: %p\n", (int * )handle->data);
    handle->release = __put_memory;

    {
      std::unique_lock < std::mutex > lock(g_allocated_fd_lock_);
      assert(g_allocated_fd_buffer_list_.count(fd) == 0);
      if (g_allocated_fd_buffer_list_.count(fd) == 0) {
        FDdata *fd_data = new FDdata(handle, camera_context, false);
        g_allocated_fd_buffer_list_.emplace(fd, fd_data);
      }
    }
  }

  handle->size = buf_size * num_bufs;
  handle->handle = NULL;
  QMMF_VERBOSE("%s handle :%p \n", __FUNCTION__, (uint32_t * )handle);
  return handle;
}

status_t CameraContext::camera_device_open(uint8_t id) {
  QMMF_INFO("In %s \n", __func__);

  hw_module_t * module;

  uint32_t rc = hw_get_module(CAMERA_HARDWARE_MODULE_ID,
    (const hw_module_t **)&module);
  if (rc < 0 || module == NULL) {
    QMMF_ERROR("Could not load camera HAL module rc %d module %p", rc,
      module);
    return BAD_VALUE;
  }

  char Id[] = {'0' + id, '\0'};
  QMMF_INFO("%s:%d: camera id %s", __func__, __LINE__, Id);

  rc = module->methods->open(module, Id, (hw_device_t **)&camera_device_);
  if (rc < 0 || camera_device_ == NULL) {
    QMMF_ERROR("Could not open camera rc %d camera_device_ %p", rc, camera_device_);
    return BAD_VALUE;
  }

  return NO_ERROR;
}

status_t CameraContext::close_camera_device() {

  QMMF_INFO("In %sn", __func__);
  uint32_t ret = ((hw_device_t *)camera_device_)->close((hw_device_t *)camera_device_);
  QMMF_INFO("X %s ret = %d\n", __func__, ret);
  return NO_ERROR;
}

status_t CameraContext::set_callbacks() {
  QMMF_INFO("E %s \n", __func__);
  ((camera_device_t *)camera_device_)->ops->set_callbacks((camera_device_t *)camera_device_,
    __notify_cb, __data_cb, __data_cb_timestamp, __get_memory, this);
  return NO_ERROR;
}

char* CameraContext::get_parameters() {
  char *parms = ((camera_device_t *)camera_device_)->ops->get_parameters((camera_device_t *)camera_device_);
  return parms;
}

void CameraContext::put_parameters(char *parms) {
  QMMF_INFO("E %s \n", __func__);
  ((camera_device_t *)camera_device_)->ops->put_parameters((camera_device_t *)camera_device_, parms);
  QMMF_INFO("X %s \n", __func__);
}

status_t CameraContext::OpenCamera(const uint32_t camera_id,
  const float frame_rate, const CameraExtraParam& extra_param,
  const ResultCb &cb, // todo map to HAL1 callbacks
  const ErrorCb &errcb) { // todo map to HAL1 callbacks

  int32_t ret = NO_ERROR;
  camera_id_ = camera_id;
  result_cb_ = cb;
  error_cb_ = errcb;

  ret = camera_device_open(camera_id);
  if (ret) {
    QMMF_ERROR("%s:%d: failed: %d ", __func__, __LINE__, ret);
    return BAD_VALUE;
  }

  set_callbacks();

  // Init camera parameters
  mParameters_.unflatten(android::String8(get_parameters()));

  QMMF_INFO("%s:%d: Supported preview sizes: %s ", __func__, __LINE__,
    mParameters_.get("preview-size-values"));
  QMMF_INFO("%s:%d: Supported video   sizes: %s ", __func__, __LINE__,
    mParameters_.get("video-size-values"));

  mParameters_.set("low-power-mode", "enable");
  mParameters_.set("no-display-mode", "1");

  uint32_t msg_type = CAMERA_MSG_ERROR | // notifyCallback
    CAMERA_MSG_SHUTTER | // notifyCallback
    //CAMERA_MSG_FOCUS            | // notifyCallback
    //CAMERA_MSG_ZOOM             | // notifyCallback
    CAMERA_MSG_PREVIEW_FRAME | // dataCallback
    CAMERA_MSG_VIDEO_FRAME | // data_timestamp_callback
    CAMERA_MSG_POSTVIEW_FRAME   | // dataCallback
    CAMERA_MSG_RAW_IMAGE        | // dataCallback
    CAMERA_MSG_COMPRESSED_IMAGE | // dataCallback
    CAMERA_MSG_RAW_IMAGE_NOTIFY | // dataCallback
    CAMERA_MSG_PREVIEW_METADATA | // dataCallback
    //CAMERA_MSG_FOCUS_MOVE       | // notifyCallback
    //CAMERA_MSG_STATS_DATA       |
    CAMERA_MSG_META_DATA        |
    CAMERA_MSG_ERROR;

  ((camera_device_t *)camera_device_)->ops->enable_msg_type((camera_device_t *)camera_device_, msg_type);

  // Create Preview port
  std::shared_ptr<CameraPort> port = std::make_shared < PreviewPort
    > (CameraPortType::kPreview, this);
  assert(port.get() != nullptr);
  // Add port to list of active ports.
  ports_.emplace((uint32_t)CameraPortType::kPreview, port);

  // Create Video port
  port = std::make_shared < VideoPort > (CameraPortType::kVideo, this);
  assert(port.get() != nullptr);
  // Add port to list of active ports.
  ports_.emplace((uint32_t)CameraPortType::kVideo, port);

  QMMF_INFO("%s: CameraContext(%u) Opened Successfully!", __func__, camera_id_);
  return NO_ERROR;
}

status_t CameraContext::CloseCamera(const uint32_t camera_id) {

  int32_t ret = NO_ERROR;
  assert(camera_id_ == camera_id);

  ret = close_camera_device();
  if (ret) {
    QMMF_ERROR("%s:%d: failed: %d ", __func__, __LINE__, ret);
    return BAD_VALUE;
  }
  camera_id_ = -1;

  QMMF_INFO("%s: CameraContext(%u) Closed Successfully!", __func__, camera_id_);
  return NO_ERROR;
}

status_t CameraContext::WaitAecToConverge(const uint32_t timeout) {

  return NO_ERROR;
}

status_t CameraContext::ConfigImageCapture(const SnapshotParam& param,
                                           const ImageExtraParam &xtraparam) {

  QMMF_DEBUG("%s Enter ", __func__);

  if (param.mode == ImageMode::kSnapshot) {
    std::unique_lock<std::mutex> lock(capture_lock_);

    snapshot_param_ = param;

    if(param.format == BufferFormat::kBLOB) {
      mParameters_.setPictureFormat(CameraParameters::PIXEL_FORMAT_JPEG);
    } else if(param.format == BufferFormat::kNV12 ||
              param.format == BufferFormat::kNV21) {
      mParameters_.setPictureFormat(CameraParameters::PIXEL_FORMAT_YUV420SP);
    }
    mParameters_.setPictureSize(param.width, param.height);
    ApplyParameters();
  }

  QMMF_INFO("%s: Exit", __func__);
  return NO_ERROR;
}

status_t CameraContext::CaptureImage(const SnapshotType type,
                                     const uint32_t n_images,
                                     const std::vector<CameraMetadata> &meta,
                                     const StreamSnapshotCb& cb) {

  QMMF_INFO("%s: Enter num_images - %d", __func__, n_images);

  int32_t ret = NO_ERROR;
  client_snapshot_cb_ = cb;
  capture_cnt_ = 0;
  uint32_t img_cnt = (n_images == 0) ? 1 : n_images;

  if (snapshot_param_.width == 0 || snapshot_param_.height == 0) {
    QMMF_ERROR("%s: No snapshot stream available", __func__);
    return BAD_VALUE;
  }

  if (snapshot_param_.mode == ImageMode::kSnapshot) {
    int64_t last_frame_number;
    for (uint32_t i = 0; i < img_cnt; i++) {
      QMMF_INFO("%s: HAL take picture", __func__);
      ((camera_device_t *)camera_device_)->
          ops->take_picture((camera_device_t *)camera_device_);
    }
  }

  QMMF_INFO("%s: Exit", __func__);

  return NO_ERROR;
}

status_t CameraContext::CancelCaptureImage(const bool cache) {

  // todo wait until image capture is done

  return NO_ERROR;
}

status_t CameraContext::CreateStream(const StreamParam& param,
  const VideoExtraParam& extra_param) {

  if (streams_params_.size() > 2) {
    QMMF_ERROR("%s: Only two streams are available", __func__);
    return BAD_VALUE; // only 2 streams are allowed
  }
  streams_params_.emplace(param.id, param);

  QMMF_INFO("%s: width = %d, height = %d, format = %d, fps = %f", __func__,
    param.width, param.height, (int32_t) param.format, param.framerate);

  return NO_ERROR;
}

status_t CameraContext::DeleteStream(const uint32_t track_id) {

  QMMF_INFO("Enter %s \n", __func__);
  std::unique_lock < std::mutex > lock(buffer_lock_);
  std::chrono::nanoseconds wait_time(4000000000);

  for (auto iter : ports_) {
    auto port = iter.second;
    if (port->GetPortId() == track_id) {
      if (port->getPortState() != PortState::PORT_CREATED) {
        QMMF_ERROR("%s: The stream is running, please stop it before delete",
          __func__);
        return BAD_VALUE;
      }
      while (port->buffer_count_ > 0) {
        auto ret = port->wait_for_buffer_.WaitFor(lock, wait_time);
        if (ret != 0) {
          QMMF_ERROR(
            "%s: Wait for buffer return timed out. Pending buffers: %d",
            __func__, port->buffer_count_);
          return TIMED_OUT;
        }
      }
    }
  }

  streams_params_.erase(track_id);
  QMMF_INFO("Exit %s \n", __func__);
  return NO_ERROR;
}

status_t CameraContext::AddConsumer(const uint32_t& track_id,
  sp<IBufferConsumer>& consumer) {

  // Save consumer in a vector since the port is not initialized yet
  consumers_.emplace(track_id, consumer);
  return NO_ERROR;
}

status_t CameraContext::RemoveConsumer(const uint32_t& track_id,
  sp<IBufferConsumer>& consumer) {

  // Check whether the consumer is used
  for (auto iter : ports_) {
    auto port = iter.second;
    if (port->GetPortId() == track_id) {
      if (port->getPortState() == PortState::PORT_STARTED) {
        QMMF_ERROR(
          "%s: The stream is running, please stop it before RemoveConsumer",
          __func__);
        return BAD_VALUE;
      }
      break;
    }
  }
  consumers_.erase(track_id);
  return NO_ERROR;
}

std::shared_ptr<CameraPort> CameraContext::GetPortById(
  const uint32_t& track_id) {

  for (auto iter : ports_) {
    auto port = iter.second;
    if (port->GetPortId() == track_id) {
      return port;
    }
  }

  return nullptr;
}

std::shared_ptr<CameraPort> CameraContext::GetPortByType(
  const CameraPortType port_type) {

  assert(ports_.count((uint32_t )port_type) > 0);
  auto port = ports_[(uint32_t)port_type];

  QMMF_VERBOSE("%s: Found port for track_id(%x)", __func__, (int32_t )port_type);
  return port;
}

std::shared_ptr<CameraPort> CameraContext::GetFreePort() {

  for (auto iter : ports_) {
    auto port = iter.second;
    if (port->getPortState() == PortState::PORT_CREATED) {
      return port;
    }
  }

  return nullptr;
}

status_t CameraContext::ApplyParameters() {

  auto ret = ((camera_device_t *)camera_device_)->ops->set_parameters((camera_device_t *)camera_device_,
    mParameters_.flatten().string());
  return ret;
}

status_t CameraContext::SetFps(float fps) {

  mParameters_.setPreviewFrameRate(fps);
  std::string fps_range = "";
  fps_range += std::to_string((int)(fps*1000.0f));
  fps_range += ",";
  fps_range += std::to_string((int)(fps*1000.0f));
  mParameters_.set(CameraParameters::KEY_PREVIEW_FPS_RANGE, fps_range.c_str());
  ApplyParameters();

  return NO_ERROR;
}

status_t CameraContext::StartStream(const uint32_t track_id) {

  QMMF_INFO("Enter %s \n", __func__);
  StreamParam param = streams_params_[track_id];
  sp<IBufferConsumer>& consumer = consumers_[track_id];
  auto port = GetFreePort();
  if (!port) {
    QMMF_ERROR("%s: Invalid Port", __func__);
    return BAD_VALUE;
  }

  auto ret = port->Init(param);
  assert(ret == NO_ERROR);

  port->AddConsumer(consumer);
  assert(ret == NO_ERROR);

  // Get higher port fps
  float max_fps = 0.0f;
  for (auto iter : ports_) {
    auto p = iter.second;
    float framerate = p->GetPortFramerate();
    if (max_fps < framerate) {
      max_fps = framerate;
    }
  }

  QMMF_INFO("%s Set fps - %f", __func__, max_fps);
  SetFps(max_fps);
  ret = port->Start();
  assert(ret == NO_ERROR);

  QMMF_INFO("Exit %s \n", __func__);
  return NO_ERROR;
}

status_t CameraContext::StopStream(const uint32_t track_id) {

  QMMF_INFO("Enter %s \n", __func__);

  std::shared_ptr<CameraPort> port = nullptr;
  bool isBothPortsActive = true;
  for (auto iter : ports_) {
    auto p = iter.second;
    if (p->GetPortId() == track_id) {
      port = p;
    }
    isBothPortsActive &= (p->getPortState() == PortState::PORT_STARTED);
  }
  assert(port);

  if (port->GetPortType() == CameraPortType::kPreview) {
    if (!isBothPortsActive) {
      QMMF_INFO("%s Stop Preview only \n", __func__);
      auto ret = port->Stop();
      assert(ret == NO_ERROR);
      port->RemoveConsumer(consumers_[track_id]);
      assert(ret == NO_ERROR);
      ret = port->DeInit();
      assert(ret == NO_ERROR);
    } else {
      QMMF_INFO("%s Stop Preview and restart Video \n", __func__);
      auto port = GetPortByType(CameraPortType::kVideo);
      if (!port) {
        QMMF_ERROR("%s: Invalid port", __func__);
        return BAD_VALUE;
      }
      auto ret = port->Stop();
      assert(ret == NO_ERROR);
      port->RemoveConsumer(consumers_[port->GetPortId()]);
      assert(ret == NO_ERROR);
      ret = port->DeInit();
      assert(ret == NO_ERROR);
      port = GetPortByType(CameraPortType::kPreview);
      if (!port) {
        QMMF_ERROR("%s: Invalid port", __func__);
        return BAD_VALUE;
      }
      ret = port->Stop();
      assert(ret == NO_ERROR);
      port->RemoveConsumer(consumers_[port->GetPortId()]);
      assert(ret == NO_ERROR);
      ret = port->DeInit();
      assert(ret == NO_ERROR);

      StreamParam stream_param = { };
      for (auto iter2 : streams_params_) {
        auto param = iter2.second;
        if (param.id != track_id) {
          stream_param = param;
          break;
        }
      }
      sp<IBufferConsumer>& consumer = consumers_[stream_param.id];
      auto new_port = GetFreePort();
      if (!new_port) {
        QMMF_ERROR("%s: Invalid Port", __func__);
        return BAD_VALUE;
      }

      ret = new_port->Init(stream_param);
      assert(ret == NO_ERROR);

      port->AddConsumer(consumer);
      assert(ret == NO_ERROR);

      // Get higher port fps
      float max_fps = 0.0f;
      for (auto iter : ports_) {
        auto p = iter.second;
        float framerate = p->GetPortFramerate();
        if (max_fps < framerate) {
          max_fps = framerate;
        }
      }

      QMMF_INFO("%s Set fps - %f", __func__, max_fps);
      SetFps(max_fps);
      ret = new_port->Start();
      assert(ret == NO_ERROR);
    }
  } else if (port->GetPortType() == CameraPortType::kVideo) {
    QMMF_INFO("%s Stop Video only \n", __func__);
    auto ret = port->Stop();
    assert(ret == NO_ERROR);
    port->RemoveConsumer(consumers_[track_id]);
    assert(ret == NO_ERROR);
    ret = port->DeInit();
    assert(ret == NO_ERROR);
  }

  QMMF_INFO("Exit %s \n", __func__);
  return NO_ERROR;
}

status_t CameraContext::PauseStream(const uint32_t /* track_id */) {

  // todo wait until image capture is done

  return NO_ERROR;
}

status_t CameraContext::ResumeStream(const uint32_t /* track_id */) {

  // todo wait until image capture is done

  return NO_ERROR;
}

status_t CameraContext::SetCameraParam(const CameraMetadata &meta) {

  if (meta.exists(ANDROID_CONTROL_AWB_MODE)) {
    uint8_t awb_mode = meta.find(ANDROID_CONTROL_AWB_MODE).data.u8[0];
    const char *val = lookupNameByValue(WHITE_BALANCE_MODES_MAP,
      PARAM_MAP_SIZE(WHITE_BALANCE_MODES_MAP), (uint32_t)awb_mode);
    if (val) {
      mParameters_.set(android::CameraParameters::KEY_WHITE_BALANCE, val);
    } else {
      QMMF_ERROR("failed: awb_mode %d not supported ", awb_mode);
    }
  }

  if (meta.exists(ANDROID_CONTROL_EFFECT_MODE)) {
    uint8_t effect = meta.find(ANDROID_CONTROL_EFFECT_MODE).data.u8[0];
    const char *val = lookupNameByValue(EFFECT_MODES_MAP,
      PARAM_MAP_SIZE(EFFECT_MODES_MAP), (uint32_t)effect);
    if (val) {
      mParameters_.set(android::CameraParameters::KEY_EFFECT, val);
    } else {
      QMMF_ERROR("failed: effect_mode %d not supported ", effect);
    }
  }

  if (meta.exists(ANDROID_CONTROL_AE_ANTIBANDING_MODE)) {
    uint8_t antibanding =
      meta.find(ANDROID_CONTROL_AE_ANTIBANDING_MODE).data.u8[0];
    const char *val = lookupNameByValue(ANTIBANDING_MODES_MAP,
      PARAM_MAP_SIZE(ANTIBANDING_MODES_MAP), (uint32_t)antibanding);
    if (val) {
      mParameters_.set(android::CameraParameters::KEY_ANTIBANDING, val);
    } else {
      QMMF_ERROR("failed: antibanding %d not supported ", antibanding);
    }
  }

  if (meta.exists(ANDROID_CONTROL_SCENE_MODE)) {
    uint8_t scene_mode = meta.find(ANDROID_CONTROL_SCENE_MODE).data.u8[0];
    const char *val = lookupNameByValue(SCENE_MODES_MAP,
      PARAM_MAP_SIZE(SCENE_MODES_MAP), (uint32_t)scene_mode);
    if (val) {
      mParameters_.set(android::CameraParameters::KEY_SCENE_MODE, val);
    } else {
      QMMF_ERROR("failed: scene_mode %d not supported ", scene_mode);
    }
  }

  if (meta.exists(ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION)) {
    int32_t ae_comp =
      meta.find(ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION).data.i32[0];
    mParameters_.set(android::CameraParameters::KEY_EXPOSURE_COMPENSATION,
      ae_comp);
  }

  if (meta.exists(ANDROID_CONTROL_AE_LOCK)) {
    uint8_t ae_lock = meta.find(ANDROID_CONTROL_AE_LOCK).data.u8[0];
    const char *val = lookupNameByValue(TRUE_FALSE_MAP,
      PARAM_MAP_SIZE(TRUE_FALSE_MAP), (uint32_t)ae_lock);
    if (val) {
      mParameters_.set(android::CameraParameters::KEY_AUTO_EXPOSURE_LOCK, val);
    } else {
      QMMF_ERROR("failed: ae_lock %d not supported ", ae_lock);
    }
  }

  if (meta.exists(ANDROID_CONTROL_AWB_LOCK)) {
    uint8_t awb_lock = meta.find(ANDROID_CONTROL_AWB_LOCK).data.u8[0];
    const char *val = lookupNameByValue(TRUE_FALSE_MAP,
      PARAM_MAP_SIZE(TRUE_FALSE_MAP), (uint32_t)awb_lock);
    if (val) {
      mParameters_.set(android::CameraParameters::KEY_AUTO_WHITEBALANCE_LOCK,
        val);
    } else {
      QMMF_ERROR("failed: awb_lock %d not supported ", awb_lock);
    }
  }
  ApplyParameters();

  return NO_ERROR;
}

status_t CameraContext::GetCameraParam(CameraMetadata &meta) {

  if (mParameters_.get(android::CameraParameters::KEY_WHITE_BALANCE)) {
    uint8_t val = (uint8_t)lookupAttr(WHITE_BALANCE_MODES_MAP,
      PARAM_MAP_SIZE(WHITE_BALANCE_MODES_MAP),
      mParameters_.get(android::CameraParameters::KEY_WHITE_BALANCE));
    metadata_.update(ANDROID_CONTROL_AWB_MODE, &val, 1);
  }

  if (mParameters_.get(android::CameraParameters::KEY_EFFECT)) {
    uint8_t val = (uint8_t)lookupAttr(EFFECT_MODES_MAP,
      PARAM_MAP_SIZE(EFFECT_MODES_MAP),
      mParameters_.get(android::CameraParameters::KEY_EFFECT));
    metadata_.update(ANDROID_CONTROL_EFFECT_MODE, &val, 1);
  }

  if (mParameters_.get(android::CameraParameters::KEY_ANTIBANDING)) {
    uint8_t val = (uint8_t)lookupAttr(ANTIBANDING_MODES_MAP,
      PARAM_MAP_SIZE(ANTIBANDING_MODES_MAP),
      mParameters_.get(android::CameraParameters::KEY_ANTIBANDING));
    metadata_.update(ANDROID_CONTROL_AE_ANTIBANDING_MODE, &val, 1);
  }

  if (mParameters_.get(android::CameraParameters::KEY_SCENE_MODE)) {
    uint8_t val = (uint8_t)lookupAttr(SCENE_MODES_MAP,
      PARAM_MAP_SIZE(SCENE_MODES_MAP),
      mParameters_.get(android::CameraParameters::KEY_SCENE_MODE));
    metadata_.update(ANDROID_CONTROL_SCENE_MODE, &val, 1);
  }

  if (mParameters_.get(android::CameraParameters::KEY_EXPOSURE_COMPENSATION)) {
    int32_t val = mParameters_.getInt(
      android::CameraParameters::KEY_EXPOSURE_COMPENSATION);
    metadata_.update(ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION, &val, 1);
  }

  if (mParameters_.get(android::CameraParameters::KEY_AUTO_EXPOSURE_LOCK)) {
    uint8_t val = (uint8_t)lookupAttr(TRUE_FALSE_MAP,
      PARAM_MAP_SIZE(TRUE_FALSE_MAP),
      mParameters_.get(android::CameraParameters::KEY_AUTO_EXPOSURE_LOCK));
    metadata_.update(ANDROID_CONTROL_AE_LOCK, &val, 1);
  }

  if (mParameters_.get(android::CameraParameters::KEY_AUTO_WHITEBALANCE_LOCK)) {
    uint8_t val = (uint8_t)lookupAttr(TRUE_FALSE_MAP,
      PARAM_MAP_SIZE(TRUE_FALSE_MAP),
      mParameters_.get(android::CameraParameters::KEY_AUTO_WHITEBALANCE_LOCK));
    metadata_.update(ANDROID_CONTROL_AWB_LOCK, &val, 1);
  }

  meta.append(metadata_);
  return NO_ERROR;
}

status_t CameraContext::GetDefaultCaptureParam(CameraMetadata &meta) {

  GetCameraParam(meta);
  return NO_ERROR;
}

status_t CameraContext::GetCameraCharacteristics(CameraMetadata &meta) {

  if (!metadata_.exists(ANDROID_SCALER_AVAILABLE_PROCESSED_SIZES)) {
    Vector<Size> picture_sizes;
    mParameters_.getSupportedPictureSizes(picture_sizes);
    metadata_.update(ANDROID_SCALER_AVAILABLE_PROCESSED_SIZES,
      (int32_t *)picture_sizes.array(), picture_sizes.size() * 2);
  }

  if (!metadata_.exists(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS)) {
    Vector<Size> preview_sizes;
    mParameters_.getSupportedPreviewSizes(preview_sizes);
    Vector<Size> video_sizes;
    mParameters_.getSupportedVideoSizes(video_sizes);
    Vector<int> preview_formats;
    mParameters_.getSupportedPreviewFormats(preview_formats);

    Vector<int32_t> available_stream_configs;
    for (auto& p_size : preview_sizes) {
      for (auto& v_size : video_sizes) {
        if (p_size.width == v_size.width && p_size.height == v_size.height) {
          for (auto& format : preview_formats) {
            available_stream_configs.add(format);
            available_stream_configs.add(p_size.width);
            available_stream_configs.add(p_size.height);
            available_stream_configs.add(
                ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT);
          }
        }
      }
    }
    metadata_.update(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS,
                     available_stream_configs.array(),
                     available_stream_configs.size());
  }

  if (!metadata_.exists(ANDROID_SCALER_AVAILABLE_RAW_SIZES)) {
    const char *p = mParameters_.get(KEY_QTI_RAW_PICUTRE_SIZE);
    if (p) {
      uint32_t available_raw_sizes[2];
      ParsePair(p, &available_raw_sizes[0], &available_raw_sizes[1], 'x');
      metadata_.update(ANDROID_SCALER_AVAILABLE_RAW_SIZES,
                       reinterpret_cast<int32_t *>(available_raw_sizes), 2);
    } else {
      QMMF_ERROR("failed: raw capture not supported ");
    }
  }

  if (!meta.exists(ANDROID_CONTROL_AWB_AVAILABLE_MODES)) {
    const char *p = mParameters_.get(
      android::CameraParameters::KEY_SUPPORTED_WHITE_BALANCE);

    std::vector<std::string> supported_strings;
    std::vector<uint8_t> supported_values;
    ParseList(p, supported_strings);

    for (auto& entry : supported_strings) {
      int32_t val = lookupAttr(WHITE_BALANCE_MODES_MAP,
        PARAM_MAP_SIZE(WHITE_BALANCE_MODES_MAP), entry.c_str());
      if (val != NAME_NOT_FOUND) {
        supported_values.push_back((uint8_t)val);
      } else {
        QMMF_ERROR("failed: awb mode %s not supported ", entry.c_str());
      }
    }

    metadata_.update(ANDROID_CONTROL_AWB_AVAILABLE_MODES,
      supported_values.data(), supported_values.size());
  }

  if (!meta.exists(ANDROID_CONTROL_AVAILABLE_EFFECTS)) {
    const char *p = mParameters_.get(
      android::CameraParameters::KEY_SUPPORTED_EFFECTS);

    std::vector<std::string> supported_strings;
    std::vector<uint8_t> supported_values;
    ParseList(p, supported_strings);

    for (auto& entry : supported_strings) {
      int32_t val = lookupAttr(EFFECT_MODES_MAP,
        PARAM_MAP_SIZE(EFFECT_MODES_MAP), entry.c_str());
      if (val != NAME_NOT_FOUND) {
        supported_values.push_back((uint8_t)val);
      } else {
        QMMF_ERROR("failed: effect %s not supported ", entry.c_str());
      }
    }

    metadata_.update(ANDROID_CONTROL_AVAILABLE_EFFECTS,
      supported_values.data(), supported_values.size());
  }

  if (!meta.exists(ANDROID_CONTROL_AVAILABLE_SCENE_MODES)) {
    const char *p = mParameters_.get(
        android::CameraParameters::KEY_SUPPORTED_SCENE_MODES);

    std::vector<std::string> supported_strings;
    std::vector<uint8_t> supported_values;
    ParseList(p, supported_strings);

    for (auto& entry : supported_strings) {
      int32_t val = lookupAttr(SCENE_MODES_MAP, PARAM_MAP_SIZE(SCENE_MODES_MAP),
        entry.c_str());
      if (val != NAME_NOT_FOUND) {
        supported_values.push_back((uint8_t)val);
      } else {
        QMMF_ERROR("failed: scene mode %s not supported ", entry.c_str());
      }
    }

    metadata_.update(ANDROID_CONTROL_AVAILABLE_SCENE_MODES,
      supported_values.data(), supported_values.size());
  }

  if (!meta.exists(ANDROID_CONTROL_AE_COMPENSATION_RANGE)) {
    int32_t exposureCompensationRange[2];
    exposureCompensationRange[0] = mParameters_.getInt(
      android::CameraParameters::KEY_MIN_EXPOSURE_COMPENSATION);
    exposureCompensationRange[1] = mParameters_.getInt(
      android::CameraParameters::KEY_MAX_EXPOSURE_COMPENSATION);

    metadata_.update(ANDROID_CONTROL_AE_COMPENSATION_RANGE,
      exposureCompensationRange, 2);
  }

  if (!meta.exists(ANDROID_CONTROL_AE_COMPENSATION_STEP)) {
    float step = mParameters_.getFloat(
      android::CameraParameters::KEY_EXPOSURE_COMPENSATION_STEP);

    camera_metadata_rational exposureCompensationStep;
    exposureCompensationStep.numerator = (int32_t)(step * 1024.0);
    exposureCompensationStep.denominator = 1024;

    metadata_.update(ANDROID_CONTROL_AE_COMPENSATION_STEP,
      &exposureCompensationStep, 1);
  }

  if (!meta.exists(ANDROID_CONTROL_AE_LOCK_AVAILABLE)) {
    const char *p = mParameters_.get(
      android::CameraParameters::KEY_AUTO_EXPOSURE_LOCK_SUPPORTED);

    std::vector<std::string> supported_strings;
    std::vector<uint8_t> supported_values;
    ParseList(p, supported_strings);

    for (auto& entry : supported_strings) {
      int32_t val = lookupAttr(TRUE_FALSE_MAP, PARAM_MAP_SIZE(TRUE_FALSE_MAP),
        entry.c_str());
      if (val != NAME_NOT_FOUND) {
        supported_values.push_back((uint8_t)val);
      } else {
        QMMF_ERROR("failed: AE Lock %s not supported ", entry.c_str());
      }
    }

    metadata_.update(ANDROID_CONTROL_AE_LOCK_AVAILABLE,
      supported_values.data(), supported_values.size());
  }

  if (!meta.exists(ANDROID_CONTROL_AWB_LOCK_AVAILABLE)) {
    const char *p = mParameters_.get(
        android::CameraParameters::KEY_AUTO_WHITEBALANCE_LOCK_SUPPORTED);

    std::vector<std::string> supported_strings;
    std::vector<uint8_t> supported_values;
    ParseList(p, supported_strings);

    for (auto& entry : supported_strings) {
      int32_t val = lookupAttr(TRUE_FALSE_MAP, PARAM_MAP_SIZE(TRUE_FALSE_MAP),
        entry.c_str());
      if (val != NAME_NOT_FOUND) {
        supported_values.push_back((uint8_t)val);
      } else {
        QMMF_ERROR("failed: AWB Lock %s not supported ", entry.c_str());
      }
    }

    metadata_.update(ANDROID_CONTROL_AWB_LOCK_AVAILABLE,
      supported_values.data(), supported_values.size());
  }

  meta.append(metadata_);
  return NO_ERROR;
}

status_t CameraContext::ReturnAllImageCaptureBuffers() {

  QMMF_DEBUG("%s: Enter", __func__);
  status_t ret = NO_ERROR;
  for (int i = 0; i < snapshot_buffer_list_.size(); i++) {
    auto entry = snapshot_buffer_list_.begin();
    ret = ReturnImageCaptureBuffer(0, entry->first);
    assert(ret == NO_ERROR);
  }
  QMMF_DEBUG("%s: Exit", __func__);
  return ret;
}

status_t CameraContext::ReturnImageCaptureBuffer(const uint32_t camera_id,
  const int32_t buffer_id) {
  QMMF_DEBUG("%s: Enter", __func__);
  if (snapshot_buffer_list_.find(buffer_id) == snapshot_buffer_list_.end()) {
    QMMF_ERROR("%s: buffer_id(%u) is not valid!!", __func__, buffer_id);
    return BAD_VALUE;
  }

  if (snapshot_hal_buff_list_.find(buffer_id) ==
      snapshot_hal_buff_list_.end()) {
    QMMF_ERROR("%s: buffer_id(%u) is not valid!!", __func__, buffer_id);
    return BAD_VALUE;
  }

  StreamBuffer buffer = snapshot_buffer_list_.find(buffer_id)->second;
  assert(buffer.fd == buffer_id);

  const camera_memory_t *data = snapshot_hal_buff_list_.find(buffer_id)->second;

  QMMF_VERBOSE("%s: SnapshotBuffer(0x%p) fd: %d", __func__, buffer.handle,
      buffer.fd);

  munmap(buffer.data, buffer.size);
  alloc_device_interface_->FreeBuffer(buffer.handle);

  snapshot_buffer_list_.erase(buffer_id);
  snapshot_hal_buff_list_.erase(buffer_id);

  ((camera_device_t *)camera_device_)->ops->release_snapshot_frame(
    (camera_device_t *)camera_device_, data);

  return NO_ERROR;
}

std::vector<int32_t>& CameraContext::GetSupportedFps() {

  std::vector<int32_t> val;
  // Not supported
  assert(0);
  return val;
}

status_t CameraContext::PopulateBufferMeta(BufferMeta &info,
                                           IBufferHandle &handle,
                                           uint32_t width,
                                           uint32_t height) {
  uint32_t stride, scanline;
  auto ret = alloc_device_interface_->Perform(handle,
                              IAllocDevice::AllocDeviceAction::GetStride,
                              static_cast<void*>(&stride));

  if (MemAllocError::kAllocOk != ret) {
    QMMF_ERROR("%s: Error in GetStrideAndHeightFromHandle() : %d\n", __func__,
      (int32_t) ret);
    return BAD_VALUE;
  }

  ret = alloc_device_interface_->Perform(handle,
                              IAllocDevice::AllocDeviceAction::GetAlignedHeight,
                              static_cast<void*>(&scanline));
  if (MemAllocError::kAllocOk != ret) {
    QMMF_ERROR("%s: Error in GetStrideAndHeightFromHandle() : %d\n", __func__,
      (int32_t) ret);
    return BAD_VALUE;
  }

  QMMF_DEBUG("%s: format(0x%x)", __func__, handle->GetFormat());

  switch (handle->GetFormat()) {
    case HAL_PIXEL_FORMAT_BLOB:
      info.format = BufferFormat::kBLOB;
      info.n_planes = 1;
      info.planes[0].width = width;
      info.planes[0].height = height;
      info.planes[0].stride = stride;
      info.planes[0].scanline = scanline;
      info.planes[0].size = stride * scanline;
      info.planes[0].offset = 0;
      break;
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS:
    case HAL_PIXEL_FORMAT_NV12_ENCODEABLE:
    case HAL_PIXEL_FORMAT_YCbCr_420_888:
    case HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED:
      info.format = BufferFormat::kNV12;
      info.n_planes = 2;
      info.planes[0].width = width;
      info.planes[0].height = height;
      info.planes[0].stride = stride;
      info.planes[0].scanline = scanline;
      info.planes[0].size = stride * scanline;
      info.planes[0].offset = 0;
      info.planes[1].width = width;
      info.planes[1].height = height / 2;
      info.planes[1].stride = stride;
      info.planes[1].scanline = scanline / 2;
      info.planes[1].size = stride * (scanline / 2);
      info.planes[1].offset = stride * scanline;
      break;
    case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC:
      info.format = BufferFormat::kNV12UBWC;
      info.n_planes = 2;
      info.planes[0].width = width;
      info.planes[0].height = height;
      info.planes[0].stride = stride;
      info.planes[0].scanline = scanline;
      info.planes[0].size = stride * scanline;
      info.planes[0].offset = 0;
      info.planes[1].width = width;
      info.planes[1].height = height / 2;
      info.planes[1].stride = stride;
      info.planes[1].scanline = scanline / 2;
      info.planes[1].size = stride * (scanline / 2);
      info.planes[1].offset = stride * scanline;
      break;
    case HAL_PIXEL_FORMAT_YCbCr_422_888:
    case HAL_PIXEL_FORMAT_YCbCr_422_SP:
      info.format = BufferFormat::kNV16;
      info.n_planes = 2;
      info.planes[0].width = width;
      info.planes[0].height = height;
      info.planes[0].stride = stride;
      info.planes[0].scanline = scanline;
      info.planes[0].size = stride * scanline;
      info.planes[0].offset = 0;
      info.planes[1].width = width;
      info.planes[1].height = height;
      info.planes[1].stride = stride;
      info.planes[1].scanline = scanline;
      info.planes[1].size = stride * scanline;
      info.planes[1].offset = stride * scanline;
      break;
    case HAL_PIXEL_FORMAT_NV21_ZSL:
      info.format = BufferFormat::kNV21;
      info.n_planes = 2;
      info.planes[0].width = width;
      info.planes[0].height = height;
      info.planes[0].stride = stride;
      info.planes[0].scanline = scanline;
      info.planes[0].size = stride * scanline;
      info.planes[0].offset = 0;
      info.planes[1].width = width;
      info.planes[1].height = height / 2;
      info.planes[1].stride = stride;
      info.planes[1].scanline = scanline / 2;
      info.planes[1].size = stride * (scanline / 2);
      info.planes[1].offset = stride * scanline;
      break;
    case HAL_PIXEL_FORMAT_RAW8:
      info.format = BufferFormat::kRAW8;
      info.n_planes = 1;
      info.planes[0].width = width;
      info.planes[0].height = height;
      info.planes[0].stride = stride;
      info.planes[0].scanline = scanline;
      info.planes[0].size = stride * scanline;
      info.planes[0].offset = 0;
      break;
    case HAL_PIXEL_FORMAT_RAW10:
      info.format = BufferFormat::kRAW10;
      info.n_planes = 1;
      info.planes[0].width = width;
      info.planes[0].height = height;
      info.planes[0].stride = stride;
      info.planes[0].scanline = scanline;
      info.planes[0].size = stride * scanline;
      info.planes[0].offset = 0;
      break;
    case HAL_PIXEL_FORMAT_RAW12:
      info.format = BufferFormat::kRAW12;
      info.n_planes = 1;
      info.planes[0].width = width;
      info.planes[0].height = height;
      info.planes[0].stride = stride;
      info.planes[0].scanline = scanline;
      info.planes[0].size = stride * scanline;
      info.planes[0].offset = 0;
      break;
    case HAL_PIXEL_FORMAT_RAW16:
      info.format = BufferFormat::kRAW16;
      info.n_planes = 1;
      info.planes[0].width = width;
      info.planes[0].height = height;
      info.planes[0].stride = stride;
      info.planes[0].scanline = scanline;
      info.planes[0].size = stride * scanline;
      info.planes[0].offset = 0;
      break;
    default:
      QMMF_ERROR("%s: Unsupported format: %d\n", __func__, handle->GetFormat());
      return BAD_VALUE;
  }

  QMMF_DEBUG("%s: format: %d ", __func__, (int32_t) info.format);
  for (int i = 0; i < info.n_planes; i++) {
    QMMF_DEBUG(
      "%s: plane[%d]: dim: %dx%d stride: %d scanline: %d size: %d offset: %d ",
      __func__, i, info.planes[i].width, info.planes[i].height,
      info.planes[i].stride, info.planes[i].scanline,
      info.planes[i].size, info.planes[i].offset);
  }

  return NO_ERROR;
}

status_t CameraContext::SnapshotCallback(const camera_memory_t *data,
  int64_t timestamp) {
  QMMF_DEBUG("%s: data: %p handle: %p size: %d", __func__, data->data,
      data->handle, data->size);

  //struct gbm_bo *bo = reinterpret_cast< struct gbm_bo *>(data->data);
  struct gbm_bo *bo = reinterpret_cast< struct gbm_bo *>(data->handle);

  int32_t fd = gbm_bo_get_fd(bo);
  void* base = (uint8_t*)mmap(0, data->size, PROT_READ | PROT_WRITE, MAP_SHARED, fd,0);

  // todo: fix me when camera HAL1 fix format in GBM buffer
  QMMF_DEBUG("%s: GBM format: %x", __func__, bo->format);
  bo->format = GBM_FORMAT_BLOB;

  StreamBuffer buffer { };
  buffer.fd = fd;
  buffer.data = base;
  buffer.size = data->size;
  buffer.frame_number = ++snapshot_frame_id_;
  buffer.timestamp = timestamp;
  buffer.camera_id = camera_id_;
  alloc_device_interface_->ImportBuffer(buffer.handle, bo, fd);
  auto ret = PopulateBufferMeta(buffer.info, buffer.handle, data->size, 1);
  assert(ret == NO_ERROR);

  snapshot_buffer_list_.insert(std::make_pair(buffer.fd, buffer));
  snapshot_hal_buff_list_.insert(std::make_pair(buffer.fd, data));

  assert(client_snapshot_cb_ != nullptr);
  client_snapshot_cb_(snapshot_frame_id_, buffer);

  return NO_ERROR;
}

status_t CameraContext::ReturnStreamBuffer(StreamBuffer buffer) {
  QMMF_DEBUG("%s: camera_id: %d, stream_id: %d, buffer: %p ts: %lld "
    "frame_number: %d", __func__, buffer.camera_id, buffer.stream_id,
    buffer.handle, buffer.timestamp, buffer.frame_number);

  // Not supported. Used for postproc
  return NO_ERROR;
}

status_t CameraContext::ParsePair(const char *str, uint32_t *first,
                                  uint32_t *second, char delim, char **endptr)
{
    // Find the first integer.
    char *end;
    uint32_t w = (uint32_t)strtol(str, &end, 10);
    // If a delimeter does not immediately follow, give up.
    if (*end != delim) {
        ALOGE("Cannot find delimeter (%c) in str=%s", delim, str);
        return BAD_VALUE;
    }

    // Find the second integer, immediately after the delimeter.
    uint32_t h = (uint32_t)strtol(end+1, &end, 10);

    *first = w;
    *second = h;

    if (endptr) {
        *endptr = end;
    }

    return NO_ERROR;
}

status_t CameraContext::ParseList(const char *list,
                                  std::vector<std::string> &sizes) {

    if (list == nullptr) {
      return BAD_VALUE;
    }

    std::string str(list);
    size_t start;
    size_t end = 0;
    while ((start = str.find_first_not_of(',', end)) != std::string::npos) {
      end = str.find(',', start);
      sizes.push_back(str.substr(start, end - start));
    }
    return NO_ERROR;
}

/** FromQmmfToHalFormat
 *
 * Translates QMMF format to HAL format
 *
 * return: HAL format
 **/
const char *CameraContext::FromQmmfToHalFormat_hal1(
  const BufferFormat &format) {

  switch (format) {
  case BufferFormat::kNV12UBWC:
  case BufferFormat::kNV12:
    return QTI_PIXEL_FORMAT_NV12_VENUS;
    break;
  case BufferFormat::kNV21:
    return QTI_PIXEL_FORMAT_NV21_VENUS;
    break;
  case BufferFormat::kNV16:
    return CameraParameters::PIXEL_FORMAT_YUV422SP;
    break;
  case BufferFormat::kRAW8:
    return CameraParameters::PIXEL_FORMAT_BAYER_RGGB;
    break;
  case BufferFormat::kRAW10:
    return CameraParameters::PIXEL_FORMAT_BAYER_RGGB;
    break;
  case BufferFormat::kRAW12:
    return CameraParameters::PIXEL_FORMAT_BAYER_RGGB;
    break;
  case BufferFormat::kRAW16:
    return CameraParameters::PIXEL_FORMAT_BAYER_RGGB;
    break;
  default:
    /* Format not supported */
    QMMF_ERROR("%s: error: unsupported format %d (0x%x)", __func__,
      (int32_t) format,
      (int32_t) format);
    return nullptr;
  }
}

CameraPort::CameraPort(CameraPortType port_type, CameraContext* context) // todo shared pointer
:
  port_type_(port_type),
  context_(context),
  frame_number_(0),
  buffer_count_(0),
  port_frame_rate_(0.0f) {

  QMMF_INFO("%s: Enter", __func__);

  BufferProducerImpl<CameraPort> *producer_impl;
  producer_impl = new BufferProducerImpl<CameraPort>(this);
  buffer_producer_impl_ = producer_impl;

  port_state_ = PortState::PORT_CREATED;

  QMMF_INFO("%s: Exit (0x%p)", __func__, this);
}

CameraPort::~CameraPort() {

  QMMF_INFO("%s: Enter ", __func__);
  buffer_producer_impl_.clear();
  buffer_producer_impl_ = nullptr;

  QMMF_INFO("%s: Exit (0x%p)", __func__, this);
}

status_t CameraPort::AddConsumer(sp<IBufferConsumer>& consumer) {

  std::lock_guard < std::mutex > lock(consumer_lock_);

  assert(consumer.get() != nullptr);

  if (IsConsumerConnected(consumer)) {
    QMMF_ERROR("%s: consumer(%p) already added to the producer!", __func__,
      consumer.get());
    return ALREADY_EXISTS;
  }

  // Add consumer to port's producer interface.
  assert(buffer_producer_impl_.get() != nullptr);
  buffer_producer_impl_->AddConsumer(consumer);
  consumer->SetProducerHandle(buffer_producer_impl_);
  QMMF_DEBUG("%s: Consumer(%p) has been added to PreviewPort(%p)."
    "Total number of consumer = %d", __func__, consumer.get(), this,
    buffer_producer_impl_->GetNumConsumer());

  consumers_.emplace(reinterpret_cast<uintptr_t>(consumer.get()), consumer);
  return NO_ERROR;
}

status_t CameraPort::RemoveConsumer(sp<IBufferConsumer>& consumer) {
  std::lock_guard < std::mutex > lock(consumer_lock_);

  assert(consumer.get() != nullptr);
  if (!IsConsumerConnected(consumer)) {
    QMMF_ERROR("%s: consumer(%p) is not connected to this port(%p)!", __func__,
      consumer.get(), this);
    return BAD_VALUE;
  }

  // Remove consumer from port's producer interface.
  assert(buffer_producer_impl_.get() != nullptr);
  buffer_producer_impl_->RemoveConsumer(consumer);
  QMMF_DEBUG("%s: Consumer(%p) has been removed from PreviewPort(%p)."
    "Total number of consumer = %d", __func__, consumer.get(), this,
    buffer_producer_impl_->GetNumConsumer());

  consumers_.erase(reinterpret_cast<uintptr_t>(consumer.get()));
  return NO_ERROR;
}

int32_t CameraPort::GetNumConsumers() {

  std::lock_guard < std::mutex > lock(consumer_lock_);
  return consumers_.size();
}

PortState& CameraPort::getPortState() {
  return port_state_;
}

bool CameraPort::IsConsumerConnected(sp<IBufferConsumer>& consumer) {

  uintptr_t key = reinterpret_cast<uintptr_t>(consumer.get());
  return (consumers_.count(key) != 0) ? true : false;
}

status_t CameraPort::DeInit() {

  std::lock_guard < std::mutex > lock(state_lock_);
  QMMF_VERBOSE("%s port type %d state %d ", __func__, (int32_t) GetPortType(),
    (int32_t) port_state_);

  assert(context_ != nullptr);

  track_id_ = 0;
  port_frame_rate_ = 0.0f;

  assert(consumers_.size() == 0);

  port_state_ = PortState::PORT_CREATED;

  QMMF_DEBUG("%s: CameraPort(0x%p) deinitialized successfully! ", __func__,
    this);
  QMMF_VERBOSE("%s: Exit consumers_ size - %d", __func__, consumers_.size());
  return NO_ERROR;
}

status_t CameraPort::release_frame(const void *opaque) {
  QMMF_VERBOSE("E %s \n", __func__);
  if (GetPortType() == CameraPortType::kPreview) {
    ((camera_device_t *)context_->camera_device_)->ops->release_preview_frame(
      (camera_device_t *)context_->camera_device_, opaque);
  } else if (GetPortType() == CameraPortType::kVideo) {
    ((camera_device_t *)context_->camera_device_)->ops->release_recording_frame(
      (camera_device_t *)context_->camera_device_, opaque);
  }
  return NO_ERROR;
}

void CameraPort::NotifyBufferReturned(const StreamBuffer& buffer) {

  QMMF_VERBOSE("%s: StreamBuffer(0x%p) Cameback to Port:%d", __func__,
    buffer.handle, (int32_t) GetPortType());

  munmap(buffer.data, buffer.size);
  context_->alloc_device_interface_->FreeBuffer(buffer.handle);

  std::unique_lock < std::mutex > lock(context_->buffer_lock_);
  assert(buffer_map_.count(buffer.fd));

  release_frame(buffer_map_.at(buffer.fd));
  buffer_map_.erase(buffer.fd);

  buffer_count_--;
  if (buffer_count_ == 0) {
    wait_for_buffer_.Signal();
  }
}

void CameraPort::StreamCallback(const void *data, int64_t timestamp) {

  QMMF_VERBOSE("%s: Enter", __func__);
  camera_memory_t * data_mem = (camera_memory_t *)data;

  if ((buffer_producer_impl_->GetNumConsumer() > 0)
    && (getPortState() == PortState::PORT_STARTED)) {
    if (GetPortType() == CameraPortType::kPreview) {
      QMMF_DEBUG("%s:%d: CAMERA_MSG_PREVIEW_FRAME", __func__, __LINE__);
    } else if (GetPortType() == CameraPortType::kVideo) {
      QMMF_DEBUG("%s:%d: CAMERA_MSG_VIDEO_FRAME", __func__, __LINE__);
    }

    encoder_media_buffer_type *packet = reinterpret_cast< encoder_media_buffer_type *>(data_mem->data);
    struct gbm_bo *bo = packet->meta_handle;
    int32_t fd = gbm_bo_get_fd(bo);
    void* base = static_cast< uint8_t *>(mmap(0, bo->size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0));

    {
      std::unique_lock < std::mutex > lock(g_allocated_fd_lock_);
      assert(g_allocated_fd_buffer_list_.count(fd) > 0);
      if (g_allocated_fd_buffer_list_.count(fd) > 0) {
        FDdata *fd_data = g_allocated_fd_buffer_list_.at(fd);
        fd_data->isInUse = true;
      }
    }

    StreamBuffer buffer { };
    buffer.fd = fd;
    buffer.data = base;
    buffer.size = bo->size;
    buffer.frame_number = ++frame_number_;
    buffer.timestamp = timestamp;
    context_->alloc_device_interface_->ImportBuffer(buffer.handle, bo, fd);
    auto ret =
        context_->PopulateBufferMeta(buffer.info, buffer.handle, width_, height_);
    assert(ret == NO_ERROR);

    context_->alloc_device_interface_->Perform(
        buffer.handle, IAllocDevice::AllocDeviceAction::GetMetaFd,
        static_cast<void*>(&buffer.metafd)
    );

    {
      std::unique_lock < std::mutex > lock(context_->buffer_lock_);
      buffer_count_++;
      buffer_map_.emplace(fd, data_mem->data);
    }

    QMMF_DEBUG("%s: NotifyBuffer!, timestamp = %llu\n", __func__,
      buffer.timestamp);
    buffer_producer_impl_->NotifyBuffer(buffer);
  } else {
    release_frame(data_mem->data);
  }

  QMMF_VERBOSE("%s: Exit ", __func__);
}

status_t PreviewPort::Init(const StreamParam& param) {

  std::lock_guard < std::mutex > lock(state_lock_);
  QMMF_VERBOSE("%s port type %d state %d ", __func__, (int32_t) GetPortType(),
    (int32_t) port_state_);

  track_id_ = param.id;
  port_frame_rate_ = param.framerate;
  port_format_ = param.format;
  width_ = param.width;
  height_ = param.height;

  context_->mParameters_.setPreviewSize(param.width, param.height); // todo check if is set
  context_->mParameters_.setVideoSize(320, 240);
  context_->mParameters_.setPreviewFormat(
    context_->FromQmmfToHalFormat_hal1(param.format));

  context_->mParameters_.set("recording-hint", "false");
  context_->mParameters_.set("store-meta-data-in-buffers", "true");
  context_->mParameters_.set("zsl", "on");
  ((camera_device_t *)context_->camera_device_)->ops->store_meta_data_in_buffers((camera_device_t *)context_->camera_device_, true);

  assert(context_ != nullptr);
  port_state_ = PortState::PORT_INITIALIZED;
  QMMF_INFO("%s: track_id(0%x)", __func__, param.id);
  return NO_ERROR;
}

status_t PreviewPort::Start() {

  std::lock_guard < std::mutex > lock(state_lock_);
  QMMF_VERBOSE("%s port type %d state %d ", __func__, (int32_t) GetPortType(),
    (int32_t) port_state_);

  if (port_state_ != PortState::PORT_INITIALIZED) {
    // Port is already in started state.
    QMMF_ERROR("%s: Port is already in started state.", __func__);
    return NO_ERROR;
  }

  frame_number_ = 0;

  ((camera_device_t *)context_->camera_device_)->ops->start_preview((camera_device_t *)context_->camera_device_);
  port_state_ = PortState::PORT_STARTED;

  QMMF_INFO("%s: PortType(%x):Port(%p) Started Succussfully!", __func__,
    (int32_t) GetPortType(), this);
  return NO_ERROR;
}

status_t PreviewPort::Stop() {

  std::lock_guard < std::mutex > lock(state_lock_);
  QMMF_VERBOSE("%s port type %d state %d ", __func__, (int32_t) GetPortType(),
    (int32_t) port_state_);

  if (port_state_ != PortState::PORT_STARTED) {
    // Port is already in stopped state.
    QMMF_ERROR("%s: Port is already in stopped state.", __func__);
    return NO_ERROR;
  }

  ((camera_device_t *)context_->camera_device_)->ops->stop_preview((camera_device_t *)context_->camera_device_);
  port_state_ = PortState::PORT_INITIALIZED;

  QMMF_INFO("%s: PortType(%x):Port(%p) Stopped Succussfully!", __func__,
    (int32_t) GetPortType(), this);
  return NO_ERROR;
}

status_t VideoPort::Init(const StreamParam& param) {

  std::lock_guard < std::mutex > lock(state_lock_);
  QMMF_VERBOSE("%s port type %d state %d ", __func__, (int32_t) GetPortType(),
    (int32_t) port_state_);

  track_id_ = param.id;
  port_frame_rate_ = param.framerate;
  port_format_ = param.format;
  width_ = param.width;
  height_ = param.height;

  context_->mParameters_.setVideoSize(param.width, param.height);
  context_->mParameters_.set(CameraParameters::KEY_VIDEO_FRAME_FORMAT,
    context_->FromQmmfToHalFormat_hal1(param.format));

  context_->mParameters_.set("recording-hint", "true");
  context_->mParameters_.set("store-meta-data-in-buffers", "true");
  context_->mParameters_.set("zsl", "off");
  ((camera_device_t *)context_->camera_device_)->ops->store_meta_data_in_buffers((camera_device_t *)context_->camera_device_, true);

  assert(context_ != nullptr);
  port_state_ = PortState::PORT_INITIALIZED;
  QMMF_INFO("%s: track_id(0%x)", __func__, param.id);
  return NO_ERROR;
}

status_t VideoPort::Start() {

  std::lock_guard < std::mutex > lock(state_lock_);
  QMMF_VERBOSE("%s port type %d state %d ", __func__, (int32_t) GetPortType(),
    (int32_t) port_state_);

  if (port_state_ != PortState::PORT_INITIALIZED) {
    // Port is already in started state.
    QMMF_ERROR("%s: Port is already in started state.", __func__);
    return NO_ERROR;
  }

  frame_number_ = 0;
  ((camera_device_t *)context_->camera_device_)->ops->start_recording(
    (camera_device_t *)context_->camera_device_);
  port_state_ = PortState::PORT_STARTED;

  QMMF_INFO("%s: PortType(%x):Port(%p) Started Succussfully!", __func__,
    (int32_t) GetPortType(), this);
  return NO_ERROR;
}

status_t VideoPort::Stop() {

  std::lock_guard < std::mutex > lock(state_lock_);
  QMMF_VERBOSE("%s port type %d state %d ", __func__, (int32_t) GetPortType(),
    (int32_t) port_state_);

  if (port_state_ != PortState::PORT_STARTED) {
    // Port is already in stopped state.
    QMMF_ERROR("%s: Port is already in stopped state.", __func__);
    return NO_ERROR;
  }

  ((camera_device_t *)context_->camera_device_)->ops->stop_recording(
    (camera_device_t *)context_->camera_device_);
  port_state_ = PortState::PORT_INITIALIZED;

  QMMF_INFO("%s: PortType(%x):Port(%p) Stopped Succussfully!", __func__,
    (int32_t) GetPortType(), this);
  return NO_ERROR;
}
}
;
// namespace recoder

}
;
// namespace qmmf
