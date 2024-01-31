/*
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
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

#define LOG_TAG "RecorderService"

#include "recorder/src/service/qmmf_recorder_service.h"

namespace qmmf {

namespace recorder {

#ifndef HAVE_BINDER
ThreadPool::ThreadPool()
    : stop_(false), total_tasks_executed_(0), total_tasks_cancelled_(0) {
  size_t num_threads = std::thread::hardware_concurrency();
  num_threads = num_threads > 0 ? num_threads : 1;
  for (size_t i = 0; i < num_threads; ++i) {
    QMMF_INFO("%s: Pool start thread %lu", __func__, i);

    worker_states_.emplace_back(
        false); // Initialize per-thread state to not canceled.
    workers_.emplace_back([this, i] {
      while (true) {
        std::function<void()> task;
        {
          std::unique_lock<std::mutex> lock(queue_mutex_);
          QMMF_INFO("%s: waiting for work on thread:%lu", __func__, i);
          condition_.wait(lock, [this] { return stop_ || HasTasks(); });
          if (stop_ && !HasTasks()) {
            return;
          }
          QMMF_INFO("%s: got work on thread:%lu", __func__, i);

          // Get the highest-priority task from the queue.
          task = std::move(GetHighestPriorityTask());
        }
        try {
          // Check for cancellation before executing the task.
          if (!IsTaskCancelled(i)) {
            task();
            std::unique_lock<std::mutex> lock(executed_mutex_);
            ++total_tasks_executed_;
          }
        } catch (const std::exception &e) {
          QMMF_ERROR("%s: Thread pool caught an exception: %s", __func__, e.what());
        }
      }
    });
  }
}

template <typename Func>
auto ThreadPool::Enqueue(Func &&func, TASK_PRIORITY priority)
    -> std::future<decltype(func())> {
  using ReturnType = decltype(func());
  auto task = std::make_shared<std::packaged_task<ReturnType()>>(
      std::forward<Func>(func));
  auto result = task->get_future();
  {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    if (stop_) {
      throw std::runtime_error("enqueue on stopped ThreadPool");
    }
    TaskPriorityWrapper TaskWrapper(priority, [task]() { (*task)(); });
    tasks_.push(std::move(TaskWrapper));
  }

  QMMF_INFO("%s: notifying task ", __func__);
  condition_.notify_one();
  return result;
}

void ThreadPool::CancelAllTasks() {
  {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    while (!tasks_.empty()) {
      tasks_.pop();
      ++total_tasks_cancelled_;
    }
  }
  condition_.notify_all();
}

// Set the cancellation flag for the specified thread.
void ThreadPool::SetTaskCancelled(size_t thread_id) {
  std::lock_guard<std::mutex> lock(thread_state_mutex_);
  if (thread_id < worker_states_.size()) {
    worker_states_[thread_id] = true;
  }
}

// Check if the task for the specified thread is canceled.
bool ThreadPool::IsTaskCancelled(size_t thread_id) const {
  std::lock_guard<std::mutex> lock(thread_state_mutex_);
  if (thread_id < worker_states_.size()) {
    return worker_states_[thread_id];
  }
  return false;
}
#endif // !HAVE_BINDER

RecorderService::RecorderService() {

  QMMF_GET_LOG_LEVEL();
  QMMF_KPI_GET_MASK();

  // Preload the recorder at bootup.
  recorder_.reset(RecorderImpl::CreateRecorder());
  if (!recorder_) {
    QMMF_ERROR("%s: Can't create Recorder Instance!!", __func__);
  } else {
#ifdef HAVE_BINDER
    std::function< const sp<RemoteCallBack>& (uint32_t id)>
      remote_cb_handle = [&] (uint32_t id) {
        QMMF_VERBOSE("%s: Client(%u): RemoteCallback request!", __func__, id);
        assert(remote_cb_list_.count(id) != 0);
        return remote_cb_list_[id];
    };
#else
    std::function< const std::shared_ptr<RemoteCallBack>& (uint32_t id)>
      remote_cb_handle = [&] (uint32_t id) ->  std::shared_ptr<RemoteCallBack>& {
        QMMF_VERBOSE("%s: Client(%u): RemoteCallback request!",
                      __func__, id);
        assert(remote_cb_list_.count(id) != 0);
        return remote_cb_list_[id];
    };

    if (SetupSocket() != 0) {
      QMMF_ERROR("%s: Socket Setup failed!", __func__);
      throw errno;
    }

#endif // HAVE_BINDER
    auto ret = recorder_->Init(remote_cb_handle);
    if (ret != 0) {
      QMMF_ERROR("%s: Recorder Initialization failed!", __func__);
      recorder_.reset();
      throw ret;
    }
  }

  QMMF_INFO("%s: RecorderService Instantiated! ", __func__);
  QMMF_KPI_DETAIL();
}

RecorderService::~RecorderService() {

  QMMF_INFO("%s: Enter ", __func__);
#ifndef HAVE_BINDER
  close(socket_);
  unlink(socket_path_.c_str());

  // Clean up all client sockets
  for (const auto& [socket, id]: client_sockets_) {
    close(socket);
  }
  client_sockets_.clear();
#endif // !HAVE_BINDER
  QMMF_INFO("%s: Exit ", __func__);
  QMMF_KPI_DETAIL();
}

#ifdef HAVE_BINDER
status_t RecorderService::onTransact(uint32_t code, const Parcel& data,
                                     Parcel* reply, uint32_t flag) {

  QMMF_DEBUG("%s: Enter:(RecorderServiceStub::onTransact)", __func__);
  CHECK_INTERFACE(IRecorderService, data, reply);
  int32_t ret = 0;

  switch (code) {
    case RECORDER_CONNECT: {
      sp<IRecorderServiceCallback> client_cb_handle = interface_cast
          <IRecorderServiceCallback>(data.readStrongBinder());
      uint32_t client_id;
      ret = Connect(client_cb_handle, &client_id);
      reply->writeUint32(client_id);
      reply->writeInt32(ret);
      return 0;
    }
    break;
      case RECORDER_DISCONNECT: {
        uint32_t client_id;
        data.readUint32(&client_id);
        ret = Disconnect(client_id);
        reply->writeInt32(ret);
        return 0;
      }
      break;
      case RECORDER_START_CAMERA: {
        uint32_t camera_id, enable_flag;
        bool enable_result_cb;
        uint32_t client_id;
        float framerate;
        data.readUint32(&client_id);
        data.readUint32(&camera_id);
        data.readFloat(&framerate);
        data.readUint32(&enable_flag);
        enable_result_cb = (1 == enable_flag) ? true : false;
        uint32_t extra_blob_size;
        android::Parcel::ReadableBlob extra_blob;
        data.readUint32(&extra_blob_size);
        data.readBlob(extra_blob_size, &extra_blob);
        CameraExtraParam extra_param(extra_blob.data(), extra_blob_size);
        ret = StartCamera(client_id, camera_id, framerate,
                          extra_param, enable_result_cb);
        extra_blob.release();
        reply->writeInt32(ret);
        return 0;
      }
      break;
      case RECORDER_STOP_CAMERA: {
        uint32_t client_id, camera_id;
        data.readUint32(&client_id);
        data.readUint32(&camera_id);
        ret = StopCamera(client_id, camera_id);
        reply->writeInt32(ret);
        return 0;
      }
      break;
      case RECORDER_CREATE_SESSION: {
        uint32_t client_id, session_id;
        data.readUint32(&client_id);
        ret = CreateSession(client_id, &session_id);
        reply->writeUint32(session_id);
        reply->writeInt32(ret);
        return 0;
      }
      break;
      case RECORDER_DELETE_SESSION: {
        uint32_t client_id, session_id;
        data.readUint32(&client_id);
        data.readUint32(&session_id);
        ret = DeleteSession(client_id, session_id);
        reply->writeInt32(ret);
        return 0;
      }
      break;
      case RECORDER_START_SESSION: {
        uint32_t client_id, session_id;
        data.readUint32(&client_id);
        data.readUint32(&session_id);
        ret = StartSession(client_id, session_id);
        reply->writeInt32(ret);
        return 0;
      }
      break;
      case RECORDER_STOP_SESSION: {
        uint32_t client_id, session_id;
        int32_t flush;
        data.readUint32(&client_id);
        data.readUint32(&session_id);
        data.readInt32(&flush);
        ret = StopSession(client_id, session_id, flush);
        reply->writeInt32(ret);
        return 0;
      }
      break;
      case RECORDER_PAUSE_SESSION: {
        uint32_t client_id, session_id;
        data.readUint32(&client_id);
        data.readUint32(&session_id);
        ret = PauseSession(client_id, session_id);
        reply->writeInt32(ret);
        return 0;
      }
      break;
      case RECORDER_RESUME_SESSION: {
        uint32_t client_id, session_id;
        data.readUint32(&client_id);
        data.readUint32(&session_id);
        ret = ResumeSession(client_id, session_id);
        reply->writeInt32(ret);
        return 0;
      }
      break;
      case RECORDER_CREATE_VIDEOTRACK: {
        uint32_t client_id, session_id, track_id;
        uint32_t blob_size, extra_blob_size;
        data.readUint32(&client_id);
        data.readUint32(&session_id);
        data.readUint32(&track_id);
        android::Parcel::ReadableBlob blob;
        data.readUint32(&blob_size);
        data.readBlob(blob_size, &blob);
        android::Parcel::ReadableBlob extra_blob;
        data.readUint32(&extra_blob_size);
        data.readBlob(extra_blob_size, &extra_blob);
        VideoTrackParam video_track_param;
        assert(blob_size == sizeof(video_track_param));
        memcpy(&video_track_param, blob.data(), blob_size);

        VideoExtraParam xtraparam(extra_blob.data(), extra_blob_size);
        ret = CreateVideoTrack(client_id, session_id, track_id,
                               video_track_param, xtraparam);
        blob.release();
        extra_blob.release();
        reply->writeInt32(ret);
        return 0;
      }
      break;
      case RECORDER_DELETE_VIDEOTRACK: {
        uint32_t client_id, session_id, track_id;
        data.readUint32(&client_id);
        data.readUint32(&session_id);
        data.readUint32(&track_id);
        ret = DeleteVideoTrack(client_id, session_id, track_id);
        reply->writeInt32(ret);
        return 0;
      }
      break;
      case RECORDER_RETURN_TRACKBUFFER: {
        uint32_t client_id, session_id, track_id;
        data.readUint32(&client_id);
        data.readUint32(&session_id);
        data.readUint32(&track_id);

        std::vector<BnBuffer> buffers;
        if (track_id < 100) {
          uint32_t vector_size;
          data.readUint32(&vector_size);
          for (uint32_t i = 0; i < vector_size; i++)  {
            uint32_t size;
            data.readUint32(&size);
            android::Parcel::ReadableBlob blob;
            data.readBlob(size, &blob);
            void* buffer = const_cast<void*>(blob.data());
            BnBuffer track_buffer;
            assert(size == sizeof(track_buffer));
            memcpy(&track_buffer, buffer, size);
            buffers.push_back(track_buffer);
            blob.release();
          }
        } else {
          size_t num_buffers = data.readInt32();
          for (size_t index = 0; index < num_buffers; ++index) {
            BnBuffer buffer;
            buffer.FromParcel(data, false);
            buffers.push_back(buffer);
          }
        }
        ret = ReturnTrackBuffer(client_id, session_id, track_id, buffers);
        reply->writeInt32(ret);
        return 0;
      }
      break;
      case RECORDER_SET_VIDEOTRACK_PARAMS: {
        uint32_t client_id, session_id, track_id;
        uint32_t param_type, blob_size;
        data.readUint32(&client_id);
        data.readUint32(&session_id);
        data.readUint32(&track_id);
        data.readUint32(&param_type);
        data.readUint32(&blob_size);
        android::Parcel::ReadableBlob blob;
        data.readBlob(blob_size, &blob);
        void* param = const_cast<void*>(blob.data());
        ret = SetVideoTrackParam(client_id, session_id, track_id,
                                 static_cast<VideoParam>(param_type),
                                 param, blob_size);
        reply->writeInt32(ret);
        blob.release();
        return 0;
      }
      break;
      case RECORDER_CAPTURE_IMAGE: {
        uint32_t client_id, camera_id, type, n_images, meta_size;
        data.readUint32(&client_id);
        data.readUint32(&camera_id);
        data.readUint32(&type);
        data.readUint32(&n_images);
        data.readUint32(&meta_size);
        std::vector<CameraMetadata> meta_array;
        for (uint32_t i = 0; i < meta_size; ++i) {
          CameraMetadata meta;
          camera_metadata_t *m = nullptr;
          ret = meta.readFromParcel(data, &m);
          if ((0 != ret) || (nullptr == m)) {
            QMMF_ERROR("%s: Metadata parcel read failed: %d meta(%p)",
                __func__, ret, m);
            reply->writeInt32(ret);
            return ret;
          }
          meta.clear();
          meta.append(m);
          meta_array.push_back(meta);
          //We need to release this memory as meta.append() makes copy of this memory
          free(m);
        }
        ret = CaptureImage(client_id, camera_id,
                           static_cast<SnapshotType>(type), n_images,
                           meta_array);

        // Clear the metadata buffers and free all storage used by it
        for (auto meta:meta_array) {
          meta.clear();
        }

        reply->writeInt32(ret);
        return 0;
      }
      break;
      case RECORDER_CONFIG_IMAGECAPTURE: {
        uint32_t client_id, camera_id, image_id, img_param_blob_size, blob_size;
        data.readUint32(&client_id);
        data.readUint32(&camera_id);
        data.readUint32(&image_id);
        data.readUint32(&img_param_blob_size);
        android::Parcel::ReadableBlob img_param_blob;
        data.readBlob(img_param_blob_size, &img_param_blob);
        ImageParam param;
        assert(img_param_blob_size == sizeof(param));
        memcpy(&param, img_param_blob.data(), img_param_blob_size);
        data.readUint32(&blob_size);
        android::Parcel::ReadableBlob blob;
        data.readBlob(blob_size, &blob);
        ImageExtraParam xtraparam(blob.data(), blob_size);
        ret = ConfigImageCapture(client_id, camera_id, image_id, param, xtraparam);
        reply->writeInt32(ret);
        blob.release();
        img_param_blob.release();
        return 0;
      }
      break;
      case RECORDER_CANCEL_IMAGECAPTURE: {
        uint32_t client_id, camera_id, image_id, cache;
        data.readUint32(&client_id);
        data.readUint32(&camera_id);
        data.readUint32(&image_id);
        data.readUint32(&cache);
        ret = CancelCaptureImage(client_id, camera_id, image_id, cache);
        reply->writeInt32(ret);
        return 0;
      }
      break;
      case  RECORDER_RETURN_IMAGECAPTURE_BUFFER: {
        uint32_t client_id, camera_id, buffer_id;
        data.readUint32(&client_id);
        data.readUint32(&camera_id);
        data.readUint32(&buffer_id);
        ret = ReturnImageCaptureBuffer(client_id, camera_id, buffer_id);
        reply->writeInt32(ret);
        return 0;
      }
      break;
      case RECORDER_SET_CAMERA_PARAMS: {
        uint32_t client_id, camera_id;
        data.readUint32(&client_id);
        data.readUint32(&camera_id);
        CameraMetadata meta;
        camera_metadata_t *m = nullptr;
        ret = meta.readFromParcel(data, &m);
        if ((0 != ret) || (nullptr == m)) {
          QMMF_ERROR("%s: Metadata parcel read failed: %d meta: %p\n",
              __func__, ret, m);
          reply->writeInt32(ret);
          return ret;
        }
        meta.clear();
        meta.append(m);
        ret = SetCameraParam(client_id, camera_id, meta);

        // Clear the metadata buffer and free all storage used by it
        meta.clear();
        //We need to release this memory as meta.append() makes copy of this memory
        free(m);
        reply->writeInt32(ret);
        return 0;
      }
      break;
      case RECORDER_GET_CAMERA_PARAMS: {
        uint32_t client_id, camera_id;
        data.readUint32(&client_id);
        data.readUint32(&camera_id);
        CameraMetadata meta;
        ret = GetCameraParam(client_id, camera_id, meta);
        reply->writeInt32(ret);
        if (0 == ret) {
          ret = meta.writeToParcel(reply);
          if (0 != ret) {
            QMMF_ERROR("%s: Metadata parcel write failed: %d\n",
                       __func__, ret);
          }
        }
        meta.clear();
        return 0;
      }
      break;
      case RECORDER_SET_CAMERA_SESSION_PARAMS: {
        uint32_t client_id, camera_id;
        data.readUint32(&client_id);
        data.readUint32(&camera_id);
        CameraMetadata meta;
        camera_metadata_t *m = nullptr;
        ret = meta.readFromParcel(data, &m);
        if ((NO_ERROR != ret) || (nullptr == m)) {
          QMMF_ERROR("%s: Metadata parcel read failed: %d meta: %p\n",
              __func__, ret, m);
          reply->writeInt32(ret);
          return ret;
        }
        meta.clear();
        meta.append(m);
        ret = SetCameraSessionParam(client_id, camera_id, meta);

        // Clear the metadata buffer and free all storage used by it
        meta.clear();
        //We need to release this memory as meta.append() makes copy of this memory
        free(m);
        reply->writeInt32(ret);
        return NO_ERROR;
      }
      break;
      case RECORDER_SET_SHDR: {
        uint32_t client_id, camera_id;
        int32_t enable;
        data.readUint32(&client_id);
        data.readUint32(&camera_id);
        data.readInt32(&enable);
        ret = SetSHDR(client_id, camera_id, enable);
        reply->writeInt32(ret);
        return 0;
      }
      break;
      case RECORDER_GET_DEFAULT_CAPTURE_PARAMS: {
        uint32_t client_id, camera_id;
        data.readUint32(&client_id);
        data.readUint32(&camera_id);
        CameraMetadata meta;
        ret = GetDefaultCaptureParam(client_id, camera_id, meta);
        reply->writeInt32(ret);
        if (0 == ret) {
          ret = meta.writeToParcel(reply);
          if (0 != ret) {
            QMMF_ERROR("%s: Metadata parcel write failed: %d\n",
                       __func__, ret);
          }
        }
        meta.clear();
        return 0;
      }
      break;
      case RECORDER_GET_CAMERA_CHARACTERISTICS: {
        uint32_t client_id, camera_id;
        data.readUint32(&client_id);
        data.readUint32(&camera_id);
        CameraMetadata meta;
        ret = GetCameraCharacteristics(client_id, camera_id, meta);
        reply->writeInt32(ret);
        if (0 == ret) {
          ret = meta.writeToParcel(reply);
          if (0 != ret) {
            QMMF_ERROR("%s: Metadata parcel write failed: %d\n",
                       __func__, ret);
          }
        }
        meta.clear();
        return 0;
      }
      break;
      case RECORDER_GET_VENDOR_TAG_DESCRIPTOR: {
        std::shared_ptr<VendorTagDescriptor> desc;
        ret = GetVendorTagDescriptor(desc);
        reply->writeInt32(ret);
        if (0 == ret) {
          ret = desc->writeToParcel(reply);
          if (0 != ret) {
            QMMF_ERROR("%s: VendorTagDescriptor parcel write failed: %d\n",
                       __func__, ret);
          }
        }
        desc.clear();
        return 0;
      }
      break;
      case RECORDER_CONFIGURE_OFFLINE_JPEG: {
        uint32_t client_id, jpeg_params_blob_size;
        data.readUint32(&client_id);
        data.readUint32(&jpeg_params_blob_size);
        android::Parcel::ReadableBlob jpeg_params_blob;
        data.readBlob(jpeg_params_blob_size, &jpeg_params_blob);
        OfflineJpegCreateParams params;
        assert(jpeg_params_blob_size == sizeof(params));
        memcpy(&params, jpeg_params_blob.data(), jpeg_params_blob_size);

        ret = CreateOfflineJPEG(client_id, params);
        reply->writeInt32(ret);
        return 0;
      }
      break;
      case RECORDER_ENCODE_OFFLINE_JPEG: {
        uint32_t client_id, meta_blob_size;
        OfflineJpegProcessParams params;
        data.readUint32(&client_id);

        uint32_t present;
        BnBuffer in_buf = {};
        BnBuffer out_buf = {};
        in_buf.ion_fd = out_buf.ion_fd = -1;
        OfflineJpegMeta metadata;
        // Input buffer
        data.readUint32(&present);
        if (!present) {
          in_buf.ion_fd = dup(data.readFileDescriptor());
        }
        data.readUint32(&in_buf.buffer_id);

        // Output buffer
        data.readUint32(&present);
        if (!present) {
          out_buf.ion_fd = dup(data.readFileDescriptor());
        }
        data.readUint32(&out_buf.buffer_id);

        data.readUint32(&meta_blob_size);
        android::Parcel::ReadableBlob meta_blob;
        data.readBlob(meta_blob_size, &meta_blob);
        assert(meta_blob_size == sizeof(metadata));
        memcpy(&metadata, meta_blob.data(), meta_blob_size);

        ret = EncodeOfflineJPEG(client_id, in_buf, out_buf, metadata);
        meta_blob.release();
        reply->writeInt32(ret);

        return 0;
      }
      break;
      case RECORDER_DESTROY_OFFLINE_JPEG: {
        uint32_t client_id;
        data.readUint32(&client_id);
        ret = DestroyOfflineJPEG(client_id);
        reply->writeInt32(ret);

        return 0;
      }
      break;
      default: {
        QMMF_ERROR("RecorderService:%s:Method is not supported !",__func__);
        reply->writeInt32(-1);
      }
      break;
  }
  return 0;
}

status_t RecorderService::Connect(uint32_t* client_id,
                                  const sp<IRecorderServiceCallback>&
                                  service_cb) {

  QMMF_DEBUG("%s: Enter ", __func__);
  QMMF_KPI_DETAIL();
  status_t ret;

  std::lock_guard<std::mutex> lock(lock_);

  if (!recorder_) {
    recorder_.reset(RecorderImpl::CreateRecorder());
    if (!recorder_) {
      QMMF_ERROR("%s: Can't create Recorder Instance!!", __func__);
      return -ENOMEM;
    }
    std::function< const sp<RemoteCallBack>& (uint32_t id)>
      remote_cb_handle = [&] (uint32_t id) -> sp<RemoteCallBack>& {
        QMMF_VERBOSE("%s: Client(%u): RemoteCallback request!",
                      __func__, id);
        assert(remote_cb_list_.count(id) != 0);
        return remote_cb_list_[id];
    };
    ret = recorder_->Init(remote_cb_handle);
    if (ret != 0) {
      QMMF_ERROR("%s: Recorder initialization failed!", __func__);
      recorder_.reset();
      return ret;
    }
  }

  ret = GetUniqueClientID(client_id);
  if (ret != 0) {
    QMMF_ERROR("%s: Too many active clients (255)!", __func__);
    return ret;
  }

  sp<RemoteCallBack> remote_callback;
  remote_callback = new RemoteCallBack(*client_id, service_cb);
  if (!remote_callback.get()) {
      QMMF_ERROR("%s: Unable to allocate remote callback!", __func__);
      return -ENODEV;
  }

  sp<DeathNotifier> death_notifier = new DeathNotifier();
  if (!death_notifier.get()) {
    QMMF_ERROR("%s: Unable to allocate death notifier!", __func__);
    return -ENODEV;
  }
  NotifyClientDeath notify_death = [this, capture_client_id = *client_id] {
      ClientDeathHandler(capture_client_id);
  };
  death_notifier->SetDeathNotifyCB(notify_death);

  // Link death notifier to remote handle.
  IInterface::asBinder(remote_callback->getRemoteClient())
      ->linkToDeath(death_notifier);

  remote_cb_list_.emplace(*client_id, remote_callback);
  death_notifier_list_.emplace(*client_id, death_notifier);

  recorder_->RegisterClient(*client_id);

  QMMF_INFO("%s: Service is connected with client (%d)", __func__, *client_id);

  QMMF_DEBUG("%s: Exit client_id(%d)", __func__, *client_id);
  return 0;
}
#else
status_t RecorderService::SetupSocket() {
  socket_path_ = "/var/run/le_cam_socket";

  if (unlink(socket_path_.c_str()) == -1) {
    QMMF_WARN("%s: unlink failure for path(%s) %s, errno: %d", __func__,
               socket_path_.c_str(), strerror(errno), errno);
  }

  // Create a socket
  socket_ = socket(AF_UNIX, SOCK_STREAM, 0);
  if (socket_ == -1) {
    QMMF_ERROR("%s: socket failure %s", __func__, strerror(errno));
    return -errno;
  }

  sockaddr_un addr;
  addr.sun_family = AF_UNIX;
  auto size = socket_path_.size();
  snprintf(addr.sun_path, size+1, "%s", socket_path_.c_str());
  addr.sun_path[size+1] = '\0';

  // Bind the socket to the address
  if (bind(socket_, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
    QMMF_ERROR("%s: bind failure %s", __func__, strerror(errno));
    return -errno;
  }

  // Listen for incoming connections
  if (listen(socket_, 5) == -1) {
    QMMF_ERROR("%s: listen failure %s", __func__, strerror(errno));
    return -errno;
  }

  // Set the server socket to non-blocking mode
  int flags = fcntl(socket_, F_GETFL, 0);
  fcntl(socket_, F_SETFL, flags | O_NONBLOCK);

  run_ = true;

  QMMF_INFO("Server is listening...");
  return 0;
}

status_t RecorderService::ReadData (int socket, void *buffer, size_t size) {
  ssize_t bytes_read = recv(socket, buffer, size, 0);

  if (bytes_read > 0) {
    QMMF_VERBOSE("%s: read %d bytes from client socket: %d",
                 __func__, bytes_read, socket);
    return bytes_read;
  }

  if (bytes_read == -1) {
    QMMF_ERROR("%s: Receive failed: %s", __func__, strerror(errno));
    return -errno;
  } else if (bytes_read == 0) {
    QMMF_ERROR("%s: connection closed: %d ", __func__, socket);
    return 0;
  }
}

status_t RecorderService::SendResponse (int socket, void *buffer, size_t size) {
  ssize_t bytesSent = send(socket, buffer, size, 0);
  QMMF_INFO("sendResponse bytes: %lu", bytesSent);
  if (bytesSent == -1) {
    QMMF_ERROR("%s: failed: %s", __func__, strerror(errno));
    close(socket);
    return -errno;
  }
  // TODO: Add death handling check

  return 0;
}

void RecorderService::ProcessRequest(int client_socket, RecorderClientReqMsg req_msg) {
  QMMF_VERBOSE("%s: received cmd:%u", __func__, req_msg.DebugString().c_str());

  RecorderClientRespMsg resp_msg;
  switch (req_msg.command()) {
  case RECORDER_SERVICE_CMDS::RECORDER_CONNECT: {
    uint32_t client_id;
    auto ret = Connect(nullptr, &client_id);
    resp_msg.set_command(RECORDER_SERVICE_CMDS::RECORDER_CONNECT);
    resp_msg.mutable_connect_resp()->set_client_id(client_id);
    resp_msg.mutable_connect_resp()->set_server_pid(getpid());
    resp_msg.set_status(ret);
  } break;
  case RECORDER_SERVICE_CMDS::RECORDER_CALLBACK_SOCKET_READY: {
    uint32_t client_id = req_msg.callback_socket_ready().client_id();
    auto ret = SetupRemoteCallback(client_id);
    resp_msg.set_command(RECORDER_SERVICE_CMDS::RECORDER_CALLBACK_SOCKET_READY);
    resp_msg.set_status(ret);
  } break;
  case RECORDER_SERVICE_CMDS::RECORDER_DISCONNECT: {
    uint32_t client_id = req_msg.disconnect().client_id();
    auto ret = Disconnect(client_id);
    resp_msg.set_command(RECORDER_SERVICE_CMDS::RECORDER_DISCONNECT);
    resp_msg.set_status(ret);
  } break;
  case RECORDER_SERVICE_CMDS::RECORDER_START_CAMERA: {
    uint32_t client_id = req_msg.start_camera().client_id();
    uint32_t camera_id = req_msg.start_camera().camera_id();
    float framerate = req_msg.start_camera().framerate();
    bool enable_result_cb = req_msg.start_camera().enable_result_cb();
    const std::string extra_data = req_msg.start_camera().extra_params();
    CameraExtraParam extra_param(extra_data.c_str(), extra_data.size());

    auto ret = StartCamera(
        client_id, camera_id, framerate, extra_param, enable_result_cb);

    resp_msg.set_command(RECORDER_SERVICE_CMDS::RECORDER_START_CAMERA);
    resp_msg.set_status(ret);
  } break;
  case RECORDER_SERVICE_CMDS::RECORDER_STOP_CAMERA: {
    uint32_t client_id = req_msg.stop_camera().client_id();
    uint32_t camera_id = req_msg.stop_camera().camera_id();

    auto ret = StopCamera(client_id, camera_id);
    resp_msg.set_command(RECORDER_SERVICE_CMDS::RECORDER_STOP_CAMERA);
    resp_msg.set_status(ret);
  } break;
  case RECORDER_SERVICE_CMDS::RECORDER_CREATE_SESSION:
  {
    uint32_t client_id = req_msg.create_session().client_id();
    uint32_t session_id;
    auto ret = CreateSession (client_id, &session_id);
    resp_msg.set_command(RECORDER_SERVICE_CMDS::RECORDER_CREATE_SESSION);
    resp_msg.set_status(ret);
    resp_msg.mutable_create_session_resp()->set_session_id(session_id);
    break;
  }
  case RECORDER_SERVICE_CMDS::RECORDER_DELETE_SESSION:
  {
    uint32_t client_id = req_msg.delete_session().client_id();
    uint32_t session_id = req_msg.delete_session().session_id();
    auto ret = DeleteSession (client_id, session_id);
    resp_msg.set_command(RECORDER_SERVICE_CMDS::RECORDER_CREATE_SESSION);
    resp_msg.set_status(ret);
    break;
  }
  case RECORDER_SERVICE_CMDS::RECORDER_START_SESSION:
  {
    uint32_t client_id = req_msg.start_session().client_id();
    uint32_t session_id = req_msg.start_session().session_id();
    auto ret = StartSession (client_id, session_id);
    resp_msg.set_command(RECORDER_SERVICE_CMDS::RECORDER_START_SESSION);
    resp_msg.set_status(ret);
    break;
  }
  case RECORDER_SERVICE_CMDS::RECORDER_STOP_SESSION:
  {
    uint32_t client_id = req_msg.stop_session().client_id();
    uint32_t session_id = req_msg.stop_session().session_id();
    bool do_flush = req_msg.stop_session().do_flush();
    auto ret = StopSession (client_id, session_id, do_flush);
    resp_msg.set_command(RECORDER_SERVICE_CMDS::RECORDER_STOP_SESSION);
    resp_msg.set_status(ret);
    break;
  }
  case RECORDER_SERVICE_CMDS::RECORDER_PAUSE_SESSION:
  {
    uint32_t client_id = req_msg.pause_session().client_id();
    uint32_t session_id = req_msg.pause_session().session_id();
    auto ret = PauseSession (client_id, session_id);
    resp_msg.set_command(RECORDER_SERVICE_CMDS::RECORDER_PAUSE_SESSION);
    resp_msg.set_status(ret);
    break;
  }
  case RECORDER_SERVICE_CMDS::RECORDER_RESUME_SESSION:
  {
    uint32_t client_id = req_msg.resume_session().client_id();
    uint32_t session_id = req_msg.resume_session().session_id();
    auto ret = ResumeSession (client_id, session_id);
    resp_msg.set_command(RECORDER_SERVICE_CMDS::RECORDER_RESUME_SESSION);
    resp_msg.set_status(ret);
    break;
  }
  case RECORDER_SERVICE_CMDS::RECORDER_CREATE_VIDEOTRACK:
  {
    uint32_t client_id = req_msg.create_video_track().client_id();
    uint32_t session_id = req_msg.create_video_track().session_id();
    uint32_t track_id = req_msg.create_video_track().track_id();

    VideoTrackParam video_param;
    video_param.camera_id = req_msg.create_video_track().video_params().camera_id();
    video_param.width = req_msg.create_video_track().video_params().width();
    video_param.height = req_msg.create_video_track().video_params().height();
    video_param.framerate = req_msg.create_video_track().video_params().framerate();
    video_param.format =
        static_cast<VideoFormat>(req_msg.create_video_track().video_params().format());
    video_param.rotation =
        static_cast<Rotation>(req_msg.create_video_track().video_params().rotation());
    video_param.xtrabufs = req_msg.create_video_track().video_params().xtrabufs();
    video_param.flags =
        static_cast<VideoFlags>(req_msg.create_video_track().video_params().flags());

    const std::string extra_data = req_msg.create_video_track().extra_params();
    VideoExtraParam extra_param(extra_data.c_str(), extra_data.size());

    auto ret = CreateVideoTrack(
        client_id, session_id, track_id, video_param, extra_param);
    // sending response
    resp_msg.set_command(RECORDER_SERVICE_CMDS::RECORDER_CREATE_VIDEOTRACK);
    resp_msg.set_status(ret);
    break;
  }
  case RECORDER_SERVICE_CMDS::RECORDER_DELETE_VIDEOTRACK:
  {
    uint32_t client_id = req_msg.delete_video_track().client_id();
    uint32_t session_id = req_msg.delete_video_track().session_id();
    uint32_t track_id = req_msg.delete_video_track().track_id();

    auto ret = DeleteVideoTrack(client_id, session_id, track_id);
    // sending response
    resp_msg.set_command(RECORDER_SERVICE_CMDS::RECORDER_DELETE_VIDEOTRACK);
    resp_msg.set_status(ret);
    break;
  }
  case RECORDER_SERVICE_CMDS::RECORDER_RETURN_TRACKBUFFER:
  {
    uint32_t client_id = req_msg.return_track_buffer().client_id();
    uint32_t session_id = req_msg.return_track_buffer().session_id();
    uint32_t track_id = req_msg.return_track_buffer().track_id();
    std::vector<BnBuffer> buffers;

    for (const auto& buffer: req_msg.return_track_buffer().buffers()) {
      BnBuffer buf;
      buf.ion_fd = buffer.ion_fd();
      buf.ion_meta_fd = buffer.ion_meta_fd();
      buf.img_id = buffer.img_id();
      buf.size  = buffer.size();
      buf.timestamp = buffer.timestamp();
      buf.seqnum = buffer.seqnum();
      buf.buffer_id = buffer.buffer_id();
      buf.flags = buffer.flags();
      buf.capacity = buffer.capacity();
      buffers.push_back(buf);
    }

    auto ret = ReturnTrackBuffer(client_id, session_id, track_id, buffers);
    // sending response
    resp_msg.set_command(RECORDER_SERVICE_CMDS::RECORDER_RETURN_TRACKBUFFER);
    resp_msg.set_status(ret);
    break;
  }
  case RECORDER_SERVICE_CMDS::RECORDER_GET_VENDOR_TAG_DESCRIPTOR:
  {
    // uint64_t vendor_tag = req_msg.get_vendor_tag_descriptor().vendor_tag();
    // QMMF_INFO("Sending Message : %s", req_msg.DebugString());

    // // sending response
    // resp_msg.set_command(
    //     RECORDER_SERVICE_CMDS::RECORDER_GET_VENDOR_TAG_DESCRIPTOR);
    // resp_msg.mutable_get_vendor_tag_descriptor_resp()->set_vendortag(
    //     "dummy tag");
    break;
  }
  case RECORDER_SERVICE_CMDS::RECORDER_GET_CAMERA_CHARACTERISTICS:
  {
    uint32_t client_id = req_msg.get_camera_characteristics().client_id();
    uint32_t camera_id = req_msg.get_camera_characteristics().camera_id();
    CameraMetadata meta;

    auto ret = GetCameraCharacteristics (client_id, camera_id, meta);
    // sending response
    resp_msg.set_command(
        RECORDER_SERVICE_CMDS::RECORDER_GET_CAMERA_CHARACTERISTICS);
    resp_msg.set_status(ret);
    const camera_metadata_t *meta_buffer = meta.getAndLock();
    uint32_t size = get_camera_metadata_compact_size(meta_buffer);
    std::string *data = new std::string(reinterpret_cast<const char *>(meta_buffer), size);
    resp_msg.mutable_get_default_capture_param_resp()->set_allocated_meta(data);
    meta.unlock(meta_buffer);
    break;
  }
  case RECORDER_SERVICE_CMDS::RECORDER_GET_CAMERA_PARAMS:
  {
    uint32_t client_id = req_msg.get_camera_param().client_id();
    uint32_t camera_id = req_msg.get_camera_param().camera_id();
    CameraMetadata meta;

    auto ret = GetCameraParam (client_id, camera_id, meta);
    // sending response
    resp_msg.set_command(
        RECORDER_SERVICE_CMDS::RECORDER_GET_CAMERA_PARAMS);
    resp_msg.set_status(ret);
    const camera_metadata_t *meta_buffer = meta.getAndLock();
    uint32_t size = get_camera_metadata_compact_size(meta_buffer);
    std::string *data = new std::string(reinterpret_cast<const char *>(meta_buffer), size);
    resp_msg.mutable_get_camera_param_resp()->set_allocated_meta(data);
    meta.unlock(meta_buffer);
    break;
  }
  case RECORDER_SERVICE_CMDS::RECORDER_SET_CAMERA_PARAMS:
  {
    uint32_t client_id = req_msg.set_camera_param().client_id();
    uint32_t camera_id = req_msg.set_camera_param().camera_id();
    CameraMetadata meta;
    const std::string& data = req_msg.set_camera_param().meta();
    const camera_metadata_t *meta_buffer =
        reinterpret_cast <const camera_metadata_t *> (data.data());
    meta.clear();
    meta.append(clone_camera_metadata(meta_buffer));

    auto ret = SetCameraParam (client_id, camera_id, meta);
    // sending response
    resp_msg.set_command(
        RECORDER_SERVICE_CMDS::RECORDER_SET_CAMERA_PARAMS);
    resp_msg.set_status(ret);
    break;
  }
  case RECORDER_SERVICE_CMDS::RECORDER_SET_CAMERA_SESSION_PARAMS:
  {
    uint32_t client_id = req_msg.set_camera_session_param().client_id();
    uint32_t camera_id = req_msg.set_camera_session_param().camera_id();
    CameraMetadata meta;
    const std::string& data = req_msg.set_camera_session_param().meta();
    const camera_metadata_t *meta_buffer =
        reinterpret_cast <const camera_metadata_t *> (data.data());
    meta.clear();
    meta.append(clone_camera_metadata(meta_buffer));

    auto ret = SetCameraSessionParam (client_id, camera_id, meta);
    // sending response
    resp_msg.set_command(
        RECORDER_SERVICE_CMDS::RECORDER_SET_CAMERA_SESSION_PARAMS);
    resp_msg.set_status(ret);
    break;
  }
  case RECORDER_SERVICE_CMDS::RECORDER_CAPTURE_IMAGE:
  {
    uint32_t client_id = req_msg.capture_image().client_id();
    uint32_t camera_id = req_msg.capture_image().camera_id();
    SnapshotType type = static_cast<SnapshotType>(req_msg.capture_image().type());
    uint32_t n_images = req_msg.capture_image().n_images();
    std::vector<CameraMetadata> meta_array;

    for (const auto &meta_proto: req_msg.capture_image().meta()) {
      CameraMetadata meta;
      const camera_metadata_t *meta_buffer =
          reinterpret_cast <const camera_metadata_t *> (meta_proto.data());
      meta.clear();
      meta.append(clone_camera_metadata(meta_buffer));
      meta_array.push_back(meta);
    }

    auto ret = CaptureImage(client_id, camera_id,
                           static_cast<SnapshotType>(type), n_images,
                           meta_array);
    // sending response
    resp_msg.set_command(
        RECORDER_SERVICE_CMDS::RECORDER_CAPTURE_IMAGE);
    resp_msg.set_status(ret);
    break;
  }
  case RECORDER_SERVICE_CMDS::RECORDER_CONFIG_IMAGECAPTURE:
  {
    uint32_t client_id, camera_id, image_id;
    client_id = req_msg.config_image_capture().client_id();
    camera_id = req_msg.config_image_capture().camera_id();
    image_id = req_msg.config_image_capture().image_id();
    ImageParam image_param;
    image_param.mode =
        static_cast<ImageMode>(req_msg.config_image_capture().image_param().mode());
    image_param.width = req_msg.config_image_capture().image_param().width();
    image_param.height = req_msg.config_image_capture().image_param().height();
    image_param.format =
        static_cast<ImageFormat>(req_msg.config_image_capture().image_param().format());
    image_param.rotation =
        static_cast<Rotation>(req_msg.config_image_capture().image_param().rotation());
    image_param.quality = req_msg.config_image_capture().image_param().quality();

    const std::string extra_param = req_msg.config_image_capture().extra_param();
    ImageExtraParam xtraparam(extra_param.c_str(), extra_param.size());
    auto ret = ConfigImageCapture(client_id, camera_id, image_id, image_param, xtraparam);

    resp_msg.set_command(
        RECORDER_SERVICE_CMDS::RECORDER_CONFIG_IMAGECAPTURE);
    resp_msg.set_status(ret);
    break;
  }
  case RECORDER_SERVICE_CMDS::RECORDER_CANCEL_IMAGECAPTURE:
  {
    uint32_t client_id, camera_id, image_id;
    bool cache;
    client_id = req_msg.cancel_image_capture().client_id();
    camera_id = req_msg.cancel_image_capture().camera_id();
    image_id = req_msg.cancel_image_capture().image_id();
    cache = req_msg.cancel_image_capture().cache();
    auto ret = CancelCaptureImage(client_id, camera_id, image_id, cache);
    // sending response
    resp_msg.set_command(
        RECORDER_SERVICE_CMDS::RECORDER_CANCEL_IMAGECAPTURE);
    resp_msg.set_status(ret);
    break;
  }
  case RECORDER_SERVICE_CMDS::RECORDER_RETURN_IMAGECAPTURE_BUFFER:
  {
    uint32_t client_id, camera_id, buffer_id;
    client_id = req_msg.return_image_capture_buffer().client_id();
    camera_id = req_msg.return_image_capture_buffer().camera_id();
    buffer_id = req_msg.return_image_capture_buffer().buffer_id();
    auto ret = ReturnImageCaptureBuffer(client_id, camera_id, buffer_id);

    // sending response
    resp_msg.set_command(
        RECORDER_SERVICE_CMDS::RECORDER_RETURN_IMAGECAPTURE_BUFFER);
    resp_msg.set_status(ret);
    break;
  }
  case RECORDER_SERVICE_CMDS::RECORDER_GET_DEFAULT_CAPTURE_PARAMS:
  {
    uint32_t client_id = req_msg.get_default_capture_param().client_id();
    uint32_t camera_id = req_msg.get_default_capture_param().camera_id();

    CameraMetadata meta;
    auto ret = GetDefaultCaptureParam(client_id, camera_id, meta);
    // sending response
    resp_msg.set_command(
        RECORDER_SERVICE_CMDS::RECORDER_GET_DEFAULT_CAPTURE_PARAMS);
    resp_msg.set_status(ret);
    const camera_metadata_t *meta_buffer = meta.getAndLock();
    uint32_t size = get_camera_metadata_compact_size(meta_buffer);
    std::string *data = new std::string(reinterpret_cast<const char *>(meta_buffer), size);
    resp_msg.mutable_get_default_capture_param_resp()->set_allocated_meta(data);
    meta.unlock(meta_buffer);
    break;
  }

  default:
    QMMF_WARN ("%s: cmd: %u, Not sending.", __func__, req_msg.command());
    break;
  }

  auto size = resp_msg.ByteSizeLong();
  void *buffer = malloc(size);
  resp_msg.SerializeToArray(buffer, size);

  if (SendResponse(client_socket, buffer, size) > 0)
    QMMF_INFO("%s: sent cmd:%u bytes:%lu", __func__,  resp_msg.command(), size);

  free (buffer);
}

void RecorderService::MainLoop() {
  while (run_) {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(socket_, &read_fds);
    int max_socket = socket_;

    // Add all active clients to the monitoring list
    for (const auto& [socket, id]: client_sockets_) {
      FD_SET(socket, &read_fds);
      max_socket = std::max(max_socket, socket);
    }

    // Use select to wait for socket activity
    if (select(max_socket + 1, &read_fds, nullptr, nullptr, nullptr) == -1) {
      QMMF_ERROR("%s: select failure %s", __func__, strerror(errno));
      break;
    }

    // Check if a new client is trying to connect
    if (FD_ISSET(socket_, &read_fds)) {
      int client_socket = accept(socket_, nullptr, nullptr);
      if (client_socket != -1) {
        client_sockets_.emplace(client_socket, 0);
        QMMF_INFO("%s: new client(%d) connected", __func__, client_socket);
      }
    }

    // Check for incoming data on client sockets
    auto it = client_sockets_.begin();
    while (it != client_sockets_.end()) {
      int client_socket = it->first;
      if (FD_ISSET(client_socket, &read_fds)) {
        char buffer[kMaxSocketBufSize] = {0};

        QMMF_INFO("%s: Waiting for data from client(%d)",
                  __func__, client_socket);

        auto bytes_read = ReadData(client_socket, buffer, kMaxSocketBufSize);
        if (bytes_read <= 0) {
          FD_CLR(client_socket, &read_fds);
          QMMF_INFO("%s: remove client(%d)", __func__, client_socket);
          // client session destructor
          it = client_sockets_.erase(it);
          continue;
        } else {
          // Deserialize the received data using protobuf
          RecorderClientReqMsg cmd_msg;
          cmd_msg.ParseFromArray(buffer, bytes_read);
          thread_pool_.Enqueue(
              [cmd_msg, this, client_socket] { ProcessRequest(client_socket, cmd_msg); },
              TASK_PRIORITY::NORMAL);
        }
      }
      ++it;
    }
  }
}

status_t RecorderService::SetupRemoteCallback(const uint32_t client_id) {

  QMMF_INFO("%s: Enter client_id(%d)", __func__, client_id);

  std::shared_ptr<IRecorderServiceCallback> service_cb =
      std::make_shared<RecorderServiceCallbackProxy> ();
  if (!service_cb) {
    QMMF_ERROR("%s: Unable to create service callback proxy!", __func__);
    return -ENODEV;
  }

  auto ret = service_cb->Init(client_id);
  if (ret != 0) {
    QMMF_ERROR("%s: service callback proxy init failed", __func__);
    return ret;
  }

  std::shared_ptr<RemoteCallBack> remote_callback =
      std::make_shared<RemoteCallBack>(client_id, service_cb);
  if (!remote_callback) {
      QMMF_ERROR("%s: Unable to allocate remote callback!", __func__);
      return -ENODEV;
  }

  remote_cb_list_.emplace(client_id, remote_callback);

  QMMF_INFO("%s: Exit client_id(%d)", __func__, client_id);

  return 0;
}

status_t RecorderService::Connect(const std::shared_ptr<IRecorderServiceCallback>&
                                  service_cb,
                                  uint32_t* client_id) {

  QMMF_DEBUG("%s: Enter ", __func__);
  QMMF_KPI_DETAIL();
  status_t ret;

  std::lock_guard<std::mutex> lock(lock_);

  if (!recorder_) {
    recorder_.reset(RecorderImpl::CreateRecorder());
    if (!recorder_) {
      QMMF_ERROR("%s: Can't create Recorder Instance!!", __func__);
      return -ENOMEM;
    }
    std::function< const std::shared_ptr<RemoteCallBack>& (uint32_t id)>
      remote_cb_handle = [&] (uint32_t id) ->  std::shared_ptr<RemoteCallBack>& {
        QMMF_VERBOSE("%s: Client(%u): RemoteCallback request!",
                      __func__, id);
        assert(remote_cb_list_.count(id) != 0);
        return remote_cb_list_[id];
    };
    ret = recorder_->Init(remote_cb_handle);
    if (ret != 0) {
      QMMF_ERROR("%s: Recorder initialization failed!", __func__);
      recorder_.reset();
      return ret;
    }
  }

  ret = GetUniqueClientID(client_id);
  if (ret != 0) {
    QMMF_ERROR("%s: Too many active clients (255)!", __func__);
    return ret;
  }

  recorder_->RegisterClient(*client_id);

  NotifyClientDeath notify_death = [this, capture_client_id = *client_id] {
      ClientDeathHandler(capture_client_id);
  };

  std::shared_ptr<DeathNotifier> death_notifier =
      std::make_shared<DeathNotifier>(notify_death);
  if (!death_notifier.get()) {
    QMMF_ERROR("%s: Unable to allocate death notifier!", __func__);
    return -ENODEV;
  }

  death_notifier_list_.emplace(*client_id, death_notifier);

  QMMF_INFO("%s: Service is connected with client (%d)", __func__, *client_id);

  QMMF_DEBUG("%s: Exit client_id(%d)", __func__, *client_id);
  return ret;
}

#endif // HAVE_BINDER

status_t RecorderService::Disconnect(uint32_t client_id) {

  QMMF_INFO("%s: Enter client_id(%d)", __func__, client_id);
  QMMF_KPI_DETAIL();
  std::lock_guard<std::mutex> lock(lock_);

  if (!recorder_) {
    QMMF_ERROR("%s: Recorder not initialized!", __func__);
    return -ENODEV;
  }

  if (death_notifier_list_.count(client_id) == 0) {
    QMMF_ERROR("%s: Death notifier for client id %d does not exist!",
        __func__, client_id);
    return -EINVAL;
  }

  if (remote_cb_list_.count(client_id) == 0) {
    QMMF_ERROR("%s: Remote callback for client id %d does not exist!",
        __func__, client_id);
    return -EINVAL;
  }

  recorder_->DeRegisterClient(client_id, true);

#ifdef HAVE_BINDER
  sp<DeathNotifier> notifier = death_notifier_list_[client_id];
  sp<RemoteCallBack> callback = remote_cb_list_[client_id];

  IInterface::asBinder(callback->getRemoteClient())->unlinkToDeath(notifier);
#endif // HAVE_BINDER

  death_notifier_list_.erase(client_id);
  remote_cb_list_.erase(client_id);

  if (death_notifier_list_.empty() && remote_cb_list_.empty()) {
    if (recorder_) {
      QMMF_INFO("%s: No client is connected! de-init the recorder!", __func__);
      recorder_->DeInit();
      recorder_.reset();
    }
  }

  QMMF_INFO("%s: Exit client_id(%d)", __func__, client_id);
  return 0;
}

status_t RecorderService::StartCamera(const uint32_t client_id,
                                      const uint32_t camera_id,
                                      const float framerate,
                                      const CameraExtraParam& extra_param,
                                      bool enable_result_cb) {

  QMMF_INFO("%s: Enter client_id(%d)", __func__, client_id);
  QMMF_KPI_DETAIL();

  if (!IsRecorderInitialized()) {
    QMMF_ERROR("%s: Recorder not initialized!", __func__);
    return -EINVAL;
  }

  auto ret = recorder_->StartCamera(client_id, camera_id, framerate,
                                    extra_param,
                                    enable_result_cb);
  if(ret != 0) {
    QMMF_ERROR("%s: Can't start Camera!!", __func__);
    return ret;
  }
  QMMF_INFO("%s: Exit client_id(%d)", __func__, client_id);
  return 0;
}

status_t RecorderService::StopCamera(const uint32_t client_id,
                                     const uint32_t camera_id) {

  QMMF_INFO("%s: Enter client_id(%d)", __func__, client_id);
  QMMF_KPI_DETAIL();

  if (!IsRecorderInitialized()) {
    QMMF_ERROR("%s: Recorder not initialized!", __func__);
    return -ENODEV;
  }

  auto ret = recorder_->StopCamera(client_id, camera_id);
  if(ret != 0) {
    QMMF_ERROR("%s: Can't Stop Camera!!", __func__);
    return ret;
  }
  QMMF_INFO("%s: Exit client_id(%d)", __func__, client_id);
  return 0;
}

status_t RecorderService::CreateSession(const uint32_t client_id,
                                        uint32_t *session_id) {

  QMMF_INFO("%s: Enter client_id(%d)", __func__, client_id);
  QMMF_KPI_DETAIL();

  if (!IsRecorderInitialized()) {
    QMMF_ERROR("%s: Recorder not initialized!", __func__);
    return -ENODEV;
  }

  uint32_t id;
  auto ret = recorder_->CreateSession(client_id, &id);
  if (ret != 0) {
    QMMF_ERROR("%s: CreateSession failed!", __func__);
    return ret;
  }
  *session_id = id;

  QMMF_INFO("%s: Exit client_id(%d)", __func__, client_id);
  return 0;
}

status_t RecorderService::DeleteSession(const uint32_t client_id,
                                        const uint32_t session_id) {

  QMMF_INFO("%s: Enter client_id(%d)", __func__, client_id);
  QMMF_KPI_DETAIL();

  if (!IsRecorderInitialized()) {
    QMMF_ERROR("%s: Recorder not initialized!", __func__);
    return -ENODEV;
  }

  auto ret = recorder_->DeleteSession(client_id, session_id);
  if (ret != 0) {
    QMMF_ERROR("%s: DeleteSession failed!", __func__);
    return ret;
  }
  QMMF_INFO("%s: Exit client_id(%d)", __func__, client_id);
  return 0;
}

status_t RecorderService::StartSession(const uint32_t client_id,
                                       const uint32_t session_id) {

  QMMF_INFO("%s: Enter client_id(%d)", __func__, client_id);
  QMMF_KPI_DETAIL();

  if (!IsRecorderInitialized()) {
    QMMF_ERROR("%s: Recorder not initialized!", __func__);
    return -ENODEV;
  }

  QMMF_INFO("%s: Session_id(%d) to be Start", __func__, session_id);

  auto ret = recorder_->StartSession(client_id, session_id);
  if (ret != 0) {
    QMMF_ERROR("%s: StartSession failed!", __func__);
    return ret;
  }
  QMMF_INFO("%s: Exit client_id(%d)", __func__, client_id);
  return 0;
}

status_t RecorderService::StopSession(const uint32_t client_id,
                                      const uint32_t session_id,
                                      bool do_flush) {

  QMMF_INFO("%s: Enter client_id(%d)", __func__, client_id);
  QMMF_KPI_DETAIL();

  if (!IsRecorderInitialized()) {
    QMMF_ERROR("%s: Recorder not initialized!", __func__);
    return -ENODEV;
  }

  QMMF_INFO("%s: Session_id(%d) to be Stop with flash=%d", __func__,
                                      session_id, do_flush);

  auto ret = recorder_->StopSession(client_id, session_id, do_flush);
  if (ret != 0) {
    QMMF_ERROR("%s: StopSession failed!", __func__);
    return ret;
  }
  QMMF_INFO("%s: Exit client_id(%d)", __func__, client_id);
  return 0;
}

status_t RecorderService::PauseSession(const uint32_t client_id,
                                       const uint32_t session_id) {

  QMMF_INFO("%s: Enter client_id(%d)", __func__, client_id);
  QMMF_KPI_DETAIL();

  if (!IsRecorderInitialized()) {
    QMMF_ERROR("%s: Recorder not initialized!", __func__);
    return -ENODEV;
  }

  QMMF_INFO("%s: Session_id(%d) to be Pause", __func__, session_id);

  auto ret = recorder_->PauseSession(client_id, session_id);
  if (ret != 0) {
    QMMF_ERROR("%s: PauseSession failed!", __func__);
    return ret;
  }
  QMMF_INFO("%s: Exit client_id(%d)", __func__, client_id);
  return 0;
}

status_t RecorderService::ResumeSession(const uint32_t client_id,
                                        const uint32_t session_id) {

  QMMF_INFO("%s: Enter client_id(%d)", __func__, client_id);
  QMMF_KPI_DETAIL();

  if (!IsRecorderInitialized()) {
    QMMF_ERROR("%s: Recorder not initialized!", __func__);
    return -ENODEV;
  }

  QMMF_INFO("%s: Session_id(%d) to be Resume", __func__, session_id);

  auto ret = recorder_->ResumeSession(client_id, session_id);
  if (ret != 0) {
    QMMF_ERROR("%s: ResumeSession failed!", __func__);
    return ret;
  }
  QMMF_INFO("%s: Exit client_id(%d)", __func__, client_id);
  return 0;
}

status_t RecorderService::CreateVideoTrack(const uint32_t client_id,
                                           const uint32_t session_id,
                                           const uint32_t track_id,
                                           const VideoTrackParam& param,
                                           const VideoExtraParam& xtraparam) {

  QMMF_INFO("%s: Enter client_id(%d)", __func__, client_id);

  if (!IsRecorderInitialized()) {
    QMMF_ERROR("%s: Recorder not initialized!", __func__);
    return -ENODEV;
  }

  uint32_t id = track_id & 0xffff0000;
  if (id > 0) {
    QMMF_INFO("%s: track_id should be 16 bit number!", __func__);
    return -EINVAL;
  }

  auto ret = recorder_->CreateVideoTrack(client_id, session_id, track_id,
                                         param, xtraparam);

  if (ret != 0) {
    QMMF_INFO("%s: CreateVideoTrackWithExtraParam failed!", __func__);
    return ret;
  }
  QMMF_INFO("%s: Exit client_id(%d)", __func__, client_id);
  return 0;
}

status_t RecorderService::DeleteVideoTrack(const uint32_t client_id,
                                           const uint32_t session_id,
                                           const uint32_t track_id) {

  QMMF_INFO("%s: Enter client_id(%d)", __func__, client_id);
  QMMF_KPI_DETAIL();

  if (!IsRecorderInitialized()) {
    QMMF_ERROR("%s: Recorder not initialized!", __func__);
    return -ENODEV;
  }

  auto ret = recorder_->DeleteVideoTrack(client_id, session_id, track_id);
  if (ret != 0) {
    QMMF_INFO("%s: DeleteVideoTrack failed!", __func__);
    return ret;
  }
  QMMF_INFO("%s: Exit client_id(%d)", __func__, client_id);
  return 0;
}

status_t RecorderService::ReturnTrackBuffer(const uint32_t client_id,
                                            const uint32_t session_id,
                                            const uint32_t track_id,
                                            std::vector<BnBuffer> &buffers) {

  QMMF_VERBOSE("%s: Enter client_id(%d)", __func__, client_id);

  if (!IsRecorderInitialized()) {
    QMMF_ERROR("%s: Recorder not initialized!", __func__);
    return -ENODEV;
  }

  auto ret = recorder_->ReturnTrackBuffer(client_id, session_id, track_id,
                                          buffers);
  if (ret != 0) {
    QMMF_INFO("%s: ReturnTrackBuffer failed!", __func__);
    return -EINVAL;
  }
  QMMF_VERBOSE("%s: Exit client_id(%d)", __func__, client_id);
  return ret;
}

status_t RecorderService::SetVideoTrackParam(const uint32_t client_id,
                                             const uint32_t session_id,
                                             const uint32_t track_id,
                                             VideoParam type,
                                             void *param,
                                             size_t size) {

  QMMF_INFO("%s: Enter client_id(%d)", __func__, client_id);

  if (!IsRecorderInitialized()) {
    QMMF_ERROR("%s: Recorder not initialized!", __func__);
    return -ENODEV;
  }

  auto ret = recorder_->SetVideoTrackParam(client_id, session_id, track_id,
                                           type, param, size);
  if (ret != 0) {
    QMMF_ERROR("%s: SetVideoTrackParam failed!", __func__);
    return ret;
  }
  QMMF_INFO("%s: Exit client_id(%d)", __func__, client_id);
  return 0;
}

status_t RecorderService::CaptureImage(const uint32_t client_id,
                                       const uint32_t camera_id,
                                       const SnapshotType type,
                                       const uint32_t n_images,
                                       const std::vector<CameraMetadata> &meta) {

  QMMF_INFO("%s: Enter client_id(%d)", __func__, client_id);

  if (!IsRecorderInitialized()) {
    QMMF_ERROR("%s: Recorder not initialized!", __func__);
    return -ENODEV;
  }

  auto ret = recorder_->CaptureImage(client_id, camera_id, type, n_images, meta);
  if (ret != 0) {
    QMMF_ERROR("%s: CaptureImage failed!", __func__);
    return ret;
  }
  QMMF_INFO("%s: Exit client_id(%d)", __func__, client_id);
  return 0;
}

status_t RecorderService::ConfigImageCapture(const uint32_t client_id,
                                             const uint32_t camera_id,
                                             const uint32_t image_id,
                                             const ImageParam &param,
                                             const ImageExtraParam &xtrapram) {

  QMMF_INFO("%s: Enter client_id(%d)", __func__, client_id);

  if (!IsRecorderInitialized()) {
    QMMF_ERROR("%s: Recorder not initialized!", __func__);
    return -ENODEV;
  }

  auto ret = recorder_->ConfigImageCapture(client_id, camera_id, image_id, 
                                           param, xtrapram);
  if (ret != 0) {
    QMMF_ERROR("%s: ConfigImageCapture failed!", __func__);
    return ret;
  }
  QMMF_INFO("%s: Exit client_id(%d)", __func__, client_id);
  return 0;
}

status_t RecorderService::CancelCaptureImage(const uint32_t client_id,
                                             const uint32_t camera_id,
                                             const uint32_t image_id,
                                             const bool cache) {

  QMMF_INFO("%s: Enter client_id(%d)", __func__, client_id);

  if (!IsRecorderInitialized()) {
    QMMF_ERROR("%s: Recorder not initialized!", __func__);
    return -ENODEV;
  }

  auto ret = recorder_->CancelCaptureImage(client_id, camera_id, image_id,
                                           cache);
  if (ret != 0) {
    QMMF_ERROR("%s: CancelCaptureImage failed!", __func__);
    return ret;
  }
  QMMF_INFO("%s: Exit client_id(%d)", __func__, client_id);
  return 0;
}


status_t RecorderService::ReturnImageCaptureBuffer(const uint32_t client_id,
                                                   const uint32_t camera_id,
                                                   const int32_t buffer_id) {

  QMMF_INFO("%s: Enter client_id(%d)", __func__, client_id);

  if (!IsRecorderInitialized()) {
    QMMF_ERROR("%s: Recorder not initialized!", __func__);
    return -ENODEV;
  }

  auto ret = recorder_->ReturnImageCaptureBuffer(client_id, camera_id,
                                                 buffer_id);
  if (ret != 0) {
    QMMF_ERROR("%s: ReturnImageCaptureBuffer failed!", __func__);
    return ret;
  }
  QMMF_INFO("%s: Exit client_id(%d)", __func__, client_id);
  return 0;
}

status_t RecorderService::SetCameraParam(const uint32_t client_id,
                                         const uint32_t camera_id,
                                         const CameraMetadata &meta) {

  QMMF_INFO("%s: Enter client_id(%d)", __func__, client_id);

  if (!IsRecorderInitialized()) {
    QMMF_ERROR("%s: Recorder not initialized!", __func__);
    return -ENODEV;
  }

  auto ret = recorder_->SetCameraParam(client_id, camera_id, meta);
  if (ret != 0) {
    QMMF_ERROR("%s: SetCameraParam failed!", __func__);
    return ret;
  }
  QMMF_INFO("%s: Exit client_id(%d)", __func__, client_id);
  return 0;
}

status_t RecorderService::GetCameraParam(const uint32_t client_id,
                                         const uint32_t camera_id,
                                         CameraMetadata &meta) {

  QMMF_INFO("%s: Enter client_id(%d)", __func__, client_id);

  if (!IsRecorderInitialized()) {
    QMMF_ERROR("%s: Recorder not initialized!", __func__);
    return -ENODEV;
  }

  auto ret = recorder_->GetCameraParam(client_id, camera_id, meta);
  if (ret != 0) {
    QMMF_ERROR("%s: GetCameraParam failed!", __func__);
    return ret;
  }
  QMMF_INFO("%s: Exit client_id(%d)", __func__, client_id);
  return 0;
}

status_t RecorderService::SetCameraSessionParam(const uint32_t client_id,
                                                const uint32_t camera_id,
                                                const CameraMetadata &meta) {

  QMMF_INFO("%s: Enter client_id(%d)", __func__, client_id);

  if (!IsRecorderInitialized()) {
    QMMF_ERROR("%s: Recorder not initialized!", __func__);
    return -ENODEV;
  }

  auto ret = recorder_->SetCameraSessionParam(client_id, camera_id, meta);
  if (ret != 0) {
    QMMF_ERROR("%s: SetCameraSessionParam failed!", __func__);
    return ret;
  }
  QMMF_INFO("%s: Exit client_id(%d)", __func__, client_id);
  return 0;
}

status_t RecorderService::SetSHDR(const uint32_t client_id,
                                     const uint32_t camera_id,
                                     const bool enable) {

  QMMF_INFO("%s: Enter client_id(%d)", __func__, client_id);

  if (!IsRecorderInitialized()) {
    QMMF_ERROR("%s: Recorder not initialized!", __func__);
    return -ENODEV;
  }

  auto ret = recorder_->SetSHDR(client_id, camera_id, enable);
  if (ret != 0) {
    QMMF_ERROR("%s: GetCameraParam failed!", __func__);
    return ret;
  }
  QMMF_INFO("%s: Exit client_id(%d)", __func__, client_id);
  return 0;
}

status_t RecorderService::GetDefaultCaptureParam(const uint32_t client_id,
                                                 const uint32_t camera_id,
                                                 CameraMetadata &meta) {

  QMMF_INFO("%s: Enter client_id(%d)", __func__, client_id);

  if (!IsRecorderInitialized()) {
    QMMF_ERROR("%s: Recorder not initialized!", __func__);
    return -ENODEV;
  }

  auto ret = recorder_->GetDefaultCaptureParam(client_id, camera_id, meta);
  if (ret != 0) {
    QMMF_ERROR("%s: GetDefaultCaptureParam failed!", __func__);
    return ret;
  }
  QMMF_INFO("%s: Exit client_id(%d)", __func__, client_id);
  return 0;
}

status_t RecorderService::GetCameraCharacteristics(const uint32_t client_id,
                                                   const uint32_t camera_id,
                                                   CameraMetadata &meta) {

  QMMF_INFO("%s: Enter client_id(%d)", __func__, client_id);

  if (!IsRecorderInitialized()) {
    QMMF_ERROR("%s: Recorder not initialized!", __func__);
    return -ENODEV;
  }

  auto ret = recorder_->GetCameraCharacteristics(client_id, camera_id, meta);
  if (ret != 0) {
    QMMF_ERROR("%s: GetCameraCharacteristics failed!", __func__);
    return ret;
  }
  QMMF_INFO("%s: Exit client_id(%d)", __func__, client_id);
  return 0;
}

status_t RecorderService::CreateOfflineJPEG(
                                      const uint32_t client_id,
                                      const OfflineJpegCreateParams &params) {

  QMMF_INFO("%s:Enter client_id(%d)", __func__, client_id);

  if (!IsRecorderInitialized()) {
    QMMF_ERROR("%s: Recorder not initialized!", __func__);
    return -ENODEV;
  }
  auto ret = recorder_->CreateOfflineJPEG(client_id, params);
  if (ret != 0) {
    QMMF_ERROR("%s: Can't create Offline JPEG PostProcessor!", __func__);
    return ret;
  }

  QMMF_INFO("%s: Exit client_id(%d)", __func__, client_id);
  return ret;
}

status_t RecorderService::EncodeOfflineJPEG(const uint32_t client_id,
                                            const BnBuffer& in_buf,
                                            const BnBuffer& out_buf,
                                            const OfflineJpegMeta& meta) {

  QMMF_INFO("%s: Enter client_id(%d)", __func__, client_id);

  if (!IsRecorderInitialized()) {
    QMMF_ERROR("%s: Recorder not initialized!", __func__);
    return -ENODEV;
  }

  auto ret = recorder_->EncodeOfflineJPEG(client_id, in_buf, out_buf, meta);
  if (ret != 0) {
    QMMF_ERROR("%s: Submitting request failed", __func__);
    return ret;
  }

  QMMF_INFO("%s: Exit client_id(%d)", __func__, client_id);

  return ret;
}

status_t RecorderService::DestroyOfflineJPEG(const uint32_t client_id) {

  QMMF_INFO("%s: Enter client_id(%d)", __func__, client_id);

  if (!IsRecorderInitialized()) {
    QMMF_ERROR("%s: Recorder not initialized!", __func__);
    return -ENODEV;
  }
  auto ret = recorder_->DestroyOfflineJPEG(client_id);
  if (ret != 0) {
    QMMF_ERROR("%s: Destroy failed", __func__);
    return ret;
  }

  QMMF_INFO("%s: Exit client_id(%d)", __func__, client_id);

  return ret;
}

void RecorderService::ClientDeathHandler(const uint32_t client_id) {
  QMMF_INFO("%s: client_id(%d) died in battle!", __func__, client_id);
  // Internal disconnect, it would trigger resource cleanup belongs to died
  // client.
  DisconnectInternal(client_id);
}

bool RecorderService::IsRecorderInitialized() {

  std::lock_guard<std::mutex> lock(lock_);
  return (recorder_) ? true : false;
}

status_t RecorderService::DisconnectInternal(const uint32_t client_id) {

  QMMF_INFO("%s: Enter client_id(%d)", __func__, client_id);
  std::lock_guard<std::mutex> lock(lock_);

  if (!recorder_) {
    QMMF_ERROR("%s: Recorder not initialized!", __func__);
    return -ENODEV;
  }

  if (death_notifier_list_.count(client_id) == 0) {
    QMMF_ERROR("%s: Death notifier for client id %d does not exist!",
        __func__, client_id);
    return -EINVAL;
  }

  if (remote_cb_list_.count(client_id) == 0) {
    QMMF_ERROR("%s: Remote callback for client id %d does not exist!",
        __func__, client_id);
    return -EINVAL;
  }

  // Forceful cleanup.
  recorder_->DeRegisterClient(client_id, true);
#ifdef HAVE_BINDER
  sp<DeathNotifier> notifier = death_notifier_list_[client_id];
  sp<RemoteCallBack> callback = remote_cb_list_[client_id];

  IInterface::asBinder(callback->getRemoteClient())->unlinkToDeath(notifier);
#endif
  death_notifier_list_.erase(client_id);
  remote_cb_list_.erase(client_id);

  if (death_notifier_list_.empty() && remote_cb_list_.empty()) {
    if (recorder_) {
      QMMF_INFO("%s: No client is connected! de-init the recorder!", __func__);
      recorder_->DeInit();
      recorder_.reset();
    }
  }

  QMMF_INFO("%s: Exit client_id(%d)", __func__, client_id);
  return 0;
}

status_t RecorderService::GetVendorTagDescriptor(std::shared_ptr<VendorTagDescriptor> &desc) {

  desc = VendorTagDescriptor::getGlobalVendorTagDescriptor();
  return (desc == nullptr) ? -EINVAL : 0;
}

status_t RecorderService::GetUniqueClientID(uint32_t *client_id) {

  for (uint32_t id = 1; id <= 0xFF; id++) {
    if (remote_cb_list_.count(id) == 0) {
      *client_id = id;
      return 0;
    }
  }
  return -EINVAL;
}

#ifndef HAVE_BINDER
status_t RecorderServiceCallbackProxy::Init (uint32_t client_id,
                                             uint32_t server_pid) {
  QMMF_INFO("%s: Enter ", __func__);

  std::stringstream ss;
  ss << "/var/run/le_cam_client." << client_id << ".sock";
  std::string socket_path = ss.str();

  QMMF_INFO("Connecting to... %s", socket_path.c_str());

  // Create a socket
  callback_socket_ = socket(AF_UNIX, SOCK_STREAM, 0);
  if (callback_socket_ == -1) {
    QMMF_ERROR("Callback socket failure %s", strerror(errno));
    return -errno;
  }

  // Set up server address
  struct sockaddr_un server_addr;
  server_addr.sun_family = AF_UNIX;
  auto size = socket_path.size();
  snprintf(server_addr.sun_path, size+1, "%s", socket_path.c_str());
  server_addr.sun_path[size+1] = '\0';

  // Connect to the server
  if (connect(callback_socket_, (struct sockaddr *)&server_addr,
              sizeof(server_addr)) == -1) {
    QMMF_ERROR("Callback socket connect failure %s", strerror(errno));
    close(callback_socket_);
    return -errno;
  }

  client_id_ = client_id;
  QMMF_INFO("%s: Exit client_id(%d) (0x%p)", __func__, client_id_);
  return 0;
}

void RecorderServiceCallbackProxy::NotifyRecorderEvent(EventType event, void *payload, size_t size) {

  QMMF_DEBUG("%s Enter ", __func__);
  RecorderClientCallbacksAsync async_msg;
  async_msg.set_cmd(RECORDER_SERVICE_CB_CMDS::RECORDER_NOTIFY_EVENT);
  NotifyRecorderEventMsg *event_msg = async_msg.mutable_recorder_event();
  event_msg->set_type(static_cast<EventTypeMsg>(event));
  std::string *data = new std::string(
    reinterpret_cast<const char*>(payload), size);
  event_msg->set_allocated_event_msg(data);

  auto msg_size = async_msg.ByteSizeLong();
  void *buffer = malloc(size);
  async_msg.SerializeToArray(buffer, msg_size);
  ssize_t bytesSent = send(callback_socket_, buffer, msg_size, 0);
  QMMF_VERBOSE("%s bytesSent: %u", __func__, bytesSent);
  if (bytesSent == -1) {
    QMMF_ERROR("%s: Closing callback socket: %d", __func__, callback_socket_);
    close (callback_socket_);
  }

  free (buffer);

  QMMF_DEBUG("%s Exit ", __func__);
}

void RecorderServiceCallbackProxy::NotifySessionEvent(EventType event_type, void *event_data,
                        size_t event_data_size) {

}

void RecorderServiceCallbackProxy::NotifySnapshotData(uint32_t camera_id, uint32_t imgcount,
                        BnBuffer& bn_buffer, BufferMeta& meta) {
  QMMF_VERBOSE("%s: Enter camera_id(%u), imgcount(%u)",
      __func__, camera_id, imgcount);

  RecorderClientCallbacksAsync async_msg;
  async_msg.set_cmd(RECORDER_SERVICE_CB_CMDS::RECORDER_NOTIFY_SNAPSHOT_DATA);
  NotifySnapshotDataMsg *snapshot_msg = async_msg.mutable_snapshot_data();
  snapshot_msg->set_camera_id(camera_id);
  snapshot_msg->set_img_count(imgcount);
  BufferInfoMsg* buffer_info = snapshot_msg->mutable_buffer();
  buffer_info->set_ion_fd(bn_buffer.ion_fd);
  buffer_info->set_ion_meta_fd(bn_buffer.ion_meta_fd);
  buffer_info->set_img_id(bn_buffer.img_id);
  buffer_info->set_size(bn_buffer.size);
  buffer_info->set_timestamp(bn_buffer.timestamp);
  buffer_info->set_seqnum(bn_buffer.seqnum);
  buffer_info->set_buffer_id(bn_buffer.buffer_id);
  buffer_info->set_flags(bn_buffer.flags);
  buffer_info->set_capacity(bn_buffer.capacity);

  BufferMetaMsg* meta_msg = snapshot_msg->mutable_meta();
  meta_msg->set_format(static_cast<BufferFormatMsg>(meta.format));
  meta_msg->set_n_planes(meta.n_planes);
  for (auto &&p_info : meta.planes) {
    PlaneInfoMsg* info = meta_msg->add_info();
    info->set_stride(p_info.stride);
    info->set_scanline(p_info.scanline);
    info->set_width(p_info.width);
    info->set_height(p_info.height);
    info->set_offset(p_info.offset);
    info->set_size(p_info.size);
  }

  auto size = async_msg.ByteSizeLong();
  void *buffer = malloc(size);
  async_msg.SerializeToArray(buffer, size);
  ssize_t bytesSent = send(callback_socket_, buffer, size, 0);
  QMMF_VERBOSE("%s bytesSent: %u", __func__, bytesSent);
  if (bytesSent == -1) {
    QMMF_ERROR("%s: Closing callback socket: %d", __func__, callback_socket_);
    close (callback_socket_);
  }

  free (buffer);
  QMMF_VERBOSE("%s: Exit camera_id(%u), imgcount(%u)",
      __func__, camera_id, imgcount);
}

void RecorderServiceCallbackProxy::NotifyOfflineJpegData(int32_t buf_fd, uint32_t encoded_size) {

}

void RecorderServiceCallbackProxy::NotifyVideoTrackData(uint32_t session_id, uint32_t track_id,
                          std::vector<BnBuffer>& buffers,
                          std::vector<BufferMeta>& metas) {

  QMMF_VERBOSE("%s: Enter client_id(%u), session_id(%u), track_id(%u)",
      __func__, client_id_, session_id, track_id);

  RecorderClientCallbacksAsync async_msg;
  async_msg.set_cmd(RECORDER_SERVICE_CB_CMDS::RECORDER_NOTIFY_VIDEO_TRACK_DATA);
  class NotifyVideoTrackDataMsg *nvt = async_msg.mutable_video_track_data();
  nvt->set_session_id(session_id);
  nvt->set_track_id(track_id);
  for (auto &&buffer : buffers) {
    QMMF_VERBOSE("%s: INPARAM: buffers[%s]", __func__,
                  buffer.ToString().c_str());
    BufferInfoMsg buffer_info;
    buffer_info.set_ion_fd(buffer.ion_fd);
    buffer_info.set_ion_meta_fd(buffer.ion_meta_fd);
    buffer_info.set_img_id(buffer.img_id);
    buffer_info.set_size(buffer.size);
    buffer_info.set_timestamp(buffer.timestamp);
    buffer_info.set_seqnum(buffer.seqnum);
    buffer_info.set_buffer_id(buffer.buffer_id);
    buffer_info.set_flags(buffer.flags);
    buffer_info.set_capacity(buffer.capacity);

    *nvt->mutable_buffers()->Add() = buffer_info;
  }
  for (auto &&b_meta : metas) {
    BufferMetaMsg meta;
    meta.set_format(static_cast<BufferFormatMsg>(b_meta.format));
    meta.set_n_planes(b_meta.n_planes);
    for (auto &&p_info : b_meta.planes) {
      PlaneInfoMsg info;
      info.set_stride(p_info.stride);
      info.set_scanline(p_info.scanline);
      info.set_width(p_info.width);
      info.set_height(p_info.height);
      info.set_offset(p_info.offset);
      info.set_size(p_info.size);

      *meta.mutable_info()->Add() = info;
    }
    *nvt->mutable_metas()->Add() = meta;
  }
  auto size = async_msg.ByteSizeLong();
  void *buffer = malloc(size);
  async_msg.SerializeToArray(buffer, size);
  ssize_t bytesSent = send(callback_socket_, buffer, size, 0);
  QMMF_VERBOSE("%s bytesSent: %u", __func__, bytesSent);
  if (bytesSent == -1) {
    QMMF_ERROR("%s: Closing callback socket: %d", __func__, callback_socket_);
    close (callback_socket_);
  }

  free (buffer);
  QMMF_VERBOSE("%s: Exit client_id(%u), session_id(%u), track_id(%u)",
      __func__, client_id_, session_id, track_id);
}

void RecorderServiceCallbackProxy::NotifyVideoTrackEvent(uint32_t session_id, uint32_t track_id,
                            EventType event_type,
                            void *event_data, size_t event_data_size) {

}

void RecorderServiceCallbackProxy::NotifyCameraResult(uint32_t camera_id, const CameraMetadata &result) {

}

void RecorderServiceCallbackProxy::NotifyDeleteVideoTrack(uint32_t track_id) {
}
#endif // !HAVE_BINDER

}; //namespace recorder

}; //namespace qmmf
