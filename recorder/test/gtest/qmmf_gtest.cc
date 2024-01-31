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
* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#define LOG_TAG "VideoGTest"

#include "recorder/test/gtest/qmmf_gtest.h"

using namespace qcamera;

/*
* ConnectToService: This test case will test Connect/Disconnect API.
* API test sequence:
*   loop Start {
*   ------------------
*  - Connect
*  - Disconnect
*   ------------------
*   } loop End
*/
TEST_F(VideoGtest, ConnectToService) {
  std::cout << "\n---------- Run Test ----------"
            << test_info_->test_case_name() << "." << test_info_->name()
            << std::endl;

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    std::cout
        << "###############################################################"
        << std::endl;

    std::cout << "Ruunnig Test Iteration: " << i << "/" << iteration_count_
              << std::endl;

    TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
              test_info_->name(), i);

    auto ret = recorder_.Connect(recorder_status_cb_);
    ASSERT_TRUE(ret == 0);

    sleep(3);

    ret = recorder_.Disconnect();
    ASSERT_TRUE(ret == 0);
  }
  std::cout << "---------- Test Completed ----------\n"
            << test_info_->test_case_name() << "." << test_info_->name();
}

/*
* StartStopCamera: This test case will test Start/Stop Camera API.
* API test sequence:
*   loop Start {
*   ------------------
*  - StartCamera
*  - StopCamera
*   ------------------
*   } loop End
*/
TEST_F(VideoGtest, StartStopCamera) {
  std::cout << "\n---------- Run Test ----------"
            << test_info_->test_case_name() << "." << test_info_->name()
            << std::endl;

  auto ret = Init();
  ASSERT_TRUE(ret == 0);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    std::cout
        << "###############################################################"
        << std::endl;

    std::cout << "Ruunnig Test Iteration: " << i << "/" << iteration_count_
              << std::endl;

    TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
              test_info_->name(), i);

    ret = recorder_.StartCamera(camera_id_, 30);
    ASSERT_TRUE(ret == 0);

    sleep(3);

    ret = recorder_.StopCamera(camera_id_);
    ASSERT_TRUE(ret == 0);
  }
  ret = DeInit();
  ASSERT_TRUE(ret == 0);

  std::cout << "---------- Test Completed ----------\n"
            << test_info_->test_case_name() << "." << test_info_->name();
}

/*
* SessionWithSingleStream:
*   This test will test Single stream of a configurable
*   resolution and format.
*   If EIS and SHDR is enabled, then they are also applied.
*   If Snapshot stream is on, Snapshot will also be taken.
* API test sequence:
*  - StartCamera [Check for EIS, SHDR]
*  - CreateSession
*  - CreateVideoTrack
*  - StartSession
*  - Check for SnapShot stream
*  - If Snapshot Stream  is on
*  - { ConfigImageCapture
*  -   CaptureImage
*  -   CancelCaptureImage }
*  - StopSession
*  - DeleteVideoTrack
*  - DeleteSession
*  - StopCamera
*/

TEST_F(VideoGtest, SessionWithSingleStream) {
  std::cout << "\n---------- Run Test ----------" <<
      test_info_->test_case_name() << "." << test_info_->name()<< std::endl;

  auto ret = Init();
  ASSERT_TRUE(ret == 0);

  // Extract Parameter of First Video Stream.
  uint32_t video_track_1 = kFirstStreamID;
  auto stream = stream_info_map_[video_track_1];
  uint32_t width = stream.width;
  uint32_t height = stream.height;
  VideoFormat format = stream.format;
  float fps = stream.fps;

  PrintStreamInfo(kFirstStreamID);

  CameraExtraParam camera_xtraparam;

  SetCameraExtraParam(camera_xtraparam);

  ret = recorder_.StartCamera(camera_id_, camera_fps_, camera_xtraparam);
  ASSERT_TRUE(ret == 0);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    std::cout
        << "###############################################################"
        << std::endl;

    std::cout << "Ruunnig Test Iteration: " << i << "/" << iteration_count_
              << std::endl;

    TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
              test_info_->name(), i);

    SessionCb session_status_cb = CreateSessionStatusCb();
    uint32_t session_id;

    ret = recorder_.CreateSession(session_status_cb, &session_id);
    ASSERT_TRUE(session_id > 0);
    ASSERT_TRUE(ret == 0);

    // Configure Single Video Stream
    VideoTrackParam video_track_param {
      camera_id_, width, height, fps, format
    };

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, metas);
    };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    VideoExtraParam xtraparam;
    ret = recorder_.CreateVideoTrack(session_id, video_track_1,
                                     video_track_param, xtraparam,
                                     video_track_cb);
    ASSERT_TRUE(ret == 0);

    std::vector<uint32_t> track_ids;
    track_ids.push_back(video_track_1);
    sessions_.insert(std::make_pair(session_id, track_ids));

    // Configure Snapshot Stream.
    if (is_snap_stream_on_) {
      ConfigureImageParam(kFirstImageID);
    }

    // Start Session
    ret = recorder_.StartSession(session_id);
    ASSERT_TRUE(ret == 0);

    sleep(record_duration_);

    // Now Take Snapshot.
    if (is_snap_stream_on_) {
      TakeSnapshot();
    }

    ret = recorder_.StopSession(session_id, false);
    ASSERT_TRUE(ret == 0);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_1);
    ASSERT_TRUE(ret == 0);

    ret = recorder_.DeleteSession(session_id);
    ASSERT_TRUE(ret == 0);

    ClearSessions();
  }

  ret = recorder_.StopCamera(camera_id_);
  ASSERT_TRUE(ret == 0);

  ret = DeInit();
  ASSERT_TRUE(ret == 0);

  std::cout <<"---------- Test Completed ----------\n" <<
      test_info_->test_case_name() << "." << test_info_->name();
}

/*
* SessionWithTwoStream:
*   This test will test dual stream of a configurable
*   resolution and format.
*   If EIS and SHDR is enabled, then they are also applied.
*   If Snapshot stream is on, Snapshot will also be taken.
* API test sequence:
*  - StartCamera [Check for EIS, SHDR]
*  - CreateSession
*  - CreateVideoTrack for 2 tracks. [Check for linked Stream]
*  - StartSession
*  - Check for SnapShot stream
*  - If Snapshot Stream  is on
*  - { ConfigImageCapture
*  -   CaptureImage
*  -   CancelCaptureImage }
*  - StopSession
*  - DeleteVideoTrack
*  - DeleteSession
*  - StopCamera
*/

TEST_F(VideoGtest, SessionWithTwoStream) {
  std::cout << "\n---------- Run Test ----------" <<
      test_info_->test_case_name() << "." << test_info_->name()<< std::endl;

  auto ret = Init();
  ASSERT_TRUE(ret == 0);

  // Extract Parameter of First Video Stream.
  uint32_t video_track_1 = kFirstStreamID;
  auto stream_1 = stream_info_map_[video_track_1];
  uint32_t stream_1_width = stream_1.width;
  uint32_t stream_1_height = stream_1.height;
  VideoFormat stream_1_format = stream_1.format;
  float stream_1_fps = stream_1.fps;

  // Extract Parameter of Second Video Stream.
  uint32_t video_track_2 = kSecondStreamID;
  auto stream_2 = stream_info_map_[video_track_2];
  uint32_t stream_2_width = stream_2.width;
  uint32_t stream_2_height = stream_2.height;
  VideoFormat stream_2_format = stream_2.format;
  float stream_2_fps = stream_2.fps;
  uint32_t stream_2_src_id = stream_2.source_stream_id;

  PrintStreamInfo(kSecondStreamID);

  CameraExtraParam camera_xtraparam;

  SetCameraExtraParam(camera_xtraparam);

  ret = recorder_.StartCamera(camera_id_, camera_fps_, camera_xtraparam);
  ASSERT_TRUE(ret == 0);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    std::cout
        << "###############################################################"
        << std::endl;

    std::cout << "Ruunnig Test Iteration: " << i << "/" << iteration_count_
              << std::endl;

    TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
              test_info_->name(), i);

    SessionCb session_status_cb = CreateSessionStatusCb();
    uint32_t session_id;

    ret = recorder_.CreateSession(session_status_cb, &session_id);
    ASSERT_TRUE(session_id > 0);
    ASSERT_TRUE(ret == 0);

    // First Track Configuration
    VideoTrackParam video_track_param_1 {
      camera_id_, stream_1_width, stream_1_height, stream_1_fps, stream_1_format
    };

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, metas);
    };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    VideoExtraParam xtraparam;
    ret = recorder_.CreateVideoTrack(session_id, video_track_1,
                                     video_track_param_1, xtraparam,
                                     video_track_cb);
    ASSERT_TRUE(ret == 0);

    std::vector<uint32_t> track_ids;
    track_ids.push_back(video_track_1);

    // Second  Track Configuration
    VideoTrackParam video_track_param_2 {
      camera_id_, stream_2_width, stream_2_height, stream_2_fps, stream_2_format
    };

    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, metas);
    };

    if (stream_2_src_id != 0) {
      SourceVideoTrack surface_video_copy;
      surface_video_copy.source_track_id = stream_2_src_id;
      xtraparam.Update(QMMF_SOURCE_VIDEO_TRACK_ID, surface_video_copy);
    }

    ret = recorder_.CreateVideoTrack(session_id, video_track_2,
                                      video_track_param_2, xtraparam,
                                      video_track_cb);
    ASSERT_TRUE(ret == 0);

    track_ids.push_back(video_track_2);

    sessions_.insert(std::make_pair(session_id, track_ids));

    // Configure Snapshot Stream.
    if (is_snap_stream_on_) {
      ConfigureImageParam(kSecondImageID);
    }

    // Start Session
    ret = recorder_.StartSession(session_id);
    ASSERT_TRUE(ret == 0);

    sleep(record_duration_);

    // Now Take Snapshot.
    if (is_snap_stream_on_) {
      TakeSnapshot();
    }

    ret = recorder_.StopSession(session_id, false);
    ASSERT_TRUE(ret == 0);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_2);
    ASSERT_TRUE(ret == 0);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_1);
    ASSERT_TRUE(ret == 0);

    ret = recorder_.DeleteSession(session_id);
    ASSERT_TRUE(ret == 0);

    ClearSessions();
  }

  ret = recorder_.StopCamera(camera_id_);
  ASSERT_TRUE(ret == 0);

  ret = DeInit();
  ASSERT_TRUE(ret == 0);

  std::cout <<"---------- Test Completed ----------\n" <<
      test_info_->test_case_name() << "." << test_info_->name();
}

/*
* SessionWithThreeStream:
*   This test will test 3 stream of a configurable
*   resolution and format.
*   If EIS and SHDR is enabled, then they are also applied.
*   If Snapshot stream is on, Snapshot will also be taken.
* API test sequence:
*  - StartCamera [Check for EIS, SHDR]
*  - CreateSession
*  - CreateVideoTrack for 3 tracks. [Check for linked Stream]
*  - StartSession
*  - Check for SnapShot stream
*  - If Snapshot Stream  is on
*  - { ConfigImageCapture
*  -   CaptureImage
*  -   CancelCaptureImage }
*  - StopSession
*  - DeleteVideoTrack
*  - DeleteSession
*  - StopCamera
*/

TEST_F(VideoGtest, SessionWithThreeStream) {
  std::cout << "\n---------- Run Test ----------" <<
      test_info_->test_case_name() << "." << test_info_->name()<< std::endl;

  auto ret = Init();
  ASSERT_TRUE(ret == 0);

  // Extract Parameter of First Video Stream.
  uint32_t video_track_1 = kFirstStreamID;
  auto stream_1 = stream_info_map_[video_track_1];
  uint32_t stream_1_width = stream_1.width;
  uint32_t stream_1_height = stream_1.height;
  VideoFormat stream_1_format = stream_1.format;
  float stream_1_fps = stream_1.fps;

  // Extract Parameter of Second Video Stream.
  uint32_t video_track_2 = kSecondStreamID;
  auto stream_2 = stream_info_map_[video_track_2];
  uint32_t stream_2_width = stream_2.width;
  uint32_t stream_2_height = stream_2.height;
  VideoFormat stream_2_format = stream_2.format;
  float stream_2_fps = stream_2.fps;
  uint32_t stream_2_src_id = stream_2.source_stream_id;

  // Extract Parameter of Third Video Stream.
  uint32_t video_track_3 = kThirdStreamID;
  auto stream_3 = stream_info_map_[video_track_3];
  uint32_t stream_3_width = stream_3.width;
  uint32_t stream_3_height = stream_3.height;
  VideoFormat stream_3_format = stream_3.format;
  float stream_3_fps = stream_3.fps;
  uint32_t stream_3_src_id = stream_3.source_stream_id;

  PrintStreamInfo(kThirdStreamID);

  CameraExtraParam camera_xtraparam;

  SetCameraExtraParam(camera_xtraparam);

  ret = recorder_.StartCamera(camera_id_, camera_fps_, camera_xtraparam);
  ASSERT_TRUE(ret == 0);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    std::cout
        << "###############################################################"
        << std::endl;

    std::cout << "Ruunnig Test Iteration: " << i << "/" << iteration_count_
              << std::endl;

    TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
              test_info_->name(), i);

    SessionCb session_status_cb = CreateSessionStatusCb();
    uint32_t session_id;

    ret = recorder_.CreateSession(session_status_cb, &session_id);
    ASSERT_TRUE(session_id > 0);
    ASSERT_TRUE(ret == 0);

    // First Track Configuration
    VideoTrackParam video_track_param_1 {
      camera_id_, stream_1_width, stream_1_height, stream_1_fps, stream_1_format
    };

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, metas);
    };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    VideoExtraParam xtraparam;
    ret = recorder_.CreateVideoTrack(session_id, video_track_1,
                                     video_track_param_1, xtraparam,
                                     video_track_cb);
    ASSERT_TRUE(ret == 0);

    std::vector<uint32_t> track_ids;
    track_ids.push_back(video_track_1);

    // Second  Track Configuration
    VideoTrackParam video_track_param_2 {
      camera_id_, stream_2_width, stream_2_height, stream_2_fps, stream_2_format
    };
    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, metas);
    };

    if (stream_2_src_id != 0) {
      SourceVideoTrack surface_video_copy;
      surface_video_copy.source_track_id = stream_2_src_id;
      xtraparam.Update(QMMF_SOURCE_VIDEO_TRACK_ID, surface_video_copy);
    }

    ret = recorder_.CreateVideoTrack(session_id, video_track_2,
                                      video_track_param_2, xtraparam,
                                      video_track_cb);
    ASSERT_TRUE(ret == 0);

    track_ids.push_back(video_track_2);

    // Third  Track Configuration
    VideoTrackParam video_track_param_3 {
      camera_id_, stream_3_width, stream_3_height, stream_3_fps, stream_3_format
    };
    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, metas);
    };

    if (stream_3_src_id != 0) {
      SourceVideoTrack surface_video_copy;
      surface_video_copy.source_track_id = stream_3_src_id;
      xtraparam.Update(QMMF_SOURCE_VIDEO_TRACK_ID, surface_video_copy);
    }

    ret = recorder_.CreateVideoTrack(session_id, video_track_3,
                                      video_track_param_3, xtraparam,
                                      video_track_cb);
    ASSERT_TRUE(ret == 0);

    track_ids.push_back(video_track_3);

    sessions_.insert(std::make_pair(session_id, track_ids));

    // Configure Snapshot Stream.
    if (is_snap_stream_on_) {
      ConfigureImageParam(kFirstImageID);
    }

    // Start Session
    ret = recorder_.StartSession(session_id);
    ASSERT_TRUE(ret == 0);

    sleep(record_duration_);

    // Now Take Snapshot.
    if (is_snap_stream_on_) {
      TakeSnapshot();
    }

    ret = recorder_.StopSession(session_id, false);
    ASSERT_TRUE(ret == 0);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_3);
    ASSERT_TRUE(ret == 0);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_2);
    ASSERT_TRUE(ret == 0);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_1);
    ASSERT_TRUE(ret == 0);

    ret = recorder_.DeleteSession(session_id);
    ASSERT_TRUE(ret == 0);

    ClearSessions();
  }

  ret = recorder_.StopCamera(camera_id_);
  ASSERT_TRUE(ret == 0);

  ret = DeInit();
  ASSERT_TRUE(ret == 0);

  std::cout <<"---------- Test Completed ----------\n" <<
      test_info_->test_case_name() << "." << test_info_->name();
}

/*
* SessionWithFourStream:
*   This test will test 4 stream of a configurable
*   resolution and format.
*   If EIS and SHDR is enabled, then they are also applied.
*   If Snapshot stream is on, Snapshot will also be taken.
* API test sequence:
*  - StartCamera [Check for EIS, SHDR]
*  - CreateSession
*  - CreateVideoTrack for 4 tracks. [Check for linked Stream]
*  - StartSession
*  - Check for SnapShot stream
*  - If Snapshot Stream  is on
*  - { ConfigImageCapture
*  -   CaptureImage
*  -   CancelCaptureImage }
*  - StopSession
*  - DeleteVideoTrack
*  - DeleteSession
*  - StopCamera
*/

TEST_F(VideoGtest, SessionWithFourStream) {
  std::cout << "\n---------- Run Test ----------" <<
      test_info_->test_case_name() << "." << test_info_->name()<< std::endl;

  auto ret = Init();
  ASSERT_TRUE(ret == 0);

  // Extract Parameter of First Video Stream.
  uint32_t video_track_1 = kFirstStreamID;
  auto stream_1 = stream_info_map_[video_track_1];
  uint32_t stream_1_width = stream_1.width;
  uint32_t stream_1_height = stream_1.height;
  VideoFormat stream_1_format = stream_1.format;
  float stream_1_fps = stream_1.fps;

  // Extract Parameter of Second Video Stream.
  uint32_t video_track_2 = kSecondStreamID;
  auto stream_2 = stream_info_map_[video_track_2];
  uint32_t stream_2_width = stream_2.width;
  uint32_t stream_2_height = stream_2.height;
  VideoFormat stream_2_format = stream_2.format;
  float stream_2_fps = stream_2.fps;
  uint32_t stream_2_src_id = stream_2.source_stream_id;

  // Extract Parameter of Third Video Stream.
  uint32_t video_track_3 = kThirdStreamID;
  auto stream_3 = stream_info_map_[video_track_3];
  uint32_t stream_3_width = stream_3.width;
  uint32_t stream_3_height = stream_3.height;
  VideoFormat stream_3_format = stream_3.format;
  float stream_3_fps = stream_3.fps;
  uint32_t stream_3_src_id = stream_3.source_stream_id;

  // Extract Parameter of Fourth Video Stream.
  uint32_t video_track_4 = kFourthStreamID;
  auto stream_4 = stream_info_map_[video_track_4];
  uint32_t stream_4_width = stream_4.width;
  uint32_t stream_4_height = stream_4.height;
  VideoFormat stream_4_format = stream_4.format;
  float stream_4_fps = stream_4.fps;
  uint32_t stream_4_src_id = stream_4.source_stream_id;

  PrintStreamInfo(kFourthStreamID);

  CameraExtraParam camera_xtraparam;

  SetCameraExtraParam(camera_xtraparam);

  ret = recorder_.StartCamera(camera_id_, camera_fps_, camera_xtraparam);
  ASSERT_TRUE(ret == 0);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    std::cout
        << "###############################################################"
        << std::endl;

    std::cout << "Ruunnig Test Iteration: " << i << "/" << iteration_count_
              << std::endl;

    TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
              test_info_->name(), i);

    SessionCb session_status_cb = CreateSessionStatusCb();
    uint32_t session_id;

    ret = recorder_.CreateSession(session_status_cb, &session_id);
    ASSERT_TRUE(session_id > 0);
    ASSERT_TRUE(ret == 0);

    // First Track Configuration
    VideoTrackParam video_track_param_1 {
      camera_id_, stream_1_width, stream_1_height, stream_1_fps, stream_1_format
    };

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, metas);
    };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    VideoExtraParam xtraparam;
    ret = recorder_.CreateVideoTrack(session_id, video_track_1,
                                     video_track_param_1, xtraparam,
                                     video_track_cb);
    ASSERT_TRUE(ret == 0);

    std::vector<uint32_t> track_ids;
    track_ids.push_back(video_track_1);

    // Second  Track Configuration
    VideoTrackParam video_track_param_2 {
      camera_id_, stream_2_width, stream_2_height, stream_2_fps, stream_2_format
    };
    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, metas);
    };

    if (stream_2_src_id != 0) {
      SourceVideoTrack surface_video_copy;
      surface_video_copy.source_track_id = stream_2_src_id;
      xtraparam.Update(QMMF_SOURCE_VIDEO_TRACK_ID, surface_video_copy);
    }

    ret = recorder_.CreateVideoTrack(session_id, video_track_2,
                                      video_track_param_2, xtraparam,
                                      video_track_cb);
    ASSERT_TRUE(ret == 0);

    track_ids.push_back(video_track_2);

    // Third  Track Configuration
    VideoTrackParam video_track_param_3 {
      camera_id_, stream_3_width, stream_3_height, stream_3_fps, stream_3_format
    };
    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, metas);
    };

    if (stream_3_src_id != 0) {
      SourceVideoTrack surface_video_copy;
      surface_video_copy.source_track_id = stream_3_src_id;
      xtraparam.Update(QMMF_SOURCE_VIDEO_TRACK_ID, surface_video_copy);
    }

    ret = recorder_.CreateVideoTrack(session_id, video_track_3,
                                      video_track_param_3, xtraparam,
                                      video_track_cb);
    ASSERT_TRUE(ret == 0);

    track_ids.push_back(video_track_3);

    // Fourth  Track Configuration
    VideoTrackParam video_track_param_4 {
      camera_id_, stream_4_width, stream_4_height, stream_4_fps, stream_4_format
    };
    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, metas);
    };

    if (stream_4_src_id != 0) {
      SourceVideoTrack surface_video_copy;
      surface_video_copy.source_track_id = stream_4_src_id;
      xtraparam.Update(QMMF_SOURCE_VIDEO_TRACK_ID, surface_video_copy);
    }

    ret = recorder_.CreateVideoTrack(session_id, video_track_4,
                                      video_track_param_4, xtraparam,
                                      video_track_cb);
    ASSERT_TRUE(ret == 0);

    track_ids.push_back(video_track_4);

    sessions_.insert(std::make_pair(session_id, track_ids));

    // Configure Snapshot Stream.
    if (is_snap_stream_on_) {
      ConfigureImageParam(kFirstImageID);
    }

    // Start Session
    ret = recorder_.StartSession(session_id);
    ASSERT_TRUE(ret == 0);

    sleep(record_duration_);

    // Now Take Snapshot.
    if (is_snap_stream_on_) {
      TakeSnapshot();
    }

    ret = recorder_.StopSession(session_id, false);
    ASSERT_TRUE(ret == 0);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_4);
    ASSERT_TRUE(ret == 0);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_3);
    ASSERT_TRUE(ret == 0);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_2);
    ASSERT_TRUE(ret == 0);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_1);
    ASSERT_TRUE(ret == 0);

    ret = recorder_.DeleteSession(session_id);
    ASSERT_TRUE(ret == 0);

    ClearSessions();
  }

  ret = recorder_.StopCamera(camera_id_);
  ASSERT_TRUE(ret == 0);

  ret = DeInit();
  ASSERT_TRUE(ret == 0);

  std::cout <<"---------- Test Completed ----------\n" <<
      test_info_->test_case_name() << "." << test_info_->name();
}

/*
* SessionWithFiveStream:
*   This test will test 5 stream of a configurable
*   resolution and format.
*   If EIS and SHDR is enabled, then they are also applied.
*   If Snapshot stream is on, Snapshot will also be taken.
* API test sequence:
*  - StartCamera [Check for EIS, SHDR]
*  - CreateSession
*  - CreateVideoTrack for 5 tracks. [Check for linked Stream]
*  - StartSession
*  - Check for SnapShot stream
*  - If Snapshot Stream  is on
*  - { ConfigImageCapture
*  -   CaptureImage
*  -   CancelCaptureImage }
*  - StopSession
*  - DeleteVideoTrack
*  - DeleteSession
*  - StopCamera
*/

TEST_F(VideoGtest, SessionWithFiveStream) {
  std::cout << "\n---------- Run Test ----------" <<
      test_info_->test_case_name() << "." << test_info_->name()<< std::endl;

  auto ret = Init();
  ASSERT_TRUE(ret == 0);

  // Extract Parameter of First Video Stream.
  uint32_t video_track_1 = kFirstStreamID;
  auto stream_1 = stream_info_map_[video_track_1];
  uint32_t stream_1_width = stream_1.width;
  uint32_t stream_1_height = stream_1.height;
  VideoFormat stream_1_format = stream_1.format;
  float stream_1_fps = stream_1.fps;

  // Extract Parameter of Second Video Stream.
  uint32_t video_track_2 = kSecondStreamID;
  auto stream_2 = stream_info_map_[video_track_2];
  uint32_t stream_2_width = stream_2.width;
  uint32_t stream_2_height = stream_2.height;
  VideoFormat stream_2_format = stream_2.format;
  float stream_2_fps = stream_2.fps;
  uint32_t stream_2_src_id = stream_2.source_stream_id;

  // Extract Parameter of Third Video Stream.
  uint32_t video_track_3 = kThirdStreamID;
  auto stream_3 = stream_info_map_[video_track_3];
  uint32_t stream_3_width = stream_3.width;
  uint32_t stream_3_height = stream_3.height;
  VideoFormat stream_3_format = stream_3.format;
  float stream_3_fps = stream_3.fps;
  uint32_t stream_3_src_id = stream_3.source_stream_id;

  // Extract Parameter of Fourth Video Stream.
  uint32_t video_track_4 = kFourthStreamID;
  auto stream_4 = stream_info_map_[video_track_4];
  uint32_t stream_4_width = stream_4.width;
  uint32_t stream_4_height = stream_4.height;
  VideoFormat stream_4_format = stream_4.format;
  float stream_4_fps = stream_4.fps;
  uint32_t stream_4_src_id = stream_4.source_stream_id;

  // Extract Parameter of Fifth Video Stream.
  uint32_t video_track_5 = kFifthStreamID;
  auto stream_5 = stream_info_map_[video_track_5];
  uint32_t stream_5_width = stream_5.width;
  uint32_t stream_5_height = stream_5.height;
  VideoFormat stream_5_format = stream_5.format;
  float stream_5_fps = stream_5.fps;
  uint32_t stream_5_src_id = stream_5.source_stream_id;

  PrintStreamInfo(kFifthStreamID);

  CameraExtraParam camera_xtraparam;

  SetCameraExtraParam(camera_xtraparam);

  ret = recorder_.StartCamera(camera_id_, camera_fps_, camera_xtraparam);
  ASSERT_TRUE(ret == 0);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    std::cout
        << "###############################################################"
        << std::endl;

    std::cout << "Ruunnig Test Iteration: " << i << "/" << iteration_count_
              << std::endl;

    TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
              test_info_->name(), i);

    SessionCb session_status_cb = CreateSessionStatusCb();
    uint32_t session_id;

    ret = recorder_.CreateSession(session_status_cb, &session_id);
    ASSERT_TRUE(session_id > 0);
    ASSERT_TRUE(ret == 0);

    // First Track Configuration
    VideoTrackParam video_track_param_1 {
      camera_id_, stream_1_width, stream_1_height, stream_1_fps, stream_1_format
    };

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, metas);
    };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    VideoExtraParam xtraparam;
    ret = recorder_.CreateVideoTrack(session_id, video_track_1,
                                     video_track_param_1, xtraparam,
                                     video_track_cb);
    ASSERT_TRUE(ret == 0);

    std::vector<uint32_t> track_ids;
    track_ids.push_back(video_track_1);

    // Second  Track Configuration
    VideoTrackParam video_track_param_2 {
      camera_id_, stream_2_width, stream_2_height, stream_2_fps, stream_2_format
    };
    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, metas);
    };

    if (stream_2_src_id != 0) {
      SourceVideoTrack surface_video_copy;
      surface_video_copy.source_track_id = stream_2_src_id;
      xtraparam.Update(QMMF_SOURCE_VIDEO_TRACK_ID, surface_video_copy);
    }

    ret = recorder_.CreateVideoTrack(session_id, video_track_2,
                                      video_track_param_2, xtraparam,
                                      video_track_cb);
    ASSERT_TRUE(ret == 0);

    track_ids.push_back(video_track_2);

    // Third  Track Configuration
    VideoTrackParam video_track_param_3 {
      camera_id_, stream_3_width, stream_3_height, stream_3_fps, stream_3_format
    };
    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, metas);
    };

    if (stream_3_src_id != 0) {
      SourceVideoTrack surface_video_copy;
      surface_video_copy.source_track_id = stream_3_src_id;
      xtraparam.Update(QMMF_SOURCE_VIDEO_TRACK_ID, surface_video_copy);
    }

    ret = recorder_.CreateVideoTrack(session_id, video_track_3,
                                      video_track_param_3, xtraparam,
                                      video_track_cb);
    ASSERT_TRUE(ret == 0);

    track_ids.push_back(video_track_3);

    // Fourth  Track Configuration
    VideoTrackParam video_track_param_4 {
      camera_id_, stream_4_width, stream_4_height, stream_4_fps, stream_4_format
    };
    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, metas);
    };

    if (stream_4_src_id != 0) {
      SourceVideoTrack surface_video_copy;
      surface_video_copy.source_track_id = stream_4_src_id;
      xtraparam.Update(QMMF_SOURCE_VIDEO_TRACK_ID, surface_video_copy);
    }

    ret = recorder_.CreateVideoTrack(session_id, video_track_4,
                                      video_track_param_4, xtraparam,
                                      video_track_cb);
    ASSERT_TRUE(ret == 0);

    track_ids.push_back(video_track_4);

    // Fifth  Track Configuration
    VideoTrackParam video_track_param_5 {
      camera_id_, stream_5_width, stream_5_height, stream_5_fps, stream_5_format
    };
    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, metas);
    };

    if (stream_5_src_id != 0) {
      SourceVideoTrack surface_video_copy;
      surface_video_copy.source_track_id = stream_5_src_id;
      xtraparam.Update(QMMF_SOURCE_VIDEO_TRACK_ID, surface_video_copy);
    }

    ret = recorder_.CreateVideoTrack(session_id, video_track_5,
                                      video_track_param_5, xtraparam,
                                      video_track_cb);
    ASSERT_TRUE(ret == 0);

    track_ids.push_back(video_track_5);

    sessions_.insert(std::make_pair(session_id, track_ids));

    // Configure Snapshot Stream.
    if (is_snap_stream_on_) {
      ConfigureImageParam(kFirstImageID);
    }

    // Start Session
    ret = recorder_.StartSession(session_id);
    ASSERT_TRUE(ret == 0);

    sleep(record_duration_);

    // Now Take Snapshot.
    if (is_snap_stream_on_) {
      TakeSnapshot();
    }

    ret = recorder_.StopSession(session_id, false);
    ASSERT_TRUE(ret == 0);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_5);
    ASSERT_TRUE(ret == 0);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_4);
    ASSERT_TRUE(ret == 0);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_3);
    ASSERT_TRUE(ret == 0);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_2);
    ASSERT_TRUE(ret == 0);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_1);
    ASSERT_TRUE(ret == 0);

    ret = recorder_.DeleteSession(session_id);
    ASSERT_TRUE(ret == 0);

    ClearSessions();
  }

  ret = recorder_.StopCamera(camera_id_);
  ASSERT_TRUE(ret == 0);

  ret = DeInit();
  ASSERT_TRUE(ret == 0);

  std::cout <<"---------- Test Completed ----------\n" <<
      test_info_->test_case_name() << "." << test_info_->name();
}

/*
* SessionWithTwoConcurrentCam1080p: This test will test two single
*                                   Cameras, each giving one 1080p stream.
*
* Api test sequence summary:
*  - StartCamera-Cam0
*  - StartCamera-Cam1
*  - CreateSession-Cam0
*  - CreateSession-Cam1
*  - Create1080pTrack-Cam0
*  - Create1080pTrack-Cam1
*  - StartSession-Cam0
*  - StartSession-Cam1
*  - StopSession-Cam0
*  - StopSession-Cam1
*  - DeleteVideoTracks-Cam0
*  - DeleteVideoTracks-Cam1
*  - DeleteSession-Cam0
*  - DeleteSession-Cam1
*  - StopCamera-Cam0
*  - StopCamera-Cam1
*/
TEST_F(VideoGtest, SessionWithTwoConcurrentCam1080p) {
  std::cout << "\n---------- Run Test ----------" <<
      test_info_->test_case_name() << "." << test_info_->name()<< std::endl;

  auto ret = Init();
  ASSERT_TRUE(ret == 0);

  uint32_t cam0_id = 0;
  uint32_t cam1_id = 1;

  uint32_t track_width = 1920;
  uint32_t track_height = 1080;

  uint32_t cam0_video_track_id_1080p = 1;
  uint32_t cam1_video_track_id_1080p = 2;

  ret = recorder_.StartCamera(cam0_id, 30);
  ASSERT_TRUE(ret == 0);

  ret = recorder_.StartCamera(cam1_id, 30);
  ASSERT_TRUE(ret == 0);

  SessionCb cam0_session_status_cb;
  cam0_session_status_cb.event_cb = [this](
      EventType event_type, void *event_data, size_t event_data_size) -> void {
    SessionCallbackHandler(event_type, event_data, event_data_size);
  };

  uint32_t cam0_session_id;
  ret = recorder_.CreateSession(cam0_session_status_cb, &cam0_session_id);
  ASSERT_TRUE(cam0_session_id > 0);
  ASSERT_TRUE(ret == 0);

  SessionCb cam1_session_status_cb;
  cam1_session_status_cb.event_cb = [this](
      EventType event_type, void *event_data, size_t event_data_size) -> void {
    SessionCallbackHandler(event_type, event_data, event_data_size);
  };

  uint32_t cam1_session_id;
  ret = recorder_.CreateSession(cam1_session_status_cb, &cam1_session_id);
  ASSERT_TRUE(cam1_session_id > 0);
  ASSERT_TRUE(ret == 0);

  TrackCb video_track_cb;
  video_track_cb.data_cb = [&, cam0_session_id](
      uint32_t track_id, std::vector<BufferDescriptor> buffers,
      std::vector<BufferMeta> metas) {
    VideoTrackYUVDataCb(cam0_session_id, track_id, buffers, metas);
  };

  video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                void *event_data, size_t event_data_size) {
    VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
  };

  VideoTrackParam video_track_param {
      cam0_id, track_width, track_height, 30, VideoFormat::kNV12
  };
  VideoExtraParam xtraparam;

  ret = recorder_.CreateVideoTrack(cam0_session_id, cam0_video_track_id_1080p,
                                   video_track_param, xtraparam,
                                   video_track_cb);
  ASSERT_TRUE(ret == 0);

  std::vector<uint32_t> cam0_track_ids;
  cam0_track_ids.push_back(cam0_video_track_id_1080p);
  sessions_.insert(std::make_pair(cam0_session_id, cam0_track_ids));

  video_track_cb.data_cb = [&, cam1_session_id](
      uint32_t track_id, std::vector<BufferDescriptor> buffers,
      std::vector<BufferMeta> metas) {
    VideoTrackYUVDataCb(cam1_session_id, track_id, buffers, metas);
  };

  video_track_param.camera_id = cam1_id;
  ret = recorder_.CreateVideoTrack(cam1_session_id, cam1_video_track_id_1080p,
                                   video_track_param, xtraparam,
                                   video_track_cb);
  ASSERT_TRUE(ret == 0);

  std::vector<uint32_t> cam1_track_ids;
  cam1_track_ids.push_back(cam1_video_track_id_1080p);
  sessions_.insert(std::make_pair(cam1_session_id, cam1_track_ids));

  ret = recorder_.StartSession(cam0_session_id);
  ASSERT_TRUE(ret == 0);

  ret = recorder_.StartSession(cam1_session_id);
  ASSERT_TRUE(ret == 0);

  // Let session run for time record_duration_, during this time buffer with
  // valid data would be received in track callback (VideoTrackYUVDataCb).
  sleep(record_duration_);

  ret = recorder_.StopSession(cam0_session_id, false);
  ASSERT_TRUE(ret == 0);

  ret = recorder_.StopSession(cam1_session_id, false);
  ASSERT_TRUE(ret == 0);

  ret = recorder_.DeleteVideoTrack(cam0_session_id,
                                   cam0_video_track_id_1080p);
  ASSERT_TRUE(ret == 0);

  ret = recorder_.DeleteVideoTrack(cam1_session_id,
                                   cam1_video_track_id_1080p);
  ASSERT_TRUE(ret == 0);

  ret = recorder_.DeleteSession(cam0_session_id);
  ASSERT_TRUE(ret == 0);

  ret = recorder_.DeleteSession(cam1_session_id);
  ASSERT_TRUE(ret == 0);

  ClearSessions();

  ret = recorder_.StopCamera(cam0_id);
  ASSERT_TRUE(ret == 0);

  ret = recorder_.StopCamera(cam1_id);
  ASSERT_TRUE(ret == 0);

  ret = DeInit();
  ASSERT_TRUE(ret == 0);

  std::cout <<"---------- Test Completed ----------\n" <<
      test_info_->test_case_name() << "." << test_info_->name();
}

/*
* SessionWithThreeConcurrentCam1080pAndRawStream: This test will test 3
*                                     single Cameras, each giving one 1080p
*                                     stream and third one will give max
*                                     Raw resolution.
*
* API test sequence summary:
*  - StartCamera-Cam0
*  - StartCamera-Cam1
*  - StartCamera-Cam2
*  - CreateSession-Cam0
*  - CreateSession-Cam1
*  - CreateSession-Cam2
*  - CreateVideoTrack-Cam0
*  - CreateVideoTrack-Cam1
*  - CreateVideoTrack-Cam2
*  - StartSession-Cam0
*  - StartSession-Cam1
*  - StartSession-Cam2
*  - StopSession-Cam0
*  - StopSession-Cam1
*  - StopSession-Cam2
*  - DeleteVideoTrack-Cam0
*  - DeleteVideoTrack-Cam1
*  - DeleteVideoTrack-Cam2
*  - DeleteSession-Cam0
*  - DeleteSession-Cam1
*  - DeleteSession-Cam2
*  - StopCamera-Cam0
*  - StopCamera-Cam1
*  - StopCamera-Cam2
*/
TEST_F(VideoGtest, SessionWithThreeConcurrentCam1080pAndRawStream) {
  std::cout << "\n---------- Run Test ----------" <<
      test_info_->test_case_name() << "." << test_info_->name()<< std::endl;

  auto ret = Init();
  ASSERT_TRUE(ret == 0);

  uint32_t cam0_id = 0;
  uint32_t cam1_id = 1;
  uint32_t cam2_id = 2;

  uint32_t track_width = 1920;
  uint32_t track_height = 1080;

  uint32_t cam0_video_track_id_1080p = 1;
  uint32_t cam1_video_track_id_1080p = 2;
  uint32_t cam2_video_track_raw = 3;

  ret = recorder_.StartCamera(cam0_id, 30);
  ASSERT_TRUE(ret == 0);

  ret = recorder_.StartCamera(cam1_id, 30);
  ASSERT_TRUE(ret == 0);

  ret = recorder_.StartCamera(cam2_id, 30);
  ASSERT_TRUE(ret == 0);

  SessionCb cam0_session_status_cb;
  cam0_session_status_cb.event_cb = [this](
      EventType event_type, void *event_data, size_t event_data_size) -> void {
    SessionCallbackHandler(event_type, event_data, event_data_size);
  };

  uint32_t cam0_session_id;
  ret = recorder_.CreateSession(cam0_session_status_cb, &cam0_session_id);
  ASSERT_TRUE(cam0_session_id > 0);
  ASSERT_TRUE(ret == 0);

  SessionCb cam1_session_status_cb;
  cam1_session_status_cb.event_cb = [this](
      EventType event_type, void *event_data, size_t event_data_size) -> void {
    SessionCallbackHandler(event_type, event_data, event_data_size);
  };

  uint32_t cam1_session_id;
  ret = recorder_.CreateSession(cam1_session_status_cb, &cam1_session_id);
  ASSERT_TRUE(cam1_session_id > 0);
  ASSERT_TRUE(ret == 0);

  SessionCb cam2_session_status_cb;
  cam2_session_status_cb.event_cb = [this](
      EventType event_type, void *event_data, size_t event_data_size) -> void {
    SessionCallbackHandler(event_type, event_data, event_data_size);
  };

  uint32_t cam2_session_id;
  ret = recorder_.CreateSession(cam2_session_status_cb, &cam2_session_id);
  ASSERT_TRUE(cam2_session_id > 0);
  ASSERT_TRUE(ret == 0);

  TrackCb video_track_cb;
  video_track_cb.data_cb = [&, cam0_session_id](
      uint32_t track_id, std::vector<BufferDescriptor> buffers,
      std::vector<BufferMeta> metas) {
    VideoTrackYUVDataCb(cam0_session_id, track_id, buffers, metas);
  };

  video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                void *event_data, size_t event_data_size) {
    VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
  };

  // First track
  VideoTrackParam video_track_param {
      cam0_id, track_width, track_height, 30, VideoFormat::kNV12
  };
  VideoExtraParam xtraparam;

  ret = recorder_.CreateVideoTrack(cam0_session_id, cam0_video_track_id_1080p,
                                   video_track_param, xtraparam,
                                   video_track_cb);
  ASSERT_TRUE(ret == 0);

  std::vector<uint32_t> cam0_track_ids;
  cam0_track_ids.push_back(cam0_video_track_id_1080p);
  sessions_.insert(std::make_pair(cam0_session_id, cam0_track_ids));

  video_track_cb.data_cb = [&, cam1_session_id](
      uint32_t track_id, std::vector<BufferDescriptor> buffers,
      std::vector<BufferMeta> metas) {
    VideoTrackYUVDataCb(cam1_session_id, track_id, buffers, metas);
  };
  // Second track
  video_track_param.camera_id = cam1_id;
  ret = recorder_.CreateVideoTrack(cam1_session_id, cam1_video_track_id_1080p,
                                   video_track_param, xtraparam,
                                   video_track_cb);
  ASSERT_TRUE(ret == 0);

  std::vector<uint32_t> cam1_track_ids;
  cam1_track_ids.push_back(cam1_video_track_id_1080p);
  sessions_.insert(std::make_pair(cam1_session_id, cam1_track_ids));

  // Starting session for first 2 camera
  ret = recorder_.StartSession(cam0_session_id);
  ASSERT_TRUE(ret == 0);

  ret = recorder_.StartSession(cam1_session_id);
  ASSERT_TRUE(ret == 0);

  uint32_t raw_width, raw_height;
  CameraMetadata static_meta;
  ret = recorder_.GetCameraCharacteristics(cam2_id, static_meta);
  ASSERT_TRUE(ret == 0);
  GtestCommon::GetMaxSupportedCameraRes(static_meta, raw_width, raw_height);

  // Third Track
  video_track_cb.data_cb = [&, cam2_session_id](
      uint32_t track_id, std::vector<BufferDescriptor> buffers,
      std::vector<BufferMeta> metas) {
    VideoTrackRawDataCb(cam2_session_id, track_id, buffers, metas);
  };

  VideoTrackParam video_track_param_raw{
      cam2_id, raw_width, raw_height, 30, VideoFormat::kBayerRDI10BIT
  };

  ret = recorder_.CreateVideoTrack(cam2_session_id, cam2_video_track_raw,
                                   video_track_param_raw, xtraparam,
                                   video_track_cb);
  ASSERT_TRUE(ret == 0);

  std::vector<uint32_t> cam2_track_ids;
  cam2_track_ids.push_back(cam2_video_track_raw);
  sessions_.insert(std::make_pair(cam2_session_id, cam2_track_ids));

  ret = recorder_.StartSession(cam2_session_id);
  ASSERT_TRUE(ret == 0);

  sleep(record_duration_);

  ret = recorder_.StopSession(cam0_session_id, false);
  ASSERT_TRUE(ret == 0);

  ret = recorder_.StopSession(cam1_session_id, false);
  ASSERT_TRUE(ret == 0);

  ret = recorder_.StopSession(cam2_session_id, false);
  ASSERT_TRUE(ret == 0);

  ret = recorder_.DeleteVideoTrack(cam0_session_id,
                                   cam0_video_track_id_1080p);
  ASSERT_TRUE(ret == 0);

  ret = recorder_.DeleteVideoTrack(cam1_session_id,
                                   cam1_video_track_id_1080p);
  ASSERT_TRUE(ret == 0);

  ret = recorder_.DeleteVideoTrack(cam2_session_id,
                                   cam2_video_track_raw);
  ASSERT_TRUE(ret == 0);

  ret = recorder_.DeleteSession(cam0_session_id);
  ASSERT_TRUE(ret == 0);

  ret = recorder_.DeleteSession(cam1_session_id);
  ASSERT_TRUE(ret == 0);

  ret = recorder_.DeleteSession(cam2_session_id);
  ASSERT_TRUE(ret == 0);

  ClearSessions();

  ret = recorder_.StopCamera(cam0_id);
  ASSERT_TRUE(ret == 0);

  ret = recorder_.StopCamera(cam1_id);
  ASSERT_TRUE(ret == 0);

  ret = recorder_.StopCamera(cam2_id);
  ASSERT_TRUE(ret == 0);

  ret = DeInit();
  ASSERT_TRUE(ret == 0);

  std::cout <<"---------- Test Completed ----------\n" <<
      test_info_->test_case_name() << "." << test_info_->name();
}

/*
* SessionWithSingleStreamSlavemode:
* This will single stream with slave mode.
* API test sequence:
*  - StartCamera With Slave Mode
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(VideoGtest, SessionWithSingleStreamSlavemode) {
  std::cout << "\n---------- Run Test ----------" <<
      test_info_->test_case_name() << "." << test_info_->name()<< std::endl;

  auto ret = Init();
  ASSERT_TRUE(ret == 0);

  // Extract Params of Video Stream
  uint32_t video_track_1 = 10;
  auto stream = stream_info_map_[kFirstStreamID];
  uint32_t width = stream.width;
  uint32_t height = stream.height;
  VideoFormat format = stream.format;
  float fps = stream.fps;

  PrintStreamInfo(kFirstStreamID);

  CameraExtraParam xtraparam;
  CameraSlaveMode camera_slave_mode;
  camera_slave_mode.mode = SlaveMode::kSlave;
  xtraparam.Update(QMMF_CAMERA_SLAVE_MODE, camera_slave_mode);

  ret = recorder_.StartCamera(camera_id_, camera_fps_, xtraparam);
  if (ret == -ENOENT) {
    // Wait for a master client to open the camera.
    std::unique_lock<std::mutex> lk(camera_state_lock_);
    std::chrono::milliseconds timeout(10000);

    camera_state_updated_.wait_for(lk, timeout, [&]() {
      return (camera_state_[camera_id_] == GtestCameraState::kOpened);
    });
    ret = recorder_.StartCamera(camera_id_, camera_fps_, xtraparam);
    ASSERT_TRUE(ret == 0);
  }

  // Random number generator.
  std::random_device rdev;
  std::mt19937 rgen(rdev());
  std::uniform_int_distribution<int32_t> idist(1, 15);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    std::cout
        << "###############################################################"
        << std::endl;

    std::cout << "Ruunnig Test Iteration: " << i << "/" << iteration_count_
              << std::endl;

    TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
              test_info_->name(), i);

    SessionCb session_status_cb = CreateSessionStatusCb();
    uint32_t session_id;

    ret = recorder_.CreateSession(session_status_cb, &session_id);
    ASSERT_TRUE(session_id > 0);
    ASSERT_TRUE(ret == 0);

    VideoTrackParam video_track_param {
      camera_id_, width, height, fps, format
    };

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, metas);
    };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    VideoExtraParam xtraparam;
    ret = recorder_.CreateVideoTrack(session_id, video_track_1,
                                     video_track_param, xtraparam,
                                     video_track_cb);
    ASSERT_TRUE(ret == 0);

    std::vector<uint32_t> track_ids;
    track_ids.push_back(video_track_1);
    sessions_.insert(std::make_pair(session_id, track_ids));

    // Start Session
    ret = recorder_.StartSession(session_id);
    ASSERT_TRUE(ret == 0);

    // Let session run for random duration or until signaled by the
    // master camera client to close camera.
    {
      std::unique_lock<std::mutex> lk(camera_state_lock_);
      std::chrono::seconds timeout(idist(rdev));

      camera_state_updated_.wait_for(lk, timeout, [&]() {
        return (camera_state_[camera_id_] == GtestCameraState::kClosing);
      });
    }

    ret = recorder_.StopSession(session_id, false);
    ASSERT_TRUE(ret == 0);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_1);
    ASSERT_TRUE(ret == 0);

    ret = recorder_.DeleteSession(session_id);
    ASSERT_TRUE(ret == 0);

    sessions_.erase(session_id);
  }

  ret = recorder_.StopCamera(camera_id_);
  ASSERT_TRUE(ret == 0);

  ret = DeInit();
  ASSERT_TRUE(ret == 0);

  std::cout <<"---------- Test Completed ----------\n" <<
      test_info_->test_case_name() << "." << test_info_->name();
}

/*
* SessionWith1080pYUVTrackMatchCameraMetaData: This test demonstrates
* how track buffer can be matched exactly with it's corresponding
* CameraMetaData using meta frame number.
* API test sequence:
*  - StartCamera
*   loop Start {
*   ------------------
*   - CreateSession
*   - CreateVideoTrack
*   - StartVideoTrack
*   - StopSession
*   - DeleteVideoTrack
*   - DeleteSession
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(VideoGtest, SessionWith1080pYUVTrackMatchCameraMetaData) {
  std::cout << "\n---------- Run Test ----------" <<
      test_info_->test_case_name() << "." << test_info_->name()<< std::endl;

  auto ret = Init();
  ASSERT_TRUE(ret == 0);

  CameraResultCb result_cb = [&](uint32_t camera_id,
                                 const CameraMetadata &result) {
    ResultCallbackHandlerMatchCameraMeta(camera_id, result);
  };

  CameraExtraParam empty_xtraparams;
  ret = recorder_.StartCamera(camera_id_, 30, empty_xtraparams, result_cb);
  ASSERT_TRUE(ret == 0);

  SessionCb session_status_cb = CreateSessionStatusCb();
  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  ASSERT_TRUE(session_id > 0);
  ASSERT_TRUE(ret == 0);
  VideoTrackParam video_track_param {
    camera_id_, 1920, 1080, 30, VideoFormat::kNV12
  };

  uint32_t video_track_id = 1;
  TrackCb video_track_cb;
  video_track_cb.data_cb = [&, session_id](
      uint32_t track_id, std::vector<BufferDescriptor> buffers,
      std::vector<BufferMeta> metas) {
    VideoTrackDataCbMatchCameraMeta(session_id, track_id, buffers,
                                    metas);
  };

  video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                void *event_data, size_t event_data_size) {
    VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
  };

  VideoExtraParam xtraparam;
  ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                   video_track_param, xtraparam,
                                   video_track_cb);
  ASSERT_TRUE(ret == 0);

  std::vector<uint32_t> track_ids;
  track_ids.push_back(video_track_id);
  sessions_.insert(std::make_pair(session_id, track_ids));

  ret = recorder_.StartSession(session_id);
  ASSERT_TRUE(ret == 0);

  // Let session run for time record_duration_, during this time buffer with
  // valid data would be received in track callback
  // (VideoTrackDataCbMatchCameraMeta).
  sleep(record_duration_ * 2);

  ret = recorder_.StopSession(session_id, false);
  ASSERT_TRUE(ret == 0);

  ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
  ASSERT_TRUE(ret == 0);

  ret = recorder_.DeleteSession(session_id);
  ASSERT_TRUE(ret == 0);

  ClearSessions();

  buffer_metadata_map_.clear();

  ret = recorder_.StopCamera(camera_id_);
  ASSERT_TRUE(ret == 0);

  ret = DeInit();
  ASSERT_TRUE(ret == 0);

  std::cout <<"---------- Test Completed ----------\n" <<
      test_info_->test_case_name() << "." << test_info_->name();
}

/*
* SessionWithSingleStreamWithCamIDOne:
*   This test will test Single stream of a configurable
*   resolution and format.
*   If EIS and SHDR is enabled, then they are also applied.
*   If Snapshot stream is on, Snapshot will also be taken.
* API test sequence:
*  - StartCamera [Check for EIS, SHDR]
*  - CreateSession
*  - CreateVideoTrack
*  - StartSession
*  - Check for SnapShot stream
*  - If Snapshot Stream  is on
*  - { ConfigImageCapture
*  -   CaptureImage
*  -   CancelCaptureImage }
*  - StopSession
*  - DeleteVideoTrack
*  - DeleteSession
*  - StopCamera
*/

TEST_F(VideoGtest, SessionWithSingleStreamWithCamIDOne) {
  std::cout << "\n---------- Run Test ----------" <<
      test_info_->test_case_name() << test_info_->name()<< std::endl;

  auto ret = Init();
  ASSERT_TRUE(ret == 0);
  camera_id_ = 1;

  // Extract Parameter of First Video Stream.
  uint32_t video_track_1 = kFirstStreamID;
  auto stream = stream_info_map_[video_track_1];
  uint32_t width = stream.width;
  uint32_t height = stream.height;
  VideoFormat format = stream.format;
  float fps = stream.fps;

  PrintStreamInfo(kFirstStreamID);

  CameraExtraParam camera_xtraparam;

  SetCameraExtraParam(camera_xtraparam);

  ret = recorder_.StartCamera(camera_id_, camera_fps_, camera_xtraparam);
  ASSERT_TRUE(ret == 0);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    std::cout
        << "###############################################################"
        << std::endl;

    std::cout << "Ruunnig Test Iteration: " << i << "/" << iteration_count_
              << std::endl;

    TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
              test_info_->name(), i);

    SessionCb session_status_cb = CreateSessionStatusCb();
    uint32_t session_id;

    ret = recorder_.CreateSession(session_status_cb, &session_id);
    ASSERT_TRUE(session_id > 0);
    ASSERT_TRUE(ret == 0);

    // Configure Single Video Stream
    VideoTrackParam video_track_param {
      camera_id_, width, height, fps, format
    };

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, metas);
    };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    VideoExtraParam xtraparam;
    ret = recorder_.CreateVideoTrack(session_id, video_track_1,
                                     video_track_param, xtraparam,
                                     video_track_cb);
    ASSERT_TRUE(ret == 0);

    std::vector<uint32_t> track_ids;
    track_ids.push_back(video_track_1);
    sessions_.insert(std::make_pair(session_id, track_ids));

    // Configure Snapshot Stream.
    if (is_snap_stream_on_) {
      ConfigureImageParam(kFirstImageID);
    }

    // Start Session
    ret = recorder_.StartSession(session_id);
    ASSERT_TRUE(ret == 0);

    sleep(record_duration_);

    // Now Take Snapshot.
    if (is_snap_stream_on_) {
      TakeSnapshot();
    }

    ret = recorder_.StopSession(session_id, false);
    ASSERT_TRUE(ret == 0);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_1);
    ASSERT_TRUE(ret == 0);

    ret = recorder_.DeleteSession(session_id);
    ASSERT_TRUE(ret == 0);

    ClearSessions();
  }

  ret = recorder_.StopCamera(camera_id_);
  ASSERT_TRUE(ret == 0);

  ret = DeInit();
  ASSERT_TRUE(ret == 0);

  std::cout <<"---------- Test Completed ----------\n" <<
      test_info_->test_case_name() << "." << test_info_->name();
}

/*
* SessionWithTwoStreamWithCamIDOne:
*   This test will test dual stream of a configurable
*   resolution and format.
*   If EIS and SHDR is enabled, then they are also applied.
*   If Snapshot stream is on, Snapshot will also be taken.
* API test sequence:
*  - StartCamera [Check for EIS, SHDR]
*  - CreateSession
*  - CreateVideoTrack for 2 tracks. [Check for linked Stream]
*  - StartSession
*  - Check for SnapShot stream
*  - If Snapshot Stream  is on
*  - { ConfigImageCapture
*  -   CaptureImage
*  -   CancelCaptureImage }
*  - StopSession
*  - DeleteVideoTrack
*  - DeleteSession
*  - StopCamera
*/

TEST_F(VideoGtest, SessionWithTwoStreamWithCamIDOne) {
  std::cout << "\n---------- Run Test ----------" <<
      test_info_->test_case_name() << test_info_->name()<< std::endl;

  auto ret = Init();
  ASSERT_TRUE(ret == 0);
  camera_id_ = 1;

  // Extract Parameter of First Video Stream.
  uint32_t video_track_1 = kFirstStreamID;
  auto stream_1 = stream_info_map_[video_track_1];
  uint32_t stream_1_width = stream_1.width;
  uint32_t stream_1_height = stream_1.height;
  VideoFormat stream_1_format = stream_1.format;
  float stream_1_fps = stream_1.fps;

  // Extract Parameter of Second Video Stream.
  uint32_t video_track_2 = kSecondStreamID;
  auto stream_2 = stream_info_map_[video_track_2];
  uint32_t stream_2_width = stream_2.width;
  uint32_t stream_2_height = stream_2.height;
  VideoFormat stream_2_format = stream_2.format;
  float stream_2_fps = stream_2.fps;
  uint32_t stream_2_src_id = stream_2.source_stream_id;

  PrintStreamInfo(kSecondStreamID);

  CameraExtraParam camera_xtraparam;

  SetCameraExtraParam(camera_xtraparam);

  ret = recorder_.StartCamera(camera_id_, camera_fps_, camera_xtraparam);
  ASSERT_TRUE(ret == 0);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    std::cout
        << "###############################################################"
        << std::endl;

    std::cout << "Ruunnig Test Iteration: " << i << "/" << iteration_count_
              << std::endl;

    TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
              test_info_->name(), i);

    SessionCb session_status_cb = CreateSessionStatusCb();
    uint32_t session_id;

    ret = recorder_.CreateSession(session_status_cb, &session_id);
    ASSERT_TRUE(session_id > 0);
    ASSERT_TRUE(ret == 0);

    // First Track Configuration
    VideoTrackParam video_track_param_1 {
      camera_id_, stream_1_width, stream_1_height, stream_1_fps, stream_1_format
    };

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, metas);
    };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    VideoExtraParam xtraparam;
    ret = recorder_.CreateVideoTrack(session_id, video_track_1,
                                     video_track_param_1, xtraparam,
                                     video_track_cb);
    ASSERT_TRUE(ret == 0);

    std::vector<uint32_t> track_ids;
    track_ids.push_back(video_track_1);

    // Second  Track Configuration
    VideoTrackParam video_track_param_2 {
      camera_id_, stream_2_width, stream_2_height, stream_2_fps, stream_2_format
    };
    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, metas);
    };

    if (stream_2_src_id != 0) {
      SourceVideoTrack surface_video_copy;
      surface_video_copy.source_track_id = stream_2_src_id;
      xtraparam.Update(QMMF_SOURCE_VIDEO_TRACK_ID, surface_video_copy);
    }

    ret = recorder_.CreateVideoTrack(session_id, video_track_2,
                                     video_track_param_2, xtraparam,
                                     video_track_cb);
    ASSERT_TRUE(ret == 0);

    track_ids.push_back(video_track_2);

    sessions_.insert(std::make_pair(session_id, track_ids));

    // Configure Snapshot Stream.
    if (is_snap_stream_on_) {
      ConfigureImageParam(kFirstImageID);
    }

    // Start Session
    ret = recorder_.StartSession(session_id);
    ASSERT_TRUE(ret == 0);

    sleep(record_duration_);

    // Now Take Snapshot.
    if (is_snap_stream_on_) {
      TakeSnapshot();
    }

    ret = recorder_.StopSession(session_id, false);
    ASSERT_TRUE(ret == 0);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_2);
    ASSERT_TRUE(ret == 0);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_1);
    ASSERT_TRUE(ret == 0);

    ret = recorder_.DeleteSession(session_id);
    ASSERT_TRUE(ret == 0);

    ClearSessions();
  }

  ret = recorder_.StopCamera(camera_id_);
  ASSERT_TRUE(ret == 0);

  ret = DeInit();
  ASSERT_TRUE(ret == 0);

  std::cout <<"---------- Test Completed ----------\n" <<
      test_info_->test_case_name() << "." << test_info_->name();
}

/*
* SessionWith1080pTrackPartialMeta: This test will test session with 1080p track.
* API test sequence:
*  - StartCamera
*  - CreateSession
*  - CreateVideoTrack
*  - StartVideoTrack
*  - StopSession
*  - DeleteVideoTrack
*  - DeleteSession
*  - StopCamera
*/

TEST_F(VideoGtest, SessionWith1080pTrackPartialMeta) {
  std::cout << "\n---------- Run Test ----------"
            << test_info_->test_case_name() << test_info_->name() << std::endl;

  auto ret = Init();
  ASSERT_TRUE(ret == 0);

  VideoFormat format = VideoFormat::kNV12;
  uint32_t width = 1920;
  uint32_t height = 1080;

  CameraResultCb result_cb = [&](uint32_t camera_id,
                                 const CameraMetadata &result) {
    if (result.exists(ANDROID_REQUEST_FRAME_COUNT)) {
      TEST_INFO("%s: MetaData FrameNumber=%d", __func__,
                result.find(ANDROID_REQUEST_FRAME_COUNT).data.i32[0]);
    }
  };

  CameraExtraParam xtraparams;
  PartialMetadata partial_metadata;
  partial_metadata.enable = true;
  xtraparams.Update(QMMF_PARTIAL_METADATA, partial_metadata);

  ret = recorder_.StartCamera(camera_id_, 30, xtraparams, result_cb);
  ASSERT_TRUE(ret == 0);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    std::cout
        << "###############################################################"
        << std::endl;

    std::cout << "Ruunnig Test Iteration: " << i << "/" << iteration_count_
              << std::endl;

    TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
              test_info_->name(), i);

    SessionCb session_status_cb = CreateSessionStatusCb();
    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    ASSERT_TRUE(session_id > 0);
    ASSERT_TRUE(ret == 0);

    VideoTrackParam video_track_param {
      camera_id_, width, height, 30, format
    };

    uint32_t video_track_id = 1;

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, metas);
    };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    VideoExtraParam xtraparam;
    ret = recorder_.CreateVideoTrack(session_id, video_track_id,
                                     video_track_param, xtraparam,
                                     video_track_cb);
    ASSERT_TRUE(ret == 0);

    std::vector<uint32_t> track_ids;
    track_ids.push_back(video_track_id);
    sessions_.insert(std::make_pair(session_id, track_ids));

    ret = recorder_.StartSession(session_id);
    ASSERT_TRUE(ret == 0);

    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    ASSERT_TRUE(ret == 0);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id);
    ASSERT_TRUE(ret == 0);

    ret = recorder_.DeleteSession(session_id);
    ASSERT_TRUE(ret == 0);
  }
  ret = recorder_.StopCamera(camera_id_);
  ASSERT_TRUE(ret == 0);

  ret = DeInit();
  ASSERT_TRUE(ret == 0);

  std::cout << "---------- Test Completed ----------\n"
            << test_info_->test_case_name() << "." << test_info_->name();
}

/*
 * SessionWith1080pYUVAnd720pYUVWithCrop: This test will test session
 *     with one 1080p YUV, 720p Cropped track.
 *
 * API test sequence:
 *  - StartCamera
 *   - CreateSession
 *   - CreateVideoTrack - Master
 *   - CreateVideoTrack - Copy
 *   - StartSession
 *   - StopSession
 *   - CreateVideoTrack - Copy
 *   - DeleteVideoTrack - Master
 *   - DeleteSession
 *  - StopCamera
 */
TEST_F(VideoGtest, SessionWith1080pYUVAnd720pYUVWithCrop) {
  std::cout << "\n---------- Run Test ----------"
            << test_info_->test_case_name() << test_info_->name() << std::endl;

  auto ret = Init();
  ASSERT_TRUE(ret == 0);

  ret = recorder_.StartCamera(camera_id_, 30);
  ASSERT_TRUE(ret == 0);

  uint32_t video_track_id_1080p = 1;
  uint32_t video_track_id_720p = 2;

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    std::cout
        << "###############################################################"
        << std::endl;

    std::cout << "Ruunnig Test Iteration: " << i << "/" << iteration_count_
              << std::endl;

    TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
              test_info_->name(), i);

    SessionCb session_status_cb = CreateSessionStatusCb();

    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    ASSERT_TRUE(session_id > 0);
    ASSERT_TRUE(ret == 0);

    // Track1: 1080p @30 AVC
    VideoTrackParam video_track_param {
      camera_id_, 1920, 1080, 30, VideoFormat::kNV12
    };

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, metas);
    };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    VideoExtraParam xtraparam;
    ret = recorder_.CreateVideoTrack(session_id, video_track_id_1080p,
                                     video_track_param, xtraparam,
                                     video_track_cb);
    ASSERT_TRUE(ret == 0);

    std::vector<uint32_t> track_ids;
    track_ids.push_back(video_track_id_1080p);

    // Track2: 1280 * 720 @30 AVC
    video_track_param.width = 1280;
    video_track_param.height = 720;

    SourceVideoTrack surface_video_copy;
    surface_video_copy.source_track_id = video_track_id_1080p;
    xtraparam.Update(QMMF_SOURCE_VIDEO_TRACK_ID, surface_video_copy);

    TrackCrop crop_param;
    crop_param.x = 100;
    crop_param.y = 100;
    crop_param.width = 1180;
    crop_param.height = 620;
    xtraparam.Update(QMMF_TRACK_CROP, crop_param);

    video_track_cb.data_cb = [&, session_id](
        uint32_t track_id, std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
      VideoTrackYUVDataCb(session_id, track_id, buffers, metas);
    };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
    };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_720p,
                                     video_track_param, xtraparam,
                                     video_track_cb);
    ASSERT_TRUE(ret == 0);

    track_ids.push_back(video_track_id_720p);

    sessions_.insert(std::make_pair(session_id, track_ids));

    ret = recorder_.StartSession(session_id);
    ASSERT_TRUE(ret == 0);

    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    ASSERT_TRUE(ret == 0);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_720p);
    ASSERT_TRUE(ret == 0);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_1080p);
    ASSERT_TRUE(ret == 0);

    ret = recorder_.DeleteSession(session_id);
    ASSERT_TRUE(ret == 0);

    ClearSessions();
  }
  ret = recorder_.StopCamera(camera_id_);
  ASSERT_TRUE(ret == 0);

  ret = DeInit();
  ASSERT_TRUE(ret == 0);

  std::cout << "---------- Test Completed ----------\n"
            << test_info_->test_case_name() << "." << test_info_->name();
}

/*
* TwoSessionsWithOneHFRStreamOneNormalStreamFromSingleCamera: This case will test two
*     sessions with one stream each from single camera, each stream gives
*     1080p,nv12 stream, one is 30fps while the other is 120fps
*
* Api test sequence summary:
*  - StartCamera
*  - CreateSession-session0
*  - CreateSession-session1
*  - CreateTrack-track0
*  - CreateTrack-track1
*  - StartSession-session0
*  - StartSession-session1
*  - StopSession-session0
*  - StopSession-session1
*  - DeleteVideoTracks-track0
*  - DeleteVideoTracks-track1
*  - DeleteSession-session0
*  - DeleteSession-session1
*  - StopCamera
*/
TEST_F(VideoGtest, TwoSessionsWithOneHFRStreamOneNormalStreamFromSingleCamera) {
  std::cout << "\n---------- Run Test ----------" <<
      test_info_->test_case_name() << "." << test_info_->name()<< std::endl;

  auto ret = Init();
  ASSERT_TRUE(ret == 0);

  // Extract Parameter of First Video Stream.
  uint32_t video_track_1 = kFirstStreamID;
  auto stream_1 = stream_info_map_[video_track_1];
  uint32_t stream_1_width = stream_1.width;
  uint32_t stream_1_height = stream_1.height;
  VideoFormat stream_1_format = stream_1.format;
  float stream_1_fps = stream_1.fps;

  // Extract Parameter of HFR Video Stream.
  uint32_t video_track_hfr = kHFRStreamID;
  auto stream_hfr = stream_info_map_[video_track_hfr];
  uint32_t stream_hfr_width = stream_hfr.width;
  uint32_t stream_hfr_height = stream_hfr.height;
  VideoFormat stream_hfr_format = stream_hfr.format;
  float stream_hfr_fps = stream_hfr.fps;

  CameraExtraParam camera_xtraparam;
  SetCameraExtraParam(camera_xtraparam);

  ret = recorder_.StartCamera(camera_id_, camera_fps_, camera_xtraparam);
  ASSERT_TRUE(ret == 0);

  SessionCb session_status_cb = CreateSessionStatusCb();
  uint32_t session_id_1;
  uint32_t session_id_hfr;

  ret = recorder_.CreateSession(session_status_cb, &session_id_1);
  ASSERT_TRUE(session_id_1 > 0);
  ASSERT_TRUE(ret == 0);

  // First Track Configuration
  VideoTrackParam video_track_param_1 {
    camera_id_, stream_1_width, stream_1_height, stream_1_fps, stream_1_format
  };

  TrackCb video_track_cb;
  video_track_cb.data_cb = [&, session_id_1](
      uint32_t track_id, std::vector<BufferDescriptor> buffers,
      std::vector<BufferMeta> metas) {
    VideoTrackYUVDataCb(session_id_1, track_id, buffers, metas);
  };

  video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                void *event_data, size_t event_data_size) {
    VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
  };

  VideoExtraParam xtraparam;
  ret = recorder_.CreateVideoTrack(session_id_1, video_track_1,
                                   video_track_param_1, xtraparam,
                                   video_track_cb);
  ASSERT_TRUE(ret == 0);

  std::vector<uint32_t> track_ids_s1;
  track_ids_s1.push_back(video_track_1);

  sessions_.insert(std::make_pair(session_id_1, track_ids_s1));

  ret = recorder_.StartSession(session_id_1);
  ASSERT_TRUE(ret == 0);

  //hfr session
  session_status_cb = CreateSessionStatusCb();

  ret = recorder_.CreateSession(session_status_cb, &session_id_hfr);
  ASSERT_TRUE(session_id_hfr > 0);
  ASSERT_TRUE(ret == 0);

  VideoTrackParam video_track_param_hfr {
    camera_id_, stream_hfr_width, stream_hfr_height, stream_hfr_fps, stream_hfr_format
  };

  video_track_cb.data_cb = [&, session_id_hfr](
      uint32_t track_id, std::vector<BufferDescriptor> buffers,
      std::vector<BufferMeta> metas) {
    VideoTrackYUVDataCb(session_id_hfr, track_id, buffers, metas);
  };

  video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                void *event_data, size_t event_data_size) {
    VideoTrackEventCb(track_id, event_type, event_data, event_data_size);
  };

  ret = recorder_.CreateVideoTrack(session_id_hfr, video_track_hfr,
                                    video_track_param_hfr, xtraparam,
                                    video_track_cb);
  ASSERT_TRUE(ret == 0);

  std::vector<uint32_t> track_ids_s2;
  track_ids_s2.push_back(video_track_hfr);

  sessions_.insert(std::make_pair(session_id_hfr, track_ids_s2));

  // Start Session
  ret = recorder_.StartSession(session_id_hfr);
  ASSERT_TRUE(ret == 0);

  sleep(record_duration_);

  ret = recorder_.StopSession(session_id_1, false);
  ASSERT_TRUE(ret == 0);

  ret = recorder_.StopSession(session_id_hfr, false);
  ASSERT_TRUE(ret == 0);

  ret = recorder_.DeleteVideoTrack(session_id_1, video_track_1);
  ASSERT_TRUE(ret == 0);

  ret = recorder_.DeleteVideoTrack(session_id_hfr, video_track_hfr);
  ASSERT_TRUE(ret == 0);

  ret = recorder_.DeleteSession(session_id_1);
  ASSERT_TRUE(ret == 0);

  ret = recorder_.DeleteSession(session_id_hfr);
  ASSERT_TRUE(ret == 0);

  ClearSessions();

  ret = recorder_.StopCamera(camera_id_);
  ASSERT_TRUE(ret == 0);

  ret = DeInit();
  ASSERT_TRUE(ret == 0);


  std::cout <<"---------- Test Completed ----------\n" <<
      test_info_->test_case_name() << "." << test_info_->name();
}
