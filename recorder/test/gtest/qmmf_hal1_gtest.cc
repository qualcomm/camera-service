/*
 * Copyright (c) 2019-2021 The Linux Foundation. All rights reserved.
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
#define LOG_TAG "RecorderHal1GTest"

#include "recorder/test/gtest/qmmf_hal1_gtest.h"

static const std::string gtest_type = "hal1_gtest";


/*
* TestMetaData: This test case will test Start & StopCamera Api query metadata.
* Api test sequence:
*   loop Start {
*   ------------------
*  - StartCamera
*  - GetCameraCharacteristics
*  - StopCamera
*   ------------------
*   } loop End
*/
TEST_F(RecorderHal1GTest, TestMetaData) {

  fprintf(stderr,"\n---------- Run Test %s.%s ------------\n",
      test_info_->test_case_name(),test_info_->name());

  auto ret = Init();
  ASSERT_TRUE(ret == NO_ERROR);
  for(uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr,"test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
        test_info_->name(), i);

    ret = recorder_.StartCamera(camera_id_, 30);
    ASSERT_TRUE(ret == NO_ERROR);

    sleep(1);

    CameraMetadata static_meta;
    ret = recorder_.GetCameraCharacteristics(camera_id_, static_meta);

    if (static_meta.exists(ANDROID_SCALER_AVAILABLE_PROCESSED_SIZES)) {
      auto entry = static_meta.find(ANDROID_SCALER_AVAILABLE_PROCESSED_SIZES);
      for (uint32_t i = 0 ; i < entry.count; i += 2) {
        QMMF_INFO("supported process sizes[%d]: %dx%d", i / 2,
          static_cast<uint32_t>(entry.data.i32[i+0]),
          static_cast<uint32_t>(entry.data.i32[i+1]));
      }
    }

    if (static_meta.exists(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS)) {
      auto entry = static_meta.find(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS);
      for (uint32_t i = 0 ; i < entry.count; i += 4) {
        QMMF_INFO("stream configurations[%d] format %d dim: %dx%d input: %d",
            i / 4,
            static_cast<uint32_t>(entry.data.i32[i+0]),
            static_cast<uint32_t>(entry.data.i32[i+1]),
            static_cast<uint32_t>(entry.data.i32[i+2]),
            static_cast<uint32_t>(entry.data.i32[i+3]));
      }
    }

    if (static_meta.exists(ANDROID_SCALER_AVAILABLE_RAW_SIZES)) {
      auto entry = static_meta.find(ANDROID_SCALER_AVAILABLE_RAW_SIZES);
      for (uint32_t i = 0 ; i < entry.count; i += 2) {
        QMMF_INFO("supported raw size[%d]: %dx%d", i / 2,
          static_cast<uint32_t>(entry.data.i32[i+0]),
          static_cast<uint32_t>(entry.data.i32[i+1]));
      }
    }

    if (static_meta.exists(ANDROID_CONTROL_AWB_AVAILABLE_MODES)) {
      auto entry = static_meta.find(ANDROID_CONTROL_AWB_AVAILABLE_MODES);
      for (uint32_t i = 0 ; i < entry.count; i++) {
        QMMF_INFO("supported awb_mode[%d]: %d", i, entry.data.u8[i]);

        CameraMetadata meta;
        uint8_t awb_mode = entry.data.u8[i];
        ret = meta.update(ANDROID_CONTROL_AWB_MODE, &awb_mode, 1);
        ASSERT_TRUE(ret == NO_ERROR);

        ret = recorder_.SetCameraParam(camera_id_, meta);
        ASSERT_TRUE(ret == NO_ERROR);

        ret = recorder_.GetCameraParam(camera_id_, meta);
        ASSERT_TRUE(ret == NO_ERROR);

        ASSERT_TRUE(meta.exists(ANDROID_CONTROL_AWB_MODE));
        ASSERT_TRUE(meta.find(ANDROID_CONTROL_AWB_MODE).data.u8[0] == awb_mode);
      }
    }

    if (static_meta.exists(ANDROID_CONTROL_AVAILABLE_EFFECTS)) {
      auto entry = static_meta.find(ANDROID_CONTROL_AVAILABLE_EFFECTS);
      for (uint32_t i = 0 ; i < entry.count; i++) {
        QMMF_INFO("supported effect[%d]: %d", i, entry.data.u8[i]);
      }
    }

    if (static_meta.exists(ANDROID_CONTROL_AVAILABLE_SCENE_MODES)) {
      auto entry = static_meta.find(ANDROID_CONTROL_AVAILABLE_SCENE_MODES);
      for (uint32_t i = 0 ; i < entry.count; i++) {
        QMMF_INFO("supported scene mode[%d]: %d", i, entry.data.u8[i]);
      }
    }

    if (static_meta.exists(ANDROID_CONTROL_AE_LOCK_AVAILABLE)) {
      auto entry = static_meta.find(ANDROID_CONTROL_AE_LOCK_AVAILABLE);
      QMMF_INFO("AE lock supported: %d", entry.data.u8[0]);
    }

    if (static_meta.exists(ANDROID_CONTROL_AWB_LOCK_AVAILABLE)) {
      auto entry = static_meta.find(ANDROID_CONTROL_AWB_LOCK_AVAILABLE);
      QMMF_INFO("AWB lock supported: %d", entry.data.u8[0]);
    }

    if (static_meta.exists(ANDROID_CONTROL_AE_COMPENSATION_RANGE)) {
      auto entry = static_meta.find(ANDROID_CONTROL_AE_COMPENSATION_RANGE);
      QMMF_INFO("AE compensation range: %d - %d",
        entry.data.i32[0], entry.data.i32[1]);
    }

    if (static_meta.exists(ANDROID_CONTROL_AE_COMPENSATION_STEP)) {
      auto entry = static_meta.find(ANDROID_CONTROL_AE_COMPENSATION_STEP);
      QMMF_INFO("AE compensation Step: %d/%d",
        entry.data.r[0].numerator, entry.data.r[0].denominator);
    }

    ret = recorder_.StopCamera(camera_id_);
    ASSERT_TRUE(ret == NO_ERROR);
  }
  ret = DeInit();
  ASSERT_TRUE(ret == NO_ERROR);
  fprintf(stderr,"---------- Test Completed %s.%s ----------\n",
      test_info_->test_case_name(), test_info_->name());
}

/*
 * SessionWith480pYUVTrack: This test case will test 480p YUV video track
 *
 * Api test sequence:
 *   loop Start {
 *   ------------------
 *   - StartCamera
 *   - CreateSession
 *   - CreateVideoTrack
 *   - StartSession
 *   - StopSession
 *   - DeleteVideoTrack
 *   - DeleteSession
 *   - StopCamera
 *   ------------------
 *   } loop End
 */
TEST_F(RecorderHal1GTest, SessionWith480pYUVTrack) {

  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
    test_info_->test_case_name(), test_info_->name());

  uint32_t video_track_id_480p_yuv = 1;
  uint32_t width = 640;
  uint32_t height = 480;
  float frame_rate = 30;

  auto ret = Init();
  ASSERT_TRUE(ret == NO_ERROR);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
      test_info_->name(), i);

    ret = recorder_.StartCamera(camera_id_, 30);
    ASSERT_TRUE(ret == NO_ERROR);

    SessionCb session_status_cb = CreateSessionStatusCb();
    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    ASSERT_TRUE(session_id > 0);
    ASSERT_TRUE(ret == NO_ERROR);

    /************************ Create Preview Track ****************************/

    VideoTrackParam video_track_param { camera_id_, VideoFormat::kNV12,
      width, height, frame_rate };

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
        std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
          VideoTrackYUVDataCb(session_id, track_id, buffers, metas); };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
        VideoTrackEventCb(track_id, event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_480p_yuv,
      video_track_param, video_track_cb);
    ASSERT_TRUE(ret == NO_ERROR);

    std::vector<uint32_t> track_ids;
    track_ids.push_back(video_track_id_480p_yuv);
    sessions_.insert(std::make_pair(session_id, track_ids));

    /************************ Start Preview  **********************************/

    ret = recorder_.StartSession(session_id);
    ASSERT_TRUE(ret == NO_ERROR);

    sleep(record_duration_);

    /************************ Stop Preview ************************************/

    ret = recorder_.StopSession(session_id, false);
    ASSERT_TRUE(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_480p_yuv);
    ASSERT_TRUE(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    ASSERT_TRUE(ret == NO_ERROR);

    ClearSessions();

    ret = recorder_.StopCamera(camera_id_);
    ASSERT_TRUE(ret == NO_ERROR);
  }

  ret = DeInit();
  ASSERT_TRUE(ret == NO_ERROR);

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
    test_info_->test_case_name(), test_info_->name());
}

/*
 * SessionWith720YUVTrack: This test case will test 720p YUV video track
 *
 * Api test sequence:
 *   loop Start {
 *   ------------------
 *   - StartCamera
 *   - CreateSession
 *   - CreateVideoTrack
 *   - StartSession
 *   - StopSession
 *   - DeleteVideoTrack
 *   - DeleteSession
 *   - StopCamera
 *   ------------------
 *   } loop End
 */
TEST_F(RecorderHal1GTest, SessionWith720YUVTrack) {

  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
    test_info_->test_case_name(), test_info_->name());

  uint32_t video_track_id_720p_yuv = 1;
  VideoFormat format = VideoFormat::kNV12;
  uint32_t width = 1280;
  uint32_t height = 720;
  float frame_rate = 30;

  auto ret = Init();
  ASSERT_TRUE(ret == NO_ERROR);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
      test_info_->name(), i);

    ret = recorder_.StartCamera(camera_id_, 30);
    ASSERT_TRUE(ret == NO_ERROR);

    SessionCb session_status_cb = CreateSessionStatusCb();
    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    ASSERT_TRUE(session_id > 0);
    ASSERT_TRUE(ret == NO_ERROR);

    /************************ Create Preview Track ****************************/

    VideoTrackParam video_track_param { camera_id_, VideoFormat::kNV12,
      width, height, frame_rate };

    if (dump_bitstream_.IsEnabled()) {
      StreamDumpInfo dumpinfo = { format, session_id, video_track_id_720p_yuv,
                                width, height };
      ret = dump_bitstream_.SetUp(dumpinfo);
      ASSERT_TRUE(ret == NO_ERROR);
    }

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
        std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
          VideoTrackYUVDataCb(session_id, track_id, buffers, metas); };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
        VideoTrackEventCb(track_id, event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_720p_yuv,
      video_track_param, video_track_cb);
    ASSERT_TRUE(ret == NO_ERROR);

    std::vector<uint32_t> track_ids;
    track_ids.push_back(video_track_id_720p_yuv);
    sessions_.insert(std::make_pair(session_id, track_ids));

    /************************ Start Preview  **********************************/

    ret = recorder_.StartSession(session_id);
    ASSERT_TRUE(ret == NO_ERROR);

    sleep(record_duration_);

    /************************ Stop Preview ************************************/

    ret = recorder_.StopSession(session_id, false);
    ASSERT_TRUE(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_720p_yuv);
    ASSERT_TRUE(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    ASSERT_TRUE(ret == NO_ERROR);

    ClearSessions();

    ret = recorder_.StopCamera(camera_id_);
    ASSERT_TRUE(ret == NO_ERROR);
  }

  ret = DeInit();
  ASSERT_TRUE(ret == NO_ERROR);

  dump_bitstream_.CloseAll();
  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
    test_info_->test_case_name(), test_info_->name());
}

/*
 * SessionWith720pYUVAnd480pYUV: This test will test session two YUV tracks,
                                 one 720p YUV and one 480 YUV Track.
 * Api test sequence:
 *  - StartCamera
*   - CreateSession
 *   loop Start {
 *   ------------------
 *   - CreateVideoTrack
 *   - CreateVideoTrack
 *   - StartSession
 *   - StopSession
 *   - DeleteVideoTrack
 *   - DeleteVideoTrack
 *   ------------------
 *   } loop End
*   - DeleteSession
 *  - StopCamera
 */
TEST_F(RecorderHal1GTest, SessionWith720pYUVAnd480pYUV) {

  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
    test_info_->test_case_name(), test_info_->name());

  auto ret = Init();
  ASSERT_TRUE(ret == NO_ERROR);

  ret = recorder_.StartCamera(camera_id_, 30);
  ASSERT_TRUE(ret == NO_ERROR);

  uint32_t video_track_id_720p_yuv = 1;
  uint32_t video_track_id_480p_yuv = 2;

  SessionCb session_status_cb = CreateSessionStatusCb();

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  ASSERT_TRUE(session_id > 0);
  ASSERT_TRUE(ret == NO_ERROR);

  if (dump_bitstream_.IsEnabled()) {
    StreamDumpInfo dumpinfo = { VideoFormat::kNV12, session_id,
        video_track_id_720p_yuv, 1280, 720 };
    ret = dump_bitstream_.SetUp(dumpinfo);
    ASSERT_TRUE(ret == NO_ERROR);

    StreamDumpInfo dumpinfo1 = { VideoFormat::kNV12, session_id,
        video_track_id_480p_yuv, 640, 480 };
    ret = dump_bitstream_.SetUp(dumpinfo1);
    ASSERT_TRUE(ret == NO_ERROR);
  }

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
      test_info_->name(), i);

    VideoTrackParam video_track_param { camera_id_, VideoFormat::kNV12,
      1280, 720, 30 };
    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
        std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
          VideoTrackYUVDataCb(session_id, track_id, buffers, metas); };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
        VideoTrackEventCb(track_id, event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_720p_yuv,
      video_track_param, video_track_cb);
    ASSERT_TRUE(ret == NO_ERROR);

    std::vector<uint32_t> track_ids;
    track_ids.push_back(video_track_id_720p_yuv);

    video_track_param.width = 640;
    video_track_param.height = 480;

    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
        std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
          VideoTrackYUVDataCb(session_id, track_id, buffers, metas); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_480p_yuv,
      video_track_param, video_track_cb);
    ASSERT_TRUE(ret == NO_ERROR);

    track_ids.push_back(video_track_id_480p_yuv);

    ret = recorder_.StartSession(session_id);
    ASSERT_TRUE(ret == NO_ERROR);

    // Let session run for time record_duration_, during this time buffer with
    // valid data would be received in track callback (VideoTrackYUVDataCb).
    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    ASSERT_TRUE(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_720p_yuv);
    ASSERT_TRUE(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_480p_yuv);
    ASSERT_TRUE(ret == NO_ERROR);

  }

  ret = recorder_.DeleteSession(session_id);
  ASSERT_TRUE(ret == NO_ERROR);

  ClearSessions();
  dump_bitstream_.CloseAll();

  ret = recorder_.StopCamera(camera_id_);
  ASSERT_TRUE(ret == NO_ERROR);

  ret = DeInit();
  ASSERT_TRUE(ret == NO_ERROR);

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
    test_info_->test_case_name(), test_info_->name());
}

/*
 * SessionWith480pYUVAnd480pYUVAndLinked480pYUV: This test will test session
                                             with three 480p streams. 3rd VGA is
                                             linked to second stream.
 * Api test sequence:
 *  - StartCamera
 *   loop Start {
 *   ------------------
 *   - CreateSession
 *   - CreateVideoTrack
 *   - CreateVideoTrack - Master
 *   - CreateVideoTrack - Linked
 *   - StartSession
 *   - StopSession
 *   - DeleteVideoTrack
 *   - DeleteVideoTrack - Master
 *   - DeleteVideoTrack - Linked
 *   - DeleteSession
 *   ------------------
 *   } loop End
 *  - StopCamera
 */
TEST_F(RecorderHal1GTest, SessionWith480pYUVAnd480pYUVAndLinked480pYUV) {

  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
    test_info_->test_case_name(), test_info_->name());

  auto ret = Init();
  ASSERT_TRUE(ret == NO_ERROR);

  ret = recorder_.StartCamera(camera_id_, 30);
  ASSERT_TRUE(ret == NO_ERROR);

  uint32_t video_track_id_480p_yuv1  = 1;
  uint32_t video_track_id_480p_yuv2  = 2;
  uint32_t video_track_id_480p_yuv3  = 3;

  SessionCb session_status_cb = CreateSessionStatusCb();

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  ASSERT_TRUE(session_id > 0);
  ASSERT_TRUE(ret == NO_ERROR);

  if (dump_bitstream_.IsEnabled()) {
    StreamDumpInfo dumpinfo = { VideoFormat::kNV12, session_id,
        video_track_id_480p_yuv1, 640, 480 };
    ret = dump_bitstream_.SetUp(dumpinfo);
    ASSERT_TRUE(ret == NO_ERROR);

    StreamDumpInfo dumpinfo1 = { VideoFormat::kNV12, session_id,
        video_track_id_480p_yuv2, 640, 480 };
    ret = dump_bitstream_.SetUp(dumpinfo1);
    ASSERT_TRUE(ret == NO_ERROR);

    StreamDumpInfo dumpinfo2 = { VideoFormat::kNV12, session_id,
        video_track_id_480p_yuv3, 640, 480 };
    ret = dump_bitstream_.SetUp(dumpinfo2);
    ASSERT_TRUE(ret == NO_ERROR);

  }

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
      test_info_->name(), i);

    VideoTrackParam video_track_param { camera_id_, VideoFormat::kNV12,
      640, 480, 30 };
    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
        std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
          VideoTrackYUVDataCb(session_id, track_id, buffers, metas); };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
        VideoTrackEventCb(track_id, event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_480p_yuv1,
      video_track_param, video_track_cb);
    ASSERT_TRUE(ret == NO_ERROR);

    std::vector<uint32_t> track_ids;
    track_ids.push_back(video_track_id_480p_yuv1);

    //VideoExtraParam extra_param;
    //SourceVideoTrack surface_video_copy;
    //surface_video_copy.source_track_id = video_track_id_1080p_yuv;
    //extra_param.Update(QMMF_SOURCE_VIDEO_TRACK_ID, surface_video_copy);

    //video_track_param.width = 640;
    //video_track_param.height = 480;

    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
        std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
          VideoTrackYUVDataCb(session_id, track_id, buffers, metas); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_480p_yuv2,
      video_track_param, video_track_cb);
    ASSERT_TRUE(ret == NO_ERROR);

    track_ids.push_back(video_track_id_480p_yuv2);

    VideoExtraParam extra_param;
    SourceVideoTrack surface_video_linked;
    surface_video_linked.source_track_id = video_track_id_480p_yuv2;
    extra_param.Update(QMMF_SOURCE_VIDEO_TRACK_ID, surface_video_linked);

    //video_track_param.width = 640;
    //video_track_param.height = 480;
    //video_track_param.format = VideoFormat::kNV12;

    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
        std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
          VideoTrackYUVDataCb(session_id, track_id, buffers, metas); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_480p_yuv3,
      video_track_param, extra_param, video_track_cb);
    ASSERT_TRUE(ret == NO_ERROR);

    track_ids.push_back(video_track_id_480p_yuv3);
    sessions_.insert(std::make_pair(session_id, track_ids));

    ret = recorder_.StartSession(session_id);
    ASSERT_TRUE(ret == NO_ERROR);

    // Let session run for time record_duration_, during this time buffer with
    // valid data would be received in track callback (VideoTrackYUVDataCb).
    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    ASSERT_TRUE(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_480p_yuv1);
    ASSERT_TRUE(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_480p_yuv2);
    ASSERT_TRUE(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_480p_yuv3);
    ASSERT_TRUE(ret == NO_ERROR);
  }

  ret = recorder_.DeleteSession(session_id);
  ASSERT_TRUE(ret == NO_ERROR);

  ClearSessions();
  dump_bitstream_.CloseAll();

  ret = recorder_.StopCamera(camera_id_);
  ASSERT_TRUE(ret == NO_ERROR);

  ret = DeInit();
  ASSERT_TRUE(ret == NO_ERROR);

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
    test_info_->test_case_name(), test_info_->name());
}

/*
 * SessionWith480pYUVand1080pYUVTracks: This test case will test 480p YUV video track
 *
 * Api test sequence:
 *   loop Start {
 *   ------------------
 *   - StartCamera
 *   - CreateSession
 *   - CreateVideoTrack
 *   - StartSession
 *   - StopSession
 *   - DeleteVideoTrack
 *   - DeleteSession
 *   - StopCamera
 *   ------------------
 *   } loop End
 */
TEST_F(RecorderHal1GTest, SessionWith480pYUVand1080pYUVTracks) {

  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
    test_info_->test_case_name(), test_info_->name());

  uint32_t video_track_id_480p_yuv = 1;
  uint32_t video_track_id_1080p_yuv = 2;
  uint32_t width = 640;
  uint32_t height = 480;
  float frame_rate = 30;

  auto ret = Init();
  ASSERT_TRUE(ret == NO_ERROR);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
      test_info_->name(), i);

    ret = recorder_.StartCamera(camera_id_, 30);
    ASSERT_TRUE(ret == NO_ERROR);

    SessionCb session_status_cb = CreateSessionStatusCb();
    uint32_t session_id;
    ret = recorder_.CreateSession(session_status_cb, &session_id);
    ASSERT_TRUE(session_id > 0);
    ASSERT_TRUE(ret == NO_ERROR);

    uint32_t session_id2;
    ret = recorder_.CreateSession(session_status_cb, &session_id2);
    ASSERT_TRUE(session_id2 > 0);
    ASSERT_TRUE(ret == NO_ERROR);

    /************************ Create Preview Track ****************************/

    VideoTrackParam video_track_param { camera_id_, VideoFormat::kNV12,
      width, height, 10 };

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
        std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
          VideoTrackYUVDataCb(session_id, track_id, buffers, metas); };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
        VideoTrackEventCb(track_id, event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_480p_yuv,
      video_track_param, video_track_cb);
    ASSERT_TRUE(ret == NO_ERROR);

    std::vector<uint32_t> track_ids;
    track_ids.push_back(video_track_id_480p_yuv);
    sessions_.insert(std::make_pair(session_id, track_ids));

    /************************ Create Preview Track ****************************/

    VideoTrackParam video_track_param2 { camera_id_, VideoFormat::kNV12,
      1920, 1080, frame_rate };

    TrackCb video_track_cb2;
    video_track_cb2.data_cb = [&, session_id2] (uint32_t track_id,
      std::vector<BufferDescriptor> buffers, std::vector<BufferMeta> metas)
    { VideoTrackYUVDataCb(session_id2, track_id, buffers, metas);};

    video_track_cb2.event_cb = [&] (uint32_t track_id, EventType event_type,
      void *event_data, size_t event_data_size) {VideoTrackEventCb(track_id,
        event_type, event_data, event_data_size);};

    ret = recorder_.CreateVideoTrack(session_id2, video_track_id_1080p_yuv,
      video_track_param2, video_track_cb2);
    ASSERT_TRUE(ret == NO_ERROR);

    track_ids.push_back(video_track_id_1080p_yuv);
    sessions_.insert(std::make_pair(session_id2, track_ids));

    /************************ Start Preview  **********************************/

    ret = recorder_.StartSession(session_id);
    ASSERT_TRUE(ret == NO_ERROR);

    sleep(1);

    ret = recorder_.StartSession(session_id2);
    ASSERT_TRUE(ret == NO_ERROR);

    sleep(record_duration_);

    /************************ Stop Preview ************************************/

    ret = recorder_.StopSession(session_id, false);
    ASSERT_TRUE(ret == NO_ERROR);

    sleep(2);

    ret = recorder_.StopSession(session_id2, false);
    ASSERT_TRUE(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id2, video_track_id_1080p_yuv);
    ASSERT_TRUE(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_480p_yuv);
    ASSERT_TRUE(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id2);
    ASSERT_TRUE(ret == NO_ERROR);

    ret = recorder_.DeleteSession(session_id);
    ASSERT_TRUE(ret == NO_ERROR);

    ClearSessions();

    ret = recorder_.StopCamera(camera_id_);
    ASSERT_TRUE(ret == NO_ERROR);
  }

  ret = DeInit();
  ASSERT_TRUE(ret == NO_ERROR);

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
    test_info_->test_case_name(), test_info_->name());
}

/*
 * SessionWith1080pYUVCopy480YUVAndLinked480YUV: This test will test session with
 *                                          one 1080 YUV track, one Copy 480 YUV
 *                                          Track and one 480 linked.
 * Api test sequence:
 *  - StartCamera
 *   loop Start {
 *   ------------------
 *   - CreateSession
 *   - CreateVideoTrack - Master
 *   - CreateVideoTrack - Copy
 *   - CreateVideoTrack - Linked
 *   - StartSession
 *   - StopSession
 *   - DeleteVideoTrack - Linked
 *   - DeleteVideoTrack - Copy
 *   - DeleteVideoTrack - Master
 *   - DeleteSession
 *   ------------------
 *   } loop End
 *  - StopCamera
 */
TEST_F(RecorderHal1GTest, SessionWith1080pYUVCopy480YUVAndLinked480YUV) {

  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
    test_info_->test_case_name(), test_info_->name());

  auto ret = Init();
  ASSERT_TRUE(ret == NO_ERROR);

  ret = recorder_.StartCamera(camera_id_, 30);
  ASSERT_TRUE(ret == NO_ERROR);

  uint32_t video_track_id_1080p_yuv = 1;
  uint32_t video_track_id_480p_yuv = 2;
  uint32_t video_track_id_480p_yuv2 = 3;

  SessionCb session_status_cb = CreateSessionStatusCb();

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  ASSERT_TRUE(session_id > 0);
  ASSERT_TRUE(ret == NO_ERROR);

  if (dump_bitstream_.IsEnabled()) {
    StreamDumpInfo dumpinfo1 = { VideoFormat::kNV12, session_id,
      video_track_id_1080p_yuv, 1920, 1080 };
    ret = dump_bitstream_.SetUp(dumpinfo1);
    ASSERT_TRUE(ret == NO_ERROR);

    StreamDumpInfo dumpinfo2 = { VideoFormat::kNV12, session_id,
      video_track_id_480p_yuv, 640, 480 };
    ret = dump_bitstream_.SetUp(dumpinfo2);
    ASSERT_TRUE(ret == NO_ERROR);
  }

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
      test_info_->name(), i);

    VideoTrackParam video_track_param { camera_id_, VideoFormat::kNV12,
      1920, 1080, 30 };
    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
        std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
          VideoTrackYUVDataCb(session_id, track_id, buffers, metas); };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
        VideoTrackEventCb(track_id, event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_1080p_yuv,
      video_track_param, video_track_cb);
    ASSERT_TRUE(ret == NO_ERROR);

    std::vector<uint32_t> track_ids;
    track_ids.push_back(video_track_id_1080p_yuv);

    VideoExtraParam extra_param;
    SourceVideoTrack surface_video_copy;
    surface_video_copy.source_track_id = video_track_id_1080p_yuv;
    extra_param.Update(QMMF_SOURCE_VIDEO_TRACK_ID, surface_video_copy);

    video_track_param.width = 640;
    video_track_param.height = 480;

    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
        std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
          VideoTrackYUVDataCb(session_id, track_id, buffers, metas); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_480p_yuv,
      video_track_param, extra_param, video_track_cb);
    ASSERT_TRUE(ret == NO_ERROR);

    track_ids.push_back(video_track_id_480p_yuv);

    VideoExtraParam extra_param2;
    SourceVideoTrack surface_video_linked;
    surface_video_linked.source_track_id = video_track_id_480p_yuv;
    extra_param2.Update(QMMF_SOURCE_VIDEO_TRACK_ID, surface_video_linked);

    video_track_param.width = 640;
    video_track_param.height = 480;
    video_track_param.format = VideoFormat::kNV12;

    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
        std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
          VideoTrackYUVDataCb(session_id, track_id, buffers, metas); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_480p_yuv2,
      video_track_param, extra_param2, video_track_cb);
    ASSERT_TRUE(ret == NO_ERROR);

    track_ids.push_back(video_track_id_480p_yuv2);
    sessions_.insert(std::make_pair(session_id, track_ids));

    ret = recorder_.StartSession(session_id);
    ASSERT_TRUE(ret == NO_ERROR);

    // Let session run for time record_duration_, during this time buffer with
    // valid data would be received in track callback (VideoTrackYUVDataCb).
    sleep(record_duration_);

    ret = recorder_.StopSession(session_id, false);
    ASSERT_TRUE(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_480p_yuv2);
    ASSERT_TRUE(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_480p_yuv);
    ASSERT_TRUE(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_1080p_yuv);
    ASSERT_TRUE(ret == NO_ERROR);
  }

  ret = recorder_.DeleteSession(session_id);
  ASSERT_TRUE(ret == NO_ERROR);

  ClearSessions();
  dump_bitstream_.CloseAll();

  ret = recorder_.StopCamera(camera_id_);
  ASSERT_TRUE(ret == NO_ERROR);

  ret = DeInit();
  ASSERT_TRUE(ret == NO_ERROR);

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
    test_info_->test_case_name(), test_info_->name());
}

/*
 * SessionWithTwo1080pYUVAndLinked1080pYUV: This test will test session with
 *                                          two 1080 YUV tracks and one 1080p
 *                                          YUV linked track.
 * Api test sequence:
 *  - StartCamera
 *   loop Start {
 *   ------------------
 *   - CreateSession
 *   - CreateVideoTrack - Master
 *   - CreateVideoTrack - Master
 *   - CreateVideoTrack - Linked
 *   - StartSession
 *   - StopSession
 *   - DeleteVideoTrack - Linked
 *   - DeleteVideoTrack - Master
 *   - DeleteVideoTrack - Master
 *   - DeleteSession
 *   ------------------
 *   } loop End
 *  - StopCamera
 */
TEST_F(RecorderHal1GTest, SessionWithTwo1080pYUVAndLinked1080pYUV) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
    test_info_->test_case_name(), test_info_->name());

  auto ret = Init();
  ASSERT_TRUE(ret == NO_ERROR);

  ret = recorder_.StartCamera(camera_id_, 30);
  ASSERT_TRUE(ret == NO_ERROR);

  uint32_t video_track_id_1080p_yuv1 = 1;
  uint32_t video_track_id_1080p_yuv2 = 2;
  uint32_t video_track_id_1080p_yuv_linked = 3;

  SessionCb session_status_cb = CreateSessionStatusCb();

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  ASSERT_TRUE(session_id > 0);
  ASSERT_TRUE(ret == NO_ERROR);

  if (dump_bitstream_.IsEnabled()) {
    StreamDumpInfo dumpinfo1 = { VideoFormat::kNV12, session_id,
      video_track_id_1080p_yuv2, 1920, 1080 };
    ret = dump_bitstream_.SetUp(dumpinfo1);
    ASSERT_TRUE(ret == NO_ERROR);

    StreamDumpInfo dumpinfo2 = { VideoFormat::kNV12, session_id,
      video_track_id_1080p_yuv2, 1920, 1080 };
    ret = dump_bitstream_.SetUp(dumpinfo2);
    ASSERT_TRUE(ret == NO_ERROR);

    StreamDumpInfo dumpinfo3 = { VideoFormat::kNV12, session_id,
      video_track_id_1080p_yuv_linked, 1920, 1080 };
    ret = dump_bitstream_.SetUp(dumpinfo2);
    ASSERT_TRUE(ret == NO_ERROR);
  }

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
      test_info_->name(), i);

    /************************ Create 1080p Track 1 ****************************/

    VideoTrackParam video_track_param {
      camera_id_, VideoFormat::kNV12, 1920, 1080, 30 };

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
        std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
          VideoTrackYUVDataCb(session_id, track_id, buffers, metas); };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
        VideoTrackEventCb(track_id, event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_1080p_yuv1,
      video_track_param, video_track_cb);
    ASSERT_TRUE(ret == NO_ERROR);

    std::vector<uint32_t> track_ids;
    track_ids.push_back(video_track_id_1080p_yuv1);

    /************************ Create 1080p Track 2 ****************************/

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_1080p_yuv2,
      video_track_param, video_track_cb);
    ASSERT_TRUE(ret == NO_ERROR);

    track_ids.push_back(video_track_id_1080p_yuv2);


    /************************ Create 1080p linked Track ***********************/

    VideoExtraParam extra_param;
    SourceVideoTrack surface_video_linked;
    surface_video_linked.source_track_id = video_track_id_1080p_yuv2;
    extra_param.Update(QMMF_SOURCE_VIDEO_TRACK_ID, surface_video_linked);

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_1080p_yuv_linked,
      video_track_param, extra_param, video_track_cb);
    ASSERT_TRUE(ret == NO_ERROR);

    track_ids.push_back(video_track_id_1080p_yuv_linked);
    sessions_.insert(std::make_pair(session_id, track_ids));

    /************************ Start Session ***********************************/

    ret = recorder_.StartSession(session_id);
    ASSERT_TRUE(ret == NO_ERROR);

    // Let session run for time record_duration_, during this time buffer with
    // valid data would be received in track callback (VideoTrackYUVDataCb).
    sleep(record_duration_);

    /************************ Stop Session ************************************/

    ret = recorder_.StopSession(session_id, false);
    ASSERT_TRUE(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_1080p_yuv1);
    ASSERT_TRUE(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_1080p_yuv2);
    ASSERT_TRUE(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_1080p_yuv_linked);
    ASSERT_TRUE(ret == NO_ERROR);
  }

  ret = recorder_.DeleteSession(session_id);
  ASSERT_TRUE(ret == NO_ERROR);

  ClearSessions();
  dump_bitstream_.CloseAll();

  ret = recorder_.StopCamera(camera_id_);
  ASSERT_TRUE(ret == NO_ERROR);

  ret = DeInit();
  ASSERT_TRUE(ret == NO_ERROR);

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
    test_info_->test_case_name(), test_info_->name());
}

/*
* SessionWith720pYUVAndSnapshotVGA: Test one 720p YUV track and b2b VGA snapshot
*
* API test sequence:
*  - StartCamera
*   loop Start {
*   ------------------
*   - CaptureImage - JPEG
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(RecorderHal1GTest, SessionWith720pYUVAndSnapshotVGA) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
    test_info_->test_case_name(), test_info_->name());

  auto ret = Init();
  ASSERT_TRUE(ret == NO_ERROR);

  ret = recorder_.StartCamera(camera_id_, 30);
  ASSERT_TRUE(ret == NO_ERROR);

  uint32_t video_track_id_yuv1 = 1;
  int32_t pending_count = 0;

  std::vector<std::pair<uint32_t, uint32_t>> test_res = {
    {640, 480} };

  SessionCb session_status_cb = CreateSessionStatusCb();

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  ASSERT_TRUE(session_id > 0);
  ASSERT_TRUE(ret == NO_ERROR);

  for (auto& res : test_res) {
    uint32_t width = res.first;
    uint32_t height = res.second;
    QMMF_INFO("Test dim: %dx%d", width, height);

    if (dump_bitstream_.IsEnabled()) {
      StreamDumpInfo dumpinfo1 = { VideoFormat::kNV12, session_id,
        video_track_id_yuv1, 1280, 720 };
      ret = dump_bitstream_.SetUp(dumpinfo1);
      ASSERT_TRUE(ret == NO_ERROR);
    }

    /************************ Create Track 1 ****************************/

    VideoTrackParam video_track_param {
      camera_id_, VideoFormat::kNV12, 1280, 720, 30 };

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
        std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
          VideoTrackYUVDataCb(session_id, track_id, buffers, metas); };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
        VideoTrackEventCb(track_id, event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_yuv1,
      video_track_param, video_track_cb);
    ASSERT_TRUE(ret == NO_ERROR);

    std::vector<uint32_t> track_ids;
    track_ids.push_back(video_track_id_yuv1);

    /*********************** Configure Snapshot ******************************/

    ImageParam image_param{};
    image_param.width   = width;
    image_param.height  = height;
    image_param.format  = ImageFormat::kJPEG;
    image_param.quality = default_jpeg_quality_;

    std::vector<CameraMetadata> meta_array;
    camera_metadata_entry_t entry;
    CameraMetadata meta;

    ret = recorder_.GetDefaultCaptureParam(camera_id_, meta);
    ASSERT_TRUE(ret == NO_ERROR);

    CameraMetadata static_meta;
    ret = recorder_.GetCameraCharacteristics(camera_id_, static_meta);
    ASSERT_TRUE(ret == NO_ERROR);

    meta_array.push_back(meta);

    ASSERT_TRUE (ValidateResFromProcessedSizes(static_meta, image_param.width,
        image_param.height) != false);

    ImageCaptureCb cb = [&] (uint32_t camera_id, uint32_t image_count,
                             BufferDescriptor buffer, BufferMeta meta) -> void
        { SnapshotCb(camera_id, image_count, buffer, meta);
          pending_count--; };

    ImageExtraParam xtraparam;

    ret = recorder_.ConfigImageCapture(camera_id_, image_param, xtraparam);
    ASSERT_TRUE(ret == NO_ERROR);

    /************************ Start Session ***********************************/

    ret = recorder_.StartSession(session_id);
    ASSERT_TRUE(ret == NO_ERROR);

    sleep(5);

    for (uint32_t i = 1; i <= iteration_count_; i++) {
      fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
      TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
        test_info_->name(), i);

      pending_count++;
      ret = recorder_.CaptureImage(camera_id_, 1, meta_array, cb);
      ASSERT_TRUE(ret == NO_ERROR);
      sleep(5);
    }

    ret = recorder_.CancelCaptureImage(camera_id_);
    ASSERT_TRUE(ret == NO_ERROR);

    // Check if all snapshot are done
    assert(pending_count == 0);

    sleep(1);

    /************************ Stop Session ************************************/

    ret = recorder_.StopSession(session_id, false);
    ASSERT_TRUE(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_yuv1);
    ASSERT_TRUE(ret == NO_ERROR);


    dump_bitstream_.CloseAll();
  }

  ret = recorder_.DeleteSession(session_id);
  ASSERT_TRUE(ret == NO_ERROR);

  ClearSessions();

  ret = recorder_.StopCamera(camera_id_);
  ASSERT_TRUE(ret == NO_ERROR);

  ret = DeInit();
  ASSERT_TRUE(ret == NO_ERROR);

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
    test_info_->test_case_name(), test_info_->name());
}


/*
* SessionWith720pYUVAndSnapshot720p: Test one 720p YUV track and b2b 720p snapshot
*
* API test sequence:
*  - StartCamera
*   loop Start {
*   ------------------
*   - CaptureImage - JPEG
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(RecorderHal1GTest, SessionWith720pYUVAndSnapshot720p) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
    test_info_->test_case_name(), test_info_->name());

  auto ret = Init();
  ASSERT_TRUE(ret == NO_ERROR);

  ret = recorder_.StartCamera(camera_id_, 30);
  ASSERT_TRUE(ret == NO_ERROR);

  uint32_t video_track_id_yuv1 = 1;
  int32_t pending_count = 0;

  std::vector<std::pair<uint32_t, uint32_t>> test_res = {
    {1280, 720} };

  SessionCb session_status_cb = CreateSessionStatusCb();

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  ASSERT_TRUE(session_id > 0);
  ASSERT_TRUE(ret == NO_ERROR);

  for (auto& res : test_res) {
    uint32_t width = res.first;
    uint32_t height = res.second;
    QMMF_INFO("Test dim: %dx%d", width, height);

    if (dump_bitstream_.IsEnabled()) {
      StreamDumpInfo dumpinfo1 = { VideoFormat::kNV12, session_id,
        video_track_id_yuv1, width, height };
      ret = dump_bitstream_.SetUp(dumpinfo1);
      ASSERT_TRUE(ret == NO_ERROR);
    }

    /************************ Create Track 1 ****************************/

    VideoTrackParam video_track_param {
      camera_id_, VideoFormat::kNV12, width, height, 30 };

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
        std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
          VideoTrackYUVDataCb(session_id, track_id, buffers, metas); };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
        VideoTrackEventCb(track_id, event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_yuv1,
      video_track_param, video_track_cb);
    ASSERT_TRUE(ret == NO_ERROR);

    std::vector<uint32_t> track_ids;
    track_ids.push_back(video_track_id_yuv1);

    /*********************** Configure Snapshot ******************************/

    ImageParam image_param{};
    image_param.width   = width;
    image_param.height  = height;
    image_param.format  = ImageFormat::kJPEG;
    image_param.quality = default_jpeg_quality_;

    std::vector<CameraMetadata> meta_array;
    camera_metadata_entry_t entry;
    CameraMetadata meta;

    ret = recorder_.GetDefaultCaptureParam(camera_id_, meta);
    ASSERT_TRUE(ret == NO_ERROR);

    CameraMetadata static_meta;
    ret = recorder_.GetCameraCharacteristics(camera_id_, static_meta);
    ASSERT_TRUE(ret == NO_ERROR);

    meta_array.push_back(meta);

    ASSERT_TRUE (ValidateResFromProcessedSizes(static_meta, image_param.width,
        image_param.height) != false);

    ImageCaptureCb cb = [&] (uint32_t camera_id, uint32_t image_count,
                             BufferDescriptor buffer, BufferMeta meta) -> void
        { SnapshotCb(camera_id, image_count, buffer, meta);
          pending_count--; };

    ImageExtraParam xtraparam;

    ret = recorder_.ConfigImageCapture(camera_id_, image_param, xtraparam);
    ASSERT_TRUE(ret == NO_ERROR);

    /************************ Start Session ***********************************/

    ret = recorder_.StartSession(session_id);
    ASSERT_TRUE(ret == NO_ERROR);

    sleep(5);

    for (uint32_t i = 1; i <= iteration_count_; i++) {
      fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
      TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
        test_info_->name(), i);

      pending_count++;
      ret = recorder_.CaptureImage(camera_id_, 1, meta_array, cb);
      ASSERT_TRUE(ret == NO_ERROR);
      sleep(5);
    }

    ret = recorder_.CancelCaptureImage(camera_id_);
    ASSERT_TRUE(ret == NO_ERROR);

    // Check if all snapshot are done
    assert(pending_count == 0);

    sleep(1);

    /************************ Stop Session ************************************/

    ret = recorder_.StopSession(session_id, false);
    ASSERT_TRUE(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_yuv1);
    ASSERT_TRUE(ret == NO_ERROR);


    dump_bitstream_.CloseAll();
  }

  ret = recorder_.DeleteSession(session_id);
  ASSERT_TRUE(ret == NO_ERROR);

  ClearSessions();

  ret = recorder_.StopCamera(camera_id_);
  ASSERT_TRUE(ret == NO_ERROR);

  ret = DeInit();
  ASSERT_TRUE(ret == NO_ERROR);

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
    test_info_->test_case_name(), test_info_->name());
}

/*
* SessionWithTwo720pYUVAndSnapshot720p: Test two YUV 720p tracks and b2b 720p snapshot
*
* API test sequence:
*  - StartCamera
*   loop Start {
*   ------------------
*   - CaptureImage - JPEG
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(RecorderHal1GTest, SessionWithTwo720pYUVAndSnapshot720p) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
    test_info_->test_case_name(), test_info_->name());

  auto ret = Init();
  ASSERT_TRUE(ret == NO_ERROR);

  ret = recorder_.StartCamera(camera_id_, 30);
  ASSERT_TRUE(ret == NO_ERROR);

  uint32_t video_track_id_yuv1 = 1;
  uint32_t video_track_id_yuv2 = 2;
  int32_t pending_count = 0;

  std::vector<std::pair<uint32_t, uint32_t>> test_res = {
    {1280, 720} };

  SessionCb session_status_cb = CreateSessionStatusCb();

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  ASSERT_TRUE(session_id > 0);
  ASSERT_TRUE(ret == NO_ERROR);

  for (auto& res : test_res) {
    uint32_t width = res.first;
    uint32_t height = res.second;
    QMMF_INFO("Test dim: %dx%d", width, height);

    if (dump_bitstream_.IsEnabled()) {
      StreamDumpInfo dumpinfo1 = { VideoFormat::kNV12, session_id,
        video_track_id_yuv1, width, height };
      ret = dump_bitstream_.SetUp(dumpinfo1);
      ASSERT_TRUE(ret == NO_ERROR);

      StreamDumpInfo dumpinfo2 = { VideoFormat::kNV12, session_id,
        video_track_id_yuv2, width, height };
      ret = dump_bitstream_.SetUp(dumpinfo2);
      ASSERT_TRUE(ret == NO_ERROR);
    }

    /************************ Create Track 1 ****************************/

    VideoTrackParam video_track_param {
      camera_id_, VideoFormat::kNV12, width, height, 30 };

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
        std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
          VideoTrackYUVDataCb(session_id, track_id, buffers, metas); };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
        VideoTrackEventCb(track_id, event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_yuv1,
      video_track_param, video_track_cb);
    ASSERT_TRUE(ret == NO_ERROR);

    std::vector<uint32_t> track_ids;
    track_ids.push_back(video_track_id_yuv1);

    /************************ Create Track 2 ****************************/

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_yuv2,
      video_track_param, video_track_cb);
    ASSERT_TRUE(ret == NO_ERROR);

    track_ids.push_back(video_track_id_yuv2);
    sessions_.insert(std::make_pair(session_id, track_ids));

    /*********************** Configure Snapshot ******************************/

    ImageParam image_param{};
    image_param.width   = width;
    image_param.height  = height;
    image_param.format  = ImageFormat::kJPEG;
    image_param.quality = default_jpeg_quality_;

    std::vector<CameraMetadata> meta_array;
    camera_metadata_entry_t entry;
    CameraMetadata meta;

    ret = recorder_.GetDefaultCaptureParam(camera_id_, meta);
    ASSERT_TRUE(ret == NO_ERROR);

    CameraMetadata static_meta;
    ret = recorder_.GetCameraCharacteristics(camera_id_, static_meta);
    ASSERT_TRUE(ret == NO_ERROR);

    meta_array.push_back(meta);

    ASSERT_TRUE (ValidateResFromProcessedSizes(static_meta, image_param.width,
        image_param.height) != false);

    ImageCaptureCb cb = [&] (uint32_t camera_id, uint32_t image_count,
                             BufferDescriptor buffer, BufferMeta meta) -> void
        { SnapshotCb(camera_id, image_count, buffer, meta);
          pending_count--; };

    ImageExtraParam xtraparam;

    ret = recorder_.ConfigImageCapture(camera_id_, image_param, xtraparam);
    ASSERT_TRUE(ret == NO_ERROR);

    /************************ Start Session ***********************************/

    ret = recorder_.StartSession(session_id);
    ASSERT_TRUE(ret == NO_ERROR);

    sleep(5);

    for (uint32_t i = 1; i <= iteration_count_; i++) {
      fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
      TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
        test_info_->name(), i);

      pending_count++;
      ret = recorder_.CaptureImage(camera_id_, 1, meta_array, cb);
      ASSERT_TRUE(ret == NO_ERROR);
      sleep(5);
    }

    ret = recorder_.CancelCaptureImage(camera_id_);
    ASSERT_TRUE(ret == NO_ERROR);

    // Check if all snapshot are done
    assert(pending_count == 0);

    sleep(1);

    /************************ Stop Session ************************************/

    ret = recorder_.StopSession(session_id, false);
    ASSERT_TRUE(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_yuv1);
    ASSERT_TRUE(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_yuv2);
    ASSERT_TRUE(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.DeleteSession(session_id);
  ASSERT_TRUE(ret == NO_ERROR);

  ClearSessions();

  ret = recorder_.StopCamera(camera_id_);
  ASSERT_TRUE(ret == NO_ERROR);

  ret = DeInit();
  ASSERT_TRUE(ret == NO_ERROR);

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
    test_info_->test_case_name(), test_info_->name());
}

/*
* SessionWithTwoVGAYUVAndSnapshotVGA: Test two YUV VGA tracks and b2b VGA snapshot
*
* API test sequence:
*  - StartCamera
*   loop Start {
*   ------------------
*   - CaptureImage - JPEG
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(RecorderHal1GTest, SessionWithTwoVGAYUVAndSnapshotVGA) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
    test_info_->test_case_name(), test_info_->name());

  auto ret = Init();
  ASSERT_TRUE(ret == NO_ERROR);

  ret = recorder_.StartCamera(camera_id_, 30);
  ASSERT_TRUE(ret == NO_ERROR);

  uint32_t video_track_id_yuv1 = 1;
  uint32_t video_track_id_yuv2 = 2;
  int32_t pending_count = 0;

  std::vector<std::pair<uint32_t, uint32_t>> test_res = {
    {640, 480} };

  SessionCb session_status_cb = CreateSessionStatusCb();

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  ASSERT_TRUE(session_id > 0);
  ASSERT_TRUE(ret == NO_ERROR);

  for (auto& res : test_res) {
    uint32_t width = res.first;
    uint32_t height = res.second;
    QMMF_INFO("Test dim: %dx%d", width, height);

    if (dump_bitstream_.IsEnabled()) {
      StreamDumpInfo dumpinfo1 = { VideoFormat::kNV12, session_id,
        video_track_id_yuv1, width, height };
      ret = dump_bitstream_.SetUp(dumpinfo1);
      ASSERT_TRUE(ret == NO_ERROR);

      StreamDumpInfo dumpinfo2 = { VideoFormat::kNV12, session_id,
        video_track_id_yuv2, width, height };
      ret = dump_bitstream_.SetUp(dumpinfo2);
      ASSERT_TRUE(ret == NO_ERROR);
    }

    /************************ Create Track 1 ****************************/

    VideoTrackParam video_track_param {
      camera_id_, VideoFormat::kNV12, width, height, 30 };

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
        std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
          VideoTrackYUVDataCb(session_id, track_id, buffers, metas); };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
        VideoTrackEventCb(track_id, event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_yuv1,
      video_track_param, video_track_cb);
    ASSERT_TRUE(ret == NO_ERROR);

    std::vector<uint32_t> track_ids;
    track_ids.push_back(video_track_id_yuv1);

    /************************ Create Track 2 ****************************/

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_yuv2,
      video_track_param, video_track_cb);
    ASSERT_TRUE(ret == NO_ERROR);

    track_ids.push_back(video_track_id_yuv2);
    sessions_.insert(std::make_pair(session_id, track_ids));

    /*********************** Configure Snapshot ******************************/

    ImageParam image_param{};
    image_param.width   = width;
    image_param.height  = height;
    image_param.format  = ImageFormat::kJPEG;
    image_param.quality = default_jpeg_quality_;

    std::vector<CameraMetadata> meta_array;
    camera_metadata_entry_t entry;
    CameraMetadata meta;

    ret = recorder_.GetDefaultCaptureParam(camera_id_, meta);
    ASSERT_TRUE(ret == NO_ERROR);

    CameraMetadata static_meta;
    ret = recorder_.GetCameraCharacteristics(camera_id_, static_meta);
    ASSERT_TRUE(ret == NO_ERROR);

    meta_array.push_back(meta);

    ASSERT_TRUE (ValidateResFromProcessedSizes(static_meta, image_param.width,
        image_param.height) != false);

    ImageCaptureCb cb = [&] (uint32_t camera_id, uint32_t image_count,
                             BufferDescriptor buffer, BufferMeta meta) -> void
        { SnapshotCb(camera_id, image_count, buffer, meta);
          pending_count--; };

    ImageExtraParam xtraparam;

    ret = recorder_.ConfigImageCapture(camera_id_, image_param, xtraparam);
    ASSERT_TRUE(ret == NO_ERROR);

    /************************ Start Session ***********************************/

    ret = recorder_.StartSession(session_id);
    ASSERT_TRUE(ret == NO_ERROR);

    sleep(5);

    for (uint32_t i = 1; i <= iteration_count_; i++) {
      fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
      TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
        test_info_->name(), i);

      pending_count++;
      ret = recorder_.CaptureImage(camera_id_, 1, meta_array, cb);
      ASSERT_TRUE(ret == NO_ERROR);
      sleep(5);
    }

    ret = recorder_.CancelCaptureImage(camera_id_);
    ASSERT_TRUE(ret == NO_ERROR);

    // Check if all snapshot are done
    assert(pending_count == 0);

    sleep(1);

    /************************ Stop Session ************************************/

    ret = recorder_.StopSession(session_id, false);
    ASSERT_TRUE(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_yuv1);
    ASSERT_TRUE(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_yuv2);
    ASSERT_TRUE(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.DeleteSession(session_id);
  ASSERT_TRUE(ret == NO_ERROR);

  ClearSessions();

  ret = recorder_.StopCamera(camera_id_);
  ASSERT_TRUE(ret == NO_ERROR);

  ret = DeInit();
  ASSERT_TRUE(ret == NO_ERROR);

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
    test_info_->test_case_name(), test_info_->name());
}

/*
* SessionWithYUVTrackAndToggleSnapshotRes: Test one video tracks and toggle
*                                          snapshot resolutions while streaming.
*
* API test sequence:
*  - StartCamera
*   loop Start {
*   ------------------
*   - CaptureImage - JPEG
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(RecorderHal1GTest, SessionWithYUVTrackAndToggleSnapshotRes) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
    test_info_->test_case_name(), test_info_->name());

  auto ret = Init();
  ASSERT_TRUE(ret == NO_ERROR);

  ret = recorder_.StartCamera(camera_id_, 30);
  ASSERT_TRUE(ret == NO_ERROR);

  uint32_t video_track_id_yuv1 = 1;
  int32_t pending_count = 0;

  std::vector<std::pair<uint32_t, uint32_t>> test_res = {
    {640, 480},
    {1280, 720} };

  SessionCb session_status_cb = CreateSessionStatusCb();

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  ASSERT_TRUE(session_id > 0);
  ASSERT_TRUE(ret == NO_ERROR);

  if (dump_bitstream_.IsEnabled()) {
    StreamDumpInfo dumpinfo1 = { VideoFormat::kNV12, session_id,
      video_track_id_yuv1, 720, 480 };
    ret = dump_bitstream_.SetUp(dumpinfo1);
    ASSERT_TRUE(ret == NO_ERROR);
  }

  /************************ Create Track 1 ****************************/

  VideoTrackParam video_track_param {
    camera_id_, VideoFormat::kNV12, 720, 480, 30 };

  TrackCb video_track_cb;
  video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
      std::vector<BufferDescriptor> buffers,
      std::vector<BufferMeta> metas) {
        VideoTrackYUVDataCb(session_id, track_id, buffers, metas); };

  video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                void *event_data, size_t event_data_size) {
      VideoTrackEventCb(track_id, event_type, event_data, event_data_size); };

  ret = recorder_.CreateVideoTrack(session_id, video_track_id_yuv1,
    video_track_param, video_track_cb);
  ASSERT_TRUE(ret == NO_ERROR);

  std::vector<uint32_t> track_ids;
  track_ids.push_back(video_track_id_yuv1);

  /*********************** Configure Snapshot ******************************/

  ImageParam image_param{};
  image_param.format  = ImageFormat::kJPEG;
  image_param.quality = default_jpeg_quality_;

  std::vector<CameraMetadata> meta_array;
  camera_metadata_entry_t entry;
  CameraMetadata meta;

  ret = recorder_.GetDefaultCaptureParam(camera_id_, meta);
  ASSERT_TRUE(ret == NO_ERROR);

  CameraMetadata static_meta;
  ret = recorder_.GetCameraCharacteristics(camera_id_, static_meta);
  ASSERT_TRUE(ret == NO_ERROR);

  meta_array.push_back(meta);

  ImageCaptureCb cb = [&] (uint32_t camera_id, uint32_t image_count,
                           BufferDescriptor buffer, BufferMeta meta) -> void
      { SnapshotCb(camera_id, image_count, buffer, meta);
        pending_count--; };

  ImageExtraParam xtraparam;

  ret = recorder_.ConfigImageCapture(camera_id_, image_param, xtraparam);
  ASSERT_TRUE(ret == NO_ERROR);

  /************************ Start Session ***********************************/

  ret = recorder_.StartSession(session_id);
  ASSERT_TRUE(ret == NO_ERROR);

  sleep(5);

  for (uint32_t i = 1; i <= iteration_count_; i++) {
    fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
    TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
      test_info_->name(), i);

    for (auto& res : test_res) {
      image_param.width  = res.first;
      image_param.height = res.second;
      QMMF_INFO("Test dim: %dx%d", res.first, res.second);

      ASSERT_TRUE (ValidateResFromProcessedSizes(static_meta, image_param.width,
          image_param.height) != false);

      pending_count++;
      ret = recorder_.ConfigImageCapture(camera_id_, image_param, xtraparam);
      ASSERT_TRUE(ret == NO_ERROR);
      ret = recorder_.CaptureImage(camera_id_, 1, meta_array, cb);
      ASSERT_TRUE(ret == NO_ERROR);
      sleep(5);

      pending_count++;
      ret = recorder_.ConfigImageCapture(camera_id_, image_param, xtraparam);
      ASSERT_TRUE(ret == NO_ERROR);
      ret = recorder_.CaptureImage(camera_id_, 1, meta_array, cb);
      ASSERT_TRUE(ret == NO_ERROR);
      sleep(5);
    }
  }

  ret = recorder_.CancelCaptureImage(camera_id_);
  ASSERT_TRUE(ret == NO_ERROR);

  // Check if all snapshot are done
  assert(pending_count == 0);

  sleep(1);

  /************************ Stop Session ************************************/

  ret = recorder_.StopSession(session_id, false);
  ASSERT_TRUE(ret == NO_ERROR);

  ret = recorder_.DeleteVideoTrack(session_id, video_track_id_yuv1);
  ASSERT_TRUE(ret == NO_ERROR);


  dump_bitstream_.CloseAll();

  ret = recorder_.DeleteSession(session_id);
  ASSERT_TRUE(ret == NO_ERROR);

  ClearSessions();

  ret = recorder_.StopCamera(camera_id_);
  ASSERT_TRUE(ret == NO_ERROR);

  ret = DeInit();
  ASSERT_TRUE(ret == NO_ERROR);

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
    test_info_->test_case_name(), test_info_->name());
}

/*
* SessionWithYUVTrackAndSnapshot: Test one video tracks and snapshot with
*                                 multiple resolutions.
* API test sequence:
*  - StartCamera
*   loop Start {
*   ------------------
*   - CaptureImage - JPEG
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(RecorderHal1GTest, SessionWithYUVTrackAndSnapshot) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
    test_info_->test_case_name(), test_info_->name());

  auto ret = Init();
  ASSERT_TRUE(ret == NO_ERROR);

  ret = recorder_.StartCamera(camera_id_, 30);
  ASSERT_TRUE(ret == NO_ERROR);

  uint32_t video_track_id_yuv1 = 1;
  int32_t pending_count = 0;

  std::vector<std::pair<uint32_t, uint32_t>> test_res = {
    {640, 480},
    {720, 480},
    {1280, 720} };

  SessionCb session_status_cb = CreateSessionStatusCb();

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  ASSERT_TRUE(session_id > 0);
  ASSERT_TRUE(ret == NO_ERROR);

  for (auto& res : test_res) {
    uint32_t width = res.first;
    uint32_t height = res.second;
    QMMF_INFO("Test dim: %dx%d", width, height);

    if (dump_bitstream_.IsEnabled()) {
      StreamDumpInfo dumpinfo1 = { VideoFormat::kNV12, session_id,
        video_track_id_yuv1, width, height };
      ret = dump_bitstream_.SetUp(dumpinfo1);
      ASSERT_TRUE(ret == NO_ERROR);
    }

    /************************ Create Track 1 ****************************/

    VideoTrackParam video_track_param {
      camera_id_, VideoFormat::kNV12, width, height, 30 };

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
        std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
          VideoTrackYUVDataCb(session_id, track_id, buffers, metas); };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
        VideoTrackEventCb(track_id, event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_yuv1,
      video_track_param, video_track_cb);
    ASSERT_TRUE(ret == NO_ERROR);

    std::vector<uint32_t> track_ids;
    track_ids.push_back(video_track_id_yuv1);

    /*********************** Configure Snapshot ******************************/

    ImageParam image_param{};
    image_param.width   = width;
    image_param.height  = height;
    image_param.format  = ImageFormat::kJPEG;
    image_param.quality = default_jpeg_quality_;

    std::vector<CameraMetadata> meta_array;
    camera_metadata_entry_t entry;
    CameraMetadata meta;

    ret = recorder_.GetDefaultCaptureParam(camera_id_, meta);
    ASSERT_TRUE(ret == NO_ERROR);

    CameraMetadata static_meta;
    ret = recorder_.GetCameraCharacteristics(camera_id_, static_meta);
    ASSERT_TRUE(ret == NO_ERROR);

    meta_array.push_back(meta);

    ASSERT_TRUE (ValidateResFromProcessedSizes(static_meta, image_param.width,
        image_param.height) != false);

    ImageCaptureCb cb = [&] (uint32_t camera_id, uint32_t image_count,
                             BufferDescriptor buffer, BufferMeta meta) -> void
        { SnapshotCb(camera_id, image_count, buffer, meta);
          pending_count--; };

    ImageExtraParam xtraparam;

    ret = recorder_.ConfigImageCapture(camera_id_, image_param, xtraparam);
    ASSERT_TRUE(ret == NO_ERROR);


    /************************ Start Session ***********************************/

    ret = recorder_.StartSession(session_id);
    ASSERT_TRUE(ret == NO_ERROR);

    sleep(5);

    for (uint32_t i = 1; i <= iteration_count_; i++) {
      fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
      TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
        test_info_->name(), i);

      pending_count++;
      ret = recorder_.CaptureImage(camera_id_, 1, meta_array, cb);
      ASSERT_TRUE(ret == NO_ERROR);
      sleep(5);
    }

    ret = recorder_.CancelCaptureImage(camera_id_);
    ASSERT_TRUE(ret == NO_ERROR);

    // Check if all snapshot are done
    assert(pending_count == 0);

    sleep(1);

    /************************ Stop Session ************************************/

    ret = recorder_.StopSession(session_id, false);
    ASSERT_TRUE(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_yuv1);
    ASSERT_TRUE(ret == NO_ERROR);


    dump_bitstream_.CloseAll();
  }

  ret = recorder_.DeleteSession(session_id);
  ASSERT_TRUE(ret == NO_ERROR);

  ClearSessions();

  ret = recorder_.StopCamera(camera_id_);
  ASSERT_TRUE(ret == NO_ERROR);

  ret = DeInit();
  ASSERT_TRUE(ret == NO_ERROR);

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
    test_info_->test_case_name(), test_info_->name());
}

/*
* SessionWithTwoYUVTracksAndSnapshot: Test two video tracks and snapshot with
*                                     multiple resolutions.
*
* API test sequence:
*  - StartCamera
*   loop Start {
*   ------------------
*   - CaptureImage - JPEG
*   ------------------
*   } loop End
*  - StopCamera
*/
TEST_F(RecorderHal1GTest, SessionWithTwoYUVTracksAndSnapshot) {
  fprintf(stderr, "\n---------- Run Test %s.%s ------------\n",
    test_info_->test_case_name(), test_info_->name());

  auto ret = Init();
  ASSERT_TRUE(ret == NO_ERROR);

  ret = recorder_.StartCamera(camera_id_, 30);
  ASSERT_TRUE(ret == NO_ERROR);

  uint32_t video_track_id_yuv1 = 1;
  uint32_t video_track_id_yuv2 = 2;
  int32_t pending_count = 0;

  std::vector<std::pair<uint32_t, uint32_t>> test_res = {
    {640, 480},
    {720, 480},
    {1280, 720} };

  SessionCb session_status_cb = CreateSessionStatusCb();

  uint32_t session_id;
  ret = recorder_.CreateSession(session_status_cb, &session_id);
  ASSERT_TRUE(session_id > 0);
  ASSERT_TRUE(ret == NO_ERROR);

  for (auto& res : test_res) {
    uint32_t width = res.first;
    uint32_t height = res.second;
    QMMF_INFO("Test dim: %dx%d", width, height);

    if (dump_bitstream_.IsEnabled()) {
      StreamDumpInfo dumpinfo1 = { VideoFormat::kNV12, session_id,
        video_track_id_yuv1, width, height };
      ret = dump_bitstream_.SetUp(dumpinfo1);
      ASSERT_TRUE(ret == NO_ERROR);

      StreamDumpInfo dumpinfo2 = { VideoFormat::kNV12, session_id,
        video_track_id_yuv2, width, height };
      ret = dump_bitstream_.SetUp(dumpinfo2);
      ASSERT_TRUE(ret == NO_ERROR);
    }

    /************************ Create Track 1 ****************************/

    VideoTrackParam video_track_param {
      camera_id_, VideoFormat::kNV12, width, height, 30 };

    TrackCb video_track_cb;
    video_track_cb.data_cb = [&, session_id] (uint32_t track_id,
        std::vector<BufferDescriptor> buffers,
        std::vector<BufferMeta> metas) {
          VideoTrackYUVDataCb(session_id, track_id, buffers, metas); };

    video_track_cb.event_cb = [&](uint32_t track_id, EventType event_type,
                                  void *event_data, size_t event_data_size) {
        VideoTrackEventCb(track_id, event_type, event_data, event_data_size); };

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_yuv1,
      video_track_param, video_track_cb);
    ASSERT_TRUE(ret == NO_ERROR);

    std::vector<uint32_t> track_ids;
    track_ids.push_back(video_track_id_yuv1);

    /************************ Create Track 2 ****************************/

    ret = recorder_.CreateVideoTrack(session_id, video_track_id_yuv2,
      video_track_param, video_track_cb);
    ASSERT_TRUE(ret == NO_ERROR);

    track_ids.push_back(video_track_id_yuv2);
    sessions_.insert(std::make_pair(session_id, track_ids));

    /*********************** Configure Snapshot ******************************/

    ImageParam image_param{};
    image_param.width   = width;
    image_param.height  = height;
    image_param.format  = ImageFormat::kJPEG;
    image_param.quality = default_jpeg_quality_;

    std::vector<CameraMetadata> meta_array;
    camera_metadata_entry_t entry;
    CameraMetadata meta;

    ret = recorder_.GetDefaultCaptureParam(camera_id_, meta);
    ASSERT_TRUE(ret == NO_ERROR);

    CameraMetadata static_meta;
    ret = recorder_.GetCameraCharacteristics(camera_id_, static_meta);
    ASSERT_TRUE(ret == NO_ERROR);

    meta_array.push_back(meta);

    ASSERT_TRUE (ValidateResFromProcessedSizes(static_meta, image_param.width,
        image_param.height) != false);

    ImageCaptureCb cb = [&] (uint32_t camera_id, uint32_t image_count,
                             BufferDescriptor buffer, BufferMeta meta) -> void
        { SnapshotCb(camera_id, image_count, buffer, meta);
          pending_count--; };

    ImageExtraParam xtraparam;

    ret = recorder_.ConfigImageCapture(camera_id_, image_param, xtraparam);
    ASSERT_TRUE(ret == NO_ERROR);


    /************************ Start Session ***********************************/

    ret = recorder_.StartSession(session_id);
    ASSERT_TRUE(ret == NO_ERROR);

    sleep(5);


    for (uint32_t i = 1; i <= iteration_count_; i++) {
      fprintf(stderr, "test iteration = %d/%d\n", i, iteration_count_);
      TEST_INFO("%s: Running Test(%s) iteration = %d ", __func__,
        test_info_->name(), i);

      pending_count++;
      ret = recorder_.CaptureImage(camera_id_, 1, meta_array, cb);
      ASSERT_TRUE(ret == NO_ERROR);
      sleep(5);
    }

    ret = recorder_.CancelCaptureImage(camera_id_);
    ASSERT_TRUE(ret == NO_ERROR);

    // Check if all snapshot are done
    assert(pending_count == 0);

    sleep(1);

    /************************ Stop Session ************************************/

    ret = recorder_.StopSession(session_id, false);
    ASSERT_TRUE(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_yuv1);
    ASSERT_TRUE(ret == NO_ERROR);

    ret = recorder_.DeleteVideoTrack(session_id, video_track_id_yuv2);
    ASSERT_TRUE(ret == NO_ERROR);

    dump_bitstream_.CloseAll();
  }

  ret = recorder_.DeleteSession(session_id);
  ASSERT_TRUE(ret == NO_ERROR);

  ClearSessions();

  ret = recorder_.StopCamera(camera_id_);
  ASSERT_TRUE(ret == NO_ERROR);

  ret = DeInit();
  ASSERT_TRUE(ret == NO_ERROR);

  fprintf(stderr, "---------- Test Completed %s.%s ----------\n",
    test_info_->test_case_name(), test_info_->name());
}
