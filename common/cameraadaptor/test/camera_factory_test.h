/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef CAMERA_FACTORY_TEST_H_
#define CAMERA_FACTORY_TEST_H_

#include <dirent.h>
#include <fcntl.h>
#include <qmmf_camera3_device_client.h>

#include <functional>
#include <iostream>
#include <sstream>

using namespace qmmf;
using namespace qmmf::cameraadaptor;
using namespace std;

typedef enum Camera_State {
  STATE_ON,
  STATE_OFF,
} CameraState;

class Camera {
 public:
  Camera(uint32_t cam_id);
  ~Camera();

  void SetUp();
  void TearDown();

  CameraState GetState() { return state_; }
  uint32_t GetCameraId() { return camera_idx_; }
  std::string GetColorFilter() { return color_filter_; }

  int32_t PowerOn();
  int32_t PowerOff();

 private:
  void StreamCb(StreamBuffer buffer) {}
  void ErrorCb(CameraErrorCode errorCode, const CaptureResultExtras &) {}
  void IdleCb() {}
  void ShutterCb(const CaptureResultExtras &, int64_t) {}
  void PreparedCb(int) {}
  void ResultCb(const CaptureResult &result) {}
  std::string GetColorFilterArrangement();

  uint32_t camera_idx_;
  int32_t stream_idx_;
  CameraClientCallbacks client_cb_;
  std::unique_ptr<Camera3DeviceClient> device_client_;
  CameraState state_;
  std::string color_filter_;
};

class CameraFactoryTest {
 public:
  CameraFactoryTest();
  ~CameraFactoryTest();
  void MainMenu();
  bool CameraMenu();
  void PrintPowerOptions();
  bool WaitForInput(std::string &input);
  void PrintCameraOptions();
  void StartThread();
  void Join();
  uint32_t GetNumsOfCamera() { return number_of_cameras_; }

 private:
  std::map<uint32_t, std::shared_ptr<Camera>> camera_list_;
  uint32_t number_of_cameras_;
  std::thread *menu_thread_;
};
#endif  // CAMERA_FACTORY_TEST_H_
