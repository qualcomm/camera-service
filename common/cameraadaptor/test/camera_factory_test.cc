/*
 * Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "camera_factory_test.h"

#define PREVIEW_WIDTH 1280
#define PREVIEW_HEIGHT 720
#define STREAM_BUFFER_COUNT 4

#define CAMERA_MENU "\n----------------- Camera Menu -----------------\n"
#define CAMERA_HEADER "Option | Cam ID | Sensor Type | Power State\n"
#define POWER_MENU "\n----------------- Power Menu -----------------\n"
#define POWER_HEADER "Option | Power State\n"
#define DASH "\n==============================================\n"

using namespace qcamera;

Camera::Camera(uint32_t cam_id)
    : camera_idx_(cam_id),
      stream_idx_(-1),
      device_client_(nullptr),
      state_(STATE_OFF) {
  memset(&client_cb_, 0, sizeof(client_cb_));
  client_cb_.errorCb = [&](CameraErrorCode errorCode,
                           const CaptureResultExtras &extras) {
    ErrorCb(errorCode, extras);
  };
  client_cb_.idleCb = [&]() { IdleCb(); };
  client_cb_.peparedCb = [&](int id) { PreparedCb(id); };
  client_cb_.shutterCb = [&](const CaptureResultExtras &extras, int64_t ts) {
    ShutterCb(extras, ts);
  };
  client_cb_.resultCb = [&](const CaptureResult &result) { ResultCb(result); };
  SetUp();
  color_filter_ = GetColorFilterArrangement();
}

Camera::~Camera() {}

void Camera::TearDown() {
  if (NULL != device_client_.get()) {
    device_client_.reset();
  }
  stream_idx_ = -1;
  state_ = STATE_OFF;
}

void Camera::SetUp() {
  if (device_client_ == nullptr) {
    device_client_ = std::make_unique<Camera3DeviceClient>(client_cb_);
    if (NULL == device_client_.get()) {
      std::cerr << "error occured at device3 client creation \n";
    }

    auto ret = device_client_->Initialize();
    if (ret != 0) {
      std::cerr << "error occured at device3 client init \n";
    }

    ret = device_client_->OpenCamera(camera_idx_);
    if (ret != 0) {
      std::cerr << "open camera failed \n";
    }
  }
}

std::string Camera::GetColorFilterArrangement() {
  CameraMetadata staticInfo;
  std::string msg;
  auto ret = device_client_->GetCameraInfo(camera_idx_, &staticInfo);
  if (ret != 0) {
    std::cerr << "GetCameraInfo failed with error code: " << ret << std::endl;
    return "";
  }

  if (staticInfo.exists(ANDROID_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT)) {
    int32_t value =
        staticInfo.find(ANDROID_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT)
            .data.i32[0];
    switch (value) {
      case ANDROID_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT_RGGB:
        msg = "RGGB";
        break;
      case ANDROID_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT_GRBG:
        msg = "GRBG";
        break;
      case ANDROID_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT_GBRG:
        msg = "GBRG";
        break;
      case ANDROID_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT_BGGR:
        msg = "BGGR";
        break;
      case ANDROID_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT_RGB:
        msg = "RGB";
        break;
      case ANDROID_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT_MONO:
        msg = "MONO";
        break;
      case ANDROID_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT_NIR:
        msg = "NIR";
        break;
      default:
        msg = "Unknown";
        break;
    }
  } else {
    std::cerr
        << "ANDROID_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT tag does not exist"
        << std::endl;
  }

  return msg;
}

int32_t Camera::PowerOn() {
  CameraStreamParameters streamParams;

  auto ret = device_client_->BeginConfigure();
  if (0 != ret) {
    return ret;
  }

  memset(&streamParams, 0, sizeof(streamParams));
  streamParams.bufferCount = STREAM_BUFFER_COUNT;
  streamParams.format = HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
  streamParams.width = PREVIEW_WIDTH;
  streamParams.height = PREVIEW_HEIGHT;
  streamParams.allocFlags = IMemAllocUsage::kHwFb;
  streamParams.cb = [&](StreamBuffer buffer) {};

  ret = device_client_->CreateStream(streamParams);
  if (0 != ret) {
    return ret;
  }
  stream_idx_ = ret;

  ret = device_client_->EndConfigure();
  if (0 != ret) {
    return ret;
  }

  state_ = STATE_ON;
  return ret;
}

int32_t Camera::PowerOff() {
  auto ret = device_client_->DeleteStream(stream_idx_, true);
  if (0 != ret) {
    return ret;
  }

  TearDown();

  return ret;
}
CameraFactoryTest::CameraFactoryTest() {
  CameraClientCallbacks client_cb_;
  auto device_client = std::make_shared<Camera3DeviceClient>(client_cb_);
  auto ret = device_client->Initialize();
  number_of_cameras_ = device_client->GetNumberOfCameras();
}

CameraFactoryTest::~CameraFactoryTest() { camera_list_.clear(); }

void CameraFactoryTest::MainMenu() {
  bool active = true;

  while (active) {
    active = CameraMenu();
  }
}

bool CameraFactoryTest::CameraMenu() {
  std::string cam_input;
  std::string power_input;
  bool active = true;

  camera_list_.clear();
  for (uint32_t i = 0; i < number_of_cameras_; ++i) {
    camera_list_.emplace(i, std::make_shared<Camera>(i));
    camera_list_[i]->SetUp();
  }

  while (active) {
    PrintCameraOptions();
    if (!WaitForInput(cam_input)) continue;

    if (cam_input == "q") {
      std::cout << "\nQuit pressed!!\n";
      return false;
    }

    int cam_index = std::stoi(cam_input);
    if (cam_index < 0 || cam_index >= number_of_cameras_) {
      std::cout << "\nInvalid option. Please choose a valid camera number or "
                   "'q' to quit.\n";
      continue;
    }

    while (true) {
      PrintPowerOptions();
      if (!WaitForInput(power_input)) continue;

      if (power_input == "b") {
        break;
      } else if (power_input == "0") {
        auto &camera = camera_list_[cam_index];
        if (camera->GetState() == STATE_OFF) {
          camera->SetUp();
          auto ret = camera->PowerOn();
          if (ret != 0)
            std::cout << "\nCamera " << cam_input
                      << " is failed in Power On\n\n";
          else
            std::cout << "\nCamera " << cam_input << " is Powered On \n\n";
        } else {
          std::cout << "\nCamera " << cam_input << " is already Powered On\n\n";
        }
        break;
      } else if (power_input == "1") {
        auto &camera = camera_list_[cam_index];
        if (camera->GetState() == STATE_ON) {
          auto ret = camera->PowerOff();
          if (ret != 0)
            std::cout << "\nCamera " << cam_input
                      << " is failed in Power Off\n\n";
          else
            std::cout << "\nCamera " << cam_input << " is Powered Off\n\n";
        } else {
          std::cout << "\nCamera " << cam_input
                    << " is already Powered Off\n\n";
        }
        break;
      } else {
        std::cout << "\nInvalid option. Please choose a valid power option (0, "
                     "1, or b).\n";
      }
    }
  }
  return active;
}

void CameraFactoryTest::PrintPowerOptions() {
  std::cout << POWER_MENU;
  std::cout << DASH;
  std::cout << POWER_HEADER;
  std::cout << DASH;
  std::cout << "(0)    | Power ON\n";
  std::cout << "(1)    | Power OFF\n";
  std::cout << "(b)    | Back\n";
  std::cout << DASH;
  std::cout << "\nChoose an option: ";
}

bool CameraFactoryTest::WaitForInput(std::string &input) {
  std::getline(std::cin, input);
  return !input.empty();
}

void CameraFactoryTest::PrintCameraOptions() {
  std::cout << CAMERA_MENU;
  std::cout << DASH;
  std::cout << CAMERA_HEADER;
  std::cout << DASH;

  for (const auto &it : camera_list_) {
    std::cout << "(" << it.first << ") | " << it.second->GetCameraId() << " | "
              << it.second->GetColorFilter() << " | "
              << (it.second->GetState() == STATE_ON ? "POWERED ON"
                                                    : "POWERED OFF")
              << "\n";
  }

  std::cout << "(q) Quit\n";
  std::cout << DASH;
  std::cout << "\nChoose an option: ";
}

void CameraFactoryTest::StartThread() {
  menu_thread_ = new std::thread(&CameraFactoryTest::MainMenu, this);
}

void CameraFactoryTest::Join() {
  if (!menu_thread_->joinable()) {
    std::cerr << "ERROR: Failed to create menu thread!\n";
    return;
  }
  menu_thread_->join();
  delete menu_thread_;
  menu_thread_ = nullptr;
}

int main() {
  auto test = CameraFactoryTest();
  std::cout << DASH
            << "Number of cameras connected : " << test.GetNumsOfCamera()
            << DASH;
  if (test.GetNumsOfCamera() != 0) {
    test.StartThread();
    test.Join();
  }
  return 0;
}
