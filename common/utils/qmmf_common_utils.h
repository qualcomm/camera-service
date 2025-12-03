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
 * Changes from Qualcomm Technologies, Inc. are provided under the following license:
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#include <chrono>
#include <condition_variable>
#include <cmath>
#include <dlfcn.h>
#include <iomanip>
#include <list>
#include <map>
#include <set>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>

#ifdef HAVE_ANDROID_UTILS
#include <system/graphics.h>
#endif // HAVE_ANDROID_UTILS

#include <sys/mman.h>
#include <sys/time.h>

#ifndef CAMERA_HAL1_SUPPORT
#include <hardware/camera3.h>
#endif //CAMERA_HAL1_SUPPORT

#include <hardware/camera_common.h>

#include "qmmf-sdk/qmmf_camera_metadata.h"
#include "qmmf-sdk/qmmf_vendor_tag_descriptor.h"
#include "qmmf-sdk/qmmf_recorder_params.h"
#include "common/utils/qmmf_log.h"
#ifndef HAVE_BINDER
#include "common/utils/qmmf_color_format.h"
#endif // !HAVE_BINDER
#include "common/utils/qmmf_condition.h"
#include "qmmf_memory_interface.h"
#include "common/utils/qmmf_common_utils_defs.h"

namespace qmmf {

#ifdef HAVE_ANDROID_UTILS
using namespace android;
#endif // HAVE_ANDROID_UTILS
using namespace recorder;

typedef int32_t status_t;

class CameraModule {
private:

  static std::mutex lock_;

  static CameraModule *instance_;

  camera_module_t *camera_module_;

  int32_t status_;

  int32_t number_of_cameras_;

  vendor_tag_ops_t vendor_tag_ops_;
  std::shared_ptr<VendorTagDescriptor> vendor_tag_desc_;

  // Private Constructor
  CameraModule() : status_(-1) {}

  int32_t LoadCamModuleAndVendorTags() {
    int32_t status = hw_get_module(CAMERA_HARDWARE_MODULE_ID,
        (const hw_module_t **)&camera_module_);

    if (camera_module_->get_vendor_tag_ops) {
      vendor_tag_ops_ = vendor_tag_ops_t();
      camera_module_->get_vendor_tag_ops(&vendor_tag_ops_);

      status = VendorTagDescriptor::createDescriptorFromOps(&vendor_tag_ops_,
                                                          vendor_tag_desc_);

      if (0 != status) {
        QMMF_ERROR("%s: Could not generate descriptor from vendor tag operations,"
            "received error %s (%d). Camera clients will not be able to use"
            "vendor tags", __FUNCTION__, strerror(status), status);
        return status;
      }

      // Set the global descriptor to use with camera metadata
      status = VendorTagDescriptor::setAsGlobalVendorTagDescriptor(vendor_tag_desc_);

      if (0 != status) {
        QMMF_ERROR("%s: Could not set vendor tag descriptor, received error %s (%d). \n",
            __func__, strerror(-status), status);
        return status;
      }
    }

    return status;
  }

public:
  // Deleting the copy constructor to prevent copies
  CameraModule(const CameraModule& obj) = delete;

  // Static method to get the CameraModule instance
  static int32_t getInstance(camera_module_t **camera_module) {

    std::lock_guard<std::mutex> lock(lock_);

    if (instance_ == nullptr) {
      instance_ = new CameraModule();
    }

    if (instance_->status_ != 0 || instance_->camera_module_ == NULL) {
      instance_->status_ = instance_->LoadCamModuleAndVendorTags();
    }

    instance_->number_of_cameras_ = instance_->camera_module_->get_number_of_cameras();

    *camera_module = instance_->camera_module_;
    return instance_->status_;
  }

  static int32_t GetCameraInfo(uint32_t idx, CameraMetadata *info) {
    std::lock_guard<std::mutex> lock(lock_);
    if (NULL == info) {
      return -EINVAL;
    }

    if (idx >= instance_->number_of_cameras_) {
      return -EINVAL;
    }

    if (NULL == instance_->camera_module_) {
      return -ENODEV;
    }

    camera_info cam_info;
    int32_t res = instance_->camera_module_->get_camera_info(idx, &cam_info);
    if (0 != res) {
      QMMF_ERROR("%s: Error during camera static info query: %s!\n", __func__,
               strerror(res));
      return res;
    }

    *info = cam_info.static_camera_characteristics;

    return res;
  }

  static void release() {
    std::lock_guard<std::mutex> lock(lock_);

    if (instance_->camera_module_ != NULL) {
      VendorTagDescriptor::clearGlobalVendorTagDescriptor();
      dlclose(instance_->camera_module_->common.dso);
      instance_->camera_module_ = NULL;
    }
  }
};

struct ReprocEntry {
  StreamBuffer    buffer;
  CameraMetadata  result;
  int64_t         timestamp;
};

inline void get_qmmf_property(const char *key, char *value, const char *default_value) {
#ifdef HAVE_BINDER
    property_get(key, value, default_value);
#else
    qmmf_property_get(key, value, default_value);
#endif // HAVE_BINDER
}

inline void set_qmmf_property(const char *key, const char *value) {
#ifdef HAVE_BINDER
    property_set(key, value);
#else
    qmmf_property_set(key, value);
#endif // HAVE_BINDER
}
/** Property:
 *
 *  This class defines property operations
 **/
class Property {
 public:
  /** Get
   *    @property: property
   *    @default_value: default value
   *
   * Gets requested property value
   *
   * return: property value
   **/
  template <typename T>
  static T Get(std::string property, T default_value)  {
    T value = default_value;
    char prop_val[QMMF_PROP_VAL_MAX];

    std::stringstream s;
    s << default_value;

    get_qmmf_property(property.c_str(), prop_val, s.str().c_str());

    std::stringstream output(prop_val);
    output >> value;
    return value;
  }

  /** Set
   *    @property: property
   *    @value: value
   *
   * Sets requested property value
   *
   * return: nothing
   **/
  template <typename T>
  static void Set(std::string property, T value) {

    std::stringstream s;
    s << value;

    set_qmmf_property(property.c_str(), s.str().c_str());
  }
};

class Common {
 public:
  /** FromQmmfToHalFormat
   *
   * Translates QMMF format to HAL format
   *
   * return: HAL format
   **/
  static int32_t FromQmmfToHalFormat(const BufferFormat &format) {
    switch (format) {
      case BufferFormat::kRGB:
        return HAL_PIXEL_FORMAT_RGB_888;
        break;
      case BufferFormat::kBLOB:
        return HAL_PIXEL_FORMAT_BLOB;
        break;
      case BufferFormat::kNV12UBWC:
      case BufferFormat::kNV12:
      case BufferFormat::kP010:
      case BufferFormat::kTP10UBWC:
      case BufferFormat::kNV12UBWCFLEX:
      case BufferFormat::kNV12FLEX:
      case BufferFormat::kP010FLEX:
      case BufferFormat::kTP10UBWCFLEX:
        return HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED;
        break;
      case BufferFormat::kNV21:
      // TODO: kNV12HEIF should return HAL_PIXEL_FORMAT_NV12_HEIF.
      // For now, camera uses HAL_PIXEL_FORMAT_YCbCr_420_888,
      // HAL_DATASPACE_HEIF and allocFlags to give the correct outputs.
      // Update it when camera implements the HAL_PIXEL_FORMAT_NV12_HEIF.
      case BufferFormat::kNV12HEIF:
        return HAL_PIXEL_FORMAT_YCbCr_420_888;
        break;
      case BufferFormat::kNV16:
        return HAL_PIXEL_FORMAT_YCbCr_422_888;
        break;
      case BufferFormat::kYUY2:
        return HAL_PIXEL_FORMAT_YCBCR_422_I;
        break;
      case BufferFormat::kRAW8:
        return HAL_PIXEL_FORMAT_RAW8;
        break;
      case BufferFormat::kRAW10:
        return HAL_PIXEL_FORMAT_RAW10;
        break;
      case BufferFormat::kRAW12:
        return HAL_PIXEL_FORMAT_RAW12;
        break;
      case BufferFormat::kRAW16:
        return HAL_PIXEL_FORMAT_RAW16;
        break;
      case BufferFormat::kUYVY:
        return HAL_PIXEL_FORMAT_CbYCrY_422_I;
        break;
      default:
        /* Format not supported */
        QMMF_ERROR("%s: error: unsupported format %d (0x%x)", __func__,
          (int32_t) format,
          (int32_t) format);
        return -1;
    }
  }

  /** FromHalToQmmfFormat
   *
   * Translates HAL format to QMMF format
   *
   * return: QMMF format
   **/
  static BufferFormat FromHalToQmmfFormat(const int32_t &format) {
    switch (format) {
      case HAL_PIXEL_FORMAT_BLOB:
        return BufferFormat::kBLOB;
        break;
      case HAL_PIXEL_FORMAT_YCbCr_420_SP_VENUS_UBWC:
        return BufferFormat::kNV12UBWC;
        break;
      case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_2_BATCH:
      case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_4_BATCH:
      case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX_8_BATCH:
      case HAL_PIXEL_FORMAT_NV12_UBWC_FLEX:
        return BufferFormat::kNV12UBWCFLEX;
        break;
      case HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED:
        return BufferFormat::kNV12;
        break;
      case HAL_PIXEL_FORMAT_NV12_FLEX_2_BATCH:
      case HAL_PIXEL_FORMAT_NV12_FLEX_4_BATCH:
      case HAL_PIXEL_FORMAT_NV12_FLEX_8_BATCH:
      case HAL_PIXEL_FORMAT_NV12_FLEX:
        return BufferFormat::kNV12FLEX;
      case HAL_PIXEL_FORMAT_YCbCr_422_I_10BIT:
        return BufferFormat::kP010;
        break;
      case HAL_PIXEL_FORMAT_P010_FLEX_2_BATCH:
      case HAL_PIXEL_FORMAT_P010_FLEX_4_BATCH:
      case HAL_PIXEL_FORMAT_P010_FLEX_8_BATCH:
      case HAL_PIXEL_FORMAT_P010_FLEX:
        return BufferFormat::kP010FLEX;
      case HAL_PIXEL_FORMAT_YCbCr_420_TP10_UBWC:
        return BufferFormat::kTP10UBWC;
        break;
      case HAL_PIXEL_FORMAT_TP10_UBWC_FLEX_2_BATCH:
      case HAL_PIXEL_FORMAT_TP10_UBWC_FLEX_4_BATCH:
      case HAL_PIXEL_FORMAT_TP10_UBWC_FLEX_8_BATCH:
      case HAL_PIXEL_FORMAT_TP10_UBWC_FLEX:
        return BufferFormat::kTP10UBWCFLEX;
      case HAL_PIXEL_FORMAT_YCbCr_420_888:
        return BufferFormat::kNV21;
        break;
      case HAL_PIXEL_FORMAT_YCbCr_422_888:
        return BufferFormat::kNV16;
        break;
      case HAL_PIXEL_FORMAT_YCBCR_422_I:
        return BufferFormat::kYUY2;
        break;
      case HAL_PIXEL_FORMAT_RAW8:
        return BufferFormat::kRAW8;
        break;
      case HAL_PIXEL_FORMAT_RAW10:
        return BufferFormat::kRAW10;
        break;
      case HAL_PIXEL_FORMAT_RAW12:
        return BufferFormat::kRAW12;
        break;
      case HAL_PIXEL_FORMAT_RAW16:
        return BufferFormat::kRAW16;
        break;
      case HAL_PIXEL_FORMAT_CbYCrY_422_I:
        return BufferFormat::kUYVY;
        break;
      default:
        /* Format not supported */
        QMMF_ERROR("%s: error: unsupported format %d (0x%x)", __func__, format,
          (int32_t) format);
        return BufferFormat::kUnsupported;
    }
  }

  /** FromImageToQmmfFormat
   *
   * Translates Image capture format to QMMF format
   *
   * return: QMMF format
   **/
  static BufferFormat FromImageToQmmfFormat(const ImageFormat& format) {
    switch (format) {
      case ImageFormat::kJPEG:
        return BufferFormat::kBLOB;
        break;
      case ImageFormat::kNV12:
        return BufferFormat::kNV12;
        break;
      case ImageFormat::kNV12HEIF:
        return BufferFormat::kNV12HEIF;
        break;
      case ImageFormat::kNV12UBWC:
        return BufferFormat::kNV12UBWC;
        break;
      case ImageFormat::kNV21:
        return BufferFormat::kNV21;
        break;
      case ImageFormat::kP010:
        return BufferFormat::kP010;
        break;
      case ImageFormat::kTP10UBWC:
        return BufferFormat::kTP10UBWC;
        break;
      case ImageFormat::kBayerRDI8BIT:
        return BufferFormat::kRAW8;
        break;
      case ImageFormat::kBayerRDI10BIT:
        return BufferFormat::kRAW10;
        break;
      case ImageFormat::kBayerRDI12BIT:
        return BufferFormat::kRAW12;
        break;
      case ImageFormat::kBayerRDI16BIT:
        return BufferFormat::kRAW16;
        break;
      default:
        /* Format not supported */
        QMMF_ERROR("%s: error: unsupported format %d (0x%x)", __func__,
          (int32_t) format,
          (int32_t) format);
        return BufferFormat::kUnsupported;
    }
  }

  /** FromVideoToQmmfFormat
   *
   * Translates Video capture format to QMMF format
   *
   * return: QMMF format
   **/
  static BufferFormat FromVideoToQmmfFormat(const VideoFormat& format) {
    switch (format) {
      case VideoFormat::kNV12:
        return BufferFormat::kNV12;
        break;
      case VideoFormat::kNV12FLEX:
        return BufferFormat::kNV12FLEX;
      case VideoFormat::kNV12UBWC:
        return BufferFormat::kNV12UBWC;
        break;
      case VideoFormat::kNV12UBWCFLEX:
        return BufferFormat::kNV12UBWCFLEX;
        break;
      case VideoFormat::kP010:
        return BufferFormat::kP010;
        break;
      case VideoFormat::kP010FLEX:
        return BufferFormat::kP010FLEX;
      case VideoFormat::kTP10UBWC:
        return BufferFormat::kTP10UBWC;
        break;
      case VideoFormat::kTP10UBWCFLEX:
        return BufferFormat::kTP10UBWCFLEX;
      case VideoFormat::kNV16:
        return BufferFormat::kNV16;
        break;
      case VideoFormat::kJPEG:
        return BufferFormat::kBLOB;
        break;
      case VideoFormat::kYUY2:
        return BufferFormat::kYUY2;
        break;
      case VideoFormat::kRGB:
        return BufferFormat::kRGB;
        break;
      case VideoFormat::kBayerRDI8BIT:
        return BufferFormat::kRAW8;
        break;
      case VideoFormat::kBayerRDI10BIT:
        return BufferFormat::kRAW10;
        break;
      case VideoFormat::kBayerRDI12BIT:
        return BufferFormat::kRAW12;
        break;
      case VideoFormat::kBayerRDI16BIT:
        return BufferFormat::kRAW16;
        break;
      case VideoFormat::kUYVY:
        return BufferFormat::kUYVY;
        break;
      default:
        /* Format not supported */
        QMMF_ERROR("%s: error: unsupported format %d (0x%x)", __func__,
          (int32_t) format,
          (int32_t) format);
        return BufferFormat::kUnsupported;
    }
  }

  /** ValidateStreamFormat
   *
   * Validates whether buffer format is available
   *
   * return: true if available
   **/
  static bool ValidateStreamFormat(const CameraMetadata& meta,
                                   const BufferFormat format,
                                   bool input = false) {
    bool is_supported = false;
    int32_t hal_format = FromQmmfToHalFormat(format);
#ifdef CAM_ARCH_V2

#if defined(CAMERA_HAL_API_VERSION) && (CAMERA_HAL_API_VERSION >= 0x0307)
    if (meta.exists(
          ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_MAXIMUM_RESOLUTION)) {
      auto entry = meta.find(
          ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_MAXIMUM_RESOLUTION);
      for (uint32_t i = 0 ; i < entry.count; i += 4) {
        if (hal_format == entry.data.i32[i] &&
            input == (entry.data.i32[i + 3] ==
              ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_INPUT)) {
          is_supported = true;
          break;
        }
      }
    }

    if (is_supported == false) {
#endif

    if (meta.exists(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS)) {
      auto entry = meta.find(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS);
      for (uint32_t i = 0 ; i < entry.count; i += 4) {
        if (hal_format == entry.data.i32[i] &&
            input == (entry.data.i32[i + 3] ==
              ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_INPUT)) {
          is_supported = true;
          break;
        }
      }
    } else {
      QMMF_ERROR("%s: Metadata ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS"
                 " not available", __func__);
      return false;
    }

#if defined(CAMERA_HAL_API_VERSION) && (CAMERA_HAL_API_VERSION >= 0x0307)
    }
#endif

#else
    assert(input == false);
    if (meta.exists(ANDROID_SCALER_AVAILABLE_FORMATS)) {
      auto entry = meta.find(ANDROID_SCALER_AVAILABLE_FORMATS);
      for (uint32_t i = 0; i < entry.count; i++) {
        if (entry.data.i32[i] == hal_format) {
          is_supported = true;
          break;
        }
      }
    } else {
      QMMF_ERROR("%s: Metadata ANDROID_SCALER_AVAILABLE_FORMATS"
                 " not available", __func__);
      return false;
    }
#endif
    return is_supported;
  }

  /** ValidateInputFormat
   *
   * Validates whether buffer format is available
   *
   * return: true if available
   **/
  static bool ValidateInputFormat(const CameraMetadata& meta,
                                  const BufferFormat in_format,
                                  const BufferFormat out_format) {
    bool is_supported = false;
#ifdef CAM_ARCH_V2
    is_supported = ValidateStreamFormat(meta, in_format, true) &&
                   ValidateStreamFormat(meta, out_format, false);
#else
    if (meta.exists(ANDROID_SCALER_AVAILABLE_INPUT_OUTPUT_FORMATS_MAP)) {
      int32_t in_hal_format = FromQmmfToHalFormat(in_format);
      int32_t out_hal_format = FromQmmfToHalFormat(out_format);
      auto entry = meta.find(ANDROID_SCALER_AVAILABLE_INPUT_OUTPUT_FORMATS_MAP);
      if (entry.count != 0) {
        size_t idx = 0;
        int32_t input_format = 0, num_output_formats = 0;

        while (idx < entry.count) {
          // Increment the idx with the number of output formats from previous entry.
          idx += num_output_formats;
          input_format       = entry.data.i32[idx++];
          num_output_formats = entry.data.i32[idx++];
          if (input_format != in_hal_format) {
            // Different input formats, skip map entry.
            continue;
          }
          for (auto i = idx; i < (idx + num_output_formats); ++i) {
            if (out_hal_format == entry.data.i32[i]) {
              is_supported = true;
              break;
            }
          }
          // Didn't find supported format mapping, no point to continue.
          break;
        }
      }
    }
#endif
    return is_supported;
  }

  static const int32_t JPEG_BUFFER_SIZE_MIN =
      256 * 1024 + sizeof(camera3_jpeg_blob);

  static int32_t CalculateBlobSize(CameraMetadata &device_info, int32_t width, int32_t height) {
    int32_t maxJpegBufferSize, maxJpegSizeWidth, maxJpegSizeHeight, res, jpegDebugDataSize;
    int32_t maxWidth, maxHeight;
    int32_t maxUHRWidth, maxUHRHeight;
    int32_t ret;
    camera_metadata_entry entry;

    maxWidth = maxHeight = maxUHRWidth = maxUHRHeight = res = jpegDebugDataSize = 0;

    entry = device_info.find(ANDROID_JPEG_MAX_SIZE);
    if (entry.count == 0) {
      QMMF_ERROR(
          "%s: Camera: Can't find maximum JPEG size in static"
          " metadata!",
          __func__);
      return -EINVAL;
    }
    maxJpegBufferSize = entry.data.i32[0];
    assert(JPEG_BUFFER_SIZE_MIN < maxJpegBufferSize);

    QMMF_INFO("%s: default maxJpegBufferSize=%d", __func__, maxJpegBufferSize);

#if defined(CAMERA_HAL_API_VERSION) && (CAMERA_HAL_API_VERSION >= 0x0307)
    if (device_info.exists(
        ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_MAXIMUM_RESOLUTION)) {
      auto entry = device_info.find(
          ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_MAXIMUM_RESOLUTION);
      for (uint32_t i = 0; i < entry.count; i += 4) {
        if (HAL_PIXEL_FORMAT_BLOB == entry.data.i32[i] &&
            ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT ==
            entry.data.i32[i+3]) {
          int32_t w = entry.data.i32[i + 1];
          int32_t h = entry.data.i32[i + 2];

          if (w * h > maxUHRWidth * maxUHRHeight) {
            maxUHRWidth = w;
            maxUHRHeight = h;
          }
        }
      }
    }

    //Calculate debuging buffer size of jpeg.
    uint32_t tag = 0;

    std::shared_ptr<VendorTagDescriptor> vTags =
        VendorTagDescriptor::getGlobalVendorTagDescriptor();

    CameraMetadata::getTagFromName(
        "org.quic.camera.jpegdebugdata.size",vTags.get(), &tag);

    if (device_info.exists(tag)) {
      auto entry  = device_info.find(tag);
      jpegDebugDataSize = entry.data.i32[0];
    }

    QMMF_INFO("%s: jpegDebugDataSize=%d",
        __func__, jpegDebugDataSize);
#endif

    if (device_info.exists(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS)) {
      auto entry = device_info.find(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS);
      for (uint32_t i = 0; i < entry.count; i += 4) {
        if (HAL_PIXEL_FORMAT_BLOB == entry.data.i32[i] &&
            ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT ==
            entry.data.i32[i+3]) {
          int32_t w = entry.data.i32[i + 1];
          int32_t h = entry.data.i32[i + 2];

          if (w * h > maxWidth * maxHeight) {
            maxWidth = w;
            maxHeight = h;
          }
        }
      }
    } else {
        QMMF_ERROR("%s: Can't find available stream configs", __func__);
        return -EINVAL;
    }

    QMMF_INFO("%s: maxUHRWidth=%d maxUHRHeight=%d maxWidth=%d maxHeight=%d",
        __func__, maxUHRWidth, maxUHRHeight, maxWidth, maxHeight);

    // if input width * height is larger than default max width * height,
    // it means ultra hight resolution has been selected, to make scaleFactor
    // calculation work correctly, we need to update buffersize, max width and
    // max height accordingly
    if ((maxUHRWidth != 0) && ((width * height) > (maxWidth * maxHeight))) {
      maxJpegSizeWidth = maxUHRWidth;
      maxJpegSizeHeight = maxUHRHeight;
      maxJpegBufferSize =
          ((maxUHRWidth * 1.0f * maxUHRHeight) / (maxWidth * maxHeight)) *
          maxJpegBufferSize;
    } else {
      maxJpegSizeWidth = maxWidth;
      maxJpegSizeHeight = maxHeight;
    }

    QMMF_INFO("%s: input width=%d height=%d"
        " maxJpegBufferSize=%d maxJpegSizeWidth=%d maxJpegSizeHeight=%d",
        __func__, width, height,
        maxJpegBufferSize, maxJpegSizeWidth, maxJpegSizeHeight);

    assert(JPEG_BUFFER_SIZE_MIN < maxJpegBufferSize);

    // Calculate final jpeg buffer size for the given resolution.
    float scaleFactor =
        ((float)(width * height)) / (maxJpegSizeWidth * maxJpegSizeHeight);
    ssize_t jpegBufferSize =
        scaleFactor * (maxJpegBufferSize - JPEG_BUFFER_SIZE_MIN
        - jpegDebugDataSize) + JPEG_BUFFER_SIZE_MIN + jpegDebugDataSize;

    if (jpegBufferSize > maxJpegBufferSize) {
      jpegBufferSize = maxJpegBufferSize;
    }

    QMMF_INFO("%s: scaleFactor=%f jpegBufferSize=%d",
        __func__, scaleFactor, jpegBufferSize);

    return jpegBufferSize;
  }

 private:

  /** ValidateResFromStreamConfigs
  *
  * Validates whether input resolution is available in
  * stream configurations.
  *
  * return: true if available
  **/
  static bool ValidateResFromStreamConfigs(const CameraMetadata& meta,
                                           const uint32_t width,
                                           const uint32_t height) {
    bool is_supported = false;

#if defined(CAMERA_HAL_API_VERSION) && (CAMERA_HAL_API_VERSION >= 0x0307)
    if (meta.exists(
          ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_MAXIMUM_RESOLUTION)) {
      auto entry = meta.find(
          ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_MAXIMUM_RESOLUTION);
      for (uint32_t i = 0 ; i < entry.count; i += 4) {
        if (HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED == entry.data.i32[i]) {
          if (ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT ==
              entry.data.i32[i+3]) {
            if (width == static_cast<uint32_t>(entry.data.i32[i+1])
                && height == static_cast<uint32_t>(entry.data.i32[i+2])) {
              is_supported = true;
              break;
            }
          }
        }
      }
    }

    if (is_supported == false) {
#endif

    if (meta.exists(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS)) {
      auto entry = meta.find(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS);
      for (uint32_t i = 0 ; i < entry.count; i += 4) {
        if (HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED == entry.data.i32[i]) {
          if (ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT ==
              entry.data.i32[i+3]) {
            if (width == static_cast<uint32_t>(entry.data.i32[i+1])
                && height == static_cast<uint32_t>(entry.data.i32[i+2])) {
              is_supported = true;
              break;
            }
          }
        }
      }
    } else {
      QMMF_ERROR("%s: Metadata ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS"
                 " not available", __func__);
      return false;
    }

#if defined(CAMERA_HAL_API_VERSION) && (CAMERA_HAL_API_VERSION >= 0x0307)
    }
#endif

    return is_supported;
  }

  /** GetMaxResFromStreamConfigs
  *
  * Searches for maximum supported resolution in stream configurations.
  *
  * return: true if available
  **/
  static bool GetMaxResFromStreamConfigs(const CameraMetadata& meta,
                                         uint32_t &width,
                                         uint32_t &height) {
    bool found = false;
    width = 0;
    height = 0;

#if defined(CAMERA_HAL_API_VERSION) && (CAMERA_HAL_API_VERSION >= 0x0307)
    if (meta.exists(
          ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_MAXIMUM_RESOLUTION)) {
      auto entry = meta.find(
          ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_MAXIMUM_RESOLUTION);
      for (uint32_t i = 0; i < entry.count; i += 4) {
        if (HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED == entry.data.i32[i] &&
            ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT ==
              entry.data.i32[i+3]) {
          if (width < static_cast<uint32_t>(entry.data.i32[i + 1]) &&
              height < static_cast<uint32_t>(entry.data.i32[i + 2])) {
            width = static_cast<uint32_t>(entry.data.i32[i + 1]);
            height = static_cast<uint32_t>(entry.data.i32[i + 2]);
            found = true;
          }
        }
      }
    }
#endif

    if (meta.exists(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS)) {
      auto entry = meta.find(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS);
      for (uint32_t i = 0; i < entry.count; i += 4) {
        if (HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED == entry.data.i32[i] &&
            ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT ==
              entry.data.i32[i+3]) {
          if (width < static_cast<uint32_t>(entry.data.i32[i + 1]) &&
              height < static_cast<uint32_t>(entry.data.i32[i + 2])) {
            width = static_cast<uint32_t>(entry.data.i32[i + 1]);
            height = static_cast<uint32_t>(entry.data.i32[i + 2]);
            found = true;
          }
        }
      }
    } else {
      QMMF_ERROR("%s: Metadata ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS"
                 " not available", __func__);
      return false;
    }

    return found;
  }

  /** GetMinResFromStreamConfigs
  *
  * Searches for minimum supported resolution in stream configurations.
  *
  * return: true if available
  **/
  static bool GetMinResFromStreamConfigs(const CameraMetadata& meta,
                                         uint32_t &width,
                                         uint32_t &height) {
    bool found = false;
    width = 0xFFFF;
    height = 0xFFFF;

#if defined(CAMERA_HAL_API_VERSION) && (CAMERA_HAL_API_VERSION >= 0x0307)
    if (meta.exists(
          ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_MAXIMUM_RESOLUTION)) {
      auto entry = meta.find(
          ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_MAXIMUM_RESOLUTION);
      for (uint32_t i = 0; i < entry.count; i += 4) {
        if (HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED == entry.data.i32[i] &&
            ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT ==
              entry.data.i32[i+3]) {
          if (width > static_cast<uint32_t>(entry.data.i32[i + 1]) &&
              height > static_cast<uint32_t>(entry.data.i32[i + 2])) {
            width = static_cast<uint32_t>(entry.data.i32[i + 1]);
            height = static_cast<uint32_t>(entry.data.i32[i + 2]);
            found = true;
          }
        }
      }
    }
#endif

    if (meta.exists(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS)) {
      auto entry = meta.find(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS);
      for (uint32_t i = 0; i < entry.count; i += 4) {
        if (HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED == entry.data.i32[i] &&
            ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT ==
              entry.data.i32[i+3]) {
          if (width > static_cast<uint32_t>(entry.data.i32[i + 1]) &&
              height > static_cast<uint32_t>(entry.data.i32[i + 2])) {
            width = static_cast<uint32_t>(entry.data.i32[i + 1]);
            height = static_cast<uint32_t>(entry.data.i32[i + 2]);
            found = true;
          }
        }
      }
    } else {
      QMMF_ERROR("%s: Metadata ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS"
                 " not available", __func__);
      return false;
    }

    return found;
  }

 public:

  /** ValidateResFromProcessedSizes
   *
   * Validates whether input resolution is available in
   * processed sizes.
   *
   * return: true if available
   **/
  static bool ValidateResFromProcessedSizes(const CameraMetadata& meta,
                                            const uint32_t width,
                                            const uint32_t height) {
    bool is_supported = false;
#ifdef CAM_ARCH_V2
    is_supported = ValidateResFromStreamConfigs(meta, width, height);
#else
    if (meta.exists(ANDROID_SCALER_AVAILABLE_PROCESSED_SIZES)) {
      auto entry = meta.find(ANDROID_SCALER_AVAILABLE_PROCESSED_SIZES);
      for (uint32_t i = 0 ; i < entry.count; i += 2) {
        if(width == static_cast<uint32_t>(entry.data.i32[i+0]) &&
          height == static_cast<uint32_t>(entry.data.i32[i+1])) {
          is_supported = true;
          break;
        }
      }
    } else {
      QMMF_ERROR("%s: Metadata ANDROID_SCALER_AVAILABLE_PROCESSED_SIZES"
                 " not available", __func__);
      return false;
    }
#endif
    return is_supported;
  }

  /** ValidateResFromJpegSizes
   *
   * Validates whether input resolution is available in jpeg sizes.
   * Since ANDROID_SCALER_AVAILABLE_JPEG_SIZES tag is not available
   * in static meta, jpeg size needs to be validated from available
   * stream configuration, by filtering the resolutions with
   * HAL_PIXEL_FORMAT_BLOB.
   *
   * return: true if available
   **/
  static bool ValidateResFromJpegSizes(const CameraMetadata& meta,
                                       const uint32_t width,
                                       const uint32_t height) {
    bool is_supported = false;

#if defined(CAMERA_HAL_API_VERSION) && (CAMERA_HAL_API_VERSION >= 0x0307)
    if (meta.exists(
          ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_MAXIMUM_RESOLUTION)) {
      auto entry = meta.find(
          ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_MAXIMUM_RESOLUTION);
      for (uint32_t i = 0 ; i < entry.count; i += 4) {
        if (HAL_PIXEL_FORMAT_BLOB == entry.data.i32[i]) {
          if (ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT ==
              entry.data.i32[i+3]) {
            if (width == static_cast<uint32_t>(entry.data.i32[i+1])
                && height == static_cast<uint32_t>(entry.data.i32[i+2])) {
              is_supported = true;
              break;
            }
          }
        }
      }
    }

    if (is_supported == false) {
#endif

    if (meta.exists(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS)) {
      auto entry = meta.find(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS);
      for (uint32_t i = 0 ; i < entry.count; i += 4) {
        if (HAL_PIXEL_FORMAT_BLOB == entry.data.i32[i]) {
          if (ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT ==
              entry.data.i32[i+3]) {
            if (width == static_cast<uint32_t>(entry.data.i32[i+1])
                && height == static_cast<uint32_t>(entry.data.i32[i+2])) {
              is_supported = true;
              break;
            }
          }
        }
      }
    } else {
      QMMF_ERROR("%s: Metadata ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS"
                 " not available", __func__);
      return false;
    }

#if defined(CAMERA_HAL_API_VERSION) && (CAMERA_HAL_API_VERSION >= 0x0307)
    }
#endif

    return is_supported;
  }

  /** ValidateResFromRawSizes
   *
   * Validates whether input resolution is available in
   * raw sizes.
   *
   * return: true if available
   **/
  static bool ValidateResFromRawSizes(const CameraMetadata& meta,
                                      const uint32_t width,
                                      const uint32_t height) {
    bool is_supported = false;
#ifdef CAM_ARCH_V2

#if defined(CAMERA_HAL_API_VERSION) && (CAMERA_HAL_API_VERSION >= 0x0307)
    if (meta.exists(
          ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_MAXIMUM_RESOLUTION)) {
      auto entry = meta.find(
          ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_MAXIMUM_RESOLUTION);
      for (uint32_t i = 0 ; i < entry.count; i += 4) {
        if (HAL_PIXEL_FORMAT_RAW8 == entry.data.i32[i] ||
            HAL_PIXEL_FORMAT_RAW10 == entry.data.i32[i] ||
            HAL_PIXEL_FORMAT_RAW12 == entry.data.i32[i] ||
            HAL_PIXEL_FORMAT_RAW16 == entry.data.i32[i] ) {
          if (ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT ==
              entry.data.i32[i+3]) {
            uint32_t w = static_cast<uint32_t>(entry.data.i32[i+1]);
            uint32_t h = static_cast<uint32_t>(entry.data.i32[i+2]);
            QMMF_DEBUG("%s: Supported width: %d, height: %d", __func__, w, h);
            if (width == w && height == h) {
              is_supported = true;
              break;
            }
          }
        }
      }
    }

    if (is_supported == false) {
#endif

    if (meta.exists(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS)) {
      auto entry = meta.find(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS);
      for (uint32_t i = 0 ; i < entry.count; i += 4) {
        if (HAL_PIXEL_FORMAT_RAW8 == entry.data.i32[i] ||
            HAL_PIXEL_FORMAT_RAW10 == entry.data.i32[i] ||
            HAL_PIXEL_FORMAT_RAW12 == entry.data.i32[i] ||
            HAL_PIXEL_FORMAT_RAW16 == entry.data.i32[i] ) {
          if (ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT ==
              entry.data.i32[i+3]) {
            uint32_t w = static_cast<uint32_t>(entry.data.i32[i+1]);
            uint32_t h = static_cast<uint32_t>(entry.data.i32[i+2]);
            QMMF_DEBUG("%s: Supported width: %d, height: %d", __func__, w, h);
            if (width == w && height == h) {
              is_supported = true;
              break;
            }
          }
        }
      }
    } else {
      QMMF_ERROR("%s: Metadata ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS"
                 " not available", __func__);
      return false;
    }

#if defined(CAMERA_HAL_API_VERSION) && (CAMERA_HAL_API_VERSION >= 0x0307)
    }
#endif

#else
    if (meta.exists(ANDROID_SCALER_AVAILABLE_RAW_SIZES)) {
      auto entry = meta.find(ANDROID_SCALER_AVAILABLE_RAW_SIZES);
      for (uint32_t i = 0 ; i < entry.count; i += 2) {
        if(width == static_cast<uint32_t>(entry.data.i32[i+0]) &&
          height == static_cast<uint32_t>(entry.data.i32[i+1])) {
          is_supported = true;
          break;
        }
      }
    } else {
      QMMF_ERROR("%s: Metadata ANDROID_SCALER_AVAILABLE_RAW_SIZES"
                 " not available", __func__);
      return false;
    }
#endif
    return is_supported;
  }

  /** ValidateResolution
   *
   * Validates whether input resolution is available.
   *
   * return: true if available
   **/
  static bool ValidateResolution(const CameraMetadata& meta,
                                  const BufferFormat format,
                                  const uint32_t width,
                                  const uint32_t height) {

    bool is_supported = false;
    switch (format) {
      case BufferFormat::kRAW8:
      case BufferFormat::kRAW10:
      case BufferFormat::kRAW12:
      case BufferFormat::kRAW16:
        is_supported = ValidateResFromRawSizes(meta, width, height);
        break;

      case BufferFormat::kNV12:
      case BufferFormat::kNV12UBWC:
      case BufferFormat::kNV12HEIF:
      case BufferFormat::kNV21:
      case BufferFormat::kNV16:
      case BufferFormat::kYUY2:
      case BufferFormat::kUYVY:
      case BufferFormat::kP010:
      case BufferFormat::kTP10UBWC:
      case BufferFormat::kRGB:
        is_supported = ValidateResFromProcessedSizes(meta, width, height);
        break;

      case BufferFormat::kBLOB:
        is_supported = ValidateResFromJpegSizes(meta, width, height);
        break;

      default:
        QMMF_ERROR("%s: Format(%d) not supported!", __func__,
          (int32_t) format);
        return -EINVAL;
    }
    return is_supported;
  }

  /** GetMaxSupportedCameraRes
   *
   * Searches for maximum supported camera resolution.
   *
   * return: true if available
   **/
  static bool GetMaxSupportedCameraRes(const CameraMetadata& meta,
      uint32_t &width, uint32_t &height,
      const BufferFormat format = BufferFormat::kRAW10) {
    bool found = false;
    width = 0;
    height = 0;
    camera_metadata_ro_entry entry;
#ifdef CAM_ARCH_V2

#if defined(CAMERA_HAL_API_VERSION) && (CAMERA_HAL_API_VERSION >= 0x0307)
    if (meta.exists(
          ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_MAXIMUM_RESOLUTION)) {
      entry = meta.find(
          ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_MAXIMUM_RESOLUTION);
      for (uint32_t i = 0; i < entry.count; i += 4) {
        if (HAL_PIXEL_FORMAT_RAW10 == entry.data.i32[i] &&
            ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT ==
              entry.data.i32[i+3]) {
          if (width < static_cast<uint32_t>(entry.data.i32[i + 1]) &&
              height < static_cast<uint32_t>(entry.data.i32[i + 2])) {
            width = static_cast<uint32_t>(entry.data.i32[i + 1]);
            height = static_cast<uint32_t>(entry.data.i32[i + 2]);
            found = true;
          }
        }
      }
      QMMF_INFO("%s: width=%d, height=%d", __func__, width, height);
    }
#endif

    if (meta.exists(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS)) {
      entry = meta.find(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS);
      for (uint32_t i = 0; i < entry.count; i += 4) {
        if (HAL_PIXEL_FORMAT_RAW10 == entry.data.i32[i] &&
            ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT ==
              entry.data.i32[i+3]) {
          if (width < static_cast<uint32_t>(entry.data.i32[i + 1]) &&
              height < static_cast<uint32_t>(entry.data.i32[i + 2])) {
            width = static_cast<uint32_t>(entry.data.i32[i + 1]);
            height = static_cast<uint32_t>(entry.data.i32[i + 2]);
            found = true;
          }
        }
      }
      QMMF_INFO("%s: width=%d, height=%d", __func__, width, height);
    } else {
      QMMF_ERROR("%s: Metadata ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS"
                 " not available", __func__);
      return false;
    }
#else
    int32_t hal_format = Common::FromQmmfToHalFormat(format);
    if (HAL_PIXEL_FORMAT_RAW8  == hal_format ||
        HAL_PIXEL_FORMAT_RAW10 == hal_format ||
        HAL_PIXEL_FORMAT_RAW12 == hal_format ||
        HAL_PIXEL_FORMAT_RAW16 == hal_format) {
      if (!meta.exists(ANDROID_SCALER_AVAILABLE_RAW_SIZES)) {
        QMMF_ERROR("%s: Metadata ANDROID_SCALER_AVAILABLE_RAW_SIZES"
                   " not available", __func__);
        return false;
      }
      entry = meta.find(ANDROID_SCALER_AVAILABLE_RAW_SIZES);
    } else {
      if (!meta.exists(ANDROID_SCALER_AVAILABLE_PROCESSED_SIZES)) {
        QMMF_ERROR("%s: Metadata ANDROID_SCALER_AVAILABLE_PROCESSED_SIZES"
                   " not available", __func__);
        return false;
      }
      entry = meta.find(ANDROID_SCALER_AVAILABLE_PROCESSED_SIZES);
    }

    for (uint32_t i = 0 ; i < entry.count; i += 2) {
      if (width < static_cast<uint32_t>(entry.data.i32[i + 0]) &&
          height < static_cast<uint32_t>(entry.data.i32[i + 1])) {
        width = static_cast<uint32_t>(entry.data.i32[i + 0]);
        height = static_cast<uint32_t>(entry.data.i32[i + 1]);
        found = true;
      }
    }
#endif
    return found;
  }

  /** GetMinSupportedCameraRes
   *
   * Searches for minumum supported camera resolution.
   *
   * return: true if available
   **/
  static bool GetMinSupportedCameraRes(const CameraMetadata& meta,
                                      uint32_t &width,
                                      uint32_t &height) {
    bool found = false;
    width = 0xFFFF;
    height = 0xFFFF;
#ifdef CAM_ARCH_V2
    found = GetMinResFromStreamConfigs(meta, width, height);
#else
    camera_metadata_ro_entry entry;
    if (!meta.exists(ANDROID_SCALER_AVAILABLE_PROCESSED_SIZES)) {
      QMMF_ERROR("%s: Metadata ANDROID_SCALER_AVAILABLE_PROCESSED_SIZES"
                 " not available", __func__);
      return false;
    }

    entry = meta.find(ANDROID_SCALER_AVAILABLE_PROCESSED_SIZES);
    for (uint32_t i = 0 ; i < entry.count; i += 2) {
      if (width > static_cast<uint32_t>(entry.data.i32[i + 0]) &&
          height > static_cast<uint32_t>(entry.data.i32[i + 1])) {
        width = static_cast<uint32_t>(entry.data.i32[i + 0]);
        height = static_cast<uint32_t>(entry.data.i32[i + 1]);
        found = true;
      }
    }
#endif
    return found;
  }

  /** GetSupportedCameraFormats
   *
   * Return supported camera formats.
   *
   * return: true if available
   **/
  static bool GetSupportedCameraFormats(const CameraMetadata& meta,
                                        std::set<BufferFormat> &formats,
                                        bool input = false) {

    bool found = false;
#ifdef CAM_ARCH_V2

#if defined(CAMERA_HAL_API_VERSION) && (CAMERA_HAL_API_VERSION >= 0x0307)
    if (meta.exists(
          ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_MAXIMUM_RESOLUTION)) {
      auto entry_max = meta.find(
          ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_MAXIMUM_RESOLUTION);
      for (uint32_t i = 0 ; i < entry_max.count; i += 4) {
        if (input == (entry_max.data.i32[i + 3] ==
                      ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_INPUT)) {
          auto format = FromHalToQmmfFormat(entry_max.data.i32[i]);
          if (format != BufferFormat::kUnsupported && !formats.count(format)) {
            formats.insert(format);
            found = true;
          }
        }
      }
    }
#endif

    assert(meta.exists(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS));
    auto entry = meta.find(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS);
    for (uint32_t i = 0 ; i < entry.count; i += 4) {
      if (input == (entry.data.i32[i + 3] ==
                    ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_INPUT)) {
        auto format = FromHalToQmmfFormat(entry.data.i32[i]);
        if (format != BufferFormat::kUnsupported && !formats.count(format)) {
          formats.insert(format);
          found = true;
        }
      }
    }
#else
    assert(meta.exists(ANDROID_SCALER_AVAILABLE_FORMATS));
    auto entry = meta.find(ANDROID_SCALER_AVAILABLE_FORMATS);
    for (uint32_t i = 0; i < entry.count; i++) {
      auto format = FromHalToQmmfFormat(entry.data.i32[i]);
      if (format != BufferFormat::kUnsupported && !formats.count(format)) {
        formats.insert(format);
        found = true;
      }
    }
#endif
    return found;
  }
};  // class Common

};  // namespace qmmf.

#ifdef QCAMERA3_TAG_LOCAL_COPY
namespace qcamera {
// With the new camera backend design coming in Android-O ,
// vendor tags names, querying mechanism and file location
// have changed. The following code is being added to unblock
// compilation, till the new tags and their querying mechanism
// are being implemented.
#define QCAMERA3_EXPOSURE_METER_AVAILABLE_MODES  0x80080001
#define QCAMERA3_EXPOSURE_METER  0x80080000
#define QCAMERA3_USE_SATURATION  0x80070000
#define QCAMERA3_SELECT_PRIORITY  0x80060001
#define QCAMERA3_USE_ISO_EXP_PRIORITY  0x80060000
#define QCAMERA3_VENDOR_STREAM_CONFIGURATION_RAW_ONLY_MODE  0x8000
#define QCAMERA3_VENDOR_STREAM_CONFIGURATION_PP_DISABLED_MODE  0x8001
#define QCAMERA3_DUALCAM_SYNCHRONIZED_REQUEST  0x800b0005
#define QCAMERA3_DUALCAM_LINK_IS_MAIN  0x800b0001
#define QCAMERA3_DUALCAM_LINK_RELATED_CAMERA_ID  0x800b0002
#define QCAMERA3_DUALCAM_LINK_ENABLE  0x800b0000
#define QCAMERA3_DUALCAM_LINK_CAMERA_ROLE_BAYER  0x1
#define QCAMERA3_DUALCAM_LINK_CAMERA_ROLE  0x800b0003
#define QCAMERA3_DUALCAM_LINK_3A_360_CAMERA  0x3
#define QCAMERA3_DUALCAM_LINK_3A_SYNC_MODE  0x800b0004
#define QCAMERA3_TARGET_LUMA  0x801e0000
#define QCAMERA3_CURRENT_LUMA  0x801e0001
#define QCAMERA3_LUMA_RANGE  0x801e0002
#define QCAMERA3_AVAILABLE_VIDEO_HDR_MODES  0x800f0001
#define QCAMERA3_VIDEO_HDR_MODE  0x800f0000
#define QCAMERA3_VIDEO_HDR_MODE_ON  0x1
#define QCAMERA3_VIDEO_HDR_MODE_OFF  0x0
#define QCAMERA3_LCAC_PROCESSING_ENABLE  0x801f0000
#define QCAMERA3_IR_MODE  0x80100000
#define QCAMERA3_IR_AVAILABLE_MODES  0x80100001
#define QCAMERA3_IR_MODE_OFF  0x0
#define QCAMERA3_IR_MODE_ON  0x1
#define QCAMERA3_AVAILABLE_BINNING_CORRECTION_MODES  0x80160001
#define QCAMERA3_BINNING_CORRECTION_MODE_ON  0x1
#define QCAMERA3_BINNING_CORRECTION_MODE_OFF  0x0
#define QCAMERA3_BINNING_CORRECTION_MODE  0x80160000
#define QCAMERA3_SHARPNESS_STRENGTH  0x80140000
#define QCAMERA3_SHARPNESS_RANGE  0x80140001
#define QCAMERA3_WNR_RANGE  0x80180000
#define QCAMERA3_TNR_INTENSITY  0x801a0000
#define QCAMERA3_TNR_MOTION_DETECTION_SENSITIVITY  0x801a0001
#define QCAMERA3_TNR_TUNING_RANGE  0x801a0002
#define QCAMERA3_HISTOGRAM_STATS  0x80150003
#define QCAMERA3_HISTOGRAM_BUCKETS  0x80150001
#define QCAMERA3_HISTOGRAM_MODE  0x80150000
#define QCAMERA3_HISTOGRAM_MODE_OFF  0x0
#define QCAMERA3_HISTOGRAM_MODE_ON  0x1
#define QCAMERA3_EXPOSURE_DATA_ON  0x1
#define QCAMERA3_EXPOSURE_DATA_OFF  0x0
#define QCAMERA3_AWB_ROI_COLOR  0x801d0000
#define QCAMERA3_EXPOSURE_DATA_ENABLE  0x80190000
#define QCAMERA3_EXPOSURE_DATA_REGION_H_NUM  0x80190001
#define QCAMERA3_EXPOSURE_DATA_REGION_V_NUM  0x80190002
#define QCAMERA3_EXPOSURE_DATA_REGION_PIXEL_CNT  0x80190003
#define QCAMERA3_EXPOSURE_DATA_REGION_HEIGHT  0x80190004
#define QCAMERA3_EXPOSURE_DATA_REGION_WIDTH  0x80190005
#define QCAMERA3_EXPOSURE_DATA_R_SUM  0x80190006
#define QCAMERA3_EXPOSURE_DATA_B_SUM  0x80190007
#define QCAMERA3_EXPOSURE_DATA_GR_SUM  0x80190008
#define QCAMERA3_EXPOSURE_DATA_GB_SUM  0x80190009
#define QCAMERA3_EXPOSURE_DATA_R_NUM  0x8019000a
#define QCAMERA3_EXPOSURE_DATA_B_NUM  0x8019000b
#define QCAMERA3_EXPOSURE_DATA_GR_NUM  0x8019000c
#define QCAMERA3_EXPOSURE_DATA_GB_NUM  0x8019000d
#define QCAMERA3_CURRENT_LUX_IDX  0x801e0003
#define QCAMERA3_CDS_MODE  0x80010000
#define QCAMERA3_IS_H_MARGIN_CFG 0x80010001
#define QCAMERA3_IS_V_MARGIN_CFG 0x80010002

// QCAMERA3_ISO_EXP_PRIORITY
typedef enum qcamera3_ext_iso_mode {
    QCAMERA3_ISO_MODE_AUTO,
    QCAMERA3_ISO_MODE_DEBLUR,
    QCAMERA3_ISO_MODE_100,
    QCAMERA3_ISO_MODE_200,
    QCAMERA3_ISO_MODE_400,
    QCAMERA3_ISO_MODE_800,
    QCAMERA3_ISO_MODE_1600,
    QCAMERA3_ISO_MODE_3200,
} qcamera3_ext_iso_mode_t;
};  // namespace qcamera.
#endif  // QCAMERA3_TAG_LOCAL_COPY
