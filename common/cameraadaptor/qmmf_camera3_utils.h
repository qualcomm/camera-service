/*
* Copyright (c) 2016, 2020, The Linux Foundation. All rights reserved.
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

#ifndef CAMERA3UTILS_H_
#define CAMERA3UTILS_H_

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

#include <pthread.h>
#include <stdint.h>

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

inline const char* kCameraHardwareLibName = "libcamera_hardware";
using HwGetModuleFn = int (*)(const char* id, const struct hw_module_t** module_out);

namespace cameraadaptor {

void cond_init(pthread_cond_t *cond);

int32_t cond_wait_relative(pthread_cond_t *cond, pthread_mutex_t *mutex,
                           uint64_t reltime);

int compare(const int32_t *left, const int32_t *right);

}  // namespace cameraadaptor ends here

#ifdef HAVE_ANDROID_UTILS
using namespace android;
#endif // HAVE_ANDROID_UTILS
using namespace recorder;

typedef int32_t status_t;

class CameraModule {
private:

  static std::mutex lock_;

  static CameraModule *instance_;

  void *handle_;

  HwGetModuleFn get_module_fn_;

  int32_t LoadHwGetModule(HwGetModuleFn *out_fn);

  void UnloadHwGetModule();

  camera_module_t *camera_module_;

  int32_t status_;

  int32_t number_of_cameras_;

  vendor_tag_ops_t vendor_tag_ops_;
  std::shared_ptr<VendorTagDescriptor> vendor_tag_desc_;

  // Private Constructor
  CameraModule();

  int32_t LoadCamModuleAndVendorTags();

public:
  // Deleting the copy constructor to prevent copies
  CameraModule(const CameraModule& obj) = delete;

  // Static method to get the CameraModule instance
  static int32_t getInstance(camera_module_t **camera_module);

  static int32_t GetCameraInfo(uint32_t idx, CameraMetadata *info);

  static void release();
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

class Common {
 public:
  /** FromQmmfToHalFormat
   *
   * Translates QMMF format to HAL format
   *
   * return: HAL format
   **/
  static int32_t FromQmmfToHalFormat(const BufferFormat &format);

  /** FromHalToQmmfFormat
   *
   * Translates HAL format to QMMF format
   *
   * return: QMMF format
   **/
  static BufferFormat FromHalToQmmfFormat(const int32_t &format);

  /** FromImageToQmmfFormat
   *
   * Translates Image capture format to QMMF format
   *
   * return: QMMF format
   **/
  static BufferFormat FromImageToQmmfFormat(const ImageFormat& format);

  /** FromVideoToQmmfFormat
   *
   * Translates Video capture format to QMMF format
   *
   * return: QMMF format
   **/
  static BufferFormat FromVideoToQmmfFormat(const VideoFormat& format);

  /** ValidateStreamFormat
   *
   * Validates whether buffer format is available
   *
   * return: true if available
   **/
  static bool ValidateStreamFormat(const CameraMetadata& meta,
                                   const BufferFormat format,
                                   bool input = false);

  /** ValidateInputFormat
   *
   * Validates whether buffer format is available
   *
   * return: true if available
   **/
  static bool ValidateInputFormat(const CameraMetadata& meta,
                                  const BufferFormat in_format,
                                  const BufferFormat out_format);

  static const int32_t JPEG_BUFFER_SIZE_MIN =
      256 * 1024 + sizeof(camera3_jpeg_blob);

  static int32_t CalculateBlobSize(CameraMetadata &device_info, int32_t width, int32_t height);

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
                                          const uint32_t height);

  /** GetMaxResFromStreamConfigs
  *
  * Searches for maximum supported resolution in stream configurations.
  *
  * return: true if available
  **/
  static bool GetMaxResFromStreamConfigs(const CameraMetadata& meta,
                                         uint32_t &width,
                                         uint32_t &height);

  /** GetMinResFromStreamConfigs
  *
  * Searches for minimum supported resolution in stream configurations.
  *
  * return: true if available
  **/
  static bool GetMinResFromStreamConfigs(const CameraMetadata& meta,
                                         uint32_t &width,
                                         uint32_t &height);

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
                                           const uint32_t height);

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
                                       const uint32_t height);

  /** ValidateResFromRawSizes
   *
   * Validates whether input resolution is available in
   * raw sizes.
   *
   * return: true if available
   **/
  static bool ValidateResFromRawSizes(const CameraMetadata& meta,
                                      const uint32_t width,
                                      const uint32_t height);

  /** ValidateResolution
   *
   * Validates whether input resolution is available.
   *
   * return: true if available
   **/
  static bool ValidateResolution(const CameraMetadata& meta,
                                  const BufferFormat format,
                                  const uint32_t width,
                                  const uint32_t height);

  /** GetMaxSupportedCameraRes
   *
   * Searches for maximum supported camera resolution.
   *
   * return: true if available
   **/
  static bool GetMaxSupportedCameraRes(const CameraMetadata& meta,
      uint32_t &width, uint32_t &height,
      const BufferFormat format = BufferFormat::kRAW10);

  /** GetMinSupportedCameraRes
   *
   * Searches for minumum supported camera resolution.
   *
   * return: true if available
   **/
  static bool GetMinSupportedCameraRes(const CameraMetadata& meta,
                                      uint32_t &width,
                                      uint32_t &height);

};  // class Common

}  // namespace qmmf ends here

#endif /* CAMERA3UTILS_H_ */

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
