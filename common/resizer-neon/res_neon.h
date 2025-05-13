/*
* Copyright (c) 2018, The Linux Foundation. All rights reserved.
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

#include <condition_variable>
#include <thread>
#include <mutex>
#include "common/utils/qmmf_condition.h"

namespace qmmf {

namespace neonresizer {

/** ResnStatus:
 *    @kRESN_ERR_HW_PLATFORM_NOT_AVAILABLE: HW Platform not available
 *    @kRESN_ERR_HW_DEVICE_NOT_AVAILABLE: HW device not available
 *    @kRESN_ERR_GET_DEVICE_INFO: failed to get device information
 *    @kRESN_FILE_DOESNT_EXIST: designated file doesn't exist
 *    @kRESN_UNABLE_TO_DETERMINE_FILESIZE: unable to determine the
 *      size of the designated file
 *    @kRESN_ERR_FAILED_TO_LOAD_IMGPROC_KERNEL: failed to enqueue
 *      image processing kernel
 *    @kRESN_ERR_SET_IMGPROC_KERNEL_ARG: error at setting image processing
 *      kernel argument
 *    @kRESN_FAILED_TO_CREATE_IMGPROC_KERNEL: error at creating
 *     image processing kernel from program
 *    @kRESN_UNABLE_TO_BUILD_IMGPROC_PROGRAM: error at compiling
 *      image processing kernel
 *    @kRESN_ERR_FAILED_TO_FLUSH_CQ: failed to flush command queue
 *    @kRESN_UNABLE_TO_READ_FILE: failed to read designated file
 *    @kRESN_WRONG_STRIDE_OUTBUF: output buffer stride and argument
 *      stride don't match
 *    @kRESN_WRONG_STRIDE_INBUF: Input buffer stride and argument
 *      stride don't match
 *    @kRESN_WRONG_WIDTH_OUTBUF: Error, input buffer value does not match
 *    @kRESN_WRONG_WIDTH_INBUF: Error, output buffer value does not match
 *    @kRESN_ERR_UNABLE_TO_MAP_OUTBUF: error at mapping output buffer
 *    @kRESN_ERR_UNABLE_TO_MAP_INBUF: : error at mapping input buffer
 *    @kRESN_ERR_INVALID_INPUT: invalid input
 *    @kRESN_ERR_INPUT_BUFFER_NOT_MAPPED: input buffer not mapped
 *    @kRESN_ERR_OUTPUT_BUFFER_NOT_MAPPED: output buffer not mapped
 *    @kRESN_FAILED_TO_CREATE_PROG_WITH_BIN: error at creating image processing
 *      program from bin file
 *    @kRESN_FAILED_TO_CREATE_PROG_WITH_SRC: error at creating compiling image
 *      processing src
 *    @kRESN_FAILED_TO_CREATE_CONTEXT: error at creating image processing context
 *    @kRESN_FAILED_TO_CREATE_CQUEUE: error at creating image processing
 *      command queue
 *    @kRESN_NOT_INITIALIZED: error, resizer library is not initialized
 *    @kRESN_ERR_NO_MEMORY: error, not enough heap memory
 *    @kRESN_FAIL: generic error
 *    @kRESN_UNABLE_TO_OPEN_FILE: error, failed to open desinated file
 *    @kRESN_UNABLE_TO_WRITE_FILE: error, failed to write designated file
 *    @kRESN_FAILED_TO_GET_BLD_LOG_SIZE: failed to get size of build log
 *    @kRESN_FAILED_TO_GET_BLD_LOG: failed to get pointer to build log
 *    @kRESN_FAILED_TO_GET_DEV_ID_FROM_PROG_INFO: Failed to get image processing
 *      device info from program ID
 *    @kRESN_PROGRAM_NOT_BUILT_FOR_DEV: program built for different device
 *    @kRESN_FAILED_TO_GET_PROG_BIN_SIZE: could not get size of image processing
 *      program binary
 *    @kRESN_FAILED_TO_GET_PROG_BINNARY: could not get image processing
 *      program binary
 *    @kRESN_SUCCESS: success
 *
 *  This enum defines the return status from resizer neon library interface
 */
enum class ResnStatus {
  kRESN_ERR_HW_PLATFORM_NOT_AVAILABLE = -1024,
  kRESN_ERR_HW_DEVICE_NOT_AVAILABLE,          // 1023
  kRESN_ERR_GET_DEVICE_INFO,                  // 1022
  kRESN_FILE_DOESNT_EXIST,                    // 1021
  kRESN_UNABLE_TO_DETERMINE_FILESIZE,         // 1020
  kRESN_ERR_FAILED_TO_LOAD_IMGPROC_KERNEL,    // 1019
  kRESN_ERR_SET_IMGPROC_KERNEL_ARG,           // 1018
  kRESN_FAILED_TO_CREATE_IMGPROC_KERNEL,      // 1017
  kRESN_UNABLE_TO_BUILD_IMGPROC_PROGRAM,      // 1016
  kRESN_ERR_FAILED_TO_FLUSH_CQ,               // 1015
  kRESN_UNABLE_TO_READ_FILE,                  // 1014
  kRESN_WRONG_STRIDE_OUTBUF,                  // 1013
  kRESN_WRONG_STRIDE_INBUF,                   // 1012
  kRESN_WRONG_WIDTH_OUTBUF,                   // 1011
  kRESN_WRONG_WIDTH_INBUF,                    // 1010
  kRESN_ERR_UNABLE_TO_MAP_OUTBUF,             // 1009
  kRESN_ERR_UNABLE_TO_MAP_INBUF,              // 1008
  kRESN_ERR_INVALID_INPUT,                    // 1007
  kRESN_ERR_INPUT_BUFFER_NOT_MAPPED,          // 1006
  kRESN_ERR_OUTPUT_BUFFER_NOT_MAPPED,         // 1005
  kRESN_FAILED_TO_CREATE_PROG_WITH_BIN,       // 1004
  kRESN_FAILED_TO_CREATE_PROG_WITH_SRC,       // 1003
  kRESN_FAILED_TO_CREATE_CONTEXT,             // 1002
  kRESN_FAILED_TO_CREATE_CQUEUE,              // 1001
  kRESN_NOT_INITIALIZED,                      // 1000
  kRESN_ERR_NO_MEMORY,                        // 999
  kRESN_FAIL,                                 // 998
  kRESN_UNABLE_TO_OPEN_FILE,                  // 997
  kRESN_UNABLE_TO_WRITE_FILE,                 // 996
  kRESN_ERR_SET_PERF_HINT,                    // 995
  kRESN_FAILED_TO_GET_BLD_LOG_SIZE,           // 994
  kRESN_FAILED_TO_GET_BLD_LOG,                // 993
  kRESN_FAILED_TO_GET_DEV_ID_FROM_PROG_INFO,  // 992
  kRESN_PROGRAM_NOT_BUILT_FOR_DEV,            // 991
  kRESN_FAILED_TO_GET_PROG_BIN_SIZE,          // 990
  kRESN_FAILED_TO_GET_PROG_BINNARY,           // 989
  kRESN_FAILED_TO_CREATE_WORK_THREADS,        // 988
  kRESN_SUCCESS = 0,
};

enum class ResMethod {
  kRES_BILINEAR_V_SKIP = 0,
  kRES_BILINEAR = 1,
  kRES_NUMBER
};

struct NeonImgArgs {
  uint32_t width;
  uint32_t height;
  uint32_t stride;
  uint32_t src_stride;
  uint32_t line_start;
  uint32_t line_end;
  uint32_t w_coef;
  uint32_t h_coef;
  uint8_t* src_luma;
  uint8_t* src_chroma;
  uint8_t* dst_luma;
  uint8_t* dst_chroma;
  uint16_t* y_coefs;
  uint16_t* uv_coefs;
  uint16_t* input_offsets;
  uint16_t* ver_offsets;
  uint16_t* ver_coefs;
  // Resizer method
  ResMethod res_method;
};

class IResnCtx {

 public:
  virtual ResnStatus CreateYPass(const uint32_t id) = 0;
  virtual ResnStatus CreateUVPass(const uint32_t id) = 0;
  virtual void DeleteYUVPass(const uint32_t id) = 0;
  virtual void setImg(const uint32_t id, NeonImgArgs& img) = 0;
  virtual void Start(const uint32_t id) = 0;
  virtual void WaitReady(const uint32_t id) = 0;
  virtual uint16_t* getInputOffsets() = 0;
  virtual uint16_t* getYCoefs() = 0;
  virtual uint16_t* getUVCoefs() = 0;
  virtual uint16_t* getVerOffsets() = 0;
  virtual uint16_t* getVerCoefs() = 0;
  virtual void update_coefs(unsigned int src_width,
                            unsigned int src_height,
                            unsigned int dst_width,
                            unsigned int dst_height) = 0;

  virtual NeonImgArgs& getImg(const uint32_t id) = 0;

  virtual ~IResnCtx() {};
};

struct ChromatixResnType {
  int enable;
};

struct Resn {
  // Tuning data parameters
  ChromatixResnType* resn_tuning;

  // Input data pointers
  unsigned char* src_luma;
  unsigned char* src_chroma;

  // Output data pointers
  unsigned char* dst_luma;
  unsigned char* dst_chroma;

  // Input buffer dimensions
  unsigned int src_width;
  unsigned int src_height;
  unsigned int src_stride;

  // Output buffer dimensions
  unsigned int dst_width;
  unsigned int dst_height;
  unsigned int dst_stride;
  // Resizer method
  ResMethod res_method;
};

class NeonCore {

 public:

  NeonCore();

  ~NeonCore();

  const char* resn_get_version();

  ResnStatus resn_init();

  void resn_deinit();

  ResnStatus resn_process(Resn* params);

 private:

  std::unique_ptr<IResnCtx> ctx_;
  uint32_t num_threads_;
  static const char version_[];
};

} // namespace neonresizer
} // namespace qmmf
