/*
* Copyright (c) 2016, 2018, 2021, The Linux Foundation. All rights reserved.
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

/*! @file qmmf_buffer.h
*/

#pragma once

#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

namespace qmmf {

#define MAX_PLANE 8

enum class BufferFormat : uint32_t {
  kRGB,
  kNV12,
  kNV12FLEX,
  kNV12UBWC,
  kNV12UBWCFLEX,
  kNV12HEIF,
  kNV21,
  kNV16,
  kYUY2,
  kUYVY,
  kP010,
  kP010FLEX,
  kTP10UBWC,
  kTP10UBWCFLEX,
  kBLOB,
  kRAW8,
  kRAW10,
  kRAW12,
  kRAW16,
  kUnsupported,
};

//  BufferFlags will be used to determine the type of encoded frame

/**
 *
 *  BufferFlags will be used to determine the type of encoded frame
 */
enum class BufferFlags : uint64_t {
  kNone = 0,
};

struct PlaneInfo {
  uint32_t stride;
  uint32_t scanline;
  uint32_t width;
  uint32_t height;
  uint32_t offset; /**< offset in bytes */ //offset in bytes
  uint32_t size;   /**< size of plane */ // size of plane

  ::std::string ToString() const {
    ::std::stringstream stream;
    stream << "stride[" << stride << "] ";
    stream << "scanline[" << scanline << "] ";
    stream << "width[" << width << "] ";
    stream << "height[" << height << "] ";
    stream << "offset[" << offset << "] ";
    stream << "size[" << size << "] ";
    return stream.str();
  }
};

struct BufferDescriptor {
  void*    data;
  int32_t  fd;
  uint32_t img_id;
  uint32_t buf_id;
  uint32_t size;
  uint32_t capacity;
  uint32_t offset;
  uint64_t timestamp;
  uint64_t seqnum;
  uint64_t flags;

  ::std::string ToString() const {
    ::std::stringstream stream;
    stream << "data[" << data << "] ";
    stream << "fd[" << fd << "] ";
    stream << "img_id[" << img_id << "] ";
    stream << "buf_id[" << buf_id << "] ";
    stream << "size[" << size << "] ";
    stream << "capacity[" << capacity << "] ";
    stream << "offset[" << offset << "] ";
    stream << "timestamp[" << timestamp << "] ";
    stream << "seqnum[" << seqnum << "] ";
    stream << "flags[" << ::std::setbase(16) << flags
           << ::std::setbase(10) << "]";
    return stream.str();
  }
};

/// @brief Meta data associated with BufferDescriptor.
struct BufferMeta {
  /// Buffer format.
  BufferFormat format;
  /// frame count in one buffer
  uint8_t      n_frames;
  /// Number of buffer planes.
  uint32_t     n_planes;
  /// Buffer plane information.
  PlaneInfo    planes[MAX_PLANE];

  ::std::string ToString() const {
    ::std::stringstream stream;
    stream << "format["
           << static_cast<::std::underlying_type<BufferFormat>::type>(format)
           << "] ";
    stream << "n_frames" << n_frames << "] ";
    stream << "n_planes[" << n_planes << "] ";
    stream << "planes[";
    for (uint32_t idx = 0; idx < n_planes; ++idx)
      stream << "[" << planes[idx].ToString() << "], ";
    return stream.str();
  }
};

}; // namespace qmmf
