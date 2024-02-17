/*
 * Copyright (c) 2016-2017, The Linux Foundation. All rights reserved.
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

/* ​​​​​Changes from Qualcomm Innovation Center, Inc. are provided under the following license:
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#pragma once

#ifdef HAVE_ANDROID_UTILS
#include <utils/Log.h>
#include <cutils/properties.h>
#else
#include <log.h>
#include "properties.h"
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#endif

#undef assert
// Invalid ptr operation just to get backtraces into logcat.
// TODO: Use "libunwind" to get proper traces.
#define assert(condition) do { \
  if (!(condition)) { \
    int *p = 0; \
    QMMF_ERROR("assert(%s) at %s:%d", #condition, __FILE__, __LINE__); \
    *p = 4; \
    ALOGE("%p", p); \
  } \
} while (0)

#define LOG_LEVEL_KPI

// INFO, ERROR and WARN logs are enabled by default
#define QMMF_INFO(fmt, args...)  ALOGI(fmt, ##args)
#define QMMF_WARN(fmt, args...)  ALOGW(fmt, ##args)
#define QMMF_ERROR(fmt, args...) ALOGE(fmt, ##args)

static inline void unused(...) {};

extern uint32_t qmmf_log_level;

#define QMMF_GET_LOG_LEVEL()                               \
  ({                                                       \
    char prop[PROP_VALUE_MAX];                         \
    property_get("persist.qmmf.sdk.log.level", prop, "0"); \
    qmmf_log_level = atoi(prop);                           \
  })


#ifdef HAVE_ANDROID_UTILS
#define QMMF_DEBUG(fmt, args...) ALOGD_IF((qmmf_log_level > 0), fmt, ##args)
#define QMMF_VERBOSE(fmt, args...) ALOGV_IF((qmmf_log_level > 1), fmt, ##args)
#else
#define QMMF_DEBUG(fmt, args...)                \
  ({                                            \
     if (qmmf_log_level > 0) {                  \
       syslog (LOG_DEBUG, LOG_TAG fmt, ##args); \
     }                                          \
  })
#define QMMF_VERBOSE(fmt, args...)               \
  ({                                             \
     if (qmmf_log_level > 1) {                   \
       syslog (LOG_NOTICE, LOG_TAG fmt, ##args); \
     }                                           \
  })
#endif // HAVE_ANDROID_UTILS


#ifdef LOG_LEVEL_KPI

#define DEFAULT_KPI_FLAG   0
#define BASE_KPI_FLAG      1
#define DETAIL_KPI_FLAG    2
#define FTRACE_BUFFER_SIZE 512

extern volatile uint32_t kpi_debug_level;
extern int ftrace_fd;

static inline int get_ftrace_fd(void) {
  if (ftrace_fd <0) {
    ftrace_fd = open("/sys/kernel/debug/tracing/trace_marker", O_WRONLY);
  }
  return ftrace_fd;
}

static inline void ftrace_write(const char *log, size_t len) {
  auto fd = get_ftrace_fd();
  write(fd, log, len);
}

static inline void ftrace_begin(const char* name) {
  char buffer[FTRACE_BUFFER_SIZE+1];
  buffer[FTRACE_BUFFER_SIZE] = 0;
  size_t len;
  len = snprintf(buffer, FTRACE_BUFFER_SIZE, "B|%s\n", name);
  ftrace_write(buffer, len);
}

static inline void ftrace_end() {
  ftrace_write("E|\n", 3);
}

static inline void ftrace_async_begin(const char* name, int32_t cookie) {
  char buffer[FTRACE_BUFFER_SIZE+1];
  buffer[FTRACE_BUFFER_SIZE] = 0;
  size_t len;
  len = snprintf(buffer, FTRACE_BUFFER_SIZE, "S|%s|%d\n", name, cookie);
  ftrace_write(buffer, len);
}

static inline void ftrace_async_end(const char* name, int32_t cookie) {
  char buffer[FTRACE_BUFFER_SIZE+1];
  buffer[FTRACE_BUFFER_SIZE] = 0;
  size_t len;
  len = snprintf(buffer, FTRACE_BUFFER_SIZE, "E|%s|%d\n", name, cookie);
  ftrace_write(buffer, len);
}

#define QMMF_KPI_GET_MASK() ({\
char prop[PROP_VALUE_MAX];\
property_get("persist.qmmf.kpi.debug", prop,\
  std::to_string(DEFAULT_KPI_FLAG).c_str()); \
kpi_debug_level = atoi (prop);})

class BaseKpiObject {
public:
    BaseKpiObject(const char* str) {
        if (kpi_debug_level >= BASE_KPI_FLAG) {
            ftrace_begin(str);
        }
    }

    ~BaseKpiObject() {
        if (kpi_debug_level >= BASE_KPI_FLAG) {
            ftrace_end();
        }
    }
};

#define QMMF_KPI_BASE() ({\
BaseKpiObject a(__func__);\
})

#define QMMF_KPI_ASYNC_BEGIN(name, cookie) ({\
if (kpi_debug_level >= BASE_KPI_FLAG) { \
    ftrace_async_begin(name, cookie); \
}\
})

#define QMMF_KPI_ASYNC_END(name, cookie) ({\
if (kpi_debug_level >= BASE_KPI_FLAG) { \
    ftrace_async_end(name, cookie); \
}\
})

class DetailKpiObject {
public:
    DetailKpiObject(const char* str) {
        if (kpi_debug_level >= DETAIL_KPI_FLAG) {
            ftrace_begin(str);
        }
    }
    ~DetailKpiObject() {
        if (kpi_debug_level >= DETAIL_KPI_FLAG) {
            ftrace_end();
        }
    }
};

#define QMMF_KPI_DETAIL() ({\
DetailKpiObject a(__func__);\
})
#else
#define QMMF_KPI_GET_MASK()                do {} while (0)
#define QMMF_KPI_BASE()                    do {} while (0)
#define QMMF_KPI_DETAIL()                  do {} while (0)
#define QMMF_KPI_ASYNC_BEGIN(name, cookie) do {} while (0)
#define QMMF_KPI_ASYNC_END(name, cookie)   do {} while (0)
#endif // LOG_LEVEL_KPI
