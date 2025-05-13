/*
* Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#pragma once

#include <string>
#include <sstream>
#include <dlfcn.h>

#ifdef HAVE_ANDROID_UTILS
#include <cutils/properties.h>
#else
#include "properties.h"
#endif

int qmmf_property_get(const char *key, char *value, const char *default_value);
int qmmf_property_set(const char *key, const char *value);

#define QMMF_KPI_GET_MASK() ({\
char prop[PROP_VALUE_MAX];\
qmmf_property_get("persist.qmmf.kpi.debug", prop,\
  std::to_string(DEFAULT_KPI_FLAG).c_str()); \
kpi_debug_level = atoi (prop);})

#define QMMF_GET_LOG_LEVEL()                               \
  ({                                                       \
    char prop[PROP_VALUE_MAX];                         \
    qmmf_property_get("persist.qmmf.sdk.log.level", prop, "0"); \
    qmmf_log_level = atoi(prop);                           \
  })

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
    char prop_val[PROP_VALUE_MAX];

    std::stringstream s;
    s << default_value;

    qmmf_property_get(property.c_str(), prop_val, s.str().c_str());

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

    qmmf_property_set(property.c_str(), s.str().c_str());
  }
};
