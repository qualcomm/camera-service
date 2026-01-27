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
#endif

#define QMMF_PROP_VAL_MAX 128

int qmmf_property_get(const char *key, char *value, const char *default_value);
int qmmf_property_set(const char *key, const char *value);

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
