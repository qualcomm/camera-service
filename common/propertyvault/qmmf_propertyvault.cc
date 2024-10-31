/*
* Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#include "qmmf_propertyvault.h"

using property_get_fnp = decltype(property_get);
using property_set_fnp = decltype(property_set);

void qmmf_libpropertyvaultOpen() __attribute__ ((constructor (101)));
void qmmf_libpropertyvaultClose() __attribute__ ((destructor (101)));

static void* libpropertyvault_handle = NULL;
static property_get_fnp* qmmf_property_get_internal = NULL;
static property_set_fnp* qmmf_property_set_internal = NULL;

int qmmf_property_get(const char *key, char *value, const char *default_value) {
    if (NULL == qmmf_property_get_internal) {
        return -1;
    }
    return qmmf_property_get_internal(key, value, default_value);
}

int qmmf_property_set(const char *key, const char *value) {
    if (NULL == qmmf_property_set_internal) {
        return -1;
    }
    return qmmf_property_set_internal(key, value);
}

void qmmf_libpropertyvaultOpen()
{
    void* libpropertyvault_handle = NULL;
    if (NULL == libpropertyvault_handle) {
        libpropertyvault_handle =
            dlopen ("libpropertyvault.so.0", RTLD_LAZY);
        char* err = dlerror();

        if ((NULL != libpropertyvault_handle) && (NULL == err)) {
            qmmf_property_get_internal =
              (property_get_fnp*)dlsym(
                  libpropertyvault_handle,
                  "property_get");
            qmmf_property_set_internal =
              (property_set_fnp*)dlsym(
                  libpropertyvault_handle,
                  "property_set");
        }
    }
}

void qmmf_libpropertyvaultClose()
{
    if (libpropertyvault_handle != NULL) {
      dlclose(libpropertyvault_handle);
    }
}
