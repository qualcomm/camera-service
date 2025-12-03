/*
* Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#define LOG_TAG "CommonConfig"

#include <cstring>
#include "common/utils/qmmf_common_utils.h"

int
qmmf_property_get (const char *key, char *value, const char *default_value)
{
  GError *error = NULL;
  GKeyFile *key_file = NULL;
  gchar *prop_val = NULL;
  gint ret = 0;

  key_file = g_key_file_new ();

  /* Load the config file from the QMMF_CONFIG_FILE/cam-server.ini */
  if (!g_key_file_load_from_file
      (key_file, QMMF_CONFIG_FILE, G_KEY_FILE_NONE, &error)) {

    /* Return the default value */
    strncpy (value, default_value, QMMF_PROP_VAL_MAX - 1);
    value[QMMF_PROP_VAL_MAX - 1] = '\0';
    QMMF_WARN ("Error loading %s file. Using default val", QMMF_CONFIG_FILE);
    ret = -1;
    goto done;
  }

  prop_val = g_key_file_get_string (key_file, "default", key, &error);

  if (!prop_val) {
    /* use default if property is not set in conf file */
    strncpy (value, default_value, QMMF_PROP_VAL_MAX - 1);
    value[QMMF_PROP_VAL_MAX - 1] = '\0';
    QMMF_WARN ("Error loading config %s. Using default val", key);
    ret = -1;
  } else
    strncpy (value, prop_val, QMMF_PROP_VAL_MAX - 1);

done:
  if (error)
    g_error_free (error);
  if (key_file)
    g_key_file_free (key_file);
  if (prop_val)
    g_free (prop_val);

  return ret;
}

int
qmmf_property_set (const char *key, const char *value)
{
  GKeyFile *key_file = NULL;
  GError *error = NULL;
  gint ret = 0;

  key_file = g_key_file_new ();
  g_key_file_set_string (key_file, "default", key, value);

  // Write to config file.
  if (!g_key_file_save_to_file (key_file, QMMF_CONFIG_FILE, &error)) {
    g_warning ("Error saving key : %s to file : %s", key, QMMF_CONFIG_FILE);
    ret = -1;
  }

  if (error)
    g_error_free (error);
  if (key_file)
    g_key_file_free (key_file);

  return ret;
}
