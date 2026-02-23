#!/bin/sh

# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause-Clear

# check-camx-overlay.sh

# Description:
#   This script checks whether any /dev/video* device is associated with a
#   driver module that contains the keyword "camera". It does this by:
#     1. Iterating through all /dev/video* devices.
#     2. Following the sysfs path to each device's driver module.
#     3. Checking if the resolved module path contains the substring "camera".
#
# Behavior:
#   - If any video device is detected whose driver module name includes
#     "camera", the script exits with status 0 (success).
#   - If no such device is found, it exits with status 1.
#
# Use case:
#   Can be used as a probe script to detect presence of camera
#   overlay drivers in systems like CAMX.
#

# Check if video4linux directory exists
if [ ! -d "/sys/class/video4linux" ]; then
    exit 1
fi

for dev in /dev/video*; do

    # Skip if glob didn't match any files
    [ -e "${dev}" ] || continue

    mod=$(readlink -f "/sys/class/video4linux/$(basename "${dev}")/device/driver/module" 2>/dev/null)
    case "${mod}" in
        *camera*) exit 0 ;;
    esac
done

exit 1
