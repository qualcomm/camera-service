/*
 * Copyright (C) 2012 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Changes from Qualcomm Technologies, Inc. are provided under the following license:
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <dlfcn.h>

#define LOG_NDEBUG 1

#define LOG_TAG "Camera2-Metadata"

#ifdef HAVE_BINDER
#include <binder/Parcel.h>
#endif
#include <common/utils/qmmf_log.h>

#include "qmmf-sdk/qmmf_camera_metadata.h"
#include "qmmf-sdk/qmmf_vendor_tag_descriptor.h"

#ifdef HAVE_BINDER
using namespace android;
#endif

namespace qmmf {

#define ALIGN_TO(val, alignment) \
    (((uintptr_t)(val) + ((alignment) - 1)) & ~((alignment) - 1))

#ifdef HAVE_BINDER
typedef Parcel::WritableBlob WritableBlob;
typedef Parcel::ReadableBlob ReadableBlob;
#endif

void* CameraMetadata::libcamera_metadata_handle = NULL;
add_camera_metadata_entry_fnp*
    CameraMetadata::add_camera_metadata_entry = NULL;
allocate_camera_metadata_fnp*
    CameraMetadata::allocate_camera_metadata = NULL;
allocate_copy_camera_metadata_checked_fnp*
    CameraMetadata::allocate_copy_camera_metadata_checked = NULL;
append_camera_metadata_fnp*
    CameraMetadata::append_camera_metadata = NULL;
calculate_camera_metadata_entry_data_size_fnp*
    CameraMetadata::calculate_camera_metadata_entry_data_size = NULL;
clone_camera_metadata_fnp*
    CameraMetadata::clone_camera_metadata = NULL;
copy_camera_metadata_fnp*
    CameraMetadata::copy_camera_metadata = NULL;
delete_camera_metadata_entry_fnp*
    CameraMetadata::delete_camera_metadata_entry = NULL;
dump_indented_camera_metadata_fnp*
    CameraMetadata::dump_indented_camera_metadata = NULL;
find_camera_metadata_entry_fnp*
    CameraMetadata::find_camera_metadata_entry = NULL;
find_camera_metadata_ro_entry_fnp*
    CameraMetadata::find_camera_metadata_ro_entry = NULL;
free_camera_metadata_fnp*
    CameraMetadata::free_camera_metadata = NULL;
get_camera_metadata_alignment_fnp*
    CameraMetadata::get_camera_metadata_alignment = NULL;
get_camera_metadata_compact_size_fnp*
    CameraMetadata::get_camera_metadata_compact_size = NULL;
get_camera_metadata_data_capacity_fnp*
    CameraMetadata::get_camera_metadata_data_capacity = NULL;
get_camera_metadata_data_count_fnp*
    CameraMetadata::get_camera_metadata_data_count = NULL;
get_camera_metadata_entry_capacity_fnp*
    CameraMetadata::get_camera_metadata_entry_capacity = NULL;
get_camera_metadata_entry_count_fnp*
    CameraMetadata::get_camera_metadata_entry_count = NULL;
get_camera_metadata_section_name_fnp*
    CameraMetadata::get_camera_metadata_section_name = NULL;
get_camera_metadata_tag_name_fnp*
    CameraMetadata::get_camera_metadata_tag_name = NULL;
get_camera_metadata_tag_type_fnp*
    CameraMetadata::get_camera_metadata_tag_type = NULL;
get_camera_metadata_size_fnp*
    CameraMetadata::get_camera_metadata_size = NULL;
sort_camera_metadata_fnp*
    CameraMetadata::sort_camera_metadata = NULL;
update_camera_metadata_entry_fnp*
    CameraMetadata::update_camera_metadata_entry = NULL;
validate_camera_metadata_structure_fnp*
    CameraMetadata::validate_camera_metadata_structure = NULL;
unsigned int (*CameraMetadata::camera_metadata_section_bounds)[2] = NULL;
const char** CameraMetadata::camera_metadata_section_names = NULL;
const char** CameraMetadata::camera_metadata_type_names = NULL;

void CameraMetadata_libCameraMetadataOpen() __attribute__ ((constructor (101)));
void CameraMetadata_libCameraMetadataClose() __attribute__ ((destructor (101)));

void CameraMetadata_libCameraMetadataOpen()
{
    if (NULL == CameraMetadata::libcamera_metadata_handle) {
        CameraMetadata::libcamera_metadata_handle =
            dlopen("libcamera_metadata.so.0", RTLD_LAZY);
        char* err = dlerror();

        if ((NULL != CameraMetadata::libcamera_metadata_handle) && (NULL == err)) {
            CameraMetadata::add_camera_metadata_entry =
                reinterpret_cast<add_camera_metadata_entry_fnp*>(
                dlsym(CameraMetadata::libcamera_metadata_handle,
                "add_camera_metadata_entry"));
            CameraMetadata::allocate_camera_metadata =
                reinterpret_cast<allocate_camera_metadata_fnp*>(
                dlsym(CameraMetadata::libcamera_metadata_handle,
                "allocate_camera_metadata"));
            CameraMetadata::allocate_copy_camera_metadata_checked =
                reinterpret_cast<allocate_copy_camera_metadata_checked_fnp*>(
                dlsym(CameraMetadata::libcamera_metadata_handle,
                "allocate_copy_camera_metadata_checked"));
            CameraMetadata::append_camera_metadata =
                reinterpret_cast<append_camera_metadata_fnp*>(
                dlsym(CameraMetadata::libcamera_metadata_handle,
                "append_camera_metadata"));
            CameraMetadata::calculate_camera_metadata_entry_data_size =
                reinterpret_cast<calculate_camera_metadata_entry_data_size_fnp*>(
                dlsym(CameraMetadata::libcamera_metadata_handle,
                "calculate_camera_metadata_entry_data_size"));
            CameraMetadata::clone_camera_metadata =
                reinterpret_cast<clone_camera_metadata_fnp*>(
                dlsym(CameraMetadata::libcamera_metadata_handle,
                "clone_camera_metadata"));
            CameraMetadata::copy_camera_metadata =
                reinterpret_cast<copy_camera_metadata_fnp*>(
                dlsym(CameraMetadata::libcamera_metadata_handle,
                "copy_camera_metadata"));
            CameraMetadata::delete_camera_metadata_entry =
                reinterpret_cast<delete_camera_metadata_entry_fnp*>(
                dlsym(CameraMetadata::libcamera_metadata_handle,
                "delete_camera_metadata_entry"));
            CameraMetadata::dump_indented_camera_metadata =
                reinterpret_cast<dump_indented_camera_metadata_fnp*>(
                dlsym(CameraMetadata::libcamera_metadata_handle,
                "dump_indented_camera_metadata"));
            CameraMetadata::find_camera_metadata_entry =
                reinterpret_cast<find_camera_metadata_entry_fnp*>(
                dlsym(CameraMetadata::libcamera_metadata_handle,
                "find_camera_metadata_entry"));
            CameraMetadata::find_camera_metadata_ro_entry =
                reinterpret_cast<find_camera_metadata_ro_entry_fnp*>(
                dlsym(CameraMetadata::libcamera_metadata_handle,
                "find_camera_metadata_ro_entry"));
            CameraMetadata::free_camera_metadata =
                reinterpret_cast<free_camera_metadata_fnp*>(
                dlsym(CameraMetadata::libcamera_metadata_handle,
                "free_camera_metadata"));
            CameraMetadata::get_camera_metadata_alignment =
                reinterpret_cast<get_camera_metadata_alignment_fnp*>(
                dlsym(CameraMetadata::libcamera_metadata_handle,
                "get_camera_metadata_alignment"));
            CameraMetadata::get_camera_metadata_compact_size =
                reinterpret_cast<get_camera_metadata_compact_size_fnp*>(
                dlsym(CameraMetadata::libcamera_metadata_handle,
                "get_camera_metadata_compact_size"));
            CameraMetadata::get_camera_metadata_data_capacity =
                reinterpret_cast<get_camera_metadata_data_capacity_fnp*>(
                dlsym(CameraMetadata::libcamera_metadata_handle,
                "get_camera_metadata_data_capacity"));
            CameraMetadata::get_camera_metadata_data_count =
                reinterpret_cast<get_camera_metadata_data_count_fnp*>(
                dlsym(CameraMetadata::libcamera_metadata_handle,
                "get_camera_metadata_data_count"));
            CameraMetadata::get_camera_metadata_entry_capacity =
                reinterpret_cast<get_camera_metadata_entry_capacity_fnp*>(
                dlsym(CameraMetadata::libcamera_metadata_handle,
                "get_camera_metadata_entry_capacity"));
            CameraMetadata::get_camera_metadata_entry_count =
                reinterpret_cast<get_camera_metadata_entry_count_fnp*>(
                dlsym(CameraMetadata::libcamera_metadata_handle,
                "get_camera_metadata_entry_count"));
            CameraMetadata::get_camera_metadata_section_name =
                reinterpret_cast<get_camera_metadata_section_name_fnp*>(
                dlsym(CameraMetadata::libcamera_metadata_handle,
                "get_camera_metadata_section_name"));
            CameraMetadata::get_camera_metadata_tag_name =
                reinterpret_cast<get_camera_metadata_tag_name_fnp*>(
                dlsym(CameraMetadata::libcamera_metadata_handle,
                "get_camera_metadata_tag_name"));
            CameraMetadata::get_camera_metadata_tag_type =
                reinterpret_cast<get_camera_metadata_tag_type_fnp*>(
                dlsym(CameraMetadata::libcamera_metadata_handle,
                "get_camera_metadata_tag_type"));
            CameraMetadata::get_camera_metadata_size =
                reinterpret_cast<get_camera_metadata_size_fnp*>(
                dlsym(CameraMetadata::libcamera_metadata_handle,
                "get_camera_metadata_size"));
            CameraMetadata::sort_camera_metadata =
                reinterpret_cast<sort_camera_metadata_fnp*>(
                dlsym(CameraMetadata::libcamera_metadata_handle,
                "sort_camera_metadata"));
            CameraMetadata::update_camera_metadata_entry =
                reinterpret_cast<update_camera_metadata_entry_fnp*>(
                dlsym(CameraMetadata::libcamera_metadata_handle,
                "update_camera_metadata_entry"));
            CameraMetadata::validate_camera_metadata_structure =
                reinterpret_cast<validate_camera_metadata_structure_fnp*>(
                dlsym(CameraMetadata::libcamera_metadata_handle,
                "validate_camera_metadata_structure"));
            CameraMetadata::camera_metadata_section_bounds =
                reinterpret_cast<unsigned int (*)[2]>(
                dlsym(CameraMetadata::libcamera_metadata_handle,
                "camera_metadata_section_bounds"));
            CameraMetadata::camera_metadata_section_names =
                reinterpret_cast<const char**>(
                dlsym(CameraMetadata::libcamera_metadata_handle,
                "camera_metadata_section_names"));
            CameraMetadata::camera_metadata_type_names =
                reinterpret_cast<const char**>(
                dlsym(CameraMetadata::libcamera_metadata_handle,
                "camera_metadata_type_names"));
            char* dlsym_err = dlerror();
            if (dlsym_err != NULL) {
                assert(CameraMetadata::add_camera_metadata_entry);
                assert(CameraMetadata::allocate_camera_metadata);
                assert(CameraMetadata::allocate_copy_camera_metadata_checked);
                assert(CameraMetadata::append_camera_metadata);
                assert(CameraMetadata::calculate_camera_metadata_entry_data_size);
                assert(CameraMetadata::clone_camera_metadata);
                assert(CameraMetadata::copy_camera_metadata);
                assert(CameraMetadata::delete_camera_metadata_entry);
                assert(CameraMetadata::dump_indented_camera_metadata);
                assert(CameraMetadata::find_camera_metadata_entry);
                assert(CameraMetadata::find_camera_metadata_ro_entry);
                assert(CameraMetadata::free_camera_metadata);
                assert(CameraMetadata::get_camera_metadata_alignment);
                assert(CameraMetadata::get_camera_metadata_compact_size);
                assert(CameraMetadata::get_camera_metadata_data_capacity);
                assert(CameraMetadata::get_camera_metadata_data_count);
                assert(CameraMetadata::get_camera_metadata_entry_capacity);
                assert(CameraMetadata::get_camera_metadata_entry_count);
                assert(CameraMetadata::get_camera_metadata_section_name);
                assert(CameraMetadata::get_camera_metadata_tag_name);
                assert(CameraMetadata::get_camera_metadata_tag_type);
                assert(CameraMetadata::get_camera_metadata_size);
                assert(CameraMetadata::sort_camera_metadata);
                assert(CameraMetadata::update_camera_metadata_entry);
                assert(CameraMetadata::validate_camera_metadata_structure);
                assert(CameraMetadata::camera_metadata_section_bounds);
                assert(CameraMetadata::camera_metadata_section_names);
                assert(CameraMetadata::camera_metadata_type_names);
            }
        }
    }
}

void CameraMetadata_libCameraMetadataClose()
{
    if (CameraMetadata::libcamera_metadata_handle != NULL) {
      dlclose(CameraMetadata::libcamera_metadata_handle);
    }
}

CameraMetadata::CameraMetadata() :
        mBuffer(NULL), mLocked(false) {}


CameraMetadata::CameraMetadata(size_t entryCapacity, size_t dataCapacity) :
        mLocked(false)
{
    if (NULL == CameraMetadata::allocate_camera_metadata) {
        add_camera_metadata_entry = NULL;
        allocate_camera_metadata = NULL;
        allocate_copy_camera_metadata_checked = NULL;
        append_camera_metadata = NULL;
        calculate_camera_metadata_entry_data_size = NULL;
        clone_camera_metadata = NULL;
        copy_camera_metadata = NULL;
        delete_camera_metadata_entry = NULL;
        dump_indented_camera_metadata = NULL;
        find_camera_metadata_entry = NULL;
        find_camera_metadata_ro_entry = NULL;
        free_camera_metadata = NULL;
        get_camera_metadata_alignment = NULL;
        get_camera_metadata_compact_size = NULL;
        get_camera_metadata_data_capacity = NULL;
        get_camera_metadata_data_count = NULL;
        get_camera_metadata_entry_capacity = NULL;
        get_camera_metadata_entry_count = NULL;
        get_camera_metadata_section_name = NULL;
        get_camera_metadata_tag_name = NULL;
        get_camera_metadata_tag_type = NULL;
        get_camera_metadata_size = NULL;
        sort_camera_metadata = NULL;
        update_camera_metadata_entry = NULL;
        validate_camera_metadata_structure = NULL;
        camera_metadata_section_bounds = NULL;
        camera_metadata_section_names = NULL;
        camera_metadata_type_names = NULL;
        CameraMetadata_libCameraMetadataClose();
    }

    if (CameraMetadata::allocate_camera_metadata != NULL) {
        mBuffer = CameraMetadata::allocate_camera_metadata(entryCapacity, dataCapacity);
    }

    assert(CameraMetadata::allocate_camera_metadata != NULL);
}

CameraMetadata::CameraMetadata(const CameraMetadata &other) :
        mLocked(false) {
    if (NULL == CameraMetadata::clone_camera_metadata) {
        add_camera_metadata_entry = NULL;
        allocate_camera_metadata = NULL;
        allocate_copy_camera_metadata_checked = NULL;
        append_camera_metadata = NULL;
        calculate_camera_metadata_entry_data_size = NULL;
        clone_camera_metadata = NULL;
        copy_camera_metadata = NULL;
        delete_camera_metadata_entry = NULL;
        dump_indented_camera_metadata = NULL;
        find_camera_metadata_entry = NULL;
        find_camera_metadata_ro_entry = NULL;
        free_camera_metadata = NULL;
        get_camera_metadata_alignment = NULL;
        get_camera_metadata_compact_size = NULL;
        get_camera_metadata_data_capacity = NULL;
        get_camera_metadata_data_count = NULL;
        get_camera_metadata_entry_capacity = NULL;
        get_camera_metadata_entry_count = NULL;
        get_camera_metadata_section_name = NULL;
        get_camera_metadata_tag_name = NULL;
        get_camera_metadata_tag_type = NULL;
        get_camera_metadata_size = NULL;
        sort_camera_metadata = NULL;
        update_camera_metadata_entry = NULL;
        validate_camera_metadata_structure = NULL;
        camera_metadata_section_bounds = NULL;
        camera_metadata_section_names = NULL;
        camera_metadata_type_names = NULL;
        CameraMetadata_libCameraMetadataClose();
    }

    if (CameraMetadata::clone_camera_metadata != NULL) {
        mBuffer = CameraMetadata::clone_camera_metadata(other.mBuffer);
    }

    assert(CameraMetadata::clone_camera_metadata != NULL);
}

CameraMetadata::CameraMetadata(camera_metadata_t *buffer) :
        mBuffer(NULL), mLocked(false) {
    acquire(buffer);
}

CameraMetadata &CameraMetadata::operator=(const CameraMetadata &other) {
    return operator=(other.mBuffer);
}

CameraMetadata &CameraMetadata::operator=(const camera_metadata_t *buffer) {
    if (mLocked) {
        QMMF_ERROR("%s: Assignment to a locked CameraMetadata!", __FUNCTION__);
        return *this;
    }

    if (!!(buffer != mBuffer)) {
        camera_metadata_t *newBuffer = CameraMetadata::clone_camera_metadata(buffer);
        clear();
        mBuffer = newBuffer;
    }
    return *this;
}

CameraMetadata::~CameraMetadata() {
    mLocked = false;
    clear();
}

camera_metadata_t* CameraMetadata::getbuffer() {
    return mBuffer;
}

const camera_metadata_t* CameraMetadata::getAndLock() const {
    mLocked = true;
    return mBuffer;
}

status_t CameraMetadata::unlock(const camera_metadata_t *buffer) {
    if (!mLocked) {
        QMMF_ERROR("%s: Can't unlock a non-locked CameraMetadata!", __FUNCTION__);
        return -ENOSYS;
    }
    if (buffer != mBuffer) {
        QMMF_ERROR("%s: Can't unlock CameraMetadata with wrong pointer!",
                __FUNCTION__);
        return -EINVAL;
    }
    mLocked = false;
    return 0;
}

camera_metadata_t* CameraMetadata::release() {
    if (mLocked) {
        QMMF_ERROR("%s: CameraMetadata is locked", __FUNCTION__);
        return NULL;
    }
    camera_metadata_t *released = mBuffer;
    mBuffer = NULL;
    return released;
}

void CameraMetadata::clear() {
    if (mLocked) {
        QMMF_ERROR("%s: CameraMetadata is locked", __FUNCTION__);
        return;
    }
    if (mBuffer) {
        CameraMetadata::free_camera_metadata(mBuffer);
        mBuffer = NULL;
    }
}

void CameraMetadata::acquire(camera_metadata_t *buffer) {
    if (mLocked) {
        QMMF_ERROR("%s: CameraMetadata is locked", __FUNCTION__);
        return;
    }
    clear();
    mBuffer = buffer;

    assert(CameraMetadata::validate_camera_metadata_structure(mBuffer, /*size*/NULL) == 0);
}

void CameraMetadata::acquire(CameraMetadata &other) {
    if (mLocked) {
        QMMF_ERROR("%s: CameraMetadata is locked", __FUNCTION__);
        return;
    }
    acquire(other.release());
}

status_t CameraMetadata::append(const CameraMetadata &other) {
    return append(other.mBuffer);
}

status_t CameraMetadata::append(const camera_metadata_t* other) {
    if (mLocked) {
        QMMF_ERROR("%s: CameraMetadata is locked", __FUNCTION__);
        return -ENOSYS;
    }
    size_t extraEntries = CameraMetadata::get_camera_metadata_entry_count(other);
    size_t extraData = CameraMetadata::get_camera_metadata_data_count(other);
    resizeIfNeeded(extraEntries, extraData);

    return CameraMetadata::append_camera_metadata(mBuffer, other);
}

size_t CameraMetadata::entryCount() const {
    return (mBuffer == NULL) ? 0 :
            CameraMetadata::get_camera_metadata_entry_count(mBuffer);
}

bool CameraMetadata::isEmpty() const {
    return entryCount() == 0;
}

status_t CameraMetadata::sort() {
    if (mLocked) {
        QMMF_ERROR("%s: CameraMetadata is locked", __FUNCTION__);
        return -ENOSYS;
    }
    return CameraMetadata::sort_camera_metadata(mBuffer);
}

status_t CameraMetadata::checkType(uint32_t tag, uint8_t expectedType) {
    int tagType = CameraMetadata::get_camera_metadata_tag_type(tag);
    if (tagType == -1) {
        QMMF_ERROR("Update metadata entry: Unknown tag %u", tag);
        return -ENOSYS;
    }
    if ( tagType != expectedType) {
        QMMF_ERROR("Mismatched tag type when updating entry %s (%u) of type %s; "
                "got type %s data instead ",
                CameraMetadata::get_camera_metadata_tag_name(tag), tag,
                CameraMetadata::camera_metadata_type_names[tagType],
                CameraMetadata::camera_metadata_type_names[expectedType]);
        return -ENOSYS;
    }
    return 0;
}

status_t CameraMetadata::update(uint32_t tag,
        const int32_t *data, size_t data_count) {
    status_t res;
    if (mLocked) {
        QMMF_ERROR("%s: CameraMetadata is locked", __FUNCTION__);
        return -ENOSYS;
    }
    if ( (res = checkType(tag, TYPE_INT32)) != 0) {
        return res;
    }
    return updateImpl(tag, (const void*)data, data_count);
}

status_t CameraMetadata::update(uint32_t tag,
        const uint8_t *data, size_t data_count) {
    status_t res;
    if (mLocked) {
        QMMF_ERROR("%s: CameraMetadata is locked", __FUNCTION__);
        return -ENOSYS;
    }
    if ( (res = checkType(tag, TYPE_BYTE)) != 0) {
        return res;
    }
    return updateImpl(tag, (const void*)data, data_count);
}

status_t CameraMetadata::update(uint32_t tag,
        const float *data, size_t data_count) {
    status_t res;
    if (mLocked) {
        QMMF_ERROR("%s: CameraMetadata is locked", __FUNCTION__);
        return -ENOSYS;
    }
    if ( (res = checkType(tag, TYPE_FLOAT)) != 0) {
        return res;
    }
    return updateImpl(tag, (const void*)data, data_count);
}

status_t CameraMetadata::update(uint32_t tag,
        const int64_t *data, size_t data_count) {
    status_t res;
    if (mLocked) {
        QMMF_ERROR("%s: CameraMetadata is locked", __FUNCTION__);
        return -ENOSYS;
    }
    if ( (res = checkType(tag, TYPE_INT64)) != 0) {
        return res;
    }
    return updateImpl(tag, (const void*)data, data_count);
}

status_t CameraMetadata::update(uint32_t tag,
        const double *data, size_t data_count) {
    status_t res;
    if (mLocked) {
        QMMF_ERROR("%s: CameraMetadata is locked", __FUNCTION__);
        return -ENOSYS;
    }
    if ( (res = checkType(tag, TYPE_DOUBLE)) != 0) {
        return res;
    }
    return updateImpl(tag, (const void*)data, data_count);
}

status_t CameraMetadata::update(uint32_t tag,
        const camera_metadata_rational_t *data, size_t data_count) {
    status_t res;
    if (mLocked) {
        QMMF_ERROR("%s: CameraMetadata is locked", __FUNCTION__);
        return -ENOSYS;
    }
    if ( (res = checkType(tag, TYPE_RATIONAL)) != 0) {
        return res;
    }
    return updateImpl(tag, (const void*)data, data_count);
}

status_t CameraMetadata::update(uint32_t tag,
        const std::string string) {
    status_t res;
    if (mLocked) {
        QMMF_ERROR("%s: CameraMetadata is locked", __FUNCTION__);
        return -ENOSYS;
    }
    if ( (res = checkType(tag, TYPE_BYTE)) != 0) {
        return res;
    }
    // string.size() doesn't count the null termination character.
    return updateImpl(tag, (const void*)string.c_str(), string.size() + 1);
}

status_t CameraMetadata::updateImpl(uint32_t tag, const void *data,
        size_t data_count) {
    status_t res;
    if (mLocked) {
        QMMF_ERROR("%s: CameraMetadata is locked", __FUNCTION__);
        return -ENOSYS;
    }
    int type = CameraMetadata::get_camera_metadata_tag_type(tag);
    if (type == -1) {
        QMMF_ERROR("%s: Tag %u not found", __FUNCTION__, tag);
        return -EINVAL;
    }
    // Safety check - ensure that data isn't pointing to this metadata, since
    // that would get invalidated if a resize is needed
    size_t bufferSize = CameraMetadata::get_camera_metadata_size(mBuffer);
    uintptr_t bufAddr = reinterpret_cast<uintptr_t>(mBuffer);
    uintptr_t dataAddr = reinterpret_cast<uintptr_t>(data);
    if (dataAddr > bufAddr && dataAddr < (bufAddr + bufferSize)) {
        QMMF_ERROR("%s: Update attempted with data from the same metadata buffer!",
                __FUNCTION__);
        return -ENOSYS;
    }

    size_t data_size = CameraMetadata::calculate_camera_metadata_entry_data_size(type,
            data_count);

    res = resizeIfNeeded(1, data_size);

    if (res == 0) {
        camera_metadata_entry_t entry;
        res = CameraMetadata::find_camera_metadata_entry(mBuffer, tag, &entry);
        if (res == -ENOENT) {
            res = CameraMetadata::add_camera_metadata_entry(mBuffer,
                    tag, data, data_count);
        } else if (res == 0) {
            res = CameraMetadata::update_camera_metadata_entry(mBuffer,
                    entry.index, data, data_count, NULL);
        }
    }

    if (res != 0) {
        QMMF_ERROR("%s: Unable to update metadata entry %s.%s (%x): %s (%u)",
                __FUNCTION__, CameraMetadata::get_camera_metadata_section_name(tag),
                CameraMetadata::get_camera_metadata_tag_name(tag), tag, strerror(-res), res);
    }

    assert(CameraMetadata::validate_camera_metadata_structure(mBuffer, /*size*/NULL) == 0);

    return res;
}

bool CameraMetadata::exists(uint32_t tag) const {
    camera_metadata_ro_entry entry;
    return CameraMetadata::find_camera_metadata_ro_entry(mBuffer, tag, &entry) == 0;
}

camera_metadata_entry_t CameraMetadata::find(uint32_t tag) {
    status_t res;
    camera_metadata_entry entry = {};
    if (mLocked) {
        QMMF_ERROR("%s: CameraMetadata is locked", __FUNCTION__);
        entry.count = 0;
        return entry;
    }
    res = CameraMetadata::find_camera_metadata_entry(mBuffer, tag, &entry);
    if (res != 0) {
        entry.count = 0;
        entry.data.u8 = NULL;
    }
    return entry;
}

camera_metadata_ro_entry_t CameraMetadata::find(uint32_t tag) const {
    status_t res;
    camera_metadata_ro_entry entry;
    res = CameraMetadata::find_camera_metadata_ro_entry(mBuffer, tag, &entry);
    if (res != 0 ) {
        entry.count = 0;
        entry.data.u8 = NULL;
    }
    return entry;
}

status_t CameraMetadata::erase(uint32_t tag) {
    camera_metadata_entry_t entry;
    status_t res;
    if (mLocked) {
        QMMF_ERROR("%s: CameraMetadata is locked", __FUNCTION__);
        return -ENOSYS;
    }
    res = CameraMetadata::find_camera_metadata_entry(mBuffer, tag, &entry);
    if (res == -ENOENT) {
        return 0;
    } else if (res != 0) {
        QMMF_ERROR("%s: Error looking for entry %s.%s (%x): %s %u",
                __FUNCTION__,
                CameraMetadata::get_camera_metadata_section_name(tag),
                CameraMetadata::get_camera_metadata_tag_name(tag), tag, strerror(-res), res);
        return res;
    }
    res = CameraMetadata::delete_camera_metadata_entry(mBuffer, entry.index);
    if (res != 0) {
        QMMF_ERROR("%s: Error deleting entry %s.%s (%x): %s %u",
                __FUNCTION__,
                CameraMetadata::get_camera_metadata_section_name(tag),
                CameraMetadata::get_camera_metadata_tag_name(tag), tag, strerror(-res), res);
    }
    return res;
}

void CameraMetadata::dump(int fd, int verbosity, int indentation) const {
    CameraMetadata::dump_indented_camera_metadata(mBuffer, fd, verbosity, indentation);
}

status_t CameraMetadata::resizeIfNeeded(size_t extraEntries, size_t extraData) {
    if (mBuffer == NULL) {
        mBuffer = CameraMetadata::allocate_camera_metadata(extraEntries * 2, extraData * 2);
        if (mBuffer == NULL) {
            QMMF_ERROR("%s: Can't allocate larger metadata buffer", __FUNCTION__);
            return -ENOENT;
        }
    } else {
        size_t currentEntryCount = CameraMetadata::get_camera_metadata_entry_count(mBuffer);
        size_t currentEntryCap = CameraMetadata::get_camera_metadata_entry_capacity(mBuffer);
        size_t newEntryCount = currentEntryCount +
                extraEntries;
        newEntryCount = (newEntryCount > currentEntryCap) ?
                newEntryCount * 2 : currentEntryCap;

        size_t currentDataCount = CameraMetadata::get_camera_metadata_data_count(mBuffer);
        size_t currentDataCap = CameraMetadata::get_camera_metadata_data_capacity(mBuffer);
        size_t newDataCount = currentDataCount +
                extraData;
        newDataCount = (newDataCount > currentDataCap) ?
                newDataCount * 2 : currentDataCap;

        if (newEntryCount > currentEntryCap ||
                newDataCount > currentDataCap) {
            camera_metadata_t *oldBuffer = mBuffer;
            mBuffer = CameraMetadata::allocate_camera_metadata(newEntryCount,
                    newDataCount);
            if (mBuffer == NULL) {
                QMMF_ERROR("%s: Can't allocate larger metadata buffer", __FUNCTION__);
                return -ENOENT;
            }
            CameraMetadata::append_camera_metadata(mBuffer, oldBuffer);
            CameraMetadata::free_camera_metadata(oldBuffer);
        }
    }
    return 0;
}

#ifdef HAVE_BINDER
status_t CameraMetadata::readFromParcel(const Parcel& data,
                                        camera_metadata_t** out) {

    status_t err = 0;

    camera_metadata_t* metadata = NULL;

    if (out) {
        *out = NULL;
    }

    // See CameraMetadata::writeToParcel for parcel data layout diagram and explanation.
    // arg0 = blobSize (int32)
    int32_t blobSizeTmp = -1;
    if ((err = data.readInt32(&blobSizeTmp)) != 0) {
        QMMF_ERROR("%s: Failed to read metadata size (error %d %s)",
              __FUNCTION__, err, strerror(-err));
        return err;
    }
    const size_t blobSize = static_cast<size_t>(blobSizeTmp);
    const size_t alignment = CameraMetadata::get_camera_metadata_alignment();

    // Special case: zero blob size means zero sized (NULL) metadata.
    if (blobSize == 0) {
        QMMF_VERBOSE("%s: Read 0-sized metadata", __FUNCTION__);
        return 0;
    }

    if (blobSize <= alignment) {
        QMMF_ERROR("%s: metadata blob is malformed, blobSize(%zu) should be larger than alignment(%zu)",
                __FUNCTION__, blobSize, alignment);
        return -EINVAL;
    }

    const size_t metadataSize = blobSize - alignment;

    // NOTE: this doesn't make sense to me. shouldn't the blob
    // know how big it is? why do we have to specify the size
    // to Parcel::readBlob ?
    ReadableBlob blob;
    // arg1 = metadata (blob)
    do {
        if ((err = data.readBlob(blobSize, &blob)) != 0) {
            QMMF_ERROR("%s: Failed to read metadata blob (sized %zu). Possible "
                  " serialization bug. Error %d %s",
                  __FUNCTION__, blobSize, err, strerror(-err));
            break;
        }

        // arg2 = offset (blob)
        // Must be after blob since we don't know offset until after writeBlob.
        int32_t offsetTmp;
        if ((err = data.readInt32(&offsetTmp)) != 0) {
            QMMF_ERROR("%s: Failed to read metadata offsetTmp (error %d %s)",
                  __FUNCTION__, err, strerror(-err));
            break;
        }
        const size_t offset = static_cast<size_t>(offsetTmp);
        if (offset >= alignment) {
            QMMF_ERROR("%s: metadata offset(%zu) should be less than alignment(%zu)",
                    __FUNCTION__, blobSize, alignment);
            err = -EINVAL;
            break;
        }

        const uintptr_t metadataStart = reinterpret_cast<uintptr_t>(blob.data()) + offset;
        const camera_metadata_t* tmp =
                       reinterpret_cast<const camera_metadata_t*>(metadataStart);
        QMMF_VERBOSE("%s: alignment is: %zu, metadata start: %p, offset: %zu",
                __FUNCTION__, alignment, tmp, offset);
        metadata = CameraMetadata::allocate_copy_camera_metadata_checked(tmp, metadataSize);
        if (metadata == NULL) {
            // We consider that allocation only fails if the validation
            // also failed, therefore the readFromParcel was a failure.
            QMMF_ERROR("%s: metadata allocation and copy failed", __FUNCTION__);
            err = -EINVAL;
        }
    } while(0);
    blob.release();

    if (out) {
        QMMF_VERBOSE("%s: Set out metadata to %p", __FUNCTION__, metadata);
        *out = metadata;
    } else if (metadata != NULL) {
        QMMF_VERBOSE("%s: Freed camera metadata at %p", __FUNCTION__, metadata);
        CameraMetadata::free_camera_metadata(metadata);
    }

    return err;
}

status_t CameraMetadata::writeToParcel(Parcel& data,
                                       const camera_metadata_t* metadata) {
    status_t res = 0;

    /**
     * Below is the camera metadata parcel layout:
     *
     * |--------------------------------------------|
     * |             arg0: blobSize                 |
     * |              (length = 4)                  |
     * |--------------------------------------------|<--Skip the rest if blobSize == 0.
     * |                                            |
     * |                                            |
     * |              arg1: blob                    |
     * | (length = variable, see arg1 layout below) |
     * |                                            |
     * |                                            |
     * |--------------------------------------------|
     * |              arg2: offset                  |
     * |              (length = 4)                  |
     * |--------------------------------------------|
     */

    // arg0 = blobSize (int32)
    if (metadata == NULL) {
        // Write zero blobSize for null metadata.
        return data.writeInt32(0);
    }

    /**
     * Always make the blob size sufficiently larger, as we need put alignment
     * padding and metadata into the blob. Since we don't know the alignment
     * offset before writeBlob. Then write the metadata to aligned offset.
     */
    const size_t metadataSize = CameraMetadata::get_camera_metadata_compact_size(metadata);
    const size_t alignment = CameraMetadata::get_camera_metadata_alignment();
    const size_t blobSize = metadataSize + alignment;
    res = data.writeInt32(static_cast<int32_t>(blobSize));
    if (res != 0) {
        return res;
    }

    size_t offset = 0;
    /**
     * arg1 = metadata (blob).
     *
     * The blob size is the sum of front padding size, metadata size and back padding
     * size, which is equal to metadataSize + alignment.
     *
     * The blob layout is:
     * |------------------------------------|<----Start address of the blob (unaligned).
     * |           front padding            |
     * |          (size = offset)           |
     * |------------------------------------|<----Aligned start address of metadata.
     * |                                    |
     * |                                    |
     * |            metadata                |
     * |       (size = metadataSize)        |
     * |                                    |
     * |                                    |
     * |------------------------------------|
     * |           back padding             |
     * |     (size = alignment - offset)    |
     * |------------------------------------|<----End address of blob.
     *                                            (Blob start address + blob size).
     */
    WritableBlob blob;
    do {
        res = data.writeBlob(blobSize, false, &blob);
        if (res != 0) {
            break;
        }
        const uintptr_t metadataStart = ALIGN_TO(blob.data(), alignment);
        offset = metadataStart - reinterpret_cast<uintptr_t>(blob.data());
        QMMF_VERBOSE("%s: alignment is: %zu, metadata start: %p, offset: %zu",
                __FUNCTION__, alignment,
                reinterpret_cast<const void *>(metadataStart), offset);
        CameraMetadata::copy_camera_metadata(reinterpret_cast<void*>(metadataStart),
                metadataSize, metadata);

        // Not too big of a problem since receiving side does hard validation
        // Don't check the size since the compact size could be larger
        if (CameraMetadata::validate_camera_metadata_structure(metadata, /*size*/NULL) != 0) {
            QMMF_WARN("%s: Failed to validate metadata %p before writing blob",
                   __FUNCTION__, metadata);
        }

    } while(false);
    blob.release();

    // arg2 = offset (int32)
    res = data.writeInt32(static_cast<int32_t>(offset));

    return res;
}

status_t CameraMetadata::readFromParcel(const Parcel *parcel) {

    QMMF_VERBOSE("%s: parcel = %p", __FUNCTION__, parcel);

    status_t res = 0;

    if (parcel == NULL) {
        QMMF_ERROR("%s: parcel is null", __FUNCTION__);
        return -EINVAL;
    }

    if (mLocked) {
        QMMF_ERROR("%s: CameraMetadata is locked", __FUNCTION__);
        return -ENOSYS;
    }

    camera_metadata *buffer = NULL;
    // TODO: reading should return a status code, in case validation fails
    res = CameraMetadata::readFromParcel(*parcel, &buffer);

    if (res != 0) {
        QMMF_ERROR("%s: Failed to read from parcel. Metadata is unchanged.",
              __FUNCTION__);
        return res;
    }

    clear();
    mBuffer = buffer;

    return 0;
}

status_t CameraMetadata::writeToParcel(Parcel *parcel) const {

    QMMF_VERBOSE("%s: parcel = %p", __FUNCTION__, parcel);

    if (parcel == NULL) {
        QMMF_ERROR("%s: parcel is null", __FUNCTION__);
        return -EINVAL;
    }

    return CameraMetadata::writeToParcel(*parcel, mBuffer);
}
#endif

void CameraMetadata::swap(CameraMetadata& other) {
    if (mLocked) {
        QMMF_ERROR("%s: CameraMetadata is locked", __FUNCTION__);
        return;
    } else if (other.mLocked) {
        QMMF_ERROR("%s: Other CameraMetadata is locked", __FUNCTION__);
        return;
    }

    camera_metadata* thisBuf = mBuffer;
    camera_metadata* otherBuf = other.mBuffer;

    other.mBuffer = thisBuf;
    mBuffer = otherBuf;
}

status_t CameraMetadata::getTagFromName(const char *name,
        const VendorTagDescriptor* vTags, uint32_t *tag) {

    if (name == nullptr || tag == nullptr) return -EINVAL;

    size_t nameLength = strlen(name);

    std::vector<std::string> vendorSections;
    size_t vendorSectionCount = 0;

    if (vTags != NULL) {
        vendorSections = vTags->getAllSectionNames();
        vendorSectionCount = vendorSections.size();
    }

    // First, find the section by the longest string match
    const char *section = NULL;
    size_t sectionIndex = 0;
    size_t sectionLength = 0;
    size_t totalSectionCount = ANDROID_SECTION_COUNT + vendorSectionCount;
    for (size_t i = 0; i < totalSectionCount; ++i) {

        const char *str = (i < ANDROID_SECTION_COUNT) ?
                CameraMetadata::camera_metadata_section_names[i] :
                vendorSections[i - ANDROID_SECTION_COUNT].c_str();

        if (str == nullptr) {
          continue;
        }

        if (strstr(name, str) == name) { // name begins with the section name
            size_t strLength = strlen(str);

            QMMF_VERBOSE("%s: Name begins with section name", __FUNCTION__);

            // section name is the longest we've found so far
            if (section == NULL || sectionLength < strLength) {
                section = str;
                sectionIndex = i;
                sectionLength = strLength;

                QMMF_VERBOSE("%s: Found new best section (%s)", __FUNCTION__, section);
            }
        }
    }

    // TODO: Make above get_camera_metadata_section_from_name ?

    if (section == NULL) {
        return -ENOENT;
    } else {
        QMMF_VERBOSE("%s: Found matched section '%s' (%zu)",
              __FUNCTION__, section, sectionIndex);
    }

    // Get the tag name component of the name
    const char *nameTagName = name + sectionLength + 1; // x.y.z -> z
    if (sectionLength + 1 >= nameLength) {
        return -EINVAL;
    }

    // Match rest of name against the tag names in that section only
    uint32_t candidateTag = 0;
    if (sectionIndex < ANDROID_SECTION_COUNT) {
        // Match built-in tags (typically android.*)
        uint32_t tagBegin, tagEnd; // [tagBegin, tagEnd)
        tagBegin = CameraMetadata::camera_metadata_section_bounds[sectionIndex][0];
        tagEnd = CameraMetadata::camera_metadata_section_bounds[sectionIndex][1];

        for (candidateTag = tagBegin; candidateTag < tagEnd; ++candidateTag) {
            const char *tagName = CameraMetadata::get_camera_metadata_tag_name(candidateTag);

            if (tagName != NULL && strcmp(nameTagName, tagName) == 0) {
                QMMF_VERBOSE("%s: Found matched tag '%s' (%u)",
                      __FUNCTION__, tagName, candidateTag);
                break;
            }
        }

        if (candidateTag == tagEnd) {
            return -ENOENT;
        }
    } else if (vTags != NULL) {
        // Match vendor tags (typically com.*)
        const std::string sectionName(section);
        const std::string tagName(nameTagName);

        status_t res = 0;
        if ((res = vTags->lookupTag(tagName, sectionName, &candidateTag)) != 0) {
            return -ENOENT;
        }
    }

    *tag = candidateTag;
    return 0;
}


}; // namespace qmmf
