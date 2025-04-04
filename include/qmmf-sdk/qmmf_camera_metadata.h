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
 * Changes from Qualcomm Innovation Center, Inc. are provided under the following license:
 * Copyright (c) 2023-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QMMF_CLIENT_CAMERA2_CAMERAMETADATA_CPP
#define QMMF_CLIENT_CAMERA2_CAMERAMETADATA_CPP

#include <string>
#include <vector>

#include <system/camera_metadata.h>

using add_camera_metadata_entry_fnp =
    decltype(add_camera_metadata_entry);
using allocate_camera_metadata_fnp =
    decltype(allocate_camera_metadata);
using allocate_copy_camera_metadata_checked_fnp =
    decltype(allocate_copy_camera_metadata_checked);
using append_camera_metadata_fnp =
    decltype(append_camera_metadata);
using calculate_camera_metadata_entry_data_size_fnp =
    decltype(calculate_camera_metadata_entry_data_size);
using clone_camera_metadata_fnp =
    decltype(clone_camera_metadata);
using copy_camera_metadata_fnp =
    decltype(copy_camera_metadata);
using delete_camera_metadata_entry_fnp =
    decltype(delete_camera_metadata_entry);
using dump_indented_camera_metadata_fnp =
    decltype(dump_indented_camera_metadata);
using find_camera_metadata_entry_fnp =
    decltype(find_camera_metadata_entry);
using find_camera_metadata_ro_entry_fnp =
    decltype(find_camera_metadata_ro_entry);
using free_camera_metadata_fnp =
    decltype(free_camera_metadata);
using get_camera_metadata_alignment_fnp =
    decltype(get_camera_metadata_alignment);
using get_camera_metadata_compact_size_fnp =
    decltype(get_camera_metadata_compact_size);
using get_camera_metadata_data_capacity_fnp =
    decltype(get_camera_metadata_data_capacity);
using get_camera_metadata_data_count_fnp =
    decltype(get_camera_metadata_data_count);
using get_camera_metadata_entry_capacity_fnp =
    decltype(get_camera_metadata_entry_capacity);
using get_camera_metadata_entry_count_fnp =
    decltype(get_camera_metadata_entry_count);
using get_camera_metadata_section_name_fnp =
    decltype(get_camera_metadata_section_name);
using get_camera_metadata_tag_name_fnp =
    decltype(get_camera_metadata_tag_name);
using get_camera_metadata_tag_type_fnp =
    decltype(get_camera_metadata_tag_type);
using get_camera_metadata_size_fnp =
    decltype(get_camera_metadata_size);
using sort_camera_metadata_fnp =
    decltype(sort_camera_metadata);
using update_camera_metadata_entry_fnp =
    decltype(update_camera_metadata_entry);
using validate_camera_metadata_structure_fnp =
    decltype(validate_camera_metadata_structure);

typedef int32_t status_t;

namespace android {
  class Parcel;
};

using Parcel = ::android::Parcel;

namespace qmmf {

class VendorTagDescriptor;

/**
 * A convenience wrapper around the C-based camera_metadata_t library.
 */
class CameraMetadata {
  public:
    /** Creates an empty object; best used when expecting to acquire contents
     * from elsewhere */
    CameraMetadata();
    /** Creates an object with space for entryCapacity entries, with
     * dataCapacity extra storage */
    CameraMetadata(size_t entryCapacity, size_t dataCapacity = 10);

    ~CameraMetadata();

    /** Takes ownership of passed-in buffer */
    CameraMetadata(camera_metadata_t *buffer);
    /** Clones the metadata */
    CameraMetadata(const CameraMetadata &other);

    /**
     * Assignment clones metadata buffer.
     */
    CameraMetadata &operator=(const CameraMetadata &other);
    CameraMetadata &operator=(const camera_metadata_t *buffer);

    /**
     * Get reference to the underlying metadata buffer. Ownership remains with
     * the CameraMetadata object, but non-const CameraMetadata methods will not
     * work until unlock() is called. Note that the lock has nothing to do with
     * thread-safety, it simply prevents the camera_metadata_t pointer returned
     * here from being accidentally invalidated by CameraMetadata operations.
     */
    const camera_metadata_t* getAndLock() const;

    /**
     * Unlock the CameraMetadata for use again. After this unlock, the pointer
     * given from getAndLock() may no longer be used. The pointer passed out
     * from getAndLock must be provided to guarantee that the right object is
     * being unlocked.
     */
    status_t unlock(const camera_metadata_t *buffer);

    /**
     * Release a raw metadata buffer to the caller. After this call,
     * CameraMetadata no longer references the buffer, and the caller takes
     * responsibility for freeing the raw metadata buffer (using
     * free_camera_metadata()), or for handing it to another CameraMetadata
     * instance.
     */
    camera_metadata_t* release();

    /**
     * Clear the metadata buffer and free all storage used by it
     */
    void clear();

    /**
     * Acquire a raw metadata buffer from the caller. After this call,
     * the caller no longer owns the raw buffer, and must not free or manipulate it.
     * If CameraMetadata already contains metadata, it is freed.
     */
    void acquire(camera_metadata_t* buffer);

    /**
     * Acquires raw buffer from other CameraMetadata object. After the call, the argument
     * object no longer has any metadata.
     */
    void acquire(CameraMetadata &other);

    /**
     * Append metadata from another CameraMetadata object.
     */
    status_t append(const CameraMetadata &other);

    /**
     * Append metadata from a raw camera_metadata buffer
     */
    status_t append(const camera_metadata* other);

    /**
     * Number of metadata entries.
     */
    size_t entryCount() const;

    /**
     * Is the buffer empty (no entires)
     */
    bool isEmpty() const;

    /**
     * Sort metadata buffer for faster find
     */
    status_t sort();

    /**
     * Update metadata entry. Will create entry if it doesn't exist already, and
     * will reallocate the buffer if insufficient space exists. Overloaded for
     * the various types of valid data.
     */
    status_t update(uint32_t tag,
            const uint8_t *data, size_t data_count);
    status_t update(uint32_t tag,
            const int32_t *data, size_t data_count);
    status_t update(uint32_t tag,
            const float *data, size_t data_count);
    status_t update(uint32_t tag,
            const int64_t *data, size_t data_count);
    status_t update(uint32_t tag,
            const double *data, size_t data_count);
    status_t update(uint32_t tag,
            const camera_metadata_rational_t *data, size_t data_count);
    status_t update(uint32_t tag,
            const std::string string);

    template<typename T>
    status_t update(uint32_t tag, std::vector<T> data) {
        return update(tag, data.data(), data.size());
    }

    /**
     * Base update entry method
     */
    status_t updateImpl(uint32_t tag, const void *data, size_t data_count);

    /**
     * Check if a metadata entry exists for a given tag id
     *
     */
    bool exists(uint32_t tag) const;

    /**
     * Get metadata entry by tag id
     */
    camera_metadata_entry find(uint32_t tag);

    /**
     * Get metadata entry by tag id, with no editing
     */
    camera_metadata_ro_entry find(uint32_t tag) const;

    /**
     * Delete metadata entry by tag
     */
    status_t erase(uint32_t tag);

    /**
     * Swap the underlying camera metadata between this and the other
     * metadata object.
     */
    void swap(CameraMetadata &other);

    /**
     * Dump contents into FD for debugging. The verbosity levels are
     * 0: Tag entry information only, no data values
     * 1: Level 0 plus at most 16 data values per entry
     * 2: All information
     *
     * The indentation parameter sets the number of spaces to add to the start
     * each line of output.
     */
    void dump(int fd, int verbosity = 1, int indentation = 0) const;

#ifdef HAVE_BINDER
    /**
     * Serialization over Binder
     */

    // Metadata object is unchanged when reading from parcel fails.
    status_t readFromParcel(const Parcel *parcel);
    status_t writeToParcel(Parcel *parcel) const;

    /**
      * Caller becomes the owner of the new metadata
      * 'const Parcel' doesnt prevent us from calling the read functions.
      *  which is interesting since it changes the internal state
      *
      * NULL can be returned when no metadata was sent, OR if there was an issue
      * unpacking the serialized data (i.e. bad parcel or invalid structure).
      */
    static status_t readFromParcel(const Parcel &parcel,
                                   camera_metadata_t** out);
    /**
      * Caller retains ownership of metadata
      * - Write 2 (int32 + blob) args in the current position
      */
    static status_t writeToParcel(Parcel &parcel,
                                  const camera_metadata_t* metadata);
#endif

    /**
     * Find tag id for a given tag name, also checking vendor tags if available.
     * On success, returns OK and writes the tag id into tag.
     *
     * This is a slow method.
     */
    static status_t getTagFromName(const char *name,
            const VendorTagDescriptor* vTags, uint32_t *tag);

    static void* libcamera_metadata_handle;
    static add_camera_metadata_entry_fnp* add_camera_metadata_entry;
    static allocate_camera_metadata_fnp* allocate_camera_metadata;
    static allocate_copy_camera_metadata_checked_fnp* allocate_copy_camera_metadata_checked;
    static append_camera_metadata_fnp* append_camera_metadata;
    static calculate_camera_metadata_entry_data_size_fnp* calculate_camera_metadata_entry_data_size;
    static clone_camera_metadata_fnp* clone_camera_metadata;
    static copy_camera_metadata_fnp* copy_camera_metadata;
    static delete_camera_metadata_entry_fnp* delete_camera_metadata_entry;
    static dump_indented_camera_metadata_fnp* dump_indented_camera_metadata;
    static find_camera_metadata_entry_fnp* find_camera_metadata_entry;
    static find_camera_metadata_ro_entry_fnp* find_camera_metadata_ro_entry;
    static free_camera_metadata_fnp* free_camera_metadata;
    static get_camera_metadata_alignment_fnp* get_camera_metadata_alignment;
    static get_camera_metadata_compact_size_fnp* get_camera_metadata_compact_size;
    static get_camera_metadata_data_capacity_fnp* get_camera_metadata_data_capacity;
    static get_camera_metadata_data_count_fnp* get_camera_metadata_data_count;
    static get_camera_metadata_entry_capacity_fnp* get_camera_metadata_entry_capacity;
    static get_camera_metadata_entry_count_fnp* get_camera_metadata_entry_count;
    static get_camera_metadata_section_name_fnp* get_camera_metadata_section_name;
    static get_camera_metadata_tag_name_fnp* get_camera_metadata_tag_name;
    static get_camera_metadata_tag_type_fnp* get_camera_metadata_tag_type;
    static get_camera_metadata_size_fnp* get_camera_metadata_size;
    static sort_camera_metadata_fnp* sort_camera_metadata;
    static update_camera_metadata_entry_fnp* update_camera_metadata_entry;
    static validate_camera_metadata_structure_fnp* validate_camera_metadata_structure;

    static unsigned int** camera_metadata_section_bounds;
    static const char** camera_metadata_section_names;
    static const char** camera_metadata_type_names;

  private:
    camera_metadata_t *mBuffer;
    mutable bool       mLocked;

    /**
     * Check if tag has a given type
     */
    status_t checkType(uint32_t tag, uint8_t expectedType);

    /**
     * Resize metadata buffer if needed by reallocating it and copying it over.
     */
    status_t resizeIfNeeded(size_t extraEntries, size_t extraData);

};

}; // namespace qmmf

using CameraMetadata = ::qmmf::CameraMetadata;

#endif
