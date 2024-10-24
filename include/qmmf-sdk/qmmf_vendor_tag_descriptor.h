/*
 * Copyright (C) 2014 The Android Open Source Project
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

#ifndef QMMF_VENDOR_TAG_DESCRIPTOR_H

#include <system/camera_metadata.h>
#include <system/camera_vendor_tags.h>
#include <memory>
#include <map>
#include <vector>
#include <mutex>

#include <stdint.h>

namespace android {
  class Parcel;
};

typedef int32_t status_t;

namespace qmmf {

using Parcel = ::android::Parcel;

/**
 * VendorTagDescriptor objects are parcelable containers for the vendor tag
 * definitions provided, and are typically used to pass the vendor tag
 * information enumerated by the HAL to clients of the camera service.
 */
class VendorTagDescriptor {
    public:
        virtual ~VendorTagDescriptor();

        /**
         * The following 'get*' methods implement the corresponding
         * functions defined in
         * system/media/camera/include/system/camera_vendor_tags.h
         */

        // Returns the number of vendor tags defined.
        int getTagCount() const;

        // Returns an array containing the id's of vendor tags defined.
        void getTagArray(uint32_t* tagArray) const;

        // Returns the section name string for a given vendor tag id.
        const char* getSectionName(uint32_t tag) const;

        // Returns the tag name string for a given vendor tag id.
        const char* getTagName(uint32_t tag) const;

        // Returns the tag type for a given vendor tag id.
        int getTagType(uint32_t tag) const;

#ifdef HAVE_BINDER
        /**
         * Write the VendorTagDescriptor object into the given parcel.
         *
         * Returns OK on success, or a negative error code.
         */
        status_t writeToParcel(
                /*out*/
                Parcel* parcel);
#endif

        /**
         * Convenience method to get a vector containing all vendor tag
         * sections, or an empty vector if none are defined.
         */
        std::vector<std::string> getAllSectionNames() const;

        /**
         * Lookup the tag id for a given tag name and section.
         *
         * Returns OK on success, or a negative error code.
         */
        status_t lookupTag(std::string name, std::string section, /*out*/uint32_t* tag) const;

        /**
         * Dump the currently configured vendor tags to a file descriptor.
         */
        void dump(int fd, int verbosity, int indentation) const;

        // Static methods:
#ifdef HAVE_BINDER
        /**
         * Create a VendorTagDescriptor object from the given parcel.
         *
         * Returns OK on success, or a negative error code.
         */
        static status_t createFromParcel(const Parcel* parcel,
                /*out*/
                std::shared_ptr<VendorTagDescriptor>& descriptor);

        /**
         * Read values VendorTagDescriptor object from the given parcel.
         *
         * Returns OK on success, or a negative error code.
         */
        virtual status_t readFromParcel(const android::Parcel* parcel);
#endif
        /**
         * Read values VendorTagDescriptor object from the given Buffer.
         *
         * Returns OK on success, or a negative error code.
         */
        int32_t readFromBuffer(const uint8_t *in);

         /**
         * Calculates reguired Buffer size to store this object.
         *
         * Returns the size.
         */
        size_t getBufferSize();

        /**
         * Writes values VendorTagDescriptor object to the given Buffer.
         *
         * Returns OK on success, or a negative error code.
         */
        int32_t writeToBuffer(uint8_t out[], size_t size);

        /**
         * Create a VendorTagDescriptor object from the given vendor_tag_ops_t
         * struct.
         *
         * Returns OK on success, or a negative error code.
         */
        static status_t createDescriptorFromOps(const vendor_tag_ops_t* vOps,
                /*out*/
                std::shared_ptr<VendorTagDescriptor>& descriptor);

        /**
         * Sets the global vendor tag descriptor to use for this process.
         * Camera metadata operations that access vendor tags will use the
         * vendor tag definitions set this way.
         *
         * Returns OK on success, or a negative error code.
         */
        static status_t setAsGlobalVendorTagDescriptor(
            const std::shared_ptr<VendorTagDescriptor>& desc);

        /**
         * Clears the global vendor tag descriptor used by this process.
         */
        static void clearGlobalVendorTagDescriptor();

        /**
         * Returns the global vendor tag descriptor used by this process.
         * This will contain NULL if no vendor tags are defined.
         */
        static std::shared_ptr<VendorTagDescriptor> getGlobalVendorTagDescriptor();

        static void* libcamera_metadata_handle;
        static const char** camera_metadata_type_names;

    protected:
        struct tag_detail {
          std::string sectionName, tagName;
          int32_t tagType;
        };
        // tag ids
        std::vector<uint32_t> mtagArray;
        // tag details
        std::map<uint32_t, tag_detail> mtagArrayDetail;
        // map from section name to map of tag name to tag id
        // for easy searching
        std::map<std::string, std::map<std::string, uint32_t>>
            mReverseMapping;
        // lexicographically ordered section names
        std::vector<std::string> mSections;
    private:
        vendor_tag_ops mVendorOps;
        struct tag_detail_raw {
          uint32_t tag, tagType, sectionIndex;
          size_t sztagName;
          char tagName[1];
        };
        struct sect_name {
          size_t szsectName;
          char sectName[1];
        };
};

} /* namespace qmmf */

using VendorTagDescriptor = ::qmmf::VendorTagDescriptor;

#define QMMF_VENDOR_TAG_DESCRIPTOR_H
#endif /* QMMF_VENDOR_TAG_DESCRIPTOR_H */
