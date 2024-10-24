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

#define LOG_TAG "VendorTagDescriptor"

#ifdef HAVE_BINDER
#include <binder/Parcel.h>
#include <utils/String8.h>
#endif

#include "qmmf-sdk/qmmf_vendor_tag_descriptor.h"
#include <common/utils/qmmf_log.h>

#include <stdio.h>
#include <string.h>
#include <mutex>
#include <algorithm>
#include <cstdlib>
#include <forward_list>
#include <dlfcn.h>

#ifdef HAVE_BINDER
using namespace android;
#endif

extern "C" {
int set_camera_metadata_vendor_ops(const vendor_tag_ops_t *query_ops);
} /* extern "C" */

using set_camera_metadata_vendor_ops_fnp = decltype(set_camera_metadata_vendor_ops);

set_camera_metadata_vendor_ops_fnp*
    libcamera_metadata_set_camera_metadata_vendor_ops = NULL;

namespace qmmf {

extern "C" {

static int vendor_tag_descriptor_get_tag_count(const vendor_tag_ops_t* v);
static void vendor_tag_descriptor_get_all_tags(const vendor_tag_ops_t* v, uint32_t* tagArray);
static const char* vendor_tag_descriptor_get_section_name(const vendor_tag_ops_t* v, uint32_t tag);
static const char* vendor_tag_descriptor_get_tag_name(const vendor_tag_ops_t* v, uint32_t tag);
static int vendor_tag_descriptor_get_tag_type(const vendor_tag_ops_t* v, uint32_t tag);
} /* extern "C" */

void* VendorTagDescriptor::libcamera_metadata_handle = NULL;
const char** VendorTagDescriptor::camera_metadata_type_names = NULL;

static std::mutex sLock;
static std::shared_ptr<VendorTagDescriptor> sGlobalVendorTagDescriptor;

void VendorTagDescriptor_libCameraMetadataOpen() __attribute__ ((constructor (101)));
void VendorTagDescriptor_libCameraMetadataClose() __attribute__ ((destructor (101)));

void VendorTagDescriptor_libCameraMetadataOpen()
{
    if (NULL == VendorTagDescriptor::libcamera_metadata_handle) {
        VendorTagDescriptor::libcamera_metadata_handle =
            dlopen("libcamera_metadata.so", RTLD_LAZY);
        char* err = dlerror();

        if ((NULL != VendorTagDescriptor::libcamera_metadata_handle) && (NULL == err)) {
            libcamera_metadata_set_camera_metadata_vendor_ops =
                reinterpret_cast<set_camera_metadata_vendor_ops_fnp*>(
                dlsym(VendorTagDescriptor::libcamera_metadata_handle,
                "set_camera_metadata_vendor_ops"));
            VendorTagDescriptor::camera_metadata_type_names =
                reinterpret_cast<const char**>(
                dlsym(VendorTagDescriptor::libcamera_metadata_handle,
                "camera_metadata_type_names"));
            char* dlsym_err = dlerror();
            if (dlsym_err != NULL) {
                assert(libcamera_metadata_set_camera_metadata_vendor_ops);
                assert(VendorTagDescriptor::camera_metadata_type_names);
            }
        }
    }
}

void VendorTagDescriptor_libCameraMetadataClose()
{
    if (VendorTagDescriptor::libcamera_metadata_handle != NULL) {
      dlclose(VendorTagDescriptor::libcamera_metadata_handle);
    }
}

VendorTagDescriptor::~VendorTagDescriptor() {}

int32_t VendorTagDescriptor::createDescriptorFromOps(
    const vendor_tag_ops_t* vOps,
    /*out*/
    std::shared_ptr<VendorTagDescriptor>& descriptor) {
    if (vOps == NULL) {
        QMMF_ERROR("%s: vendor_tag_ops argument was NULL.", __FUNCTION__);
        return -1;
    }

    int tagCount = vOps->get_tag_count(vOps);
    if (tagCount < 0 || tagCount > INT32_MAX) {
        QMMF_ERROR("%s: tag count %d from vendor ops is invalid.", __FUNCTION__, tagCount);
        return -1;
    }

    VendorTagDescriptor tmpDesc;

    tmpDesc.mtagArray.resize(tagCount);
    vOps->get_all_tags(vOps, /*out*/tmpDesc.mtagArray.data());

    for (size_t i = 0; i < static_cast<size_t>(tagCount); ++i) {
        uint32_t tag = tmpDesc.mtagArray[i];
        if (tag < CAMERA_METADATA_VENDOR_TAG_BOUNDARY) {
            QMMF_ERROR("%s: vendor tag %d not in vendor tag section.",
                __FUNCTION__, tag);
            return -1;
        }

        const char *tagName = vOps->get_tag_name(vOps, tag);
        if (tagName == NULL) {
            QMMF_ERROR("%s: no tag name defined for vendor tag %d.",
                __FUNCTION__, tag);
            return -1;
        }

        const char *sectionName = vOps->get_section_name(vOps, tag);
        if (sectionName == NULL) {
            QMMF_ERROR("%s: no section name defined for vendor tag %d.",
                __FUNCTION__, tag);
            return -1;
        }

        int tagType = vOps->get_tag_type(vOps, tag);
        if (tagType < 0 || tagType >= NUM_TYPES) {
            QMMF_ERROR("%s: tag type %d from vendor ops does not exist.",
                __FUNCTION__, tagType);
            return -1;
        }

        tmpDesc.mtagArrayDetail[tag] = tag_detail{sectionName, tagName, tagType};
    }

    for (auto &tag_detail_pair : tmpDesc.mtagArrayDetail) {
        auto tag_detail = tag_detail_pair.second;
        tmpDesc.mReverseMapping[tag_detail.sectionName][tag_detail.tagName]
            = tag_detail_pair.first;
    }

    tmpDesc.mSections.resize(tmpDesc.mReverseMapping.size());

    // assign c strs from reverse mapping map since they are already sorted
    // in it
    auto iter_sect = tmpDesc.mSections.begin();
    for (auto &map_pair : tmpDesc.mReverseMapping) {
        *iter_sect++ = map_pair.first;
    }

    // Move tmpDesc because it's going to be destructed
    descriptor = std::make_shared<VendorTagDescriptor>(std::move(tmpDesc));
    return 0;
}

int32_t VendorTagDescriptor::readFromBuffer(const uint8_t *in_buf) {
    int32_t res = 0;
    if (in_buf == NULL) {
        QMMF_ERROR("%s: in argument was NULL.", __FUNCTION__);
        return -1;
    }

    const uint8_t *current = in_buf;
    auto &tagCountPtrRef = reinterpret_cast<const uint32_t*&>(current);
    uint32_t tagCount = *tagCountPtrRef++;
    if (tagCount < 0 || tagCount > INT32_MAX) {
        QMMF_ERROR("%s: tag count %d from vendor ops is invalid.",
            __FUNCTION__, tagCount);
        return -1;
    }

    std::map<uint32_t, std::forward_list<uint32_t>> sectionToTagsIndices;
    for (uint32_t i = 0; i < tagCount; ++i) {
        auto &det_raw_ptr_ref = reinterpret_cast<const tag_detail_raw*&>(
            current);
        auto &det_raw = *det_raw_ptr_ref++;
        if (det_raw.tag < CAMERA_METADATA_VENDOR_TAG_BOUNDARY) {
            QMMF_ERROR("%s: vendor tag %d not in vendor tag section.",
                __FUNCTION__, det_raw.tag);
            res = -1;
            break;
        }
        if (det_raw.tagType < 0 || det_raw.tagType >= NUM_TYPES) {
            QMMF_ERROR("%s: tag type %d from vendor ops does not exist.",
                __FUNCTION__, det_raw.tagType);
            res = -1;
            break;
        }
        if (det_raw.sztagName == 0) {
            QMMF_ERROR("%s: parcel tag name was NULL for tag %d.",
                __FUNCTION__, det_raw.tag);
            res = -1;
            break;
        }
        current += det_raw.sztagName - 1;
        tag_detail det = {{}, det_raw.tagName, (int32_t)det_raw.tagType};
        mtagArray.push_back(det_raw.tag);
        mtagArrayDetail[det_raw.tag] = det;
        sectionToTagsIndices[det_raw.sectionIndex].push_front(det_raw.tag);
    }

    if (res != 0) {
        QMMF_ERROR("%s: failed with %d.", __FUNCTION__, res);
        return res;
    }

    uint32_t sectionCount = 0;
    if (tagCount > 0) {
        auto &sectionCountPtrRef = reinterpret_cast<const uint32_t*&>(current);
        sectionCount = *sectionCountPtrRef++;
        mSections.resize(sectionCount);
        for (uint32_t i_section = 0; i_section < sectionCount; ++i_section) {
            auto &sec_ptr_ref = reinterpret_cast<const sect_name*&>(current);
            const auto &sec = *sec_ptr_ref++;
            std::string sectName_string = sec.sectName;
            current += sec.szsectName - 1;
            auto &revNamesMap = mReverseMapping[sectName_string];
            auto &sectionTags = sectionToTagsIndices[i_section];
            for (auto &tag : sectionTags) {
              auto &det = mtagArrayDetail[tag];
              mReverseMapping[sectName_string][det.tagName] = tag;
              det.sectionName = sectName_string;
            }
        }
    }

    // assign c strs from reverse mapping map since they are already sorted
    // in it
    auto iter_sect = mSections.begin();
    for (auto &map_pair : mReverseMapping) {
        *iter_sect++ = map_pair.first;
    }

    return res;
}

int32_t VendorTagDescriptor::writeToBuffer(uint8_t out_buf[], size_t size) {
    status_t res = 0;
    if (out_buf==NULL) {
        QMMF_ERROR("%s: out argument was NULL.", __FUNCTION__);
        return -1;
    }

    uint8_t *current = out_buf;

    uint32_t TagCount = mtagArray.size();
    if ((out_buf + size) - current < sizeof(int32_t)) {
        QMMF_ERROR("%s: buffer overflow, buffer size is %zu, offset is 0",
            __FUNCTION__, size);
        return -1;
    }
    auto &TagCountPtrRef = reinterpret_cast<int32_t*&>(current);
    *TagCountPtrRef++ = TagCount;

    uint32_t tag, sectionIndex;
    uint32_t tagType;
    for (size_t i = 0; i < TagCount; ++i) {
        tag = mtagArray[i];
        auto &det = mtagArrayDetail[(uint32_t)tag];
        sectionIndex = std::distance(mReverseMapping.begin(),
            mReverseMapping.find(det.sectionName));
        tagType = det.tagType;
        auto &det_raw_ptr_ref = reinterpret_cast<tag_detail_raw*&>(current);
        auto &det_raw = *det_raw_ptr_ref++;
        det_raw = tag_detail_raw{
            tag, tagType, sectionIndex, det.tagName.size() + 1};
        memcpy(det_raw.tagName, det.tagName.c_str(), det_raw.sztagName);
        current += det_raw.sztagName - 1;
    }

    int32_t numSections = (int32_t)mReverseMapping.size();
    if (numSections > 0) {
        if ((out_buf + size) - current < sizeof(int32_t)) {
            QMMF_ERROR("%s: buffer overflow, buffer size is %zu, offset is %zu",
                __FUNCTION__, size, current - out_buf);
            return -1;
        }
        auto &numSectionsPtrRef = reinterpret_cast<int32_t*&>(current);
        *numSectionsPtrRef++ = numSections;
        for (auto &pair : mReverseMapping) {
            auto &sec_ptr_ref = reinterpret_cast<sect_name*&>(current);
            auto &sec = *sec_ptr_ref++;
            sec.szsectName = pair.first.size() + 1;
            memcpy(sec.sectName, pair.first.c_str(), sec.szsectName);
            current += sec.szsectName - 1;
        }
    }

    if (current - out_buf > size) {
        QMMF_ERROR("%s: buffer corruption, buffer size is %zu, offset is %zu",
            __FUNCTION__, size, current - out_buf);
        return -1;
    }

    return res;
}

size_t VendorTagDescriptor::getBufferSize() {
    size_t size = mtagArray.size() * sizeof(tag_detail_raw);
    for (const auto &tag : mtagArray)
      size += mtagArrayDetail[tag].tagName.size();
    size += mReverseMapping.size() * sizeof(sect_name);
    for (const auto &pair : mReverseMapping)
      size += pair.first.size();
    size += sizeof(int32_t) * 2;
    return size;
}

#ifdef HAVE_BINDER
int32_t VendorTagDescriptor::readFromParcel(const Parcel* parcel) {
    int32_t res = 0;
    if (parcel == NULL) {
        QMMF_ERROR("%s: parcel argument was NULL.", __FUNCTION__);
        return -1;
    }
    const size_t len = parcel->dataAvail();
    uint8_t *buff = new uint8_t[len];
    if ((res = parcel->read(buff, len)) != OK ||
        (res = readFromBuffer(buff)) != OK) {
        QMMF_ERROR("%s: failed with %d", __FUNCTION__, res);
    }
    delete[] buff;
    return res;
}

int32_t VendorTagDescriptor::createFromParcel(const Parcel* parcel,
    std::shared_ptr<VendorTagDescriptor>& descriptor) {
    VendorTagDescriptor tmpDesc;

    // If non zero readFromParcel has returned an error
    if (int32_t err = tmpDesc.readFromParcel(parcel)) {
        QMMF_ERROR("%s: could not create from parcel.", __FUNCTION__);
        return err;
    }

    // Move tmpDesc because it's going to be destructed
    descriptor = std::make_shared<VendorTagDescriptor>(std::move(tmpDesc));
    return 0;
}
#endif
int VendorTagDescriptor::getTagCount() const {
    size_t size = mtagArray.size();
    if (size == 0) {
        return -1;
    }
    return size;
}

void VendorTagDescriptor::getTagArray(uint32_t* tagArray) const {
    size_t size = mtagArray.size();
    memcpy(tagArray, mtagArray.data(), size * sizeof(uint32_t));
}

const char* VendorTagDescriptor::getSectionName(uint32_t tag) const {
    auto det_iter = mtagArrayDetail.find(tag);
    if (det_iter == mtagArrayDetail.end()) {
        return NULL;
    }
    return det_iter->second.sectionName.c_str();
}

const char* VendorTagDescriptor::getTagName(uint32_t tag) const {
    auto det_iter = mtagArrayDetail.find(tag);
    if (det_iter == mtagArrayDetail.end()) {
        return NULL;
    }
    return det_iter->second.tagName.c_str();
}

int VendorTagDescriptor::getTagType(uint32_t tag) const {
    auto det_iter = mtagArrayDetail.find(tag);
    if (det_iter == mtagArrayDetail.end()) {
        return -1;
    }
    return det_iter->second.tagType;
}
#ifdef HAVE_BINDER
int32_t VendorTagDescriptor::writeToParcel(Parcel* parcel) {
    status_t res = OK;
    size_t len = getBufferSize();
    uint8_t *buffer = new uint8_t[len];
    if ((res = writeToBuffer(buffer, len)) != OK) {
        QMMF_ERROR("%s: can't write to buffer %d, len %zu", __FUNCTION__, res,
          len);
        return res;
    }
    if ((res = parcel->write(buffer, len)) != OK) {
        QMMF_ERROR("%s: failed to write buffer to parcel", __FUNCTION__);
        return res;
    }
    delete[] buffer;
    return res;
}
#endif
std::vector<std::string> VendorTagDescriptor::getAllSectionNames() const {
    return mSections;
}

int32_t VendorTagDescriptor::lookupTag(std::string name, std::string section,
    /*out*/uint32_t* tag) const {
    auto iter_sect_map = mReverseMapping.find(section);
    if (iter_sect_map == mReverseMapping.end()) {
        QMMF_ERROR("%s: Section '%s' does not exist.", __FUNCTION__,
            section.c_str());
        return -1;
    }

    auto iter_name_map = iter_sect_map->second.find(name);
    if (iter_name_map == iter_sect_map->second.end()) {
        QMMF_ERROR("%s: Tag name '%s' does not exist.", __FUNCTION__,
            name.c_str());
        return -1;
    }

    if (tag != NULL) {
        *tag = iter_name_map->second;
    }
    return 0;
}

void VendorTagDescriptor::dump(int fd, int verbosity, int indentation) const {

    size_t size = mtagArray.size();
    if (size == 0) {
        dprintf(fd, "%*sDumping configured vendor tag descriptors: None set\n",
                indentation, "");
        return;
    }

    dprintf(fd, "%*sDumping configured vendor tag descriptors: %zu entries\n",
            indentation, "", size);
    for (size_t i = 0; i < size; ++i) {
        uint32_t tag = mtagArray[i];

        if (verbosity < 1) {
            dprintf(fd, "%*s0x%x\n", indentation + 2, "", tag);
            continue;
        }
        auto &desc = mtagArrayDetail.at(tag);
        const char* typeName = (desc.tagType >= 0 && desc.tagType < NUM_TYPES) ?
                VendorTagDescriptor::camera_metadata_type_names[desc.tagType] : "UNKNOWN";
        dprintf(fd, "%*s0x%x (%s) with type %d (%s) defined in section %s\n", indentation + 2,
            "", tag, desc.tagName.c_str(), desc.tagType, typeName, desc.sectionName.c_str());
    }

}

int32_t VendorTagDescriptor::setAsGlobalVendorTagDescriptor(
    const std::shared_ptr<VendorTagDescriptor>& desc) {
    status_t res = 0;
    std::unique_lock<std::mutex> unl{sLock};
    sGlobalVendorTagDescriptor = desc;

    vendor_tag_ops_t* opsPtr = NULL;
    if (desc != NULL) {
        opsPtr = &(desc->mVendorOps);
        opsPtr->get_tag_count = vendor_tag_descriptor_get_tag_count;
        opsPtr->get_all_tags = vendor_tag_descriptor_get_all_tags;
        opsPtr->get_section_name = vendor_tag_descriptor_get_section_name;
        opsPtr->get_tag_name = vendor_tag_descriptor_get_tag_name;
        opsPtr->get_tag_type = vendor_tag_descriptor_get_tag_type;
    }
    if((res = libcamera_metadata_set_camera_metadata_vendor_ops(opsPtr)) != 0) {
        QMMF_ERROR("%s: Could not set vendor tag descriptor, received error %s (%d)."
                , __FUNCTION__, strerror(-res), res);
    }
    return res;
}

void VendorTagDescriptor::clearGlobalVendorTagDescriptor() {
    std::unique_lock<std::mutex> unl{sLock};
    libcamera_metadata_set_camera_metadata_vendor_ops(NULL);
    sGlobalVendorTagDescriptor.reset();
}

std::shared_ptr<VendorTagDescriptor> VendorTagDescriptor::getGlobalVendorTagDescriptor() {
    std::unique_lock<std::mutex> unl{sLock};
    return sGlobalVendorTagDescriptor;
}

extern "C" {

int vendor_tag_descriptor_get_tag_count(const vendor_tag_ops_t* /*v*/) {
    std::unique_lock<std::mutex> unl{sLock};
    if (!sGlobalVendorTagDescriptor) {
        QMMF_ERROR("%s: Vendor tag descriptor not initialized.", __FUNCTION__);
        return -1;
    }
    return sGlobalVendorTagDescriptor->getTagCount();
}

void vendor_tag_descriptor_get_all_tags(const vendor_tag_ops_t* /*v*/, uint32_t* tagArray) {
    std::unique_lock<std::mutex> unl{sLock};
    if (!sGlobalVendorTagDescriptor) {
        QMMF_ERROR("%s: Vendor tag descriptor not initialized.", __FUNCTION__);
        return;
    }
    sGlobalVendorTagDescriptor->getTagArray(tagArray);
}

const char* vendor_tag_descriptor_get_section_name(const vendor_tag_ops_t* /*v*/, uint32_t tag) {
    std::unique_lock<std::mutex> unl{sLock};
    if (!sGlobalVendorTagDescriptor) {
        QMMF_ERROR("%s: Vendor tag descriptor not initialized.", __FUNCTION__);
        return NULL;
    }
    return sGlobalVendorTagDescriptor->getSectionName(tag);
}

const char* vendor_tag_descriptor_get_tag_name(const vendor_tag_ops_t* /*v*/, uint32_t tag) {
    std::unique_lock<std::mutex> unl{sLock};
    if (!sGlobalVendorTagDescriptor) {
        QMMF_ERROR("%s: Vendor tag descriptor not initialized.", __FUNCTION__);
        return NULL;
    }
    return sGlobalVendorTagDescriptor->getTagName(tag);
}

int vendor_tag_descriptor_get_tag_type(const vendor_tag_ops_t* /*v*/, uint32_t tag) {
    std::unique_lock<std::mutex> unl{sLock};
    if (!sGlobalVendorTagDescriptor) {
        QMMF_ERROR("%s: Vendor tag descriptor not initialized.", __FUNCTION__);
        return -1;
    }
    return sGlobalVendorTagDescriptor->getTagType(tag);
}

} /* extern "C" */
} /* namespace qmmf */
