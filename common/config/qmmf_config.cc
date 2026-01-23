/*
* Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
* SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#define LOG_TAG "CommonConfig"
#include <fstream>
#include <vector>
#include <string>
#include <cstring>
#include <cctype>
#include "common/utils/qmmf_log.h"

// Helper function to trim whitespace from strings
static std::string trim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    size_t end = s.find_last_not_of(" \t\r\n");
    return (start == std::string::npos) ? "" : s.substr(start, end - start + 1);
}

int qmmf_property_get(const char *key, char *value, const char *default_value) {
    std::ifstream file(QMMF_CONFIG_FILE);
    if (!file.is_open()) {
        strncpy(value, default_value, QMMF_PROP_VAL_MAX - 1);
        value[QMMF_PROP_VAL_MAX - 1] = '\0';
        QMMF_WARN("Error loading %s file. Using default val", QMMF_CONFIG_FILE);
        return -1;
    }

    std::string current_section;
    std::string line;

    while (std::getline(file, line)) {
        // Remove comments (anything after # or ;)
        size_t comment_pos = line.find_first_of("#;");
        if (comment_pos != std::string::npos) {
            line = line.substr(0, comment_pos);
        }

        line = trim(line);
        if (line.empty()) continue;

        if (line[0] == '[' && line[line.size() - 1] == ']') {
            current_section = line.substr(1, line.size() - 2);
            continue;
        }

        // Process entries in [default] section
        if (current_section != "default") continue;

        size_t eq_pos = line.find('=');
        if (eq_pos == std::string::npos) continue;

        std::string k = trim(line.substr(0, eq_pos));
        std::string v = trim(line.substr(eq_pos + 1));

        if (k == key) {
            strncpy(value, v.c_str(), QMMF_PROP_VAL_MAX - 1);
            value[QMMF_PROP_VAL_MAX - 1] = '\0';
            return 0;
        }
    }

    // Key not found in configuration file
    strncpy(value, default_value, QMMF_PROP_VAL_MAX - 1);
    value[QMMF_PROP_VAL_MAX - 1] = '\0';
    QMMF_WARN("Error loading config %s. Using default val", key);
    return -1;
}

int qmmf_property_set(const char *key, const char *value) {
    std::vector<std::string> lines;
    std::string line;
    std::ifstream infile(QMMF_CONFIG_FILE);

    // Read file content if file exists
    if (infile.is_open()) {
        while (std::getline(infile, line)) {
            lines.push_back(line);
        }
        infile.close();
    }

    bool section_found = false;
    size_t section_start = 0;
    size_t section_end = lines.size();

    // Search for [default] section
    for (size_t i = 0; i < lines.size(); ++i) {
        std::string trimmed_line = trim(lines[i]);

        if (trimmed_line.size() >= 2 &&
            trimmed_line[0] == '[' &&
            trimmed_line[trimmed_line.size() - 1] == ']') {

            std::string section_name = trimmed_line.substr(1, trimmed_line.size() - 2);

            if (section_name == "default") {
                section_found = true;
                section_start = i;

                section_end = lines.size();
                for (size_t j = i + 1; j < lines.size(); ++j) {
                    std::string next_line = trim(lines[j]);
                    if (next_line.size() >= 2 &&
                        next_line[0] == '[' &&
                        next_line[next_line.size() - 1] == ']') {
                    section_end = j;
                    break;
                    }
                }
                break; // Exit section search loop after finding [default]
            }
        }
    }

    if (section_found) {
        bool key_updated = false;
        // Search within the section for the key
        for (size_t i = section_start + 1; i < section_end; ++i) {
            size_t eq_pos = lines[i].find('=');
            if (eq_pos != std::string::npos) {
                std::string existing_key = trim(lines[i].substr(0, eq_pos));
                if (existing_key == key) {
                    lines[i] = std::string(key) + " = " + value;
                    key_updated = true;
                    break;
                }
            }
        }
        if (!key_updated) {
            // Insert new key at the end of the section
            lines.insert(lines.begin() + section_end, std::string(key) + " = " + value);
        }
    } else {
        // Create new [default] section
        if (!lines.empty() && !lines.back().empty()) {
            lines.push_back("");
        }
        lines.push_back("[default]");
        lines.push_back(std::string(key) + " = " + value);
    }

    // Write to config file
    std::ofstream outfile(QMMF_CONFIG_FILE);
    if (!outfile.is_open()) {
        QMMF_WARN("Error opening %s for writing", QMMF_CONFIG_FILE);
        return -1;
    }
    for (const auto& out_line : lines) {
        outfile << out_line << '\n';
    }
    if (!outfile) {
        QMMF_WARN("Error writing to %s", QMMF_CONFIG_FILE);
        return -1;
    }
    return 0;
}
