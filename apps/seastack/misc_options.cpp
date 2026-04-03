/**
 * @file misc_options.cpp
 * @brief Hidden miscellaneous CLI options handler (e.g., special outputs)
 */

#include "misc_options.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace seastack::app {

static std::string ReadAsciiFromFile(const std::string& path) {
    std::ifstream file(path, std::ios::in | std::ios::binary);
    if (!file.is_open()) {
        return {};
    }
    std::ostringstream ss;
    ss << file.rdbuf();
    return ss.str();
}

static std::string LoadBannerAsset() {
    const char* candidates[] = {
        "./utils/term_layout.txt"
    };
    for (const char* c : candidates) {
        std::string s = ReadAsciiFromFile(c);
        if (!s.empty()) return s;
    }
    return {};
}

bool HandleHiddenOptions(int argc, char* argv[]) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--salter") {
            std::string art = LoadBannerAsset();
            if (art.empty()) {
                return false;
            }
            std::cout << art << std::endl;
            return true;
        }
    }
    return false;
}

} // namespace seastack::app
