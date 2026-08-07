/*********************************************************************
 * @file  yaml_read_helpers.h
 * @brief Scalar readers shared by the app-layer YAML scenario parsers.
 *
 * Every reader leaves `out` untouched when the key is absent, so a caller
 * can initialise `out` with its default and read over it unconditionally.
 * A malformed value still throws YAML::TypedBadConversion, because a key
 * that is present but unreadable is an input error, not a default.
 *********************************************************************/

#ifndef SEASTACK_APP_YAML_READ_HELPERS_H
#define SEASTACK_APP_YAML_READ_HELPERS_H

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cstddef>
#include <string>

namespace seastack::app::yaml_read {

inline std::string ToUpper(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return s;
}

inline void ReadDouble(const YAML::Node& node, const char* key, double& out) {
    if (node && node[key]) {
        out = node[key].as<double>();
    }
}

inline void ReadInt(const YAML::Node& node, const char* key, int& out) {
    if (node && node[key]) {
        out = node[key].as<int>();
    }
}

inline void ReadString(const YAML::Node& node, const char* key, std::string& out) {
    if (node && node[key]) {
        out = node[key].as<std::string>();
    }
}

inline void ReadBool(const YAML::Node& node, const char* key, bool& out) {
    if (node && node[key]) {
        out = node[key].as<bool>();
    }
}

/// Read a fixed-length numeric sequence.  A sequence of the wrong length is
/// ignored rather than truncated, so a mis-sized vector keeps the default
/// instead of silently changing geometry.
template <std::size_t N>
void ReadArray(const YAML::Node& node, const char* key, std::array<double, N>& out) {
    if (node && node[key] && node[key].IsSequence() && node[key].size() == N) {
        for (std::size_t i = 0; i < N; ++i) {
            out[i] = node[key][i].as<double>();
        }
    }
}

}  // namespace seastack::app::yaml_read

#endif  // SEASTACK_APP_YAML_READ_HELPERS_H
