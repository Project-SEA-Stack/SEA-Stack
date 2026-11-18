/*********************************************************************
 * @file  yaml_read_helpers.h
 * @brief Strict readers shared by the app-layer YAML scenario parsers.
 *
 * Every reader leaves `out` untouched when the key is absent, so a caller
 * can initialise `out` with its default and read over it unconditionally.
 * A key that is present but malformed (wrong type, wrong length) throws
 * std::runtime_error naming the key path: a present but unreadable value is
 * an input error, never a silent default.
 *
 * `where` is a human-readable location prefix for messages, for example
 * "structure_file bridge.structure.yaml: beams[2]".
 *********************************************************************/

#ifndef SEASTACK_APP_YAML_READ_HELPERS_H
#define SEASTACK_APP_YAML_READ_HELPERS_H

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace seastack::app::yaml_read {

inline std::string ToUpper(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return s;
}

inline std::size_t EditDistance(const std::string& a, const std::string& b) {
    std::vector<std::size_t> prev(b.size() + 1);
    std::vector<std::size_t> cur(b.size() + 1);
    for (std::size_t j = 0; j <= b.size(); ++j) {
        prev[j] = j;
    }
    for (std::size_t i = 1; i <= a.size(); ++i) {
        cur[0] = i;
        for (std::size_t j = 1; j <= b.size(); ++j) {
            const std::size_t sub = prev[j - 1] + (a[i - 1] == b[j - 1] ? 0 : 1);
            cur[j] = std::min({prev[j] + 1, cur[j - 1] + 1, sub});
        }
        std::swap(prev, cur);
    }
    return prev[b.size()];
}

/// " Did you mean 'x'?" for a close candidate, else "".
inline std::string DidYouMean(const std::string& word, const std::vector<std::string>& candidates) {
    std::string best;
    std::size_t best_d = std::numeric_limits<std::size_t>::max();
    const std::size_t max_d = std::max<std::size_t>(2, word.size() / 3);
    for (const auto& c : candidates) {
        const std::size_t d = EditDistance(word, c);
        if (d <= max_d && d < best_d) {
            best_d = d;
            best = c;
        }
    }
    return best.empty() ? std::string() : " Did you mean '" + best + "'?";
}

inline std::string JoinNames(const std::vector<std::string>& names) {
    if (names.empty()) {
        return "(none)";
    }
    std::string out;
    for (std::size_t i = 0; i < names.size(); ++i) {
        out += (i == 0 ? "" : ", ") + names[i];
    }
    return out;
}

/// Throw unless `node` is a map.
inline void RequireMap(const YAML::Node& node, const std::string& where) {
    if (!node || !node.IsMap()) {
        throw std::runtime_error(where + " must be a map of key: value entries.");
    }
}

/// Throw unless `node` is a sequence (list).
inline void RequireSequence(const YAML::Node& node, const std::string& where) {
    if (!node || !node.IsSequence()) {
        throw std::runtime_error(where + " must be a list.");
    }
}

/// Throw on any key of `map` that is not in `allowed`.
inline void CheckKeys(const YAML::Node& map, const std::vector<std::string>& allowed,
                      const std::string& where) {
    RequireMap(map, where);
    for (const auto& kv : map) {
        const std::string key = kv.first.as<std::string>();
        if (std::find(allowed.begin(), allowed.end(), key) == allowed.end()) {
            throw std::runtime_error(where + ": unknown key '" + key + "'." +
                                     DidYouMean(key, allowed) +
                                     " Valid keys: " + JoinNames(allowed) + ".");
        }
    }
}

/// Throw unless `key` is present.
inline void RequireKey(const YAML::Node& node, const char* key, const std::string& where) {
    if (!node[key]) {
        throw std::runtime_error(where + ": '" + key + "' is required.");
    }
}

inline void ReadDouble(const YAML::Node& node, const char* key, double& out,
                       const std::string& where) {
    if (!node[key]) {
        return;
    }
    try {
        out = node[key].as<double>();
    } catch (const YAML::Exception&) {
        throw std::runtime_error(where + "." + key + " must be a number.");
    }
    if (!std::isfinite(out)) {
        throw std::runtime_error(where + "." + key + " must be a finite number.");
    }
}

inline void ReadInt(const YAML::Node& node, const char* key, int& out, const std::string& where) {
    if (!node[key]) {
        return;
    }
    try {
        out = node[key].as<int>();
    } catch (const YAML::Exception&) {
        throw std::runtime_error(where + "." + key + " must be a whole number.");
    }
}

inline void ReadString(const YAML::Node& node, const char* key, std::string& out,
                       const std::string& where) {
    if (!node[key]) {
        return;
    }
    if (!node[key].IsScalar()) {
        throw std::runtime_error(where + "." + key + " must be a single value, not a list or map.");
    }
    out = node[key].Scalar();
}

inline void ReadBool(const YAML::Node& node, const char* key, bool& out, const std::string& where) {
    if (!node[key]) {
        return;
    }
    try {
        out = node[key].as<bool>();
    } catch (const YAML::Exception&) {
        throw std::runtime_error(where + "." + key + " must be true or false.");
    }
}

/// Read a fixed-length numeric sequence.  A sequence of the wrong length
/// throws, so a mis-sized vector cannot silently change geometry.
template <std::size_t N>
void ReadArray(const YAML::Node& node, const char* key, std::array<double, N>& out,
               const std::string& where) {
    if (!node[key]) {
        return;
    }
    const YAML::Node seq = node[key];
    if (!seq.IsSequence() || seq.size() != N) {
        throw std::runtime_error(where + "." + key + " must be a list of " + std::to_string(N) +
                                 " numbers.");
    }
    std::array<double, N> values{};
    for (std::size_t i = 0; i < N; ++i) {
        try {
            values[i] = seq[i].as<double>();
        } catch (const YAML::Exception&) {
            throw std::runtime_error(where + "." + key + "[" + std::to_string(i) +
                                     "] must be a number.");
        }
        if (!std::isfinite(values[i])) {
            throw std::runtime_error(where + "." + key + "[" + std::to_string(i) +
                                     "] must be a finite number.");
        }
    }
    out = values;
}

}  // namespace seastack::app::yaml_read

#endif  // SEASTACK_APP_YAML_READ_HELPERS_H
