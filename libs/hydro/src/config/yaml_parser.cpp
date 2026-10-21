/*********************************************************************
 * @file  yaml_parser.cpp
 *
 * @brief Implementation of YAML parser for hydro.yaml files.
 *********************************************************************/

#include <seastack/hydro/config/yaml_parser.h>
#include <seastack/infra/logging.h>
#include <array>
#include <cmath>
#include <fstream>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <filesystem>
#include <algorithm>
#include <vector>

namespace {

/// DOF order of per-body 6-vectors (damping coefficients) in hydro YAML.
constexpr const char* kSixDofOrder = "[surge, sway, heave, roll, pitch, yaw]";

std::string Trim(const std::string& s) {
    const size_t first = s.find_first_not_of(" \t");
    if (first == std::string::npos) {
        return "";
    }
    const size_t last = s.find_last_not_of(" \t");
    return s.substr(first, last - first + 1);
}

std::string StripMatchingQuotes(const std::string& s) {
    if (s.size() >= 2 && ((s.front() == '"' && s.back() == '"') ||
                          (s.front() == '\'' && s.back() == '\''))) {
        return s.substr(1, s.size() - 2);
    }
    return s;
}

/// "<file>:<line>: " prefix for error and warning messages.
std::string Location(const std::string& file, int line) {
    return file + ":" + std::to_string(line) + ": ";
}

/// True for a YAML block-sequence item line ("- value" or a bare "-").
bool IsSequenceItem(const std::string& trimmed) {
    return trimmed == "-" ||
           (trimmed.size() >= 2 && trimmed[0] == '-' && (trimmed[1] == ' ' || trimmed[1] == '\t'));
}

/// Value of a block-sequence item line: the text after "-", without an
/// inline comment, surrounding whitespace or matching quotes.
std::string SequenceItemValue(const std::string& trimmed) {
    std::string v = trimmed.substr(1);
    const size_t hash_pos = v.find('#');
    if (hash_pos != std::string::npos) {
        v = v.substr(0, hash_pos);
    }
    return StripMatchingQuotes(Trim(v));
}

/**
 * @brief Parse a finite double; throw if the whole token is not a number.
 *
 * @param token Text to parse (surrounding whitespace and quotes allowed)
 * @param what  Message prefix naming the entry, e.g. "file:12: body 'b': linear_damping entry 3"
 */
double ParseStrictDouble(const std::string& token, const std::string& what) {
    const std::string t = StripMatchingQuotes(Trim(token));
    if (t.empty()) {
        throw std::runtime_error(what + " is empty");
    }
    size_t pos = 0;
    double v = 0.0;
    try {
        v = std::stod(t, &pos);
    } catch (const std::exception&) {
        pos = 0;
    }
    if (pos == 0 || pos != t.size() || !std::isfinite(v)) {
        throw std::runtime_error(what + " (\"" + t + "\") is not a finite number");
    }
    return v;
}

/**
 * @brief Split an inline flow list "[a, b, c]" into trimmed items.
 *
 * Throws if the value is not a single-line inline list. An empty entry
 * (e.g. "[1, , 3]" or a trailing comma) is kept as "" so the caller reports it.
 *
 * @param context Message prefix, e.g. "file:12: body 'b': linear_damping"
 */
std::vector<std::string> SplitInlineList(const std::string& value, const std::string& context) {
    const size_t lb = value.find('[');
    if (lb == std::string::npos || !Trim(value.substr(0, lb)).empty()) {
        throw std::runtime_error(context + " must be an inline list [a, b, ...] or a block list of '- value' "
                                 "items on the following lines; got \"" + value + "\"");
    }
    const size_t rb = value.find(']', lb);
    if (rb == std::string::npos) {
        throw std::runtime_error(context + " has an unterminated inline list (missing ']'); multi-line [...] "
                                 "lists are not supported, use a block list of '- value' items instead");
    }
    if (!Trim(value.substr(rb + 1)).empty()) {
        throw std::runtime_error(context + " has unexpected text after ']': \"" + value + "\"");
    }
    const std::string inner = Trim(value.substr(lb + 1, rb - lb - 1));
    std::vector<std::string> items;
    if (inner.empty()) {
        return items;
    }
    std::stringstream ss(inner);
    std::string token;
    while (std::getline(ss, token, ',')) {
        items.push_back(Trim(token));
    }
    if (inner.back() == ',') {
        items.emplace_back();
    }
    return items;
}

/**
 * @brief Convert exactly six list items to a per-DOF vector [surge, sway, heave, roll, pitch, yaw].
 *
 * @param item_lines Line number of each item (block form); empty for inline lists
 * @param key_line   Line number of the key
 * @param context    Message context without location, e.g. "body 'b': linear_damping"
 */
std::array<double, 6> ParseSixVector(const std::vector<std::string>& items,
                                     const std::vector<int>& item_lines,
                                     const std::string& file,
                                     int key_line,
                                     const std::string& context) {
    if (items.size() != 6) {
        throw std::runtime_error(Location(file, key_line) + context + " has " + std::to_string(items.size()) +
                                 " values; expected 6 " + kSixDofOrder);
    }
    std::array<double, 6> out{};
    for (size_t i = 0; i < 6; ++i) {
        const int line = item_lines.empty() ? key_line : item_lines[i];
        out[i] = ParseStrictDouble(items[i], Location(file, line) + context + " entry " + std::to_string(i + 1));
    }
    return out;
}

/**
 * @brief Get the indentation level of a line.
 * 
 * @param line The line to check
 * @return Number of spaces at the beginning of the line
 */
int GetIndentation(const std::string& line) {
    int indent = 0;
    for (char c : line) {
        if (c == ' ' || c == '\t') {
            indent++;
        } else {
            break;
        }
    }
    return indent;
}

/**
 * @brief Simple YAML line parser for key-value pairs.
 * 
 * @param line The line to parse
 * @param key Output key
 * @param value Output value
 * @return true if successfully parsed, false otherwise
 */
bool ParseYAMLLine(const std::string& line, std::string& key, std::string& value) {
    // Remove leading/trailing whitespace
    std::string trimmed = line;
    trimmed.erase(0, trimmed.find_first_not_of(" \t"));
    trimmed.erase(trimmed.find_last_not_of(" \t") + 1);
    // Strip trailing carriage return if present (Windows CRLF)
    if (!trimmed.empty() && trimmed.back() == '\r') {
        trimmed.pop_back();
    }
    
    // Skip empty lines and comments
    if (trimmed.empty() || trimmed[0] == '#') {
        return false;
    }
    
    // Find the colon separator
    size_t colon_pos = trimmed.find(':');
    if (colon_pos == std::string::npos) {
        return false;
    }
    
    // Extract key and value
    key = trimmed.substr(0, colon_pos);
    value = trimmed.substr(colon_pos + 1);
    
    // Remove inline comments (anything after '#')
    size_t hash_pos = std::string::npos;
    // simple heuristic: treat first '#' as comment start
    hash_pos = value.find('#');
    if (hash_pos != std::string::npos) {
        value = value.substr(0, hash_pos);
    }

    // Trim whitespace
    key.erase(0, key.find_first_not_of(" \t"));
    key.erase(key.find_last_not_of(" \t") + 1);
    value.erase(0, value.find_first_not_of(" \t"));
    value.erase(value.find_last_not_of(" \t") + 1);
    if (!value.empty() && value.back() == '\r') {
        value.pop_back();
    }
    
    // Remove quotes if present
    if (value.length() >= 2 && value[0] == '"' && value[value.length()-1] == '"') {
        value = value.substr(1, value.length() - 2);
    }
    
    return true;
}

/**
 * @brief Parse a double value from string.
 * 
 * @param str The string to parse
 * @param default_val Default value if parsing fails
 * @return Parsed double value
 */
double ParseDouble(const std::string& str, double default_val = 0.0) {
    try {
        return std::stod(str);
    } catch (const std::exception&) {
        return default_val;
    }
}

/**
 * @brief Parse a boolean value from string.
 * 
 * @param str The string to parse
 * @param default_val Default value if parsing fails
 * @return Parsed boolean value
 */
bool ParseBool(const std::string& str, bool default_val = true) {
    std::string lower = str;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
    
    if (lower == "true" || lower == "yes" || lower == "1") {
        return true;
    } else if (lower == "false" || lower == "no" || lower == "0") {
        return false;
    }
    return default_val;
}

/**
 * @brief Resolve a relative path based on the YAML file's location.
 * 
 * @param path The path to resolve (can be relative or absolute)
 * @param yaml_file_path The path to the YAML file
 * @return Resolved absolute path
 */
std::string ResolvePath(const std::string& path, const std::string& yaml_file_path) {
    std::filesystem::path file_path(path);
    
    if (file_path.is_absolute()) {
        return path;
    }
    
    std::filesystem::path yaml_path(yaml_file_path);
    std::filesystem::path yaml_dir = yaml_path.parent_path();
    std::filesystem::path resolved_path = yaml_dir / file_path;
    
    try {
        return std::filesystem::weakly_canonical(resolved_path).string();
    } catch (const std::exception&) {
        return resolved_path.string();
    }
}

} // anonymous namespace

namespace seastack::hydro {

YAMLHydroData ReadHydroYAML(const std::string& hydro_file_path) {
    YAMLHydroData data;
    
    std::ifstream file(hydro_file_path);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open hydro file: " + hydro_file_path);
    }
    
    std::string line;
    bool in_hydrodynamics = false;
    bool hydrodynamics_found = false;
    bool in_bodies = false;
    bool in_waves = false;
    bool in_excitation = false;             // excitation: section
    bool in_radiation = false;              // radiation: section
    bool in_radiation_state_space = false;  // radiation.state_space: subsection
    bool in_radiation_smoothing = false;    // radiation.smoothing: subsection
    bool in_radiation_taper = false;        // radiation.taper: subsection
    bool in_radiation_diagnostics = false;  // radiation.diagnostics: subsection
    bool in_body = false;
    bool in_body_hydrostatics = false;
    bool body_had_hydrostatics_block = false;
    bool in_moordyn = false;
    bool in_divergence = false;             // divergence: section
    HydroBody current_body;
    int line_number = 0;

    // Track which bodies had an explicit hydrostatics: block for legacy compat
    std::vector<bool> explicit_hydrostatics_flags;
    
    bool in_period_block = false;
    int period_block_indent = 0;
    bool period_block_seen = false;
    bool period_form_values = false;
    bool period_form_linspace = false;
    bool period_form_range = false;

    // Wave sub-section tracking for spreading/discretization/partitions
    bool in_waves_spreading = false;
    bool in_waves_discretization = false;
    bool in_waves_partitions = false;
    bool in_waves_partition_item = false;
    bool in_waves_partition_spreading = false;
    WavePartitionSettings current_partition;

    // Wave convenience fields
    bool waves_amplitude_set = false;
    double waves_amplitude = 0.0;
    bool period_set_by_synonym = false; // scalar period given via t/p/tp

    auto parse_inline_brace_kv = [](const std::string& v) -> std::vector<std::pair<std::string, std::string>> {
        // Expect something like: { start: 6.0, stop: 9.0, num: 4 }
        std::vector<std::pair<std::string, std::string>> out;
        size_t lb = v.find('{');
        size_t rb = v.find('}');
        if (lb == std::string::npos || rb == std::string::npos || rb <= lb) return out;
        std::string inner = v.substr(lb + 1, rb - lb - 1);
        // split on commas
        std::stringstream ss(inner);
        std::string token;
        while (std::getline(ss, token, ',')) {
            auto pos = token.find(':');
            if (pos == std::string::npos) continue;
            std::string k = token.substr(0, pos);
            std::string val = token.substr(pos + 1);
            // trim
            k.erase(0, k.find_first_not_of(" \t"));
            k.erase(k.find_last_not_of(" \t") + 1);
            val.erase(0, val.find_first_not_of(" \t"));
            val.erase(val.find_last_not_of(" \t") + 1);
            // strip optional quotes
            if (val.size() >= 2 && ((val.front() == '"' && val.back() == '"') || (val.front() == '\'' && val.back() == '\''))) {
                val = val.substr(1, val.size() - 2);
            }
            out.emplace_back(k, val);
        }
        return out;
    };

    // Block-sequence support: a recognised list key with an empty value
    // ("key:") opens a pending list that collects the following "- value"
    // lines (indented or at the key's own indent). The list is handed to
    // on_complete at the first other significant line or at end of file.
    struct PendingBlockList {
        bool active = false;
        int key_indent = 0;
        int key_line = 0;
        std::vector<std::string> items;
        std::vector<int> item_lines;
        std::function<void(const PendingBlockList&)> on_complete;
    };
    PendingBlockList pending_list;
    auto open_block_list = [&](int key_indent, std::function<void(const PendingBlockList&)> on_complete) {
        pending_list = PendingBlockList();
        pending_list.active = true;
        pending_list.key_indent = key_indent;
        pending_list.key_line = line_number;
        pending_list.on_complete = std::move(on_complete);
    };
    auto finish_block_list = [&]() {
        if (!pending_list.active) {
            return;
        }
        PendingBlockList done = std::move(pending_list);
        pending_list = PendingBlockList();
        done.on_complete(done);
    };

    // Most recent "key:" line and whether its value was empty; used to name
    // the parent key when an unsupported "- item" line is found.
    std::string last_key;
    bool last_key_empty = false;

    while (std::getline(file, line)) {
        line_number++;
        // Strip CR from CRLF files (std::getline keeps it on POSIX).
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        
        // Get indentation level
        int indent = GetIndentation(line);
        
        // Remove leading/trailing whitespace
        std::string trimmed = line;
        trimmed.erase(0, trimmed.find_first_not_of(" \t"));
        trimmed.erase(trimmed.find_last_not_of(" \t") + 1);
        
        // Skip empty lines and comments
        if (trimmed.empty() || trimmed[0] == '#') {
            continue;
        }

        if (pending_list.active) {
            if (IsSequenceItem(trimmed) && indent >= pending_list.key_indent) {
                pending_list.items.push_back(SequenceItemValue(trimmed));
                pending_list.item_lines.push_back(line_number);
                continue;
            }
            finish_block_list();
        }
        if (!IsSequenceItem(trimmed)) {
            std::string k;
            std::string v;
            if (ParseYAMLLine(line, k, v)) {
                last_key = k;
                last_key_empty = v.empty();
            }
        }
        
        // Check for section headers based on indentation
        if (indent == 0 && trimmed == "hydrodynamics:") {
            in_hydrodynamics = true;
            hydrodynamics_found = true;
            in_bodies = false;
            in_waves = false;
            in_body = false;
            in_excitation = false;
            in_radiation = false;
            in_radiation_state_space = false;
            in_radiation_smoothing = in_radiation_taper = in_radiation_diagnostics = false;
            in_moordyn = false;
            in_divergence = false;
            continue;
        }

        // Any other top-level key ends the hydrodynamics section.
        if (indent == 0) {
            in_hydrodynamics = false;
            continue;
        }
        
        if (!in_hydrodynamics) {
            continue; // Skip everything until we find hydrodynamics section
        }
        
        // Check for subsections (indent = 2)
        if (indent == 2 && trimmed == "bodies:") {
            in_bodies = true;
            in_waves = false;
            in_body = false;
            in_excitation = false;
            in_radiation = false;
            in_radiation_state_space = false;
            in_radiation_smoothing = in_radiation_taper = in_radiation_diagnostics = false;
            in_moordyn = false;
            in_divergence = false;
            continue;
        }
        
        if (indent == 2 && trimmed == "waves:") {
            if (in_body && !current_body.name.empty()) {
                data.bodies.push_back(current_body);
                explicit_hydrostatics_flags.push_back(body_had_hydrostatics_block);
            }
            in_waves = true;
            in_bodies = false;
            in_body = false;
            in_excitation = false;
            in_radiation = false;
            in_radiation_state_space = false;
            in_radiation_smoothing = in_radiation_taper = in_radiation_diagnostics = false;
            in_moordyn = false;
            in_divergence = false;
            continue;
        }

        if (indent == 2 && trimmed == "excitation:") {
            if (in_body && !current_body.name.empty()) {
                data.bodies.push_back(current_body);
                explicit_hydrostatics_flags.push_back(body_had_hydrostatics_block);
            }
            in_excitation = true;
            in_radiation = false;
            in_radiation_state_space = false;
            in_radiation_smoothing = in_radiation_taper = in_radiation_diagnostics = false;
            in_bodies = false;
            in_waves = false;
            in_body = false;
            in_moordyn = false;
            in_divergence = false;
            continue;
        }

        if (indent == 2 && trimmed == "radiation:") {
            if (in_body && !current_body.name.empty()) {
                data.bodies.push_back(current_body);
                explicit_hydrostatics_flags.push_back(body_had_hydrostatics_block);
            }
            in_radiation = true;
            in_radiation_state_space = false;
            in_radiation_smoothing = in_radiation_taper = in_radiation_diagnostics = false;
            in_excitation = false;
            in_bodies = false;
            in_waves = false;
            in_body = false;
            in_moordyn = false;
            in_divergence = false;
            continue;
        }

        if (indent == 2 && trimmed == "moordyn:") {
            if (in_body && !current_body.name.empty()) {
                data.bodies.push_back(current_body);
                explicit_hydrostatics_flags.push_back(body_had_hydrostatics_block);
            }
            in_moordyn = true;
            in_bodies = false;
            in_waves = false;
            in_excitation = false;
            in_radiation = false;
            in_radiation_state_space = false;
            in_radiation_smoothing = in_radiation_taper = in_radiation_diagnostics = false;
            in_body = false;
            in_divergence = false;
            continue;
        }

        if (indent == 2 && trimmed == "divergence:") {
            if (in_body && !current_body.name.empty()) {
                data.bodies.push_back(current_body);
                explicit_hydrostatics_flags.push_back(body_had_hydrostatics_block);
            }
            in_divergence = true;
            data.divergence.has_any = true;
            in_bodies = false;
            in_waves = false;
            in_excitation = false;
            in_radiation = false;
            in_radiation_state_space = false;
            in_radiation_smoothing = in_radiation_taper = in_radiation_diagnostics = false;
            in_body = false;
            in_moordyn = false;
            continue;
        }
        
        // Check for body list item (indent = 4, starts with "- name")
        if (in_bodies && indent == 4 && trimmed.substr(0, 6) == "- name") {
            // Save previous body if exists
            if (in_body && !current_body.name.empty()) {
                data.bodies.push_back(current_body);
                explicit_hydrostatics_flags.push_back(body_had_hydrostatics_block);
            }
            
            // Start new body
            current_body = HydroBody();
            in_body = true;
            in_body_hydrostatics = false;
            body_had_hydrostatics_block = false;
            
            // Parse the name from this line (remove the "- " prefix first)
            std::string name_line = trimmed.substr(2); // Remove "- "
            std::string key, value;
            if (ParseYAMLLine(name_line, key, value)) {
                if (key == "name") {
                    current_body.name = value;
                }
            }
            continue;
        }
        
        // Check for partition list item in waves section (indent = 6, starts with "- ")
        if (in_waves && in_waves_partitions && indent == 6
            && trimmed.size() >= 2 && trimmed.substr(0, 2) == "- ") {
            if (in_waves_partition_item) {
                data.waves.partitions.push_back(current_partition);
            }
            current_partition = WavePartitionSettings();
            in_waves_partition_item = true;
            in_waves_partition_spreading = false;

            std::string item_line = trimmed.substr(2);
            std::string item_key, item_value;
            if (ParseYAMLLine(item_line, item_key, item_value)) {
                std::string k = item_key;
                std::transform(k.begin(), k.end(), k.begin(), ::tolower);
                if (k == "spectrum") current_partition.spectrum_type = item_value;
                else if (k == "hs") current_partition.Hs = ParseDouble(item_value, 0.0);
                else if (k == "tp") current_partition.Tp = ParseDouble(item_value, 0.0);
                else if (k == "gamma") current_partition.gamma = ParseDouble(item_value, 3.3);
                else if (k == "direction") current_partition.mean_direction_deg = ParseDouble(item_value, 0.0);
            }
            continue;
        }

        // Any "- item" line not consumed above would be silently dropped.
        if (IsSequenceItem(trimmed)) {
            const std::string where = Location(hydro_file_path, line_number);
            if (in_bodies && trimmed.compare(0, 6, "- name") == 0) {
                throw std::runtime_error(where + "body list items must be indented by 4 spaces "
                                         "(\"    - name: <name>\"); got \"" + trimmed + "\"");
            }
            if (in_bodies && indent <= 4) {
                throw std::runtime_error(where + "each body in 'bodies' must start with '- name: <name>'; got \"" +
                                         trimmed + "\"");
            }
            std::string parent;
            if (last_key_empty && !last_key.empty()) {
                parent = " under '" + last_key + "'";
            }
            throw std::runtime_error(where + "unexpected YAML list item \"" + trimmed + "\"" + parent +
                                     ": block lists are not supported here; use an inline list [a, b, ...]");
        }

        // Parse key-value pairs
        // - Global hydrodynamics properties at indent == 2
        // - Body properties at indent == 6
        // - Wave properties at indent == 4
        // - Nested period block under waves at indent >= period_block_indent + 2 when in_period_block
        {
            std::string key;
            std::string value;
            bool should_parse = (
                (!in_bodies && !in_waves && in_hydrodynamics && !in_excitation && !in_radiation && !in_moordyn && !in_divergence && indent == 2) ||
                (in_excitation && indent == 4) ||
                (in_radiation && indent == 4) ||
                (in_radiation_state_space && indent == 6) ||
                (in_radiation_smoothing && indent == 6) ||
                (in_radiation_taper && indent == 6) ||
                (in_radiation_diagnostics && indent == 6) ||
                (in_body && indent == 6) ||
                (in_body_hydrostatics && indent == 8) ||
                (in_waves && (indent == 4 || (in_period_block && indent >= period_block_indent + 2) || ((in_waves_spreading || in_waves_discretization) && indent == 6) || (in_waves_partition_item && indent == 8) || (in_waves_partition_spreading && indent == 10))) ||
                (in_moordyn && indent == 4) ||
                (in_divergence && indent == 4)
            );
            if (should_parse && ParseYAMLLine(line, key, value)) {
                // ─────────────────────────────────────────────────────────────
                // excitation: section parsing
                // ─────────────────────────────────────────────────────────────
                if (in_excitation && indent == 4) {
                    if (key == "truncation_time") {
                        data.excitation_truncation_time = ParseDouble(value, 0.0);
                    } else if (key == "method") {
                        data.excitation_method = value;
                    }
                }
                // ─────────────────────────────────────────────────────────────
                // radiation: section parsing
                // ─────────────────────────────────────────────────────────────
                else if (in_radiation && indent == 4) {
                    if (key == "method") {
                        data.radiation_method = value;
                    } else if (key == "truncation_time") {
                        data.radiation_truncation_time = ParseDouble(value, 0.0);
                    } else if (key == "frequency_tolerance") {
                        data.radiation_frequency_tolerance =
                            ParseDouble(value, data.radiation_frequency_tolerance);
                    } else if (key == "smoothing") {
                        if (!value.empty()) {
                            data.radiation_kernel_processing.smoothing_type = value;
                        } else {
                            in_radiation_smoothing = true;
                            in_radiation_taper = in_radiation_diagnostics = in_radiation_state_space = false;
                        }
                    } else if (key == "taper") {
                        in_radiation_taper = true;
                        in_radiation_smoothing = in_radiation_diagnostics = in_radiation_state_space = false;
                    } else if (key == "diagnostics") {
                        in_radiation_diagnostics = true;
                        in_radiation_smoothing = in_radiation_taper = in_radiation_state_space = false;
                    } else if (key == "state_space") {
                        if (value.empty()) {
                            in_radiation_state_space = true;
                            in_radiation_smoothing = in_radiation_taper = in_radiation_diagnostics = false;
                        }
                    }
                } else if (in_radiation_smoothing && indent == 6) {
                    auto& kp = data.radiation_kernel_processing;
                    if (key == "type") {
                        kp.smoothing_type = value;
                    } else if (key == "window_length") {
                        try { kp.smoothing_window = std::stoi(value); }
                        catch (const std::exception&) {
                            LOG_WARNING("Invalid integer for radiation.smoothing.window_length: \""
                                        << value << "\"; using default");
                        }
                    }
                } else if (in_radiation_taper && indent == 6) {
                    auto& kp = data.radiation_kernel_processing;
                    if (key == "enabled") {
                        kp.taper_enabled = ParseBool(value, false);
                    } else if (key == "start_percent") {
                        kp.taper_enabled = true;
                        kp.taper_start_fraction = ParseDouble(value, kp.taper_start_fraction);
                    } else if (key == "end_percent") {
                        kp.taper_enabled = true;
                        kp.taper_end_fraction = ParseDouble(value, kp.taper_end_fraction);
                    } else if (key == "final_amplitude") {
                        kp.taper_enabled = true;
                        kp.taper_final_amplitude = ParseDouble(value, kp.taper_final_amplitude);
                    }
                } else if (in_radiation_diagnostics && indent == 6) {
                    if (key == "export_csv") {
                        data.radiation_kernel_processing.export_csv = ParseBool(value, false);
                    } else if (key == "output_kernel_fit") {
                        data.output_kernel_fit = ParseBool(value, false);
                    }
                } else if (in_radiation_state_space && indent == 6) {
                    if (key == "max_order") {
                        try { data.ss_max_order = std::stoi(value); }
                        catch (const std::exception&) {
                            LOG_WARNING("Invalid integer for state_space.max_order: \""
                                        << value << "\"; using default");
                        }
                    } else if (key == "r2_threshold") {
                        data.ss_r2_threshold = ParseDouble(value, data.ss_r2_threshold);
                    } else if (key == "max_hankel_size") {
                        try { data.ss_max_hankel_size = std::stoi(value); }
                        catch (const std::exception&) {
                            LOG_WARNING("Invalid integer for state_space.max_hankel_size: \""
                                        << value << "\"; using default");
                        }
                    } else if (key == "r2_num_samples") {
                        try { data.ss_r2_num_samples = std::stoi(value); }
                        catch (const std::exception&) {
                            LOG_WARNING("Invalid integer for state_space.r2_num_samples: \""
                                        << value << "\"; using default");
                        }
                    }
                } else if (!in_bodies && !in_waves && in_hydrodynamics && !in_excitation && !in_radiation && !in_moordyn && !in_divergence && indent == 2) {
                    // Global hydrodynamics properties at top level
                    if (key == "radiation_method") {
                        data.radiation_method = value;
                    } else if (key == "nonlinear_hydrostatics") {
                        data.nonlinear_hydrostatics = ParseBool(value, false);
                    } else if (key == "attach_infinite_frequency_added_mass") {
                        LOG_WARNING(
                            "hydrodynamics.attach_infinite_frequency_added_mass is deprecated and "
                            "ignored; infinite-frequency added mass from H5 is always applied via "
                            "Chrono ChLoadHydrodynamics when using HydroSystem.");
                    }
                } else if (in_body) {
                    // Reset hydrostatics sub-block when indent returns to body level
                    if (indent <= 6) {
                        in_body_hydrostatics = false;
                    }

                    // hydrostatics keys, shared by the block and inline-brace forms
                    auto apply_hydrostatics_key = [&](const std::string& k, const std::string& v) {
                        if (k == "model") {
                            std::string val_lower = v;
                            std::transform(val_lower.begin(), val_lower.end(), val_lower.begin(), ::tolower);
                            if (val_lower == "nonlinear") {
                                current_body.hydrostatics.model = HydrostaticsModel::kNonlinear;
                            } else if (val_lower == "linear") {
                                current_body.hydrostatics.model = HydrostaticsModel::kLinear;
                            } else {
                                throw std::runtime_error(Location(hydro_file_path, line_number) + "body '" +
                                                         current_body.name + "': hydrostatics.model must be "
                                                         "'linear' or 'nonlinear'; got \"" + v + "\"");
                            }
                        } else if (k == "mesh_file") {
                            current_body.hydrostatics.mesh_file = ResolvePath(v, hydro_file_path);
                        } else {
                            LOG_WARNING(Location(hydro_file_path, line_number)
                                        << "unknown key 'hydrostatics." << k << "' in body '"
                                        << current_body.name << "' ignored");
                        }
                    };

                    if (in_body_hydrostatics && indent == 8) {
                        apply_hydrostatics_key(key, value);
                    } else if (key == "hydrostatics" && value.empty()) {
                        in_body_hydrostatics = true;
                        body_had_hydrostatics_block = true;
                    } else if (key == "hydrostatics" && value.find('{') == std::string::npos) {
                        // Scalar form: hydrostatics: nonlinear (mesh from the body-level mesh_file)
                        apply_hydrostatics_key("model", value);
                        body_had_hydrostatics_block = true;
                    } else if (key == "hydrostatics") {
                        // Inline form: hydrostatics: { model: nonlinear, mesh_file: hull.stl }
                        const auto kv = parse_inline_brace_kv(value);
                        if (kv.empty()) {
                            throw std::runtime_error(Location(hydro_file_path, line_number) + "body '" +
                                                     current_body.name + "': hydrostatics must be a block or an "
                                                     "inline mapping { model: ..., mesh_file: ... }; got \"" +
                                                     value + "\"");
                        }
                        body_had_hydrostatics_block = true;
                        for (const auto& [k, v] : kv) {
                            apply_hydrostatics_key(k, v);
                        }
                    } else if (key == "name") {
                        current_body.name = value;
                    } else if (key == "h5_file") {
                        current_body.h5_file = ResolvePath(value, hydro_file_path);
                    } else if (key == "mesh_file") {
                        current_body.mesh_file = ResolvePath(value, hydro_file_path);
                    } else if (key == "froude_krylov") {
                        std::string val_lower = value;
                        std::transform(val_lower.begin(), val_lower.end(), val_lower.begin(), ::tolower);
                        if (val_lower == "nonlinear") {
                            current_body.froude_krylov = FroudeKrylovModel::kNonlinear;
                        } else if (val_lower == "linear") {
                            current_body.froude_krylov = FroudeKrylovModel::kLinear;
                        } else {
                            throw std::runtime_error(Location(hydro_file_path, line_number) + "body '" +
                                                     current_body.name + "': froude_krylov must be "
                                                     "'linear' or 'nonlinear'; got \"" + value + "\"");
                        }
                    } else if (key == "froude_krylov_pressure") {
                        std::string val_lower = value;
                        std::transform(val_lower.begin(), val_lower.end(), val_lower.begin(), ::tolower);
                        if (val_lower == "airy_wheeler") {
                            current_body.froude_krylov_pressure = FroudeKrylovPressureModel::kAiryWheeler;
                        } else if (val_lower == "hydrostatic") {
                            current_body.froude_krylov_pressure = FroudeKrylovPressureModel::kHydrostatic;
                        } else {
                            throw std::runtime_error(Location(hydro_file_path, line_number) + "body '" +
                                                     current_body.name + "': froude_krylov_pressure must be "
                                                     "'airy_wheeler' or 'hydrostatic'; got \"" + value + "\"");
                        }
                    } else if (key == "include_excitation") {
                        current_body.include_excitation = ParseBool(value, true);
                    } else if (key == "include_radiation") {
                        current_body.include_radiation = ParseBool(value, true);
                    } else if (key == "linear_damping" || key == "quadratic_damping") {
                        // Per-DOF hull damping [surge, sway, heave, roll, pitch, yaw], either
                        // inline "[a, b, c, d, e, f]" or as six "- value" lines. Anything else
                        // throws: a silently zeroed damping vector changes the physics.
                        const bool is_linear = (key == "linear_damping");
                        const std::string context = "body '" + current_body.name + "': " + key;
                        auto assign = [&current_body, is_linear](const std::array<double, 6>& v) {
                            (is_linear ? current_body.linear_damping : current_body.quadratic_damping) = v;
                        };
                        if (value.empty()) {
                            open_block_list(indent, [&hydro_file_path, context, assign](const PendingBlockList& list) {
                                if (list.items.empty()) {
                                    throw std::runtime_error(
                                        Location(hydro_file_path, list.key_line) + context +
                                        " has no value; expected an inline list of 6 values " + kSixDofOrder +
                                        " or 6 '- value' lines below the key");
                                }
                                assign(ParseSixVector(list.items, list.item_lines, hydro_file_path,
                                                      list.key_line, context));
                            });
                        } else {
                            const auto items =
                                SplitInlineList(value, Location(hydro_file_path, line_number) + context);
                            assign(ParseSixVector(items, {}, hydro_file_path, line_number, context));
                        }
                    } else {
                        LOG_WARNING(Location(hydro_file_path, line_number)
                                    << "unknown key '" << key << "' in body '" << current_body.name
                                    << "' ignored");
                    }
                } else if (in_waves) {
                    // Parse wave properties
                    // normalize key for shorthand handling
                    std::string key_lower = key;
                    std::transform(key_lower.begin(), key_lower.end(), key_lower.begin(), ::tolower);

                    // Handle partition spreading sub-properties (indent >= 10)
                    if (in_waves_partition_spreading && indent >= 10) {
                        if (key_lower == "type") {
                            current_partition.spreading.type = value;
                        } else if (key_lower == "s") {
                            current_partition.spreading.s = ParseDouble(value, 12.0);
                        }
                        continue;
                    }
                    // Handle partition item properties (indent >= 8)
                    if (in_waves_partition_item && indent >= 8) {
                        if (indent < 10) in_waves_partition_spreading = false;
                        if (key_lower == "spreading" && value.empty()) {
                            in_waves_partition_spreading = true;
                        } else if (key_lower == "spectrum") {
                            current_partition.spectrum_type = value;
                        } else if (key_lower == "hs") {
                            current_partition.Hs = ParseDouble(value, 0.0);
                        } else if (key_lower == "tp") {
                            current_partition.Tp = ParseDouble(value, 0.0);
                        } else if (key_lower == "gamma") {
                            current_partition.gamma = ParseDouble(value, 3.3);
                        } else if (key_lower == "direction") {
                            current_partition.mean_direction_deg = ParseDouble(value, 0.0);
                        }
                        continue;
                    }

                    // Handle wave sub-sections (spreading, discretization)
                    if (in_waves_spreading && indent >= 6) {
                        if (key_lower == "type") {
                            data.waves.spreading.type = value;
                        } else if (key_lower == "s") {
                            data.waves.spreading.s = ParseDouble(value, 12.0);
                        }
                        continue;
                    }
                    if (in_waves_discretization && indent >= 6) {
                        if (key_lower == "n_omega") {
                            try { data.waves.discretization.n_omega = std::stoi(value); }
                            catch (const std::exception&) {
                                LOG_WARNING("Invalid integer for waves.discretization.n_omega: \""
                                            << value << "\"; using default");
                            }
                        } else if (key_lower == "n_theta") {
                            try { data.waves.discretization.n_theta = std::stoi(value); }
                            catch (const std::exception&) {
                                LOG_WARNING("Invalid integer for waves.discretization.n_theta: \""
                                            << value << "\"; using default");
                            }
                        }
                        continue;
                    }

                    // Leaving sub-sections when indent drops back to waves level
                    if (indent <= 4) {
                        in_waves_spreading = false;
                        in_waves_discretization = false;
                        if (in_waves_partition_item) {
                            data.waves.partitions.push_back(current_partition);
                            current_partition = WavePartitionSettings();
                        }
                        in_waves_partitions = false;
                        in_waves_partition_item = false;
                        in_waves_partition_spreading = false;
                    }
                    // A key at or above the period: key's indent ends the period
                    // block and is itself a waves key (otherwise it was dropped).
                    if (in_period_block && indent <= period_block_indent) {
                        in_period_block = false;
                    }

                    if (!in_period_block && key_lower == "type") {
                        data.waves.type = value;
                    } else if (!in_period_block && (key_lower == "height" || key_lower == "h")) {
                        data.waves.height = ParseDouble(value, 0.0);
                    } else if (!in_period_block && (key_lower == "amplitude" || key_lower == "a")) {
                        waves_amplitude = ParseDouble(value, 0.0);
                        waves_amplitude_set = true;
                    } else if (!in_period_block && (key_lower == "period" || key_lower == "t" || key_lower == "tp" || key_lower == "p")) {
                        period_block_seen = true;
                        period_form_values = period_form_linspace = period_form_range = false;
                        data.waves.period_values.clear();
                        // Scalar on same line
                        bool looks_structured = (value.find('{') != std::string::npos || value.find('[') != std::string::npos || value.empty());
                        if (!looks_structured) {
                            data.waves.period = ParseDouble(value, 0.0);
                            data.waves.period_values.push_back(data.waves.period);
                            period_set_by_synonym = (key_lower != "period");
                        } else if (!value.empty() && value.front() == '[') {
                            // Bare inline list: period: [6.0, 7.0, 8.0] (same as values: [...])
                            const std::string where = Location(hydro_file_path, line_number);
                            const auto items = SplitInlineList(value, where + "waves.period");
                            if (items.empty()) {
                                throw std::runtime_error(where + "waves.period list is empty");
                            }
                            for (size_t i = 0; i < items.size(); ++i) {
                                data.waves.period_values.push_back(ParseStrictDouble(
                                    items[i], where + "waves.period entry " + std::to_string(i + 1)));
                            }
                            data.waves.period = data.waves.period_values.front();
                            period_form_values = true;
                        } else {
                            // Inline forms on same line
                            // values inline list inside braces
                            if (value.find("values") != std::string::npos && value.find('[') != std::string::npos) {
                                auto lb = value.find('['); auto rb = value.find(']');
                                if (lb != std::string::npos && rb != std::string::npos && rb > lb) {
                                    std::string inner = value.substr(lb + 1, rb - lb - 1);
                                    for (char& ch : inner) if (ch == ',') ch = ' ';
                                    std::istringstream iss(inner);
                                    double v; while (iss >> v) data.waves.period_values.push_back(v);
                                    if (!data.waves.period_values.empty()) {
                                        data.waves.period = data.waves.period_values.front();
                                        period_form_values = true;
                                    }
                                }
                            }
                            // Start nested block
                            if (value.empty() || value == "|" || value == ">") {
                                in_period_block = true;
                                period_block_indent = indent;
                            }
                        }
                    } else if (in_period_block && key == "values" && value.empty()) {
                        // period:\n  values:\n    - a\n    - b
                        open_block_list(indent, [&](const PendingBlockList& list) {
                            if (list.items.empty()) {
                                throw std::runtime_error(Location(hydro_file_path, list.key_line) +
                                                         "waves.period.values has no values");
                            }
                            if (period_form_linspace || period_form_range) {
                                throw std::runtime_error("waves.period: multiple forms specified (values + other)");
                            }
                            data.waves.period_values.clear();
                            for (size_t i = 0; i < list.items.size(); ++i) {
                                data.waves.period_values.push_back(ParseStrictDouble(
                                    list.items[i], Location(hydro_file_path, list.item_lines[i]) +
                                                       "waves.period.values entry " + std::to_string(i + 1)));
                            }
                            data.waves.period = data.waves.period_values.front();
                            period_form_values = true;
                        });
                    } else if (in_period_block && key == "values") {
                        // period:\n  values: [a, b, c]
                        const std::string where = Location(hydro_file_path, line_number);
                        const auto items = SplitInlineList(value, where + "waves.period.values");
                        if (items.empty()) {
                            throw std::runtime_error(where + "waves.period.values list is empty");
                        }
                        data.waves.period_values.clear();
                        for (size_t i = 0; i < items.size(); ++i) {
                            data.waves.period_values.push_back(ParseStrictDouble(
                                items[i], where + "waves.period.values entry " + std::to_string(i + 1)));
                        }
                        data.waves.period = data.waves.period_values.front();
                        if (period_form_linspace || period_form_range) {
                            throw std::runtime_error("waves.period: multiple forms specified (values + other)");
                        }
                        period_form_values = true;
                    } else if (in_period_block && key == "linspace") {
                        // linspace: { start: a, stop: b, num: n }
                        auto kv = parse_inline_brace_kv(value);
                        double start = 0.0, stop = 0.0; int num = 0; bool hasS=false, hasE=false, hasN=false;
                        for (auto& p : kv) {
                            if (p.first == "start") { start = ParseDouble(p.second, 0.0); hasS = true; }
                            else if (p.first == "stop") { stop = ParseDouble(p.second, 0.0); hasE = true; }
                            else if (p.first == "num") {
                                try { num = std::stoi(p.second); }
                                catch (const std::exception&) {
                                    LOG_WARNING("Invalid integer for waves.period.linspace.num: \""
                                                << p.second << "\"; defaulting to 0");
                                    num = 0;
                                }
                                hasN = true;
                            }
                        }
                        if (!(hasS && hasE && hasN) || num < 2) {
                            throw std::runtime_error("waves.period: invalid linspace (require start, stop, num>=2)");
                        }
                        if (period_form_values || period_form_range) {
                            throw std::runtime_error("waves.period: multiple forms specified");
                        }
                        period_form_linspace = true;
                        data.waves.period_values.clear();
                        if (num == 2) {
                            data.waves.period_values.push_back(start);
                            data.waves.period_values.push_back(stop);
                        } else {
                            double step = (stop - start) / static_cast<double>(num - 1);
                            for (int k = 0; k < num; ++k) {
                                data.waves.period_values.push_back(start + step * static_cast<double>(k));
                            }
                        }
                        data.waves.period = data.waves.period_values.front();
                    } else if (in_period_block && key == "range") {
                        // range: { start: a, stop: b, step: s, inclusive: true }
                        auto kv = parse_inline_brace_kv(value);
                        double start = 0.0, stop = 0.0, step = 0.0; bool inclusive = true; bool hasS=false, hasE=false, hasStep=false;
                        for (auto& p : kv) {
                            if (p.first == "start") { start = ParseDouble(p.second, 0.0); hasS = true; }
                            else if (p.first == "stop") { stop = ParseDouble(p.second, 0.0); hasE = true; }
                            else if (p.first == "step") { step = ParseDouble(p.second, 0.0); hasStep = true; }
                            else if (p.first == "inclusive") { inclusive = ParseBool(p.second, true); }
                        }
                        if (!(hasS && hasE && hasStep) || step <= 0.0 || stop < start) {
                            throw std::runtime_error("waves.period: invalid range (require start<=stop, step>0)");
                        }
                        if (period_form_values || period_form_linspace) {
                            throw std::runtime_error("waves.period: multiple forms specified");
                        }
                        period_form_range = true;
                        data.waves.period_values.clear();
                        double t = start;
                        const double eps = 1e-9;
                        while (t < stop - eps) {
                            data.waves.period_values.push_back(t);
                            t += step;
                        }
                        if (inclusive) {
                            if (std::abs((data.waves.period_values.empty() ? start : data.waves.period_values.back()) - stop) > eps) {
                                // add last if not already close to stop and within tolerance by stepping once more would overshoot but inclusive means include stop
                                data.waves.period_values.push_back(stop);
                            } else {
                                // if already at stop within eps, ensure exact stop
                                data.waves.period_values.back() = stop;
                            }
                        }
                        if (data.waves.period_values.empty()) {
                            // Degenerate (start==stop and inclusive false) not allowed per rules
                            throw std::runtime_error("waves.period: range produced no values");
                        }
                        data.waves.period = data.waves.period_values.front();
                    } else if (!in_period_block && key_lower == "direction") {
                        data.waves.direction = ParseDouble(value, 0.0);
                    } else if (!in_period_block && key_lower == "phase") {
                        data.waves.phase = ParseDouble(value, 0.0);
                    } else if (!in_period_block && key_lower == "spectrum") {
                        data.waves.spectrum = value;
                    } else if (!in_period_block && key_lower == "seed") {
                        try { data.waves.seed = std::stoi(value); }
                        catch (const std::exception&) {
                            LOG_WARNING("Invalid integer for waves.seed: \""
                                        << value << "\"; using random seed");
                            data.waves.seed = -1;
                        }
                    } else if (!in_period_block && (key_lower == "eta_file" || key_lower == "eta_file_path")) {
                        data.waves.eta_file = ResolvePath(value, hydro_file_path);
                    } else if (!in_period_block && key_lower == "method") {
                        data.waves.method = value;
                    } else if (!in_period_block && key_lower == "ramp_type") {
                        data.waves.ramp_type = value;
                    } else if (!in_period_block && key_lower == "ramp_duration") {
                        data.waves.ramp_duration = ParseDouble(value, 0.0);
                    } else if (!in_period_block && key_lower == "frequency_min") {
                        data.waves.frequency_min = ParseDouble(value, 0.0);
                    } else if (!in_period_block && key_lower == "frequency_max") {
                        data.waves.frequency_max = ParseDouble(value, 0.0);
                    } else if (!in_period_block && key_lower == "nfrequencies") {
                        try { data.waves.nfrequencies = std::stoi(value); }
                        catch (const std::exception&) {
                            LOG_WARNING("Invalid integer for waves.nfrequencies: \""
                                        << value << "\"; using default");
                            data.waves.nfrequencies = 0;
                        }
                    } else if (!in_period_block && key_lower == "gamma") {
                        data.waves.gamma = ParseDouble(value, 3.3);
                    } else if (!in_period_block && key_lower == "depth") {
                        data.waves.depth = ParseDouble(value, 0.0);
                    } else if (!in_period_block && key_lower == "spreading") {
                        if (value.empty()) {
                            in_waves_spreading = true;
                            in_waves_discretization = false;
                        }
                    } else if (!in_period_block && key_lower == "discretization") {
                        if (value.empty()) {
                            in_waves_discretization = true;
                            in_waves_spreading = false;
                        }
                    } else if (!in_period_block && key_lower == "partitions") {
                        if (value.empty()) {
                            in_waves_partitions = true;
                            in_waves_spreading = false;
                            in_waves_discretization = false;
                        }
                    }
                } else if (in_moordyn) {
                    std::string key_lower = key;
                    std::transform(key_lower.begin(), key_lower.end(), key_lower.begin(), ::tolower);
                    if (key_lower == "enabled") {
                        std::string val_lower = value;
                        std::transform(val_lower.begin(), val_lower.end(), val_lower.begin(), ::tolower);
                        data.moordyn_enabled = (val_lower == "true" || val_lower == "1" || val_lower == "yes");
                    } else if (key_lower == "input_file") {
                        data.moordyn_input_file = ResolvePath(value, hydro_file_path);
                    } else if (key_lower == "bodies" && value.empty()) {
                        // Block form: bodies: followed by "- body1" lines
                        open_block_list(indent, [&](const PendingBlockList& list) {
                            if (list.items.empty()) {
                                throw std::runtime_error(Location(hydro_file_path, list.key_line) +
                                                         "moordyn.bodies has no value; expected [body1, ...] "
                                                         "or '- body1' lines below the key");
                            }
                            for (size_t i = 0; i < list.items.size(); ++i) {
                                if (list.items[i].empty()) {
                                    throw std::runtime_error(Location(hydro_file_path, list.item_lines[i]) +
                                                             "moordyn.bodies entry is empty");
                                }
                                data.moordyn_body_names.push_back(list.items[i]);
                            }
                        });
                    } else if (key_lower == "bodies") {
                        // Parse [body1, body2] list
                        std::string list_str = value;
                        // Strip brackets
                        size_t lb = list_str.find('[');
                        size_t rb = list_str.find(']');
                        if (lb != std::string::npos && rb != std::string::npos && rb > lb) {
                            list_str = list_str.substr(lb + 1, rb - lb - 1);
                        }
                        // Split by commas
                        std::istringstream iss(list_str);
                        std::string token;
                        while (std::getline(iss, token, ',')) {
                            token.erase(0, token.find_first_not_of(" \t"));
                            token.erase(token.find_last_not_of(" \t") + 1);
                            if (!token.empty()) {
                                data.moordyn_body_names.push_back(token);
                            }
                        }
                    } else if (key_lower == "visualization_line_radius") {
                        data.moordyn_visualization_line_radius = ParseDouble(value, -1.0);
                    } else if (key_lower == "visualization_endpoint_radius") {
                        data.moordyn_visualization_endpoint_radius = ParseDouble(value, -1.0);
                    } else if (key_lower == "visualization_node_marker_radius") {
                        data.moordyn_visualization_node_marker_radius = ParseDouble(value, -1.0);
                    }
                } else if (in_divergence) {
                    std::string key_lower = key;
                    std::transform(key_lower.begin(), key_lower.end(), key_lower.begin(), ::tolower);
                    if (key_lower == "enabled") {
                        data.divergence.enabled = ParseBool(value, true);
                        data.divergence.has_enabled = true;
                    } else if (key_lower == "max_roll_pitch") {
                        data.divergence.max_roll_pitch_deg = ParseDouble(value, 90.0);
                        data.divergence.has_max_roll_pitch = true;
                    } else if (key_lower == "max_position") {
                        data.divergence.max_position = ParseDouble(value, 200.0);
                        data.divergence.has_max_position = true;
                    } else if (key_lower == "max_velocity") {
                        data.divergence.max_velocity = ParseDouble(value, 20.0);
                        data.divergence.has_max_velocity = true;
                    } else if (key_lower == "max_angular_velocity") {
                        data.divergence.max_angular_velocity = ParseDouble(value, 5.0);
                        data.divergence.has_max_angular_velocity = true;
                    } else if (key_lower == "max_force") {
                        data.divergence.max_force = ParseDouble(value, 1.0e10);
                        data.divergence.has_max_force = true;
                    }
                }
            }
        }
            // Detect leaving the period block when indentation drops below the
            // period: key (a sibling at the same indent is handled in the waves
            // branch; "<=" here would close the block on its own opening line).
            if (in_period_block && indent < period_block_indent) {
                in_period_block = false;
            }
    }

    // A block list may run to the end of the file.
    finish_block_list();
    
    // Don't forget to add the last body
    if (in_body && !current_body.name.empty()) {
        data.bodies.push_back(current_body);
        explicit_hydrostatics_flags.push_back(body_had_hydrostatics_block);
    }

    // Save last partition if still building one
    if (in_waves_partition_item) {
        data.waves.partitions.push_back(current_partition);
    }
    
    // Finalize period block validation if it was started
    if (period_block_seen) {
        // Exactly one form or a scalar must have been provided
        int forms = 0;
        if (period_form_values) forms++;
        if (period_form_linspace) forms++;
        if (period_form_range) forms++;
        if (forms > 1) {
            throw std::runtime_error("waves.period: multiple forms specified");
        }
        if (data.waves.period_values.empty()) {
            if (data.waves.period > 0.0) {
                data.waves.period_values.push_back(data.waves.period);
            } else {
                throw std::runtime_error("waves.period: invalid or empty specification");
            }
        }
    } else {
        // If no structured values captured, ensure period_values mirrors scalar period
        if (data.waves.period_values.empty() && data.waves.period > 0.0) {
            data.waves.period_values.push_back(data.waves.period);
        }
    }

    // Apply amplitude to height if provided
    if (waves_amplitude_set) {
        const double derived_height = 2.0 * waves_amplitude;
        if (data.waves.height > 0.0) {
            const double eps = 1e-9;
            if (std::abs(data.waves.height - derived_height) > eps) {
                throw std::runtime_error("waves: both height and amplitude provided but inconsistent (expected height = 2*amplitude)");
            }
        } else {
            data.waves.height = derived_height;
        }
    }

    // Validation and helpful errors for regular waves
    {
        std::string type_lower = data.waves.type;
        std::transform(type_lower.begin(), type_lower.end(), type_lower.begin(), ::tolower);
        if (type_lower == "regular") {
            if (data.waves.height <= 0.0) {
                throw std::runtime_error("waves: regular requires wave height (use 'height' or 'h', or 'amplitude'/'a')");
            }
            bool has_period = (data.waves.period > 0.0) || !data.waves.period_values.empty();
            if (!has_period) {
                throw std::runtime_error("waves: regular requires wave period (use 'period' or shorthand 't', 'tp', or 'p')");
            }
        }
    }

    // Validate that we found the required sections
    if (!hydrodynamics_found) {
        throw std::runtime_error("No 'hydrodynamics:' section found in hydro file: " + hydro_file_path);
    }
    
    if (data.bodies.empty()) {
        LOG_WARNING("WARNING: No bodies found in hydro file: " << hydro_file_path);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Legacy backward compatibility: map global nonlinear_hydrostatics flag
    // + bare mesh_file to per-body HydrostaticsSettings.
    // This is the single normalization point: all downstream code only ever
    // sees the normalized per-body HydrostaticsSettings.
    // ─────────────────────────────────────────────────────────────────────────
    if (data.nonlinear_hydrostatics) {
        bool any_explicit = false;
        for (size_t i = 0; i < data.bodies.size(); ++i) {
            if (i < explicit_hydrostatics_flags.size() && explicit_hydrostatics_flags[i]) {
                any_explicit = true;
            }
        }
        if (any_explicit) {
            LOG_WARNING(
                "WARNING: Both global 'nonlinear_hydrostatics' flag and "
                "per-body 'hydrostatics:' blocks are present. Per-body "
                "config takes precedence. The global flag is deprecated.");
        } else {
            LOG_WARNING(
                "WARNING: Global 'nonlinear_hydrostatics' flag is deprecated. "
                "Use per-body 'hydrostatics: { model: nonlinear }' instead.");
        }
        for (size_t i = 0; i < data.bodies.size(); ++i) {
            bool has_explicit = (i < explicit_hydrostatics_flags.size())
                                && explicit_hydrostatics_flags[i];
            if (has_explicit) continue;  // per-body config wins

            data.bodies[i].hydrostatics.model = HydrostaticsModel::kNonlinear;
            if (!data.bodies[i].mesh_file.empty()) {
                data.bodies[i].hydrostatics.mesh_file = data.bodies[i].mesh_file;
            }
        }
    }

    // Body-level mesh_file is the canonical hull for all nonlinear models;
    // hydrostatics.mesh_file is still accepted. Downstream code reads
    // hydrostatics.mesh_file only.
    for (auto& body : data.bodies) {
        if (body.hydrostatics.model == HydrostaticsModel::kNonlinear && !body.mesh_file.empty()) {
            if (body.hydrostatics.mesh_file.empty()) {
                body.hydrostatics.mesh_file = body.mesh_file;
            } else if (body.hydrostatics.mesh_file != body.mesh_file) {
                throw std::runtime_error("body '" + body.name + "': mesh_file ('" + body.mesh_file +
                                         "') and hydrostatics.mesh_file ('" + body.hydrostatics.mesh_file +
                                         "') differ. Set the hull once, with the body-level mesh_file.");
            }
        }
        if (body.froude_krylov == FroudeKrylovModel::kNonlinear && !body.include_excitation) {
            throw std::runtime_error("body '" + body.name + "': froude_krylov: nonlinear needs "
                                     "include_excitation: true. The nonlinear force replaces the "
                                     "linear Froude-Krylov part of the BEM excitation.");
        }
    }
    
    return data;
}

}  // namespace seastack::hydro

