#include <seastack/external/protocol.h>

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <string>

namespace seastack {
namespace external {
namespace protocol {
namespace {

size_t SkipWs(const std::string& s, size_t i) {
    while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) {
        ++i;
    }
    return i;
}

bool FindKey(const std::string& json, const char* key, size_t& value_pos) {
    // Naive scan for "key" : value — adequate for our flat protocol messages.
    const std::string pattern = std::string("\"") + key + "\"";
    size_t pos = 0;
    while (true) {
        pos = json.find(pattern, pos);
        if (pos == std::string::npos) {
            return false;
        }
        size_t i = pos + pattern.size();
        i = SkipWs(json, i);
        if (i < json.size() && json[i] == ':') {
            value_pos = SkipWs(json, i + 1);
            return true;
        }
        pos += 1;
    }
}

/// Append a Unicode code point to `out` as UTF-8.
void AppendUtf8(unsigned long cp, std::string& out) {
    if (cp < 0x80) {
        out += static_cast<char>(cp);
    } else if (cp < 0x800) {
        out += static_cast<char>(0xC0 | (cp >> 6));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    } else if (cp < 0x10000) {
        out += static_cast<char>(0xE0 | (cp >> 12));
        out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    } else {
        out += static_cast<char>(0xF0 | (cp >> 18));
        out += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
        out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
        out += static_cast<char>(0x80 | (cp & 0x3F));
    }
}

/// Read 4 hex digits at json[pos]; returns false if malformed.
bool ReadHex4(const std::string& json, size_t pos, unsigned long& cp) {
    if (pos + 4 > json.size()) {
        return false;
    }
    char* end_ptr = nullptr;
    const std::string hex = json.substr(pos, 4);
    cp = std::strtoul(hex.c_str(), &end_ptr, 16);
    return end_ptr == hex.c_str() + 4;
}

}  // namespace

bool DecodeJsonString(const std::string& json, size_t start,
                      std::string& value, size_t& end_pos) {
    size_t pos = start;
    if (pos >= json.size() || json[pos] != '"') {
        return false;
    }
    ++pos;
    value.clear();
    while (pos < json.size()) {
        const char c = json[pos];
        if (c == '"') {
            end_pos = pos + 1;
            return true;
        }
        if (c != '\\') {
            value += c;
            ++pos;
            continue;
        }
        if (pos + 1 >= json.size()) {
            return false;
        }
        const char e = json[pos + 1];
        pos += 2;
        switch (e) {
            case 'n': value += '\n'; break;
            case 't': value += '\t'; break;
            case 'r': value += '\r'; break;
            case 'b': value += '\b'; break;
            case 'f': value += '\f'; break;
            case 'u': {
                unsigned long cp = 0;
                if (!ReadHex4(json, pos, cp)) {
                    return false;
                }
                pos += 4;
                // UTF-16 surrogate pair (e.g. emoji from Python json.dumps).
                if (cp >= 0xD800 && cp <= 0xDBFF && pos + 6 <= json.size() &&
                    json[pos] == '\\' && json[pos + 1] == 'u') {
                    unsigned long lo = 0;
                    if (ReadHex4(json, pos + 2, lo) && lo >= 0xDC00 && lo <= 0xDFFF) {
                        cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                        pos += 6;
                    }
                }
                AppendUtf8(cp, value);
                break;
            }
            default:  // '"', '\\', '/'
                value += e;
                break;
        }
    }
    return false;
}

std::string JsonEscape(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 8);
    for (char c : s) {
        switch (c) {
            case '"':
                out += "\\\"";
                break;
            case '\\':
                out += "\\\\";
                break;
            case '\n':
                out += "\\n";
                break;
            case '\r':
                out += "\\r";
                break;
            case '\t':
                out += "\\t";
                break;
            default:
                out += c;
                break;
        }
    }
    return out;
}

std::string EncodeDoubleArray(const std::vector<double>& values) {
    std::ostringstream oss;
    oss << '[';
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0) {
            oss << ',';
        }
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%.17g", values[i]);
        oss << buf;
    }
    oss << ']';
    return oss.str();
}

std::string EncodeStringArray(const std::vector<std::string>& values) {
    std::ostringstream oss;
    oss << '[';
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0) {
            oss << ',';
        }
        oss << '"' << JsonEscape(values[i]) << '"';
    }
    oss << ']';
    return oss.str();
}

bool DecodeDoubleArray(const std::string& json, size_t start,
                       std::vector<double>& out, size_t& end_pos) {
    out.clear();
    size_t i = SkipWs(json, start);
    if (i >= json.size() || json[i] != '[') {
        return false;
    }
    ++i;
    i = SkipWs(json, i);
    if (i < json.size() && json[i] == ']') {
        end_pos = i + 1;
        return true;
    }
    while (i < json.size()) {
        i = SkipWs(json, i);
        char* end_ptr = nullptr;
        const double v = std::strtod(json.c_str() + i, &end_ptr);
        if (end_ptr == json.c_str() + i) {
            return false;
        }
        out.push_back(v);
        i = static_cast<size_t>(end_ptr - json.c_str());
        i = SkipWs(json, i);
        if (i < json.size() && json[i] == ',') {
            ++i;
            continue;
        }
        if (i < json.size() && json[i] == ']') {
            end_pos = i + 1;
            return true;
        }
        return false;
    }
    return false;
}

bool ExtractStringField(const std::string& json, const char* key,
                        std::string& value) {
    size_t pos = 0;
    if (!FindKey(json, key, pos)) {
        return false;
    }
    if (pos >= json.size() || json[pos] != '"') {
        return false;
    }
    size_t end_pos = 0;
    return DecodeJsonString(json, pos, value, end_pos);
}

bool ExtractIntField(const std::string& json, const char* key, int& value) {
    size_t pos = 0;
    if (!FindKey(json, key, pos)) {
        return false;
    }
    char* end_ptr = nullptr;
    const long v = std::strtol(json.c_str() + pos, &end_ptr, 10);
    if (end_ptr == json.c_str() + pos) {
        return false;
    }
    value = static_cast<int>(v);
    return true;
}

bool ExtractDoubleField(const std::string& json, const char* key, double& value) {
    size_t pos = 0;
    if (!FindKey(json, key, pos)) {
        return false;
    }
    char* end_ptr = nullptr;
    const double v = std::strtod(json.c_str() + pos, &end_ptr);
    if (end_ptr == json.c_str() + pos) {
        return false;
    }
    value = v;
    return true;
}

std::string MakeInitializeRequest(const ExternalInit& init) {
    std::ostringstream oss;
    const std::string& cfg =
        init.config_json.empty() ? std::string("{}") : init.config_json;
    oss << "{\"op\":\"initialize\""
        << ",\"protocol\":" << kProtocolVersion
        << ",\"kind\":\"" << JsonEscape(init.kind) << "\""
        << ",\"n_inputs\":" << init.n_inputs
        << ",\"n_outputs\":" << init.n_outputs
        << ",\"dt\":";
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.17g", init.dt);
    oss << buf;
    if (!init.in_names.empty()) {
        oss << ",\"in_names\":" << EncodeStringArray(init.in_names);
    }
    if (!init.out_names.empty()) {
        oss << ",\"out_names\":" << EncodeStringArray(init.out_names);
    }
    oss << ",\"config\":" << cfg << '}';
    return oss.str();
}

std::string MakeEvaluateRequest(double time, double dt,
                                const std::vector<double>& in) {
    std::ostringstream oss;
    char tbuf[64];
    char dbuf[64];
    std::snprintf(tbuf, sizeof(tbuf), "%.17g", time);
    std::snprintf(dbuf, sizeof(dbuf), "%.17g", dt);
    oss << "{\"op\":\"evaluate\",\"t\":" << tbuf << ",\"dt\":" << dbuf
        << ",\"in\":" << EncodeDoubleArray(in) << '}';
    return oss.str();
}

std::string MakeSimpleOpRequest(const char* op) {
    return std::string("{\"op\":\"") + op + "\"}";
}

bool ParseStatusReply(const std::string& json,
                      std::string& status,
                      std::string& message) {
    if (!ExtractStringField(json, "status", status)) {
        return false;
    }
    message.clear();
    if (status != "ok") {
        ExtractStringField(json, "message", message);
    }
    return true;
}

bool ParseInitializeReply(const std::string& json, ExternalMeta& meta) {
    if (!ExtractStringField(json, "name", meta.name)) {
        meta.name = "external";
    }
    if (!ExtractStringField(json, "version", meta.version)) {
        meta.version = "0";
    }
    if (!ExtractIntField(json, "n_states", meta.n_states)) {
        meta.n_states = 0;
    }
    return true;
}

bool ExtractStringArrayField(const std::string& json, const char* key,
                             std::vector<std::string>& values) {
    values.clear();
    size_t pos = 0;
    if (!FindKey(json, key, pos)) {
        return false;
    }
    if (pos >= json.size() || json[pos] != '[') {
        return false;
    }
    pos = SkipWs(json, pos + 1);
    if (pos < json.size() && json[pos] == ']') {
        return true;
    }
    while (pos < json.size()) {
        std::string item;
        size_t end_pos = 0;
        if (!DecodeJsonString(json, pos, item, end_pos)) {
            return false;
        }
        values.push_back(std::move(item));
        pos = SkipWs(json, end_pos);
        if (pos < json.size() && json[pos] == ',') {
            pos = SkipWs(json, pos + 1);
            continue;
        }
        return pos < json.size() && json[pos] == ']';
    }
    return false;
}

bool ParseEvaluateReply(const std::string& json, std::vector<double>& out) {
    size_t pos = 0;
    if (!FindKey(json, "out", pos)) {
        return false;
    }
    size_t end_pos = 0;
    return DecodeDoubleArray(json, pos, out, end_pos);
}

}  // namespace protocol
}  // namespace external
}  // namespace seastack
