/*********************************************************************
 * @file  protocol.h
 * @brief Framing and JSON helpers for the external-force wire protocol.
 *
 * Length-prefixed (uint32 BE) UTF-8 JSON messages. See
 * docs/extending/EXTERNAL_FORCE_MODULES.md.
 *********************************************************************/
#ifndef SEASTACK_EXTERNAL_PROTOCOL_H
#define SEASTACK_EXTERNAL_PROTOCOL_H

#include <seastack/external/external_force_model.h>

#include <cstdint>
#include <string>
#include <vector>

namespace seastack {
namespace external {
namespace protocol {

/// Build initialize request JSON.
std::string MakeInitializeRequest(const ExternalInit& init);

/// Build evaluate request JSON.
std::string MakeEvaluateRequest(double time, double dt,
                                const std::vector<double>& in);

std::string MakeSimpleOpRequest(const char* op);  // reset / commit / rollback / shutdown

/// Parse a reply. Returns false on malformed JSON.
/// On success, status is "ok" or "error"; message filled on error.
bool ParseStatusReply(const std::string& json,
                      std::string& status,
                      std::string& message);

/// Parse initialize reply fields (assumes status already checked as ok).
bool ParseInitializeReply(const std::string& json, ExternalMeta& meta);

/// Parse evaluate reply `out` array (assumes status already checked as ok).
bool ParseEvaluateReply(const std::string& json, std::vector<double>& out);

/// Encode a double array as a JSON array string.
std::string EncodeDoubleArray(const std::vector<double>& values);

/// Decode a JSON array of doubles from a substring starting at '['.
bool DecodeDoubleArray(const std::string& json, size_t start,
                       std::vector<double>& out, size_t& end_pos);

/// Minimal escape of a string for JSON (quotes and backslashes).
std::string JsonEscape(const std::string& s);

/// Extract a string value for key "key" from a flat JSON object.
bool ExtractStringField(const std::string& json, const char* key,
                        std::string& value);

/// Extract an integer value for key "key".
bool ExtractIntField(const std::string& json, const char* key, int& value);

/// Extract a double value for key "key".
bool ExtractDoubleField(const std::string& json, const char* key, double& value);

}  // namespace protocol
}  // namespace external
}  // namespace seastack

#endif  // SEASTACK_EXTERNAL_PROTOCOL_H
