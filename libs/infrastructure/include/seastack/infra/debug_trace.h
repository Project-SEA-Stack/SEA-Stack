/**
 * @file debug_trace.h
 * @brief Environment-gated trace utilities for low-level debugging.
 *
 * Enable MoorDyn/mooring coupling trace with: SEASTACK_MOORDYN_TRACE=1
 */

#ifndef SEASTACK_INFRA_DEBUG_TRACE_H
#define SEASTACK_INFRA_DEBUG_TRACE_H

#include <cstdlib>
#include <seastack/infra/logging.h>
#include <string>

namespace seastack::infra {

/// Returns true if the SEASTACK_MOORDYN_TRACE environment variable is set.
inline bool IsMoorDynTraceEnabled() {
    static const bool enabled = []() {
        const char* env = std::getenv("SEASTACK_MOORDYN_TRACE");
        return env != nullptr && env[0] != '\0' && env[0] != '0';
    }();
    return enabled;
}

/// Emit a trace message via the debug logger if MoorDyn tracing is enabled.
inline void EmitMoorDynTrace(const std::string& message) {
    if (IsMoorDynTraceEnabled()) {
        seastack::infra::debug::LogDebug(std::string("[MOORDYN_TRACE] ") + message);
    }
}

}  // namespace seastack::infra

/// Emit a trace message at most once per call site.
#define SEASTACK_TRACE_ONCE(message)                       \
    do {                                                    \
        static bool seastack_traced_ = false;               \
        if (!seastack_traced_) {                            \
            seastack::infra::EmitMoorDynTrace(message);     \
            seastack_traced_ = true;                        \
        }                                                   \
    } while (0)

#endif  // SEASTACK_INFRA_DEBUG_TRACE_H
