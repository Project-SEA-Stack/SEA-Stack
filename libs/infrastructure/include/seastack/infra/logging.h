/**
 * @file logging.h
 * @brief Unified logging system for SEA-Stack
 * 
 * This header provides the main public interface for the SEA-Stack logging system.
 * It includes configuration structures, initialization functions, and namespace-based
 * APIs for CLI and debug logging.
 * 
 * @note This is the primary public interface for logging in SEA-Stack.
 * @note All logging operations are thread-safe when properly configured.
 * @note The system supports both CLI and debug logging modes.

 */

// NOTE: This header must remain public because it's included by public headers
// (e.g., adapters/chrono/include/seastack/adapters/chrono/helper.h) that expose
// logging functionality to users via seastack::infra::cli::LogError() and LOG_* macros.

#ifndef SEASTACK_INFRA_LOGGING_H
#define SEASTACK_INFRA_LOGGING_H

#include <cstddef>
#include <chrono>
#include <sstream>
#include <string>
#include <vector>

// Version information - provided by the build system via version.h / config.h
#ifndef SEASTACK_VERSION
    #define SEASTACK_VERSION "unknown"
#endif
#ifndef SEASTACK_BUILD_TYPE
    #define SEASTACK_BUILD_TYPE "unknown"
#endif

namespace seastack::infra {

// Minimal shared log types
enum class LogLevel { Debug = 0, Info = 1, Success = 2, Warning = 3, Error = 4 };
enum class LogColor { White, Green, Yellow, Red, Cyan, Blue, Gray, BrightWhite, BrightCyan, BrightGreen };

// Keep this in sync with LogLevel.
constexpr size_t kNumLogLevels = static_cast<size_t>(LogLevel::Error) + 1;
static_assert(static_cast<int>(LogLevel::Error) == 4, "Update kNumLogLevels when LogLevel changes.");

struct LogContext {
    std::string source_file;
    int source_line = 0;
    std::string function_name;
    std::string thread_id;
    std::string component;
    std::chrono::system_clock::time_point timestamp;
    LogContext() : timestamp(std::chrono::system_clock::now()) {}
};

// Utility helpers
std::string LogLevelToString(LogLevel level) noexcept;
std::string GetColorCode(LogColor color) noexcept;
std::string GetTimestamp();
std::string GetTimestampISO8601();
int GetVisibleWidth(const std::string& str);
std::string FormatNumber(double value, int decimal_places = 2);

/// Normalize path separators for CLI display (backslashes on Windows, forward slashes elsewhere).
std::string FormatCliPathForDisplay(const std::string& path);

/**
 * @brief Configuration for the logging system
 *
 * Controls various aspects of logging behavior including
 * output destinations, verbosity levels, and file management.
 */
struct LoggingConfig {
    std::string log_file_path;           ///< Path for log file (empty = no file logging)
    bool enable_cli_output = true;       ///< Enable console output
    bool enable_file_output = true;      ///< Enable file output
    bool enable_debug_logging = false;   ///< Enable debug-level logging
    // Verbosity thresholds (use LogLevel)
    LogLevel console_level = LogLevel::Info;   ///< Minimum level for console output
    LogLevel file_level = LogLevel::Debug;     ///< Minimum level for file output
    bool enable_colors = true;           ///< Enable ANSI color codes in console
    bool enable_timestamps = true;       ///< Include timestamps in log messages
    bool enable_source_location = false; ///< Include source location in debug logs
    
    LoggingConfig() = default;
};

/**
 * @brief Directory containing the current executable (for exe-relative data path resolution).
 * @return Absolute path to the executable's directory, or empty string on failure.
 */
std::string GetExecutableDirectory();

/**
 * @brief Full path to the current executable.
 * @return Absolute path to the executable, or empty string on failure.
 */
std::string GetExecutablePath();

/**
 * @brief One-line platform summary (OS, architecture, RAM) for diagnostics.
 * @return Human-readable string, e.g. "Windows x64 (AMD or Intel), 16 GB RAM".
 */
std::string GetPlatformSummary();

//-----------------------------------------------------------------------------
// Main Logging Interface
//-----------------------------------------------------------------------------

/**
 * @brief Initialize the logging system.
 * @param config Configuration for the logging system.
 */
[[nodiscard]] bool Initialize(const LoggingConfig& config);

/**
 * @brief Update logging configuration without tearing down stream capture.
 *
 * Used after CLI parsing when the process was initialized with defaults first
 * (e.g. --run-cell, --quiet, --debug).
 */
void UpdateLoggingConfig(const LoggingConfig& config);

/**
 * @brief Shutdown the logging system.
 *
 * Performs cleanup of logging resources. Call at program end to ensure
 * logs are flushed and intercepted streams are restored.
 *
 * @note This function is safe to call multiple times.
 */
void Shutdown();

/**
 * @brief Check if logging is initialized.
 * @return true if the logging system is initialized and ready.
 * @note All functions are thread-safe after Initialize() returns true; internal
 *       synchronization is handled by the logging library.
 */
[[nodiscard]] bool IsInitialized() noexcept;

//-----------------------------------------------------------------------------
// CLI Logging Namespace
//-----------------------------------------------------------------------------

/**
 * @brief CLI-focused logging namespace
 * 
 * Provides convenient functions for user-facing logging operations.
 * These functions delegate to the current CLI logger instance.
 */
namespace cli {

/**
 * @brief Log an informational message
 * @param message Message content
 */
void LogInfo(const std::string& message);

/**
 * @brief Log a success message (green)
 * @param message Message content
 */
void LogSuccess(const std::string& message);

/**
 * @brief Log a warning message (yellow)
 * @param message Message content
 */
void LogWarning(const std::string& message);

/**
 * @brief Log an error message (red)
 * @param message Message content
 */
void LogError(const std::string& message);

/**
 * @brief Log a debug message.
 *
 * Emitted at LogLevel::Debug: suppressed on the console when the console
 * threshold is above Debug, but still written to the log file when file logging
 * is enabled and file_level is Debug (e.g. with --log).
 */
void LogDebug(const std::string& message);

/**
 * @brief Display the SEA-Stack banner
 */
void ShowBanner();

/**
 * @brief Display a compact SEA-Stack header (version/build) for normal runs
 */
void ShowBannerCompact();

/**
 * @brief Display a section separator
 */
void ShowSectionSeparator();

/**
 * @brief Display an empty line for spacing
 */
void ShowEmptyLine();

/**
 * @brief Display a flat section header line with normalized width (60 chars)
 * @param title Title text (may include emojis/multibyte)
 */
void ShowHeader(const std::string& title);

/**
 * @brief Display a section box with title and content
 * @param title Section title
 * @param content_lines Vector of content lines
 */
void ShowSectionBox(const std::string& title,
                   const std::vector<std::string>& content_lines);

/**
 * @brief Display wave model parameters
 * @param wave_type Type of wave model
 * @param height Wave height in meters
 * @param period Wave period in seconds
 * @param direction Wave direction in degrees (optional)
 * @param phase Wave phase in degrees (optional)
 */
void ShowWaveModel(const std::string& wave_type, double height,
                   double period, double direction = 0.0, double phase = 0.0);

struct WavePartitionSummary {
    std::string spectrum_type;
    double Hs;
    double Tp;
    double direction_deg;
    std::string spreading_type;
    double spreading_s;
};

void ShowDirectionalWaveModel(const std::string& wave_type,
                              const std::vector<WavePartitionSummary>& partitions,
                              int n_components,
                              int n_omega,
                              int n_theta);

/**
 * @brief Display simulation completion results
 * @param final_time Final simulation time in seconds
 * @param steps Total number of simulation steps
 * @param wall_time Total wall-clock time in seconds
 * @param real_time_factor Simulated time elapsed divided by wall time, or negative to omit
 * @param hdf5_path Primary HDF5 artifact path when persisted (empty if none)
 * @param artifact_note Shown when HDF5 was not kept (e.g. temp/metrics-only run)
 * @param log_path Active log file path when file logging is enabled
 */
void ShowSimulationResults(double final_time, int steps, double wall_time,
                           double real_time_factor = -1.0,
                           const std::string& hdf5_path = std::string(),
                           const std::string& artifact_note = std::string(),
                           const std::string& log_path = std::string());

/**
 * @brief Display log file location
 * @param log_path Path to the log file
 */
void ShowLogFileLocation(const std::string& log_path);

/**
 * @brief Display the application footer
 */
void ShowFooter();

/**
 * @brief Collect a warning for later display
 * @param warning_message Warning to collect
 */
void CollectWarning(const std::string& warning_message);

/**
 * @brief Display all collected warnings
 */
void DisplayWarnings();

// Additional helpers used by apps
void ShowSummaryLine(const std::string& icon, const std::string& label,
                     const std::string& value, LogColor color = LogColor::White);
std::string CreateAlignedLine(const std::string& icon, const std::string& label, const std::string& value);

// Progress helpers (quiet mode visual feedback)
void ShowProgress(size_t current, size_t total, const std::string& message = "");
void StopProgress();

} // namespace cli

//-----------------------------------------------------------------------------
// Debug Logging Namespace
//-----------------------------------------------------------------------------

/**
 * @brief Debug-focused logging namespace.
 * 
 * Provides convenient functions for developer-facing logging operations.
 * These functions delegate to the current debug logger instance.
 */
namespace debug {

/**
 * @brief Log a debug message.
 * @param message Debug message content
 */
void LogDebug(const std::string& message);

/**
 * @brief Log a trace message (most verbose level).
 * @param message Trace message content
 */
void LogTrace(const std::string& message);

/**
 * @brief Log an informational message.
 * @param message Info message content
 */
void LogInfo(const std::string& message);

/**
 * @brief Log a warning message.
 * @param message Warning message content
 */
void LogWarning(const std::string& message);

/**
 * @brief Log an error message.
 * @param message Error message content
 */
void LogError(const std::string& message);

/**
 * @brief Check if debug logging is enabled for interactive diagnostics.
 * @return true if --debug/--trace (enable_debug_logging) or console threshold
 *         is Debug. Does not become true for --log alone; verbose file logs
 *         use file_level without implying this "debug mode".
 */
[[nodiscard]] bool IsDebugEnabled() noexcept;

} // namespace debug

} // namespace seastack::infra

//-----------------------------------------------------------------------------
// Convenience Macros
//-----------------------------------------------------------------------------

// Variadic stream-friendly macros that build a string with operator<<.

#define HC_BUILD_LOG_STRING(...) \
    std::ostringstream hc_oss__; \
    hc_oss__ << __VA_ARGS__; \
    const std::string hc_msg__ = hc_oss__.str();

// Debug/trace -> debug logger if available
#define LOG_DEBUG(...) \
    do { \
        if (seastack::infra::debug::IsDebugEnabled()) { \
            HC_BUILD_LOG_STRING(__VA_ARGS__); \
            seastack::infra::debug::LogDebug(hc_msg__); \
        } \
    } while (0)

#define LOG_TRACE(...) \
    do { \
        if (seastack::infra::debug::IsDebugEnabled()) { \
            HC_BUILD_LOG_STRING(__VA_ARGS__); \
            seastack::infra::debug::LogTrace(hc_msg__); \
        } \
    } while (0)

// Info/Warning/Error/Success helpers for convenience
#define LOG_INFO(...) \
    do { \
        HC_BUILD_LOG_STRING(__VA_ARGS__); \
        seastack::infra::cli::LogInfo(hc_msg__); \
    } while (0)

#define LOG_WARNING(...) \
    do { \
        HC_BUILD_LOG_STRING(__VA_ARGS__); \
        seastack::infra::cli::LogWarning(hc_msg__); \
    } while (0)

#define LOG_ERROR(...) \
    do { \
        HC_BUILD_LOG_STRING(__VA_ARGS__); \
        seastack::infra::cli::LogError(hc_msg__); \
    } while (0)

#define LOG_SUCCESS(...) \
    do { \
        HC_BUILD_LOG_STRING(__VA_ARGS__); \
        seastack::infra::cli::LogSuccess(hc_msg__); \
    } while (0)

// Function entry/exit and variable logging macros were removed to avoid
// implying features that are not implemented yet. Add back when needed.

#endif  // SEASTACK_INFRA_LOGGING_H