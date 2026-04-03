#ifndef SEASTACK_BENCH_UTILS_H
#define SEASTACK_BENCH_UTILS_H

/**
 * Benchmarking utilities for SEA-Stack performance tests.
 *
 * Provides:
 *   - Timer: high-resolution wall-clock timer
 *   - BenchmarkMetadata: machine / build / thread info
 *   - TrialResult: per-trial timing and component breakdown
 *   - run_trials(): multi-trial execution with optional warmup
 *   - write_benchmark_json(): schema-v2 JSON output
 *
 * Uses std::chrono + OpenMP headers; no other external dependencies.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <seastack/hydro/hydro_forces.h>

#ifndef SEASTACK_BENCH_BUILD_TYPE
#define SEASTACK_BENCH_BUILD_TYPE "Unknown"
#endif
#ifndef SEASTACK_BENCH_COMPILER_ID
#define SEASTACK_BENCH_COMPILER_ID "Unknown"
#endif
#ifndef SEASTACK_BENCH_COMPILER_VERSION
#define SEASTACK_BENCH_COMPILER_VERSION "Unknown"
#endif
#ifndef SEASTACK_BENCH_GIT_COMMIT
#define SEASTACK_BENCH_GIT_COMMIT "unknown"
#endif
#ifndef SEASTACK_BENCH_GIT_DIRTY
#define SEASTACK_BENCH_GIT_DIRTY false
#endif
#ifndef SEASTACK_BENCH_SEASTACK_VERSION
#define SEASTACK_BENCH_SEASTACK_VERSION "unknown"
#endif
#ifndef SEASTACK_BENCH_SYSTEM_NAME
#define SEASTACK_BENCH_SYSTEM_NAME "Unknown"
#endif
#ifndef SEASTACK_BENCH_SYSTEM_VERSION
#define SEASTACK_BENCH_SYSTEM_VERSION ""
#endif

namespace seastack::bench {

// ═══════════════════════════════════════════════════════════════════════════════
// Timer
// ═══════════════════════════════════════════════════════════════════════════════

class Timer {
 public:
    void Start() { start_ = std::chrono::high_resolution_clock::now(); }

    double StopSeconds() {
        auto end = std::chrono::high_resolution_clock::now();
        return std::chrono::duration<double>(end - start_).count();
    }

 private:
    std::chrono::high_resolution_clock::time_point start_;
};

// ═══════════════════════════════════════════════════════════════════════════════
// ISO 8601 timestamp
// ═══════════════════════════════════════════════════════════════════════════════

inline std::string GetISOTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    std::tm gmt{};
#ifdef _WIN32
    gmtime_s(&gmt, &t);
#else
    gmtime_r(&t, &gmt);
#endif
    std::ostringstream oss;
    oss << std::put_time(&gmt, "%Y-%m-%dT%H:%M:%SZ");
    return oss.str();
}

// ═══════════════════════════════════════════════════════════════════════════════
// Component breakdown (from HydroForcesProfileStats)
// ═══════════════════════════════════════════════════════════════════════════════

struct ComponentBreakdown {
    double hydrostatics_s = 0.0;
    double radiation_s = 0.0;
    double excitation_s = 0.0;
    double mooring_s = 0.0;
};

inline ComponentBreakdown FromProfileStats(const hydro::HydroForcesProfileStats& stats) {
    return {stats.hydrostatics_seconds, stats.radiation_seconds,
            stats.excitation_seconds, stats.mooring_seconds};
}

// ═══════════════════════════════════════════════════════════════════════════════
// TrialResult
// ═══════════════════════════════════════════════════════════════════════════════

struct TrialResult {
    double setup_wall_s = 0.0;
    double sim_wall_s = 0.0;
    double total_wall_s = 0.0;
    ComponentBreakdown components;
};

// ═══════════════════════════════════════════════════════════════════════════════
// Aggregate statistics
// ═══════════════════════════════════════════════════════════════════════════════

struct AggregateStats {
    double mean = 0.0;
    double min_val = 0.0;
    double max_val = 0.0;
    double stddev = 0.0;
};

inline AggregateStats ComputeStats(const std::vector<double>& values) {
    if (values.empty()) return {};
    AggregateStats s;
    s.min_val = *std::min_element(values.begin(), values.end());
    s.max_val = *std::max_element(values.begin(), values.end());
    s.mean = std::accumulate(values.begin(), values.end(), 0.0) / (double)values.size();
    double sq_sum = 0.0;
    for (double v : values) sq_sum += (v - s.mean) * (v - s.mean);
    s.stddev = values.size() > 1 ? std::sqrt(sq_sum / (double)(values.size() - 1)) : 0.0;
    return s;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Multi-trial runner
// ═══════════════════════════════════════════════════════════════════════════════

inline std::vector<TrialResult> RunTrials(int num_trials, bool warmup,
                                          const std::function<TrialResult()>& trial_fn) {
    if (warmup) {
        std::cout << "  Warmup trial..." << std::endl;
        trial_fn();
    }
    std::vector<TrialResult> results;
    results.reserve(num_trials);
    for (int i = 0; i < num_trials; ++i) {
        std::cout << "  Trial " << (i + 1) << "/" << num_trials << "..." << std::flush;
        auto r = trial_fn();
        std::cout << " sim=" << std::fixed << std::setprecision(2) << r.sim_wall_s << "s"
                  << "  total=" << r.total_wall_s << "s" << std::endl;
        results.push_back(r);
    }
    return results;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Metadata collection
// ═══════════════════════════════════════════════════════════════════════════════

struct BenchmarkMetadata {
    std::string seastack_version;
    std::string git_commit;
    bool git_dirty = false;
    std::string hostname;
    std::string cpu;
    int physical_cores = 0;
    int logical_cores = 0;
    int omp_max_threads = 1;
    int omp_num_procs = 1;
    std::string build_type;
    std::string compiler;
    std::string os;
};

inline std::string GetHostname() {
    char buf[256] = {};
#ifdef _WIN32
    DWORD size = sizeof(buf);
    if (GetComputerNameA(buf, &size)) return std::string(buf);
#else
    if (gethostname(buf, sizeof(buf)) == 0) return std::string(buf);
#endif
    return "unknown";
}

inline std::string GetCPUName() {
#ifdef _WIN32
    HKEY key;
    if (RegOpenKeyExA(HKEY_LOCAL_MACHINE,
                      "HARDWARE\\DESCRIPTION\\System\\CentralProcessor\\0",
                      0, KEY_READ, &key) == ERROR_SUCCESS) {
        char value[256] = {};
        DWORD size = sizeof(value);
        DWORD type = REG_SZ;
        if (RegQueryValueExA(key, "ProcessorNameString", nullptr, &type,
                             (LPBYTE)value, &size) == ERROR_SUCCESS) {
            RegCloseKey(key);
            return std::string(value);
        }
        RegCloseKey(key);
    }
#endif
    return "unknown";
}

inline BenchmarkMetadata CollectMetadata() {
    BenchmarkMetadata m;
    m.seastack_version = SEASTACK_BENCH_SEASTACK_VERSION;
    m.git_commit = SEASTACK_BENCH_GIT_COMMIT;
    m.git_dirty = SEASTACK_BENCH_GIT_DIRTY;
    m.hostname = GetHostname();
    m.cpu = GetCPUName();
    m.logical_cores = static_cast<int>(std::thread::hardware_concurrency());
    m.physical_cores = std::max(1, m.logical_cores / 2);
#ifdef _OPENMP
    m.omp_max_threads = omp_get_max_threads();
    m.omp_num_procs = omp_get_num_procs();
#else
    m.omp_max_threads = 1;
    m.omp_num_procs = m.logical_cores;
#endif
    m.build_type = SEASTACK_BENCH_BUILD_TYPE;
    m.compiler = std::string(SEASTACK_BENCH_COMPILER_ID) + " " + SEASTACK_BENCH_COMPILER_VERSION;
    m.os = std::string(SEASTACK_BENCH_SYSTEM_NAME) + " " + SEASTACK_BENCH_SYSTEM_VERSION;
    return m;
}

// ═══════════════════════════════════════════════════════════════════════════════
// Benchmark settings
// ═══════════════════════════════════════════════════════════════════════════════

struct BenchmarkSettings {
    double timestep = 0.0;
    double sim_duration = 0.0;
    int num_steps = 0;
    int num_bodies = 0;
    std::string radiation_method = "rirf_convolution";
    std::string excitation_method = "none";
    std::string wave_type = "none";
    bool moordyn = false;
    int num_trials = 3;
    bool warmup = false;
};

// ═══════════════════════════════════════════════════════════════════════════════
// Command-line argument parsing
//
// All benchmark executables accept:
//   --trials N         Override number of measured trials
//   --warmup           Force a warmup trial before measurement
//   --no-warmup        Disable warmup
//   --condition N      Run only condition N (for batch-style benchmarks)
// ═══════════════════════════════════════════════════════════════════════════════

struct BenchmarkCLIArgs {
    int num_trials = -1;   // -1 = use test default
    int warmup = -1;       // -1 = use test default, 0 = off, 1 = on
    int condition = -1;    // -1 = all conditions (batch mode)
};

inline BenchmarkCLIArgs ParseBenchmarkArgs(int argc, char* argv[]) {
    BenchmarkCLIArgs args;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--trials" && i + 1 < argc) {
            args.num_trials = std::atoi(argv[++i]);
        } else if (arg == "--warmup") {
            args.warmup = 1;
        } else if (arg == "--no-warmup" || arg == "--no_warmup") {
            args.warmup = 0;
        } else if (arg == "--condition" && i + 1 < argc) {
            args.condition = std::atoi(argv[++i]);
        }
    }
    return args;
}

inline int ResolveTrialCount(int test_default, const BenchmarkCLIArgs& cli) {
    return (cli.num_trials > 0) ? cli.num_trials : test_default;
}

inline bool ResolveWarmup(bool test_default, const BenchmarkCLIArgs& cli) {
    if (cli.warmup == 0) return false;
    if (cli.warmup == 1) return true;
    return test_default;
}

// ═══════════════════════════════════════════════════════════════════════════════
// JSON writing helpers (hand-rolled to avoid dependency)
// ═══════════════════════════════════════════════════════════════════════════════

namespace detail {

inline std::string Escape(const std::string& s) { return s; }

inline void WriteStatsObject(std::ofstream& f, const std::string& indent,
                             const AggregateStats& s) {
    f << indent << "\"mean\": " << std::fixed << std::setprecision(4) << s.mean << ",\n"
      << indent << "\"min\": " << s.min_val << ",\n"
      << indent << "\"max\": " << s.max_val << ",\n"
      << indent << "\"stddev\": " << s.stddev << "\n";
}

}  // namespace detail

inline void WriteBenchmarkJSON(
    const std::string& path,
    const std::string& case_id,
    const std::string& started_at,
    const std::string& finished_at,
    const BenchmarkMetadata& meta,
    const BenchmarkSettings& settings,
    const std::vector<TrialResult>& trials) {

    std::ofstream f(path);
    if (!f.is_open()) {
        std::cerr << "ERROR: Could not open " << path << " for writing\n";
        return;
    }

    // Collect per-trial values for summary
    std::vector<double> sim_vals, total_vals;
    for (const auto& t : trials) {
        sim_vals.push_back(t.sim_wall_s);
        total_vals.push_back(t.total_wall_s);
    }
    auto sim_stats = ComputeStats(sim_vals);
    auto total_stats = ComputeStats(total_vals);

    // Derived metrics
    std::vector<double> sps_vals, rtf_vals;
    for (const auto& t : trials) {
        if (settings.num_steps > 0)
            sps_vals.push_back(t.sim_wall_s / settings.num_steps);
        if (t.sim_wall_s > 0)
            rtf_vals.push_back(settings.sim_duration / t.sim_wall_s);
    }
    auto sps_stats = ComputeStats(sps_vals);
    auto rtf_stats = ComputeStats(rtf_vals);

    // Component breakdown aggregates
    std::vector<double> hs_vals, rad_vals, exc_vals, moor_vals;
    for (const auto& t : trials) {
        hs_vals.push_back(t.components.hydrostatics_s);
        rad_vals.push_back(t.components.radiation_s);
        exc_vals.push_back(t.components.excitation_s);
        moor_vals.push_back(t.components.mooring_s);
    }

    f << "{\n";
    f << "  \"schema_version\": 2,\n";
    f << "  \"case_id\": \"" << case_id << "\",\n";
    f << "  \"started_at\": \"" << started_at << "\",\n";
    f << "  \"finished_at\": \"" << finished_at << "\",\n";

    // Metadata
    f << "  \"metadata\": {\n";
    f << "    \"seastack_version\": \"" << meta.seastack_version << "\",\n";
    f << "    \"git_commit\": \"" << meta.git_commit << "\",\n";
    f << "    \"git_dirty\": " << (meta.git_dirty ? "true" : "false") << ",\n";
    f << "    \"hostname\": \"" << meta.hostname << "\",\n";
    f << "    \"cpu\": \"" << meta.cpu << "\",\n";
    f << "    \"physical_cores\": " << meta.physical_cores << ",\n";
    f << "    \"logical_cores\": " << meta.logical_cores << ",\n";
    f << "    \"omp_max_threads\": " << meta.omp_max_threads << ",\n";
    f << "    \"omp_num_procs\": " << meta.omp_num_procs << ",\n";
    f << "    \"build_type\": \"" << meta.build_type << "\",\n";
    f << "    \"compiler\": \"" << meta.compiler << "\",\n";
    f << "    \"os\": \"" << meta.os << "\"\n";
    f << "  },\n";

    // Settings
    f << "  \"settings\": {\n";
    f << "    \"timestep\": " << std::setprecision(6) << settings.timestep << ",\n";
    f << "    \"sim_duration\": " << std::setprecision(1) << settings.sim_duration << ",\n";
    f << "    \"num_steps\": " << settings.num_steps << ",\n";
    f << "    \"num_bodies\": " << settings.num_bodies << ",\n";
    f << "    \"radiation_method\": \"" << settings.radiation_method << "\",\n";
    f << "    \"excitation_method\": \"" << settings.excitation_method << "\",\n";
    f << "    \"wave_type\": \"" << settings.wave_type << "\",\n";
    f << "    \"moordyn\": " << (settings.moordyn ? "true" : "false") << ",\n";
    f << "    \"num_trials\": " << settings.num_trials << ",\n";
    f << "    \"warmup\": " << (settings.warmup ? "true" : "false") << "\n";
    f << "  },\n";

    // Trials array
    f << "  \"trials\": [\n";
    for (size_t i = 0; i < trials.size(); ++i) {
        const auto& t = trials[i];
        f << "    {\n";
        f << "      \"trial\": " << (i + 1) << ",\n";
        f << "      \"setup_wall_s\": " << std::setprecision(4) << t.setup_wall_s << ",\n";
        f << "      \"sim_wall_s\": " << t.sim_wall_s << ",\n";
        f << "      \"total_wall_s\": " << t.total_wall_s << ",\n";
        f << "      \"component_breakdown\": {\n";
        f << "        \"hydrostatics_s\": " << t.components.hydrostatics_s << ",\n";
        f << "        \"radiation_s\": " << t.components.radiation_s << ",\n";
        f << "        \"excitation_s\": " << t.components.excitation_s << ",\n";
        f << "        \"mooring_s\": " << t.components.mooring_s << "\n";
        f << "      }\n";
        f << "    }";
        if (i + 1 < trials.size()) f << ",";
        f << "\n";
    }
    f << "  ],\n";

    // Summary
    f << "  \"summary\": {\n";
    f << "    \"sim_wall_s\": {\n";
    detail::WriteStatsObject(f, "      ", sim_stats);
    f << "    },\n";
    f << "    \"total_wall_s\": {\n";
    detail::WriteStatsObject(f, "      ", total_stats);
    f << "    },\n";
    f << "    \"s_per_step\": {\n";
    detail::WriteStatsObject(f, "      ", sps_stats);
    f << "    },\n";
    f << "    \"realtime_factor\": {\n";
    detail::WriteStatsObject(f, "      ", rtf_stats);
    f << "    },\n";
    f << "    \"component_breakdown\": {\n";
    f << "      \"hydrostatics_s\": { \"mean\": " << ComputeStats(hs_vals).mean << " },\n";
    f << "      \"radiation_s\": { \"mean\": " << ComputeStats(rad_vals).mean << " },\n";
    f << "      \"excitation_s\": { \"mean\": " << ComputeStats(exc_vals).mean << " },\n";
    f << "      \"mooring_s\": { \"mean\": " << ComputeStats(moor_vals).mean << " }\n";
    f << "    }\n";
    f << "  }\n";
    f << "}\n";

    std::cout << "  Benchmark JSON written to " << path << std::endl;
}

// Console summary for a multi-trial run
inline void PrintTrialSummary(const std::string& case_id,
                              const BenchmarkSettings& settings,
                              const std::vector<TrialResult>& trials) {
    std::vector<double> sim_vals;
    for (const auto& t : trials) sim_vals.push_back(t.sim_wall_s);
    auto stats = ComputeStats(sim_vals);

    std::cout << "\n=== " << case_id << " ===\n"
              << "  Trials: " << trials.size() << "\n"
              << "  sim_wall_s:  mean=" << std::fixed << std::setprecision(2) << stats.mean
              << "  min=" << stats.min_val << "  max=" << stats.max_val
              << "  stddev=" << stats.stddev << "\n";
    if (stats.mean > 0)
        std::cout << "  realtime_factor: " << std::setprecision(2)
                  << (settings.sim_duration / stats.mean) << "x\n";
    std::cout << std::endl;
}

}  // namespace seastack::bench

#endif  // SEASTACK_BENCH_UTILS_H
