/*********************************************************************
 * @file  campaign_runner.cpp
 * @brief Implementation of the power-matrix campaign runner.
 *
 * Parses a campaign YAML, enumerates cells, applies a steepness filter,
 * runs each cell serially via RunSingleCase(), and writes a sparse
 * summary HDF5 (+ optional CSV, + optional AEP).
 *********************************************************************/

#include "campaign_runner.h"
#include "single_run.h"
#include "cell_io.h"

#include <seastack/config.h>
#include <seastack/version.h>
#include <seastack/infra/config/yaml_discovery.h>
#include <seastack/infra/logging.h>
#include <seastack/hydro/config/yaml_parser.h>
#include <seastack/hydro/config/hydro_config.h>

#ifdef SEASTACK_HAVE_HYDRO_IO
#include <H5Cpp.h>
#include <seastack/hydro_io/h5_writer.h>
#endif

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace seastack::app {

using seastack::hydro::YAMLHydroData;
using seastack::hydro::ReadHydroYAML;

// ============================================================================
// Campaign config types (internal)
// ============================================================================
namespace {

struct AxisSpec {
    std::vector<double> values;
};

struct ScatterEntry {
    double hs;
    double tp;
    double weight;
};

struct CampaignConfig {
    // Case paths (resolved)
    std::string case_directory;
    std::string setup_file;   // empty = auto-detect
    std::string model_file;
    std::string simulation_file;
    std::string hydro_file;

    // Axes
    AxisSpec hs;
    AxisSpec tp;
    AxisSpec heading;   // empty = single value from hydro template
    AxisSpec seed;      // empty = single value from hydro template

    // Filters
    double steepness_max = 0.07;
    bool   filters_enabled = true;

    // Scatter (optional)
    std::string scatter_file;
    std::string scatter_hs_column = "Hs";
    std::string scatter_tp_column = "Tp";
    std::string scatter_weight_column = "probability";

    // Output
    std::string output_directory;
    bool per_cell_h5 = false;
    bool csv_output  = false;
    /// Embed per-cell total PTO power time series under /timeseries (serial campaigns only).
    /// Default true; set output.summary_timeseries: false to reduce summary HDF5 size.
    bool summary_timeseries = true;

    // Execution
    int max_workers = 1;  // 1 = serial (in-process); >1 = subprocess parallelism
};

enum class CellStatus : uint8_t {
    kOk             = 0,
    kSkippedInvalid = 1,
    kFailed         = 2
};

struct CellRecord {
    double hs         = 0.0;
    double tp         = 0.0;
    double heading_deg = 0.0;
    int    seed       = 1;
    CellStatus status = CellStatus::kOk;
    std::string skip_reason;

    double mean_absorbed_power_W   = std::numeric_limits<double>::quiet_NaN();
    double total_absorbed_energy_J = std::numeric_limits<double>::quiet_NaN();
    double wall_time_s             = 0.0;
    int    exit_code               = -1;

    std::vector<double> pto_total_time_s;
    std::vector<double> pto_total_power_W;
};

std::string CellRecordLabel(const CellRecord& c) {
    std::ostringstream label;
    label << "Hs=" << c.hs << "_Tp=" << c.tp << "_Dir=" << c.heading_deg << "_S=" << c.seed;
    return label.str();
}

// ============================================================================
// Helpers
// ============================================================================

/// Parse a YAML node that is either a bare list [1,2,3] or {linspace: {start,stop,num}}.
AxisSpec ParseAxisValues(const YAML::Node& node) {
    AxisSpec spec;
    if (node.IsSequence()) {
        for (const auto& v : node) {
            spec.values.push_back(v.as<double>());
        }
    } else if (node.IsMap() && node["linspace"]) {
        auto ls = node["linspace"];
        double start = ls["start"].as<double>();
        double stop  = ls["stop"].as<double>();
        int    num   = ls["num"].as<int>();
        if (num < 1) num = 1;
        for (int i = 0; i < num; ++i) {
            double val = (num == 1) ? start : start + i * (stop - start) / (num - 1);
            spec.values.push_back(val);
        }
    } else if (node.IsScalar()) {
        spec.values.push_back(node.as<double>());
    }
    return spec;
}

/// Deep-water wave steepness proxy: Hs / (g * Tp^2 / (2*pi))
double WaveSteepness(double hs, double tp) {
    constexpr double g  = 9.80665;
    constexpr double pi = 3.14159265358979323846;
    if (tp <= 0.0) return 0.0;
    return hs / (g * tp * tp / (2.0 * pi));
}

/// Parse a simple CSV scatter table.
std::vector<ScatterEntry> LoadScatterCSV(const std::string& path,
                                          const std::string& hs_col,
                                          const std::string& tp_col,
                                          const std::string& weight_col) {
    std::vector<ScatterEntry> entries;
    std::ifstream in(path);
    if (!in.is_open()) return entries;

    std::string header_line;
    if (!std::getline(in, header_line)) return entries;

    // Parse header to find column indices
    std::vector<std::string> headers;
    {
        std::stringstream ss(header_line);
        std::string tok;
        while (std::getline(ss, tok, ',')) {
            // Trim whitespace
            auto ltrim = [](std::string& s) { s.erase(0, s.find_first_not_of(" \t\r\n")); };
            auto rtrim = [](std::string& s) { auto p = s.find_last_not_of(" \t\r\n"); if (p == std::string::npos) s.clear(); else s.erase(p+1); };
            ltrim(tok); rtrim(tok);
            headers.push_back(tok);
        }
    }

    int hs_idx = -1, tp_idx = -1, w_idx = -1;
    for (int i = 0; i < static_cast<int>(headers.size()); ++i) {
        if (headers[i] == hs_col) hs_idx = i;
        if (headers[i] == tp_col) tp_idx = i;
        if (headers[i] == weight_col) w_idx = i;
    }
    if (hs_idx < 0 || tp_idx < 0 || w_idx < 0) return entries;

    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::vector<std::string> cols;
        std::stringstream ss(line);
        std::string tok;
        while (std::getline(ss, tok, ',')) cols.push_back(tok);
        if (static_cast<int>(cols.size()) <= std::max({hs_idx, tp_idx, w_idx})) continue;
        try {
            ScatterEntry e;
            e.hs     = std::stod(cols[hs_idx]);
            e.tp     = std::stod(cols[tp_idx]);
            e.weight = std::stod(cols[w_idx]);
            entries.push_back(e);
        } catch (...) {
            continue;
        }
    }
    return entries;
}

/// Find the nearest scatter entry within tolerance for a given (hs, tp) pair.
/// Returns the weight if a match is found within tol, or -1.0 if no match.
double FindScatterWeight(const std::vector<ScatterEntry>& entries,
                         double hs, double tp,
                         double tol = 1e-3) {
    double best_dist2 = std::numeric_limits<double>::max();
    double best_weight = -1.0;
    for (const auto& e : entries) {
        double dh = e.hs - hs;
        double dt = e.tp - tp;
        double d2 = dh * dh + dt * dt;
        if (d2 < best_dist2) {
            best_dist2 = d2;
            best_weight = e.weight;
        }
    }
    if (std::sqrt(best_dist2) <= tol) {
        return best_weight;
    }
    return -1.0;
}

// ============================================================================
// Campaign YAML parsing
// ============================================================================

CampaignConfig ParseCampaignYAML(const std::string& yaml_path) {
    CampaignConfig cfg;
    auto root = YAML::LoadFile(yaml_path);
    std::filesystem::path yaml_dir = std::filesystem::path(yaml_path).parent_path();

    // Validate kind
    if (!root["kind"] || root["kind"].as<std::string>() != "performance_matrix") {
        throw std::runtime_error("Campaign YAML must have 'kind: performance_matrix'");
    }

    // Case resolution
    if (root["case"]) {
        if (root["case"].IsScalar()) {
            cfg.case_directory = (yaml_dir / root["case"].as<std::string>()).lexically_normal().string();
        } else if (root["case"].IsMap()) {
            if (root["case"]["directory"]) {
                cfg.case_directory = (yaml_dir / root["case"]["directory"].as<std::string>()).lexically_normal().string();
            }
            if (root["case"]["setup"]) {
                cfg.setup_file = root["case"]["setup"].as<std::string>();
            }
        }
    }
    if (cfg.case_directory.empty()) {
        throw std::runtime_error("Campaign YAML must specify 'case' directory");
    }

    // Axes (required: hs and tp; optional: heading, seed)
    if (!root["axes"] || !root["axes"]["hs"] || !root["axes"]["tp"]) {
        throw std::runtime_error("Campaign YAML must have 'axes' with at least 'hs' and 'tp'");
    }
    cfg.hs = ParseAxisValues(root["axes"]["hs"]);
    cfg.tp = ParseAxisValues(root["axes"]["tp"]);
    if (root["axes"]["heading"]) {
        cfg.heading = ParseAxisValues(root["axes"]["heading"]);
    }
    if (root["axes"]["seed"]) {
        cfg.seed = ParseAxisValues(root["axes"]["seed"]);
    }

    // Filters
    if (root["filters"]) {
        if (root["filters"]["steepness_max"]) {
            cfg.steepness_max = root["filters"]["steepness_max"].as<double>();
        }
        if (root["filters"]["enabled"]) {
            cfg.filters_enabled = root["filters"]["enabled"].as<bool>();
        }
    }

    // Scatter
    if (root["scatter"]) {
        if (root["scatter"]["file"]) {
            cfg.scatter_file = (yaml_dir / root["scatter"]["file"].as<std::string>()).lexically_normal().string();
        }
        if (root["scatter"]["hs_column"]) cfg.scatter_hs_column = root["scatter"]["hs_column"].as<std::string>();
        if (root["scatter"]["tp_column"]) cfg.scatter_tp_column = root["scatter"]["tp_column"].as<std::string>();
        if (root["scatter"]["weight_column"]) cfg.scatter_weight_column = root["scatter"]["weight_column"].as<std::string>();
    }

    // Output
    if (root["output"]) {
        if (root["output"]["directory"]) {
            cfg.output_directory = (yaml_dir / root["output"]["directory"].as<std::string>()).lexically_normal().string();
        }
        if (root["output"]["per_cell_h5"]) cfg.per_cell_h5 = root["output"]["per_cell_h5"].as<bool>();
        if (root["output"]["csv"]) cfg.csv_output = root["output"]["csv"].as<bool>();
        if (root["output"]["summary_timeseries"]) {
            cfg.summary_timeseries = root["output"]["summary_timeseries"].as<bool>();
        }
    }
    if (cfg.output_directory.empty()) {
        cfg.output_directory = (yaml_dir / "power_matrix_output").lexically_normal().string();
    }

    // Execution
    if (root["execution"]) {
        if (root["execution"]["max_workers"]) {
            cfg.max_workers = std::max(1, root["execution"]["max_workers"].as<int>());
        }
    }

    return cfg;
}

/// Resolve model/sim/hydro from the case directory using setup discovery.
bool ResolveCaseFiles(CampaignConfig& cfg) {
    namespace fs = std::filesystem;
    fs::path case_dir(cfg.case_directory);
    if (!fs::exists(case_dir) || !fs::is_directory(case_dir)) {
        seastack::infra::cli::LogError("Campaign case directory not found: " + cfg.case_directory);
        return false;
    }

    auto setup_path = seastack::infra::FindSetupFile(case_dir);
    if (setup_path.empty() && !cfg.setup_file.empty()) {
        setup_path = case_dir / cfg.setup_file;
    }

    if (!setup_path.empty() && fs::exists(setup_path)) {
        auto sc = seastack::infra::ParseSetupFile(setup_path);
        if (sc.has_model_file)
            cfg.model_file = (case_dir / sc.model_file).lexically_normal().string();
        if (sc.has_simulation_file)
            cfg.simulation_file = (case_dir / sc.simulation_file).lexically_normal().string();
        if (sc.has_hydro_file)
            cfg.hydro_file = (case_dir / sc.hydro_file).lexically_normal().string();
    }

    // Fallback auto-detection
    auto find_first = [&](const std::string& pattern) -> std::string {
        std::vector<fs::path> matches;
        for (const auto& entry : fs::directory_iterator(case_dir)) {
            if (entry.is_regular_file() && entry.path().filename().string().find(pattern) != std::string::npos) {
                matches.push_back(entry.path());
            }
        }
        if (!matches.empty()) {
            std::sort(matches.begin(), matches.end());
            return matches.front().lexically_normal().string();
        }
        return {};
    };

    if (cfg.model_file.empty()) cfg.model_file = find_first(".model.yaml");
    if (cfg.simulation_file.empty()) cfg.simulation_file = find_first(".simulation.yaml");
    if (cfg.hydro_file.empty()) cfg.hydro_file = find_first(".hydro.yaml");

    if (cfg.model_file.empty() || cfg.simulation_file.empty()) {
        seastack::infra::cli::LogError("Could not resolve model and simulation YAML from case: " + cfg.case_directory);
        return false;
    }

    return true;
}

#ifdef SEASTACK_HAVE_HYDRO_IO
/// Floating-point compare for axis values (YAML-derived doubles in cells vs axes).
bool NearlyEqualAxis(double a, double b) {
    const double scale = std::max({1.0, std::abs(a), std::abs(b)});
    return std::abs(a - b) <= 1e-9 * scale;
}

/// When the campaign is a full hs×tp grid with one heading and one seed, fill row-major
/// 2D buffers (index = i_hs * n_tp + i_tp) for /matrix datasets. Returns false if layout
/// does not match enumeration order or axis values disagree.
bool FillPowerMatrixBuffers(const CampaignConfig& cfg,
                            const std::vector<CellRecord>& cells,
                            size_t& n_hs_out,
                            size_t& n_tp_out,
                            std::vector<double>& power2d,
                            std::vector<double>& energy2d,
                            std::vector<double>& status2d) {
    const size_t n_hs = cfg.hs.values.size();
    const size_t n_tp = cfg.tp.values.size();
    n_hs_out = n_hs;
    n_tp_out = n_tp;
    if (cfg.heading.values.size() != 1 || cfg.seed.values.size() != 1) {
        return false;
    }
    if (n_hs == 0 || n_tp == 0) {
        return false;
    }
    if (cells.size() != n_hs * n_tp) {
        return false;
    }

    const double nanv = std::numeric_limits<double>::quiet_NaN();
    power2d.assign(n_hs * n_tp, nanv);
    energy2d.assign(n_hs * n_tp, nanv);
    status2d.assign(n_hs * n_tp, nanv);

    const double heading0 = cfg.heading.values[0];
    const int seed0       = static_cast<int>(std::lround(cfg.seed.values[0]));

    size_t idx = 0;
    for (size_t i_hs = 0; i_hs < n_hs; ++i_hs) {
        for (size_t i_tp = 0; i_tp < n_tp; ++i_tp, ++idx) {
            const CellRecord& c = cells[idx];
            if (!NearlyEqualAxis(c.hs, cfg.hs.values[i_hs]) ||
                !NearlyEqualAxis(c.tp, cfg.tp.values[i_tp]) ||
                !NearlyEqualAxis(c.heading_deg, heading0) || c.seed != seed0) {
                return false;
            }
            const size_t k = i_hs * n_tp + i_tp;
            status2d[k] = static_cast<double>(static_cast<uint8_t>(c.status));
            if (c.status == CellStatus::kOk) {
                power2d[k]  = c.mean_absorbed_power_W;
                energy2d[k] = c.total_absorbed_energy_J;
            }
        }
    }
    return true;
}

/// Write the sparse summary HDF5 file.
void WriteSummaryHDF5(const std::string& path,
                      const std::vector<CellRecord>& cells,
                      const CampaignConfig& cfg,
                      const std::string& campaign_yaml_path,
                      double aep_wh,
                      bool scatter_requested,
                      int matched_cells,
                      int total_ok_cells) {
    seastack::hydro_io::H5Writer writer(path);

    // /meta
    auto g_meta = writer.RequireGroup("/meta");
    g_meta.WriteAttribute("version", std::string("1"));
    g_meta.WriteAttribute("seastack_version", std::string(SEASTACK_VERSION));
    g_meta.WriteAttribute("campaign_file", campaign_yaml_path);
    g_meta.WriteAttribute("model_file", cfg.model_file);
    g_meta.WriteAttribute("simulation_file", cfg.simulation_file);
    g_meta.WriteAttribute("hydro_template", cfg.hydro_file);
    g_meta.WriteAttribute("mean_absorbed_power_W_units", std::string("W"));
    g_meta.WriteAttribute("mean_absorbed_power_W_long_name",
                          std::string("time_mean_viscous_damper_power_sum_PTOs"));
    g_meta.WriteAttribute(
        "mean_absorbed_power_W_definition",
        std::string("Sum over exported PTO links (TSDA/RSDA) of time-mean viscous damper power: "
                    "c*v^2 (TSDA) and c*omega^2 (RSDA) using Chrono attachment rates; c is taken from "
                    "the link when set, else from model YAML damping_coefficient (Chrono MBS YAML often "
                    "leaves link damping zero). Excludes spring and preload mechanical power. TSDA links "
                    "using SEA-Stack PTOForceFunctor (IPTOModel) fall back to time-mean -(F*v) instead "
                    "of c*v^2; the CLI logs a one-time warning per link. Non-negative for linear damper "
                    "power when c*v^2 / c*omega^2 applies."));
    g_meta.WriteAttribute("total_absorbed_energy_J_units", std::string("J"));
    g_meta.WriteAttribute("total_absorbed_energy_J_long_name",
                          std::string("total_absorbed_energy_sum_PTOs_end_of_run"));

    const size_t n = cells.size();
    bool any_timeseries = false;
    for (size_t i = 0; i < n; ++i) {
        if (!cells[i].pto_total_time_s.empty() &&
            cells[i].pto_total_time_s.size() == cells[i].pto_total_power_W.size()) {
            any_timeseries = true;
            break;
        }
    }
    if (any_timeseries) {
        g_meta.WriteAttribute("summary_timeseries", std::string("true"));
        g_meta.WriteAttribute(
            "total_pto_power_timeseries_definition",
            std::string("Under /timeseries/cell_*: decimated time base (time_s) and sum of all exported "
                        "TSDA/RSDA absorbed_power samples (total_absorbed_power_W) per step — same "
                        "physics semantics as mean_absorbed_power_W (see mean_absorbed_power_W_definition)."));
    }

    std::array<hsize_t, 1> dims{static_cast<hsize_t>(n)};

    // Build column vectors
    std::vector<double> hs_vec(n), tp_vec(n), heading_vec(n), seed_vec(n);
    std::vector<double> status_vec(n);
    std::vector<std::string> reason_vec(n);
    std::vector<double> power_vec(n), energy_vec(n), wall_vec(n), exit_vec(n);

    for (size_t i = 0; i < n; ++i) {
        hs_vec[i]      = cells[i].hs;
        tp_vec[i]      = cells[i].tp;
        heading_vec[i] = cells[i].heading_deg;
        seed_vec[i]    = static_cast<double>(cells[i].seed);
        status_vec[i]  = static_cast<double>(static_cast<uint8_t>(cells[i].status));
        reason_vec[i]  = cells[i].skip_reason;
        power_vec[i]   = cells[i].mean_absorbed_power_W;
        energy_vec[i]  = cells[i].total_absorbed_energy_J;
        wall_vec[i]    = cells[i].wall_time_s;
        exit_vec[i]    = static_cast<double>(cells[i].exit_code);
    }

    auto g_cells = writer.RequireGroup("/cells");
    g_cells.WriteAttribute("table_role", std::string("sparse_cell_records"));
    g_cells.WriteDataset("hs",                      hs_vec,      dims);
    g_cells.WriteDataset("tp",                      tp_vec,      dims);
    g_cells.WriteDataset("heading_deg",             heading_vec, dims);
    g_cells.WriteDataset("seed",                    seed_vec,    dims);
    g_cells.WriteDataset("status",                  status_vec,  dims);
    g_cells.WriteStringArray("skip_reason",         reason_vec);
    g_cells.WriteDataset("mean_absorbed_power_W",   power_vec,   dims);
    g_cells.WriteDataset("total_absorbed_energy_J", energy_vec,  dims);
    g_cells.WriteDataset("wall_time_s",             wall_vec,    dims);
    g_cells.WriteDataset("exit_code",               exit_vec,    dims);

    // /axes
    auto g_axes = writer.RequireGroup("/axes");
    {
        std::array<hsize_t, 1> hs_d{static_cast<hsize_t>(cfg.hs.values.size())};
        g_axes.WriteDataset("hs", cfg.hs.values, hs_d);
    }
    {
        std::array<hsize_t, 1> tp_d{static_cast<hsize_t>(cfg.tp.values.size())};
        g_axes.WriteDataset("tp", cfg.tp.values, tp_d);
    }

    // /matrix — dense hs×tp slabs for HDFView (single heading, single seed, full grid only)
    size_t n_hs_mat = 0;
    size_t n_tp_mat = 0;
    std::vector<double> power_mat;
    std::vector<double> energy_mat;
    std::vector<double> status_mat;
    if (FillPowerMatrixBuffers(cfg, cells, n_hs_mat, n_tp_mat, power_mat, energy_mat, status_mat)) {
        auto g_matrix = writer.RequireGroup("/matrix");
        g_matrix.WriteAttribute("layout", std::string("row_major"));
        g_matrix.WriteAttribute("axis_0", std::string("hs"));
        g_matrix.WriteAttribute("axis_1", std::string("tp"));
        g_matrix.WriteAttribute("mean_absorbed_power_W_units", std::string("W"));
        g_matrix.WriteAttribute("total_absorbed_energy_J_units", std::string("J"));
        g_matrix.WriteAttribute("cell_status_codes", std::string("0=ok 1=skipped_invalid 2=failed"));
        std::array<hsize_t, 1> ax0{static_cast<hsize_t>(n_hs_mat)};
        std::array<hsize_t, 1> ax1{static_cast<hsize_t>(n_tp_mat)};
        g_matrix.WriteDataset("hs", cfg.hs.values, ax0);
        g_matrix.WriteDataset("tp", cfg.tp.values, ax1);
        std::array<hsize_t, 2> dim2{static_cast<hsize_t>(n_hs_mat), static_cast<hsize_t>(n_tp_mat)};
        g_matrix.WriteDataset("mean_absorbed_power_W", power_mat, dim2);
        g_matrix.WriteDataset("total_absorbed_energy_J", energy_mat, dim2);
        g_matrix.WriteDataset("cell_status", status_mat, dim2);
    }

    if (any_timeseries) {
        seastack::hydro_io::WriteOptions wopts;
        wopts.compression_level = 1;
        auto g_ts = writer.RequireGroup("/timeseries");
        g_ts.WriteAttribute("time_units", std::string("s"));
        g_ts.WriteAttribute("total_absorbed_power_W_units", std::string("W"));
        g_ts.WriteAttribute("cell_group_naming", std::string("cell_<index> matches row index in /cells"));
        for (size_t i = 0; i < n; ++i) {
            if (cells[i].pto_total_time_s.empty() ||
                cells[i].pto_total_time_s.size() != cells[i].pto_total_power_W.size()) {
                continue;
            }
            std::ostringstream gname;
            gname << "cell_" << std::setw(5) << std::setfill('0') << i;
            auto gc = g_ts.CreateGroup(gname.str());
            std::array<hsize_t, 1> d1{static_cast<hsize_t>(cells[i].pto_total_time_s.size())};
            gc.WriteDataset("time_s", cells[i].pto_total_time_s, d1, wopts);
            gc.WriteDataset("total_absorbed_power_W", cells[i].pto_total_power_W, d1, wopts);
            gc.WriteAttribute("hs", cells[i].hs);
            gc.WriteAttribute("tp", cells[i].tp);
            gc.WriteAttribute("heading_deg", cells[i].heading_deg);
            gc.WriteAttribute("seed", static_cast<double>(cells[i].seed));
            gc.WriteAttribute("cell_label", CellRecordLabel(cells[i]));
        }
    }

    // /aep — always written when scatter data was requested
    if (scatter_requested) {
        auto g_aep = writer.RequireGroup("/aep");
        g_aep.WriteAttribute("annual_energy_Wh", aep_wh);
        g_aep.WriteAttribute("hours_per_year", 8766.0);
        g_aep.WriteAttribute("method", std::string("weighted_sum"));
        g_aep.WriteAttribute("matched_cells", static_cast<double>(matched_cells));
        g_aep.WriteAttribute("total_ok_cells", static_cast<double>(total_ok_cells));
    }
}
#endif

/// Write flat CSV summary.
void WriteSummaryCSV(const std::string& path,
                     const std::vector<CellRecord>& cells) {
    std::ofstream out(path);
    if (!out.is_open()) {
        seastack::infra::cli::LogWarning("Could not write CSV: " + path);
        return;
    }
    out << "hs,tp,heading_deg,seed,status,skip_reason,mean_absorbed_power_W,"
           "total_absorbed_energy_J,wall_time_s\n";
    for (const auto& c : cells) {
        out << c.hs << "," << c.tp << "," << c.heading_deg << "," << c.seed << ","
            << static_cast<int>(c.status) << ",\"" << c.skip_reason << "\","
            << c.mean_absorbed_power_W << "," << c.total_absorbed_energy_J << ","
            << c.wall_time_s << "\n";
    }
}

}  // anonymous namespace


// ============================================================================
// RunCampaign
// ============================================================================
int RunCampaign(const std::string& campaign_yaml_path,
                bool debug_mode,
                bool profile_mode,
                bool quiet_mode) {
    namespace fs = std::filesystem;
    const bool concise_cells = !debug_mode;
    const bool subprocess_quiet = !debug_mode || quiet_mode;

    seastack::infra::cli::ShowSectionSeparator();
    seastack::infra::cli::LogInfo("[campaign] Power matrix generation starting");

    // 1. Parse campaign YAML
    CampaignConfig cfg;
    try {
        cfg = ParseCampaignYAML(campaign_yaml_path);
    } catch (const std::exception& e) {
        seastack::infra::cli::LogError(std::string("[campaign] YAML parse error: ") + e.what());
        return 1;
    }

    // 2. Resolve base-case files
    if (!ResolveCaseFiles(cfg)) {
        return 1;
    }
    seastack::infra::cli::LogInfo("[campaign] Base case: " + cfg.case_directory);
    seastack::infra::cli::LogInfo("[campaign] Model:     " + fs::path(cfg.model_file).filename().string());
    seastack::infra::cli::LogInfo("[campaign] Sim:       " + fs::path(cfg.simulation_file).filename().string());
    seastack::infra::cli::LogInfo("[campaign] Hydro:     " + (cfg.hydro_file.empty() ? "none" : fs::path(cfg.hydro_file).filename().string()));

    // 3. Load hydro template (needed for default heading/seed)
    YAMLHydroData hydro_template;
    if (!cfg.hydro_file.empty() && fs::exists(cfg.hydro_file)) {
        hydro_template = ReadHydroYAML(cfg.hydro_file);
    }

    // Fill in default heading/seed from template if not specified in axes
    if (cfg.heading.values.empty()) {
        cfg.heading.values.push_back(hydro_template.waves.direction);
    }
    if (cfg.seed.values.empty()) {
        int template_seed = (hydro_template.waves.seed >= 0) ? hydro_template.waves.seed : 1;
        cfg.seed.values.push_back(static_cast<double>(template_seed));
    }

    // 4. Enumerate cells (Cartesian product) and apply steepness filter
    std::vector<CellRecord> cells;
    for (double hs : cfg.hs.values) {
        for (double tp : cfg.tp.values) {
            for (double heading : cfg.heading.values) {
                for (double seed_d : cfg.seed.values) {
                    CellRecord cell;
                    cell.hs          = hs;
                    cell.tp          = tp;
                    cell.heading_deg = heading;
                    cell.seed        = static_cast<int>(seed_d);

                    if (cfg.filters_enabled) {
                        double steepness = WaveSteepness(hs, tp);
                        if (steepness > cfg.steepness_max) {
                            cell.status = CellStatus::kSkippedInvalid;
                            std::ostringstream reason;
                            reason << std::fixed << std::setprecision(4)
                                   << "steepness " << steepness << " > " << cfg.steepness_max;
                            cell.skip_reason = reason.str();
                        }
                    }
                    cells.push_back(cell);
                }
            }
        }
    }

    int n_total   = static_cast<int>(cells.size());
    int n_skipped = static_cast<int>(std::count_if(cells.begin(), cells.end(),
        [](const CellRecord& c){ return c.status == CellStatus::kSkippedInvalid; }));
    int n_runnable = n_total - n_skipped;

    seastack::infra::cli::LogInfo("[campaign] Cells: " + std::to_string(n_total) +
        " total, " + std::to_string(n_runnable) + " runnable, " +
        std::to_string(n_skipped) + " filtered");

    // 5. Create output directory
    std::error_code ec;
    fs::create_directories(cfg.output_directory, ec);
    {
        fs::path out_abs = fs::absolute(cfg.output_directory);
        seastack::infra::cli::LogInfo("[campaign] Output directory (absolute): " + out_abs.string());
#ifdef SEASTACK_HAVE_HYDRO_IO
        seastack::infra::cli::LogInfo("[campaign] Summary HDF5 will be written to: " +
                                      (out_abs / "power_matrix_summary.h5").string());
#endif
        if (cfg.csv_output) {
            seastack::infra::cli::LogInfo("[campaign] Summary CSV will be written to: " +
                                          (out_abs / "power_matrix_summary.csv").string());
        }
    }

    const bool capture_summary_timeseries =
        cfg.summary_timeseries && cfg.max_workers <= 1;
    if (cfg.summary_timeseries && cfg.max_workers > 1) {
        seastack::infra::cli::LogWarning(
            "[campaign] output.summary_timeseries is ignored when execution.max_workers > 1 "
            "(subprocess IPC does not return time series). Use max_workers: 1.");
    }

    // Build cell labels and base configs for runnable cells
    auto MakeCellLabel = [](const CellRecord& c) {
        std::ostringstream label;
        label << "Hs=" << c.hs << "_Tp=" << c.tp
              << "_Dir=" << c.heading_deg << "_S=" << c.seed;
        return label.str();
    };

    auto ApplyResult = [](CellRecord& cell, const SingleRunResult& result) {
        cell.exit_code   = result.exit_code;
        cell.wall_time_s = result.wall_time_s;
        cell.pto_total_time_s.clear();
        cell.pto_total_power_W.clear();
        if (result.exit_code == 0 && !result.diverged) {
            cell.status = CellStatus::kOk;
            cell.mean_absorbed_power_W   = result.total_mean_power_W;
            cell.total_absorbed_energy_J = result.total_energy_J;
            cell.pto_total_time_s        = result.pto_total_time_s;
            cell.pto_total_power_W       = result.pto_total_power_W;
        } else {
            cell.status = CellStatus::kFailed;
            cell.skip_reason = result.error_message.empty()
                ? (result.diverged ? "diverged" : "unknown failure")
                : result.error_message;
        }
    };

    // 6. Execute cells
    if (cfg.max_workers <= 1) {
        // ---- Serial (in-process) execution ----
        int run_index = 0;
        for (auto& cell : cells) {
            if (cell.status == CellStatus::kSkippedInvalid) continue;

            run_index++;
            std::string label = MakeCellLabel(cell);

            if (debug_mode) {
                seastack::infra::cli::ShowSectionSeparator();
            }
            seastack::infra::cli::LogInfo("[campaign] Cell " + std::to_string(run_index) +
                "/" + std::to_string(n_runnable) + ": " + label);

            YAMLHydroData cell_hydro = hydro_template;
            cell_hydro.waves.height    = cell.hs;
            cell_hydro.waves.period    = cell.tp;
            cell_hydro.waves.direction = cell.heading_deg;
            cell_hydro.waves.seed      = cell.seed;

            SingleRunConfig run_cfg;
            run_cfg.model_file      = cfg.model_file;
            run_cfg.simulation_file = cfg.simulation_file;
            run_cfg.hydro_source    = cell_hydro;
            run_cfg.nogui           = true;
            run_cfg.debug_mode      = debug_mode;
            run_cfg.profile_mode    = profile_mode;
            run_cfg.cell_label      = label;
            run_cfg.concise_cli     = concise_cells;
            run_cfg.capture_pto_total_timeseries = capture_summary_timeseries;

            std::string cell_diag_dir = (fs::path(cfg.output_directory) / label).string();
            fs::create_directories(cell_diag_dir, ec);
            run_cfg.diagnostics_output_directory = cell_diag_dir;

            if (cfg.per_cell_h5) {
                std::string cell_dir = (fs::path(cfg.output_directory) / label).string();
                fs::create_directories(cell_dir, ec);
                run_cfg.output_directory = cell_dir;
            }

            SingleRunResult result = RunSingleCase(run_cfg);
            ApplyResult(cell, result);

            seastack::infra::cli::LogInfo("[campaign] Cell " + label +
                " -> " + (cell.status == CellStatus::kOk ? "OK" : "FAILED") +
                " (wall=" + seastack::infra::FormatNumber(cell.wall_time_s, 1) + "s" +
                ", P=" + seastack::infra::FormatNumber(cell.mean_absorbed_power_W, 1) + " W)");
        }
    } else {
        // ---- Parallel (subprocess) execution [EXPERIMENTAL] ----
        seastack::infra::cli::LogWarning(
            "[campaign] Parallel execution is experimental. "
            "Set execution.max_workers: 1 (or omit it) for stable serial mode.");
        seastack::infra::cli::LogInfo("[campaign] Parallel mode: max_workers=" +
            std::to_string(cfg.max_workers));

        // Locate the executable for subprocess spawning.
        std::string exe_path;
        bool use_serial_fallback = false;
        try {
            std::string exe_dir = seastack::infra::GetExecutableDirectory();
            if (!exe_dir.empty()) {
#ifdef _WIN32
                exe_path = (fs::path(exe_dir) / "run_seastack.exe").string();
#else
                exe_path = (fs::path(exe_dir) / "run_seastack").string();
#endif
            }
        } catch (...) {}
        if (exe_path.empty() || !fs::exists(exe_path)) {
            seastack::infra::cli::LogWarning(
                "[campaign] Cannot locate run_seastack executable for parallel mode; "
                "falling back to serial execution.");
            use_serial_fallback = true;
        }

        if (use_serial_fallback) {
            int run_index = 0;
            for (auto& cell : cells) {
                if (cell.status == CellStatus::kSkippedInvalid) continue;
                run_index++;
                std::string label = MakeCellLabel(cell);
                if (debug_mode) {
                    seastack::infra::cli::ShowSectionSeparator();
                }
                seastack::infra::cli::LogInfo("[campaign] Cell " + std::to_string(run_index) +
                    "/" + std::to_string(n_runnable) + ": " + label + " (serial fallback)");

                YAMLHydroData cell_hydro = hydro_template;
                cell_hydro.waves.height    = cell.hs;
                cell_hydro.waves.period    = cell.tp;
                cell_hydro.waves.direction = cell.heading_deg;
                cell_hydro.waves.seed      = cell.seed;

                SingleRunConfig run_cfg;
                run_cfg.model_file      = cfg.model_file;
                run_cfg.simulation_file = cfg.simulation_file;
                run_cfg.hydro_source    = cell_hydro;
                run_cfg.nogui           = true;
                run_cfg.debug_mode      = debug_mode;
                run_cfg.profile_mode    = profile_mode;
                run_cfg.cell_label      = label;
                run_cfg.concise_cli     = concise_cells;
                run_cfg.capture_pto_total_timeseries = capture_summary_timeseries;

                std::string cell_diag_dir = (fs::path(cfg.output_directory) / label).string();
                fs::create_directories(cell_diag_dir, ec);
                run_cfg.diagnostics_output_directory = cell_diag_dir;

                if (cfg.per_cell_h5) {
                    std::string cell_dir = (fs::path(cfg.output_directory) / label).string();
                    fs::create_directories(cell_dir, ec);
                    run_cfg.output_directory = cell_dir;
                }

                SingleRunResult result = RunSingleCase(run_cfg);
                ApplyResult(cell, result);
                seastack::infra::cli::LogInfo("[campaign] Cell " + label +
                    " -> " + (cell.status == CellStatus::kOk ? "OK" : "FAILED") +
                    " (wall=" + seastack::infra::FormatNumber(cell.wall_time_s, 1) + "s" +
                    ", P=" + seastack::infra::FormatNumber(cell.mean_absorbed_power_W, 1) + " W)");
            }
        } else {

        // Prepare cell configs as YAML files in a temp subdirectory
        fs::path cells_dir = fs::path(cfg.output_directory) / ".cells";
        fs::create_directories(cells_dir, ec);

        // Build list of runnable cell indices
        std::vector<size_t> runnable_indices;
        for (size_t i = 0; i < cells.size(); ++i) {
            if (cells[i].status != CellStatus::kSkippedInvalid) {
                runnable_indices.push_back(i);
            }
        }

        // Write all cell configs
        std::vector<std::string> config_paths(cells.size());
        for (size_t idx : runnable_indices) {
            auto& cell = cells[idx];
            std::string label = MakeCellLabel(cell);

            SingleRunConfig run_cfg;
            run_cfg.model_file      = cfg.model_file;
            run_cfg.simulation_file = cfg.simulation_file;
            run_cfg.hydro_source    = cfg.hydro_file;
            run_cfg.nogui           = true;
            run_cfg.debug_mode      = debug_mode;
            run_cfg.profile_mode    = profile_mode;
            run_cfg.cell_label      = label;
            run_cfg.concise_cli     = concise_cells;

            std::string cell_diag_dir = (fs::path(cfg.output_directory) / label).string();
            fs::create_directories(cell_diag_dir, ec);
            run_cfg.diagnostics_output_directory = cell_diag_dir;

            if (cfg.per_cell_h5) {
                std::string cell_dir = (fs::path(cfg.output_directory) / label).string();
                fs::create_directories(cell_dir, ec);
                run_cfg.output_directory = cell_dir;
            }

            std::string cfg_path = (cells_dir / ("cell_" + std::to_string(idx) + ".yaml")).string();
            WriteCellConfigYAML(cfg_path, run_cfg,
                                cell.hs, cell.tp, cell.heading_deg, cell.seed);
            config_paths[idx] = cfg_path;
        }

        // Launch subprocesses using a simple thread pool (each thread spawns
        // one subprocess via std::system and blocks until it completes).
        auto RunSubprocess = [&](size_t idx) -> SingleRunResult {
            std::string cmd = "\"" + exe_path + "\"";
            if (subprocess_quiet) {
                cmd += " --quiet";
            }
            if (debug_mode) {
                cmd += " --debug";
            }
            cmd += " --run-cell \"" + config_paths[idx] + "\"";
            int sys_ret = std::system(cmd.c_str());

            std::string result_path = config_paths[idx] + ".result.yaml";
            if (fs::exists(result_path)) {
                return ReadCellResultYAML(result_path);
            }
            SingleRunResult fail;
            fail.exit_code = (sys_ret != 0) ? sys_ret : 1;
            fail.error_message = "Subprocess exited with code " +
                std::to_string(sys_ret) + "; no result file written";
            return fail;
        };

        // Process in batches of max_workers
        size_t cursor = 0;
        int completed = 0;
        while (cursor < runnable_indices.size()) {
            size_t batch_size = std::min(
                static_cast<size_t>(cfg.max_workers),
                runnable_indices.size() - cursor);

            std::vector<std::future<SingleRunResult>> futures;
            futures.reserve(batch_size);

            for (size_t b = 0; b < batch_size; ++b) {
                size_t idx = runnable_indices[cursor + b];
                futures.push_back(std::async(std::launch::async, RunSubprocess, idx));
            }

            for (size_t b = 0; b < batch_size; ++b) {
                size_t idx = runnable_indices[cursor + b];
                SingleRunResult result = futures[b].get();
                ApplyResult(cells[idx], result);
                completed++;

                std::string label = MakeCellLabel(cells[idx]);
                seastack::infra::cli::LogInfo("[campaign] [" + std::to_string(completed) +
                    "/" + std::to_string(n_runnable) + "] " + label +
                    " -> " + (cells[idx].status == CellStatus::kOk ? "OK" : "FAILED") +
                    " (wall=" + seastack::infra::FormatNumber(cells[idx].wall_time_s, 1) + "s)");
            }

            cursor += batch_size;
        }

        // Clean up temp cell configs
        try { fs::remove_all(cells_dir); } catch (...) {}

        }  // end actual parallel path
    }

    // 7. Compute AEP if scatter data present
    double aep_wh = 0.0;
    bool scatter_requested = !cfg.scatter_file.empty();
    int scatter_matched_cells = 0;
    int scatter_total_ok_cells = 0;
    if (scatter_requested && fs::exists(cfg.scatter_file)) {
        auto scatter_entries = LoadScatterCSV(cfg.scatter_file,
                                              cfg.scatter_hs_column,
                                              cfg.scatter_tp_column,
                                              cfg.scatter_weight_column);
        if (!scatter_entries.empty()) {
            constexpr double hours_per_year = 8766.0;
            for (const auto& cell : cells) {
                if (cell.status != CellStatus::kOk) continue;
                scatter_total_ok_cells++;
                double w = FindScatterWeight(scatter_entries, cell.hs, cell.tp);
                if (w >= 0.0) {
                    aep_wh += cell.mean_absorbed_power_W * w * hours_per_year;
                    scatter_matched_cells++;
                }
            }

            if (scatter_matched_cells == 0) {
                seastack::infra::cli::LogError(
                    "[campaign] AEP: no scatter entries matched any OK cells — "
                    "check that scatter CSV (Hs, Tp) values align with the campaign axes.");
            } else if (scatter_matched_cells < scatter_total_ok_cells / 2) {
                seastack::infra::cli::LogWarning(
                    "[campaign] AEP: only " + std::to_string(scatter_matched_cells) +
                    " of " + std::to_string(scatter_total_ok_cells) +
                    " OK cells matched scatter entries.");
            }
            seastack::infra::cli::LogInfo("[campaign] AEP = " +
                seastack::infra::FormatNumber(aep_wh / 1000.0, 1) + " kWh/year" +
                " (" + std::to_string(scatter_matched_cells) + "/" +
                std::to_string(scatter_total_ok_cells) + " cells matched)");
        } else {
            seastack::infra::cli::LogWarning("[campaign] Scatter file could not be parsed: " + cfg.scatter_file);
        }
    } else if (scatter_requested) {
        seastack::infra::cli::LogWarning("[campaign] Scatter file not found: " + cfg.scatter_file);
    }

    // 8. Write summary outputs
    seastack::infra::cli::ShowSectionSeparator();

#ifdef SEASTACK_HAVE_HYDRO_IO
    std::string h5_path = (fs::path(cfg.output_directory) / "power_matrix_summary.h5").string();
    try {
        WriteSummaryHDF5(h5_path, cells, cfg, campaign_yaml_path, aep_wh,
                         scatter_requested, scatter_matched_cells, scatter_total_ok_cells);
        seastack::infra::cli::LogSuccess("[campaign] Summary HDF5: " + h5_path);
    } catch (const H5::Exception& e) {
        seastack::infra::cli::LogError(
            std::string("[campaign] Failed to write summary HDF5 (HDF5): ") + e.getFuncName() + " — " +
            e.getDetailMsg());
    } catch (const std::exception& e) {
        seastack::infra::cli::LogError("[campaign] Failed to write summary HDF5: " + std::string(e.what()));
    } catch (...) {
        seastack::infra::cli::LogError(
            "[campaign] Failed to write summary HDF5: unexpected exception (possible HDF5 cross-library issue).");
    }
#else
    seastack::infra::cli::LogWarning("[campaign] HDF5 support not available — summary file not written.");
#endif

    if (cfg.csv_output) {
        std::string csv_path = (fs::path(cfg.output_directory) / "power_matrix_summary.csv").string();
        WriteSummaryCSV(csv_path, cells);
        seastack::infra::cli::LogSuccess("[campaign] Summary CSV:  " + csv_path);
    }

    // 9. Final summary
    int n_ok     = static_cast<int>(std::count_if(cells.begin(), cells.end(),
        [](const CellRecord& c){ return c.status == CellStatus::kOk; }));
    int n_failed = static_cast<int>(std::count_if(cells.begin(), cells.end(),
        [](const CellRecord& c){ return c.status == CellStatus::kFailed; }));

    std::vector<std::string> summary_lines;
    summary_lines.push_back(seastack::infra::cli::CreateAlignedLine("📊", "Total cells", std::to_string(n_total)));
    summary_lines.push_back(seastack::infra::cli::CreateAlignedLine("✅", "Completed", std::to_string(n_ok)));
    summary_lines.push_back(seastack::infra::cli::CreateAlignedLine("⏭️", "Filtered", std::to_string(n_skipped)));
    if (n_failed > 0) {
        summary_lines.push_back(seastack::infra::cli::CreateAlignedLine("❌", "Failed", std::to_string(n_failed)));
    }
    if (aep_wh > 0.0) {
        summary_lines.push_back(seastack::infra::cli::CreateAlignedLine("⚡", "AEP", seastack::infra::FormatNumber(aep_wh / 1000.0, 1) + " kWh/year"));
    }
    seastack::infra::cli::ShowSectionBox("Power Matrix Results", summary_lines);

    return (n_failed > 0) ? 3 : 0;
}

}  // namespace seastack::app
