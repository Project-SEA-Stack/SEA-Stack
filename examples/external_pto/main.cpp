// Minimal Chrono + ExternalPtoModel example.
// Compares an external (Python or mock) linear damper against in-process LinearPTO
// kinematics by applying both force laws at a fixed (disp, vel) sample.
//
// For a live Chrono step, wire ExternalPtoModel through PTOForceFunctor onto a
// ChLinkTSDA (see demos/rm3/demo_rm3_hydraulic_pto.cpp for the pattern).
//
// Usage:
//   external_pto_example
//   external_pto_example --python path/to/linear_damper_pto.py
//   external_pto_example --mock   (uses mock_external_module if on PATH)
//   external_pto_example --replay --python script.py --config '{...}' \
//       --dt 0.01 --trace in.csv --out out.csv
//
// --replay drives a prescribed (time,displacement,velocity) CSV through the
// full IPC transport (ExternalPtoModel + IpcExternalForceModel) and writes
// time,force to --out. compare_ipc_replay.py then checks that the IPC result
// matches an in-process run of the same module.

#include <seastack/external/external_pto_model.h>
#include <seastack/external/ipc_external_force_model.h>
#include <seastack/pto/linear_pto.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::string FindPython() {
    const char* env = std::getenv("PYTHON");
    if (env && env[0] != '\0') {
        return env;
    }
#ifdef _WIN32
    return "python";
#else
    return "python3";
#endif
}

struct TraceRow {
    double t;
    double disp;
    double vel;
};

// Reads a CSV with header "time,displacement,velocity".
std::vector<TraceRow> ReadTrace(const std::string& path) {
    std::vector<TraceRow> rows;
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("cannot open trace: " + path);
    }
    std::string line;
    std::getline(in, line);  // header
    while (std::getline(in, line)) {
        if (line.empty()) {
            continue;
        }
        std::stringstream ss(line);
        std::string a, b, c;
        std::getline(ss, a, ',');
        std::getline(ss, b, ',');
        std::getline(ss, c, ',');
        rows.push_back({std::stod(a), std::stod(b), std::stod(c)});
    }
    return rows;
}

// Runs a prescribed trace through the IPC transport and writes time,force.
int RunReplay(const std::vector<std::string>& command,
              const std::string& work_dir,
              const std::string& config_json,
              double dt,
              const std::string& trace_path,
              const std::string& out_path) {
    const std::vector<TraceRow> trace = ReadTrace(trace_path);

    seastack::external::IpcExternalForceOptions opts;
    opts.command = command;
    opts.working_directory = work_dir;
    opts.timeout_ms = 20000;

    auto ipc = std::make_unique<seastack::external::IpcExternalForceModel>(opts);
    seastack::external::ExternalPtoModel ext(std::move(ipc));
    ext.Initialize(dt, config_json.empty() ? "{}" : config_json);

    std::ostream* out = &std::cout;
    std::ofstream file;
    if (!out_path.empty()) {
        file.open(out_path);
        if (!file) {
            std::cerr << "cannot open output: " << out_path << "\n";
            return 1;
        }
        out = &file;
    }
    *out << "time,force\n";
    (*out).setf(std::ios::scientific);
    out->precision(17);
    for (const TraceRow& row : trace) {
        const double f = ext.ComputeForce(row.disp, row.vel, row.t);
        *out << row.t << ',' << f << '\n';
    }
    ext.Shutdown();
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    std::vector<std::string> command;
    std::string work_dir;
    bool replay = false;
    std::string config_json;
    std::string trace_path;
    std::string out_path;
    double replay_dt = 0.01;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--python" && i + 1 < argc) {
            // Spawn as (python, <filename>) with the script's directory as the
            // working directory, mirroring run_seastack's external_pto command.
            // The child cwd is the script dir, so pass the bare filename.
            const std::filesystem::path script(argv[++i]);
            work_dir = script.parent_path().string();
            command = {FindPython(), script.filename().string()};
        } else if (arg == "--mock" && i + 1 < argc) {
            command = {argv[++i]};
        } else if (arg == "--replay") {
            replay = true;
        } else if (arg == "--config" && i + 1 < argc) {
            config_json = argv[++i];
        } else if (arg == "--dt" && i + 1 < argc) {
            replay_dt = std::stod(argv[++i]);
        } else if (arg == "--trace" && i + 1 < argc) {
            trace_path = argv[++i];
        } else if (arg == "--out" && i + 1 < argc) {
            out_path = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0]
                      << " [--python script.py | --mock mock_external_module]\n"
                      << "       " << argv[0]
                      << " --replay --python script.py [--config JSON]"
                      << " [--dt DT] --trace in.csv [--out out.csv]\n";
            return 0;
        }
    }

    if (replay) {
        if (command.empty() || trace_path.empty()) {
            std::cerr << "--replay requires --python/--mock and --trace\n";
            return 2;
        }
        try {
            return RunReplay(command, work_dir, config_json, replay_dt,
                             trace_path, out_path);
        } catch (const std::exception& e) {
            std::cerr << "replay failed: " << e.what() << "\n";
            return 1;
        }
    }

    constexpr double k = 0.0;
    constexpr double c = 50.0;
    constexpr double disp = 0.1;
    constexpr double vel = 2.0;
    constexpr double t = 1.0;

    seastack::pto::LinearPTO local(k, c);
    const double local_force = local.ComputeForce(disp, vel, t);

    if (command.empty()) {
        std::cout << "No external module requested; local LinearPTO force = "
                  << local_force << " N\n";
        std::cout << "Pass --python linear_damper_pto.py to compare via IPC.\n";
        return 0;
    }

    seastack::external::IpcExternalForceOptions opts;
    opts.command = command;
    opts.working_directory = work_dir;
    opts.timeout_ms = 20000;

    auto ipc = std::make_unique<seastack::external::IpcExternalForceModel>(opts);
    seastack::external::ExternalPtoModel ext(std::move(ipc));
    ext.Initialize(0.01, "{\"damping\":50}");

    const double ext_force = ext.ComputeForce(disp, vel, t);
    // HHT-style re-query at same time must not change the force / call count.
    const double ext_force2 = ext.ComputeForce(disp + 1.0, vel + 1.0, t);
    const int calls = ext.evaluate_call_count();
    ext.Shutdown();

    std::cout << "Local LinearPTO force  = " << local_force << " N\n";
    std::cout << "External module force  = " << ext_force << " N\n";
    std::cout << "Cached re-eval force   = " << ext_force2 << " N\n";
    std::cout << "Backend evaluate calls = " << calls << "\n";

    const double tol = 1e-6;
    if (std::abs(ext_force - local_force) > tol) {
        std::cerr << "FAIL: external force differs from LinearPTO\n";
        return 1;
    }
    if (std::abs(ext_force2 - ext_force) > tol || calls != 1) {
        std::cerr << "FAIL: time-caching did not hold\n";
        return 1;
    }
    std::cout << "PASS\n";
    return 0;
}
