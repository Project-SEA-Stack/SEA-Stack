/**
 * @file test_structure_run.cpp
 * @brief End-to-end structure_file runs through RunSingleCase.
 *
 * Uses the FEA cantilever demo (data/demos/run_seastack/fea/cantilever).
 *   R1  The run succeeds, the load-path check passes, and the HDF5 file has
 *       /inputs/structure/<beam> and /results/structure/<beam> datasets whose
 *       frame count matches /results/time/time.
 *   R2  A check that cannot be evaluated (check time after the end) fails
 *       the run instead of passing silently.
 *   R3  A structure check without a structure_file fails the run.
 *   R4  A variational-inequality solver (PSOR) is rejected for FEA.
 */

#include "app_init.h"
#include "setup_checks.h"
#include "single_run.h"
#include "test_macros.h"

#include <H5Cpp.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

namespace fs = std::filesystem;

static TestResults test_results;

namespace {

const fs::path kCase = fs::path(SEASTACK_SOURCE_DIR) / "data/demos/run_seastack/fea/cantilever";

fs::path FreshDir(const std::string& name) {
    const fs::path dir = fs::temp_directory_path() / ("seastack_test_structure_run_" + name);
    std::error_code ec;
    fs::remove_all(dir, ec);
    fs::create_directories(dir);
    return dir;
}

seastack::app::SingleRunConfig Cantilever(const fs::path& out_dir) {
    seastack::app::SingleRunConfig cfg;
    cfg.model_file = (kCase / "cantilever.model.yaml").string();
    cfg.simulation_file = (kCase / "cantilever.simulation.yaml").string();
    cfg.structure_file = (kCase / "cantilever.structure.yaml").string();
    cfg.hydro_source = std::string{};
    cfg.output_directory = out_dir.string();
    cfg.concise_cli = true;
    seastack::app::CheckConfig check;
    check.type = seastack::app::kStructureStaticLoadPathCheck;
    check.time_s = 4.0;
    check.tolerance_percent = 2.0;
    cfg.checks.push_back(check);
    return cfg;
}

fs::path FindH5(const fs::path& dir) {
    for (const auto& entry : fs::directory_iterator(dir)) {
        if (entry.path().extension() == ".h5") return entry.path();
    }
    return {};
}

std::vector<hsize_t> Dims(H5::H5File& file, const std::string& path) {
    try {
        H5::DataSet ds = file.openDataSet(path);
        H5::DataSpace space = ds.getSpace();
        std::vector<hsize_t> dims(static_cast<std::size_t>(space.getSimpleExtentNdims()));
        space.getSimpleExtentDims(dims.data());
        return dims;
    } catch (const H5::Exception&) {
        return {};
    }
}

std::string DimsText(const std::vector<hsize_t>& d) {
    std::string s = "[";
    for (std::size_t i = 0; i < d.size(); ++i) s += (i ? ", " : "") + std::to_string(d[i]);
    return s + "]";
}

}  // namespace

int main() {
    std::cout << "test_structure_run\n";
    seastack::app::InitChronoEnvironment();
    H5::Exception::dontPrint();

    {  // R1
        const fs::path out = FreshDir("ok");
        const auto r = seastack::app::RunSingleCase(Cantilever(out));
        TEST_ASSERT(r.exit_code == 0, "R1: run and load-path check pass: " + r.error_message);
        const fs::path h5 = FindH5(out);
        TEST_ASSERT(!h5.empty(), "R1: HDF5 written");
        if (!h5.empty()) {
            H5::H5File file(h5.string(), H5F_ACC_RDONLY);
            const auto time = Dims(file, "/results/time/time");
            TEST_ASSERT(time.size() == 1 && time[0] > 10, "R1: time series recorded");
            const hsize_t nf = time.empty() ? 0 : time[0];
            const std::string in = "/inputs/structure/connector/";
            const std::string res = "/results/structure/connector/";
            const std::vector<hsize_t> conn = Dims(file, in + "connectivity");
            const std::vector<hsize_t> und = Dims(file, in + "undeformed_node_positions");
            const std::vector<hsize_t> pos = Dims(file, res + "node_positions");
            const std::vector<hsize_t> rot = Dims(file, res + "node_orientations");
            const std::vector<hsize_t> frc = Dims(file, res + "element_forces");
            TEST_ASSERT(conn == std::vector<hsize_t>({6, 2}), "R1: connectivity [6, 2]: " + DimsText(conn));
            TEST_ASSERT(und == std::vector<hsize_t>({7, 3}),
                        "R1: undeformed_node_positions [7, 3]: " + DimsText(und));
            TEST_ASSERT(pos == std::vector<hsize_t>({nf, 7, 3}),
                        "R1: node_positions [frames, 7, 3]: " + DimsText(pos));
            TEST_ASSERT(rot == std::vector<hsize_t>({nf, 7, 4}),
                        "R1: node_orientations [frames, 7, 4]: " + DimsText(rot));
            TEST_ASSERT(frc == std::vector<hsize_t>({nf, 6, 6}),
                        "R1: element_forces [frames, 6, 6]: " + DimsText(frc));
        }
        TEST_ASSERT(fs::exists(out / "structure_diagnostics.csv"),
                    "R1: diagnostics CSV written to the output directory");
    }

    {  // R2
        const fs::path out = FreshDir("late_check");
        auto cfg = Cantilever(out);
        cfg.checks[0].time_s = 100.0;
        const auto r = seastack::app::RunSingleCase(cfg);
        TEST_ASSERT(r.exit_code != 0, "R2: a check that never ran fails the run");
        TEST_ASSERT(r.error_message.find("check") != std::string::npos,
                    "R2: error names the check: " + r.error_message);
    }

    {  // R3
        const fs::path out = FreshDir("no_structure");
        auto cfg = Cantilever(out);
        cfg.structure_file.clear();
        const auto r = seastack::app::RunSingleCase(cfg);
        TEST_ASSERT(r.exit_code != 0, "R3: structure check without structure_file fails");
        TEST_ASSERT(r.error_message.find("no structure_file") != std::string::npos,
                    "R3: error explains why: " + r.error_message);
    }

    {  // R4
        const fs::path out = FreshDir("psor");
        std::ifstream in(kCase / "cantilever.simulation.yaml");
        std::string sim((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        const auto pos = sim.find("SPARSE_LU");
        TEST_ASSERT(pos != std::string::npos, "R4: template has SPARSE_LU");
        if (pos != std::string::npos) {
            sim.replace(pos, std::string("SPARSE_LU").size(), "PSOR");
            const fs::path sim_path = out / "psor.simulation.yaml";
            std::ofstream(sim_path) << sim;
            auto cfg = Cantilever(out);
            cfg.simulation_file = sim_path.string();
            const auto r = seastack::app::RunSingleCase(cfg);
            TEST_ASSERT(r.exit_code != 0, "R4: PSOR is rejected for FEA");
            TEST_ASSERT(r.error_message.find("SPARSE_LU") != std::string::npos,
                        "R4: error suggests a direct solver: " + r.error_message);
        }
    }

    test_results.Summary();
    return test_results.failed == 0 ? 0 : 1;
}
