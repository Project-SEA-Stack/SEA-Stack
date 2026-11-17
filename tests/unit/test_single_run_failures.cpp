/**
 * @file test_single_run_failures.cpp
 * @brief RunSingleCase keeps the results of a run when optional parts fail.
 *
 * Cases (RM3 regular waves demo, short runs):
 *   F1  A viewer error while drawing a frame ends the interactive loop; the
 *       results up to that time are still written and the run reports the
 *       error (non-zero exit code).
 */

#include "single_run.h"
#include "guihelper.h"
#include "test_macros.h"

#include <H5Cpp.h>

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace fs = std::filesystem;

static TestResults test_results;

namespace {

/// Interactive UI with no window: runs unpaced and throws on frame `fail_frame`.
class FailingUI : public seastack::viz::UI {
  public:
    explicit FailingUI(int fail_frame) : fail_frame_(fail_frame) {}
    bool IsRunning(double /*timestep*/) override {
        simulationStarted = true;
        playbackSpeed = 0.0;  // unlimited: no pacing waits
        if (++frames_ >= fail_frame_) {
            throw std::runtime_error("injected viewer failure");
        }
        return true;
    }

  private:
    int fail_frame_;
    int frames_ = 0;
};

seastack::app::SingleRunConfig Rm3Regular(const fs::path& out_dir) {
    const fs::path dir = fs::path(SEASTACK_SOURCE_DIR) / "data/demos/run_seastack/rm3/regular_waves";
    seastack::app::SingleRunConfig cfg;
    cfg.model_file = (dir / "rm3_regular_waves.model.yaml").string();
    cfg.simulation_file = (dir / "rm3_regular_waves.simulation.yaml").string();
    cfg.hydro_source = (dir / "rm3_regular_waves.hydro.yaml").string();
    cfg.output_directory = out_dir.string();
    cfg.concise_cli = true;
    return cfg;
}

/// Number of samples in /results/time/time of the one .h5 in `dir` (-1: none or unreadable).
long long RecordedSamples(const fs::path& dir) {
    for (const auto& entry : fs::directory_iterator(dir)) {
        if (entry.path().extension() != ".h5") continue;
        try {
            H5::Exception::dontPrint();
            H5::H5File file(entry.path().string(), H5F_ACC_RDONLY);
            H5::DataSet ds = file.openDataSet("/results/time/time");
            return static_cast<long long>(ds.getSpace().getSimpleExtentNpoints());
        } catch (const H5::Exception&) {
            return -1;
        }
    }
    return -1;
}

fs::path FreshDir(const std::string& name) {
    const fs::path dir = fs::temp_directory_path() / ("seastack_test_single_run_failures_" + name);
    std::error_code ec;
    fs::remove_all(dir, ec);
    fs::create_directories(dir);
    return dir;
}

}  // namespace

int main() {
    std::cout << "test_single_run_failures\n";

    {  // F1
        const fs::path out = FreshDir("viewer");
        seastack::app::SingleRunConfig cfg = Rm3Regular(out);
        cfg.ui_override = std::make_shared<FailingUI>(4);
        const seastack::app::SingleRunResult r = seastack::app::RunSingleCase(cfg);
        TEST_ASSERT(r.exit_code != 0, "F1: a viewer error is reported (non-zero exit code)");
        TEST_ASSERT(r.error_message.find("viewer error") != std::string::npos,
                    "F1: error message names the viewer: " + r.error_message);
        TEST_ASSERT(r.sim_time_final > 0.0, "F1: the run stepped before the failure");
        const long long n = RecordedSamples(out);
        TEST_ASSERT(n > 0, "F1: results written up to the failure (" + std::to_string(n) + " samples)");
    }

    test_results.Summary();
    return test_results.failed == 0 ? 0 : 1;
}
