/*********************************************************************
 * @file  replay_run.cpp
 * @brief Offline replay of a recorded run (see replay_run.h).
 *********************************************************************/

#include "replay_run.h"

#include "app_init.h"
#include "gui/guihelper.h"

#include <seastack/adapters/chrono/visualization_recording.h>
#include <seastack/infra/logging.h>

#include <chrono>
#include <exception>
#include <filesystem>
#include <memory>
#include <thread>

namespace seastack::app {

int RunReplay(const std::string& results_path) {
    namespace cli = seastack::infra::cli;
    // Frames beyond this are thinned to every n-th output sample.
    constexpr size_t kReplayBudgetBytes = size_t{512} << 20;
    constexpr double kFramePeriod = 1.0 / 60.0;  // [s] the viewer redraws at most at 60 Hz

    if (!std::filesystem::is_regular_file(results_path)) {
        cli::LogError("ERROR: Replay file does not exist: " + results_path);
        return 1;
    }
    InitChronoEnvironment();  // fonts and textures come from the Chrono data folder

    auto recording = std::make_unique<seastack::chrono::ReplayRecording>();
    std::string error;
    if (!seastack::chrono::LoadReplayRecording(results_path, kReplayBudgetBytes, recording.get(), &error)) {
        cli::LogError("ERROR: " + error);
        return 1;
    }
    const auto& frames = recording->frames;
    cli::LogInfo("Replay: " + std::to_string(frames.size()) + " frames, t = " +
                 std::to_string(frames.front().time) + " to " + std::to_string(frames.back().time) +
                 " s. Visual only; nothing is simulated.");
    if (recording->frame_stride > 1) {
        cli::LogWarning("Replay: long record; showing every " + std::to_string(recording->frame_stride) +
                        "th of " + std::to_string(recording->samples_in_file) + " output samples to stay within " +
                        std::to_string(kReplayBudgetBytes >> 20) + " MB.");
    }
    if (!recording->wave_note.empty()) {
        cli::LogWarning("Replay: " + recording->wave_note);
    }
    if (!recording->mooring_note.empty()) {
        cli::LogWarning("Replay: " + recording->mooring_note);
    }

    auto ui = seastack::viz::CreateUI(true, seastack::viz::ViewerBackend::kSeastack);
    try {
        const std::string title = "SEA-Stack replay - " + std::filesystem::path(results_path).filename().string();
        ui->InitReplay(std::move(recording), title.c_str());
    } catch (...) {
        cli::LogError("ERROR: Cannot open the replay viewer: " + seastack::viz::CurrentExceptionMessage());
        return 1;
    }

    auto next_frame = std::chrono::steady_clock::now();
    try {
        while (ui->IsRunning(0.0)) {
            next_frame += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(kFramePeriod));
            const auto now = std::chrono::steady_clock::now();
            if (next_frame > now) {
                std::this_thread::sleep_until(next_frame);
            } else {
                next_frame = now;  // fell behind (slow frame); do not try to catch up
            }
        }
    } catch (...) {
        cli::LogError("ERROR: Replay viewer failed: " + seastack::viz::CurrentExceptionMessage());
        return 1;
    }
    return 0;
}

}  // namespace seastack::app
