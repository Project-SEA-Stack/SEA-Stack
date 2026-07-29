/*********************************************************************
 * @file  sph_run.cpp
 * @brief Chrono::FSI (SPH) single-case runner implementation.
 *
 * Mirrors Chrono's demo_YAML_fsi flow (build a coupled FSI system from YAML,
 * then step it) but adds SEA-Stack conventions: headless operation with a
 * finite end time, structured logging, and rigid-body state export to the same
 * HDF5 layout produced by the potential-flow path (SimulationExporter).
 *********************************************************************/

#include "sph_run.h"

#include <seastack/config.h>
#include <seastack/infra/logging.h>

#include "chrono_parsers/yaml/ChParserFsiYAML.h"

#include "chrono/physics/ChSystem.h"
#include "chrono/ChVersion.h"

#if defined(SEASTACK_HAVE_VSG)
#include "chrono/assets/ChVisualSystem.h"
#include "chrono_vsg/ChVisualSystemVSG.h"
#endif

#include <chrono>
#include <cmath>
#include <filesystem>
#include <memory>
#include <string>

namespace seastack::app {

namespace {

// Best-effort creation of an output directory; returns false on failure.
bool EnsureDirectory(const std::filesystem::path& dir) {
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    return !ec;
}

}  // namespace

SphRunResult RunSphCase(const SphRunConfig& config) {
    // Note: '::chrono' (global) — inside seastack::app, an unqualified
    // 'using namespace chrono' would resolve to seastack::chrono (the adapter).
    using namespace ::chrono;
    SphRunResult result;

    try {
        seastack::infra::debug::LogDebug(std::string("[sph] FSI YAML: ") + config.fsi_file);

        if (!std::filesystem::exists(config.fsi_file)) {
            result.error_message = std::string("FSI YAML file does not exist: ") + config.fsi_file;
            seastack::infra::cli::LogError(result.error_message);
            return result;
        }

        // -----------------------------------------------------------------
        // 1. Build the coupled FSI (MBS + SPH fluid) system from YAML.
        // -----------------------------------------------------------------
        parsers::ChParserFsiYAML parser(config.fsi_file, /*verbose=*/config.debug_mode);
        parser.CreateFsiSystem();

        auto sysFSI = parser.GetFsiSystem();
        auto sysMBS = parser.GetMultibodySystem();
        if (!sysFSI || !sysMBS) {
            result.error_message = "Chrono failed to construct the FSI system from the YAML files.";
            seastack::infra::cli::LogError(result.error_message);
            return result;
        }

        const std::string model_name = parser.GetName();
        const double time_step = parser.GetTimestep();
        const double time_end = parser.GetEndtime();  // -1 => unbounded
        const bool has_finite_end = (time_end > 0.0);

        if (time_step <= 0.0) {
            result.error_message = "FSI YAML specifies a non-positive simulation time_step.";
            seastack::infra::cli::LogError(result.error_message);
            return result;
        }

        // Headless runs need a finite end time to terminate.
        const bool want_render = (!config.nogui) && parser.Render();
        if (!want_render && !has_finite_end) {
            result.error_message =
                "Headless SPH run requires a finite simulation.end_time in the FSI YAML "
                "(found end_time <= 0). Set a positive end_time or run with a GUI.";
            seastack::infra::cli::LogError(result.error_message);
            return result;
        }

        seastack::infra::cli::LogInfo(std::string("[sph] Model: ") + model_name +
                                      "  dt=" + std::to_string(time_step) + " s" +
                                      (has_finite_end ? ("  end=" + std::to_string(time_end) + " s")
                                                      : std::string("  end=unbounded")));

        // -----------------------------------------------------------------
        // 2. Output configuration.
        // -----------------------------------------------------------------
        auto& parserMBS = parser.GetMbsParser();
        auto& parserCFD = parser.GetCfdParser();
        const bool chrono_output_mbs = parserMBS.Output();
        const bool chrono_output_cfd = parserCFD.Output();
        double output_fps = parser.GetOutputFPS();
        if (output_fps <= 0.0) output_fps = 20.0;

        std::filesystem::path out_dir;
        const bool persist = !config.output_directory.empty();
        if (persist) {
            out_dir = std::filesystem::path(config.output_directory);
            if (!EnsureDirectory(out_dir)) {
                seastack::infra::cli::LogWarning(
                    std::string("[sph] Could not create output directory: ") + out_dir.string());
            }
        }

        // Chrono native output (particle data for CFD; body states for MBS).
        if (persist && (chrono_output_mbs || chrono_output_cfd)) {
            std::filesystem::path chrono_out = out_dir / "chrono";
            if (EnsureDirectory(chrono_out)) {
                parser.SetOutputDir(chrono_out.generic_string());
            }
        }

        // -----------------------------------------------------------------
        // 3. SEA-Stack rigid-body HDF5 exporter (same layout as potential flow).
        // -----------------------------------------------------------------
        std::unique_ptr<seastack::chrono::SimulationExporter> exporter;
        if (persist) {
            try {
                std::filesystem::path output_h5 = out_dir / "results.sph.h5";
                seastack::chrono::SimulationExporter::Options exp_opts;
                exp_opts.output_path = output_h5.generic_string();
                exp_opts.input_model_file = config.fsi_file;
                exp_opts.output_directory = out_dir.generic_string();
                exp_opts.scenario_type = "sph";
                exp_opts.export_config = config.export_config;
                if (config.cli_output_level == "compact")
                    exp_opts.export_config.level = seastack::chrono::ExportLevel::kCompact;
                else if (config.cli_output_level == "detailed")
                    exp_opts.export_config.level = seastack::chrono::ExportLevel::kDetailed;
                else if (config.cli_output_level == "standard")
                    exp_opts.export_config.level = seastack::chrono::ExportLevel::kStandard;
                exp_opts.log_final_output_path = false;

                exporter = std::make_unique<seastack::chrono::SimulationExporter>(exp_opts);
                exporter->WriteSimulationInfo(sysMBS.get(), std::string(CHRONO_VERSION), model_name,
                                              time_step, has_finite_end ? time_end : 0.0);
                exporter->WriteModel(sysMBS.get());
                int est_steps = 0;
                if (has_finite_end) est_steps = static_cast<int>(time_end * output_fps) + 1;
                exporter->BeginResults(sysMBS.get(), est_steps);
                result.primary_artifact_path = output_h5.generic_string();
            } catch (const std::exception& e) {
                seastack::infra::cli::LogWarning(std::string("[sph] HDF5 exporter disabled: ") + e.what());
                exporter.reset();
                result.primary_artifact_path.clear();
            }
        } else {
            result.artifact_note =
                "No persistent HDF5 (add output_directory to the setup YAML to save results).";
        }

        // -----------------------------------------------------------------
        // 4. Optional run-time visualization.
        // -----------------------------------------------------------------
#if defined(SEASTACK_HAVE_VSG)
        std::shared_ptr<ChVisualSystem> vis;
        if (want_render) {
            auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
            visVSG->AttachSystem(sysMBS.get());
            visVSG->SetWindowTitle("SEA-Stack SPH - " + model_name);
            visVSG->AddCamera(parser.GetCameraLocation(), parser.GetCameraTarget());
            visVSG->SetWindowSize(1280, 800);
            visVSG->SetWindowPosition(100, 100);
            visVSG->SetBackgroundColor(ChColor(0.04f, 0.11f, 0.18f));
            visVSG->SetCameraVertical(parser.GetCameraVerticalDir());
            visVSG->SetCameraAngleDeg(40.0);
            visVSG->SetLightIntensity(1.0f);
            visVSG->SetLightDirection(-CH_PI_4, CH_PI_4);
            visVSG->EnableShadows(parser.EnableShadows());
            auto plugin = parserCFD.GetVisualizationPlugin();
            if (plugin)
                visVSG->AttachPlugin(plugin);
            visVSG->Initialize();
            vis = visVSG;
        }
#else
        const bool vis = false;
        if (want_render) {
            seastack::infra::cli::LogWarning(
                "[sph] GUI requested but SEA-Stack was built without VSG; running headless.");
        }
#endif

        // -----------------------------------------------------------------
        // 5. Co-simulation time loop.
        // -----------------------------------------------------------------
        const double render_fps = parser.GetRenderFPS() > 0.0 ? parser.GetRenderFPS() : 60.0;
        double time = 0.0;
        long step_count = 0;
        int render_frame = 0;
        int record_frame = 0;
        int out_frame_mbs = 0;
        int out_frame_cfd = 0;
        double next_progress = 0.0;

        const auto wall_start = std::chrono::steady_clock::now();

        while (true) {
#if defined(SEASTACK_HAVE_VSG)
            if (want_render) {
                if (!vis->Run())
                    break;
                if (time >= render_frame / render_fps) {
                    vis->BeginScene();
                    vis->Render();
                    vis->EndScene();
                    render_frame++;
                }
            }
#endif
            if (has_finite_end && time >= time_end)
                break;

            // SEA-Stack rigid-body state (throttled to output_fps).
            if (exporter && time >= record_frame / output_fps) {
                exporter->RecordStep(sysMBS.get());
                record_frame++;
            }
            // Chrono native output.
            if (chrono_output_mbs && time >= out_frame_mbs / output_fps) {
                parserMBS.SaveOutput(*sysMBS, out_frame_mbs);
                out_frame_mbs++;
            }
            if (chrono_output_cfd && time >= out_frame_cfd / output_fps) {
                parserCFD.SaveOutput(out_frame_cfd);
                out_frame_cfd++;
            }

            sysFSI->DoStepDynamics(time_step);
            time += time_step;
            step_count++;

            if (!want_render && has_finite_end && time >= next_progress) {
                double pct = 100.0 * time / time_end;
                seastack::infra::cli::LogInfo("[sph] t = " + std::to_string(time) + " s (" +
                                              std::to_string(static_cast<int>(pct)) + "%)");
                next_progress += std::max(time_end / 20.0, time_step);
            }
        }

        const auto wall_end = std::chrono::steady_clock::now();
        result.wall_time_s =
            std::chrono::duration<double>(wall_end - wall_start).count();
        result.sim_time_final = time;

        // -----------------------------------------------------------------
        // 6. Finalize HDF5 output.
        // -----------------------------------------------------------------
        if (exporter) {
            try {
                exporter->SetRunMetadata(std::string(""), std::string(""), result.wall_time_s,
                                         static_cast<int>(step_count), time_step, time);
                exporter->Finalize();
            } catch (const std::exception& e) {
                seastack::infra::cli::LogWarning(std::string("[sph] HDF5 finalize failed: ") + e.what());
                result.primary_artifact_path.clear();
            }
        }

        result.exit_code = 0;
        return result;

    } catch (const std::exception& e) {
        result.error_message = std::string("SPH run failed: ") + e.what();
        seastack::infra::cli::LogError(result.error_message);
        return result;
    } catch (...) {
        result.error_message = "SPH run failed: unknown fatal error.";
        seastack::infra::cli::LogError(result.error_message);
        return result;
    }
}

}  // namespace seastack::app
