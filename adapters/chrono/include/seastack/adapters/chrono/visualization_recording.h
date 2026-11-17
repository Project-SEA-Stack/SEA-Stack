/**
 * @file visualization_recording.h
 * @brief Read a SEA-Stack results file back into a visual scene and frames for
 *        offline replay (run_seastack --replay), with no Chrono system.
 *
 * Reads what SimulationExporter::WriteVisualization and the per-step recorders
 * wrote (HDF5 schema 1.1):
 *   /visualization                         scene: shapes, input pose, environment
 *   /results/time/time                     frame times [s]
 *   /results/model/bodies/<name>           centre-of-mass position and orientation
 *   /results/mooring/line_<i>              MoorDyn node positions and tensions (own sample_time)
 *   /events/waves                          live sea changes (optional)
 *   /visualization/cables/cable_<i>        TSDA glyph endpoints
 *   /visualization/forces                  hydro loads by component (optional)
 *   /inputs/simulation/waves/components    linear wave components (optional)
 *
 * Conventions match viz_core: world frame, Z up, SI units, quaternions (w, x, y, z).
 */

#pragma once

#include <seastack/viz_core/scene_model.h>

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace seastack::hydro {
class WaveBase;
}

namespace seastack::chrono {

struct ReplayRecording {
    viz::SceneDescription scene;

    /// Input pose (model file / initial conditions) at input_frame.time, before
    /// the first integration step. /results starts after the first step (SIM-160).
    viz::VizFrame input_frame;

    /// Recorded frames in time order (every frame_stride-th output sample).
    std::vector<viz::VizFrame> frames;
    int frame_stride = 1;          ///< > 1 when the file was thinned to fit the memory budget
    size_t samples_in_file = 0;    ///< Output samples in /results/time/time

    /// Wave field rebuilt from the stored components; null when the run had no
    /// component wave model (still water, or a tabulated elevation record).
    std::shared_ptr<hydro::WaveBase> wave;
    /// For the user: why wave is null, or why it shows only the initial sea; empty otherwise.
    std::string wave_note;
    /// For the user: why mooring lines are not drawn (e.g. compact output); empty otherwise.
    std::string mooring_note;

    /// True if the shape colours were already painted by a GUI viewer; false if
    /// they are as authored in the model (headless run) and still need the palette.
    bool viewer_colors = false;

    std::string schema_version;  ///< /meta schema_version
};

/// Load a results file for replay.
///
/// @param path          HDF5 results file written by run_seastack.
/// @param budget_bytes  Memory budget for the frames; longer records are thinned
///                      to every frame_stride-th sample.
/// @param out           Filled on success.
/// @param error         Message for the user on failure (e.g. a file written
///                      before schema 1.1, without /visualization).
/// @return true on success.
bool LoadReplayRecording(const std::string& path, size_t budget_bytes, ReplayRecording* out,
                         std::string* error);

}  // namespace seastack::chrono
