/*********************************************************************
 * @file  model_yaml_postprocess.h
 * @brief SEA-Stack post-Populate fixes for Chrono MBS model YAML.
 *
 * Chrono 10's ChParserMbsYAML does not honour visualization.color /
 * shape_location on triangle meshes, and does not parse collision_family.
 * These helpers apply those fields by body name after Populate().
 *********************************************************************/

#ifndef SEASTACK_APP_MODEL_YAML_POSTPROCESS_H
#define SEASTACK_APP_MODEL_YAML_POSTPROCESS_H

#include <string>

namespace chrono {
class ChSystem;
}

namespace YAML {
class Node;
}

namespace seastack::app {

/// Apply per-body visualization.color and visualization.shape_location from
/// the loaded model YAML.  Paths in visualization.model_file / shapes are
/// resolved relative to model_dir.  Failures are logged at debug level and
/// never throw.
void ApplyBodyVisualizationFromModelYaml(::chrono::ChSystem& system,
                                         const YAML::Node& model_yaml,
                                         const std::string& model_dir);

/// Apply bodies[].collision_family from the loaded model YAML.  Chrono's
/// parser leaves this as an unimplemented TODO; SEA-Stack applies it here so
/// deck / plank / tire families can be expressed without C++.
void ApplyCollisionFamiliesFromModelYaml(::chrono::ChSystem& system,
                                         const YAML::Node& model_yaml);

/// Convenience: run visualization + collision-family post-processing.
void ApplyModelYamlPostProcess(::chrono::ChSystem& system,
                               const YAML::Node& model_yaml,
                               const std::string& model_dir);

}  // namespace seastack::app

#endif  // SEASTACK_APP_MODEL_YAML_POSTPROCESS_H
