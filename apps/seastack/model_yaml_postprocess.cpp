/*********************************************************************
 * @file  model_yaml_postprocess.cpp
 * @brief Implementation of SEA-Stack model-YAML post-Populate helpers.
 *********************************************************************/

#include "model_yaml_postprocess.h"

#include <seastack/infra/logging.h>

#include <chrono/assets/ChVisualMaterial.h>
#include <chrono/assets/ChVisualModel.h>
#include <chrono/assets/ChVisualShapeModelFile.h>
#include <chrono/assets/ChVisualShapeTriangleMesh.h>
#include <chrono/geometry/ChTriangleMeshConnected.h>
#include <chrono/physics/ChBody.h>
#include <chrono/physics/ChSystem.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>

namespace seastack::app {

void ApplyBodyVisualizationFromModelYaml(::chrono::ChSystem& system,
                                         const YAML::Node& model_yaml,
                                         const std::string& model_dir) {
    try {
        auto model_node = model_yaml["model"];
        if (!model_node || !model_node["bodies"]) {
            return;
        }

        std::unordered_map<std::string, YAML::Node> vis_by_name;
        for (const auto& entry : model_node["bodies"]) {
            if (!entry["name"]) {
                continue;
            }
            const std::string nm = entry["name"].as<std::string>();
            vis_by_name.emplace(nm, entry["visualization"]);
        }

        const std::filesystem::path model_dir_path(model_dir);
        auto resolve_mesh_path = [&](const std::string& path_str) -> std::string {
            std::filesystem::path p(path_str);
            if (p.is_relative()) {
                p = model_dir_path / p;
            }
            std::error_code ec;
            std::filesystem::path c = std::filesystem::weakly_canonical(p, ec);
            if (!ec) {
                return c.generic_string();
            }
            return p.lexically_normal().generic_string();
        };

        for (const auto& body : system.GetBodies()) {
            if (!body) {
                continue;
            }
            auto vit = vis_by_name.find(body->GetName());
            if (vit == vis_by_name.end()) {
                continue;
            }
            const YAML::Node& vis_node = vit->second;
            if (!vis_node) {
                continue;
            }
            const bool has_color = static_cast<bool>(vis_node["color"]) &&
                                   vis_node["color"].IsSequence() && vis_node["color"].size() >= 3;
            const bool has_shape_location =
                static_cast<bool>(vis_node["shape_location"]) &&
                vis_node["shape_location"].IsSequence() && vis_node["shape_location"].size() >= 3;
            // Optional scalar opacity in [0,1]. Values < 1 make the mesh translucent
            // (e.g. a see-through hull revealing internal bodies). The VSG scene-paint
            // pass preserves bodies whose material opacity is < 1 (see guihelperVSG.cpp).
            const bool has_opacity = static_cast<bool>(vis_node["opacity"]);
            float opacity = 1.f;
            if (has_opacity) {
                opacity = vis_node["opacity"].as<float>();
                opacity = std::clamp(opacity, 0.f, 1.f);
            }
            if (!has_color && !has_shape_location && !has_opacity) {
                continue;
            }
            float r = 1.f;
            float g = 1.f;
            float b = 1.f;
            if (has_color) {
                auto c = vis_node["color"];
                r = c[0].as<float>();
                g = c[1].as<float>();
                b = c[2].as<float>();
            }
            ::chrono::ChVector3d shape_pos(0, 0, 0);
            if (has_shape_location) {
                auto sl = vis_node["shape_location"];
                shape_pos.x() = sl[0].as<double>();
                shape_pos.y() = sl[1].as<double>();
                shape_pos.z() = sl[2].as<double>();
            }
            auto vis = body->GetVisualModel();
            if (!vis) {
                continue;
            }
            // Apply YAML colour and/or opacity to a rebuilt mesh shape. When an opacity
            // is given we attach an explicit material so the renderer alpha-blends it
            // (a plain SetColor leaves opacity at 1).
            const auto apply_appearance =
                [&](const std::shared_ptr<::chrono::ChVisualShapeTriangleMesh>& tri) {
                    if (has_opacity) {
                        auto mat = ::chrono_types::make_shared<::chrono::ChVisualMaterial>();
                        mat->SetDiffuseColor(has_color ? ::chrono::ChColor(r, g, b)
                                                       : ::chrono::ChColor(0.7f, 0.7f, 0.75f));
                        mat->SetOpacity(opacity);
                        tri->GetMaterials().clear();
                        tri->AddMaterial(mat);
                    } else if (has_color) {
                        tri->SetColor(::chrono::ChColor(r, g, b));
                    }
                };
            for (unsigned si = 0; si < vis->GetNumShapes(); ++si) {
                auto shape = body->GetVisualShape(si);
                auto existing_tri =
                    std::dynamic_pointer_cast<::chrono::ChVisualShapeTriangleMesh>(shape);
                if (existing_tri && existing_tri->GetMesh()) {
                    auto mesh = existing_tri->GetMesh();
                    auto tri_shape =
                        ::chrono_types::make_shared<::chrono::ChVisualShapeTriangleMesh>(mesh, false);
                    if (has_color || has_opacity) {
                        apply_appearance(tri_shape);
                    } else {
                        for (unsigned mi = 0; mi < existing_tri->GetNumMaterials(); ++mi) {
                            tri_shape->SetMaterial(mi, existing_tri->GetMaterial(mi));
                        }
                    }
                    body->GetVisualModel()->Clear();
                    body->AddVisualShape(tri_shape, ::chrono::ChFrame<>(shape_pos));
                    break;
                }
                auto mf = std::dynamic_pointer_cast<::chrono::ChVisualShapeModelFile>(shape);
                if (!mf) {
                    continue;
                }
                std::string obj_path = resolve_mesh_path(mf->GetFilename());
                auto trimesh =
                    ::chrono::ChTriangleMeshConnected::CreateFromWavefrontFile(obj_path, true, false);
                if (!trimesh) {
                    seastack::infra::debug::LogDebug(
                        std::string("CreateFromWavefrontFile failed for body ") + body->GetName() +
                        ": " + obj_path);
                    continue;
                }
                auto tri_shape =
                    ::chrono_types::make_shared<::chrono::ChVisualShapeTriangleMesh>(trimesh, false);
                apply_appearance(tri_shape);
                body->GetVisualModel()->Clear();
                body->AddVisualShape(tri_shape, ::chrono::ChFrame<>(shape_pos));
                break;
            }
        }
    } catch (const std::exception& e) {
        seastack::infra::debug::LogDebug(std::string("Body visualization YAML application failed: ") +
                                         e.what());
    } catch (...) {
        seastack::infra::debug::LogDebug("Body visualization YAML application failed (unknown error)");
    }
}

void ApplyCollisionFamiliesFromModelYaml(::chrono::ChSystem& system, const YAML::Node& model_yaml) {
    try {
        auto model_node = model_yaml["model"];
        if (!model_node || !model_node["bodies"]) {
            return;
        }

        std::unordered_map<std::string, int> family_by_name;
        for (const auto& entry : model_node["bodies"]) {
            if (!entry["name"]) {
                continue;
            }
            // Accept either bodies[].collision_family or bodies[].contact.collision_family
            // (Chrono's own example YAML uses the nested form; both are unparsed upstream).
            int family = -1;
            bool found = false;
            if (entry["collision_family"]) {
                family = entry["collision_family"].as<int>();
                found = true;
            } else if (entry["contact"] && entry["contact"]["collision_family"]) {
                family = entry["contact"]["collision_family"].as<int>();
                found = true;
            }
            if (!found) {
                continue;
            }
            family_by_name.emplace(entry["name"].as<std::string>(), family);
        }
        if (family_by_name.empty()) {
            return;
        }

        for (const auto& body : system.GetBodies()) {
            if (!body) {
                continue;
            }
            auto it = family_by_name.find(body->GetName());
            if (it == family_by_name.end()) {
                continue;
            }
            // Chrono collision families are 0..15.  Out-of-range values are
            // ignored with a warning rather than clamping silently.
            if (it->second < 0 || it->second > 15) {
                seastack::infra::cli::LogWarning(
                    "Body '" + body->GetName() + "' collision_family=" +
                    std::to_string(it->second) + " is outside Chrono's 0..15 range; ignored.");
                continue;
            }
            auto cmodel = body->GetCollisionModel();
            if (!cmodel) {
                seastack::infra::cli::LogWarning(
                    "Body '" + body->GetName() +
                    "' has collision_family but no collision model; ignored.");
                continue;
            }
            cmodel->SetFamily(it->second);
            // Bodies that share a family typically should not self-collide
            // (deck planks, paired hull decks).
            cmodel->DisallowCollisionsWith(it->second);
        }
    } catch (const std::exception& e) {
        seastack::infra::debug::LogDebug(std::string("Collision family YAML application failed: ") +
                                         e.what());
    } catch (...) {
        seastack::infra::debug::LogDebug("Collision family YAML application failed (unknown error)");
    }
}

void ApplyModelYamlPostProcess(::chrono::ChSystem& system,
                               const YAML::Node& model_yaml,
                               const std::string& model_dir) {
    ApplyBodyVisualizationFromModelYaml(system, model_yaml, model_dir);
    ApplyCollisionFamiliesFromModelYaml(system, model_yaml);
}

}  // namespace seastack::app
