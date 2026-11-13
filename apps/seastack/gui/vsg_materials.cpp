// =============================================================================
// SEA-Stack VSG Material Utilities - Implementation
// =============================================================================
#include "vsg_materials.h"

#include "vsg_config.h"

#include <algorithm>
#include <cmath>

#include <chrono/core/ChTypes.h>

namespace seastack::viz {

using namespace vsg_config;

::chrono::ChVisualMaterial MakePaintedMetalBase() {
    ::chrono::ChVisualMaterial mat;

    mat.SetDiffuseColor(::chrono::ChColor(kPaintedMetalR, kPaintedMetalG, kPaintedMetalB));
    mat.SetSpecularColor(::chrono::ChColor(kSpecular, kSpecular, kSpecular));
    mat.SetSpecularExponent(kSpecularExponent);
    mat.SetRoughness(kRoughness);
    mat.SetMetallic(kMetallic);
    mat.SetOpacity(1.0f);

    return mat;
}

::chrono::ChVisualMaterial MakeBodyPaletteMaterial(int index) {
    // Distinct, slightly desaturated industrial colours so neighbouring bodies
    // (e.g. float and spar) are easy to tell apart on dark and light backgrounds.
    static constexpr float kPalette[][3] = {
        {0.92f, 0.72f, 0.12f},  // safety yellow
        {0.60f, 0.63f, 0.68f},  // steel grey
        {0.90f, 0.45f, 0.15f},  // signal orange
        {0.16f, 0.58f, 0.62f},  // teal
        {0.86f, 0.86f, 0.83f},  // off-white
        {0.25f, 0.42f, 0.75f},  // cobalt
        {0.55f, 0.60f, 0.25f},  // olive
        {0.70f, 0.30f, 0.25f},  // brick
    };
    constexpr int kCount = static_cast<int>(sizeof(kPalette) / sizeof(kPalette[0]));
    const auto& rgb = kPalette[((index % kCount) + kCount) % kCount];

    // The palette is authored in sRGB; Chrono passes diffuse colours to a
    // linear-space shader unchanged, so convert or they display washed out.
    auto to_linear = [](float c) {
        return (c <= 0.04045f) ? c / 12.92f : std::pow((c + 0.055f) / 1.055f, 2.4f);
    };
    ::chrono::ChVisualMaterial mat = MakePaintedMetalBase();
    mat.SetDiffuseColor(::chrono::ChColor(to_linear(rgb[0]), to_linear(rgb[1]), to_linear(rgb[2])));
    return mat;
}

bool IsChronoDefaultColor(const ::chrono::ChColor& c) {
    // White is ChVisualMaterial's default; the others are ChBodyGeometry's
    // per-primitive defaults (boxes, spheres, cylinders, cones, meshes).
    static constexpr float kDefaults[][3] = {
        {1.0f, 1.0f, 1.0f},   {0.75f, 0.5f, 0.5f},  {0.5f, 0.75f, 0.5f},
        {0.5f, 0.5f, 0.75f},  {0.75f, 0.75f, 0.5f}, {0.5f, 0.75f, 0.75f},
    };
    for (const auto& d : kDefaults) {
        if (std::abs(c.R - d[0]) < 1e-4f && std::abs(c.G - d[1]) < 1e-4f && std::abs(c.B - d[2]) < 1e-4f) {
            return true;
        }
    }
    return false;
}

::chrono::ChVisualMaterial MakeWaterSurfaceMaterial() {
    ::chrono::ChVisualMaterial mat;

    mat.SetDiffuseColor(::chrono::ChColor(kWaterR, kWaterG, kWaterB));
    mat.SetSpecularColor(::chrono::ChColor(kWaterSpecular, kWaterSpecular, kWaterSpecular));
    mat.SetRoughness(kWaterRoughness);
    mat.SetMetallic(kWaterMetallic);
    mat.SetOpacity(kWaterOpacity);

    return mat;
}

void ApplyMaterialToAllVisualShapes(::chrono::ChBody& body, const ::chrono::ChVisualMaterial& mat,
                                    bool keep_model_colors) {
    auto model = body.GetVisualModel();
    if (!model) {
        return;
    }

    auto mat_ptr = ::chrono_types::make_shared<::chrono::ChVisualMaterial>(mat);

    const unsigned int num_shapes = model->GetNumShapes();
    for (unsigned int i = 0; i < num_shapes; ++i) {
        auto shape = model->GetShape(i);
        if (!shape) {
            continue;
        }

        const unsigned int num_materials = shape->GetNumMaterials();
        if (num_materials == 0) {
            shape->AddMaterial(mat_ptr);
            continue;
        }

        // Translucent shapes come from model YAML opacity < 1. Keep that alpha
        // (and the YAML diffuse colour) but still apply painted-metal PBR fields
        // so Chrono VSG gets a complete material for the blend pipeline.
        float preserved_opacity = 1.0f;
        ::chrono::ChColor preserved_diffuse = mat.GetDiffuseColor();
        bool has_translucent = false;
        for (unsigned int m = 0; m < num_materials; ++m) {
            const auto& existing = shape->GetMaterial(m);
            if (existing && existing->GetOpacity() < 1.0f) {
                has_translucent = true;
                preserved_opacity = existing->GetOpacity();
                preserved_diffuse = existing->GetDiffuseColor();
                break;
            }
        }
        if (has_translucent) {
            auto merged = ::chrono_types::make_shared<::chrono::ChVisualMaterial>(mat);
            merged->SetDiffuseColor(preserved_diffuse);
            merged->SetOpacity(preserved_opacity);
            for (unsigned int m = 0; m < num_materials; ++m) {
                shape->SetMaterial(m, merged);
            }
            continue;
        }

        // Opaque shape with a user-set colour: keep the colour, take the PBR
        // fields from `mat`.
        if (keep_model_colors) {
            const auto& first = shape->GetMaterial(0);
            if (first && !IsChronoDefaultColor(first->GetDiffuseColor())) {
                auto merged = ::chrono_types::make_shared<::chrono::ChVisualMaterial>(mat);
                merged->SetDiffuseColor(first->GetDiffuseColor());
                for (unsigned int m = 0; m < num_materials; ++m) {
                    shape->SetMaterial(m, merged);
                }
                continue;
            }
        }

        for (unsigned int m = 0; m < num_materials; ++m) {
            shape->SetMaterial(m, mat_ptr);
        }
    }
}

}  // namespace seastack::viz

