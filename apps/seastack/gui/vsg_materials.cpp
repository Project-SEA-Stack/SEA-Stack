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

namespace {

/// sRGB colour (as authored in YAML or a colour picker) to linear. Chrono
/// passes diffuse colours to a linear-space shader unchanged, so unconverted
/// colours display washed out.
::chrono::ChColor SrgbToLinear(const ::chrono::ChColor& c) {
    auto to_linear = [](float v) {
        return (v <= 0.04045f) ? v / 12.92f : std::pow((v + 0.055f) / 1.055f, 2.4f);
    };
    return ::chrono::ChColor(to_linear(c.R), to_linear(c.G), to_linear(c.B));
}

}  // namespace

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
    // Marine high-visibility paints alternating with neutral structural
    // greys, so adjacent bodies (e.g. float and spar) stay distinct and read
    // like real offshore hardware on dark and light backgrounds.
    static constexpr float kPalette[][3] = {
        {0.95f, 0.75f, 0.10f},  // safety yellow
        {0.33f, 0.36f, 0.40f},  // gunmetal
        {0.80f, 0.16f, 0.12f},  // signal red
        {0.60f, 0.64f, 0.69f},  // steel grey
        {0.95f, 0.42f, 0.10f},  // international orange
        {0.20f, 0.24f, 0.30f},  // dark slate
        {0.86f, 0.87f, 0.88f},  // satin white
        {0.17f, 0.22f, 0.33f},  // navy
    };
    constexpr int kCount = static_cast<int>(sizeof(kPalette) / sizeof(kPalette[0]));
    const auto& rgb = kPalette[((index % kCount) + kCount) % kCount];

    ::chrono::ChVisualMaterial mat = MakePaintedMetalBase();
    mat.SetDiffuseColor(SrgbToLinear(::chrono::ChColor(rgb[0], rgb[1], rgb[2])));
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
            merged->SetDiffuseColor(SrgbToLinear(preserved_diffuse));
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
                merged->SetDiffuseColor(SrgbToLinear(first->GetDiffuseColor()));
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

void ApplyBodyPalette(int body_index, bool keep_model_colors, VisualShape* shape) {
    const ::chrono::ChVisualMaterial mat = MakeBodyPaletteMaterial(body_index);
    const ::chrono::ChColor authored(shape->color[0], shape->color[1], shape->color[2]);
    const float opacity = shape->color[3];
    // Same rules as ApplyMaterialToAllVisualShapes: translucent shapes and
    // (optionally) user-set colours keep their colour, the rest take the palette.
    ::chrono::ChColor diffuse = mat.GetDiffuseColor();
    if (opacity < 1.0f || (keep_model_colors && !IsChronoDefaultColor(authored))) {
        diffuse = SrgbToLinear(authored);
    }
    shape->color = {diffuse.R, diffuse.G, diffuse.B, opacity < 1.0f ? opacity : 1.0f};
    shape->roughness = mat.GetRoughness();
    shape->metallic = mat.GetMetallic();
}

}  // namespace seastack::viz

