// =============================================================================
// SEA-Stack VSG Water Shader - Implementation
// =============================================================================
// Pipeline built by hand as in vsg_skybox.cpp; GLSL is compiled at run time
// by VSG's shader compiler. All colours are linear (the swapchain is sRGB).
// =============================================================================

#include "vsg_water_shader.h"

#include <seastack/viz_core/water_colormap.h>

#include <algorithm>
#include <sstream>
#include <string>

namespace seastack::viz {

namespace {

// Uniform block layout (vec4 each), shared by the water and the sky dome.
enum Param : uint32_t {
    kEye = 0,           // xyz: camera position [m, world]
    kSunDir = 1,        // xyz: unit vector towards the key light; w: sun haze strength in the sky
    kSunColor = 2,      // rgb: key light colour x intensity
    kSkyZenith = 3,     // rgb
    kSkyHorizon = 4,    // rgb
    kPatch = 5,         // xy: patch centre [m]; zw: half extents [m]
    kWaterDeep = 6,     // rgb: water body colour in troughs and seen straight down
    kWaves = 7,         // x: GPU component count; y: elevation scale for crest shading [m];
                        // z: grid cells per drawn grid line (0 = no lines)
    kWaterScatter = 8,  // rgb: water body colour on crests (light scattered in thin water)
    kSkyGround = 9,     // rgb: below the horizon (distant sea and ground)
    kStyle = 10,        // x: 0 shaded, 1 elevation; y: legend range [m]; z: contour interval [m] (0 = none)
    kColormap = 11,     // rgb, kColormapStops slots: elevation colour map stops (sRGB), t = 0 .. 1
    kParamCount = 16,
};

static_assert(static_cast<int>(kColormap) + kColormapStops <= static_cast<int>(kParamCount),
              "colour map stops overflow the parameter block");

const char* kParamBlock = R"(
layout(set = 0, binding = 0) uniform WaterParams { vec4 p[16]; } u;
)";

// Sky radiance seen along unit direction `d` (world, z up): a gradient from
// the horizon to the zenith above, the ground colour below with a narrow
// haze band at the horizon, and a glow around the sun. The sky dome draws
// it and the water reflects it, so reflections match the backdrop.
const char* kSkyFunction = R"(
vec3 SkyColor(vec3 d) {
    vec3 zenith = u.p[3].rgb;
    vec3 horizon = u.p[4].rgb;
    vec3 ground = u.p[9].rgb;
    vec3 c;
    if (d.z >= 0.0) {
        c = mix(horizon, zenith, pow(d.z, 0.5));
    } else {
        c = mix(horizon, ground, smoothstep(0.0, 0.06, -d.z));
    }
    float towards_sun = max(dot(d, normalize(u.p[1].xyz)), 0.0);
    c += u.p[1].w * u.p[2].rgb * pow(towards_sun, 8.0) * smoothstep(-0.05, 0.05, d.z);
    return c;
}
)";

const std::string kVersion = "#version 450\n#extension GL_ARB_separate_shader_objects : enable\n";

const char* kWaterVert = R"(
layout(push_constant) uniform PushConstants {
    mat4 projection;
    mat4 modelView;
} pc;

layout(location = 0) in vec3 vsg_Vertex;
layout(location = 1) in vec3 vsg_Normal;
layout(location = 2) in vec2 vsg_TexCoord0;
layout(location = 3) in vec4 vsg_Color;

layout(location = 0) out vec3 world_pos;
layout(location = 1) out vec3 world_normal;
layout(location = 3) out vec2 grid_coord;

out gl_PerVertex { vec4 gl_Position; };

void main() {
    // The water mesh is placed directly in world coordinates (no transform).
    world_pos = vsg_Vertex;
    world_normal = vsg_Normal;
    grid_coord = vsg_TexCoord0;  // grid column and row
    gl_Position = pc.projection * pc.modelView * vec4(vsg_Vertex, 1.0);
}
)";

// GPU grid: a flat grid at z = 0 displaced to
//   eta(x, y) = sum_i a_i cos(kx_i x + ky_i y + psi_i),
// with the components prepared on the CPU (water_display_components.h). The
// normal is analytic, from the gradient of eta.
const char* kWaterGpuVert = R"(
layout(push_constant) uniform PushConstants {
    mat4 projection;
    mat4 modelView;
} pc;

// One vec4 per component: (a [m], kx [rad/m], ky [rad/m], psi [rad]).
layout(std430, set = 0, binding = 1) readonly buffer WaveComponents { vec4 c[]; } waves;

layout(location = 0) in vec3 vsg_Vertex;     // flat grid point (x, y, 0) [m, world]
layout(location = 1) in vec2 vsg_TexCoord0;  // grid column and row

layout(location = 0) out vec3 world_pos;
layout(location = 1) out vec3 world_normal;
layout(location = 3) out vec2 grid_coord;

out gl_PerVertex { vec4 gl_Position; };

void main() {
    int count = int(u.p[7].x);
    float x = vsg_Vertex.x;
    float y = vsg_Vertex.y;
    float eta = 0.0;
    float deta_dx = 0.0;
    float deta_dy = 0.0;
    for (int i = 0; i < count; ++i) {
        vec4 c = waves.c[i];
        float arg = c.y * x + c.z * y + c.w;
        float a_sin = c.x * sin(arg);
        eta += c.x * cos(arg);
        deta_dx -= a_sin * c.y;
        deta_dy -= a_sin * c.z;
    }
    world_pos = vec3(x, y, eta);
    world_normal = normalize(vec3(-deta_dx, -deta_dy, 1.0));
    grid_coord = vsg_TexCoord0;
    gl_Position = pc.projection * pc.modelView * vec4(world_pos, 1.0);
}
)";

const char* kWaterFrag = R"(
layout(location = 0) in vec3 world_pos;
layout(location = 1) in vec3 world_normal;
layout(location = 3) in vec2 grid_coord;
layout(location = 0) out vec4 out_color;

const float kF0 = 0.02;            // normal-incidence reflectance of water (n = 1.33)
const float kAmbient = 0.6;        // share of the body colour lit without the sun
const float kCrestScatter = 0.8;   // how far crests move from the deep to the scatter colour
const float kTroughDarken = 0.25;  // troughs this much darker than the deep colour
const float kBodyOpacity = 0.72;   // seen straight down; submerged hulls still show
const float kMaxOpacity = 0.95;    // at grazing angles
// Sun glint: GGX microfacet specular. The roughness stands in for the ripples
// far smaller than the grid; GGX alpha^2 / 2 is about the slope variance,
// ~0.03 for a moderate wind (Cox & Munk, 1954). Without it, long-crested
// waves light up along whole crests where the slope mirrors the sun.
const float kGlintAlpha = 0.25;
const float kGlintOpacity = 0.3;   // share of the glint added to opacity
const float kPi = 3.14159265;
const float kEdgeFade = 0.08;      // fade over this fraction of the half extent
const float kGridLineOpacity = 0.35;
const float kGridLineDarken = 0.5;  // lines are the surface colour this much darker
// Visual choices, not physics: the reflected sky is dimmed, and the Fresnel
// term uses a normal part-way to vertical (as averaging over unresolved
// ripples would), so a bright sky does not wash out the water and slope
// changes do not streak the crests.
const float kSkyReflection = 0.6;
const float kFresnelFlatten = 0.4;
const float kSciAmbient = 0.7;   // elevation style: share of the colour lit without the sun
const float kSciOpacity = 0.85;  // elevation style: same from every angle
const float kContourDarken = 0.4;  // contour lines are the surface colour this much darker

void main() {
    vec3 eye = u.p[0].xyz;
    vec3 to_sun = normalize(u.p[1].xyz);
    vec3 sun_color = u.p[2].rgb;
    vec4 patch_rect = u.p[5];
    vec3 deep = u.p[6].rgb;
    vec3 scatter = u.p[8].rgb;

    vec3 n = normalize(world_normal);
    vec3 v = normalize(eye - world_pos);
    if (dot(n, v) < 0.0) n = -n;  // seen from below the surface
    float cos_v = clamp(dot(n, v), 0.0, 1.0);

    // Schlick's approximation to the Fresnel reflectance.
    vec3 n_fresnel = normalize(mix(n, vec3(0.0, 0.0, sign(n.z)), kFresnelFlatten));
    float cos_f = clamp(dot(n_fresnel, v), 0.0, 1.0);
    float fresnel = kF0 + (1.0 - kF0) * pow(1.0 - cos_f, 5.0);

    // Reflected sky (the same sky the dome draws).
    vec3 sky = kSkyReflection * SkyColor(reflect(-v, n));

    // Water body: deep colour in troughs, towards the scatter colour on
    // crests, by elevation over the sea's elevation scale; lit softly.
    float s = clamp(world_pos.z / max(u.p[7].y, 0.025), -1.0, 1.0);
    vec3 body = mix(deep, scatter, kCrestScatter * smoothstep(0.0, 1.0, s));
    body *= 1.0 - kTroughDarken * max(-s, 0.0);
    float diffuse = max(dot(n, to_sun), 0.0);
    body *= kAmbient + (1.0 - kAmbient) * diffuse;

    // Reflected sun: F D G / (4 n.v), the n.l of the BRDF cancelling the
    // irradiance's. Fresnel at the half vector, Smith-Schlick shadowing.
    vec3 h = normalize(to_sun + v);
    float n_h = max(dot(n, h), 0.0);
    float n_l = max(dot(n, to_sun), 0.0);
    float n_v = max(cos_v, 1e-3);
    float a2 = kGlintAlpha * kGlintAlpha;
    float d_denom = n_h * n_h * (a2 - 1.0) + 1.0;
    float d = a2 / (kPi * d_denom * d_denom);
    float k = 0.5 * kGlintAlpha;
    float g = (n_l / (n_l * (1.0 - k) + k)) * (n_v / (n_v * (1.0 - k) + k));
    float f_h = kF0 + (1.0 - kF0) * pow(1.0 - max(dot(h, v), 0.0), 5.0);
    float glint = f_h * d * g / (4.0 * n_v);

    vec3 rgb = mix(body, sky, fresnel) + sun_color * glint;

    // More opaque at grazing angles, where the surface mostly reflects.
    float alpha = mix(kBodyOpacity, kMaxOpacity, fresnel);
    alpha = clamp(alpha + kGlintOpacity * glint, 0.0, 1.0);

    // Elevation style: colour by eta over the legend range, contour lines,
    // soft shading for shape, constant opacity (no Fresnel glare or glint).
    if (u.p[10].x > 0.5) {
        float range = max(u.p[10].y, 1e-6);
        rgb = ElevationColor(0.5 + 0.5 * world_pos.z / range);
        rgb *= kSciAmbient + (1.0 - kSciAmbient) * max(dot(n, to_sun), 0.0);
        alpha = kSciOpacity;
        float dz = u.p[10].z;
        if (dz > 0.0) {
            float q = world_pos.z / dz;              // contours at integer q
            float q_per_pixel = max(fwidth(q), 1e-6);
            float dist = abs(fract(q + 0.5) - 0.5) / q_per_pixel;  // pixels to the nearest contour
            float line = 1.0 - smoothstep(0.0, 1.0, dist);          // about one pixel wide
            line *= 1.0 - smoothstep(0.25, 0.5, q_per_pixel);       // fade where contours crowd
            rgb = mix(rgb, (1.0 - kContourDarken) * rgb, line);
        }
    }

    // Grid lines: about one pixel wide at any distance, fading out
    // where cells are under a few pixels so far water does not turn solid.
    float cells_per_line = u.p[7].z;
    if (cells_per_line > 0.0) {
        vec2 gc = grid_coord / cells_per_line;
        vec2 gc_per_pixel = max(fwidth(gc), vec2(1e-6));
        vec2 dist = abs(fract(gc - 0.5) - 0.5) / gc_per_pixel;
        float resolvable = 1.0 - smoothstep(0.2, 0.5, max(gc_per_pixel.x, gc_per_pixel.y));
        float line = kGridLineOpacity * resolvable * (1.0 - clamp(min(dist.x, dist.y), 0.0, 1.0));
        rgb = mix(rgb, (1.0 - kGridLineDarken) * rgb, line / kGridLineOpacity);
        alpha = max(alpha, line);
    }

    // Soft patch edges, so the end of the drawn sea is not a hard line.
    vec2 inside = patch_rect.zw - abs(world_pos.xy - patch_rect.xy);
    float edge = min(inside.x / max(patch_rect.z, 1e-6), inside.y / max(patch_rect.w, 1e-6));
    alpha *= smoothstep(0.0, kEdgeFade, edge);

    out_color = vec4(rgb, alpha);
}
)";

// Sky dome: a unit cube around the camera, drawn at the far plane.
const char* kSkyVert = R"(
layout(push_constant) uniform PushConstants {
    mat4 projection;
    mat4 modelView;
} pc;

layout(location = 0) in vec3 vsg_Vertex;
layout(location = 0) out vec3 direction;

out gl_PerVertex { vec4 gl_Position; };

void main() {
    direction = vsg_Vertex;
    // Remove translation so the sky stays at infinity.
    mat4 modelView = pc.modelView;
    modelView[3] = vec4(0.0, 0.0, 0.0, 1.0);
    vec4 pos = pc.projection * modelView * vec4(vsg_Vertex, 1.0);
    gl_Position = vec4(pos.xy, 0.0, pos.w);  // depth 0 = far plane (reversed depth)
}
)";

const char* kSkyFrag = R"(
layout(location = 0) in vec3 direction;
layout(location = 0) out vec4 out_color;

const float kSunCos = 0.99996;  // cos(0.5 deg): angular radius of the drawn sun disc

void main() {
    vec3 d = normalize(direction);
    vec3 c = SkyColor(d);
    float disc = smoothstep(kSunCos - 2e-5, kSunCos, dot(d, normalize(u.p[1].xyz)));
    c += u.p[1].w * disc * u.p[2].rgb * 4.0;
    out_color = vec4(c, 1.0);
}
)";

/// GLSL ElevationColor(t): the colour map in the kColormap slots (sRGB),
/// interpolated in sRGB as the legend is and returned linear.
std::string ElevationColorFunction() {
    std::ostringstream s;
    s << "vec3 SrgbToLinear(vec3 c) {\n"
         "    return mix(c / 12.92, pow((c + 0.055) / 1.055, vec3(2.4)), step(vec3(0.04045), c));\n"
         "}\n"
         "vec3 ElevationColor(float t) {\n"
         "    float x = clamp(t, 0.0, 1.0) * "
      << (kColormapStops - 1) << ".0;\n"
         "    int i = min(int(x), " << (kColormapStops - 2) << ");\n"
         "    vec3 a = u.p[" << kColormap << " + i].rgb;\n"
         "    vec3 b = u.p[" << kColormap << " + i + 1].rgb;\n"
         "    return SrgbToLinear(mix(a, b, x - float(i)));\n"
         "}\n";
    return s.str();
}

std::string WaterVertSource() { return kVersion + kWaterVert; }
std::string WaterGpuVertSource() { return kVersion + kParamBlock + kWaterGpuVert; }
std::string WaterFragSource() {
    return kVersion + kParamBlock + kSkyFunction + ElevationColorFunction() + kWaterFrag;
}
std::string SkyVertSource() { return kVersion + kSkyVert; }
std::string SkyFragSource() { return kVersion + kParamBlock + kSkyFunction + kSkyFrag; }

/// Pipeline, descriptor set and `draw` in one state group. Both water
/// pipelines share the fragment shader, blending and depth state; they
/// differ in vertex input and in what the descriptor set holds.
vsg::ref_ptr<vsg::Node> CreateWaterStateGroup(const std::string& vertex_source,
                                              const vsg::VertexInputState::Bindings& vertex_bindings,
                                              const vsg::VertexInputState::Attributes& vertex_attributes,
                                              const vsg::DescriptorSetLayoutBindings& layout_bindings,
                                              const vsg::Descriptors& descriptors,
                                              vsg::ref_ptr<vsg::VertexIndexDraw> draw) {
    const vsg::ShaderStages shaders{vsg::ShaderStage::create(VK_SHADER_STAGE_VERTEX_BIT, "main", vertex_source),
                                    vsg::ShaderStage::create(VK_SHADER_STAGE_FRAGMENT_BIT, "main", WaterFragSource())};

    auto descriptor_set_layout = vsg::DescriptorSetLayout::create(layout_bindings);
    // Projection and model-view matrices, pushed by VSG's RecordTraversal.
    const vsg::PushConstantRanges push_constant_ranges{{VK_SHADER_STAGE_VERTEX_BIT, 0, 128}};

    auto raster_state = vsg::RasterizationState::create();
    raster_state->cullMode = VK_CULL_MODE_NONE;  // visible from above and below

    // Drawn after the bodies (overlay group), so submerged hulls show through.
    auto blend_state = vsg::ColorBlendState::create();
    for (auto& att : blend_state->attachments) {
        att.blendEnable = VK_TRUE;
        att.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
        att.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        att.colorBlendOp = VK_BLEND_OP_ADD;
        att.srcAlphaBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
        att.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        att.alphaBlendOp = VK_BLEND_OP_ADD;
    }

    const vsg::GraphicsPipelineStates pipeline_states{
        vsg::VertexInputState::create(vertex_bindings, vertex_attributes),
        vsg::InputAssemblyState::create(),
        raster_state,
        vsg::MultisampleState::create(),
        blend_state,
        vsg::DepthStencilState::create()};

    auto pipeline_layout =
        vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{descriptor_set_layout}, push_constant_ranges);
    auto pipeline = vsg::GraphicsPipeline::create(pipeline_layout, shaders, pipeline_states);
    auto descriptor_set = vsg::DescriptorSet::create(descriptor_set_layout, descriptors);

    auto state_group = vsg::StateGroup::create();
    state_group->add(vsg::BindGraphicsPipeline::create(pipeline));
    state_group->add(
        vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_layout, 0, descriptor_set));
    state_group->addChild(draw);
    return state_group;
}

}  // namespace

WaterShader::WaterShader()
    : params_(vsg::vec4Array::create(kParamCount)), components_(vsg::vec4Array::create(kMaxGpuComponents)) {
    params_->properties.dataVariance = vsg::DataVariance::DYNAMIC_DATA;
    (*params_)[kEye] = vsg::vec4(0.0f, 0.0f, 100.0f, 0.0f);
    (*params_)[kSunDir] = vsg::vec4(0.0f, 0.0f, 1.0f, 0.0f);
    (*params_)[kSunColor] = vsg::vec4(1.0f, 1.0f, 1.0f, 0.0f);
    (*params_)[kSkyZenith] = vsg::vec4(0.02f, 0.03f, 0.04f, 0.0f);
    (*params_)[kSkyHorizon] = vsg::vec4(0.08f, 0.10f, 0.13f, 0.0f);
    (*params_)[kPatch] = vsg::vec4(0.0f, 0.0f, 1.0f, 1.0f);
    (*params_)[kWaterDeep] = vsg::vec4(0.004f, 0.04f, 0.09f, 0.0f);
    (*params_)[kWaves] = vsg::vec4(0.0f, 1.0f, 0.0f, 0.0f);
    (*params_)[kWaterScatter] = vsg::vec4(0.02f, 0.17f, 0.22f, 0.0f);
    (*params_)[kSkyGround] = vsg::vec4(0.005f, 0.007f, 0.01f, 0.0f);
    components_->properties.dataVariance = vsg::DataVariance::DYNAMIC_DATA;
}

void WaterShader::SetWaterColors(const vsg::vec3& deep, const vsg::vec3& scatter) {
    (*params_)[kWaterDeep] = vsg::vec4(deep.x, deep.y, deep.z, 0.0f);
    (*params_)[kWaterScatter] = vsg::vec4(scatter.x, scatter.y, scatter.z, 0.0f);
    params_->dirty();
}

void WaterShader::SetElevationStyle(bool on, float range_m, float contour_m, WaterPalette palette) {
    (*params_)[kStyle] = vsg::vec4(on ? 1.0f : 0.0f, range_m, contour_m, 0.0f);
    const ColormapStops& stops = PaletteStops(palette);
    for (int i = 0; i < kColormapStops; ++i) {
        (*params_)[kColormap + i] = vsg::vec4(stops[i][0], stops[i][1], stops[i][2], 0.0f);
    }
    params_->dirty();
}

void WaterShader::SetElevationScale(float elevation_scale) {
    (*params_)[kWaves].y = elevation_scale;
    params_->dirty();
}

void WaterShader::SetPatch(const vsg::vec2& patch_min, const vsg::vec2& patch_max) {
    const vsg::vec2 centre = (patch_min + patch_max) * 0.5f;
    const vsg::vec2 half = (patch_max - patch_min) * 0.5f;
    (*params_)[kPatch] = vsg::vec4(centre.x, centre.y, std::max(half.x, 1e-3f), std::max(half.y, 1e-3f));
    params_->dirty();
}

vsg::ref_ptr<vsg::Node> WaterShader::CreateNode(vsg::ref_ptr<vsg::vec3Array> vertices,
                                                vsg::ref_ptr<vsg::vec3Array> normals,
                                                vsg::ref_ptr<vsg::vec2Array> texcoords,
                                                vsg::ref_ptr<vsg::vec4Array> colors,
                                                vsg::ref_ptr<vsg::uintArray> indices, const vsg::vec2& patch_min,
                                                const vsg::vec2& patch_max) {
    if (!vertices || !normals || !texcoords || !colors || !indices || indices->size() == 0) {
        return {};
    }
    SetPatch(patch_min, patch_max);
    // GPU-grid waves are unused here; clear them in case a GPU grid came before.
    (*params_)[kWaves].x = 0.0f;

    const vsg::VertexInputState::Bindings vertex_bindings{
        VkVertexInputBindingDescription{0, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX},
        VkVertexInputBindingDescription{1, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX},
        VkVertexInputBindingDescription{2, sizeof(vsg::vec2), VK_VERTEX_INPUT_RATE_VERTEX},
        VkVertexInputBindingDescription{3, sizeof(vsg::vec4), VK_VERTEX_INPUT_RATE_VERTEX}};
    const vsg::VertexInputState::Attributes vertex_attributes{
        VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},
        VkVertexInputAttributeDescription{1, 1, VK_FORMAT_R32G32B32_SFLOAT, 0},
        VkVertexInputAttributeDescription{2, 2, VK_FORMAT_R32G32_SFLOAT, 0},
        VkVertexInputAttributeDescription{3, 3, VK_FORMAT_R32G32B32A32_SFLOAT, 0}};
    const vsg::DescriptorSetLayoutBindings layout_bindings{
        {0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr}};
    const vsg::Descriptors descriptors{vsg::DescriptorBuffer::create(params_, 0, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER)};

    auto draw = vsg::VertexIndexDraw::create();
    draw->assignArrays(vsg::DataList{vertices, normals, texcoords, colors});
    draw->assignIndices(indices);
    draw->indexCount = static_cast<uint32_t>(indices->size());
    draw->instanceCount = 1;
    return CreateWaterStateGroup(WaterVertSource(), vertex_bindings, vertex_attributes, layout_bindings, descriptors, draw);
}

vsg::ref_ptr<vsg::Node> WaterShader::CreateGpuNode(vsg::ref_ptr<vsg::vec3Array> vertices,
                                                   vsg::ref_ptr<vsg::vec2Array> grid_coords,
                                                   vsg::ref_ptr<vsg::uintArray> indices, const vsg::vec2& patch_min,
                                                   const vsg::vec2& patch_max) {
    if (!vertices || !grid_coords || !indices || indices->size() == 0) {
        return {};
    }
    SetPatch(patch_min, patch_max);

    const vsg::VertexInputState::Bindings vertex_bindings{
        VkVertexInputBindingDescription{0, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX},
        VkVertexInputBindingDescription{1, sizeof(vsg::vec2), VK_VERTEX_INPUT_RATE_VERTEX}};
    const vsg::VertexInputState::Attributes vertex_attributes{
        VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0},
        VkVertexInputAttributeDescription{1, 1, VK_FORMAT_R32G32_SFLOAT, 0}};
    const vsg::DescriptorSetLayoutBindings layout_bindings{
        {0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, nullptr},
        {1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_VERTEX_BIT, nullptr}};
    const vsg::Descriptors descriptors{
        vsg::DescriptorBuffer::create(params_, 0, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER),
        vsg::DescriptorBuffer::create(components_, 1, 0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER)};

    auto draw = vsg::VertexIndexDraw::create();
    draw->assignArrays(vsg::DataList{vertices, grid_coords});
    draw->assignIndices(indices);
    draw->indexCount = static_cast<uint32_t>(indices->size());
    draw->instanceCount = 1;
    return CreateWaterStateGroup(WaterGpuVertSource(), vertex_bindings, vertex_attributes, layout_bindings, descriptors,
                                 draw);
}

int WaterShader::SetWaves(const std::vector<DisplayWaveComponent>& components) {
    const int count = static_cast<int>(std::min(components.size(), static_cast<size_t>(kMaxGpuComponents)));
    for (int i = 0; i < count; ++i) {
        const DisplayWaveComponent& c = components[static_cast<size_t>(i)];
        (*components_)[static_cast<size_t>(i)] = vsg::vec4(c.amplitude, c.kx, c.ky, c.phase);
    }
    (*params_)[kWaves].x = static_cast<float>(count);
    components_->dirty();
    params_->dirty();
    return count;
}

void WaterShader::SetGridLines(float cells_per_line) {
    (*params_)[kWaves].z = std::max(cells_per_line, 0.0f);
    params_->dirty();
}

void WaterShader::SetEye(const vsg::dvec3& eye) {
    (*params_)[kEye] = vsg::vec4(static_cast<float>(eye.x), static_cast<float>(eye.y), static_cast<float>(eye.z), 0.0f);
    params_->dirty();
}

void WaterShader::SetSun(const vsg::vec3& towards_sun, const vsg::vec3& color) {
    const float haze = (*params_)[kSunDir].w;
    (*params_)[kSunDir] = vsg::vec4(towards_sun.x, towards_sun.y, towards_sun.z, haze);
    (*params_)[kSunColor] = vsg::vec4(color.x, color.y, color.z, 0.0f);
    params_->dirty();
}

void WaterShader::SetSky(const vsg::vec3& zenith, const vsg::vec3& horizon, const vsg::vec3& ground,
                         float sun_haze) {
    (*params_)[kSkyZenith] = vsg::vec4(zenith.x, zenith.y, zenith.z, 0.0f);
    (*params_)[kSkyHorizon] = vsg::vec4(horizon.x, horizon.y, horizon.z, 0.0f);
    (*params_)[kSkyGround] = vsg::vec4(ground.x, ground.y, ground.z, 0.0f);
    (*params_)[kSunDir].w = std::max(sun_haze, 0.0f);
    params_->dirty();
}

vsg::ref_ptr<vsg::Node> WaterShader::CreateSkyNode() {
    const vsg::ShaderStages shaders{vsg::ShaderStage::create(VK_SHADER_STAGE_VERTEX_BIT, "main", SkyVertSource()),
                                    vsg::ShaderStage::create(VK_SHADER_STAGE_FRAGMENT_BIT, "main", SkyFragSource())};
    auto descriptor_set_layout = vsg::DescriptorSetLayout::create(vsg::DescriptorSetLayoutBindings{
        {0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr}});
    const vsg::PushConstantRanges push_constant_ranges{{VK_SHADER_STAGE_VERTEX_BIT, 0, 128}};

    const vsg::VertexInputState::Bindings vertex_bindings{
        VkVertexInputBindingDescription{0, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX}};
    const vsg::VertexInputState::Attributes vertex_attributes{
        VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0}};

    // Seen from inside the cube; drawn from either side so winding does not matter.
    auto raster_state = vsg::RasterizationState::create();
    raster_state->cullMode = VK_CULL_MODE_NONE;

    // Depth 0 is the far plane (VSG uses reversed depth): drawn only where
    // nothing else is, and never hides anything.
    auto depth_state = vsg::DepthStencilState::create();
    depth_state->depthTestEnable = VK_TRUE;
    depth_state->depthWriteEnable = VK_FALSE;
    depth_state->depthCompareOp = VK_COMPARE_OP_GREATER_OR_EQUAL;

    const vsg::GraphicsPipelineStates pipeline_states{
        vsg::VertexInputState::create(vertex_bindings, vertex_attributes),
        vsg::InputAssemblyState::create(),
        raster_state,
        vsg::MultisampleState::create(),
        vsg::ColorBlendState::create(),
        depth_state};

    auto pipeline_layout =
        vsg::PipelineLayout::create(vsg::DescriptorSetLayouts{descriptor_set_layout}, push_constant_ranges);
    auto pipeline = vsg::GraphicsPipeline::create(pipeline_layout, shaders, pipeline_states);
    auto uniform = vsg::DescriptorBuffer::create(params_, 0, 0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
    auto descriptor_set = vsg::DescriptorSet::create(descriptor_set_layout, vsg::Descriptors{uniform});

    auto root = vsg::StateGroup::create();
    root->add(vsg::BindGraphicsPipeline::create(pipeline));
    root->add(vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_layout, 0, descriptor_set));

    // Unit cube in world axes (z up); the vertex shader drops the translation.
    auto vertices = vsg::vec3Array::create({{-1.0f, -1.0f, -1.0f},
                                            {1.0f, -1.0f, -1.0f},
                                            {1.0f, 1.0f, -1.0f},
                                            {-1.0f, 1.0f, -1.0f},
                                            {-1.0f, -1.0f, 1.0f},
                                            {1.0f, -1.0f, 1.0f},
                                            {1.0f, 1.0f, 1.0f},
                                            {-1.0f, 1.0f, 1.0f}});
    auto indices = vsg::ushortArray::create({0, 1, 2, 0, 2, 3,    // bottom
                                             4, 6, 5, 4, 7, 6,    // top
                                             0, 5, 1, 0, 4, 5,    // -y
                                             3, 2, 6, 3, 6, 7,    // +y
                                             0, 3, 7, 0, 7, 4,    // -x
                                             1, 5, 6, 1, 6, 2});  // +x
    root->addChild(vsg::BindVertexBuffers::create(0, vsg::DataList{vertices}));
    root->addChild(vsg::BindIndexBuffer::create(indices));
    root->addChild(vsg::DrawIndexed::create(static_cast<uint32_t>(indices->size()), 1, 0, 0, 0));
    return root;
}

}  // namespace seastack::viz
