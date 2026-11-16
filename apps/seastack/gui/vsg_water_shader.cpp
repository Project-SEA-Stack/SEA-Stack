// =============================================================================
// SEA-Stack VSG Water Shader - Implementation
// =============================================================================
// Pipeline built by hand as in vsg_skybox.cpp; GLSL is compiled at run time
// by VSG's shader compiler. All colours are linear (the swapchain is sRGB).
// =============================================================================

#include "vsg_water_shader.h"

#include <algorithm>
#include <string>

namespace seastack::viz {

namespace {

// Uniform block layout (vec4 each).
enum Param : uint32_t {
    kEye = 0,           // xyz: camera position [m, world]
    kSunDir = 1,        // xyz: unit vector towards the key light
    kSunColor = 2,      // rgb: key light colour x intensity
    kSkyZenith = 3,     // rgb
    kSkyHorizon = 4,    // rgb
    kPatch = 5,         // xy: patch centre [m]; zw: half extents [m]
    kWaterDeep = 6,     // rgb: water body colour in troughs and seen straight down
    kWaves = 7,         // x: GPU component count; y: elevation scale for crest shading [m];
                        // z: grid cells per drawn grid line (0 = no lines)
    kWaterScatter = 8,  // rgb: water body colour on crests (light scattered in thin water)
    kParamCount = 9,
};

const char* kParamBlock = R"(
layout(set = 0, binding = 0) uniform WaterParams { vec4 p[9]; } u;
)";

// Sky radiance reflected along unit direction `d` (world, z up): a gradient
// from the horizon to the zenith; the horizon colour below it.
const char* kSkyFunction = R"(
vec3 SkyColor(vec3 d) {
    return mix(u.p[4].rgb, u.p[3].rgb, pow(max(d.z, 0.0), 0.5));
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
    grid_coord = vec2(0.0);  // grid lines are a separate mesh on this path
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

    // Reflected sky.
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

    // Grid lines (GPU grid): about one pixel wide at any distance, fading out
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

std::string WaterVertSource() { return kVersion + kWaterVert; }
std::string WaterGpuVertSource() { return kVersion + kParamBlock + kWaterGpuVert; }
std::string WaterFragSource() { return kVersion + kParamBlock + kSkyFunction + kWaterFrag; }

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
    components_->properties.dataVariance = vsg::DataVariance::DYNAMIC_DATA;
}

void WaterShader::SetWaterColors(const vsg::vec3& deep, const vsg::vec3& scatter) {
    (*params_)[kWaterDeep] = vsg::vec4(deep.x, deep.y, deep.z, 0.0f);
    (*params_)[kWaterScatter] = vsg::vec4(scatter.x, scatter.y, scatter.z, 0.0f);
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
    // GPU-grid inputs are unused here; clear them in case a GPU grid came before.
    (*params_)[kWaves].x = 0.0f;
    (*params_)[kWaves].z = 0.0f;

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
    (*params_)[kSunDir] = vsg::vec4(towards_sun.x, towards_sun.y, towards_sun.z, 0.0f);
    (*params_)[kSunColor] = vsg::vec4(color.x, color.y, color.z, 0.0f);
    params_->dirty();
}

void WaterShader::SetSky(const vsg::vec3& zenith, const vsg::vec3& horizon) {
    (*params_)[kSkyZenith] = vsg::vec4(zenith.x, zenith.y, zenith.z, 0.0f);
    (*params_)[kSkyHorizon] = vsg::vec4(horizon.x, horizon.y, horizon.z, 0.0f);
    params_->dirty();
}

}  // namespace seastack::viz
