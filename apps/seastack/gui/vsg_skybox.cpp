// =============================================================================
// SEA-Stack VSG Skybox - Implementation
// =============================================================================
// Shaders, cube and pipeline states follow vsgExamples' vsgskybox
// (https://github.com/vsg-dev/vsgExamples), as does Chrono's createSkybox.
// =============================================================================

#include "vsg_skybox.h"

#include <vsgXchange/all.h>

#include <iostream>

namespace seastack::viz {

namespace {

const char* kSkyboxVert = R"(
#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(push_constant) uniform PushConstants {
    mat4 projection;
    mat4 modelView;
} pc;

layout(location = 0) in vec3 vsg_Vertex;
layout(location = 0) out vec3 UVW;

out gl_PerVertex{ vec4 gl_Position; };

void main()
{
    UVW = vsg_Vertex;

    // Remove translation so the sky stays at infinity.
    mat4 modelView = pc.modelView;
    modelView[3] = vec4(0.0, 0.0, 0.0, 1.0);

    vec4 pos = pc.projection * modelView * vec4(vsg_Vertex, 1.0);
    gl_Position = vec4(pos.xy, 0.0, pos.w);
}
)";

// 0.7 dims the sky to match the Chrono viewer.
const char* kSkyboxFrag = R"(
#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(binding = 0) uniform samplerCube envMap;
layout(location = 0) in vec3 UVW;
layout(location = 0) out vec4 outColor;

void main()
{
    outColor = 0.7 * textureLod(envMap, UVW, 0);
}
)";

}  // namespace

vsg::ref_ptr<vsg::Node> CreateSkybox(const std::string& path) {
    auto options = vsg::Options::create(vsgXchange::all::create());
    auto data = vsg::read_cast<vsg::Data>(path, options);
    if (!data) {
        std::cerr << "[SeastackViewer] Could not read skybox cubemap " << path << std::endl;
        return {};
    }

    const vsg::ShaderStages shaders{
        vsg::ShaderStage::create(VK_SHADER_STAGE_VERTEX_BIT, "main", kSkyboxVert),
        vsg::ShaderStage::create(VK_SHADER_STAGE_FRAGMENT_BIT, "main", kSkyboxFrag)};

    auto descriptor_set_layout = vsg::DescriptorSetLayout::create(vsg::DescriptorSetLayoutBindings{
        {0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr}});
    // Projection and model-view matrices, pushed by VSG's RecordTraversal.
    const vsg::PushConstantRanges push_constant_ranges{{VK_SHADER_STAGE_VERTEX_BIT, 0, 128}};

    const vsg::VertexInputState::Bindings vertex_bindings{
        VkVertexInputBindingDescription{0, sizeof(vsg::vec3), VK_VERTEX_INPUT_RATE_VERTEX}};
    const vsg::VertexInputState::Attributes vertex_attributes{
        VkVertexInputAttributeDescription{0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0}};

    // Viewed from inside the cube, so the front faces are culled.
    auto raster_state = vsg::RasterizationState::create();
    raster_state->cullMode = VK_CULL_MODE_FRONT_BIT;

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

    auto texture =
        vsg::DescriptorImage::create(vsg::Sampler::create(), data, 0, 0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
    auto descriptor_set = vsg::DescriptorSet::create(descriptor_set_layout, vsg::Descriptors{texture});

    auto root = vsg::StateGroup::create();
    root->add(vsg::BindGraphicsPipeline::create(pipeline));
    root->add(vsg::BindDescriptorSet::create(VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_layout, 0, descriptor_set));

    // Four vertices per face so each face can be wound independently.
    auto vertices = vsg::vec3Array::create({
        {-1.0f, -1.0f, -1.0f}, {1.0f, -1.0f, -1.0f}, {-1.0f, 1.0f, -1.0f}, {1.0f, 1.0f, -1.0f},  // back
        {-1.0f, -1.0f, 1.0f},  {1.0f, -1.0f, 1.0f},  {-1.0f, 1.0f, 1.0f},  {1.0f, 1.0f, 1.0f},   // front
        {-1.0f, -1.0f, -1.0f}, {-1.0f, -1.0f, 1.0f}, {-1.0f, 1.0f, -1.0f}, {-1.0f, 1.0f, 1.0f},  // left
        {1.0f, -1.0f, -1.0f},  {1.0f, -1.0f, 1.0f},  {1.0f, 1.0f, -1.0f},  {1.0f, 1.0f, 1.0f},   // right
        {-1.0f, -1.0f, -1.0f}, {-1.0f, -1.0f, 1.0f}, {1.0f, -1.0f, -1.0f}, {1.0f, -1.0f, 1.0f},  // bottom
        {-1.0f, 1.0f, -1.0f},  {-1.0f, 1.0f, 1.0f},  {1.0f, 1.0f, -1.0f},  {1.0f, 1.0f, 1.0f},   // top
    });
    auto indices = vsg::ushortArray::create({
        0,  2,  1,  1,  2,  3,   // back
        6,  4,  5,  7,  6,  5,   // front
        10, 8,  9,  11, 10, 9,   // left
        14, 13, 12, 15, 13, 14,  // right
        17, 16, 19, 19, 16, 18,  // bottom
        23, 20, 21, 22, 20, 23,  // top
    });
    root->addChild(vsg::BindVertexBuffers::create(0, vsg::DataList{vertices}));
    root->addChild(vsg::BindIndexBuffer::create(indices));
    root->addChild(vsg::DrawIndexed::create(static_cast<uint32_t>(indices->size()), 1, 0, 0, 0));

    // Cubemaps are Y-up; rotate +90 deg about x so the sky's up is world +z.
    // Filled element by element (m[column][row]) to avoid the dmat4
    // constructor that Chrono_vsg.dll also exports (LNK2005).
    vsg::dmat4 y_up_to_z_up;  // identity
    y_up_to_z_up[1][1] = 0.0;
    y_up_to_z_up[1][2] = 1.0;   // y -> +z
    y_up_to_z_up[2][1] = -1.0;  // z -> -y
    y_up_to_z_up[2][2] = 0.0;
    auto transform = vsg::MatrixTransform::create(y_up_to_z_up);
    transform->addChild(root);
    return transform;
}

}  // namespace seastack::viz
